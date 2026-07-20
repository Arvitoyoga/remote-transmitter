#include "elrs.h"
#include <stdint.h>
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h> 

#undef ESP_LOGI
#define ESP_LOGI(tag, format, ...) ((void)0)

#define TAG "ELRS"

#define CRSF_CRC_POLY 0xD5
#define CRSF_CRC_COMMAND_POLY 0xBA

// Utils
uint8_t get_crc8(uint8_t *buf, size_t size, uint8_t poly) {
    uint8_t crc8 = 0x00;

    for (int i = 0; i < size; i++) {
        crc8 ^= buf[i];
        for (int j = 0; j < 8; j++) {
            if (crc8 & 0x80) {
                crc8 <<= 1;
                crc8 ^= poly;
            } else {
                crc8 <<= 1;
            }
        }
    }
    return crc8;
}

float normalize_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

float get_angle_error(float target, float current) {
    float error = normalize_angle(target - current);
    return error;
}

// PID
void init_yaw_pid(pid_controller_t *pid) {
//     If too slow → Increase Kp
// If oscillating → Increase Kd or decrease Kp
// If not reaching target → Add small Ki
// If overshooting → Decrease Kp or increase Kd
// Very conservative starting values
    pid->kp = 0.5f;    // Start with low proportional gain
    pid->ki = 0.0f;    // Start with no integral
    pid->kd = 1.0f;    // Moderate derivative for dampening
    pid->prev_error = 0;
    pid->output_min = -500.0f;  // Maps to RC_MIN when added to RC_MID
    pid->output_max = 500.0f;   // Maps to RC_MAX when added to RC_MID
    pid->last_time = 0;
    pid->is_active = false;
    pid->target_angle = 90.0f;
    pid->start_angle = 0.0f;
    pid->angle_threshold = 2.0f;  // Consider movement complete within 2 degrees
}
void start_yaw_movement(pid_controller_t *pid, float current_angle, float target_angle) {
    pid->is_active = true;
    pid->target_angle = target_angle;
    pid->start_angle = current_angle;
    pid->integral = 0.0f;  // Reset integral term
    pid->prev_error = get_angle_error(pid->target_angle, current_angle);  // Reset error term
}
void update_yaw_pid(uint16_t *channels, pid_controller_t *pid, float current_angle, uint32_t current_time) {
    // If PID is not active, return center position
    if (!pid->is_active) {
        return;
    }

    // Calculate time delta
    float dt = (current_time - pid->last_time) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f) {
        pid->last_time = current_time;
        return;
    }

    // Calculate error
    float error = get_angle_error(pid->target_angle, current_angle);

    // Check if we've reached the target
    if (fabs(error) <= pid->angle_threshold || (fabs(pid->prev_error) < fabs(error))) {
        pid->is_active = false;
        return;  // Return to center position
    }

    // Rest of PID calculation remains the same
    float p_term = pid->kp * error;
    pid->integral += error * dt;
    float i_term = pid->ki * pid->integral;
    float d_term = pid->kd * ((error - pid->prev_error) / dt);
    pid->prev_error = error;

    float output = p_term + i_term + d_term;

    if (output > pid->output_max) {
        output = pid->output_max;
        pid->integral -= error * dt;
    } else if (output < pid->output_min) {
        output = pid->output_min;
        pid->integral -= error * dt;
    }

    pid->last_time = current_time;

    uint16_t rc_value = (uint16_t)(RC_MID + output);
    if (rc_value > RC_MAX) rc_value = RC_MAX;
    if (rc_value < RC_MIN) rc_value = RC_MIN;

    channels[YAW] = rc_value;
}
float get_interpolated_yaw(attitude_data_t *attitude) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float time_since_update = (current_time - attitude->last_update_time);

    return normalize_angle(attitude->yaw + (attitude->yaw_rate * time_since_update));
}


// Convert packed altitude to meters
float unpack_altitude_to_meters(uint16_t packed) {
    int32_t altitude_dm = (packed & 0x8000) ?
        ((packed & 0x7fff) * 10) :  // Meter resolution case
        (packed - ALT_MIN_DM);      // Decimeter resolution case

    return (float)altitude_dm / 10.0f;  // Convert decimeters to meters
}

// Unpack vertical speed (implementation depends on your needs)
float unpack_vertical_speed(int8_t packed) {
    // Vertical speed is packed in 3cm/s units with 25m/s range
    return (float)packed * 0.03f;  // Convert to m/s
}


// ELRS
bool crsf_validate_frame(uint8_t *frame, uint8_t len) {
    return get_crc8(&frame[2], len - 3, CRSF_CRC_POLY) == frame[len - 1];
}


void parse_frame(const uint8_t *data, crsf_data_t *crsf_data) {
    if (!crsf_data) return;

    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    // printf("0x%02X ", data[2]);
    switch (data[2]) {
        case FRAME_TYPE_BARO_ALTITUDE: {
            uint16_t alt_packed = (uint16_t)(data[3] << 8 | data[4]);
            int8_t speed_packed = (int8_t)data[5];

            crsf_data->baro_alt = unpack_altitude_to_meters(alt_packed);
            crsf_data->vspd = unpack_vertical_speed(speed_packed);

            // For debug printing
            ESP_LOGI("tlm", "Altitude: %.2f m, Vertical Speed: %.2f m/s\n",
                crsf_data->baro_alt,
                crsf_data->vspd);
            break;

        case FRAME_TYPE_ATTITUDE:
            crsf_data->attitude.pitch = (float)((int16_t)((data[3] << 8 | data[4])) / 10000.0f) * RAD_TO_DEG;
            crsf_data->attitude.roll = (float)((int16_t)((data[5] << 8 | data[6])) / 10000.0f) * RAD_TO_DEG;

            crsf_data->attitude.prev_yaw = crsf_data->attitude.yaw;
            crsf_data->attitude.yaw = (float)((int16_t)((data[7] << 8 | data[8])) / 10000.0f) * RAD_TO_DEG;

            float time_diff = (current_time - crsf_data->attitude.last_update_time);
            if (time_diff > 0) {
                float yaw_diff = normalize_angle(crsf_data->attitude.yaw - crsf_data->attitude.prev_yaw);
                crsf_data->attitude.yaw_rate = yaw_diff / time_diff;
            }

            crsf_data->attitude.last_update_time = current_time;
            break;
        // case FRAME_TYPE_VARIO:
        //     crsf_data->vspd = (float)((int16_t)(data[3] << 8 | data[5])) / 10.0;


        default:
            break;
        }
    }
}
void process_crsf_data(uint8_t *input_buffer, size_t *input_len, crsf_data_t *crsf_data) {
    if (!input_buffer || !input_len || !crsf_data) return;

    static uint8_t parse_buffer[64];
    static size_t parse_buffer_len = 0;

    // Add new data to parse buffer
    if (*input_len > 0 && parse_buffer_len < sizeof(parse_buffer)) {
        // Calculate safe copy length
        size_t remaining_space = sizeof(parse_buffer) - parse_buffer_len;
        size_t copy_len = (*input_len < remaining_space) ? *input_len : remaining_space;

        // Ensure source and destination don't overlap
        if (input_buffer + copy_len <= parse_buffer || input_buffer >= parse_buffer + sizeof(parse_buffer)) {
            memcpy(parse_buffer + parse_buffer_len, input_buffer, copy_len);
        } else {
            memmove(parse_buffer + parse_buffer_len, input_buffer, copy_len);
        }
        parse_buffer_len += copy_len;

        // Remove processed data from input buffer
        if (copy_len < *input_len) {
            size_t remaining_data = *input_len - copy_len;
            memmove(input_buffer, input_buffer + copy_len, remaining_data);
            *input_len = remaining_data;
        } else {
            *input_len = 0;
        }
    }

    // Process packets while we have enough data
    while (parse_buffer_len >= 2) {  // Changed from > 2 to >= 2 to prevent underflow
        uint8_t expected_len = parse_buffer[1] + 2;

        // Validate packet length
        if (expected_len > sizeof(parse_buffer) || expected_len < 4) {
            // Invalid length, clear buffer and start fresh
            parse_buffer_len = 0;
            break;
        }

        // Check if we have a complete packet
        if (parse_buffer_len >= expected_len) {
            // Validate CRC
            if (crsf_validate_frame(parse_buffer, expected_len)) {
                parse_frame(parse_buffer, crsf_data);
            } else {
                ESP_LOGW(TAG, "CRC error in CRSF packet");
            }

            // Remove processed packet from buffer
            if (parse_buffer_len > expected_len) {
                memmove(parse_buffer, parse_buffer + expected_len, parse_buffer_len - expected_len);
            }
            parse_buffer_len -= expected_len;
        } else {
            // Not enough data for a complete packet
            break;
        }
    }
}

void elrs_send_data(const int port, const uint8_t *data, size_t len) {
    if (uart_write_bytes(port, data, len) != len) {
        ESP_LOGE(TAG, "Send data critical failure.");
    }
}


// Packet creation
void pack_crsf_to_bytes(uint16_t *channels, uint8_t *result) {
    size_t result_idx = 0;
    uint32_t newVal = 0;
    int destShift = 0;

    for (int ch_idx = 0; ch_idx < 16; ch_idx++) {
        // Validate channel value (0-1984)
        if (channels[ch_idx] > MAX_CHANNEL_VALUE) {
            channels[ch_idx] = channels[ch_idx] > MAX_CHANNEL_VALUE ? MAX_CHANNEL_VALUE : 0;
        }

        // Put the low bits in any remaining dest capacity
        newVal |= (channels[ch_idx] << destShift) & 0xff;
        result[result_idx++] = newVal & 0xff;

        // Shift the high bits down and place them into the next dest byte
        int srcBitsLeft = 11 - 8 + destShift;
        newVal = channels[ch_idx] >> (11 - srcBitsLeft);

        // When there's at least a full byte remaining, consume that as well
        if (srcBitsLeft >= 8) {
            result[result_idx++] = newVal & 0xff;
            newVal >>= 8;
            srcBitsLeft -= 8;
        }

        // Next dest should be shifted up by the bits consumed
        destShift = srcBitsLeft;
    }
}

void create_crsf_channels_packet(uint16_t *channels, uint8_t *packet) {
    if (channels == NULL || packet == NULL) {
        return;
    }

    packet[0] = DEVICE_ADDRESS_FLIGHT_CONTROLLER;
    packet[1] = 24;
    packet[2] = FRAME_TYPE_RC_CHANNELS;
    pack_crsf_to_bytes(channels, &packet[3]);
    packet[25] = get_crc8(&packet[2], 23, CRSF_CRC_POLY);
}

void create_model_switch_packet(uint8_t id, uint8_t *packet) {
    ESP_LOGI(TAG, "switching to id %d", id);

    packet[0] = DEVICE_ADDRESS_FLIGHT_CONTROLLER;
    packet[1] = 8;
    packet[2] = FRAME_TYPE_DIRECT_COMMANDS;
    packet[3] = DEVICE_ADDRESS_TX_MODULE;
    packet[4] = DEVICE_ADDRESS_REMOTE_CONTROL;
    packet[5] = COMMAND_CROSSFIRE;
    packet[6] = CROSSFIRE_COMMAND_MODEL_SELECT;
    packet[7] = id;
    packet[8] = get_crc8(&packet[2], 6, CRSF_CRC_COMMAND_POLY);
    packet[9] = get_crc8(&packet[2], 7, CRSF_CRC_POLY);
}

void create_subscribe_packet(uint8_t id, uint8_t *packet) {
    ESP_LOGI(TAG, "subcribed to id 0x%02X", id);

    packet[0] = DEVICE_ADDRESS_FLIGHT_CONTROLLER;
    packet[1] = 8;
    packet[2] = FRAME_TYPE_DIRECT_COMMANDS;
    packet[3] = DEVICE_ADDRESS_TX_MODULE;
    packet[4] = DEVICE_ADDRESS_REMOTE_CONTROL;
    packet[5] = 0x20; // 0x32 0x10
    packet[6] = 0x01;
    packet[7] = id;//barometer address
    packet[8] = 20; // 20ms
    packet[9] = get_crc8(&packet[2], 6, CRSF_CRC_COMMAND_POLY);
    packet[10] = get_crc8(&packet[2], 8, CRSF_CRC_POLY);
}