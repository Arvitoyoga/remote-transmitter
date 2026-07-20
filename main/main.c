#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "modules/utils.c"
#include "modules/imu.h"
#include "modules/elrs.h"
#include "modules/filters.h"

#include "iot_button.h"

#include "esp_timer.h"
#include "esp_log.h"

#undef ESP_LOGI
#define ESP_LOGI(tag, format, ...) ((void)0)

#define TAG "main"

#define DRONE_COUNT 2

// ELRS
#define INTERVALMS 3 * 1000 // in microseconds
#define SIM_OUTPUT_INTERVAL_MS 20

#define UART_NUM UART_NUM_2
#define TX_PIN 17
#define RX_PIN 16

#define ARMING_CHANNEL AUX1
#define CAMSWITCH_CHANNEL AUX2
#define MECHANISM_CHANNEL AUX3
#define FAILSAFE_CHANNEL AUX4
// TODO: nyambungin ke tele
// Offsets
#define ROLL_CENTER 0
#define PITCH_CENTER 0
#define THROTTLE_CENTER 0
#define YAW_CENTER 0

// Buttons
#define RIGHT_POINT GPIO_NUM_33
#define RIGHT_MIDDLE GPIO_NUM_32
#define RIGHT_RING GPIO_NUM_35
#define RIGHT_LITTLE GPIO_NUM_34

#define LEFT_POINT GPIO_NUM_14
#define LEFT_MIDDLE GPIO_NUM_27
#define LEFT_RING GPIO_NUM_26
#define LEFT_LITTLE GPIO_NUM_25

#define MECHANISM_CHANGE 10


button_handle_t arming_button;
// button_handle_t turn180_button;
button_handle_t switch_id_button;
button_handle_t mechanism_btn;
button_handle_t failsafe_btn;
button_handle_t cam_switch_btn;
button_handle_t increment_mechanism_btn;
button_handle_t decrement_mechanism_btn;


// Remote variables
int8_t current_id = 1;
uint16_t channels[16] = { 0 };
uint16_t last_throttle = 0;

int8_t left_error = 0;
int8_t right_error = 0;
int8_t imu_stuck_counter = 0;
int8_t IMU_STUCK_THRESHOLD = 3;

bool left_calibrated = 0;
bool right_calibrated = 0;
bool should_transmit = 0;

bool should_switch = 1;

uint16_t max_mechanism_value = 1378;
float last_left_imu_y = 0.0f;
float last_left_imu_x = 0.0f;
float last_right_imu_y = 0.0f;
float last_right_imu_x = 0.0f;

static int read_active_low_button(gpio_num_t gpio) {
    return gpio_get_level(gpio) == 0;
}

static void sim_input_init(void) {
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << LEFT_POINT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&gpio_conf);
}

// crsf_data_t crsf_data = {0};
// pid_controller_t yaw_pid;

// Callbacks
void toggle_channel(void *arg, void *data) {
    (void)arg;

    crsf_channels_type channel = (crsf_channels_type)data;
    channels[channel] = (channels[channel] <= 1000) * MAX_CHANNEL_VALUE;
    ESP_LOGI("gpio", "channel %d = %d", channel, channels[channel]);
}

void toggle_channel_mechanism(void *arg, void *data) {
    (void)arg;
    (void)data;

    channels[MECHANISM_CHANNEL] = (channels[MECHANISM_CHANNEL] <= 100) * max_mechanism_value;
    ESP_LOGI("gpio", "channel %d = %d", MECHANISM_CHANNEL, channels[MECHANISM_CHANNEL]);
}


void increment_max_mechanism(void *arg, void *data) {
    (void)arg;
    (void)data;

    max_mechanism_value += MECHANISM_CHANGE;
    ESP_LOGI("gpio", "max mech = %d", max_mechanism_value);
}

void decrement_max_mechanism(void *arg, void *data) {
    (void)arg;
    (void)data;

    max_mechanism_value -= MECHANISM_CHANGE;
    ESP_LOGI("gpio", "max mech = %d", max_mechanism_value);
}

void toggle_arming(void *arg, void *data) {
    (void)arg;
    (void)data;

    if (iot_button_get_ticks_time(arming_button) < 500) return;

    channels[ARMING_CHANNEL] = (channels[ARMING_CHANNEL] <= 1000) * MAX_CHANNEL_VALUE;
    ESP_LOGI("gpio", "channel %d = %d", ARMING_CHANNEL, channels[ARMING_CHANNEL]);
}


void switch_id_cb(void *arg, void *data) {
    (void)arg;
    (void)data;

    if (iot_button_get_ticks_time(switch_id_button) < 500) return;

    should_switch = 1;
    current_id++;
    current_id %= DRONE_COUNT;
    ESP_LOGI("gpio", "Switch id to %d", current_id);
}

// void subscribe_cb(){
//     should_subscribe = !should_subscribe;
//     ESP_LOGI("subscribe", "subscribe is %d", should_subscribe);
// }

// void turn180_cb(){
//     if(!yaw_pid.is_active){
//         start_yaw_movement(&yaw_pid, crsf_data.attitude.yaw, normalize_angle(crsf_data.attitude.yaw + 180));
//     } else {
//         yaw_pid.is_active = false;
//     }
//     ESP_LOGI("gpio", "turn is %d", yaw_pid.is_active);
// }

// Timer
static void timer_callback(void *arg) {
    should_transmit = 1;
}

void timer_init() {
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .name = "timer_callback"
    };

    esp_timer_handle_t timer_periodic;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_periodic));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_periodic, INTERVALMS));
}

// Initializers
void gpio_init() {
    // init buttons
    arming_button = init_debounced_btn(RIGHT_RING, BUTTON_PRESS_UP, toggle_arming, NULL, 200);
    cam_switch_btn = init_debounced_btn(RIGHT_MIDDLE,
        BUTTON_PRESS_DOWN,
        toggle_channel,
        (void *)CAMSWITCH_CHANNEL,
        200);
    mechanism_btn = init_debounced_btn(RIGHT_POINT,
        BUTTON_PRESS_DOWN,
        toggle_channel_mechanism,
        NULL,
        200);
// turn180_button = init_btn(RIGHT_LITTLE, BUTTON_SINGLE_CLICK, turn180_cb, NULL);
    switch_id_button = init_debounced_btn(LEFT_RING, BUTTON_PRESS_UP, switch_id_cb, NULL, 200);
    failsafe_btn = init_debounced_btn(LEFT_MIDDLE, BUTTON_PRESS_UP, toggle_channel, (void *)FAILSAFE_CHANNEL, 200);
    increment_mechanism_btn = init_debounced_btn(RIGHT_LITTLE, BUTTON_PRESS_DOWN, increment_max_mechanism, NULL, 200);
    decrement_mechanism_btn = init_debounced_btn(LEFT_LITTLE, BUTTON_PRESS_DOWN, decrement_max_mechanism, NULL, 200);

    // subscribe_button = init_btn(LEFT_POINT, BUTTON_SINGLE_CLICK, subscribe_cb, NULL);

}

void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = 921600, //912600
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_driver_install(UART_NUM, 1024, 1024, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_mode(UART_NUM, UART_MODE_UART);
    uart_set_line_inverse(UART_NUM, UART_SIGNAL_TXD_INV || UART_SIGNAL_RXD_INV);
}

void i2c_init() {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 22,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 300000,
    };
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    // i2c_set_timeout(I2C_NUM_0, 0xFFFFF);
}

// Tasks
void left_imu_task(void *pvParameters) {
    (void)pvParameters;

    struct full_imu_data left_imu_data = create_full_imu_data();
    mpu6050_handle_t imu = imu_init(I2C_NUM_0, MPU6050_I2C_ADDRESS_1);

    // Calibration
    for (int i = 0; i < 2000; i++) {
        imu_read_raw(imu, &left_imu_data.gyro, &left_imu_data.acce);

        imu_add(&left_imu_data.offset, left_imu_data.gyro);
    }
    imu_divide_single(&left_imu_data.offset, 2000);
    left_calibrated = 1;
    ESP_LOGI(TAG, "Left Gyro calibration done!");

    struct mahony_filter mahony = create_mahony_filter(NULL);

    while (true) {
        if (!imu_read(imu, &left_imu_data)) {
            ESP_LOGE("imu left", "err left imu, T=%d, Y=%d", channels[THROTTLE], channels[YAW]);

            if (!channels[FAILSAFE_CHANNEL]) {
                left_error++;

                if (left_error > 10) {
                    ESP_LOGI("imu left", "enabling failsafe");
                    channels[FAILSAFE_CHANNEL] = MAX_CHANNEL_VALUE;
                }
            }
            continue;
        } else if (left_error) {
            left_error = 0;
        }

        apply_mahony_filter(&mahony, &left_imu_data.gyro, &left_imu_data.acce,
            left_imu_data.delta_t, left_imu_data.q);
        imu_process(&left_imu_data);

        applyDeadzone(&left_imu_data.processed.y, 0, 10);

        channels[THROTTLE] = mapValue(left_imu_data.processed.x, -45, 45, MAX_CHANNEL_VALUE, 0);
        channels[YAW] = mapValue(left_imu_data.processed.y, -45, 45, MAX_CHANNEL_VALUE, 0);


        last_left_imu_y = left_imu_data.processed.y;
        last_left_imu_x = left_imu_data.processed.x;

        if (imu_stuck_counter > IMU_STUCK_THRESHOLD) {
            /* Consolidated log emitted by the right IMU task to keep output on one line. */
        } else if (channels[THROTTLE] > 0 && channels[THROTTLE] < MAX_CHANNEL_VALUE) {
            if (channels[THROTTLE] == last_throttle) {
                imu_stuck_counter++;
            } else {
                imu_stuck_counter = 0;
            }
        }

        last_throttle = channels[THROTTLE];

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void right_imu_task(void *pvParameters) {
    (void)pvParameters;

    struct full_imu_data right_imu_data = create_full_imu_data();
    mpu6050_handle_t imu = imu_init(I2C_NUM_0, MPU6050_I2C_ADDRESS);

    // Calibration
    for (int i = 0; i < 2000; i++) {
        imu_read_raw(imu, &right_imu_data.gyro, &right_imu_data.acce);

        imu_add(&right_imu_data.offset, right_imu_data.gyro);
    }
    imu_divide_single(&right_imu_data.offset, 2000);
    ESP_LOGI(TAG, "Right Gyro calibration done!");
    right_calibrated = 1;

    struct mahony_filter mahony = create_mahony_filter(NULL);

    while (true) {
        if (!imu_read(imu, &right_imu_data)) {
            // ESP_LOGE("imu right", "err right imu, R=%d, P=%d", channels[ROLL], channels[PITCH]);

            if (!channels[FAILSAFE_CHANNEL]) {
                right_error++;

                if (right_error > 10) {
                    ESP_LOGI("imu right", "enabling failsafe");
                    channels[FAILSAFE_CHANNEL] = MAX_CHANNEL_VALUE;
                }
            }

            continue;
        } else if (right_error) {
            right_error = 0;
        }


        apply_mahony_filter(&mahony, &right_imu_data.gyro, &right_imu_data.acce,
            right_imu_data.delta_t, right_imu_data.q);
        imu_process(&right_imu_data);

        applyDeadzone(&right_imu_data.processed.y, 0, 10);
        applyDeadzone(&right_imu_data.processed.x, 0, 10);

        channels[ROLL] = mapValue(right_imu_data.processed.y, -45, 45, MAX_CHANNEL_VALUE, 0);
        channels[PITCH] = mapValue(right_imu_data.processed.x, -45, 45, MAX_CHANNEL_VALUE, 0);

        last_right_imu_y = right_imu_data.processed.y;
        last_right_imu_x = right_imu_data.processed.x;

        if (imu_stuck_counter > IMU_STUCK_THRESHOLD) {
            ESP_LOGI(TAG, "IMU stuck -> left y=%.2f x=%.2f | right y=%.2f x=%.2f",
                last_left_imu_y, last_left_imu_x, last_right_imu_y, last_right_imu_x);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void elrs_task(void *pvParameters) {
    (void)pvParameters;

    uint8_t packet[MAX_PACKET_LENGTH] = { 0 };
    // uint8_t buffer[256] = {0};
    // size_t len = 0;

    while (true) {
        // update_yaw_pid(channels, &yaw_pid, get_interpolated_yaw(&crsf_data.attitude), xTaskGetTickCount() * portTICK_PERIOD_MS);
        // if(should_subscribe){
        //     uint8_t packet2[MAX_PACKET_LENGTH] = { 0 };
        //     for(int i = 0; i < 5; i++){
        //     create_subscribe_packet(0x09, packet2); //baro altitude
        //     elrs_send_data(UART_NUM, packet2, MODEL_SWITCH_PACKET_LENGTH);

        //     vTaskDelay(pdMS_TO_TICKS(15));

        // }
        // should_subscribe = 0;
        // }
        if (should_transmit && left_calibrated && right_calibrated) {
            should_transmit = 0;
            if (should_switch) {
                create_model_switch_packet(current_id, packet);
                elrs_send_data(UART_NUM, packet, MODEL_SWITCH_PACKET_LENGTH);
                should_switch = 0;
            } else {
                create_crsf_channels_packet(channels, packet);
                elrs_send_data(UART_NUM, packet, CHANNEL_PACKET_LENGTH);
                // ESP_LOGI("channel", "mech = %d", channels[MECHANISM_CHANNEL]);
                // ESP_LOGI("channel", "r%dp%dy%da%d", channels[ROLL], channels[PITCH], channels[YAW], channels[ARMING_CHANNEL]);
                // ESP_LOGI("telemetry", "alt:%.2f,vspd:%.2f,y:%.2f", crsf_data.baro_alt, crsf_data.vspd, crsf_data.attitude.yaw );
            }

        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }


}



void sim_output_task(void *pvParameters) {
    (void)pvParameters;

    while (true) {
        printf("SIM,%u,%u,%u,%u,%d,%d\n",
            (unsigned)channels[THROTTLE],
            (unsigned)channels[ROLL],
            (unsigned)channels[PITCH],
            (unsigned)channels[YAW],
            read_active_low_button(LEFT_POINT),
            read_active_low_button(RIGHT_POINT));
        fflush(stdout);

        vTaskDelay(pdMS_TO_TICKS(SIM_OUTPUT_INTERVAL_MS));
    }
}

void app_main(void) {
    gpio_init();
    sim_input_init();
    uart_init();
    i2c_init();
    timer_init();

    xTaskCreatePinnedToCore(left_imu_task, "left_imu", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(right_imu_task, "right_imu", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(elrs_task, "elrs_writer", 4096, NULL, tskIDLE_PRIORITY, NULL, 0);
    xTaskCreatePinnedToCore(sim_output_task, "sim_output", 4096, NULL, tskIDLE_PRIORITY, NULL, 0);
}
