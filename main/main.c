#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "modules/utils.c"  
#include "modules/kalman.h"
#include "modules/elrs.h"
#include "modules/button.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "driver/spi_master.h"

#define TAG "main"

#define INTERVALMS 3 * 1000

#define UART_NUM UART_NUM_2
#define TX_PIN 17
#define RX_PIN 16

#define ARMING_CHANNEL AUX1
#define CAMSWITCH_CHANNEL AUX2
#define MECHANISM_CHANNEL AUX3
#define FAILSAFE_CHANNEL AUX4

#define RIGHT_POINT GPIO_NUM_33
#define RIGHT_MIDDLE GPIO_NUM_32
#define RIGHT_RING GPIO_NUM_35
#define RIGHT_LITTLE GPIO_NUM_34

#define LEFT_POINT GPIO_NUM_14
#define LEFT_MIDDLE GPIO_NUM_27
#define LEFT_RING GPIO_NUM_26
#define LEFT_LITTLE GPIO_NUM_25

#define SPI_BUS         SPI2_HOST
#define PIN_NUM_CLK     18
#define PIN_NUM_MOSI    23
#define PIN_NUM_MISO    19
#define PIN_NUM_CSA      5
#define PIN_NUM_CSB      4

#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75
#define REG_USER_CTRL       0x6A
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_CONFIG2   0x1D
#define REG_CONFIG          0x1A

spi_device_handle_t spi_handleA;
spi_device_handle_t spi_handleB;

uint16_t channels[16] = { 0 };

#define DRONE_COUNT 2
int8_t current_id = 1;
int8_t shift = 0;
bool should_transmit = 0;
bool should_switch = 1;

//////////////////////////////////////BUTTON////////////////////////////////////////////////////
void button_control () {
    while(1) {
    bool L_Point = button_get_state(4);
    bool L_Mid = button_get_state(5);
    bool L_Ring = button_get_state(6);
    bool L_Little = button_get_state(7);

    uint8_t val = (L_Little << 3) | (L_Ring << 2) | (L_Mid << 1) | (L_Point << 0);
    switch(val) {
    case 0b1110: shift = 1; break;  
    case 0b1101: shift = 2; break;  
    case 0b1011: shift = 3; break;  
    case 0b0111: shift = 4; break; 
    default: shift = 0;
}
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//////////////////////////////////////IMU///////////////////////////////////////////////////////
void mpu_write_byte(spi_device_handle_t handle, uint8_t reg, uint8_t data) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.flags = SPI_TRANS_USE_TXDATA; 
    t.cmd = reg & 0x7F;             
    t.tx_data[0] = data;
    t.length = 8;                

    esp_err_t ret = spi_device_polling_transmit(handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI Write to reg 0x%02X failed", reg);
    }
}

void mpu_read_bytes(spi_device_handle_t handle, uint8_t reg, uint8_t *buffer, size_t len) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.cmd = reg | 0x80;           
    t.length = 8;                  
    t.rxlength = len * 8;
    t.flags = 0;      
    t.rx_buffer = buffer;

    esp_err_t ret = spi_device_polling_transmit(handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI Read from reg 0x%02X failed", reg);
    }
}

void left_imu_task() {
    uint8_t buffer[14];
    ESP_LOGI(TAG, "Starting MPU-6500 Initialization...");
    
    mpu_write_byte(spi_handleA, REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));

    mpu_write_byte(spi_handleA, REG_USER_CTRL, 0x10);  
    mpu_write_byte(spi_handleA, REG_PWR_MGMT_1, 0x01); 
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t whoA;
    mpu_read_bytes(spi_handleA, REG_WHO_AM_I, &whoA, 1);
    if (whoA != 0x70) {
        ESP_LOGE(TAG, "WhoAmIA check failed. Expected 0x70, Found: 0x%02X", whoA);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "WhoAmIA check passed. Found MPU-6500 (0x%02X)", whoA);
    mpu_write_byte(spi_handleA, REG_ACCEL_CONFIG, 0x08);
    float accel_scale = 8192.0f; 
    
    mpu_write_byte(spi_handleA, REG_ACCEL_CONFIG2, 0x03);

    mpu_write_byte(spi_handleA, REG_USER_CTRL, 0x80);

    ESP_LOGI(TAG, "MPU-6500 configured successfully. Starting data acquisition.");

    while(1) {
        mpu_read_bytes(spi_handleA, REG_ACCEL_XOUT_H, buffer, 14);

        int16_t ax_raw = (int16_t)(buffer[0] << 8 | buffer[1]);
        int16_t ay_raw = (int16_t)(buffer[2] << 8 | buffer[3]);
        int16_t az_raw = (int16_t)(buffer[4] << 8 | buffer[5]);

        float ax = (float)ax_raw / accel_scale;
        float ay = (float)ay_raw / accel_scale;
        float az = (float)az_raw / accel_scale;

        float F1 = kalmanFilterMulti(ay, 0);
        float F2 = kalmanFilterMulti(ax, 1);
        float F3 = kalmanFilterMulti(az, 2);

        float yaw  = atan2(F1, F3) * 180.0 / M_PI;
        float throttle = atan2(-F2, sqrt(F1 * F1 + F3 * F3)) * 180.0 / M_PI;

        if (throttle <= -50) {
         channels[THROTTLE] = 0; 
        }
        else {
        channels[THROTTLE] = mapFloatToInt(throttle, -51, 50, 0, MAX_CHANNEL_VALUE);
        }

        if (yaw >= -20 && yaw <=20) {
         channels[YAW] = 1000; 
        }
        else if (yaw >=-60 && yaw <=-21) {
        channels[YAW] = mapFloatToInt(yaw, -50, -21, 0, 1000);
        }
        else if (yaw >=21 && yaw <=60) {
        channels[YAW] = mapFloatToInt(yaw, 21, 60, 1000, 2000);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void right_imu_task() {
    uint8_t buffer[14];
    ESP_LOGI(TAG, "Starting MPU-6500 Initialization...");
    
    mpu_write_byte(spi_handleB, REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));

    mpu_write_byte(spi_handleB, REG_USER_CTRL, 0x10);  
    mpu_write_byte(spi_handleB, REG_PWR_MGMT_1, 0x01); 
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t whoB;
    mpu_read_bytes(spi_handleB, REG_WHO_AM_I, &whoB, 1);
    if (whoB != 0x70) {
        ESP_LOGE(TAG, "WhoAmIB check failed. Expected 0x70, Found: 0x%02X", whoB);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "WhoAmIB check passed. Found MPU-6500 (0x%02X)", whoB);
    mpu_write_byte(spi_handleB, REG_ACCEL_CONFIG, 0x08);
    float accel_scale = 8192.0f; 
    
    mpu_write_byte(spi_handleB, REG_ACCEL_CONFIG2, 0x03);

    mpu_write_byte(spi_handleB, REG_USER_CTRL, 0x80);

    ESP_LOGI(TAG, "MPU-6500 configured successfully. Starting data acquisition.");

    while(1) {
        mpu_read_bytes(spi_handleB, REG_ACCEL_XOUT_H, buffer, 14);

        int16_t ax_raw = (int16_t)(buffer[0] << 8 | buffer[1]);
        int16_t ay_raw = (int16_t)(buffer[2] << 8 | buffer[3]);
        int16_t az_raw = (int16_t)(buffer[4] << 8 | buffer[5]);

        float ax = (float)ax_raw / accel_scale;
        float ay = (float)ay_raw / accel_scale;
        float az = (float)az_raw / accel_scale;

        float F4 = kalmanFilterMulti(ay, 3);
        float F5 = kalmanFilterMulti(ax, 4);
        float F6 = kalmanFilterMulti(az, 5);

        float roll =atan2(F4, F6) * 180.0 / M_PI;
        float pitch = atan2(-F5, sqrt(F4 * F4 + F6 * F6)) * 180.0 / M_PI;

        if (roll >= -20 && roll <=20) {
         channels[ROLL] = 1000; 
        }
        else if (roll >=-60 && roll <=-21) {
        channels[ROLL] = mapFloatToInt(roll, -50, -21, 0, 1000);
        }
        else if (roll >=21 && roll <=60) {
        channels[ROLL] = mapFloatToInt(roll, 21, 60, 1000, 2000);
        }

        if (pitch >= -20 && pitch <=20) {
         channels[PITCH] = 1000; 
        }
        else if (pitch >=-60 && pitch <=-21) {
        channels[PITCH] = mapFloatToInt(pitch, -50, -21, 0, 1000);
        }
        else if (pitch >=21 && pitch <=60) {
        channels[PITCH] = mapFloatToInt(pitch, 21, 60, 1000, 2000);
        }

        int R=kalmanFilterMulti(channels[ROLL],6);
        ESP_LOGI("OUT", "RAW: %d | OK: %d", channels[ROLL], R);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

////////////////////////////////////TIMER///////////////////////////////////////////////////////
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

//////////////////////////////////////ELRS//////////////////////////////////////////////////////
void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = 921600,
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

void elrs_task(void *pvParameters) {
    uint8_t packet[MAX_PACKET_LENGTH] = { 0 };

    while (true) {
        if (should_transmit) {
            should_transmit = 0;
            if (should_switch) {
                create_model_switch_packet(current_id, packet);
                elrs_send_data(UART_NUM, packet, MODEL_SWITCH_PACKET_LENGTH);
                should_switch = 0;
            } else {
                create_crsf_channels_packet(channels, packet);
                elrs_send_data(UART_NUM, packet, CHANNEL_PACKET_LENGTH);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
void logS() {
    while(1) {
    ESP_LOGI("OUT", "Roll: %d | Pitch: %d", channels[ROLL],channels[PITCH]);
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    uart_init();
    timer_init();
    button_init();

    button_register(RIGHT_POINT);
    button_register(RIGHT_MIDDLE);
    button_register(RIGHT_RING);
    button_register(RIGHT_LITTLE);
    button_register(LEFT_POINT);
    button_register(LEFT_MIDDLE);
    button_register(LEFT_RING);
    button_register(LEFT_LITTLE);

    spi_bus_config_t bus_config = {
        .sclk_io_num = PIN_NUM_CLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_BUS, &bus_config, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_configA = {
        .command_bits = 8,
        .address_bits = 0,
        .clock_speed_hz = 1000000,
        .mode = 0,                
        .spics_io_num = PIN_NUM_CSA,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX, 
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_BUS, &dev_configA, &spi_handleA));

    spi_device_interface_config_t dev_configB = {
        .command_bits = 8,
        .address_bits = 0,
        .clock_speed_hz = 1000000,
        .mode = 0,                
        .spics_io_num = PIN_NUM_CSB,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX, 
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_BUS, &dev_configB, &spi_handleB));

    xTaskCreatePinnedToCore(left_imu_task, "left_imu", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(right_imu_task, "right_imu", 4096, NULL, 4, NULL, 1);
    // xTaskCreatePinnedToCore(logS, "log", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(button_control, "button", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(elrs_task, "elrs_writer", 4096, NULL, tskIDLE_PRIORITY, NULL, 0);
}