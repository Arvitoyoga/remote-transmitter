#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "modules/utils.c"  
#include "modules/elrs.h"
#include "modules/button.h"
#include "modules/filter.h"
#include "modules/imu.h"
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

#define DRONE_COUNT 3

filter_t imu_left;
filter_t imu_right;
uint16_t channels[16] = { 0 };

int8_t current_id = 1;
int8_t shift = 0;
int16_t max_mechanism_value = 1378;
imu_data_t imuR, imuL;

bool should_transmit = 0;
bool should_switch = 1;

bool R_Point;
bool R_Mid;
bool R_Ring;
bool R_Little;
bool L_Point;
bool L_Mid;
bool L_Ring;
bool L_Little;


//////////////////////////////////////BUTTON////////////////////////////////////////////////////
void button_control () {
    while(1) {
    L_Point = button_get_state(4);
    L_Mid = button_get_state(5);
    L_Ring = button_get_state(6);
    L_Little = button_get_state(7);

    uint8_t val = (L_Little << 3) | (L_Ring << 2) | (L_Mid << 1) | (L_Point << 0);
    switch(val) {
    case 0b1110: shift = 1; break;  
    case 0b1101: shift = 2; break;  
    case 0b1011: shift = 3; break;  
    case 0b0111: shift = 4; break; 
    default: shift = 0;
    }

    switch (shift)
    {
    case 0:
        R_Point = button_check_hold_release(0,500,500);
        channels[ARMING_CHANNEL] = (R_Point == 0) ? 1000 : MAX_CHANNEL_VALUE;

        R_Mid = button_toggle(1);
        channels[CAMSWITCH_CHANNEL] = (R_Mid == 0) ? 1000 : MAX_CHANNEL_VALUE;

        R_Ring = button_toggle(2);
        channels[FAILSAFE_CHANNEL] = (R_Ring == 0) ? 1000 : MAX_CHANNEL_VALUE;

        R_Little = button_toggle_delay(3,10);
        if(R_Little==0) {
            should_switch = 1;
            current_id++;
            current_id %= DRONE_COUNT;
        }
        break;


        case 1:
        {
        R_Point = button_get_state(0);
        if(R_Point==0) {max_mechanism_value++;}

        R_Mid = button_get_state(1);
        if(R_Mid==0) {max_mechanism_value--;}

        R_Ring = button_toggle_delay(2,10);
        if(R_Ring==0) {
        channels[MECHANISM_CHANNEL] = (channels[MECHANISM_CHANNEL] <= 100) * max_mechanism_value;
        }
        }
        break;
        
        default:
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//////////////////////////////////////IMU///////////////////////////////////////////////////////
void imu_task(void *pvParameters)
{
    imu_init();
    filter_init(&imu_right,500.0f);
    filter_set_gains(&imu_right,1.0f, 0.002f);
    filter_set_deadzone(&imu_right,1.0f);

    filter_init(&imu_left,500.0f);
    filter_set_gains(&imu_left,1.0f, 0.002f);
    filter_set_deadzone(&imu_left,1.0f);

    while (1)
    {
    imu_read(IMU_LEFT, &imuR);
    imu_read(IMU_RIGHT, &imuL);

    // update filter
    filter_update_raw(&imu_right,imuR.ax, imuR.ay, imuR.az, imuR.gx, imuR.gy, imuR.gz);
    filter_update_raw(&imu_left,imuL.ax, imuL.ay, imuL.az, imuL.gx, imuL.gy, imuL.gz);

    float roll, pitch, ncA;
    filter_get_euler(&imu_right,&roll, &pitch, &ncA);
    float yaw, throttle, ncB;
    filter_get_euler(&imu_left,&yaw, &throttle, &ncB);

        // Mapping ROLL
        if (roll >= -10 && roll <= 10) {
            channels[ROLL] = 1000; 
        }
        else if (roll >= -60 && roll <= -21) {
            channels[ROLL] = mapFloatToInt(roll, -60, -21, 0, 1000);
        }
        else if (roll >= 21 && roll <= 60) {
            channels[ROLL] = mapFloatToInt(roll, 21, 60, 1000, 2000);
        }

        // Mapping PITCH
        if (pitch >= -10 && pitch <= 10) {
            channels[PITCH] = 1000; 
        }
        else if (pitch >= -60 && pitch <= -21) {
            channels[PITCH] = mapFloatToInt(pitch, -60, -21, 0, 1000);
        }
        else if (pitch >= 21 && pitch <= 60) {
            channels[PITCH] = mapFloatToInt(pitch, 21, 60, 1000, 2000);
        }

        // Mapping THROTTLE
        if (throttle <= -50) {
         channels[THROTTLE] = 0; 
        }
        else {
        channels[THROTTLE] = mapFloatToInt(throttle, -51, 50, 0, MAX_CHANNEL_VALUE);
        }

        // Mapping YAW
        if (yaw >= -10 && yaw <=10) {
         channels[YAW] = 1000; 
        }
        else if (yaw >=-60 && yaw <=-21) {
        channels[YAW] = mapFloatToInt(yaw, -50, -21, 0, 1000);
        }
        else if (yaw >=21 && yaw <=60) {
        channels[YAW] = mapFloatToInt(yaw, 21, 60, 1000, 2000);
        }


    vTaskDelay(pdMS_TO_TICKS(2));
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

////////////////////////////////////////////////////////////////////////////////////////////////
void logS() {
    while(1) {
    ESP_LOGI("OUT", "ARM: %d | CAM: %d | ID: %d | FAIL: %d | mV: %d | m: %d", channels[AUX1],channels[AUX2],current_id,channels[AUX4],max_mechanism_value,channels[MECHANISM_CHANNEL]);
    // printf("THROTTLE = %d | ROLL = %d | YAW = %d | PITCH = %d\n", channels[THROTTLE],channels[ROLL],channels[YAW],channels[PITCH]);
    // fflush(stdout);
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

    xTaskCreatePinnedToCore(imu_task, "right_imu", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(logS, "log", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(button_control, "button", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(elrs_task, "elrs_writer", 4096, NULL, tskIDLE_PRIORITY, NULL, 0);
}