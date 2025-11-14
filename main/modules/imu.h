#ifndef IMU_H
#define IMU_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>

// ======================= PIN CONFIG =========================
#define SPI_BUS         SPI2_HOST
#define PIN_NUM_CLK     18
#define PIN_NUM_MOSI    23
#define PIN_NUM_MISO    19
#define PIN_NUM_CSA      5
#define PIN_NUM_CSB      4

// ======================= REGISTER MAP =======================
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75
#define REG_USER_CTRL       0x6A
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_CONFIG2   0x1D
#define REG_GYRO_CONFIG     0x1B

// ======================= DATA STRUCT ========================
typedef enum {
    IMU_LEFT,
    IMU_RIGHT
} imu_side_t;

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} imu_data_t;

// ======================= FUNCTION ===========================
esp_err_t imu_init(void);
esp_err_t imu_read(imu_side_t side, imu_data_t *data);

#endif
