#include "imu.h"
#include <string.h>

static const char *TAG = "IMU";
static spi_device_handle_t imu_left, imu_right;

static const float accel_scale = 8192.0f; // ±4g
static const float gyro_scale  = 65.5f;   // ±500°/s

// =============================================================
// Low level SPI access
// =============================================================
static void imu_write_byte(spi_device_handle_t handle, uint8_t reg, uint8_t data) {
    spi_transaction_t t = {0};
    uint8_t tx_data[2] = {reg & 0x7F, data};
    t.length = 8 * 2;
    t.tx_buffer = tx_data;
    esp_err_t ret = spi_device_transmit(handle, &t);
if (ret != ESP_OK) {
    ESP_LOGE("IMU", "SPI transmit error: %s", esp_err_to_name(ret));
    return;
}

    if (ret != ESP_OK) ESP_LOGE(TAG, "Write reg 0x%02X failed", reg);
}

static void imu_read_bytes(spi_device_handle_t handle, uint8_t reg, uint8_t *buffer, size_t len) {
    uint8_t tx = reg | 0x80;
    spi_transaction_t t = {0};
    t.length = 8 * (len + 1);
    t.tx_buffer = &tx;
    t.rx_buffer = buffer;
    esp_err_t ret = spi_device_transmit(handle, &t);
if (ret != ESP_OK) {
    ESP_LOGE("IMU", "SPI transmit error: %s", esp_err_to_name(ret));
    return;
}

    if (ret != ESP_OK) ESP_LOGE(TAG, "Read reg 0x%02X failed", reg);
}

// =============================================================
// Initialization
// =============================================================
esp_err_t imu_init(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_BUS, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_cfgA = {
        .mode = 0,
        .clock_speed_hz = 1 * 1000 * 1000,
        .spics_io_num = PIN_NUM_CSA,
        .queue_size = 1,
    };
    spi_device_interface_config_t dev_cfgB = {
        .mode = 0,
        .clock_speed_hz = 1 * 1000 * 1000,
        .spics_io_num = PIN_NUM_CSB,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI_BUS, &dev_cfgA, &imu_right));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_BUS, &dev_cfgB, &imu_left));

    vTaskDelay(pdMS_TO_TICKS(50));

    spi_device_handle_t devices[2] = {imu_right, imu_left};
    const char *names[2] = {"RIGHT", "LEFT"};

    for (int i = 0; i < 2; i++) {
        spi_device_handle_t h = devices[i];

        imu_write_byte(h, REG_PWR_MGMT_1, 0x80);
        vTaskDelay(pdMS_TO_TICKS(100));

        imu_write_byte(h, REG_USER_CTRL, 0x10);
        imu_write_byte(h, REG_PWR_MGMT_1, 0x01);
        vTaskDelay(pdMS_TO_TICKS(50));

        uint8_t who = 0;
        imu_read_bytes(h, REG_WHO_AM_I, &who, 1);
        if (who != 0x70) {
            ESP_LOGE(TAG, "%s IMU WHO_AM_I mismatch: 0x%02X", names[i], who);
            continue;
        }
        ESP_LOGI(TAG, "%s IMU detected (WHO_AM_I=0x%02X)", names[i], who);

        imu_write_byte(h, REG_ACCEL_CONFIG, 0x08);
        imu_write_byte(h, REG_GYRO_CONFIG, 0x08);
        imu_write_byte(h, REG_ACCEL_CONFIG2, 0x03);
        imu_write_byte(h, REG_USER_CTRL, 0x80);
    }

    ESP_LOGI(TAG, "Both IMUs initialized successfully.");
    return ESP_OK;
}

// =============================================================
// Read data
// =============================================================
esp_err_t imu_read(imu_side_t side, imu_data_t *data) {
    spi_device_handle_t handle = (side == IMU_RIGHT) ? imu_right : imu_left;
    uint8_t buffer[15] = {0};

    imu_read_bytes(handle, REG_ACCEL_XOUT_H, buffer, 14);

    int16_t ax_raw = (int16_t)((buffer[1] << 8) | buffer[2]);
    int16_t ay_raw = (int16_t)((buffer[3] << 8) | buffer[4]);
    int16_t az_raw = (int16_t)((buffer[5] << 8) | buffer[6]);
    int16_t gx_raw = (int16_t)((buffer[9] << 8) | buffer[10]);
    int16_t gy_raw = (int16_t)((buffer[11] << 8) | buffer[12]);
    int16_t gz_raw = (int16_t)((buffer[13] << 8) | buffer[14]);

    data->ax = ax_raw / accel_scale;
    data->ay = ay_raw / accel_scale;
    data->az = az_raw / accel_scale;
    data->gx = gx_raw / gyro_scale;
    data->gy = gy_raw / gyro_scale;
    data->gz = gz_raw / gyro_scale;

    return ESP_OK;
}
