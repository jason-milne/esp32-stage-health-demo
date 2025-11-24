#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "IMU_SPI_DEMO";

// SPI pin mapping (ESP32-S3 DevKitC)
#define PIN_NUM_MOSI  11     // SDI (IMU)
#define PIN_NUM_MISO  13     // SDO (IMU)
#define PIN_NUM_CLK   12     // SCK
#define PIN_NUM_CS    10     // CS

// ICM-42688-P registers (bank 0)
#define ICM42688_REG_WHO_AM_I       0x75
#define ICM42688_REG_PWR_MGMT0      0x4E
#define ICM42688_REG_TEMP_DATA1     0x1D
#define ICM42688_REG_TEMP_DATA0     0x1E
// Sensor data registers (User Bank 0)
// TEMP_DATA1 is at 0x1D in the datasheet, everything after that is contiguous
#define ICM42688_TEMP_DATA1      0x1D
#define ICM42688_TEMP_DATA0      0x1E

#define ICM42688_ACCEL_DATA_X1   0x1F
#define ICM42688_ACCEL_DATA_X0   0x20
#define ICM42688_ACCEL_DATA_Y1   0x21
#define ICM42688_ACCEL_DATA_Y0   0x22
#define ICM42688_ACCEL_DATA_Z1   0x23
#define ICM42688_ACCEL_DATA_Z0   0x24

#define ICM42688_GYRO_DATA_X1    0x25
#define ICM42688_GYRO_DATA_X0    0x26
#define ICM42688_GYRO_DATA_Y1    0x27
#define ICM42688_GYRO_DATA_Y0    0x28
#define ICM42688_GYRO_DATA_Z1    0x29
#define ICM42688_GYRO_DATA_Z0    0x2A


#define ICM42688_WHO_AM_I_EXPECTED  0x47

static spi_device_handle_t icm_handle;

// --- Low-level helpers ------------------------------------------------

static esp_err_t icm_spi_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;        // write: MSB = 0
    tx[1] = value;

    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
        .rx_buffer = NULL,
    };

    return spi_device_transmit(icm_handle, &t);
}

static esp_err_t icm42688_spi_read(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t txbuf[1 + len];
    uint8_t rxbuf[1 + len];

    txbuf[0] = reg | 0x80;   // MSB=1 for read
    for (size_t i = 1; i < 1 + len; i++) {
        txbuf[i] = 0x00;     // dummy bytes
    }

    spi_transaction_t t = {
        .length    = (1 + len) * 8,
        .tx_buffer = txbuf,
        .rx_buffer = rxbuf,
    };

    esp_err_t ret = spi_device_transmit(icm_handle, &t);
    if (ret != ESP_OK) {
        return ret;
    }

    // Skip the first dummy byte — the rest are data
    for (size_t i = 0; i < len; i++) {
        data[i] = rxbuf[i + 1];
    }

    return ESP_OK;
}


static esp_err_t icm_spi_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg | 0x80;        // read: MSB = 1
    tx[1] = 0x00;

    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t ret = spi_device_transmit(icm_handle, &t);
    if (ret != ESP_OK) return ret;

    *value = rx[1];            // second byte is the data
    return ESP_OK;
}


static esp_err_t icm42688_read_accel_gyro_raw(
        int16_t *ax, int16_t *ay, int16_t *az,
        int16_t *gx, int16_t *gy, int16_t *gz)
{
    esp_err_t err;
    uint8_t hi, lo;

    // Accel X
    err = icm42688_spi_read(ICM42688_ACCEL_DATA_X1, &hi, 1);
    if (err != ESP_OK) return err;
    err = icm42688_spi_read(ICM42688_ACCEL_DATA_X0, &lo, 1);
    if (err != ESP_OK) return err;
    *ax = (int16_t)((hi << 8) | lo);

    // Accel Y
    err = icm42688_spi_read(ICM42688_ACCEL_DATA_Y1, &hi, 1);
    if (err != ESP_OK) return err;
    err = icm42688_spi_read(ICM42688_ACCEL_DATA_Y0, &lo, 1);
    if (err != ESP_OK) return err;
    *ay = (int16_t)((hi << 8) | lo);

    // Accel Z
    err = icm42688_spi_read(ICM42688_ACCEL_DATA_Z1, &hi, 1);
    if (err != ESP_OK) return err;
    err = icm42688_spi_read(ICM42688_ACCEL_DATA_Z0, &lo, 1);
    if (err != ESP_OK) return err;
    *az = (int16_t)((hi << 8) | lo);

    // Gyro X
    err = icm42688_spi_read(ICM42688_GYRO_DATA_X1, &hi, 1);
    if (err != ESP_OK) return err;
    err = icm42688_spi_read(ICM42688_GYRO_DATA_X0, &lo, 1);
    if (err != ESP_OK) return err;
    *gx = (int16_t)((hi << 8) | lo);

    // Gyro Y
    err = icm42688_spi_read(ICM42688_GYRO_DATA_Y1, &hi, 1);
    if (err != ESP_OK) return err;
    err = icm42688_spi_read(ICM42688_GYRO_DATA_Y0, &lo, 1);
    if (err != ESP_OK) return err;
    *gy = (int16_t)((hi << 8) | lo);

    // Gyro Z
    err = icm42688_spi_read(ICM42688_GYRO_DATA_Z1, &hi, 1);
    if (err != ESP_OK) return err;
    err = icm42688_spi_read(ICM42688_GYRO_DATA_Z0, &lo, 1);
    if (err != ESP_OK) return err;
    *gz = (int16_t)((hi << 8) | lo);

    return ESP_OK;
}

// --- Mid-level sensor helpers -----------------------------------------

static esp_err_t icm_read_who_am_i(uint8_t *who)
{
    return icm_spi_read_reg(ICM42688_REG_WHO_AM_I, who);
}

// Minimal power-up: enable accel+gyro in low-noise mode so temp updates
static esp_err_t icm_init(void)
{
    // PWR_MGMT0 = 0x0F : gyro+accel low-noise mode, temp enabled
    esp_err_t ret = icm_spi_write_reg(ICM42688_REG_PWR_MGMT0, 0x0F);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10));  // allow sensors to start

    return ESP_OK;
}

static esp_err_t icm_read_temperature(float *temp_c)
{
    uint8_t hi = 0, lo = 0;
    esp_err_t r1 = icm_spi_read_reg(ICM42688_REG_TEMP_DATA1, &hi);
    esp_err_t r2 = icm_spi_read_reg(ICM42688_REG_TEMP_DATA0, &lo);
    if (r1 != ESP_OK) return r1;
    if (r2 != ESP_OK) return r2;

    int16_t raw = ((int16_t)hi << 8) | lo;

    // Datasheet: Temp[°C] = TEMP_DATA / 132.48 + 25
    *temp_c = ((float)raw / 132.48f) + 25.0f;
    return ESP_OK;
}

// --- SPI bus / device init --------------------------------------------

static void icm_spi_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num   = PIN_NUM_MOSI,
        .miso_io_num   = PIN_NUM_MISO,
        .sclk_io_num   = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,   // 1 MHz
        .mode           = 0,
        .spics_io_num   = PIN_NUM_CS,
        .queue_size     = 1,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &icm_handle));
}

// --- app_main ----------------------------------------------------------

void app_main(void)
{
    ESP_LOGI(TAG, "IMU SPI demo starting");

    icm_spi_init();
    ESP_LOGI(TAG, "SPI initialised");

    uint8_t who = 0;
    if (icm_read_who_am_i(&who) == ESP_OK) {
        ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (expected 0x%02X)",
                 who, ICM42688_WHO_AM_I_EXPECTED);
    } else {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I");
    }

    if (icm_init() != ESP_OK) {
        ESP_LOGE(TAG, "ICM init failed");
    } else {
        ESP_LOGI(TAG, "ICM init OK");
    }

    while (1) {
        float temp_c;
        int16_t ax, ay, az;
        int16_t gx, gy, gz;

        esp_err_t err = icm_read_temperature(&temp_c);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "IMU_SPI_DEMO: Failed to read temperature: %s", esp_err_to_name(err));
        }

        err = icm42688_read_accel_gyro_raw(&ax, &ay, &az, &gx, &gy, &gz);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "IMU_SPI_DEMO: Failed to read accel/gyro: %s", esp_err_to_name(err));
        } else {
            // Raw counts for now, we can add scaling later
            ESP_LOGI(TAG,
                    "IMU_SPI_DEMO: Temp=%.2f C, "
                    "ACCEL [x y z] = [%6d %6d %6d], "
                    "GYRO [x y z] = [%6d %6d %6d]",
                    temp_c, ax, ay, az, gx, gy, gz);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
