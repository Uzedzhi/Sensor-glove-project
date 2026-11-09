#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define BNO055_CHIP_ID_VALUE 0xA0

#define BNO055_REG_CHIP_ID 0x00
#define BNO055_REG_EULER_HEADING_LSB 0x1A
#define BNO055_REG_CALIB_STATUS 0x35
#define BNO055_REG_SELFTEST_RESULT 0x36
#define BNO055_REG_SYSTEM_STATUS 0x39
#define BNO055_REG_SYSTEM_ERROR 0x3A
#define BNO055_REG_UNIT_SELECT 0x3B
#define BNO055_REG_OPERATION_MODE 0x3D
#define BNO055_REG_POWER_MODE 0x3E
#define BNO055_REG_SYSTEM_TRIGGER 0x3F
#define BNO055_REG_PAGE_ID 0x07

#define BNO055_OPERATION_MODE_CONFIG 0x00
#define BNO055_OPERATION_MODE_NDOF 0x0C
#define BNO055_POWER_MODE_NORMAL 0x00

#define BNO055_I2C_TIMEOUT_MS 1000
#define BNO055_POWER_ON_DELAY_MS 700

static const char *TAG = "bno055";
static i2c_master_dev_handle_t bno055_device;

static esp_err_t bno055_read(uint8_t register_address, uint8_t *data,
                             size_t length)
{
    return i2c_master_transmit_receive(bno055_device,
                                       &register_address,
                                       sizeof(register_address),
                                       data,
                                       length,
                                       BNO055_I2C_TIMEOUT_MS);
}

static esp_err_t bno055_read_byte(uint8_t register_address, uint8_t *value)
{
    return bno055_read(register_address, value, 1);
}

static esp_err_t bno055_write_byte(uint8_t register_address, uint8_t value)
{
    const uint8_t bytes[] = {register_address, value};
    return i2c_master_transmit(bno055_device,
                               bytes,
                               sizeof(bytes),
                               BNO055_I2C_TIMEOUT_MS);
}

static int16_t read_little_endian_i16(const uint8_t *bytes)
{
    return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static i2c_master_bus_handle_t init_i2c_bus(void)
{
    i2c_master_bus_handle_t bus_handle;
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_BNO055_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_BNO055_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    ESP_LOGI(TAG, "I2C initialized: SDA=GPIO%d, SCL=GPIO%d, frequency=%d Hz",
             CONFIG_BNO055_I2C_SDA_GPIO,
             CONFIG_BNO055_I2C_SCL_GPIO,
             CONFIG_BNO055_I2C_FREQUENCY_HZ);
    return bus_handle;
}

static uint8_t detect_bno055_address(i2c_master_bus_handle_t bus_handle)
{
    static const uint8_t possible_addresses[] = {0x28, 0x29};

    while (true) {
        for (size_t index = 0; index < sizeof(possible_addresses); ++index) {
            const uint8_t address = possible_addresses[index];
            if (i2c_master_probe(bus_handle, address, 100) == ESP_OK) {
                ESP_LOGI(TAG, "I2C device detected at address 0x%02X", address);
                return address;
            }
        }

        ESP_LOGE(TAG, "BNO055 not found at 0x28 or 0x29; check 3V3, GND, SDA and SCL");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void add_bno055_device(i2c_master_bus_handle_t bus_handle,
                              uint8_t address)
{
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = CONFIG_BNO055_I2C_FREQUENCY_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle,
                                               &device_config,
                                               &bno055_device));
}

static bool init_bno055(void)
{
    uint8_t chip_id = 0;
    ESP_ERROR_CHECK(bno055_read_byte(BNO055_REG_CHIP_ID, &chip_id));

    if (chip_id != BNO055_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "Unexpected CHIP_ID: 0x%02X (expected 0x%02X)",
                 chip_id, BNO055_CHIP_ID_VALUE);
        return false;
    }

    ESP_LOGI(TAG, "BNO055 confirmed, CHIP_ID=0x%02X", chip_id);

    ESP_ERROR_CHECK(bno055_write_byte(BNO055_REG_PAGE_ID, 0x00));
    ESP_ERROR_CHECK(bno055_write_byte(BNO055_REG_OPERATION_MODE,
                                      BNO055_OPERATION_MODE_CONFIG));
    vTaskDelay(pdMS_TO_TICKS(30));

    ESP_ERROR_CHECK(bno055_write_byte(BNO055_REG_POWER_MODE,
                                      BNO055_POWER_MODE_NORMAL));
    ESP_ERROR_CHECK(bno055_write_byte(BNO055_REG_UNIT_SELECT, 0x00));
    ESP_ERROR_CHECK(bno055_write_byte(BNO055_REG_SYSTEM_TRIGGER, 0x00));
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_ERROR_CHECK(bno055_write_byte(BNO055_REG_OPERATION_MODE,
                                      BNO055_OPERATION_MODE_NDOF));
    vTaskDelay(pdMS_TO_TICKS(1000));

    uint8_t selftest = 0;
    uint8_t system_status = 0;
    uint8_t system_error = 0;
    ESP_ERROR_CHECK(bno055_read_byte(BNO055_REG_SELFTEST_RESULT, &selftest));
    ESP_ERROR_CHECK(bno055_read_byte(BNO055_REG_SYSTEM_STATUS, &system_status));
    ESP_ERROR_CHECK(bno055_read_byte(BNO055_REG_SYSTEM_ERROR, &system_error));

    ESP_LOGI(TAG, "NDOF mode enabled: self-test=0x%02X, system=%u, error=%u",
             selftest, system_status, system_error);
    return true;
}

static void log_bno055_data(void)
{
    uint8_t raw[20];
    uint8_t calibration;

    const esp_err_t data_result = bno055_read(BNO055_REG_EULER_HEADING_LSB,
                                               raw,
                                               sizeof(raw));
    const esp_err_t calibration_result = bno055_read_byte(
        BNO055_REG_CALIB_STATUS, &calibration);

    if (data_result != ESP_OK || calibration_result != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 read failed: data=%s, calibration=%s",
                 esp_err_to_name(data_result),
                 esp_err_to_name(calibration_result));
        return;
    }

    const float heading = read_little_endian_i16(&raw[0]) / 16.0f;
    const float roll = read_little_endian_i16(&raw[2]) / 16.0f;
    const float pitch = read_little_endian_i16(&raw[4]) / 16.0f;

    const float quaternion_w = read_little_endian_i16(&raw[6]) / 16384.0f;
    const float quaternion_x = read_little_endian_i16(&raw[8]) / 16384.0f;
    const float quaternion_y = read_little_endian_i16(&raw[10]) / 16384.0f;
    const float quaternion_z = read_little_endian_i16(&raw[12]) / 16384.0f;

    const float linear_x = read_little_endian_i16(&raw[14]) / 100.0f;
    const float linear_y = read_little_endian_i16(&raw[16]) / 100.0f;
    const float linear_z = read_little_endian_i16(&raw[18]) / 100.0f;

    const unsigned calibration_system = (calibration >> 6) & 0x03;
    const unsigned calibration_gyro = (calibration >> 4) & 0x03;
    const unsigned calibration_accel = (calibration >> 2) & 0x03;
    const unsigned calibration_mag = calibration & 0x03;

    ESP_LOGI(TAG,
             "Euler[deg] H=%7.2f R=%7.2f P=%7.2f | Linear[m/s2] X=%6.2f Y=%6.2f Z=%6.2f",
             heading, roll, pitch, linear_x, linear_y, linear_z);
    ESP_LOGI(TAG,
             "Quaternion W=%6.3f X=%6.3f Y=%6.3f Z=%6.3f | Cal SYS=%u GYR=%u ACC=%u MAG=%u",
             quaternion_w, quaternion_x, quaternion_y, quaternion_z,
             calibration_system, calibration_gyro,
             calibration_accel, calibration_mag);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting BNO055 on Seeed Studio XIAO ESP32-C3");
    vTaskDelay(pdMS_TO_TICKS(BNO055_POWER_ON_DELAY_MS));

    const i2c_master_bus_handle_t bus_handle = init_i2c_bus();
    const uint8_t address = detect_bno055_address(bus_handle);
    add_bno055_device(bus_handle, address);

    if (!init_bno055()) {
        ESP_LOGE(TAG, "BNO055 initialization stopped");
        return;
    }

    while (true) {
        log_bno055_data();
        vTaskDelay(pdMS_TO_TICKS(CONFIG_BNO055_OUTPUT_INTERVAL_MS));
    }
}
