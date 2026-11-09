#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

namespace {

using u8 = std::uint8_t;

constexpr char Tag[] = "bno055";
constexpr int I2cTimeoutMs = 20;
constexpr u8 ChipIdRegister = 0x00;
constexpr u8 PageRegister = 0x07;
constexpr u8 QuaternionRegister = 0x20;
constexpr u8 CalibrationRegister = 0x35;
constexpr u8 SystemErrorRegister = 0x3A;
constexpr u8 UnitsRegister = 0x3B;
constexpr u8 ModeRegister = 0x3D;
constexpr u8 PowerRegister = 0x3E;
constexpr u8 TriggerRegister = 0x3F;
constexpr u8 NdofMode = 0x0C;
constexpr std::size_t MaxRetryCount = 3;

constexpr TickType_t TicksForMs(std::uint32_t milliseconds)
{
    // Round up, including when the configured tick is longer than a millisecond.
    return static_cast<TickType_t>(
        (static_cast<std::uint64_t>(milliseconds) * configTICK_RATE_HZ + 999) / 1000);
}

struct BnoSample {
    std::array<float, 4> quaternion{}; // W, X, Y, Z; unchanged sensor coordinates.
    std::array<unsigned, 4> calibration{}; // SYS, GYR, ACC, MAG.
};

BnoSample DecodeSample(const std::array<u8, 8>& bytes, u8 calibration)
{
    BnoSample sample;
    for (std::size_t i = 0; i < sample.quaternion.size(); ++i) {
        const std::int32_t word = bytes[2 * i] | (static_cast<std::uint32_t>(bytes[2 * i + 1]) << 8);
        const std::int32_t signed_word = word >= 0x8000 ? word - 0x10000 : word;
        sample.quaternion[i] = static_cast<float>(signed_word) / 16384.0f;
        sample.calibration[i] = (calibration >> (6 - 2 * i)) & 0x03;
    }
    return sample;
}

class Bno055 {
public:
    Bno055() = default;
    Bno055(const Bno055&) = delete;
    Bno055& operator=(const Bno055&) = delete;

    ~Bno055()
    {
        RemoveDevice();
        if (bus_ != nullptr) {
            i2c_del_master_bus(bus_);
        }
    }

    bool BeginBus()
    {
        static_assert(CONFIG_BNO055_I2C_SDA_GPIO != CONFIG_BNO055_I2C_SCL_GPIO, "SDA and SCL must use different GPIOs");

        i2c_master_bus_config_t config{};
        config.i2c_port   = I2C_NUM_0;
        config.sda_io_num = static_cast<gpio_num_t>(CONFIG_BNO055_I2C_SDA_GPIO);
        config.scl_io_num = static_cast<gpio_num_t>(CONFIG_BNO055_I2C_SCL_GPIO);
        config.clk_source = I2C_CLK_SRC_DEFAULT;
        config.glitch_ignore_cnt = 7;
        config.flags.enable_internal_pullup = true;
        const esp_err_t result = i2c_new_master_bus(&config, &bus_);
        if (result != ESP_OK) {
            ESP_LOGE(Tag, "I2C bus init failed: %s", esp_err_to_name(result));
            return false;
        }
        ESP_LOGI(Tag, "I2C initialized: SDA=GPIO%d, SCL=GPIO%d, frequency=%d Hz, poll=%d ms",
                 CONFIG_BNO055_I2C_SDA_GPIO, CONFIG_BNO055_I2C_SCL_GPIO,
                 CONFIG_BNO055_I2C_FREQUENCY_HZ, CONFIG_BNO055_OUTPUT_INTERVAL_MS);
        return true;
    }

    bool Detect()
    {
        RemoveDevice();
        for (const u8 address : {0x28, 0x29}) {
            if (i2c_master_probe(bus_, address, I2cTimeoutMs) != ESP_OK) {
                continue;
            }
            i2c_device_config_t config{};
            config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
            config.device_address = address;
            config.scl_speed_hz = CONFIG_BNO055_I2C_FREQUENCY_HZ;
            esp_err_t result = i2c_master_bus_add_device(bus_, &config, &device_);
            if (result != ESP_OK) {
                ESP_LOGE(Tag, "Cannot add I2C device at 0x%02X: %s", address, esp_err_to_name(result));
                continue;
            }
            u8 chip_id = 0;
            result = Read(ChipIdRegister, &chip_id, 1);
            if (result == ESP_OK && chip_id == 0xA0) {
                ESP_LOGI(Tag, "BNO055 confirmed at 0x%02X, CHIP_ID=0x%02X", address, chip_id);
                return true;
            }
            ESP_LOGW(Tag, "Rejected device at 0x%02X: CHIP_ID=0x%02X, read=%s",
                     address, chip_id, esp_err_to_name(result));
            RemoveDevice();
        }
        ESP_LOGW(Tag, "BNO055 not found at 0x28 or 0x29; check 3V3, GND, SDA and SCL");
        return false;
    }

    bool Initialize()
    {
        ready_ = false;
        if (!Write(PageRegister, 0x00) || !Write(ModeRegister, 0x00)) {
            return false;
        }
        vTaskDelay(TicksForMs(30));
        if (!Write(PowerRegister, 0x00) || !Write(UnitsRegister, 0x00) || !Write(TriggerRegister, 0x00)) {
            return false;
        }
        vTaskDelay(TicksForMs(20));
        if (!Write(ModeRegister, NdofMode)) {
            return false;
        }
        vTaskDelay(TicksForMs(1000));
        u8 mode = 0, system_error = 0;
        const esp_err_t mode_result = Read(ModeRegister, &mode, 1);
        const esp_err_t error_result = Read(SystemErrorRegister, &system_error, 1);
        if (mode_result != ESP_OK || error_result != ESP_OK || (mode & 0x0F) != NdofMode || system_error != 0) {
            ESP_LOGE(Tag, "NDOF verification failed: mode=0x%02X, system_error=%u, reads=%s/%s",
                     mode, system_error, esp_err_to_name(mode_result), esp_err_to_name(error_result));
            return false;
        }
        ready_ = true;
        ESP_LOGI(Tag, "NDOF ready; polling every %d ms", CONFIG_BNO055_OUTPUT_INTERVAL_MS);
        return true;
    }

    esp_err_t ReadSample(BnoSample& sample)
    {
        if (!ready_) {
            return ESP_ERR_INVALID_STATE;
        }
        std::array<u8, 8> bytes{};
        u8 calibration = 0;
        esp_err_t result = Read(QuaternionRegister, bytes.data(), bytes.size());
        if (result == ESP_OK) {
            result = Read(CalibrationRegister, &calibration, 1);
        }
        if (result == ESP_OK) {
            sample = DecodeSample(bytes, calibration);
        }
        return result;
    }

    bool ReadAndLog()
    {
        BnoSample sample;
        const esp_err_t result = ReadSample(sample);
        if (result != ESP_OK) {
            ESP_LOGE(Tag, "BNO055 read failed: %s", esp_err_to_name(result));
            return false;
        }
        if (std::all_of(sample.quaternion.begin(), sample.quaternion.end(), [](float x) { return x == 0; })) {
            const std::int64_t now = esp_timer_get_time();
            if (now - last_wait_log_us_ >= 1000000) {
                ESP_LOGW(Tag, "Waiting for nonzero orientation");
                last_wait_log_us_ = now;
            }
            return false;
        }
        ESP_LOGI(Tag, "Quaternion W=%.4f X=%.4f Y=%.4f Z=%.4f | Cal SYS=%u GYR=%u ACC=%u MAG=%u",
                 sample.quaternion[0], sample.quaternion[1], sample.quaternion[2], sample.quaternion[3],
                 sample.calibration[0], sample.calibration[1], sample.calibration[2], sample.calibration[3]);
        return true;
    }

private:
    esp_err_t Read(u8 reg, u8* data, std::size_t size)
    {
        return i2c_master_transmit_receive(device_, &reg, 1, data, size, I2cTimeoutMs);
    }

    bool Write(u8 reg, u8 value)
    {
        const u8 bytes[] = {reg, value};
        const esp_err_t result = i2c_master_transmit(device_, bytes, sizeof(bytes), I2cTimeoutMs);
        if (result != ESP_OK) {
            ESP_LOGE(Tag, "BNO055 write 0x%02X failed: %s", reg, esp_err_to_name(result));
        }
        return result == ESP_OK;
    }

    void RemoveDevice()
    {
        ready_ = false;
        if (device_ != nullptr) {
            i2c_master_bus_rm_device(device_);
            device_ = nullptr;
        }
    }

    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t device_ = nullptr;
    bool ready_ = false;
    std::int64_t last_wait_log_us_ = -1000000;
};

struct TimingStats {
    std::uint32_t polls = 0, samples = 0, overruns = 0;
    std::int64_t total_us = 0, maximum_us = 0;
    std::int64_t first_poll_us = 0, last_poll_us = 0;
    std::int64_t report_since_us = esp_timer_get_time();

    void Record(std::int64_t start_us, std::int64_t end_us, bool valid)
    {
        if (polls == 0) {
            first_poll_us = start_us;
        }
        last_poll_us = start_us;
        ++polls;
        samples += valid ? 1 : 0;
        const auto duration = end_us - start_us;
        total_us += duration;
        maximum_us = std::max(maximum_us, duration);
    }

    void ReportIfDue()
    {
        constexpr std::int64_t interval_us = CONFIG_BNO055_TIMING_REPORT_INTERVAL_MS * 1000LL;
        if (interval_us == 0 || esp_timer_get_time() - report_since_us < interval_us || polls == 0) {
            return;
        }
        const double rate = polls > 1 && last_poll_us > first_poll_us
            ? (polls - 1) * 1000000.0 / (last_poll_us - first_poll_us) : 0;
        ESP_LOGI(Tag, "Timing: poll_hz=%.2f polls=%lu samples=%lu read_log_avg_us=%.1f read_log_max_us=%lld overruns=%lu",
                 rate, static_cast<unsigned long>(polls), static_cast<unsigned long>(samples),
                 static_cast<double>(total_us) / polls, static_cast<long long>(maximum_us),
                 static_cast<unsigned long>(overruns));
        *this = TimingStats{};
    }
};

void WaitForNextPoll(TickType_t& last_wake, TickType_t period, TimingStats& timing)
{
    const TickType_t now = xTaskGetTickCount();
    if (static_cast<TickType_t>(now - last_wake) >= period) {
        ++timing.overruns;
        last_wake = now; // No burst of catch-up reads after a slow log or I2C timeout.
    }
    xTaskDelayUntil(&last_wake, period);
}

} // namespace

extern "C" void app_main()
{
    ESP_LOGI(Tag, "Starting BNO055 on Seeed Studio XIAO ESP32-C3");
    vTaskDelay(TicksForMs(700));
    Bno055 sensor;
    if (!sensor.BeginBus()) {
        return;
    }

    size_t count = 0;
    while ((!sensor.Detect() || !sensor.Initialize())) {
        ESP_LOGW(Tag, "Retrying BNO055 initialization in 2 seconds");
        vTaskDelay(TicksForMs(2000));

        if (++count > MaxRetryCount)
            ESP_LOGI(Tag, "Failed after multiple retries");
    }



    TimingStats timing;
    TickType_t last_wake = xTaskGetTickCount();
    constexpr TickType_t period = TicksForMs(CONFIG_BNO055_OUTPUT_INTERVAL_MS);
    static_assert(period > 0, "Polling period must be at least one tick");

    while (true) {
        const auto start_us = esp_timer_get_time();
        const bool valid = sensor.ReadAndLog();
        timing.Record(start_us, esp_timer_get_time(), valid);
        timing.ReportIfDue();
        WaitForNextPoll(last_wake, period, timing);
    }
}
