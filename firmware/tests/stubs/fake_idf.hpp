#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using esp_err_t = int;
constexpr int ESP_OK = 0, ESP_FAIL = -1, ESP_ERR_INVALID_STATE = 0x103;
using TickType_t = std::uint32_t;
constexpr int configTICK_RATE_HZ = 1000;
using gpio_num_t = int;
constexpr int I2C_NUM_0 = 0, I2C_CLK_SRC_DEFAULT = 0, I2C_ADDR_BIT_LEN_7 = 0;
struct FakeBus {};
struct FakeDevice { int address; };
using i2c_master_bus_handle_t = FakeBus*;
using i2c_master_dev_handle_t = FakeDevice*;
struct i2c_master_bus_config_t {
    int i2c_port, sda_io_num, scl_io_num, clk_source, glitch_ignore_cnt;
    struct { bool enable_internal_pullup; } flags;
};
struct i2c_device_config_t {
    int dev_addr_length, device_address;
    std::uint32_t scl_speed_hz;
};

namespace fake {
struct Sensor {
    std::array<std::uint8_t, 256> registers{};
    int read_failure = -1;
    int write_failure = -1;
    int mode_override = -1;
};
struct Stop {};
inline std::map<int, Sensor> sensors;
inline std::vector<std::string> logs;
inline std::vector<std::pair<int, int>> writes;
inline std::vector<TickType_t> delays;
inline std::vector<std::int64_t> quaternion_read_times;
inline std::int64_t now_us = 0, log_cost_us = 0;
inline int active_devices = 0, active_buses = 0, removed_devices = 0;
inline int stop_after_polls = 0, wait_calls = 0;
inline bool stop_on_retry = false;

inline void Reset()
{
    assert(active_devices == 0 && active_buses == 0);
    sensors.clear(); logs.clear(); writes.clear(); delays.clear(); quaternion_read_times.clear();
    now_us = log_cost_us = 0;
    removed_devices = stop_after_polls = wait_calls = 0;
    stop_on_retry = false;
}

inline Sensor& AddSensor(int address = 0x28)
{
    auto& sensor = sensors[address];
    sensor.registers[0] = 0xA0;
    sensor.registers[0x21] = 0x40; // W = 1.0, other components zero.
    sensor.registers[0x35] = 0x1B; // SYS=0, GYR=1, ACC=2, MAG=3.
    return sensor;
}

inline bool HasLog(const std::string& needle)
{
    return std::any_of(logs.begin(), logs.end(), [&](const auto& line) { return line.find(needle) != std::string::npos; });
}

inline void Log(char level, const char* tag, const char* format, ...)
{
    char text[512];
    va_list args;
    va_start(args, format);
    std::vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    logs.push_back(std::string(1, level) + " (" + std::to_string(now_us / 1000) + ") " + tag + ": " + text);
    now_us += log_cost_us;
}
} // namespace fake

#define ESP_LOGI(tag, ...) fake::Log('I', tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) fake::Log('W', tag, __VA_ARGS__)
#define ESP_LOGE(tag, ...) fake::Log('E', tag, __VA_ARGS__)

inline const char* esp_err_to_name(esp_err_t result) { return result == ESP_OK ? "ESP_OK" : "ESP_FAIL"; }
inline std::int64_t esp_timer_get_time() { return fake::now_us; }
inline TickType_t xTaskGetTickCount() { return static_cast<TickType_t>(fake::now_us / 1000); }
inline void vTaskDelay(TickType_t ticks)
{
    fake::delays.push_back(ticks);
    fake::now_us += ticks * 1000LL;
    if (fake::stop_on_retry && ticks == 2000) { throw fake::Stop{}; }
}
inline int xTaskDelayUntil(TickType_t* last, TickType_t period)
{
    *last += period;
    const auto target_us = *last * 1000LL;
    const bool slept = target_us > fake::now_us;
    fake::now_us = std::max<std::int64_t>(fake::now_us, target_us);
    if (fake::stop_after_polls && ++fake::wait_calls == fake::stop_after_polls) { throw fake::Stop{}; }
    return slept;
}
inline esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t*, i2c_master_bus_handle_t* bus)
{
    *bus = new FakeBus;
    ++fake::active_buses;
    return ESP_OK;
}
inline esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t bus)
{
    delete bus;
    --fake::active_buses;
    return ESP_OK;
}
inline esp_err_t i2c_master_probe(i2c_master_bus_handle_t, std::uint16_t address, int timeout)
{
    assert(timeout > 0 && timeout <= 20);
    return fake::sensors.count(address) ? ESP_OK : ESP_FAIL;
}
inline esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t, const i2c_device_config_t* config, i2c_master_dev_handle_t* device)
{
    *device = new FakeDevice{config->device_address};
    ++fake::active_devices;
    return ESP_OK;
}
inline esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t device)
{
    delete device;
    --fake::active_devices;
    ++fake::removed_devices;
    return ESP_OK;
}
inline esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t device,
    const std::uint8_t* tx, std::size_t tx_size, std::uint8_t* rx, std::size_t rx_size, int timeout)
{
    assert(device && tx_size == 1 && timeout > 0 && timeout <= 20);
    auto& sensor = fake::sensors.at(device->address);
    if (*tx == 0x20) { fake::quaternion_read_times.push_back(fake::now_us); }
    fake::now_us += 100;
    if (sensor.read_failure == *tx) { return ESP_FAIL; }
    std::copy_n(sensor.registers.data() + *tx, rx_size, rx);
    if (*tx == 0x3D && sensor.mode_override >= 0) { rx[0] = sensor.mode_override; }
    return ESP_OK;
}
inline esp_err_t i2c_master_transmit(i2c_master_dev_handle_t device, const std::uint8_t* data, std::size_t size, int timeout)
{
    assert(device && size == 2 && timeout > 0 && timeout <= 20);
    auto& sensor = fake::sensors.at(device->address);
    fake::now_us += 100;
    fake::writes.emplace_back(data[0], data[1]);
    if (sensor.write_failure == data[0]) { return ESP_FAIL; }
    sensor.registers[data[0]] = data[1];
    return ESP_OK;
}
