// Exercise the actual firmware with deterministic I2C, clock and console substitutes.
#include "../main/main.cpp"

#include <iostream>

#ifdef HAVE_BNO055_VIEWER
#include "BnoSample.h"
#endif

#define CHECK(condition) do { if (!(condition)) { throw std::runtime_error(#condition); } } while (false)

void TestDetection()
{
    for (const int address : {0x28, 0x29}) {
        fake::Reset();
        fake::AddSensor(address);
        Bno055 sensor;
        CHECK(sensor.BeginBus());
        CHECK(sensor.Detect());
        CHECK(fake::active_devices == 1);
    }
    fake::Reset();
    {
        Bno055 sensor;
        CHECK(sensor.BeginBus());
        CHECK(!sensor.Detect());
        CHECK(!sensor.ReadAndLog());
        CHECK(!fake::HasLog("Quaternion W="));
        CHECK(fake::HasLog("BNO055 not found"));
    }
    fake::Reset();
    fake::AddSensor(0x28).registers[0] = 0xFF;
    fake::AddSensor(0x29);
    {
        Bno055 sensor;
        CHECK(sensor.BeginBus());
        CHECK(sensor.Detect());
        CHECK(fake::HasLog("Rejected device at 0x28"));
        CHECK(fake::HasLog("BNO055 confirmed at 0x29"));
        CHECK(fake::active_devices == 1);
        CHECK(fake::removed_devices == 1);
    }
    CHECK(fake::active_devices == 0 && fake::active_buses == 0);
}

void TestInitialization()
{
    fake::Reset();
    fake::AddSensor();
    {
        Bno055 sensor;
        CHECK(sensor.BeginBus() && sensor.Detect() && sensor.Initialize());
        const std::vector<std::pair<int, int>> expected{{7, 0}, {0x3D, 0}, {0x3E, 0}, {0x3B, 0}, {0x3F, 0}, {0x3D, 0x0C}};
        CHECK(fake::writes == expected);
        CHECK((fake::delays == std::vector<TickType_t>{30, 20, 1000}));
        CHECK(fake::HasLog("NDOF ready"));
    }
    // Every configuration register can fail; none may lead to sample output.
    for (const int reg : {7, 0x3D, 0x3E, 0x3B, 0x3F}) {
        fake::Reset();
        fake::AddSensor().write_failure = reg;
        Bno055 sensor;
        CHECK(sensor.BeginBus() && sensor.Detect());
        CHECK(!sensor.Initialize());
        CHECK(!sensor.ReadAndLog());
        CHECK(!fake::HasLog("NDOF ready"));
        CHECK(!fake::HasLog("Quaternion W="));
    }
    for (const int failure : {0, 1, 2, 3}) {
        fake::Reset();
        auto& hardware = fake::AddSensor();
        if (failure == 0) { hardware.mode_override = 0; }
        if (failure == 1) { hardware.registers[0x3A] = 1; }
        if (failure == 2) { hardware.read_failure = 0x3D; }
        if (failure == 3) { hardware.read_failure = 0x3A; }
        Bno055 sensor;
        CHECK(sensor.BeginBus() && sensor.Detect());
        CHECK(!sensor.Initialize());
        CHECK(!sensor.ReadAndLog());
        CHECK(fake::HasLog("NDOF verification failed"));
        CHECK(!fake::HasLog("Quaternion W="));
    }
}

void TestDecodeAndReadFailures()
{
    const std::array<std::uint8_t, 8> bytes{0, 0x40, 0, 0xC0, 0, 0x20, 0, 0xE0};
    const auto sample = DecodeSample(bytes, 0x1B);
    CHECK((sample.quaternion == std::array<float, 4>{1, -1, 0.5f, -0.5f}));
    CHECK((sample.calibration == std::array<unsigned, 4>{0, 1, 2, 3}));
    const auto extremes = DecodeSample({0, 0x80, 0xFF, 0x7F, 0xFF, 0xFF, 1, 0}, 0xE4);
    CHECK((extremes.quaternion == std::array<float, 4>{-2, 32767.0f / 16384, -1.0f / 16384, 1.0f / 16384}));
    CHECK((extremes.calibration == std::array<unsigned, 4>{3, 2, 1, 0}));
    for (const int reg : {0x20, 0x35}) {
        fake::Reset();
        auto& hardware = fake::AddSensor();
        Bno055 sensor;
        CHECK(sensor.BeginBus() && sensor.Detect() && sensor.Initialize());
        BnoSample destination = sample;
        hardware.read_failure = reg;
        CHECK(sensor.ReadSample(destination) != ESP_OK);
        CHECK(destination.quaternion == sample.quaternion);
        fake::logs.clear();
        CHECK(!sensor.ReadAndLog());
        CHECK(fake::HasLog("BNO055 read failed"));
        CHECK(!fake::HasLog("Quaternion W="));
        hardware.read_failure = -1;
        CHECK(sensor.ReadAndLog());
        CHECK(fake::HasLog("Quaternion W=1.0000"));
    }
}

void TestZeroAndLowCalibration()
{
    fake::Reset();
    auto& hardware = fake::AddSensor();
    Bno055 sensor;
    CHECK(sensor.BeginBus() && sensor.Detect() && sensor.Initialize());
    hardware.registers[0x35] = 0;
    CHECK(sensor.ReadAndLog());
    CHECK(fake::HasLog("Cal SYS=0 GYR=0 ACC=0 MAG=0"));
    hardware.registers[0x21] = 0;
    fake::logs.clear();
    CHECK(!sensor.ReadAndLog());
    CHECK(fake::HasLog("Waiting for nonzero orientation"));
    CHECK(!fake::HasLog("Quaternion W="));
    fake::logs.clear();
    CHECK(!sensor.ReadAndLog());
    CHECK(fake::logs.empty()); // Waiting diagnostic is rate-limited.
}

void TestTimingAndSchedule()
{
    fake::Reset();
    CHECK(TicksForMs(10) == 10 && TicksForMs(1) == 1);
    TimingStats stats;
    stats.Record(0, 2000, true);
    stats.Record(10000, 14000, false);
    CHECK(stats.polls == 2 && stats.samples == 1 && stats.total_us == 6000 && stats.maximum_us == 4000);
    fake::now_us = 5000000;
    stats.ReportIfDue();
    CHECK(fake::HasLog("poll_hz=100.00"));
    CHECK(fake::HasLog("read_log_avg_us=3000.0 read_log_max_us=4000"));
    CHECK(stats.polls == 0);
    TickType_t last = xTaskGetTickCount();
    fake::now_us += 25000;
    WaitForNextPoll(last, 10, stats);
    CHECK(stats.overruns == 1);
    CHECK(fake::now_us == 5035000); // Full pause after the slow operation, no catch-up burst.
}

void TestApplicationLoop()
{
    fake::Reset();
    fake::AddSensor();
    fake::log_cost_us = 100;
    fake::stop_after_polls = 6;
    try { app_main(); CHECK(false); } catch (const fake::Stop&) {}
    CHECK(fake::quaternion_read_times.size() == 6);
    for (std::size_t i = 1; i < fake::quaternion_read_times.size(); ++i) {
        // The first wake is anchored to the tick boundary; subsequent reads are exactly 10 ms apart.
        if (i > 1) { CHECK(fake::quaternion_read_times[i] - fake::quaternion_read_times[i - 1] == 10000); }
    }
    CHECK(fake::delays.front() == 700);
    CHECK(fake::active_devices == 0 && fake::active_buses == 0);
    fake::Reset();
    fake::stop_on_retry = true;
    try { app_main(); CHECK(false); } catch (const fake::Stop&) {}
    CHECK(fake::HasLog("Retrying BNO055 initialization in 2 seconds"));
    CHECK(!fake::HasLog("Quaternion W="));
    CHECK(fake::quaternion_read_times.empty());
}

#ifdef HAVE_BNO055_VIEWER
void TestViewerCompatibility()
{
    fake::Reset();
    auto& hardware = fake::AddSensor();
    // Unit quaternion with both positive and negative components.
    const std::array<std::uint8_t, 8> bytes{0, 0x20, 0, 0xE0, 0, 0x20, 0, 0xE0};
    std::copy(bytes.begin(), bytes.end(), hardware.registers.begin() + 0x20);
    Bno055 sensor;
    CHECK(sensor.BeginBus() && sensor.Detect() && sensor.Initialize());
    CHECK(sensor.ReadAndLog());
    TBnoLineParser parser;
    int count = 0;
    std::string body;
    for (const auto& line : fake::logs) {
        const auto kind = parser.Feed(line);
        if (line.find("Quaternion W=") == std::string::npos) {
            CHECK(kind == ELineKind::Other);
        } else {
            CHECK(kind == ELineKind::Sample);
            body = line.substr(line.find("Quaternion W="));
            ++count;
        }
    }
    CHECK(count == 1);
    CHECK((parser.Sample().Quaternion == std::array<float, 4>{0.5f, -0.5f, 0.5f, -0.5f}));
    CHECK((parser.Sample().Calibration == std::array<unsigned, 4>{0, 1, 2, 3}));
    CHECK(parser.Sample().DeviceTimeMs.has_value());
    CHECK(parser.Feed(body) == ELineKind::Sample);
    CHECK(parser.Feed("I (5000) bno055: Timing: poll_hz=100.00 polls=501") == ELineKind::Other);
    std::cout << "PASS: actual firmware log parsed by unchanged bno055_viz TBnoLineParser\n";
}
#endif

int main()
{
    try {
        TestDetection();
        TestInitialization();
        TestDecodeAndReadFailures();
        TestZeroAndLowCalibration();
        TestTimingAndSchedule();
        TestApplicationLoop();
#ifdef HAVE_BNO055_VIEWER
        TestViewerCompatibility();
#endif
        std::cout << "PASS: detection, initialization, decoding, read failures, calibration, timing, application loop\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
