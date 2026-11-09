#pragma once
#include "json.hpp"
#include <filesystem>

namespace glove {
enum class Plane { XY, XZ, YZ, None };
const char* PlaneName(Plane plane);
Plane ParsePlane(const std::string& name);
struct AxisConfig { Plane plane = Plane::XY; bool invert = true; double gain = 20; };
struct Config {
    std::string serial_port = "/dev/ttyACM0";
    AxisConfig x;
    AxisConfig y{Plane::YZ, true, 20};
    double deadzone_deg_s = 2;
};
void Validate(const Config& config);
Json ConfigJson(const Config& config);
Config ParseConfig(json_object* value);
Config LoadConfig(const std::filesystem::path& path);
void SaveConfig(const std::filesystem::path& path, const Config& config);
std::filesystem::path DefaultConfigPath();
std::string DefaultSocketPath();
} // namespace glove
