#include "glove_mouse/config.hpp"
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <unistd.h>

namespace glove {
const char* PlaneName(Plane p)
{
    switch (p) {
    case Plane::XY: return "XY"; case Plane::XZ: return "XZ";
    case Plane::YZ: return "YZ"; case Plane::None: return "None";
    }
    throw std::runtime_error("Invalid plane");
}
Plane ParsePlane(const std::string& name)
{
    for (auto p : {Plane::XY, Plane::XZ, Plane::YZ, Plane::None}) if (name == PlaneName(p)) return p;
    throw std::runtime_error("Plane must be XY, XZ, YZ or None");
}
void Validate(const Config& c)
{
    if (c.serial_port.empty() || c.serial_port.size() > 4096 || c.serial_port.find('\0') != std::string::npos)
        throw std::runtime_error("Serial port must be a nonempty path (up to 4096 bytes)");
    for (auto a : {c.x, c.y}) {
        (void)PlaneName(a.plane);
        if (!std::isfinite(a.gain) || a.gain < .1 || a.gain > 200)
            throw std::runtime_error("Sensitivity must be between 0.1 and 200 units/degree");
    }
    if (!std::isfinite(c.deadzone_deg_s) || c.deadzone_deg_s < 0 || c.deadzone_deg_s > 20)
        throw std::runtime_error("Dead zone must be between 0 and 20 degrees/second");
}
Json ConfigJson(const Config& c)
{
    Validate(c);
    auto j = Object(); Put(j.get(), "version", Integer(1)); Put(j.get(), "serial_port", Text(c.serial_port));
    for (auto item : {std::pair{"x", c.x}, std::pair{"y", c.y}}) {
        auto a = Object(); Put(a.get(), "plane", Text(PlaneName(item.second.plane)));
        Put(a.get(), "invert", Bool(item.second.invert)); Put(a.get(), "gain", Real(item.second.gain));
        Put(j.get(), item.first, std::move(a));
    }
    Put(j.get(), "deadzone_deg_s", Real(c.deadzone_deg_s)); return j;
}
Config ParseConfig(json_object* j)
{
    if (json_object_get_int64(Required(j, "version", json_type_int)) != 1)
        throw std::runtime_error("Unsupported profile version");
    Config c; c.serial_port = String(j, "serial_port");
    auto axis = [&](const char* key) {
        auto* a = Required(j, key, json_type_object);
        return AxisConfig{ParsePlane(String(a, "plane")), Boolean(a, "invert"), Number(a, "gain")};
    };
    c.x = axis("x"); c.y = axis("y"); c.deadzone_deg_s = Number(j, "deadzone_deg_s");
    Validate(c); return c;
}
Config LoadConfig(const std::filesystem::path& path)
{
    std::error_code ec;
    if (!std::filesystem::exists(path, ec) && !ec) return {};
    if (ec) throw std::runtime_error("Cannot inspect profile: " + ec.message());
    std::ifstream file(path, std::ios::binary);
    if (!file) throw std::runtime_error("Cannot read profile: " + path.string());
    std::string text;
    char buffer[4096];
    while (file.read(buffer, sizeof(buffer)) || file.gcount()) {
        text.append(buffer, static_cast<std::size_t>(file.gcount()));
        if (text.size() > 65536) throw std::runtime_error("Profile exceeds 64 KiB");
    }
    if (file.bad()) throw std::runtime_error("Error reading profile");
    auto j = ParseJson(text); return ParseConfig(j.get());
}
void SaveConfig(const std::filesystem::path& path, const Config& c)
{
    const std::string data = DumpJson(ConfigJson(c).get()) + "\n";
    const auto parent = path.has_parent_path() ? path.parent_path() : std::filesystem::path(".");
    std::filesystem::create_directories(parent);
    std::string name = (parent / (path.filename().string() + ".tmp.XXXXXX")).string();
    const int fd = mkstemp(name.data());
    if (fd < 0) throw std::runtime_error("Cannot create profile temporary file: " + std::string(std::strerror(errno)));
    try {
        std::size_t offset = 0;
        while (offset < data.size()) {
            const ssize_t n = write(fd, data.data() + offset, data.size() - offset);
            if (n < 0 && errno == EINTR) continue;
            if (n <= 0) throw std::runtime_error("Cannot write profile");
            offset += static_cast<std::size_t>(n);
        }
        if (fsync(fd) != 0) throw std::runtime_error("Cannot sync profile");
        if (rename(name.c_str(), path.c_str()) != 0) throw std::runtime_error("Cannot replace profile: " + std::string(std::strerror(errno)));
    } catch (...) { close(fd); unlink(name.c_str()); throw; }
    close(fd);
}
std::filesystem::path DefaultConfigPath()
{
    if (const char* s = std::getenv("XDG_CONFIG_HOME"); s && *s) return std::filesystem::path(s) / "glove-mouse/config.json";
    if (const char* s = std::getenv("HOME"); s && *s) return std::filesystem::path(s) / ".config/glove-mouse/config.json";
    throw std::runtime_error("HOME and XDG_CONFIG_HOME are unset; use --config");
}
std::string DefaultSocketPath()
{
    if (const char* s = std::getenv("XDG_RUNTIME_DIR"); s && *s) return (std::filesystem::path(s) / "glove-mouse/driver.sock").string();
    return "/tmp/glove-mouse-" + std::to_string(getuid()) + "/driver.sock";
}
} // namespace glove
