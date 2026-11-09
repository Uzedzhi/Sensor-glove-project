#pragma once
#include "io.hpp"
#include <functional>

namespace glove {
class Runtime {
public:
    using Saver = std::function<void(const Config&)>;
    Runtime(Config config, bool dry_run, Saver saver, std::unique_ptr<EventDevice> device = LinuxMouse());
    const Config& Settings() const { return config_; }
    void Connected(bool connected, std::string reason = {});
    bool IsConnected() const { return connected_; }
    void Receive(const Sample& sample, double now);
    void Tick(double now);
    bool Ready(double now) const;
    void Enable(bool enabled, double now);
    void Center(double now);
    void Apply(const Config& config);
    Json Status(double now) const;
    Json Request(json_object* request, double now);
    void ProfileError(std::string reason) { profile_error_ = std::move(reason); }
    void Rejected() { ++rejected_; }
    bool StopRequested() const { return stop_; }
private:
    void LoseData(const std::string& reason);
    Config config_;
    bool dry_, connected_ = false, enabled_ = false, has_reference_ = false, stale_ = false, stop_ = false;
    Saver saver_;
    Mapper mapper_;
    MouseOutput output_;
    std::optional<Sample> last_;
    std::optional<std::uint64_t> previous_device_ms_;
    double last_host_ = -1, warmup_ = -1;
    std::uint64_t samples_ = 0, rejected_ = 0, epoch_ = 0;
    std::array<int,2> last_move_{};
    std::array<std::int64_t,2> total_{};
    std::string session_, profile_error_, serial_error_, motion_error_;
};
} // namespace glove
