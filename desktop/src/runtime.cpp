#include "glove_mouse/runtime.hpp"
#include <algorithm>
#include <unistd.h>

namespace glove {
namespace {
template<class T, std::size_t N> Json Array(const std::array<T,N>& values)
{
    Json a(json_object_new_array());
    for (auto v : values) json_object_array_add(a.get(), Real(static_cast<double>(v)).release());
    return a;
}
}
Runtime::Runtime(Config config, bool dry_run, Saver saver, std::unique_ptr<EventDevice> device)
    : config_(std::move(config)), dry_(dry_run), saver_(std::move(saver)), output_(dry_run,std::move(device)),
      session_(std::to_string(getpid()) + ":" + std::to_string(Monotonic()))
{
    Validate(config_);
    output_.Prepare();
}
void Runtime::LoseData(const std::string& reason)
{
    enabled_ = false; stale_ = true; has_reference_ = false; warmup_ = -1;
    mapper_.ResetReference(); output_.ResetRemainders(); previous_device_ms_.reset();
    last_move_ = {}; motion_error_ = reason; ++epoch_;
}
void Runtime::Connected(bool connected, std::string reason)
{
    connected_ = connected; serial_error_ = std::move(reason);
    if (!connected) LoseData("Serial disconnected; control paused");
}
bool Runtime::Ready(double now) const
{
    return connected_ && last_host_ >= 0 && now - last_host_ <= .25 && warmup_ >= 0 && now - warmup_ >= 1.0;
}
void Runtime::Tick(double now)
{
    if (!stale_ && last_host_ >= 0 && now - last_host_ > .25)
        LoseData("No valid telemetry for 250 ms; control paused");
}
void Runtime::Receive(const Sample& sample, double now)
{
    Tick(now);
    double dt = last_host_ >= 0 && now > last_host_ ? now - last_host_ : .01;
    if (previous_device_ms_ && sample.device_ms) {
        if (*sample.device_ms < *previous_device_ms_) LoseData("ESP clock restarted; control paused");
        else if (*sample.device_ms > *previous_device_ms_)
            dt = static_cast<double>(*sample.device_ms - *previous_device_ms_) / 1000;
    }
    if (!has_reference_) {
        mapper_.Center(sample.quaternion); has_reference_ = true; warmup_ = now; ++epoch_;
    }
    stale_ = false; last_host_ = now; last_ = sample; previous_device_ms_ = sample.device_ms; ++samples_;
    last_move_ = {};
    if (enabled_) {
        try {
            last_move_ = output_.MoveBy(mapper_.Step(sample.quaternion, std::clamp(dt,.001,.25),config_));
            for (unsigned i=0;i<2;++i) total_[i] += last_move_[i];
        } catch (const std::exception& e) {
            enabled_ = false; mapper_.ResetPrevious(); output_.ResetRemainders(); motion_error_ = e.what();
        }
    }
}
void Runtime::Enable(bool enabled, double now)
{
    Tick(now);
    if (enabled) {
        if (!Ready(now)) throw std::runtime_error("Fresh telemetry and 1 second warmup required");
        if (!output_.Prepare()) throw std::runtime_error(output_.Error());
        motion_error_.clear();
    }
    enabled_ = enabled; mapper_.ResetPrevious(); output_.ResetRemainders(); last_move_ = {};
}
void Runtime::Center(double now)
{
    Tick(now);
    if (!connected_ || !last_ || last_host_ < 0 || now-last_host_ > .25)
        throw std::runtime_error("A fresh orientation is required for centering");
    mapper_.Center(last_->quaternion); has_reference_ = true;
    output_.ResetRemainders(); last_move_ = {}; ++epoch_;
}
void Runtime::Apply(const Config& config)
{
    Validate(config); saver_(config); // No runtime state changes before a successful save.
    const bool port_changed = config.serial_port != config_.serial_port;
    config_ = config; mapper_.ResetPrevious(); output_.ResetRemainders(); last_move_ = {}; profile_error_.clear();
    if (port_changed) Connected(false,"Serial port changed; reconnecting");
}
Json Runtime::Status(double now) const
{
    auto j = Object();
    Put(j.get(),"pid",Integer(getpid())); Put(j.get(),"session_id",Text(session_));
    Put(j.get(),"serial_port",Text(config_.serial_port)); Put(j.get(),"serial_connected",Bool(connected_));
    const std::string state = !connected_ ? "disconnected" : !last_ ? "waiting" : stale_ ? "stale" : Ready(now) ? "ready" : "warming";
    Put(j.get(),"telemetry_state",Text(state)); Put(j.get(),"ready",Bool(Ready(now)));
    Put(j.get(),"enabled",Bool(enabled_)); Put(j.get(),"dry_run",Bool(dry_));
    Put(j.get(),"output_available",Bool(output_.Available()));
    const auto error = !profile_error_.empty() ? profile_error_ : !serial_error_.empty() ? serial_error_ : !output_.Error().empty() ? output_.Error() : motion_error_;
    Put(j.get(),"last_error",Text(error)); Put(j.get(),"config",ConfigJson(config_));
    Put(j.get(),"sample_age_ms",last_host_ < 0 ? Json{} : Real(std::max(0.,(now-last_host_)*1000)));
    Put(j.get(),"last_valid_monotonic",last_host_ < 0 ? Json{} : Real(last_host_));
    Put(j.get(),"samples",Integer(static_cast<std::int64_t>(samples_)));
    Put(j.get(),"rejected",Integer(static_cast<std::int64_t>(rejected_)));
    Put(j.get(),"reference_epoch",Integer(static_cast<std::int64_t>(epoch_)));
    Put(j.get(),"last_dx",Integer(last_move_[0])); Put(j.get(),"last_dy",Integer(last_move_[1]));
    Put(j.get(),"total_dx",Integer(total_[0])); Put(j.get(),"total_dy",Integer(total_[1]));
    if (last_) {
        auto s = Object(); Put(s.get(),"quaternion",Array(last_->quaternion)); Put(s.get(),"calibration",Array(last_->calibration));
        Put(s.get(),"angles_deg",has_reference_ ? Array(mapper_.Angles(last_->quaternion)) : Json{});
        Put(s.get(),"device_ms",last_->device_ms ? Integer(static_cast<std::int64_t>(*last_->device_ms)) : Json{});
        Put(j.get(),"sample",std::move(s));
    } else Put(j.get(),"sample",Json{});
    return j;
}
Json Runtime::Request(json_object* request, double now)
{
    auto reply = Object(); Put(reply.get(),"version",Integer(1));
    json_object* id = nullptr;
    if (request && json_object_is_type(request,json_type_object)) json_object_object_get_ex(request,"id",&id);
    Put(reply.get(),"id",id && json_object_is_type(id,json_type_int) ? Integer(json_object_get_int64(id)) : Json{});
    try {
        if (json_object_get_int64(Required(request,"version",json_type_int)) != 1) throw std::runtime_error("Unsupported protocol version");
        if (json_object_get_int64(Required(request,"id",json_type_int)) < 0) throw std::runtime_error("id must be nonnegative");
        const std::string command = String(request,"command");
        Json result;
        if (command=="get_status") result=Status(now);
        else if (command=="get_config") result=ConfigJson(config_);
        else if (command=="set_config") { Apply(ParseConfig(Required(request,"config",json_type_object))); result=Status(now); }
        else if (command=="set_enabled") { Enable(Boolean(request,"enabled"),now); result=Status(now); }
        else if (command=="recenter") { Center(now); result=Status(now); }
        else if (command=="shutdown") { Enable(false,now); stop_=true; result=Status(now); }
        else throw std::runtime_error("Unknown command: " + command);
        Put(reply.get(),"ok",Bool(true)); Put(reply.get(),"result",std::move(result));
    } catch (const std::exception& e) {
        Put(reply.get(),"ok",Bool(false)); auto error=Object();
        Put(error.get(),"code",Text("request_failed")); Put(error.get(),"message",Text(e.what())); Put(reply.get(),"error",std::move(error));
    }
    return reply;
}
} // namespace glove
