#pragma once
#include "telemetry.hpp"
#include <memory>
#include <sys/types.h>

struct input_event; // Keep Linux KEY_* macros out of the raylib translation unit.

namespace glove {
class Fd {
public:
    explicit Fd(int fd = -1) : fd_(fd) {}
    ~Fd();
    Fd(const Fd&) = delete;
    Fd& operator=(const Fd&) = delete;
    Fd(Fd&& other) noexcept;
    Fd& operator=(Fd&& other) noexcept;
    int Get() const { return fd_; }
    explicit operator bool() const { return fd_ >= 0; }
    void Reset(int fd = -1);
private:
    int fd_;
};
double Monotonic();
std::string SystemError(const std::string& operation);
class SerialReader {
public:
    ~SerialReader() { Close(); }
    void Open(const std::string& path);
    void Close();
    bool IsOpen() const { return static_cast<bool>(fd_); }
    std::vector<std::string> Read();
private:
    Fd fd_;
    LineFramer lines_;
};
class EventDevice {
public:
    virtual ~EventDevice() = default;
    virtual void Open() = 0;
    virtual ssize_t Write(const input_event& event) = 0;
    virtual void Close() = 0;
};
std::unique_ptr<EventDevice> LinuxMouse();
class MouseOutput {
public:
    explicit MouseOutput(bool dry_run, std::unique_ptr<EventDevice> device = LinuxMouse());
    ~MouseOutput();
    bool Prepare();
    bool Available() const { return available_; }
    const std::string& Error() const { return error_; }
    std::array<int, 2> MoveBy(Move move);
    void ResetRemainders() { remainder_ = {}; }
private:
    bool dry_, available_;
    std::unique_ptr<EventDevice> device_;
    std::string error_;
    Move remainder_{};
};
} // namespace glove
