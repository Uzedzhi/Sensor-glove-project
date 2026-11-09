#include "glove_mouse/io.hpp"
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/uinput.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace glove {
Fd::~Fd() { Reset(); }
Fd::Fd(Fd&& other) noexcept : fd_(std::exchange(other.fd_, -1)) {}
Fd& Fd::operator=(Fd&& other) noexcept { if (this != &other) Reset(std::exchange(other.fd_, -1)); return *this; }
void Fd::Reset(int fd) { if (fd_ >= 0) close(fd_); fd_ = fd; }
double Monotonic() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
std::string SystemError(const std::string& operation) { return operation + ": " + std::strerror(errno); }
void SerialReader::Open(const std::string& path)
{
    Close(); fd_.Reset(open(path.c_str(), O_RDONLY | O_NONBLOCK | O_NOCTTY | O_CLOEXEC));
    if (!fd_) throw std::runtime_error(SystemError("Open serial " + path));
    if (ioctl(fd_.Get(), TIOCEXCL) != 0) { const auto error = SystemError("Exclusive serial access"); Close(); throw std::runtime_error(error); }
    termios tty{};
    if (tcgetattr(fd_.Get(), &tty) != 0) { const auto error = SystemError("Read termios"); Close(); throw std::runtime_error(error); }
    cfmakeraw(&tty); cfsetispeed(&tty, B115200); cfsetospeed(&tty, B115200);
    tty.c_cflag |= CLOCAL | CREAD; tty.c_cc[VMIN] = 1; tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd_.Get(), TCSANOW, &tty) != 0) { const auto error = SystemError("Set termios"); Close(); throw std::runtime_error(error); }
    tcflush(fd_.Get(), TCIFLUSH);
}
void SerialReader::Close()
{
    if (fd_) ioctl(fd_.Get(), TIOCNXCL);
    fd_.Reset(); lines_.Clear();
}
std::vector<std::string> SerialReader::Read()
{
    if (!fd_) return {};
    pollfd p{fd_.Get(), POLLIN, 0};
    if (poll(&p, 1, 0) > 0 && (p.revents & (POLLHUP | POLLERR | POLLNVAL)))
        throw std::runtime_error("Serial device disconnected");
    std::vector<std::string> result;
    char bytes[4096];
    // Bounded work: a noisy tty cannot starve IPC and the freshness watchdog.
    for (unsigned batch = 0; batch < 16; ++batch) {
        const ssize_t n = read(fd_.Get(), bytes, sizeof(bytes));
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
        if (n <= 0) throw std::runtime_error(n == 0 ? "Serial device disconnected" : SystemError("Read serial"));
        auto lines = lines_.Feed(std::string_view(bytes, static_cast<std::size_t>(n)));
        for (auto& line : lines) result.push_back(std::move(line));
    }
    return result;
}
namespace {
class UinputDevice final : public EventDevice {
public:
    ~UinputDevice() override { Close(); }
    void Open() override
    {
        Close(); fd_.Reset(open("/dev/uinput", O_WRONLY | O_NONBLOCK | O_CLOEXEC));
        if (!fd_) throw std::runtime_error(SystemError("Open /dev/uinput (write permission required)"));
        auto check = [&](int result) { if (result < 0) throw std::runtime_error(SystemError("Configure uinput")); };
        try {
            check(ioctl(fd_.Get(), UI_SET_EVBIT, EV_KEY));
            check(ioctl(fd_.Get(), UI_SET_KEYBIT, BTN_LEFT));
            check(ioctl(fd_.Get(), UI_SET_EVBIT, EV_REL));
            check(ioctl(fd_.Get(), UI_SET_RELBIT, REL_X));
            check(ioctl(fd_.Get(), UI_SET_RELBIT, REL_Y));
            uinput_setup setup{}; setup.id.bustype = BUS_VIRTUAL;
            std::strncpy(setup.name, "Sensor glove relative mouse", UINPUT_MAX_NAME_SIZE - 1);
            check(ioctl(fd_.Get(), UI_DEV_SETUP, &setup)); check(ioctl(fd_.Get(), UI_DEV_CREATE));
        } catch (...) { Close(); throw; }
    }
    ssize_t Write(const input_event& event) override { return write(fd_.Get(), &event, sizeof(event)); }
    void Close() override { if (fd_) ioctl(fd_.Get(), UI_DEV_DESTROY); fd_.Reset(); }
private:
    Fd fd_;
};
}
std::unique_ptr<EventDevice> LinuxMouse() { return std::make_unique<UinputDevice>(); }
MouseOutput::MouseOutput(bool dry_run, std::unique_ptr<EventDevice> device)
    : dry_(dry_run), available_(dry_run), device_(std::move(device)) {}
MouseOutput::~MouseOutput() { if (device_) device_->Close(); }
bool MouseOutput::Prepare()
{
    if (available_) return true;
    try { device_->Open(); available_ = true; error_.clear(); }
    catch (const std::exception& e) { device_->Close(); error_ = e.what(); available_ = false; }
    return available_;
}
std::array<int, 2> MouseOutput::MoveBy(Move move)
{
    if (!available_) throw std::runtime_error("Mouse output unavailable");
    std::array<int, 2> steps{};
    for (unsigned axis = 0; axis < 2; ++axis) {
        if (!std::isfinite(move[axis]) || std::abs(move[axis]) > 1000000) throw std::runtime_error("Invalid relative movement");
        remainder_[axis] += move[axis];
        steps[axis] = static_cast<int>(std::trunc(remainder_[axis])); remainder_[axis] -= steps[axis];
    }
    if (dry_ || (steps[0] == 0 && steps[1] == 0)) return steps;
    try {
        auto emit = [&](unsigned short type, unsigned short code, int value) {
            input_event event{}; event.type = type; event.code = code; event.value = value;
            ssize_t n = -1;
            for (int attempt = 0; attempt < 3; ++attempt) {
                n = device_->Write(event); if (n >= 0 || errno != EINTR) break;
            }
            if (n != static_cast<ssize_t>(sizeof(event)))
                throw std::runtime_error(n < 0 ? SystemError("Write uinput event") : "Partial uinput event write");
        };
        if (steps[0]) emit(EV_REL, REL_X, steps[0]);
        if (steps[1]) emit(EV_REL, REL_Y, steps[1]);
        emit(EV_SYN, SYN_REPORT, 0);
    } catch (const std::exception& e) {
        error_ = e.what(); available_ = false; ResetRemainders(); device_->Close(); throw;
    }
    return steps;
}
} // namespace glove
