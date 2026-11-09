#include "glove_mouse/telemetry.hpp"
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <cctype>
#include <limits>

namespace glove {
std::vector<std::string> LineFramer::Feed(std::string_view bytes)
{
    std::vector<std::string> lines;
    for (char c : bytes) {
        if (c == '\n') {
            if (!dropping_) {
                if (!pending_.empty() && pending_.back() == '\r') pending_.pop_back();
                lines.push_back(std::move(pending_));
            }
            pending_.clear(); dropping_ = false;
        } else if (!dropping_) {
            if (pending_.size() == MaxBytes) { pending_.clear(); dropping_ = true; ++overflows_; }
            else pending_ += c;
        }
    }
    return lines;
}
std::string StripAnsi(std::string_view line)
{
    std::string clean;
    for (std::size_t i = 0; i < line.size(); ++i) {
        if (line[i] == '\x1b' && i + 1 < line.size() && line[i + 1] == '[') {
            i += 2;
            while (i < line.size() && !(line[i] >= '@' && line[i] <= '~')) ++i;
        } else clean += line[i];
    }
    return clean;
}
std::optional<Sample> ParseSample(std::string_view line)
{
    const auto text = StripAnsi(line);
    const auto pos = text.find("Quaternion W=");
    if (pos == std::string::npos) return {};
    const char* p = text.c_str() + pos;
    bool valid = true;
    auto literal = [&](std::string_view s) {
        if (!valid) return;
        if (std::string_view(p).substr(0, s.size()) != s) { valid = false; return; }
        p += s.size();
    };
    auto real = [&]() {
        if (!valid) return 0.0;
        char* end = nullptr;
        const double n = std::strtod(p, &end);
        if (p == end || !std::isfinite(n)) valid = false;
        p = end; return n;
    };
    auto calibration = [&]() -> unsigned {
        if (!valid) return 0;
        while (*p == ' ') ++p;
        const char* end = p; while (*end >= '0' && *end <= '9') ++end;
        unsigned n = 0;
        const auto result = std::from_chars(p, end, n);
        if (p == end || result.ec != std::errc{} || n > 3) valid = false;
        p = end; return n;
    };
    Sample s;
    literal("Quaternion W="); s.quaternion[0] = real();
    literal(" X="); s.quaternion[1] = real();
    literal(" Y="); s.quaternion[2] = real();
    literal(" Z="); s.quaternion[3] = real();
    literal(" | Cal SYS="); s.calibration[0] = calibration();
    literal(" GYR="); s.calibration[1] = calibration();
    literal(" ACC="); s.calibration[2] = calibration();
    literal(" MAG="); s.calibration[3] = calibration();
    while (*p && std::isspace(static_cast<unsigned char>(*p))) ++p;
    if (!valid || *p) return {};
    const double norm = std::hypot(std::hypot(s.quaternion[0], s.quaternion[1]), std::hypot(s.quaternion[2], s.quaternion[3]));
    if (!std::isfinite(norm) || norm < .5 || norm > 1.5) return {};
    s.quaternion = Normalize(s.quaternion);
    const auto open = text.find('('), close = text.find(')');
    if (open != std::string::npos && close != std::string::npos && open < close && close < pos) {
        std::uint64_t time = 0;
        const char* first = text.data() + open + 1;
        const auto result = std::from_chars(first, text.data() + close, time);
        if (result.ec == std::errc{} && result.ptr == text.data() + close) s.device_ms = time;
    }
    return s;
}
} // namespace glove
