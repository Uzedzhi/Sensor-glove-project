#pragma once
#include "motion.hpp"
#include <cstdint>
#include <string_view>
#include <vector>

namespace glove {
class LineFramer {
public:
    static constexpr std::size_t MaxBytes = 65536;
    std::vector<std::string> Feed(std::string_view bytes);
    void Clear() { pending_.clear(); dropping_ = false; }
    std::uint64_t Overflows() const { return overflows_; }
private:
    std::string pending_;
    bool dropping_ = false;
    std::uint64_t overflows_ = 0;
};
struct Sample {
    Quat quaternion{};
    std::array<unsigned, 4> calibration{};
    std::optional<std::uint64_t> device_ms;
};
std::string StripAnsi(std::string_view line);
std::optional<Sample> ParseSample(std::string_view line);
} // namespace glove
