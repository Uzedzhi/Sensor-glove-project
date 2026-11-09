#pragma once
#include "config.hpp"
#include <array>
#include <optional>

namespace glove {
using Quat = std::array<double, 4>; // W, X, Y, Z.
using Move = std::array<double, 2>;
Quat Normalize(Quat q);
Quat Conjugate(Quat q);
Quat Multiply(const Quat& a, const Quat& b);
class Mapper {
public:
    void ResetPrevious() { previous_.reset(); }
    void ResetReference() { reference_.reset(); ResetPrevious(); }
    void Center(Quat q) { reference_ = Normalize(q); ResetPrevious(); }
    Move Step(Quat q, double dt, const Config& config);
    std::array<double, 3> Angles(Quat q) const; // yaw, pitch, roll; ZYX, degrees.
private:
    std::optional<Quat> reference_, previous_;
};
} // namespace glove
