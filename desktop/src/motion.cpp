#include "glove_mouse/motion.hpp"
#include <algorithm>
#include <cmath>
#include <utility>

namespace glove {
constexpr double Degrees = 180.0 / 3.14159265358979323846;
Quat Normalize(Quat q)
{
    const double length = std::hypot(std::hypot(q[0], q[1]), std::hypot(q[2], q[3]));
    if (!std::isfinite(length) || length < 1e-12) throw std::runtime_error("Invalid quaternion");
    for (auto& x : q) x /= length;
    return q;
}
Quat Conjugate(Quat q) { return {q[0], -q[1], -q[2], -q[3]}; }
Quat Multiply(const Quat& a, const Quat& b)
{
    return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
            a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
            a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
            a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
}
Move Mapper::Step(Quat q, double dt, const Config& config)
{
    q = Normalize(q);
    if (!reference_) reference_ = q;
    const auto previous = std::exchange(previous_, q);
    if (!previous) return {};
    const auto r0 = Multiply(Conjugate(*reference_), *previous);
    const auto r1 = Multiply(Conjugate(*reference_), q);
    auto delta = Normalize(Multiply(r1, Conjugate(r0)));
    if (delta[0] < 0) for (auto& x : delta) x = -x;
    const double sin_half = std::hypot(delta[1], delta[2], delta[3]);
    const double angle = 2 * std::atan2(sin_half, delta[0]) * Degrees;
    if (angle > 30.0 + 1e-9) return {}; // Previous already replaced: no delayed compensation.
    const double scale = sin_half > 1e-12 ? angle / sin_half : 2 * Degrees;
    const std::array<double, 3> turn{delta[1]*scale, delta[2]*scale, delta[3]*scale};
    auto axis = [&](const AxisConfig& c) {
        if (c.plane == Plane::None) return 0.0;
        const auto index = c.plane == Plane::XY ? 2 : c.plane == Plane::XZ ? 1 : 0;
        const double a = turn[index];
        const double magnitude = std::max(std::abs(a) - config.deadzone_deg_s * std::clamp(dt, .001, .25), 0.0);
        return std::copysign(magnitude, a) * c.gain * (c.invert ? -1 : 1);
    };
    return {axis(config.x), axis(config.y)};
}
std::array<double, 3> Mapper::Angles(Quat q) const
{
    q = Normalize(q);
    if (reference_) q = Multiply(Conjugate(*reference_), q);
    const auto [w,x,y,z] = q;
    return {std::atan2(2*(w*z+x*y), 1-2*(y*y+z*z))*Degrees,
            std::asin(std::clamp(2*(w*y-z*x), -1.0, 1.0))*Degrees,
            std::atan2(2*(w*x+y*z), 1-2*(x*x+y*y))*Degrees};
}
} // namespace glove
