#pragma once

#include <cmath>
#include <cstdint>

namespace sfm {

constexpr double kEpsilon = 1.0e-12;

struct Vec2 {
    double x{};
    double y{};

    constexpr Vec2& operator+=(const Vec2& other) noexcept {
        x += other.x;
        y += other.y;
        return *this;
    }
    constexpr Vec2& operator-=(const Vec2& other) noexcept {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    constexpr Vec2& operator*=(double scalar) noexcept {
        x *= scalar;
        y *= scalar;
        return *this;
    }
};

constexpr Vec2 operator+(Vec2 lhs, const Vec2& rhs) noexcept { return lhs += rhs; }
constexpr Vec2 operator-(Vec2 lhs, const Vec2& rhs) noexcept { return lhs -= rhs; }
constexpr Vec2 operator-(Vec2 value) noexcept { return {-value.x, -value.y}; }
constexpr Vec2 operator*(Vec2 value, double scalar) noexcept { return value *= scalar; }
constexpr Vec2 operator*(double scalar, Vec2 value) noexcept { return value *= scalar; }
constexpr double dot(Vec2 lhs, Vec2 rhs) noexcept { return lhs.x * rhs.x + lhs.y * rhs.y; }
constexpr double squared_norm(Vec2 value) noexcept { return dot(value, value); }
inline double norm(Vec2 value) noexcept { return std::sqrt(squared_norm(value)); }
inline Vec2 normalized(Vec2 value) noexcept {
    const double length = norm(value);
    return length > kEpsilon ? value * (1.0 / length) : Vec2{};
}

enum class DirectionMode { goal_point, fixed_vector };

struct AgentState {
    std::uint64_t id{};
    Vec2 position{};
    Vec2 velocity{};
    Vec2 preferred_velocity{};
    Vec2 goal{1.0, 0.0};
    double desired_speed_m_per_s{1.34};
    double relaxation_time_s{0.5};
    double maximum_speed_factor{1.3};
    DirectionMode direction_mode{DirectionMode::goal_point};
    Vec2 fixed_direction{1.0, 0.0};
};

inline Vec2 desired_direction(const AgentState& state) noexcept {
    return normalized(state.direction_mode == DirectionMode::fixed_vector
                          ? state.fixed_direction
                          : state.goal - state.position);
}

}  // namespace sfm
