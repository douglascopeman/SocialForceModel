#pragma once

#include <cstdint>

namespace sfm {

struct Vec2 {
    double x{};
    double y{};
};

struct AgentState {
    std::uint64_t id{};
    Vec2 position{};
    Vec2 velocity{};
    Vec2 desired_direction{1.0, 0.0};
};

}  // namespace sfm
