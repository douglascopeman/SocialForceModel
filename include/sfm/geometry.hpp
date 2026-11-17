#pragma once

#include <cmath>

#include "sfm/model.hpp"
#include "sfm/types.hpp"

namespace sfm {

// Domain extents must be positive (validated by Simulation and RunConfig).
inline double wrap_coordinate(double value, double lower, double upper) noexcept {
    const double length = upper - lower;
    double offset = std::fmod(value - lower, length);
    if (offset < 0.0) offset += length;
    const double wrapped = lower + offset;
    return wrapped < upper ? wrapped : lower;
}

inline Vec2 wrap_position(Vec2 position, const RectangularDomain& domain) noexcept {
    if (domain.boundary_x == BoundaryX::periodic) {
        position.x = wrap_coordinate(position.x, domain.min_x, domain.max_x);
    }
    if (domain.boundary_y == BoundaryY::periodic) {
        position.y = wrap_coordinate(position.y, domain.min_y, domain.max_y);
    }
    return position;
}

inline Vec2 minimum_image_displacement(Vec2 delta, const RectangularDomain& domain) noexcept {
    if (domain.boundary_x == BoundaryX::periodic) {
        const double length = domain.max_x - domain.min_x;
        delta.x -= length * std::round(delta.x / length);
    }
    if (domain.boundary_y == BoundaryY::periodic) {
        const double length = domain.max_y - domain.min_y;
        delta.y -= length * std::round(delta.y / length);
    }
    return delta;
}

}  // namespace sfm
