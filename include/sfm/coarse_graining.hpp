#pragma once

#include "sfm/types.hpp"

namespace sfm {

struct CoarseField {
    double density{};
    Vec2 mean_velocity{};
    double effective_samples{};
};

}  // namespace sfm
