#pragma once

namespace sfm {

struct InteractionParameters {
    double pair_potential_amplitude_m2_per_s2{2.1};
    double pair_range_m{0.3};
    double anticipation_time_s{2.0};
    double wall_potential_amplitude_m2_per_s2{10.0};
    double wall_range_m{0.2};
    double field_of_view_angle_rad{3.490658503988659};  // 200 degrees
    double rear_influence_weight{0.5};
};

struct RectangularDomain {
    double min_x{0.0};
    double max_x{10.0};
    double min_y{0.0};
    double max_y{10.0};
    bool wall_left{true};
    bool wall_right{true};
    bool wall_bottom{true};
    bool wall_top{true};
};

}  // namespace sfm
