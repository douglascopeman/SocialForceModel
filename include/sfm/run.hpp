#pragma once

#include <filesystem>
#include <string>

#include "sfm/model.hpp"

namespace sfm {

struct RunConfig {
    std::string name{"unnamed"};
    std::string scenario{"bidirectional"};
    unsigned int seed{1};
    double duration_s{10.0};
    double time_step_s{0.01};
    double trajectory_interval_s{0.1};
    double render_interval_s{2.0};
    std::filesystem::path output_directory{"results/unnamed"};
    RectangularDomain domain{.min_x = 0.0, .max_x = 20.0, .min_y = 0.0, .max_y = 10.0,
                             .wall_left = false, .wall_right = false};
    InteractionParameters interactions{};
    int agent_count{40};
    double desired_speed_mean_m_per_s{1.34};
    double desired_speed_stddev_m_per_s{0.26};
    double relaxation_time_s{0.5};
    double maximum_speed_factor{1.3};
};

[[nodiscard]] RunConfig load_run_config(const std::filesystem::path& path);
void run_experiment(const RunConfig& config);

}  // namespace sfm
