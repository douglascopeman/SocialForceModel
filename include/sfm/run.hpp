#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "sfm/agent.hpp"
#include "sfm/model.hpp"
#include "sfm/types.hpp"

namespace sfm {

struct RunConfig {
    std::string name{"unnamed"};
    std::string scenario{"bidirectional"};
    DirectionMode direction_mode{DirectionMode::goal_point};
    unsigned int seed{1};
    double duration_s{10.0};
    double time_step_s{0.01};
    double trajectory_interval_s{0.1};
    double render_interval_s{2.0};
    std::filesystem::path output_directory{"results/unnamed"};
    RectangularDomain domain{.min_x = 0.0, .max_x = 20.0, .min_y = 0.0, .max_y = 10.0,
                             .boundary_x = BoundaryX::open};
    InteractionParameters interactions{};
    int agent_count{40};
    std::string initial_placement{"end_groups"};
    double initial_minimum_separation_m{0.4};
    double desired_speed_mean_m_per_s{1.34};
    double desired_speed_stddev_m_per_s{0.26};
    double relaxation_time_s{0.5};
    double maximum_speed_factor{1.3};
};

[[nodiscard]] RunConfig load_run_config(const std::filesystem::path& path);
[[nodiscard]] std::vector<Agent> create_population(const RunConfig& config);
void run_experiment(const RunConfig& config);

}  // namespace sfm
