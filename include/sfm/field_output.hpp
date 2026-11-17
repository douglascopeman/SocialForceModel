#pragma once

#include <filesystem>
#include <fstream>
#include <span>

#include "sfm/coarse_graining.hpp"

namespace sfm {

// Accepts consecutive integration intervals, using their right-endpoint state.
// Window boundaries do not alter the microscopic integration steps.
class FieldOutput {
  public:
    FieldOutput(const std::filesystem::path& run_directory, RectangularDomain domain,
                CoarseGrainingConfig config);
    void sample(std::span<const Agent> agents, double duration_s);
    void finish();

  private:
    void write_window();
    std::filesystem::path directory_;
    CoarseGrainingConfig config_;
    CoarseGrainer estimator_;
    std::ofstream index_;
    std::size_t window_index_{};
    double window_start_s_{};
};

}  // namespace sfm
