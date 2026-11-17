#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "sfm/agent.hpp"
#include "sfm/model.hpp"

namespace sfm {

enum class MacroscopicField { density, velocity, polarisation };

[[nodiscard]] std::string_view field_name(MacroscopicField field);
[[nodiscard]] MacroscopicField parse_field(std::string_view name);

struct CoarseGrainingConfig {
    bool enabled{false};
    std::vector<MacroscopicField> fields{MacroscopicField::density, MacroscopicField::velocity,
                                        MacroscopicField::polarisation};
    double grid_spacing_m{0.5};
    std::string kernel{"gaussian"};
    double kernel_width_m{0.5};  // Gaussian standard deviation on each axis.
    double kernel_cutoff_sigma{3.0};
    double window_s{1.0};
    std::string boundary_policy{"renormalize"};  // Or truncate on nonperiodic axes.
};

void validate_coarse_graining(const CoarseGrainingConfig& config, const RectangularDomain& domain);

struct GridCell {
    std::size_t ix{}, iy{};
    Vec2 centre{};
    double width_m{}, height_m{};
};

struct CoarseField {
    double density{};
    Vec2 mean_velocity{};
    Vec2 polarisation_vector{};
    double polarisation{};
    std::uint64_t sample_count{};
    double effective_samples{};
};

// Streaming, duration-weighted moments. Positions outside nonperiodic axes are excluded.
// finish() returns cell averages and resets the window; empty conditional fields are NaN.
class CoarseGrainer {
  public:
    CoarseGrainer(RectangularDomain domain, CoarseGrainingConfig config);
    void accumulate(std::span<const Agent> agents, double duration_s);
    [[nodiscard]] std::vector<CoarseField> finish();
    [[nodiscard]] const std::vector<GridCell>& cells() const noexcept { return cells_; }
    [[nodiscard]] double duration_s() const noexcept { return duration_s_; }
    [[nodiscard]] double mean_in_domain_agents() const noexcept;
    [[nodiscard]] double mean_excluded_agents() const noexcept;
    [[nodiscard]] std::uint64_t time_samples() const noexcept { return time_samples_; }

  private:
    struct Moments {
        double weight{}, squared_weight{};
        Vec2 velocity{}, heading{};
        std::uint64_t count{};
    };
    RectangularDomain domain_;
    CoarseGrainingConfig config_;
    std::vector<double> x_edges_, y_edges_;
    std::vector<GridCell> cells_;
    std::vector<Moments> moments_;
    bool velocity_{}, polarisation_{};
    double duration_s_{}, in_domain_time_{}, excluded_time_{};
    std::uint64_t time_samples_{};
};

}  // namespace sfm
