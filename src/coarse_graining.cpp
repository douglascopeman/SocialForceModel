#include "sfm/coarse_graining.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "sfm/geometry.hpp"

namespace sfm {
namespace {

constexpr MacroscopicField supported_fields[]{MacroscopicField::density, MacroscopicField::velocity,
                                             MacroscopicField::polarisation};

std::vector<double> edges(double lower, double upper, double spacing) {
    const auto count = static_cast<std::size_t>(std::ceil((upper - lower) / spacing));
    std::vector<double> result;
    result.reserve(count + 1);
    for (std::size_t i = 0; i < count; ++i) {
        const double edge = lower + static_cast<double>(i) * spacing;
        if (edge >= upper) break; // ceil(extent/spacing) can round up at an exact multiple.
        if (!result.empty() && edge <= result.back()) {
            throw std::invalid_argument("coarse_graining grid edges are not representable at this origin");
        }
        result.push_back(edge);
    }
    result.push_back(upper);
    return result;
}

// Integrate the separable Gaussian over cells, summing every supported periodic image.
// The square cutoff is |dx|, |dy| < cutoff*sigma, normalized before boundary treatment.
std::vector<std::pair<std::size_t, double>> axis_weights(
    double position, const std::vector<double>& cell_edges, bool periodic,
    const CoarseGrainingConfig& config) {
    const double sigma = config.kernel_width_m;
    const double radius = sigma * config.kernel_cutoff_sigma;
    const double length = cell_edges.back() - cell_edges.front();
    const double normalization = std::erf(config.kernel_cutoff_sigma / std::sqrt(2.0));
    std::vector<std::pair<std::size_t, double>> result;
    double total = 0.0;
    for (std::size_t i = 0; i + 1 < cell_edges.size(); ++i) {
        double mass = 0.0;
        const int first = periodic ? static_cast<int>(std::ceil((cell_edges[i] - position - radius) / length)) : 0;
        const int last = periodic ? static_cast<int>(std::floor((cell_edges[i + 1] - position + radius) / length)) : 0;
        for (int image = first; image <= last; ++image) {
            const double centre = position + static_cast<double>(image) * length;
            const double lo = std::max(-config.kernel_cutoff_sigma, (cell_edges[i] - centre) / sigma);
            const double hi = std::min(config.kernel_cutoff_sigma, (cell_edges[i + 1] - centre) / sigma);
            if (hi <= lo) continue;
            // erfc retains accuracy when both endpoints are in a Gaussian tail.
            const double integral = lo >= 0.0
                ? std::erfc(lo / std::sqrt(2.0)) - std::erfc(hi / std::sqrt(2.0))
                : hi <= 0.0 ? std::erfc(-hi / std::sqrt(2.0)) - std::erfc(-lo / std::sqrt(2.0))
                            : std::erf(hi / std::sqrt(2.0)) - std::erf(lo / std::sqrt(2.0));
            mass += 0.5 * integral / normalization;
        }
        if (mass > 0.0) {
            result.emplace_back(i, mass);
            total += mass;
        }
    }
    if (config.boundary_policy == "renormalize" || periodic) {
        for (auto& entry : result) entry.second /= total;
    }
    return result;
}

}  // namespace

std::string_view field_name(MacroscopicField field) {
    switch (field) {
        case MacroscopicField::density: return "density";
        case MacroscopicField::velocity: return "velocity";
        case MacroscopicField::polarisation: return "polarisation";
    }
    throw std::invalid_argument("unsupported macroscopic field");
}

MacroscopicField parse_field(std::string_view name) {
    for (auto field : supported_fields) {
        if (field_name(field) == name) return field;
    }
    throw std::invalid_argument("unknown coarse_graining field: " + std::string(name));
}

void validate_coarse_graining(const CoarseGrainingConfig& config, const RectangularDomain& domain) {
    const auto positive = [](double x) { return std::isfinite(x) && x > 0.0; };
    const double width = domain.max_x - domain.min_x;
    const double height = domain.max_y - domain.min_y;
    if (!std::isfinite(domain.min_x) || !std::isfinite(domain.max_x) ||
        !std::isfinite(domain.min_y) || !std::isfinite(domain.max_y) ||
        !positive(width) || !positive(height) || !positive(config.grid_spacing_m) ||
        !positive(config.kernel_width_m) || !positive(config.kernel_cutoff_sigma) ||
        !positive(config.window_s)) {
        throw std::invalid_argument("coarse_graining requires finite positive scales and domain extents");
    }
    if (config.kernel != "gaussian") throw std::invalid_argument("coarse_graining.kernel must be gaussian");
    if (config.boundary_policy != "renormalize" && config.boundary_policy != "truncate") {
        throw std::invalid_argument("coarse_graining.boundary_policy must be renormalize or truncate");
    }
    if (config.kernel_cutoff_sigma > 8.0 ||
        !std::isfinite(config.kernel_width_m * config.kernel_cutoff_sigma) ||
        config.kernel_width_m * config.kernel_cutoff_sigma / std::min(width, height) > 1000.0) {
        throw std::invalid_argument("coarse_graining kernel cutoff must be <= 8 sigma and radius <= 1000 domain lengths");
    }
    const double nx = std::ceil(width / config.grid_spacing_m);
    const double ny = std::ceil(height / config.grid_spacing_m);
    if (!std::isfinite(nx * ny) || nx * ny > 1000000.0 ||
        domain.min_x + config.grid_spacing_m == domain.min_x ||
        domain.min_y + config.grid_spacing_m == domain.min_y) {
        throw std::invalid_argument("coarse_graining grid is too fine (maximum 1000000 cells)");
    }
    if (config.fields.empty()) throw std::invalid_argument("coarse_graining.fields must not be empty");
    for (std::size_t i = 0; i < config.fields.size(); ++i) {
        (void)field_name(config.fields[i]);
        if (std::find(config.fields.begin(), config.fields.begin() + i, config.fields[i]) != config.fields.begin() + i) {
            throw std::invalid_argument("duplicate coarse_graining field");
        }
    }
}

CoarseGrainer::CoarseGrainer(RectangularDomain domain, CoarseGrainingConfig config)
    : domain_(domain), config_(std::move(config)) {
    validate_coarse_graining(config_, domain_);
    x_edges_ = edges(domain_.min_x, domain_.max_x, config_.grid_spacing_m);
    y_edges_ = edges(domain_.min_y, domain_.max_y, config_.grid_spacing_m);
    for (std::size_t iy = 0; iy + 1 < y_edges_.size(); ++iy) {
        for (std::size_t ix = 0; ix + 1 < x_edges_.size(); ++ix) {
            cells_.push_back({ix, iy, {(x_edges_[ix] + x_edges_[ix + 1]) / 2.0,
                                      (y_edges_[iy] + y_edges_[iy + 1]) / 2.0},
                              x_edges_[ix + 1] - x_edges_[ix], y_edges_[iy + 1] - y_edges_[iy]});
        }
    }
    moments_.resize(cells_.size());
    velocity_ = std::find(config_.fields.begin(), config_.fields.end(), MacroscopicField::velocity) != config_.fields.end();
    polarisation_ = std::find(config_.fields.begin(), config_.fields.end(), MacroscopicField::polarisation) != config_.fields.end();
}

void CoarseGrainer::accumulate(std::span<const Agent> agents, double duration_s) {
    if (!std::isfinite(duration_s) || duration_s <= 0.0) throw std::invalid_argument("sample duration must be finite and positive");
    for (const auto& agent : agents) {
        const auto& state = agent.state();
        if (!std::isfinite(state.position.x) || !std::isfinite(state.position.y) ||
            !std::isfinite(state.velocity.x) || !std::isfinite(state.velocity.y)) {
            throw std::invalid_argument("nonfinite microscopic state in coarse graining");
        }
    }
    duration_s_ += duration_s;
    ++time_samples_;
    for (const auto& agent : agents) {
        const auto& state = agent.state();
        const Vec2 position = wrap_position(state.position, domain_);
        if (position.x < domain_.min_x || position.x >= domain_.max_x ||
            position.y < domain_.min_y || position.y >= domain_.max_y) {
            excluded_time_ += duration_s;
            continue;
        }
        in_domain_time_ += duration_s;
        const auto wx = axis_weights(position.x, x_edges_, domain_.boundary_x == BoundaryX::periodic, config_);
        const auto wy = axis_weights(position.y, y_edges_, domain_.boundary_y == BoundaryY::periodic, config_);
        const Vec2 heading = polarisation_ ? normalized(state.velocity) : Vec2{};
        for (const auto& [iy, y_mass] : wy) {
            for (const auto& [ix, x_mass] : wx) {
                const double weight = duration_s * x_mass * y_mass;
                auto& moment = moments_[iy * (x_edges_.size() - 1) + ix];
                moment.weight += weight;
                moment.squared_weight += weight * weight;
                if (velocity_) moment.velocity += weight * state.velocity;
                if (polarisation_) moment.heading += weight * heading;
                ++moment.count;
            }
        }
    }
}

double CoarseGrainer::mean_in_domain_agents() const noexcept {
    return duration_s_ > 0.0 ? in_domain_time_ / duration_s_ : 0.0;
}

double CoarseGrainer::mean_excluded_agents() const noexcept {
    return duration_s_ > 0.0 ? excluded_time_ / duration_s_ : 0.0;
}

std::vector<CoarseField> CoarseGrainer::finish() {
    if (duration_s_ <= 0.0) throw std::logic_error("cannot finish an empty coarse-graining window");
    std::vector<CoarseField> result(cells_.size());
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < cells_.size(); ++i) {
        const auto& m = moments_[i];
        auto& field = result[i];
        field.density = m.weight / (duration_s_ * cells_[i].width_m * cells_[i].height_m);
        field.sample_count = m.count;
        field.effective_samples = m.squared_weight > 0.0 ? m.weight * m.weight / m.squared_weight : 0.0;
        field.mean_velocity = m.weight > 0.0 && velocity_ ? m.velocity * (1.0 / m.weight) : Vec2{nan, nan};
        field.polarisation_vector = m.weight > 0.0 && polarisation_ ? m.heading * (1.0 / m.weight) : Vec2{nan, nan};
        field.polarisation = m.weight > 0.0 && polarisation_ ? std::min(1.0, norm(field.polarisation_vector)) : nan;
    }
    std::fill(moments_.begin(), moments_.end(), Moments{});
    duration_s_ = in_domain_time_ = excluded_time_ = 0.0;
    time_samples_ = 0;
    return result;
}

}
