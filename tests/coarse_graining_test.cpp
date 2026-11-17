#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

#include "sfm/coarse_graining.hpp"

namespace {

void require(bool condition, const char* message) {
    if (!condition) { std::cerr << "Test failure: " << message << '\n'; std::exit(1); }
}

bool near(double x, double y) { return std::abs(x - y) < 1e-10; }

sfm::Agent particle(sfm::Vec2 position, sfm::Vec2 velocity) {
    // Preferred velocity deliberately differs from realised velocity.
    return sfm::Agent({.position = position, .velocity = velocity, .preferred_velocity = {9.0, 9.0}});
}

double integral(const std::vector<sfm::CoarseField>& fields, const sfm::CoarseGrainer& estimator) {
    double sum = 0.0;
    for (std::size_t i = 0; i < fields.size(); ++i) {
        sum += fields[i].density * estimator.cells()[i].width_m * estimator.cells()[i].height_m;
    }
    return sum;
}

}  // namespace

int main() {
    sfm::RectangularDomain domain{.min_x = -1.0, .max_x = 2.3, .min_y = -2.0, .max_y = 1.2};
    sfm::CoarseGrainingConfig config;
    config.grid_spacing_m = 0.7;
    config.kernel_width_m = 0.2;
    config.kernel_cutoff_sigma = 3.0;

    { // Conservation includes irregular edge cells and excludes soft-wall escapees.
        sfm::CoarseGrainer estimator(domain, config);
        const std::vector agents{particle({-0.99, -1.99}, {2.0, 0.0}), particle({2.29, 1.19}, {2.0, 0.0}),
                                 particle({0.1, -0.3}, {2.0, 0.0}), particle({2.4, 0.0}, {2.0, 0.0})};
        estimator.accumulate(agents, 0.1);
        estimator.accumulate(agents, 0.3);
        require(near(estimator.mean_in_domain_agents(), 3.0), "in-domain time average");
        require(near(estimator.mean_excluded_agents(), 1.0), "outside agents explicitly counted");
        const auto fields = estimator.finish();
        require(near(integral(fields, estimator), 3.0), "renormalized density conserves particle number");
        bool saw_empty = false;
        for (const auto& f : fields) {
            if (f.sample_count) {
                require(near(f.mean_velocity.x, 2.0) && near(f.mean_velocity.y, 0.0), "uniform realised velocity recovered");
                require(near(f.polarisation, 1.0) && near(f.polarisation_vector.x, 1.0), "aligned flow has unit polarisation");
                require(f.effective_samples > 0.0 && f.effective_samples <= f.sample_count + 1e-10, "effective support bounded by raw count");
            } else {
                saw_empty = true;
                require(f.density == 0.0 && f.effective_samples == 0.0 && std::isnan(f.polarisation) &&
                        std::isnan(f.mean_velocity.x), "empty conditional fields are undefined");
            }
        }
        require(saw_empty, "compact kernel leaves unsupported cells");
        require(estimator.duration_s() == 0.0 && estimator.time_samples() == 0, "finish resets window");
    }
    { // Unequal speeds distinguish average unit headings from normalized mean velocity.
        sfm::CoarseGrainer estimator(domain, config);
        const std::vector agents{particle({0.0, 0.0}, {3.0, 0.0}), particle({0.0, 0.0}, {-1.0, 0.0})};
        estimator.accumulate(agents, 1.0);
        for (const auto& f : estimator.finish()) if (f.sample_count) {
            require(near(f.mean_velocity.x, 1.0), "opposing unequal speeds retain net velocity");
            require(near(f.polarisation, 0.0), "opposing headings cancel regardless of speed");
            require(near(f.effective_samples, 2.0), "two equal agent weights give ESS two");
        }
    }
    { // Time weights apply to moments before taking ratios and magnitudes.
        sfm::CoarseGrainer estimator(domain, config);
        const std::vector right{particle({0.0, 0.0}, {2.0, 0.0})};
        const std::vector left{particle({0.0, 0.0}, {-2.0, 0.0})};
        estimator.accumulate(right, 0.75);
        estimator.accumulate(left, 0.25);
        for (const auto& f : estimator.finish()) if (f.sample_count) {
            require(near(f.mean_velocity.x, 1.0) && near(f.polarisation, 0.5), "duration weighted moments");
            require(near(f.effective_samples, 1.6), "duration weights included in ESS");
        }
        const std::vector stopped{particle({0.0, 0.0}, {})};
        estimator.accumulate(stopped, 1.0);
        for (const auto& f : estimator.finish()) if (f.sample_count) {
            require(f.polarisation == 0.0 && f.mean_velocity.x == 0.0, "stationary agents have zero heading and velocity");
        }
        const std::vector mixed{particle({0.0, 0.0}, {2.0, 0.0}), particle({0.0, 0.0}, {})};
        estimator.accumulate(mixed, 1.0);
        for (const auto& f : estimator.finish()) if (f.sample_count) {
            require(near(f.polarisation, 0.5), "stationary agents remain in polarisation denominator");
        }
        const std::vector two_left{particle({0.0, 0.0}, {-2.0, 0.0}), particle({0.0, 0.0}, {-2.0, 0.0})};
        estimator.accumulate(right, 0.75);
        estimator.accumulate(two_left, 0.25);
        const auto varying = estimator.finish();
        require(near(integral(varying, estimator), 1.25), "conservation uses time-averaged population");
        for (const auto& f : varying) if (f.sample_count) {
            require(near(f.mean_velocity.x, 0.4) && near(f.polarisation, 0.2), "moments normalized by density over whole window");
        }
    }
    { // Independent analytic Gaussian cell integrals, with truncation at a corner.
        const sfm::RectangularDomain box{.max_x = 2.0, .max_y = 2.0};
        auto c = config;
        c.grid_spacing_m = 1.0;
        c.kernel_width_m = 0.5;
        c.boundary_policy = "truncate";
        sfm::CoarseGrainer estimator(box, c);
        const std::vector agents{particle({0.0, 0.0}, {})};
        estimator.accumulate(agents, 1.0);
        const auto fields = estimator.finish();
        const double axis_mass = 0.5 * std::erf(std::sqrt(2.0)) / std::erf(3.0 / std::sqrt(2.0));
        require(near(fields[0].density, axis_mass * axis_mass), "cell density matches analytic Gaussian integral");
        require(near(integral(fields, estimator), 0.25), "corner truncation retains quarter kernel mass");
    }
    { // Multiple periodic images, both axes, nonzero origins, and wrapped coordinates.
        auto torus = domain;
        torus.boundary_x = sfm::BoundaryX::periodic;
        torus.boundary_y = sfm::BoundaryY::periodic;
        auto c = config;
        c.kernel_width_m = 2.0; // Support is wider than one box.
        c.boundary_policy = "truncate";
        sfm::CoarseGrainer first(torus, c), shifted(torus, c);
        const std::vector agents{particle({-0.9, -1.9}, {0.0, 1.0})};
        const std::vector images{particle({-0.9 + 3.3 * 3.0, -1.9 - 3.2 * 2.0}, {0.0, 1.0})};
        first.accumulate(agents, 1.0);
        shifted.accumulate(images, 1.0);
        const auto a = first.finish(), b = shifted.finish();
        require(near(integral(a, first), 1.0), "periodic density conserves with wide kernels");
        for (std::size_t i = 0; i < a.size(); ++i) require(near(a[i].density, b[i].density), "periodic translation invariance");
    }
    { // Kernel mass crosses periodic joins, while physical edges can still truncate.
        sfm::RectangularDomain box{.max_x = 4.0, .max_y = 4.0,
                                   .boundary_x = sfm::BoundaryX::periodic,
                                   .boundary_y = sfm::BoundaryY::periodic};
        auto c = config;
        c.grid_spacing_m = 1.0;
        c.kernel_width_m = 0.4;
        c.boundary_policy = "truncate";
        const std::vector agents{particle({0.0, 0.0}, {1.0, 0.0})};
        sfm::CoarseGrainer estimator(box, c);
        estimator.accumulate(agents, 1.0);
        const auto fields = estimator.finish();
        const double axis_mass = 0.5 * std::erf(2.5 / std::sqrt(2.0)) / std::erf(3.0 / std::sqrt(2.0));
        require(near(fields[0].density, axis_mass * axis_mass), "periodic image mass matches independent cell integral");
        require(near(fields[0].density, fields[3].density) && near(fields[0].density, fields[12].density) &&
                near(fields[0].density, fields[15].density), "corner kernel wraps symmetrically to all four corners");
        box.boundary_y = sfm::BoundaryY::walls;
        sfm::CoarseGrainer channel(box, c);
        channel.accumulate(agents, 1.0);
        require(near(integral(channel.finish(), channel), 0.5), "only nonperiodic edge loses kernel mass");
    }
    { // Decimal extents must not introduce zero-area edge cells through rounding.
        const sfm::RectangularDomain box{.max_x = 0.28, .max_y = 0.28};
        auto c = config;
        c.grid_spacing_m = 0.01;
        c.kernel_width_m = 0.01;
        sfm::CoarseGrainer estimator(box, c);
        const std::vector agents{particle({0.27, 0.27}, {})};
        estimator.accumulate(agents, 1.0);
        const auto fields = estimator.finish();
        for (std::size_t i = 0; i < fields.size(); ++i) {
            require(estimator.cells()[i].width_m > 0.0 && estimator.cells()[i].height_m > 0.0 &&
                    std::isfinite(fields[i].density), "finite density in positive-area cells");
        }
        require(near(integral(fields, estimator), 1.0), "conservation for decimal grid extents");
    }
    { // Selecting conditional fields alone still keeps their normalization weights.
        auto c = config;
        c.fields = {sfm::MacroscopicField::polarisation};
        sfm::CoarseGrainer estimator(domain, c);
        const std::vector agents{particle({0.0, 0.0}, {0.0, 2.0})};
        estimator.accumulate(agents, 1.0);
        for (const auto& f : estimator.finish()) if (f.sample_count) require(near(f.polarisation, 1.0), "polarisation without density output");
    }
    for (int invalid = 0; invalid < 6; ++invalid) {
        auto c = config;
        if (invalid == 0) c.kernel_width_m = 0.0;
        if (invalid == 1) c.window_s = std::numeric_limits<double>::quiet_NaN();
        if (invalid == 2) c.fields.clear();
        if (invalid == 3) c.fields = {sfm::MacroscopicField::density, sfm::MacroscopicField::density};
        if (invalid == 4) c.kernel = "unknown";
        if (invalid == 5) c.grid_spacing_m = 1e-9;
        bool rejected = false;
        try { sfm::CoarseGrainer estimator(domain, c); } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "invalid coarse-graining settings rejected");
    }
}
