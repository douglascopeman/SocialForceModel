#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <stdexcept>

#include "sfm/simulation.hpp"

namespace {
void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "Test failure: " << message << '\n';
        std::exit(1);
    }
}
bool near(double actual, double expected, double tolerance = 1.e-10) {
    return std::abs(actual - expected) <= tolerance * std::max(1.0, std::abs(expected));
}

sfm::Agent make_agent(sfm::Vec2 position, sfm::Vec2 velocity = {1.0, 0.0},
                      sfm::Vec2 direction = {1.0, 0.0}, std::uint64_t id = 0) {
    sfm::AgentState state;
    state.id = id;
    state.position = position;
    state.velocity = velocity;
    state.preferred_velocity = velocity;
    state.desired_speed_m_per_s = std::hypot(velocity.x, velocity.y);
    state.direction_mode = sfm::DirectionMode::fixed_vector;
    state.fixed_direction = direction;
    return sfm::Agent(state);
}

double potential(sfm::Vec2 r, sfm::Vec2 direction, double stride) {
    const auto shifted = r - stride * direction;
    const double sum = std::hypot(r.x, r.y) + std::hypot(shifted.x, shifted.y);
    const double b = 0.5 * std::sqrt((sum - stride) * (sum + stride));
    return 2.1 * std::exp(-b / 0.3);
}
}  // namespace

int main() {
    const sfm::RectangularDomain domain{.min_x = -100, .max_x = 100, .min_y = -100, .max_y = 100,
                                      .boundary_x = sfm::BoundaryX::open,
                                      .wall_bottom = false, .wall_top = false};
    sfm::InteractionParameters interactions;
    interactions.field_of_view_angle_rad = 2.0 * std::acos(-1.0);
    sfm::Simulation simulation(domain, interactions);
    const auto other = make_agent({0, 0});

    // At the segment midpoint b = |y|, so the exact force is known independently.
    for (double y : {1.e-3, 1.e-5, 1.e-7, 1.e-12, 1.e-100, 1.e-200}) {
        for (double sign : {-1.0, 1.0}) {
            const auto force = simulation.pair_force(make_agent({1, sign * y}), other);
            require(near(force.x, 0.0) && near(force.y, sign * 7.0 * std::exp(-y / 0.3)),
                    "near-axis force retains its magnitude and correct side");
        }
    }
    const auto tie = simulation.pair_force(make_agent({1, 0}), other);
    require(near(tie.x, 0) && near(tie.y, 7), "midpoint exact tie selects upward limit");
    const auto off_midpoint = simulation.pair_force(make_agent({0.5, 0}), other);
    require(near(off_midpoint.y, 7.0 / std::sqrt(0.75)), "off-midpoint tie uses geometry-dependent magnitude");
    for (double x : {0.25, 0.5, 1.5, 1.75}) {
        for (double sign : {-1.0, 1.0}) {
            const auto near_line = simulation.pair_force(make_agent({x, sign * 1.e-12}), other);
            require(near(near_line.y, sign * 7.0 / std::sqrt(x * (2.0 - x))) && std::abs(near_line.x) < 1.e-9,
                    "off-midpoint forces approach their correct one-sided limits");
        }
    }
    const auto reversed = simulation.pair_force(make_agent({-1, 0}), make_agent({0, 0}, {-1, 0}, {-1, 0}));
    require(near(reversed.y, 7), "leftward motion still selects global upward limit");
    const auto rotated = simulation.pair_force(make_agent({0.6, 0.8}), make_agent({0, 0}, {0.6, 0.8}, {0.6, 0.8}));
    require(near(rotated.x, -5.6) && near(rotated.y, 4.2), "oblique tie selects perpendicular direction with positive y");
    const auto vertical = simulation.pair_force(make_agent({0, 1}), make_agent({0, 0}, {0, 1}, {0, 1}));
    require(near(vertical.x, 7) && near(vertical.y, 0), "vertical heading has deterministic positive-x fallback");

    for (auto position : {sfm::Vec2{0, 0}, sfm::Vec2{2, 0}}) {
        bool rejected = false;
        try { (void)simulation.pair_force(make_agent(position), other); }
        catch (const std::domain_error&) { rejected = true; }
        require(rejected, "exact endpoints are explicitly flagged");
    }
    for (double x : {0.0, 2.0}) {
        const auto force = simulation.pair_force(make_agent({x, 1.e-12}), other);
        require(std::isfinite(force.x) && std::isfinite(force.y) && force.y > 0,
                "near endpoints remain finite when the analytical force is finite");
    }
    const auto stationary = simulation.pair_force(make_agent({0, 1}), make_agent({0, 0}, {0, 0}));
    require(near(stationary.x, 0) && near(stationary.y, 7.0 * std::exp(-1.0 / 0.3)),
            "zero anticipation displacement reduces to radial repulsion");
    auto disabled = interactions;
    disabled.pair_potential_amplitude_m2_per_s2 = 0;
    sfm::Simulation no_pairs(domain, disabled);
    require(sfm::norm(no_pairs.pair_force(other, other)) == 0, "disabled interactions need no singular evaluation");

    // Independent central differences away from the nonsmooth line, including rotated headings.
    std::mt19937 generator(937);
    std::uniform_real_distribution<double> coordinates(-3, 3);
    for (int index = 0; index < 1000; ++index) {
        const sfm::Vec2 r{coordinates(generator), coordinates(generator)};
        const sfm::Vec2 direction = index % 2 == 0 ? sfm::Vec2{1, 0} : sfm::Vec2{0.6, 0.8};
        if (std::abs(-direction.y * r.x + direction.x * r.y) < 0.1) continue;
        const auto actual = simulation.pair_force(make_agent(r), make_agent({0, 0}, direction, direction));
        const double h = 1.e-6;
        const sfm::Vec2 expected{
            -(potential(r + sfm::Vec2{h, 0}, direction, 2) - potential(r - sfm::Vec2{h, 0}, direction, 2)) / (2 * h),
            -(potential(r + sfm::Vec2{0, h}, direction, 2) - potential(r - sfm::Vec2{0, h}, direction, 2)) / (2 * h)};
        require(sfm::norm(actual - expected) <= 1.e-7 * std::max(1.0, sfm::norm(expected)),
                "stable force matches independent potential differences");
    }

    // Force queries have no telemetry side effects; only integration counts events.
    require(simulation.encounter_statistics().pair_force_evaluations == 0 &&
            simulation.encounter_statistics().upward_tie_breaks == 0, "diagnostic queries do not inflate counters");
    simulation.add_agent(make_agent({1, 0}, {1, 0}, {1, 0}, 1));
    simulation.add_agent(make_agent({0, 0}, {1, 0}, {1, 0}, 2));
    simulation.step(0.001);
    require(simulation.encounter_statistics().pair_force_evaluations == 2 &&
            simulation.encounter_statistics().upward_tie_breaks == 1, "forced tie is counted once among two directed evaluations");
    require(simulation.agents()[0].state().velocity.y > 0, "forced tie actually moves the subject upward");
    simulation.step(0.001);
    require(simulation.encounter_statistics().pair_force_evaluations == 4 &&
            simulation.encounter_statistics().upward_tie_breaks == 1, "nonzero displacement does not invoke tie breaker");

    auto periodic_domain = domain;
    periodic_domain.min_x = 0;
    periodic_domain.max_x = 20;
    periodic_domain.boundary_x = sfm::BoundaryX::periodic;
    sfm::Simulation periodic(periodic_domain, interactions);
    const auto across_join = periodic.pair_force(make_agent({0.1, 0}), make_agent({19.1, 0}));
    require(near(across_join.y, 7), "tie convention uses periodic image geometry");
    return 0;
}
