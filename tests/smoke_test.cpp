#include <cmath>
#include <cstdlib>
#include <iostream>

#include "sfm/simulation.hpp"
#include "sfm/run.hpp"

namespace {

bool near(double actual, double expected, double tolerance = 1.0e-9) {
    return std::abs(actual - expected) <= tolerance;
}

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "Test failure: " << message << '\n';
        std::exit(1);
    }
}

sfm::Agent make_agent(std::uint64_t id, sfm::Vec2 position, sfm::Vec2 goal) {
    return sfm::Agent({.id = id, .position = position, .goal = goal});
}

}  // namespace

int main(int argc, char** argv) {
    require(argc == 3, "provide default-mode and fixed-vector configuration paths");
    require(sfm::load_run_config(argv[1]).direction_mode == sfm::DirectionMode::goal_point,
            "omitting the mode preserves goal-point steering");
    require(sfm::load_run_config(argv[2]).direction_mode == sfm::DirectionMode::fixed_vector,
            "configuration reader selects fixed-vector mode");
    const sfm::RectangularDomain domain{.min_x = -100.0, .max_x = 100.0, .min_y = -100.0, .max_y = 100.0};

    {  // The objective changes steering after displacement, independently of the goal.
        sfm::Simulation simulation(domain);
        auto agent = make_agent(1, {0.0, 2.0}, {10.0, 0.0});
        require(simulation.driving_force(agent).y < 0.0, "goal-point steering corrects lateral displacement");
        agent.state().direction_mode = sfm::DirectionMode::fixed_vector;
        agent.state().fixed_direction = {3.0, 0.0};
        const auto force = simulation.driving_force(agent);
        require(near(force.x, 2.68) && near(force.y, 0.0), "fixed direction is normalized without lateral steering");
        agent.state().position = {20.0, -4.0};
        require(near(simulation.driving_force(agent).x, force.x), "fixed heading persists beyond the goal point");
        agent.state().fixed_direction = {-1.0, 0.0};
        require(near(simulation.driving_force(agent).x, -2.68), "leftward fixed heading drives left");

        agent.state().fixed_direction = {1.0, 0.0};
        simulation.add_agent(agent);
        for (int step = 0; step < 100; ++step) simulation.step(0.01);
        require(near(simulation.agents()[0].state().position.y, -4.0),
                "isolated fixed-vector trajectory has no lateral drift");
    }
    {  // Perception and elliptical anticipation must use the same selected heading.
        sfm::Simulation simulation(domain);
        auto subject = make_agent(1, {0.0, 0.0}, {-10.0, 0.0});
        auto other = make_agent(2, {1.0, 0.4}, {-10.0, 0.4});
        subject.state().direction_mode = sfm::DirectionMode::fixed_vector;
        other.state().direction_mode = sfm::DirectionMode::fixed_vector;
        other.state().velocity = {0.5, 0.0};
        const auto fixed_force = simulation.pair_force(subject, other);
        subject.state().direction_mode = sfm::DirectionMode::goal_point;
        other.state().direction_mode = sfm::DirectionMode::goal_point;
        subject.state().goal = {10.0, 0.0};
        other.state().goal = {10.0, 0.4};
        const auto goal_force = simulation.pair_force(subject, other);
        require(sfm::norm(fixed_force) > 0.0 && near(fixed_force.x, goal_force.x) && near(fixed_force.y, goal_force.y),
                "matching selected headings give identical pair forces regardless of mode");
    }
    {  // An isolated pedestrian relaxes toward the desired velocity (paper eq. 2).
        sfm::Simulation simulation(domain);
        simulation.add_agent(make_agent(1, {0.0, 0.0}, {100.0, 0.0}));
        const auto force = simulation.driving_force(simulation.agents().front());
        require(near(force.x, 2.68) && near(force.y, 0.0), "driving force has expected direction and magnitude");
    }
    {  // No driving mismatch and negligible walls yields a preserved preferred velocity.
        sfm::Simulation simulation(domain);
        auto agent = make_agent(1, {0.0, 0.0}, {100.0, 0.0});
        agent.state().velocity = {1.0, 0.0};
        agent.state().preferred_velocity = {1.0, 0.0};
        agent.state().desired_speed_m_per_s = 1.0;
        simulation.add_agent(agent);
        simulation.step(0.01);
        require(near(simulation.agents().front().state().preferred_velocity.x, 1.0, 1.0e-8),
                "zero net force preserves preferred velocity");
    }
    {  // Realized speed obeys the paper's cap (eqs. 11--12).
        sfm::Simulation simulation(domain);
        auto agent = make_agent(1, {0.0, 0.0}, {100.0, 0.0});
        agent.state().preferred_velocity = {100.0, 0.0};
        agent.state().desired_speed_m_per_s = 1.0;
        agent.state().maximum_speed_factor = 1.3;
        simulation.add_agent(agent);
        simulation.step(0.01);
        require(norm(simulation.agents().front().state().velocity) <= 1.3 + 1.0e-12, "speed cap is enforced");
    }
    {  // Forces point away from a person and from the nearest wall.
        sfm::Simulation simulation({.min_x = 0.0, .max_x = 10.0, .min_y = 0.0, .max_y = 10.0});
        simulation.add_agent(make_agent(1, {0.1, 5.0}, {10.0, 5.0}));
        simulation.add_agent(make_agent(2, {1.0, 5.0}, {10.0, 5.0}));
        require(simulation.wall_force(simulation.agents()[0]).x > 0.0, "wall force points into domain");
        require(simulation.pair_force(simulation.agents()[0], simulation.agents()[1]).x < 0.0,
                "pair force points away from other pedestrian");
    }
    {  // Euler error falls under timestep refinement for isolated relaxation.
        const auto position_after = [&domain](double dt) {
            sfm::Simulation simulation(domain);
            auto agent = make_agent(1, {0.0, 0.0}, {100.0, 0.0});
            agent.state().desired_speed_m_per_s = 1.0;
            agent.state().relaxation_time_s = 0.5;
            simulation.add_agent(agent);
            const int steps = static_cast<int>(1.0 / dt);
            for (int step = 0; step < steps; ++step) {
                simulation.step(dt);
            }
            return simulation.agents().front().state().position.x;
        };
        // Integral of 1 - exp(-t/tau) over [0, 1] with tau = 0.5.
        const double exact = 1.0 - 0.5 * (1.0 - std::exp(-2.0));
        const double coarse_error = std::abs(position_after(0.01) - exact);
        const double fine_error = std::abs(position_after(0.005) - exact);
        require(fine_error < coarse_error, "trajectory converges under timestep refinement");
    }
    return 0;
}
