#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

#include "sfm/run.hpp"
#include "sfm/simulation.hpp"

namespace {
void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "Test failure: " << message << '\n';
        std::exit(1);
    }
}
bool near(double lhs, double rhs) { return std::abs(lhs - rhs) < 1.e-10; }

sfm::Agent moving_agent(double x, double speed) {
    sfm::AgentState state;
    state.position = {x, 0.0};
    state.velocity = {speed, 0.0};
    state.preferred_velocity = state.velocity;
    state.desired_speed_m_per_s = std::abs(speed);
    state.direction_mode = sfm::DirectionMode::fixed_vector;
    state.fixed_direction = {speed < 0.0 ? -1.0 : 1.0, 0.0};
    return sfm::Agent(state);
}
}  // namespace

int main(int argc, char** argv) {
    require(argc == 2, "provide periodic configuration path");
    const auto config = sfm::load_run_config(argv[1]);
    require(config.domain.boundary_x == sfm::BoundaryX::periodic, "periodic configuration is parsed");

    // Nonzero origin verifies wrapping into [min_x, max_x), not just [0, L).
    sfm::RectangularDomain domain{.min_x = -5.0, .max_x = 5.0, .min_y = -10.0, .max_y = 10.0,
                                 .boundary_x = sfm::BoundaryX::periodic,
                                 .wall_bottom = false, .wall_top = false};
    for (double sign : {-1.0, 1.0}) {
        sfm::Simulation simulation(domain);
        auto agent = moving_agent(sign * 4.9, sign);
        simulation.add_agent(agent);
        simulation.step(0.2);
        require(near(simulation.agents()[0].state().position.x, -sign * 4.9), "crossing either end wraps position");
        require(near(simulation.agents()[0].state().velocity.x, sign), "wrapping retains velocity");
        simulation.step(30.0);
        require(near(simulation.agents()[0].state().position.x, -sign * 4.9), "multiple-length displacement wraps correctly");
        require(simulation.agent_count() == 1, "wrapping preserves population");
    }
    {
        sfm::Simulation simulation(domain);
        simulation.add_agent(moving_agent(25.0, 1.0));
        require(near(simulation.agents()[0].state().position.x, -5.0), "initial positions and upper endpoint are wrapped");
        require(sfm::norm(simulation.wall_force(simulation.agents()[0])) == 0.0, "periodic ends exert no wall force");
        bool rejected = false;
        try { simulation.add_agent(sfm::Agent({})); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "periodic simulation rejects ambiguous point-goal objectives");
    }
    { // Compare a moving pair across the seam with an equivalent interior pair.
        sfm::Simulation periodic(domain);
        auto interior_domain = domain;
        interior_domain.boundary_x = sfm::BoundaryX::open;
        sfm::Simulation interior(interior_domain);
        auto subject = moving_agent(-4.8, 0.7);
        auto other = moving_agent(4.7, -0.8);
        other.state().position.y = 0.4;
        const auto seam_force = periodic.pair_force(subject, other);
        other.state().position.x -= 10.0;
        const auto interior_force = interior.pair_force(subject, other);
        require(sfm::norm(seam_force) > 0.0 && near(seam_force.x, interior_force.x) && near(seam_force.y, interior_force.y),
                "elliptical force and perception agree across seam and in interior");
    }
    { // Wrapping is exclusive to periodic domains.
        for (auto boundary : {sfm::BoundaryX::open, sfm::BoundaryX::walls}) {
            auto nonperiodic_domain = domain;
            nonperiodic_domain.boundary_x = boundary;
            sfm::InteractionParameters interactions;
            interactions.wall_potential_amplitude_m2_per_s2 = 0.0;
            sfm::Simulation simulation(nonperiodic_domain, interactions);
            simulation.add_agent(moving_agent(4.9, 1.0));
            simulation.step(0.2);
            require(near(simulation.agents()[0].state().position.x, 5.1), "nonperiodic modes do not wrap");
        }
    }
    return 0;
}
