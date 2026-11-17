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
    require(argc == 3, "provide x-periodic and doubly periodic configuration paths");
    const auto config = sfm::load_run_config(argv[1]);
    require(config.domain.boundary_x == sfm::BoundaryX::periodic, "periodic configuration is parsed");
    require(config.domain.boundary_y == sfm::BoundaryY::walls, "omitting boundary_y retains transverse walls");
    const auto bulk_config = sfm::load_run_config(argv[2]);
    require(bulk_config.domain.boundary_y == sfm::BoundaryY::periodic && bulk_config.scenario == "unidirectional",
            "doubly periodic unidirectional configuration is parsed");

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
    const sfm::RectangularDomain torus{.min_x = -5.0, .max_x = 5.0, .min_y = -3.0, .max_y = 3.0,
                                      .boundary_x = sfm::BoundaryX::periodic,
                                      .boundary_y = sfm::BoundaryY::periodic};
    for (double sign : {-1.0, 1.0}) {
        sfm::Simulation simulation(torus);
        auto agent = moving_agent(sign * 4.9, sign);
        agent.state().position.y = sign * 2.9;
        agent.state().velocity.y = sign;
        agent.state().preferred_velocity = agent.state().velocity;
        agent.state().desired_speed_m_per_s = std::sqrt(2.0);
        agent.state().fixed_direction = {sign, sign};
        agent.state().id = 123;
        simulation.add_agent(agent);
        simulation.step(0.2);
        const auto& state = simulation.agents()[0].state();
        require(near(state.position.x, -sign * 4.9) && near(state.position.y, -sign * 2.9),
                "simultaneous corner crossings wrap both coordinates");
        require(near(state.velocity.x, sign) && near(state.velocity.y, sign) &&
                    near(state.preferred_velocity.x, sign) && near(state.preferred_velocity.y, sign),
                "corner crossing preserves realized and preferred velocities");
        simulation.step(30.0);
        require(near(state.position.x, -sign * 4.9) && near(state.position.y, -sign * 2.9),
                "multiple-length corner displacements wrap correctly");
        require(state.id == 123 && simulation.agent_count() == 1, "corner crossing preserves identity and population");
        require(sfm::norm(simulation.wall_force(simulation.agents()[0])) == 0.0,
                "fully periodic domain has no wall force even with wall flags enabled");
    }
    {
        sfm::Simulation simulation(torus);
        auto agent = moving_agent(25.0, 1.0);
        agent.state().position.y = 15.0;
        simulation.add_agent(agent);
        require(near(simulation.agents()[0].state().position.x, -5.0) &&
                    near(simulation.agents()[0].state().position.y, -3.0),
                "insertion wraps upper endpoints on both axes with nonzero origins");
    }
    { // Test y alone and corners against equivalent interior moving pairs.
        sfm::Simulation periodic(torus);
        auto interior_domain = torus;
        interior_domain.boundary_x = sfm::BoundaryX::open;
        interior_domain.boundary_y = sfm::BoundaryY::walls;
        sfm::Simulation interior(interior_domain);
        for (bool across_x : {false, true}) {
            auto subject = moving_agent(across_x ? -4.8 : 0.0, 0.7);
            auto other = moving_agent(across_x ? 4.7 : 0.5, -0.8);
            subject.state().position.y = -2.8;
            other.state().position.y = 2.7;
            const auto seam_force = periodic.pair_force(subject, other);
            if (across_x) other.state().position.x -= 10.0;
            other.state().position.y -= 6.0;
            const auto interior_force = interior.pair_force(subject, other);
            require(sfm::norm(seam_force) > 0.0 && near(seam_force.x, interior_force.x) && near(seam_force.y, interior_force.y),
                    "elliptical forces across y or corner join equal interior forces");
        }
    }
    {
        auto y_only = torus;
        y_only.boundary_x = sfm::BoundaryX::open;
        sfm::Simulation simulation(y_only);
        auto agent = moving_agent(4.9, 1.0);
        agent.state().position.y = 2.9;
        agent.state().velocity.y = 1.0;
        agent.state().preferred_velocity = agent.state().velocity;
        agent.state().desired_speed_m_per_s = std::sqrt(2.0);
        agent.state().fixed_direction = {1.0, 1.0};
        simulation.add_agent(agent);
        simulation.step(0.2);
        require(near(simulation.agents()[0].state().position.x, 5.1) &&
                    near(simulation.agents()[0].state().position.y, -2.9),
                "y periodicity is independent of x boundary choice");
        bool rejected = false;
        try { simulation.add_agent(sfm::Agent({})); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "y-only periodicity also rejects point goals");
    }
    {
        auto wall_domain = torus;
        wall_domain.boundary_y = sfm::BoundaryY::walls;
        sfm::Simulation simulation(wall_domain);
        auto agent = moving_agent(0.0, 1.0);
        agent.state().position.y = 2.9;
        require(simulation.wall_force(agent).y < 0.0, "wall mode retains inward y repulsion");
        sfm::InteractionParameters interactions;
        interactions.wall_potential_amplitude_m2_per_s2 = 0.0;
        sfm::Simulation no_force(wall_domain, interactions);
        agent.state().velocity = {0.0, 1.0};
        agent.state().preferred_velocity = agent.state().velocity;
        agent.state().desired_speed_m_per_s = 1.0;
        agent.state().fixed_direction = {0.0, 1.0};
        no_force.add_agent(agent);
        no_force.step(0.2);
        require(near(no_force.agents()[0].state().position.y, 3.1), "wall mode does not wrap y positions");
    }
    {
        const auto population = sfm::create_population(bulk_config);
        const auto repeat = sfm::create_population(bulk_config);
        require(population.size() == static_cast<std::size_t>(bulk_config.agent_count), "bulk population size is correct");
        bool near_bottom = false;
        bool near_top = false;
        for (std::size_t i = 0; i < population.size(); ++i) {
            const auto& state = population[i].state();
            require(near(state.fixed_direction.x, 1.0) && near(state.fixed_direction.y, 0.0),
                    "unidirectional population has identical rightward objectives");
            require(near(state.position.x, repeat[i].state().position.x) && near(state.position.y, repeat[i].state().position.y),
                    "bulk population is reproducible");
            near_bottom = near_bottom || state.position.y < bulk_config.domain.min_y + 0.5;
            near_top = near_top || state.position.y > bulk_config.domain.max_y - 0.5;
            for (std::size_t j = 0; j < i; ++j) {
                auto delta = state.position - population[j].state().position;
                const double lx = bulk_config.domain.max_x - bulk_config.domain.min_x;
                const double ly = bulk_config.domain.max_y - bulk_config.domain.min_y;
                delta.x -= lx * std::round(delta.x / lx);
                delta.y -= ly * std::round(delta.y / ly);
                require(sfm::norm(delta) >= bulk_config.initial_minimum_separation_m,
                        "initial separation includes both periodic axes");
            }
        }
        require(near_bottom && near_top, "periodic placement uses the old wall-margin regions");
    }
    {
        auto small = bulk_config;
        small.domain.max_y = 0.6;
        small.agent_count = 4;
        small.initial_minimum_separation_m = 0.1;
        require(sfm::create_population(small).size() == 4, "periodic placement does not require space for wall margins");
        small.domain.boundary_y = sfm::BoundaryY::walls;
        bool rejected = false;
        try { (void)sfm::create_population(small); }
        catch (const std::runtime_error&) { rejected = true; }
        require(rejected, "wall placement still validates room for margins");
    }
    return 0;
}
