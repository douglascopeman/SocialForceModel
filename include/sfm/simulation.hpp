#pragma once

#include <vector>

#include "sfm/agent.hpp"
#include "sfm/model.hpp"

namespace sfm {

class Simulation {
  public:
    explicit Simulation(RectangularDomain domain = {}, InteractionParameters interactions = {});

    void add_agent(Agent agent);
    [[nodiscard]] std::size_t agent_count() const noexcept;
    [[nodiscard]] const std::vector<Agent>& agents() const noexcept;

    // Advances preferred velocity and then realized position (paper eqs. 10--12).
    void step(double time_step_s);

    [[nodiscard]] Vec2 driving_force(const Agent& agent) const noexcept;
    [[nodiscard]] Vec2 pair_force(const Agent& subject, const Agent& other) const noexcept;
    [[nodiscard]] Vec2 wall_force(const Agent& agent) const noexcept;

  private:
    [[nodiscard]] Vec2 desired_direction(const Agent& agent) const noexcept;
    [[nodiscard]] double perception_weight(Vec2 direction, Vec2 stimulus) const noexcept;
    [[nodiscard]] Vec2 realized_velocity(Vec2 preferred_velocity, double maximum_speed) const noexcept;
    [[nodiscard]] double wrapped_x(double x) const noexcept;

    RectangularDomain domain_;
    InteractionParameters interactions_;
    std::vector<Agent> agents_;
};

}  // namespace sfm
