#pragma once

#include <vector>

#include "sfm/agent.hpp"
#include "sfm/model.hpp"

namespace sfm {

struct EncounterStatistics {
    std::uint64_t pair_force_evaluations{};
    std::uint64_t upward_tie_breaks{};
};

class Simulation {
  public:
    explicit Simulation(RectangularDomain domain = {}, InteractionParameters interactions = {});

    void add_agent(Agent agent);
    [[nodiscard]] std::size_t agent_count() const noexcept;
    [[nodiscard]] const std::vector<Agent>& agents() const noexcept;
    [[nodiscard]] const EncounterStatistics& encounter_statistics() const noexcept;

    // Advances preferred velocity and then realized position (paper eqs. 10--12).
    void step(double time_step_s);

    [[nodiscard]] Vec2 driving_force(const Agent& agent) const noexcept;
    // Throws for coincident positions or an exact anticipation endpoint.
    [[nodiscard]] Vec2 pair_force(const Agent& subject, const Agent& other) const;
    [[nodiscard]] Vec2 wall_force(const Agent& agent) const noexcept;

  private:
    struct PairForceEvaluation {
        Vec2 force{};
        bool used_upward_tie_break{};
    };
    [[nodiscard]] PairForceEvaluation evaluate_pair_force(const Agent& subject, const Agent& other) const;
    [[nodiscard]] Vec2 desired_direction(const Agent& agent) const noexcept;
    [[nodiscard]] double perception_weight(Vec2 direction, Vec2 stimulus) const noexcept;
    [[nodiscard]] Vec2 realized_velocity(Vec2 preferred_velocity, double maximum_speed) const noexcept;

    RectangularDomain domain_;
    InteractionParameters interactions_;
    std::vector<Agent> agents_;
    EncounterStatistics encounter_statistics_{};
};

}  // namespace sfm
