#include "sfm/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "sfm/geometry.hpp"

namespace sfm {

Simulation::Simulation(RectangularDomain domain, InteractionParameters interactions)
    : domain_(domain), interactions_(interactions) {
    if (!(domain_.min_x < domain_.max_x && domain_.min_y < domain_.max_y)) {
        throw std::invalid_argument("domain bounds must have positive extent");
    }
    if (interactions_.pair_range_m <= 0.0 || interactions_.wall_range_m <= 0.0 ||
        interactions_.anticipation_time_s < 0.0 || interactions_.rear_influence_weight < 0.0 ||
        interactions_.rear_influence_weight > 1.0) {
        throw std::invalid_argument("invalid interaction parameters");
    }
}

void Simulation::add_agent(Agent agent) {
    if (domain_.boundary_x == BoundaryX::periodic || domain_.boundary_y == BoundaryY::periodic) {
        if (agent.state().direction_mode != DirectionMode::fixed_vector) {
            throw std::invalid_argument("periodic boundaries require fixed_vector direction mode");
        }
    }
    agent.state().position = wrap_position(agent.state().position, domain_);
    agents_.push_back(std::move(agent));
}
std::size_t Simulation::agent_count() const noexcept { return agents_.size(); }
const std::vector<Agent>& Simulation::agents() const noexcept { return agents_; }
const EncounterStatistics& Simulation::encounter_statistics() const noexcept { return encounter_statistics_; }

Vec2 Simulation::desired_direction(const Agent& agent) const noexcept {
    return sfm::desired_direction(agent.state());
}

Vec2 Simulation::driving_force(const Agent& agent) const noexcept {
    const AgentState& state = agent.state();
    if (state.relaxation_time_s <= 0.0) {
        return {};
    }
    return (state.desired_speed_m_per_s * desired_direction(agent) - state.velocity) *
           (1.0 / state.relaxation_time_s);
}

double Simulation::perception_weight(Vec2 direction, Vec2 stimulus) const noexcept {
    const double stimulus_norm = norm(stimulus);
    if (stimulus_norm <= kEpsilon) {
        return 1.0;
    }
    const double threshold = std::cos(interactions_.field_of_view_angle_rad / 2.0);
    return dot(direction, stimulus) >= stimulus_norm * threshold ? 1.0
                                                                  : interactions_.rear_influence_weight;
}

Vec2 Simulation::pair_force(const Agent& subject, const Agent& other) const {
    return evaluate_pair_force(subject, other).force;
}

Simulation::PairForceEvaluation Simulation::evaluate_pair_force(const Agent& subject, const Agent& other) const {
    if (interactions_.pair_potential_amplitude_m2_per_s2 == 0.0) return {};
    // Choose one image and use it throughout the elliptical-distance gradient.
    const Vec2 r = minimum_image_displacement(subject.state().position - other.state().position, domain_);
    if (r.x == 0.0 && r.y == 0.0) {
        throw std::domain_error("coincident positions in pair interaction between agents " +
                                std::to_string(subject.state().id) + " and " + std::to_string(other.state().id));
    }
    const double stride = std::hypot(other.state().velocity.x, other.state().velocity.y) *
                          interactions_.anticipation_time_s;
    double b{};
    Vec2 gradient_b{};
    bool used_tie_break = false;
    if (stride == 0.0) {
        b = std::hypot(r.x, r.y);
        gradient_b = r * (1.0 / b);
    } else {
        const Vec2 direction = desired_direction(other);
        if (direction.x == 0.0 && direction.y == 0.0) {
            throw std::domain_error("moving pedestrian has no desired direction in pair interaction");
        }
        const Vec2 normal{-direction.y, direction.x};
        const double x = dot(r, direction);
        const double y = dot(r, normal);
        const double remaining = stride - x;
        const double distance = std::hypot(x, y);
        const double anticipated_distance = std::hypot(remaining, y);
        if (distance == 0.0 || anticipated_distance == 0.0) {
            throw std::domain_error("singular anticipation endpoint in pair interaction between agents " +
                                    std::to_string(subject.state().id) + " and " + std::to_string(other.state().id));
        }
        const double sum = distance + anticipated_distance;
        const double coefficient = 1.0 / (distance + std::abs(x)) +
                                   1.0 / (anticipated_distance + std::abs(remaining));
        // Rationalise (distance + anticipated_distance - stride) to avoid
        // subtracting nearly equal numbers as the ellipse collapses.
        const double transverse_scale = 0.5 * std::sqrt((sum + stride) * coefficient);
        if (x > 0.0 && remaining > 0.0) {
            b = std::abs(y) * transverse_scale;
            if (y == 0.0) {
                // One-sided limit inside the segment, with global +y preferred.
                Vec2 upward = normal;
                if (upward.y < 0.0 || (upward.y == 0.0 && upward.x < 0.0)) upward = -upward;
                gradient_b = (stride / (2.0 * std::sqrt(x) * std::sqrt(remaining))) * upward;
                used_tie_break = true;
            } else {
                // Algebraically equivalent gradient without cancellation or 1/|y|.
                const double factor = sum / (4.0 * transverse_scale);
                const double longitudinal = factor * std::abs(y) * (x - remaining) * stride /
                    (distance * anticipated_distance * (x * anticipated_distance + remaining * distance));
                const double transverse = std::copysign(factor * (1.0 / distance + 1.0 / anticipated_distance), y);
                gradient_b = longitudinal * direction + transverse * normal;
            }
        } else {
            const double gap = 2.0 * std::max(-x, -remaining);
            b = std::hypot(0.5 * std::sqrt((sum + stride) * gap), std::abs(y) * transverse_scale);
            const double factor = sum / (4.0 * b);
            gradient_b = (factor * (x / distance - remaining / anticipated_distance)) * direction +
                         (factor * (y / distance + y / anticipated_distance)) * normal;
        }
    }
    const double potential = interactions_.pair_potential_amplitude_m2_per_s2 *
                             std::exp(-b / interactions_.pair_range_m);
    const Vec2 force = (potential / interactions_.pair_range_m) * gradient_b;
    if (!std::isfinite(force.x) || !std::isfinite(force.y)) {
        throw std::domain_error("nonfinite pair force in close encounter");
    }
    return {perception_weight(desired_direction(subject), -force) * force, used_tie_break};
}

Vec2 Simulation::wall_force(const Agent& agent) const noexcept {
    const Vec2 p = agent.state().position;
    struct Candidate { double distance; Vec2 outward; };
    Candidate nearest{};
    bool has_wall = false;
    const auto consider = [&nearest, &has_wall](Candidate candidate) {
        if (!has_wall || candidate.distance < nearest.distance) {
            nearest = candidate;
            has_wall = true;
        }
    };
    if (domain_.boundary_x == BoundaryX::walls) {
        consider({p.x - domain_.min_x, {1.0, 0.0}});
        consider({domain_.max_x - p.x, {-1.0, 0.0}});
    }
    if (domain_.boundary_y == BoundaryY::walls) {
        if (domain_.wall_bottom) consider({p.y - domain_.min_y, {0.0, 1.0}});
        if (domain_.wall_top) consider({domain_.max_y - p.y, {0.0, -1.0}});
    }
    if (!has_wall) {
        return {};
    }
    const double distance = std::max(0.0, nearest.distance);
    const double magnitude = interactions_.wall_potential_amplitude_m2_per_s2 /
                             interactions_.wall_range_m * std::exp(-distance / interactions_.wall_range_m);
    return magnitude * nearest.outward;
}

Vec2 Simulation::realized_velocity(Vec2 preferred_velocity, double maximum_speed) const noexcept {
    const double preferred_speed = norm(preferred_velocity);
    if (preferred_speed <= kEpsilon || maximum_speed <= 0.0) {
        return {};
    }
    return preferred_velocity * std::min(1.0, maximum_speed / preferred_speed);
}

void Simulation::step(double time_step_s) {
    if (time_step_s <= 0.0) {
        throw std::invalid_argument("time step must be positive");
    }
    std::vector<Vec2> total_forces(agents_.size());
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        total_forces[i] = driving_force(agents_[i]) + wall_force(agents_[i]);
        for (std::size_t j = 0; j < agents_.size(); ++j) {
            if (i != j) {
                const auto evaluation = evaluate_pair_force(agents_[i], agents_[j]);
                total_forces[i] += evaluation.force;
                ++encounter_statistics_.pair_force_evaluations;
                if (evaluation.used_upward_tie_break) ++encounter_statistics_.upward_tie_breaks;
            }
        }
    }
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        AgentState& state = agents_[i].state();
        state.preferred_velocity += time_step_s * total_forces[i];
        state.velocity = realized_velocity(
            state.preferred_velocity, state.maximum_speed_factor * state.desired_speed_m_per_s);
        state.position += time_step_s * state.velocity;
        state.position = wrap_position(state.position, domain_);
    }
}

}  // namespace sfm
