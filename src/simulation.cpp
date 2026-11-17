#include "sfm/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
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

Vec2 Simulation::pair_force(const Agent& subject, const Agent& other) const noexcept {
    // Choose one image and use it throughout the elliptical-distance gradient.
    const Vec2 r = minimum_image_displacement(subject.state().position - other.state().position, domain_);
    const double distance = norm(r);
    if (distance <= kEpsilon) {
        return {};
    }
    const Vec2 other_direction = desired_direction(other);
    const double stride = norm(other.state().velocity) * interactions_.anticipation_time_s;
    const Vec2 anticipated_displacement = r - stride * other_direction;
    const double anticipated_distance = norm(anticipated_displacement);
    const double sum_distance = distance + anticipated_distance;
    const double radicand = sum_distance * sum_distance - stride * stride;
    if (radicand <= kEpsilon) {
        return {};
    }
    const double b = 0.5 * std::sqrt(radicand);
    if (b <= kEpsilon || anticipated_distance <= kEpsilon) {
        return {};
    }

    // Gradient of b from eq. (4), then -grad[V0 exp(-b/sigma)] from eq. (3).
    const Vec2 gradient_b = (sum_distance / (4.0 * b)) *
                            (r * (1.0 / distance) + anticipated_displacement * (1.0 / anticipated_distance));
    const double potential = interactions_.pair_potential_amplitude_m2_per_s2 *
                             std::exp(-b / interactions_.pair_range_m);
    const Vec2 force = (potential / interactions_.pair_range_m) * gradient_b;
    return perception_weight(desired_direction(subject), -force) * force;
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
                total_forces[i] += pair_force(agents_[i], agents_[j]);
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
