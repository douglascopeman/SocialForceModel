#include "sfm/simulation.hpp"

#include <utility>

namespace sfm {

void Simulation::add_agent(Agent agent) { agents_.push_back(std::move(agent)); }

std::size_t Simulation::agent_count() const noexcept { return agents_.size(); }

}  // namespace sfm
