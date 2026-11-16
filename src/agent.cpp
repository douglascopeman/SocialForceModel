#include "sfm/agent.hpp"

namespace sfm {

Agent::Agent(AgentState state) : state_(state) {}

const AgentState& Agent::state() const noexcept { return state_; }

}  // namespace sfm
