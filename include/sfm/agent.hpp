#pragma once

#include "sfm/types.hpp"

namespace sfm {

class Agent {
  public:
    explicit Agent(AgentState state);
    [[nodiscard]] const AgentState& state() const noexcept;

  private:
    AgentState state_;
};

}  // namespace sfm
