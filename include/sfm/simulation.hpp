#pragma once

#include <vector>

#include "sfm/agent.hpp"

namespace sfm {

class Simulation {
  public:
    void add_agent(Agent agent);
    [[nodiscard]] std::size_t agent_count() const noexcept;

  private:
    std::vector<Agent> agents_;
};

}  // namespace sfm
