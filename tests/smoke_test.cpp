#include "sfm/simulation.hpp"

int main() {
    sfm::Simulation simulation;
    simulation.add_agent(sfm::Agent({.id = 1}));
    return simulation.agent_count() == 1 ? 0 : 1;
}
