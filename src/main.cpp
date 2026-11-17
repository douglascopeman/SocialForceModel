#include <exception>
#include <iostream>

#include "sfm/run.hpp"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: sfm_sim <configuration.toml>\n";
        return 2;
    }
    try {
        const sfm::RunConfig config = sfm::load_run_config(argv[1]);
        sfm::run_experiment(config);
        std::cout << "Completed " << config.name << "; output: " << config.output_directory << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "sfm_sim: " << error.what() << '\n';
        return 1;
    }
}
