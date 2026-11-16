#include <iostream>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: sfm_sim <configuration.toml>\n";
        return 2;
    }
    std::cout << "Social Force Model scaffold; configuration: " << argv[1] << '\n';
    return 0;
}
