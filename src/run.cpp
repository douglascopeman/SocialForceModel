#include "sfm/run.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <vector>

#include "sfm/simulation.hpp"

namespace sfm {
namespace {

std::string trim(std::string value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string unquote(std::string value) {
    value = trim(std::move(value));
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
        return value.substr(1, value.size() - 2);
    }
    return value;
}

double as_double(const std::string& value, std::string_view key) {
    try {
        return std::stod(value);
    } catch (const std::exception&) {
        throw std::runtime_error("invalid number for " + std::string(key));
    }
}

bool as_bool(const std::string& value, std::string_view key) {
    if (value == "true") return true;
    if (value == "false") return false;
    throw std::runtime_error("invalid Boolean for " + std::string(key));
}

void apply_value(RunConfig& config, const std::string& section, const std::string& key,
                 const std::string& value) {
    const std::string full_key = section.empty() ? key : section + "." + key;
    if (full_key == "run.name") config.name = unquote(value);
    else if (full_key == "run.seed") config.seed = static_cast<unsigned int>(as_double(value, full_key));
    else if (full_key == "run.duration_s") config.duration_s = as_double(value, full_key);
    else if (full_key == "run.time_step_s") config.time_step_s = as_double(value, full_key);
    else if (full_key == "domain.width_m") config.domain.max_x = config.domain.min_x + as_double(value, full_key);
    else if (full_key == "domain.height_m") config.domain.max_y = config.domain.min_y + as_double(value, full_key);
    else if (full_key == "domain.open_x") {
        const bool open = as_bool(value, full_key);
        config.domain.wall_left = !open;
        config.domain.wall_right = !open;
    } else if (full_key == "agents.count") config.agent_count = static_cast<int>(as_double(value, full_key));
    else if (full_key == "agents.desired_speed_mean_m_per_s" || full_key == "agents.desired_speed_m_per_s") config.desired_speed_mean_m_per_s = as_double(value, full_key);
    else if (full_key == "agents.desired_speed_stddev_m_per_s") config.desired_speed_stddev_m_per_s = as_double(value, full_key);
    else if (full_key == "agents.relaxation_time_s") config.relaxation_time_s = as_double(value, full_key);
    else if (full_key == "agents.maximum_speed_factor") config.maximum_speed_factor = as_double(value, full_key);
    else if (full_key == "output.trajectory_interval_s") config.trajectory_interval_s = as_double(value, full_key);
    else if (full_key == "output.render_interval_s") config.render_interval_s = as_double(value, full_key);
    else if (full_key == "output.directory") config.output_directory = unquote(value);
    else if (full_key == "scenario.type") config.scenario = unquote(value);
    else if (full_key == "interactions.pair_potential_amplitude_m2_per_s2") config.interactions.pair_potential_amplitude_m2_per_s2 = as_double(value, full_key);
    else if (full_key == "interactions.pair_range_m") config.interactions.pair_range_m = as_double(value, full_key);
    else if (full_key == "interactions.anticipation_time_s") config.interactions.anticipation_time_s = as_double(value, full_key);
    else if (full_key == "interactions.wall_potential_amplitude_m2_per_s2") config.interactions.wall_potential_amplitude_m2_per_s2 = as_double(value, full_key);
    else if (full_key == "interactions.wall_range_m") config.interactions.wall_range_m = as_double(value, full_key);
    else if (full_key == "interactions.field_of_view_angle_deg") config.interactions.field_of_view_angle_rad = as_double(value, full_key) * 3.14159265358979323846 / 180.0;
    else if (full_key == "interactions.rear_influence_weight") config.interactions.rear_influence_weight = as_double(value, full_key);
}

void validate(const RunConfig& config) {
    if (config.duration_s <= 0.0 || config.time_step_s <= 0.0 || config.trajectory_interval_s <= 0.0 ||
        config.render_interval_s <= 0.0 ||
        config.agent_count <= 0 || config.desired_speed_mean_m_per_s <= 0.0 ||
        config.desired_speed_stddev_m_per_s < 0.0 || config.relaxation_time_s <= 0.0 ||
        config.maximum_speed_factor <= 0.0) {
        throw std::runtime_error("run configuration contains a non-positive required value");
    }
    if (config.scenario != "bidirectional") {
        throw std::runtime_error("only scenario.type = \"bidirectional\" is currently supported");
    }
}

void write_frame(std::ofstream& stream, double time, const Simulation& simulation) {
    for (const Agent& agent : simulation.agents()) {
        const AgentState& state = agent.state();
        stream << time << ',' << state.id << ',' << state.position.x << ',' << state.position.y << ','
               << state.velocity.x << ',' << state.velocity.y << ',' << state.preferred_velocity.x << ','
               << state.preferred_velocity.y << '\n';
    }
}

void write_svg(const std::filesystem::path& path, const RunConfig& config, const Simulation& simulation,
               double time_s) {
    constexpr double width = 960.0;
    constexpr double height = 480.0;
    constexpr double margin = 30.0;
    const double scale_x = (width - 2.0 * margin) / (config.domain.max_x - config.domain.min_x);
    const double scale_y = (height - 2.0 * margin) / (config.domain.max_y - config.domain.min_y);
    std::ofstream svg(path);
    if (!svg) throw std::runtime_error("cannot write " + path.string());
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"960\" height=\"480\" viewBox=\"0 0 960 480\">\n"
        << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n"
        << "<rect x=\"" << margin << "\" y=\"" << margin << "\" width=\"" << width - 2.0 * margin
        << "\" height=\"" << height - 2.0 * margin << "\" fill=\"#f8fafc\" stroke=\"#334155\"/>\n";
    for (const Agent& agent : simulation.agents()) {
        const AgentState& state = agent.state();
        const double x = margin + (state.position.x - config.domain.min_x) * scale_x;
        const double y = height - margin - (state.position.y - config.domain.min_y) * scale_y;
        const bool rightward = state.goal.x > state.position.x;
        svg << "<circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"5\" fill=\""
            << (rightward ? "#2563eb" : "#dc2626") << "\"/>\n";
    }
    svg << "<text x=\"36\" y=\"52\" font-family=\"sans-serif\" font-size=\"16\" fill=\"#0f172a\">"
        << config.name << " at t = " << std::fixed << std::setprecision(1) << time_s
        << " s — blue: right, red: left</text>\n</svg>\n";
}

}  // namespace

RunConfig load_run_config(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("cannot open configuration " + path.string());
    RunConfig config;
    std::string section;
    std::string line;
    int line_number = 0;
    while (std::getline(stream, line)) {
        ++line_number;
        const auto comment = line.find('#');
        if (comment != std::string::npos) line.erase(comment);
        line = trim(std::move(line));
        if (line.empty()) continue;
        if (line.front() == '[' && line.back() == ']') {
            section = line.substr(1, line.size() - 2);
            continue;
        }
        const auto separator = line.find('=');
        if (separator == std::string::npos) {
            throw std::runtime_error("invalid configuration at line " + std::to_string(line_number));
        }
        apply_value(config, section, trim(line.substr(0, separator)), trim(line.substr(separator + 1)));
    }
    validate(config);
    return config;
}

void run_experiment(const RunConfig& config) {
    validate(config);
    std::filesystem::create_directories(config.output_directory);
    Simulation simulation(config.domain, config.interactions);
    std::mt19937 generator(config.seed);
    std::normal_distribution<double> speed_distribution(
        config.desired_speed_mean_m_per_s, config.desired_speed_stddev_m_per_s);
    std::uniform_real_distribution<double> y_distribution(config.domain.min_y + 0.5, config.domain.max_y - 0.5);
    std::uniform_real_distribution<double> x_offset(0.5, 3.0);
    for (int index = 0; index < config.agent_count; ++index) {
        const bool rightward = index % 2 == 0;
        const double speed = std::max(0.1, speed_distribution(generator));
        const double x = rightward ? config.domain.min_x + x_offset(generator)
                                  : config.domain.max_x - x_offset(generator);
        const Vec2 goal{rightward ? config.domain.max_x + 10.0 : config.domain.min_x - 10.0, y_distribution(generator)};
        simulation.add_agent(Agent({.id = static_cast<std::uint64_t>(index), .position = {x, y_distribution(generator)},
                                    .goal = goal, .desired_speed_m_per_s = speed,
                                    .relaxation_time_s = config.relaxation_time_s,
                                    .maximum_speed_factor = config.maximum_speed_factor}));
    }

    std::ofstream trajectory(config.output_directory / "trajectory.csv");
    if (!trajectory) throw std::runtime_error("cannot write trajectory.csv");
    trajectory << "time_s,agent_id,x_m,y_m,vx_m_per_s,vy_m_per_s,wx_m_per_s,wy_m_per_s\n";
    trajectory << std::setprecision(12);
    double time = 0.0;
    double next_output = 0.0;
    double next_render = 0.0;
    const auto render_frame = [&config, &simulation](double frame_time) {
        std::ostringstream filename;
        filename << "frame_" << std::setw(5) << std::setfill('0')
                 << static_cast<int>(std::lround(frame_time * 1000.0)) << "ms.svg";
        write_svg(config.output_directory / filename.str(), config, simulation, frame_time);
    };
    write_frame(trajectory, time, simulation);
    render_frame(time);
    while (time + kEpsilon < config.duration_s) {
        const double dt = std::min(config.time_step_s, config.duration_s - time);
        simulation.step(dt);
        time += dt;
        if (time + kEpsilon >= next_output + config.trajectory_interval_s) {
            next_output += config.trajectory_interval_s;
            write_frame(trajectory, time, simulation);
        }
        if (time + kEpsilon >= next_render + config.render_interval_s) {
            next_render += config.render_interval_s;
            render_frame(time);
        }
    }
    write_svg(config.output_directory / "final_state.svg", config, simulation, time);
    std::ofstream summary(config.output_directory / "summary.txt");
    if (!summary) throw std::runtime_error("cannot write summary.txt");
    summary << "name: " << config.name << '\n' << "scenario: " << config.scenario << '\n'
            << "seed: " << config.seed << '\n' << "agents: " << simulation.agent_count() << '\n'
            << "duration_s: " << config.duration_s << '\n' << "time_step_s: " << config.time_step_s << '\n'
            << "trajectory: trajectory.csv\nrender: final_state.svg\n";
}

}  // namespace sfm
