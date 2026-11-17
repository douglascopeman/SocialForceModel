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
#include "sfm/geometry.hpp"

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
        config.domain.boundary_x = open ? BoundaryX::open : BoundaryX::walls;
    } else if (full_key == "domain.boundary_x") {
        const auto boundary = unquote(value);
        if (boundary == "open") config.domain.boundary_x = BoundaryX::open;
        else if (boundary == "walls") config.domain.boundary_x = BoundaryX::walls;
        else if (boundary == "periodic") config.domain.boundary_x = BoundaryX::periodic;
        else throw std::runtime_error("domain.boundary_x must be open, walls, or periodic");
    } else if (full_key == "domain.boundary_y") {
        const auto boundary = unquote(value);
        if (boundary == "walls") config.domain.boundary_y = BoundaryY::walls;
        else if (boundary == "periodic") config.domain.boundary_y = BoundaryY::periodic;
        else throw std::runtime_error("domain.boundary_y must be walls or periodic");
    } else if (full_key == "agents.count") config.agent_count = static_cast<int>(as_double(value, full_key));
    else if (full_key == "agents.initial_placement") config.initial_placement = unquote(value);
    else if (full_key == "agents.initial_minimum_separation_m") config.initial_minimum_separation_m = as_double(value, full_key);
    else if (full_key == "agents.desired_speed_mean_m_per_s" || full_key == "agents.desired_speed_m_per_s") config.desired_speed_mean_m_per_s = as_double(value, full_key);
    else if (full_key == "agents.desired_speed_stddev_m_per_s") config.desired_speed_stddev_m_per_s = as_double(value, full_key);
    else if (full_key == "agents.relaxation_time_s") config.relaxation_time_s = as_double(value, full_key);
    else if (full_key == "agents.maximum_speed_factor") config.maximum_speed_factor = as_double(value, full_key);
    else if (full_key == "output.trajectory_interval_s") config.trajectory_interval_s = as_double(value, full_key);
    else if (full_key == "output.render_interval_s") config.render_interval_s = as_double(value, full_key);
    else if (full_key == "output.directory") config.output_directory = unquote(value);
    else if (full_key == "scenario.type") config.scenario = unquote(value);
    else if (full_key == "scenario.direction_mode") {
        const std::string mode = unquote(value);
        if (mode == "goal_point") config.direction_mode = DirectionMode::goal_point;
        else if (mode == "fixed_vector") config.direction_mode = DirectionMode::fixed_vector;
        else throw std::runtime_error("scenario.direction_mode must be goal_point or fixed_vector");
    }
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
    if (config.scenario != "bidirectional" && config.scenario != "unidirectional") {
        throw std::runtime_error("scenario.type must be bidirectional or unidirectional");
    }
    if (config.initial_placement != "end_groups" && config.initial_placement != "uniform") {
        throw std::runtime_error("agents.initial_placement must be end_groups or uniform");
    }
    const double minimum_height = config.domain.boundary_y == BoundaryY::periodic ? 0.0 : 1.0;
    if (config.domain.max_x <= config.domain.min_x || config.domain.max_y - config.domain.min_y <= minimum_height ||
        config.initial_minimum_separation_m < 0.0) {
        throw std::runtime_error("initial placement requires positive extents, space for wall margins and nonnegative separation");
    }
    if ((config.domain.boundary_x == BoundaryX::periodic || config.domain.boundary_y == BoundaryY::periodic) &&
        config.direction_mode != DirectionMode::fixed_vector) {
        throw std::runtime_error("periodic boundaries require scenario.direction_mode = fixed_vector");
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
    constexpr double margin = 30.0;
    constexpr double plot_top = 70.0;
    const double scale_x = (width - 2.0 * margin) / (config.domain.max_x - config.domain.min_x);
    const double scale_y = scale_x;
    const double plot_height = (config.domain.max_y - config.domain.min_y) * scale_y;
    const double height = plot_top + plot_height + margin;
    std::ofstream svg(path);
    if (!svg) throw std::runtime_error("cannot write " + path.string());
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"960\" height=\"" << height
        << "\" viewBox=\"0 0 960 " << height << "\">\n"
        << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n"
        << "<rect x=\"" << margin << "\" y=\"" << plot_top << "\" width=\"" << width - 2.0 * margin
        << "\" height=\"" << plot_height << "\" fill=\"#f8fafc\"/>\n";
    for (double y : {plot_top, plot_top + plot_height}) {
        svg << "<line x1=\"30\" x2=\"930\" y1=\"" << y << "\" y2=\"" << y
            << "\" stroke=\"#334155\" stroke-width=\"2\""
            << (config.domain.boundary_y == BoundaryY::periodic ? " stroke-dasharray=\"6 5\"" : "") << "/>\n";
    }
    for (double x : {margin, width - margin}) {
        svg << "<line x1=\"" << x << "\" x2=\"" << x << "\" y1=\"" << plot_top
            << "\" y2=\"" << plot_top + plot_height << "\" stroke=\"#64748b\""
            << (config.domain.boundary_x == BoundaryX::walls ? "" : " stroke-dasharray=\"6 5\"") << "/>\n";
    }
    for (const Agent& agent : simulation.agents()) {
        const AgentState& state = agent.state();
        const double x = margin + (state.position.x - config.domain.min_x) * scale_x;
        const double y = height - margin - (state.position.y - config.domain.min_y) * scale_y;
        const bool rightward = desired_direction(state).x > 0.0;
        svg << "<circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"5\" fill=\""
            << (rightward ? "#2563eb" : "#dc2626") << "\"/>\n";
        svg << "<line x1=\"" << x << "\" y1=\"" << y << "\" x2=\"" << x + 0.3 * state.velocity.x * scale_x
            << "\" y2=\"" << y - 0.3 * state.velocity.y * scale_y
            << "\" stroke=\"" << (rightward ? "#2563eb" : "#dc2626") << "\" stroke-width=\"2\"/>\n";
    }
    svg << "<text x=\"30\" y=\"25\" font-family=\"sans-serif\" font-size=\"16\" fill=\"#0f172a\">"
        << config.name << " at t = " << std::fixed << std::setprecision(1) << time_s
        << " s — blue: right, red: left</text>\n"
        << "<text x=\"30\" y=\"48\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#475569\">"
        << "x boundaries: " << boundary_x_name(config.domain.boundary_x)
        << "; y boundaries: " << boundary_y_name(config.domain.boundary_y)
        << "; lines show velocity (0.3 s scale)</text>\n</svg>\n";
}

}  // namespace

RunConfig load_run_config(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("cannot open configuration " + path.string());
    RunConfig config;
    std::string section;
    bool saw_boundary_x = false;
    bool saw_open_x = false;
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
        const auto key = trim(line.substr(0, separator));
        if (section == "domain" && key == "boundary_x") saw_boundary_x = true;
        if (section == "domain" && key == "open_x") saw_open_x = true;
        if (saw_boundary_x && saw_open_x) {
            throw std::runtime_error("use domain.boundary_x or legacy domain.open_x, not both");
        }
        apply_value(config, section, key, trim(line.substr(separator + 1)));
    }
    validate(config);
    return config;
}

std::vector<Agent> create_population(const RunConfig& config) {
    validate(config);
    std::vector<Agent> population;
    population.reserve(static_cast<std::size_t>(config.agent_count));
    std::mt19937 generator(config.seed);
    std::normal_distribution<double> speed_distribution(
        config.desired_speed_mean_m_per_s, std::max(config.desired_speed_stddev_m_per_s, kEpsilon));
    const double y_margin = config.domain.boundary_y == BoundaryY::periodic ? 0.0 : 0.5;
    std::uniform_real_distribution<double> y_distribution(config.domain.min_y + y_margin, config.domain.max_y - y_margin);
    std::uniform_real_distribution<double> x_offset(0.5, 3.0);
    std::uniform_real_distribution<double> uniform_x(config.domain.min_x, config.domain.max_x);
    for (int index = 0; index < config.agent_count; ++index) {
        const bool rightward = config.scenario == "unidirectional" || index % 2 == 0;
        const double speed = std::max(0.1, config.desired_speed_stddev_m_per_s == 0.0
                                              ? config.desired_speed_mean_m_per_s : speed_distribution(generator));
        double x = rightward ? config.domain.min_x + x_offset(generator)
                                  : config.domain.max_x - x_offset(generator);
        const Vec2 goal{rightward ? config.domain.max_x + 10.0 : config.domain.min_x - 10.0, y_distribution(generator)};
        double y = y_distribution(generator);
        if (config.initial_placement == "uniform") {
            bool placed = false;
            for (int attempt = 0; attempt < 100000 && !placed; ++attempt) {
                x = uniform_x(generator);
                y = y_distribution(generator);
                placed = true;
                for (const auto& other : population) {
                    const Vec2 delta = minimum_image_displacement(Vec2{x, y} - other.state().position, config.domain);
                    if (squared_norm(delta) < config.initial_minimum_separation_m * config.initial_minimum_separation_m) {
                        placed = false;
                        break;
                    }
                }
            }
            if (!placed) throw std::runtime_error("could not place population at requested initial minimum separation");
        }
        population.push_back(Agent({.id = static_cast<std::uint64_t>(index), .position = {x, y},
                                    .goal = goal, .desired_speed_m_per_s = speed,
                                    .relaxation_time_s = config.relaxation_time_s,
                                    .maximum_speed_factor = config.maximum_speed_factor,
                                    .direction_mode = config.direction_mode,
                                    .fixed_direction = {rightward ? 1.0 : -1.0, 0.0}}));
    }
    return population;
}

void run_experiment(const RunConfig& config) {
    validate(config);
    auto population = create_population(config);
    std::filesystem::create_directories(config.output_directory);
    Simulation simulation(config.domain, config.interactions);
    for (auto& agent : population) simulation.add_agent(std::move(agent));

    std::ofstream trajectory(config.output_directory / "trajectory.csv");
    if (!trajectory) throw std::runtime_error("cannot write trajectory.csv");
    trajectory << "time_s,agent_id,x_m,y_m,vx_m_per_s,vy_m_per_s,wx_m_per_s,wy_m_per_s\n";
    trajectory << std::setprecision(12);
    double time = 0.0;
    double next_output = 0.0;
    double next_render = 0.0;
    std::size_t step_index = 0;
    const auto render_frame = [&config, &simulation](double frame_time) {
        std::ostringstream filename;
        filename << "frame_" << std::setw(5) << std::setfill('0')
                 << static_cast<int>(std::lround(frame_time * 1000.0)) << "ms.svg";
        write_svg(config.output_directory / filename.str(), config, simulation, frame_time);
    };
    write_frame(trajectory, time, simulation);
    render_frame(time);
    while (time + kEpsilon < config.duration_s) {
        const double next_time = std::min(static_cast<double>(++step_index) * config.time_step_s, config.duration_s);
        const double dt = next_time - time;
        simulation.step(dt);
        time = next_time;
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
            << "direction_mode: " << (config.direction_mode == DirectionMode::fixed_vector ? "fixed_vector" : "goal_point") << '\n'
            << "boundary_x: " << boundary_x_name(config.domain.boundary_x) << '\n'
            << "boundary_y: " << boundary_y_name(config.domain.boundary_y) << '\n'
            << "initial_placement: " << config.initial_placement << '\n'
            << "domain_width_m: " << config.domain.max_x - config.domain.min_x << '\n'
            << "domain_height_m: " << config.domain.max_y - config.domain.min_y << '\n'
            << "seed: " << config.seed << '\n' << "agents: " << simulation.agent_count() << '\n'
            << "duration_s: " << config.duration_s << '\n' << "time_step_s: " << config.time_step_s << '\n'
            << "pair_force_evaluations: " << simulation.encounter_statistics().pair_force_evaluations << '\n'
            << "upward_tie_breaks: " << simulation.encounter_statistics().upward_tie_breaks << '\n'
            << "trajectory: trajectory.csv\nrender: final_state.svg\n";
}

}  // namespace sfm
