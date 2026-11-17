#include "sfm/field_output.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace sfm {
namespace {

// Extend this table and the estimator moments when adding another observable.
struct FieldColumns {
    MacroscopicField field;
    const char* header;
    void (*write)(std::ostream&, const CoarseField&);
};

constexpr FieldColumns columns[]{
    {MacroscopicField::density, "rho_per_m2", [](std::ostream& out, const CoarseField& f) { out << f.density; }},
    {MacroscopicField::velocity, "u_x_m_per_s,u_y_m_per_s", [](std::ostream& out, const CoarseField& f) {
         out << f.mean_velocity.x << ',' << f.mean_velocity.y;
     }},
    {MacroscopicField::polarisation, "polarisation_x,polarisation_y,polarisation", [](std::ostream& out, const CoarseField& f) {
         out << f.polarisation_vector.x << ',' << f.polarisation_vector.y << ',' << f.polarisation;
     }},
};

const FieldColumns& descriptor(MacroscopicField field) {
    for (const auto& entry : columns) if (entry.field == field) return entry;
    throw std::invalid_argument("missing macroscopic field output descriptor");
}

}  // namespace

FieldOutput::FieldOutput(const std::filesystem::path& run_directory, RectangularDomain domain,
                         CoarseGrainingConfig config)
    : directory_(run_directory / "fields"), config_(std::move(config)), estimator_(domain, config_) {
    std::filesystem::create_directories(directory_);
    index_.exceptions(std::ios::failbit | std::ios::badbit);
    index_.open(directory_ / "windows.csv");
    index_ << std::setprecision(17)
           << "file,time_start_s,time_end_s,duration_s,time_samples,mean_in_domain_agents,mean_excluded_agents,integrated_density\n";
    std::ofstream metadata;
    metadata.exceptions(std::ios::failbit | std::ios::badbit);
    metadata.open(directory_ / "metadata.json");
    metadata << std::setprecision(17)
             << "{\n  \"schema_version\": 1,\n  \"fields\": [";
    for (std::size_t i = 0; i < config_.fields.size(); ++i) {
        metadata << (i ? ", " : "") << '"' << field_name(config_.fields[i]) << '"';
    }
    metadata << "],\n  \"kernel\": \"gaussian\",\n  \"kernel_width_m\": " << config_.kernel_width_m
             << ",\n  \"kernel_cutoff_sigma\": " << config_.kernel_cutoff_sigma
             << ",\n  \"kernel_support\": \"square, normalized before domain clipping\""
             << ",\n  \"grid_spacing_m\": " << config_.grid_spacing_m
             << ",\n  \"window_s\": " << config_.window_s
             << ",\n  \"boundary_policy\": \"" << config_.boundary_policy << '"'
             << ",\n  \"boundary_x\": \"" << boundary_x_name(domain.boundary_x) << '"'
             << ",\n  \"boundary_y\": \"" << boundary_y_name(domain.boundary_y) << '"'
             << ",\n  \"domain_m\": [" << domain.min_x << ", " << domain.max_x << ", " << domain.min_y << ", " << domain.max_y << ']'
             << ",\n  \"domain_membership\": \"half-open on both axes; exclude outside agents, wrap periodic axes\""
             << ",\n  \"spatial_estimator\": \"cell-integrated kernel; final edge cells may be smaller\""
             << ",\n  \"temporal_estimator\": \"non-overlapping duration-weighted windows; integration right endpoints; final partial window retained\""
             << ",\n  \"velocity_source\": \"realised velocity\""
             << ",\n  \"polarisation_definition\": \"weighted mean of unit realised velocities, plus its magnitude; speeds <= 1e-12 m/s contribute zero\""
             << ",\n  \"effective_samples_definition\": \"(sum dt*kernel_mass)^2 / sum (dt*kernel_mass)^2; correlated agent-time contributions, not independent samples\""
             << ",\n  \"sample_count_definition\": \"number of agent-time contributions with nonzero kernel mass\""
             << ",\n  \"empty_cells\": \"zero density and support; NaN conditional fields\""
             << ",\n  \"units\": {\"position\": \"m\", \"time\": \"s\", \"density\": \"m^-2\", \"velocity\": \"m/s\", \"polarisation\": \"1\"}"
             << ",\n  \"window_index\": \"windows.csv is the authoritative list for this run\"\n}\n";
    metadata.close();
}

void FieldOutput::sample(std::span<const Agent> agents, double duration_s) {
    if (!std::isfinite(duration_s) || duration_s <= 0.0) throw std::invalid_argument("field sample duration must be positive and finite");
    double remaining = duration_s;
    while (remaining > 0.0) {
        const double available = config_.window_s - estimator_.duration_s();
        const double segment = std::abs(remaining - available) <= 1e-12 * config_.window_s
            ? remaining : std::min(remaining, available);
        estimator_.accumulate(agents, segment);
        remaining -= segment;
        if (segment >= available || config_.window_s - estimator_.duration_s() <= 1e-12 * config_.window_s) {
            write_window();
        }
    }
}

void FieldOutput::write_window() {
    const double duration = estimator_.duration_s();
    const double in_domain = estimator_.mean_in_domain_agents();
    const double excluded = estimator_.mean_excluded_agents();
    const auto samples = estimator_.time_samples();
    const auto fields = estimator_.finish();
    std::ostringstream filename;
    filename << "window_" << std::setw(6) << std::setfill('0') << window_index_++ << ".csv";
    std::ofstream out;
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out.open(directory_ / filename.str());
    out << std::setprecision(17)
        << "time_start_s,time_end_s,ix,iy,x_m,y_m,cell_width_m,cell_height_m,sample_count,effective_samples";
    for (auto field : config_.fields) out << ',' << descriptor(field).header;
    out << '\n';
    double integrated_density = 0.0;
    for (std::size_t i = 0; i < fields.size(); ++i) {
        const auto& cell = estimator_.cells()[i];
        const auto& field = fields[i];
        out << window_start_s_ << ',' << window_start_s_ + duration << ',' << cell.ix << ',' << cell.iy << ','
            << cell.centre.x << ',' << cell.centre.y << ',' << cell.width_m << ',' << cell.height_m << ','
            << field.sample_count << ',' << field.effective_samples;
        for (auto selected : config_.fields) {
            out << ',';
            descriptor(selected).write(out, field);
        }
        out << '\n';
        integrated_density += field.density * cell.width_m * cell.height_m;
    }
    out.close();
    index_ << filename.str() << ',' << window_start_s_ << ',' << window_start_s_ + duration << ',' << duration
           << ',' << samples << ',' << in_domain << ',' << excluded << ',' << integrated_density << '\n';
    index_.flush();
    window_start_s_ += duration;
}

void FieldOutput::finish() {
    if (estimator_.duration_s() > 0.0) write_window();
    index_.flush();
}

}  // namespace sfm
