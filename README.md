# Social Force Model

A C++ codebase for simulating pedestrian motion with a Social Force Model,
coarse-graining the microscopic trajectories into continuum fields, and testing
whether a local hydrodynamic description is admissible. The project reports
admissibility; it does not attempt to infer a constitutive hydrodynamic model.

## Scope

1. Simulate particles/agents under configurable social, contact, and boundary
   forces.
2. Produce reproducible trajectories and microscopic diagnostics.
3. Estimate density, mean velocity, momentum flux, and fluctuation fields on a
   spatial-temporal grid using a documented smoothing kernel.
4. Evaluate objective closure and scale-separation diagnostics, then label each
   field region as admissible, indeterminate, or inadmissible.

See [the project plan](docs/project-plan.md) for milestones and
[the data contract](docs/data-contract.md) for intended outputs.

## Layout

```
config/       Versioned experiment configurations
docs/         Scientific decisions, plan, and data formats
include/sfm/  Public C++ interfaces
src/          Implementation and command-line entry point
tests/        Automated tests
results/      Ignored generated run artefacts
```

## Build

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/sfm_sim config/example.toml
```

## Run a visual smoke experiment

```bash
./build/sfm_sim config/bidirectional_smoke.toml
```

This runs a deterministic 40-agent, bidirectional open-channel scenario and
writes the following ignored artefacts to `results/bidirectional-smoke/`:

- `trajectory.csv` — sampled microscopic state (`x`, `y`, realized velocity,
  and preferred velocity) for every agent;
- `summary.txt` — run identity and numerical settings;
- `final_state.svg` — simple final-state render (blue agents target right; red
  agents target left).
- `frame_*.svg` — timestamped renders at the configured output interval.

The configuration reader supports the scalar TOML fields used by the examples
and a single-line string array for coarse-grained field selection. Unknown
`[coarse_graining]` keys are rejected; unsupported keys elsewhere are ignored.
A full TOML library or schema validation is a later engineering decision.

## Dynamic macroscopic fields

Add a `[coarse_graining]` section to compute and save fields during a run:

```toml
[coarse_graining]
enabled = true # Defaults to false.
fields = ["density", "velocity", "polarisation"]
grid_spacing_m = 0.5
kernel = "gaussian"
kernel_width_m = 0.5 # Standard deviation on each axis.
kernel_cutoff_sigma = 3.0
window_s = 1.0 # Non-overlapping duration-weighted windows.
boundary_policy = "renormalize" # Or "truncate" at nonperiodic edges.
```

Any nonempty subset of fields can be selected. Velocity uses realised motion;
polarisation is the weighted mean unit-velocity vector and its magnitude.
Output goes to `fields/`, with one CSV per window, a `windows.csv` index and
`metadata.json`. Empty conditional fields are `nan`; support columns accompany
every cell. Enabling fields leaves microscopic trajectories unchanged.

```bash
./build/sfm_sim config/macroscopic_smoke.toml
```

See [the estimator and output conventions](docs/coarse-graining.md) for kernel
integration, periodic and physical boundaries, conservation, and support limits.

## Direction objective

Set the direction mode in the configuration's existing `[scenario]` section:

```toml
[scenario]
type = "bidirectional"
direction_mode = "fixed_vector" # Or "goal_point" (the default when omitted).
```

`goal_point` steers each agent toward its assigned point beyond the channel end,
recalculating its desired unit vector as it moves. `fixed_vector` assigns constant
horizontal desired directions `(1, 0)` and `(-1, 0)` to the opposing populations.
Interactions can still deflect their actual velocities. Both perception weighting
and the elliptical anticipation calculation use the selected desired direction.
Runs record the mode in `summary.txt`. Switching modes preserves the generated
initial population for a given seed so that trajectories can be compared.

For a separate comparison run without overwriting the original outputs:

```bash
./build/sfm_sim config/bidirectional_fixed_vector_smoke.toml
```

Outputs go to `results/bidirectional-fixed-vector-smoke/`.

## Periodic channel and animation

Choose x boundaries in `[domain]` with `boundary_x = "open"`, `"walls"`, or
`"periodic"`. Periodic boundaries wrap x into the domain after each step and use
the nearest periodic image for pair interactions, including the elliptical
anticipation term. Identity and velocity persist through a crossing; the agent
count stays constant. Top and bottom retain soft repulsive walls by default;
`boundary_y = "periodic"` connects those edges and removes their wall forces.
The periodic domain uses a single nearest image, so choose a channel length
well above interaction and anticipation distances.

Periodic runs currently require `direction_mode = "fixed_vector"`; point-goal
steering with periodic geometry requires a further goal convention. Legacy
`open_x = true/false` maps to open/walls; setting both boundary keys is rejected.

The longer example uses 40 pedestrians in a 20 m by 6 m domain for 60 s, at
`dt = 0.01 s`, saving trajectory/render samples every 0.2 s:

```bash
./build/sfm_sim config/periodic_counterflow.toml
python3 scripts/check_periodic_run.py results/periodic-counterflow --length 20 --agents 40
python3 scripts/animate_run.py results/periodic-counterflow --fps 20
```

The animation helper requires Python 3, `rsvg-convert`, and FFmpeg with an H.264
encoder. It creates `animation_frames/*.png`, `evolution.mp4`, and `evolution.gif`
inside the run directory. At 20 fps, 0.2 s simulation samples play at 4x speed.
SVG/PNG frames use equal spatial scale on both axes. Dashed vertical edges mark
the periodic join; colours indicate desired direction and short lines show
realized velocity. The original end-group initialisation is retained, so this
run begins with a transient concentration near the join.

`validation.json` checks finite states, wrapped x coordinates, fixed population
and identities, and reports crossings inferred from sampled jumps. Renderings
and these checks provide a qualitative boundary-condition smoke test;
hydrodynamic admissibility and timestep convergence of this interacting case
remain separate scientific checks.

## Single-direction flow with periodic x and y

```toml
[domain]
boundary_x = "periodic"
boundary_y = "periodic" # Default: "walls".

[scenario]
type = "unidirectional" # Every desired direction is (1, 0).
direction_mode = "fixed_vector"
```

The example configuration uses 80 pedestrians in a 20 m by 10 m periodic
domain for 60 s, with uniform initial placement and no y wall margin:

```bash
./build/sfm_sim config/unidirectional_periodic.toml
python3 scripts/check_periodic_run.py results/unidirectional-periodic --length 20 --height 10 --agents 80
python3 scripts/animate_run.py results/unidirectional-periodic --fps 10
```

The renderer marks all four joins with dashed lines. Wrapping preserves agent
identity and velocity, while pair forces and the initial minimum separation use
nearest periodic images in both axes. The default scenario remains bidirectional.
Desired-speed dispersion remains configurable; set it to zero for an
identical-speed control. The lane-analysis script is specific to balanced
bidirectional populations and should not be used for this single-direction case.

## Close encounters

Exact collinear ties inside another pedestrian's anticipation segment choose
the upward one-sided force limit. Nearby nonzero offsets retain their own force
direction; the potential is evaluated without the old cancellation-induced
zero-force region. Exact coincident positions and anticipation endpoints raise
an explicit error. Each run's `summary.txt` records `pair_force_evaluations` and
`upward_tie_breaks`; diagnostic force queries do not increment these counters.
See [the convention and frequency-check configuration](docs/close-encounters.md)
for details and reproduction instructions.

## Lane-formation experiments

`config/lanes/` contains density comparisons and a shorter-anticipation example.
These start from a mixed population throughout the periodic channel, selected by:

```toml
[agents]
initial_placement = "uniform" # Default is the original "end_groups".
initial_minimum_separation_m = 0.4
```

Uniform placement rejects positions closer than the requested distance to any
existing agent, including across the periodic join. It does not assign lanes.
The run fails explicitly if the requested population cannot be placed.

To reproduce the promising 80-agent experiment:

```bash
./build/sfm_sim config/lanes/n080-short-anticipation.toml
python3 scripts/analyse_lanes.py results/lanes-n080-short-anticipation
python3 scripts/animate_run.py results/lanes-n080-short-anticipation --fps 20
```

This run changes anticipation time to 0.5 s (the paper uses 2 s) and timestep
to 0.002 s; it runs for 180 s. See [the experiment notes](docs/lane-experiments.md)
for comparison settings, the exploratory segregation metric and limitations.
