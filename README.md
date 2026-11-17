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

The configuration reader intentionally supports only the scalar TOML fields
used by the supplied example configurations. Unsupported keys are ignored at
this stage; a full TOML library or schema validation is a later engineering
decision.
