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

The executable currently establishes the application seam only. Numerical
implementation begins with the validated force law and integrator milestone.
