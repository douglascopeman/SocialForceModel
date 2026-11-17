# Project plan

## Research question

Given an ensemble of Social Force Model trajectories, can a chosen spatial and
temporal coarse-graining scale support a *local* hydrodynamic description? The
decision is about existence/admissibility, not the specific continuum equations.

## Scientific decisions to freeze first

- The source-model traceability is recorded in `docs/paper-notes.md`. Implement
  its preferred-velocity equations (1)–(13) as the first model variant; any
  acceleration-only simplification or contact-force extension is a distinct,
  documented variant.
- Define the domain, obstacle representation, entrances/exits, and initial and
  desired-velocity distributions.
- State the candidate conserved/slow fields: at minimum number density and
  momentum density; define whether agent mass is unity.
- Choose smoothing kernel, grid spacing, temporal window, and boundary handling.
- Pre-register diagnostic thresholds and confidence/replicate requirements.

## Milestones

### M0 — Reproducible foundation

Deliver CMake build, compiler warnings/sanitizers in CI, deterministic random
seeds, structured configuration, run metadata, and a trajectory schema.

### M1 — Microscopic model validation

Implement agent state, desired-motion term, inter-agent force, wall/contact
force, and a stable integrator. Test force symmetry where applicable, isolated
relaxation, two-agent interactions, wall avoidance, and timestep convergence.

### M2 — Experiment engine

Implement scenario loading, ensemble runs, periodic/checkpoint output, and
reference cases (free flow, counterflow, bottleneck, dense crowd). Archive every
run's configuration, seed, build revision, and units.

### M3 — Coarse-grained fields

For kernel W_h, estimate

`rho(x,t) = sum_i W_h(x-x_i)` and
`u(x,t) = [sum_i v_i W_h(x-x_i)] / rho(x,t)`.

Also output momentum density, velocity covariance/granular stress proxy, sample
count/effective sample size, and kernel/boundary metadata. Verify conservation
against particle totals away from boundaries.

Initial implementation: configurable in-run density, realised velocity and
polarisation, with cell-integrated Gaussian smoothing, non-overlapping temporal
windows, periodic image sums, configurable nonperiodic boundary treatment,
support counts and conservation totals. See `docs/coarse-graining.md`.
Momentum density and velocity covariance/stress remain to complete this milestone.

### M4 — Hydrodynamic admissibility diagnostics

At each cell/window and across replicate runs, evaluate:

1. **Occupancy/statistical support:** enough effective agents and samples.
2. **Scale separation:** microscopic correlation length/time is smaller than the
   selected coarse-graining scales; fields vary slowly across them.
3. **Local equilibration:** velocity fluctuation/covariance is sufficiently
   stable over the analysis window.
4. **Closure locality:** conditionally sampled unresolved stress/flux has small
   variance when conditioned on candidate local fields, and weak dependence on
   their gradients/history.
5. **Robustness:** decisions persist across nearby grid/kernel/window choices,
   timesteps, seeds, and resolutions.

The result is a tri-state mask: `admissible`, `indeterminate`, or
`inadmissible`, with every failed metric retained.

### M5 — Validation and reporting

Use synthetic cases where local hydrodynamic behavior is expected and where it
should fail (strong transients, boundaries, sparse regions, multi-stream flow).
Generate field maps, diagnostic distributions, sensitivity plots, and a concise
methods report.

## Initial acceptance criteria

- Re-running a named configuration with the same seed reproduces its trajectory
  within a declared floating-point tolerance.
- Coarse-grained density integrates to the in-domain particle count within a
  boundary-aware tolerance.
- Diagnostic outputs include units, kernel scale, time window, thresholds, and
  uncertainty estimates.
- An admissibility label is never emitted without sufficient support; it is
  `indeterminate` instead.
