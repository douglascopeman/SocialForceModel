# Close-encounter convention

The pair force still uses the paper's exponential elliptical potential. No
softening length, extra noise, or force cap has been introduced.

## Exact collinear ties

Let `s` be the other pedestrian's speed times its anticipation time, `x` the
subject's longitudinal displacement along that pedestrian's desired direction,
and `y` the perpendicular displacement. Inside the anticipation segment,
`0 < x < s`, the ellipse collapses when `y = 0`.

For nonzero y the force retains its original side. At exactly zero y we select
the one-sided normal pointing towards positive global y. For a vertical desired
direction neither normal has a y component, so positive global x is the fallback.
For leftward horizontal motion the tie also resolves towards global positive y.

The unweighted transverse force magnitude at the tie is

`(V0 / sigma) * s / (2 * sqrt(x) * sqrt(s - x))`.

The existing perception weight is then applied. At the midpoint with the paper's
parameters, this gives 7 m/s² before perception weighting. The magnitude depends
on the geometry and is not fixed at 7 elsewhere. This deterministic convention
explicitly breaks reflection symmetry only when the exact tie is encountered.

## Numerical evaluation nearby

The old expression subtracted nearly equal quantities to compute b squared and
returned zero below a tolerance. The replacement evaluates the same potential
gradient using longitudinal/perpendicular coordinates and a rationalised
distance expression. It avoids that cancellation and the artificial zero-force
region. At the segment midpoint, for example, b is exactly |y| and the force
retains its expected one-sided limit even for very small nonzero offsets.

The stationary-other / zero-anticipation case remains ordinary radial repulsion.
Coincident pedestrian positions and an exact anticipation endpoint have a
different singularity; these now raise an explicit domain error. Nonfinite force
results also raise an error. This convention does not remove the large forces
possible near segment endpoints or establish timestep convergence by itself.

## Counting events

`Simulation::encounter_statistics()` exposes:

- `pair_force_evaluations`: ordered pair-force evaluations completed during
  integration; a successful N-agent step contributes N(N-1).
- `upward_tie_breaks`: evaluations that invoked the exact-tie convention.

These totals are saved in `summary.txt`. Public `pair_force()` queries do not
change the counters, so diagnostics and tests cannot inflate a run's counts.
Repeated events for the same pair are counted separately; the counters do not
count distinct pedestrian pairs or statistically independent encounters.

To run a frequency check with the existing bulk settings while preserving the
earlier outputs:

```bash
cmake -S . -B build/experiments -DCMAKE_BUILD_TYPE=Release
cmake --build build/experiments
./build/experiments/sfm_sim config/close_encounter_smoke.toml
```

This uses 80 uniformly placed, rightward pedestrians in a 20 m by 10 m periodic
domain for 60 s at dt=0.002 s, with the paper's 2 s anticipation time and seed
20261116. The expected total is 189,600,000 ordered force evaluations. A zero
tie count in such a run establishes only that no exact ties occurred there.

The initial run of this configuration completed in approximately 11 seconds
and recorded 0 upward tie-breaks in 189,600,000 directed evaluations. Its 121
sampled frames retained all 80 agents with finite state and wrapped x/y positions.
The output is in `results/close-encounter-smoke/`, separate from earlier runs.

The dedicated test constructs exact ties to verify both the chosen force and
counter increments, checks both sides down to very small offsets, covers
rotated headings and periodic images, checks endpoint errors, and compares
ordinary forces against independent finite differences of the potential.
