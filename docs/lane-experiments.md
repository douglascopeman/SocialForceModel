# Initial lane-formation experiments

## Question and controls

Can a mixed bidirectional population spontaneously develop horizontal lanes?
The original periodic smoke run used 40 agents, a 20 m by 6 m channel, and
60 s of simulation. These experiments use the same geometry with 180 s of
simulation, fixed-vector objectives, equal opposing populations, and uniformly
mixed initial positions. There is no imposed preference for a lane or side.

`agents.initial_placement = "uniform"` distributes positions across the channel
with a 0.4 m minimum initial distance, including across the periodic seam.
Both directions start at rest. The minimum distance is an initialisation
constraint, not an extra dynamical force or a hard-core particle radius.

Except where specified, use paper parameters: desired speed mean 1.34 m/s,
standard deviation 0.26 m/s, relaxation time 0.5 s, anticipation time 2 s,
pair potential amplitude 2.1 m²/s² and range 0.3 m, wall amplitude 10 m²/s²
and range 0.2 m, 200 degree field of view, and rear weight 0.5. Attraction
and noise remain absent. Configurations and random seeds are versioned in
`config/lanes/`; each result directory includes trajectories, frames and metrics.

## Results

Metrics below average over 120 < t <= 180 s in 0.5 m horizontal strips.

| Configuration | N | Density (m^-2) | dt (s) | Anticipation (s) | Corrected segregation | Forward speed (m/s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `density_040.toml` | 40 | 0.333 | 0.01 | 2 | 0.024 | 1.191 |
| `density_080.toml` | 80 | 0.667 | 0.01 | 2 | 0.050 | 1.002 |
| `density_120.toml` | 120 | 1.000 | 0.01 | 2 | 0.013 | 0.864 |
| `n080-fine.toml` | 80 | 0.667 | 0.002 | 2 | 0.028 | 1.007 |
| `n080-short-anticipation.toml` | 80 | 0.667 | 0.002 | 0.5 | 0.488 | 1.222 |
| `n080-short-seed2.toml` | 80 | 0.667 | 0.002 | 0.5 | 0.331 | 1.261 |

All use seed 20261116 except `n080-short-seed2.toml`, which uses 20261117.
The two fine-timestep runs with seed 20261116 have identical initial populations
and differ only in anticipation time. The first three runs form a density
comparison. The 120-agent run has seven sampled agent states outside the soft
y walls, so its numerical result should be treated cautiously. The other five
runs have no sampled y-wall crossings.

The shorter-anticipation setting produces visibly stronger alternating bands
and retains increased segregation with a second seed. Increasing density alone
did not establish lanes in this small sweep. This result is an exploratory
parameter experiment, not a reproduction of the paper's parameter set: its
anticipation time is 2 s. Each fine run took approximately 24 s of compute on
this environment; the three coarse density runs took about 17 s in total.

## Exploratory diagnostic

`scripts/analyse_lanes.py` computes, for each snapshot:

`P = sum_bins |N_right - N_left| / N`.

A perfectly segregated horizontal-strip state has `P = 1`, but random mixing
has a nonzero baseline at finite N. The script calculates the exact expected
baseline `B` under balanced random relabelling at the observed strip occupancies
(a hypergeometric distribution), then reports `(P - B) / (1 - B)`.

Zero therefore matches random mixing in expectation; one indicates pure strips.
Negative fluctuations are possible. This indicator depends on strip width and
can miss meandering lanes; it is not a proof of stability or hydrodynamic
admissibility. Outside-wall agents are counted in the nearest edge strip and
flagged, rather than silently discarded. Mean forward speed is the average of
`sign(desired_direction_x) * vx`, distinguishing progress from transverse motion.

## Reproduce

For inexpensive sweeps, build with optimisation in a separate build directory:

```bash
cmake -S . -B build/experiments -DCMAKE_BUILD_TYPE=Release
cmake --build build/experiments
ctest --test-dir build/experiments --output-on-failure
./build/experiments/sfm_sim config/lanes/n080-fine.toml
./build/experiments/sfm_sim config/lanes/n080-short-anticipation.toml
python3 scripts/analyse_lanes.py results/lanes-n080-fine results/lanes-n080-short-anticipation
python3 scripts/animate_run.py results/lanes-n080-fine --fps 20
python3 scripts/animate_run.py results/lanes-n080-short-anticipation --fps 20
```

Frames every 0.5 s play at 10x speed when encoded at 20 fps.

## Useful follow-ups

- More seeds and sensitivity to horizontal strip width.
- Timestep refinement of the promising 0.5 s anticipation setting; the current
  timestep comparison covers the 2 s reference setting only.
- Lower desired-speed dispersion to reduce overtaking, changing one parameter
  at a time.
- A narrower channel at fixed number density to test confinement and lane count.
- Check close-encounter force singularities and wall treatment before increasing
  density further. Soft walls and the original absence of hard-core contact
  make arbitrarily dense experiments inappropriate.
