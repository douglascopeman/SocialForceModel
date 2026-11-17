# Periodic boundary implementation

`RectangularDomain::boundary_x` selects `open`, `walls`, or `periodic`.
The experiment default remains open; the bare domain default remains walls.
`RectangularDomain::boundary_y` selects `walls` (default) or `periodic`.

For length `L = max_x - min_x`, positions are wrapped into `[min_x, max_x)` on
insertion and after every integration step, without changing preferred or
realized velocity. Modulo arithmetic handles negative positions and crossings
of more than one domain length.

Pair displacement uses `dx -= L * round(dx/L)` before calculating the paper's
elliptical distance, its gradient, and perception weight. The same image is
used in `r` and `r - speed * anticipation_time * desired_direction`. There are
no x wall forces for periodic or open boundaries. The y wall treatment is
the existing soft repulsion when `boundary_y = "walls"`; periodic y disables
both y walls. Periodic y uses the same wrapping and minimum-image operations
with `L = max_y - min_y`. Corner crossings wrap both coordinates in one step.
Shared helpers in `include/sfm/geometry.hpp` serve both force evaluation and
population placement.

This is the nearest-image approximation, not a sum over all periodic images;
keep the domain large compared with the significant interaction range and
anticipation distance. Exact half-box separations select the image via
`std::round`, which introduces the usual nearest-image tie convention.

Periodic mode requires fixed-vector objectives at both configuration loading
and agent insertion. Rendering uses wrapped coordinates, so a trajectory
visibly jumps from one end to the other at a crossing. Displacements and
transport statistics must account for these jumps; the CSV contains wrapped
positions, not cumulative unwrapped positions.

The dedicated periodic test checks crossings in both directions, multiple-box
displacements, nonzero domain origins, endpoint wrapping, velocity retention,
population retention, disabled end-wall forces, and equivalence of a moving
pair across the join to an identical pair in the interior. Existing force and
direction tests remain in place.

The tests also cover y-only periodicity, simultaneous corner crossings,
pair-force equivalence across y/corner joins, disabled transverse wall forces,
and deterministic unidirectional placement with minimum distances measured
across both periodic axes.

## Single-direction bulk configuration

`config/unidirectional_periodic.toml` runs 80 pedestrians for 60 s in a
20 m by 10 m torus, with desired direction `(1, 0)` for every agent. It uses
the paper's 2 s anticipation parameter and 0.26 m/s desired-speed dispersion.
Set `agents.desired_speed_stddev_m_per_s = 0.0` for an identical-speed control.
Uniform initial placement covers the entire y interval, including the strips
previously reserved for wall margins. This makes the setup translation invariant
in distribution; it does not enforce uniform instantaneous density or velocity.

The renderer draws periodic joins as dashed edges in both axes and records
both boundary settings in the run summary. `check_periodic_run.py --height 10`
additionally checks that y is wrapped and reports transverse seam crossings.

Optional coarse-grained fields now use this periodic geometry for grid membership
and kernel image sums on both axes. See [coarse-graining.md](coarse-graining.md).
Macroscopic output does not yet assess hydrodynamic admissibility.
