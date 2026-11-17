# Periodic x implementation

`RectangularDomain::boundary_x` selects `open`, `walls`, or `periodic`.
The experiment default remains open; the bare domain default remains walls.

For length `L = max_x - min_x`, positions are wrapped into `[min_x, max_x)` on
insertion and after every integration step, without changing preferred or
realized velocity. Modulo arithmetic handles negative positions and crossings
of more than one domain length.

Pair displacement uses `dx -= L * round(dx/L)` before calculating the paper's
elliptical distance, its gradient, and perception weight. The same image is
used in `r` and `r - speed * anticipation_time * desired_direction`. There are
no x wall forces for periodic or open boundaries. The y wall treatment is
unchanged.

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
