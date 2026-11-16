# Helbing–Molnár source-model notes

## Source

Dirk Helbing and Péter Molnár, *Social force model for pedestrian dynamics*.
The supplied PDF is arXiv `cond-mat/9805244v1` (20 May 1998) and describes the
model used in the paper's 1995 work. This ledger refers to the PDF's printed
equation numbers and pages.

## Baseline model to implement

The first implementation should reproduce equations (1)–(13), including its
separate **preferred velocity** `w`, before offering a simplified or later
Social Force Model variant. It evolves

`d w_alpha / dt = F_alpha + fluctuation`,

then uses a speed-limited realized velocity `v_alpha` to evolve position. This
is materially different from directly integrating an acceleration equation for
`v_alpha`.

| Item | Paper equation/page | Code concept | Implementation decision |
| --- | --- | --- | --- |
| Goal direction | (1), p. 3 | `GoalDirection` | Unit vector to next waypoint or nearest gate point. |
| Driving relaxation | (2), p. 3 | `DrivingForce` | `(v0 * e - v) / tau`; acts on preferred velocity. |
| Anisotropic pair distance | (3)–(4), p. 4 | `EllipticalDistance` | Ellipse minor semi-axis `b`; pair potential gradient. |
| Wall force | (5), p. 4 | `BoundaryForce` | Gradient of distance-based repulsive potential at closest boundary point. |
| Attraction | (6), p. 4 | `AttractionForce` | Optional; omit only as a documented baseline-run choice. |
| Field of view | (7)–(8), pp. 5–6 | `PerceptionWeight` | Weight 1 within sight cone; `c` otherwise. |
| Total force | (9), p. 6 | `ForceAssembler` | Sum driving, pedestrian, boundary, and attraction terms. |
| Fluctuation | (10), p. 6 | `NoiseModel` | Interface exists from M1; disabled in paper reference runs. Distribution/correlation requires an explicit later choice. |
| Realized speed cap | (11)–(12), pp. 6–7 | `SpeedLimiter` | `v = w * min(1, vmax / |w|)`; handle `|w| = 0` safely. |

## Reference simulation values

The paper's illustrative simulations (p. 7–8) use exponential potentials:

`V(b) = V0 exp(-b / sigma)`, with `V0 = 2.1 m^2 s^-2`, `sigma = 0.3 m`;
`U(r) = U0 exp(-r / R)`, with `U0 = 10 m^2 s^-2`, `R = 0.2 m`.

It uses a Gaussian desired-speed distribution with mean `1.34 m s^-1` and
standard deviation `0.26 m s^-1`, `vmax = 1.3 v0`, `tau = 0.5 s`, anticipation
parameter `Delta t = 2 s`, field-of-view angle `2 phi = 200 degrees`, and rear
weight `c = 0.5`. The illustrated walkways are 10 m wide. Attraction and
fluctuation terms are disabled for those runs.

## Deliberate non-baseline extensions

Physical body-contact and tangential-friction forces are not included in this
source paper's equations (1)–(13); do not label such later extensions as
Helbing–Molnár baseline behavior. Any addition requires its own source,
configuration namespace, and regression tests.
