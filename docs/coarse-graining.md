# Dynamic macroscopic fields

Macroscopic output is opt-in and computed during integration. It does not depend
on trajectory or rendering output intervals. Existing configurations default to
disabled. Run the small example with:

```bash
./build/sfm_sim config/macroscopic_smoke.toml
```

## Configuration

```toml
[coarse_graining]
enabled = true
fields = ["density", "velocity", "polarisation"]
grid_spacing_m = 0.5
kernel = "gaussian"
kernel_width_m = 0.5
kernel_cutoff_sigma = 3.0
window_s = 1.0
boundary_policy = "renormalize"
```

These values are the defaults except `enabled`, which defaults to `false`.
Remove entries from `fields` to omit those observables from output. Any nonempty
subset and order is supported, including velocity or polarisation without
density. Shared normalization and support weights are always accumulated.
Unknown keys in this section, unsupported field names, duplicate fields, malformed
field lists, and invalid enabled settings are errors. Field lists are single-line
arrays of double-quoted strings. Other sections retain the existing minimal
configuration reader behaviour. With output disabled, no estimator or new field
files are created; any files from earlier runs remain on disk.

`grid_spacing_m` specifies square cells anchored at the domain minimum. The
last row and column are shortened when the extent is not an integer multiple
of spacing. Their actual areas are used in density calculations and output.
The grid is limited to one million cells. All scales must be finite and positive;
the cutoff must be at most eight standard deviations and the support radius at
most 1,000 times the smaller domain extent.

## Spatial and temporal estimators

The current kernel is a separable 2-D Gaussian with standard deviation
`kernel_width_m = h` on each axis. It is cut off at `|dx|, |dy| < c*h`, where
`c = kernel_cutoff_sigma`, and normalized over that square support. This cutoff
is rectangular rather than radial. Kernel integrals over cells are evaluated
with Gaussian error functions; fields are **cell averages**, rather than
point samples at cell centres.

For agent `i` and cell `C`, let `q_iC` be the integrated kernel mass assigned to
the cell after boundary treatment. For each integration interval, its final
microscopic state represents that interval (right-endpoint quadrature). With
duration weight `dt`, define `a_iC = dt*q_iC`. Over a window of actual duration
`T` and cell area `A`, the fields are:

- Density: `rho_C = sum a_iC / (T*A)`, in agents per square metre.
- Velocity: `u_C = sum (a_iC*v_i) / sum a_iC`, in metres per second.
- Polarisation vector: `p_C = sum (a_iC*e_i) / sum a_iC`, dimensionless,
  where `e_i = v_i/|v_i|` for speed above `1e-12 m/s`, and zero otherwise.
- Polarisation magnitude: `P_C = |p_C|`, in `[0,1]`.

Velocities are realised microscopic velocities, not preferred velocities or
desired directions. Equal opposing headings cancel in polarisation even when
their speeds differ. Resting agents remain in its denominator and contribute
zero to its numerator, so an occupied stationary cell has `P = 0`. Polarisation
is the magnitude of the window-averaged vector, not the average of instantaneous
magnitudes. Time-varying opposing directions can therefore cancel.

Windows start at time zero, do not overlap, and have width `window_s`. A final
partial window is retained. No instantaneous time-zero field is emitted.
If a simulation step crosses a window boundary, its duration is partitioned
between the windows using that same right-endpoint state. The simulation steps
are unchanged, so enabling output does not change microscopic trajectories.
Windows shorter than a timestep do not add temporal information. Temporal
discretization error still requires timestep refinement.

## Boundary treatment and conservation

Periodic axes wrap positions and sum **all kernel images within the cutoff**,
including when support exceeds one domain length. No kernel mass is lost at a
periodic join. This differs from the single nearest-image approximation used
by the microscopic interaction force.

On nonperiodic axes:

- `renormalize` rescales each included agent's clipped kernel to unit mass within
  the domain. Density integrates to the duration-weighted mean in-domain count,
  up to floating-point roundoff. This changes the kernel shape near boundaries.
- `truncate` discards kernel mass outside the domain without rescaling. Density
  undercounts near physical/open edges; conservation is expected for agents
  whose entire kernel support lies inside those edges.

Membership uses half-open intervals `[min,max)` on both axes. Agents outside a
nonperiodic axis are excluded entirely, including their kernel tails; their
duration-weighted count is reported for every window. This is explicit because
the microscopic soft walls can permit excursions and open-x agents persist
after leaving the domain. Periodic positions are wrapped before this check.

## Outputs and statistical support

The run's `fields/` directory contains:

- `metadata.json`: schema version, selected fields, units, domain geometry,
  smoothing scales, boundary policy, and estimator conventions.
- `windows.csv`: authoritative window/file index, actual start/end/duration,
  temporal sample count, mean included/excluded population, and integrated density.
- `window_000000.csv`, etc.: one row per cell with window bounds, indices,
  cell centre and dimensions, support, and selected observables.

Selected field columns are:

| Field | Columns | Units |
| --- | --- | --- |
| `density` | `rho_per_m2` | agents/m² |
| `velocity` | `u_x_m_per_s`, `u_y_m_per_s` | m/s |
| `polarisation` | `polarisation_x`, `polarisation_y`, `polarisation` | dimensionless |

Support columns are always present. `sample_count` counts agent-time
contributions with nonzero kernel mass. `effective_samples` is the weighted
support measure `(sum a)^2 / sum(a^2)`, or zero without support. It includes
repeated observations of the same agent and does **not** correct for temporal
correlation; it is not an independent sample count or an uncertainty estimate.
Empty cells have zero density and support, and `nan` velocity/polarisation.
No hydrodynamic admissibility label is produced.

Outputs are streamed with memory proportional to grid size, independent of
simulation duration. Rerunning in the same directory rewrites metadata, the
index, and matching window filenames. Use `windows.csv` to select current files;
old windows beyond the new run's end may remain. Distinct experiment output
directories preserve earlier results.

## Extending the field set

`CoarseGrainingConfig` contains the selected `MacroscopicField` values.
`CoarseGrainer` owns shared spatial weighting, temporal moments and support;
`FieldOutput` owns window scheduling, metadata and serialization. To add an
observable, extend the enum/name mapping, its moments and reduction in the
estimator, and the output column descriptor table. Configuration parsing and
the run loop need no new per-field branches. Kernels and alternative temporal
estimators can be added independently of observable selection.

This is the first M3 field set. Momentum density, covariance/stress, and the
M4 admissibility diagnostics remain future work.
