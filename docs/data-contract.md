# Data contract

All outputs carry SI-like units explicitly in metadata; the final unit system is
chosen with the model specification.

## Run artefacts

```
results/<run-id>/
  manifest.json          configuration copy, seed, version, units, timestamps
  trajectory.csv         time, agent_id, x, y, vx, vy, ax, state
  fields/                optional; enabled by coarse_graining.enabled
    metadata.json        kernel, grid, boundary, temporal and unit conventions
    windows.csv          authoritative index and per-window conservation totals
    window_000000.csv    one file per coarse-grained time window
  diagnostics/           per-cell metric values and final labels
  logs/
```

## Coarse-field rows

The initial M3 implementation supports selectable density (`rho_per_m2`),
velocity (`u_x_m_per_s`, `u_y_m_per_s`), and polarisation (vector and magnitude).
Each row represents one `(time_window, grid_cell)` and contains window bounds,
grid coordinates and actual cell dimensions, raw/effective support, and the
selected field columns. Smoothing scales, boundaries, units and definitions
are shared in `fields/metadata.json`. See [coarse-graining.md](coarse-graining.md)
for the implemented contract. Momentum density and covariance remain planned.

The manifest, diagnostic and log artefacts above describe the intended full
pipeline; current runs provide `summary.txt`, trajectories, renders, and optional
fields, rather than all planned artefacts.

## Admissibility rows

Each row contains the corresponding field key, tri-state label, each diagnostic
score/threshold, uncertainty, and an enumerated reason code. Keep raw metrics;
never preserve only a final Boolean.
