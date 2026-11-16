# Data contract

All outputs carry SI-like units explicitly in metadata; the final unit system is
chosen with the model specification.

## Run artefacts

```
results/<run-id>/
  manifest.json          configuration copy, seed, version, units, timestamps
  trajectory.csv         time, agent_id, x, y, vx, vy, ax, state
  fields/                one file per coarse-grained time window
  diagnostics/           per-cell metric values and final labels
  logs/
```

## Coarse-field rows

Each row represents one `(time_window, grid_cell)` and contains grid coordinates,
`rho`, `u_x`, `u_y`, momentum-density components, velocity covariance components,
raw/effective support, and the smoothing scales used.

## Admissibility rows

Each row contains the corresponding field key, tri-state label, each diagnostic
score/threshold, uncertainty, and an enumerated reason code. Keep raw metrics;
never preserve only a final Boolean.
