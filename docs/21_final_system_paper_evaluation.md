# Final-System Simulation Evidence

The paper-evidence workflow is an explicit simulator-only diagnostic mode. It leaves the
normal controller and safety defaults unchanged and writes a new result tree below
`evaluation_results/final_system_<UTC timestamp>/`.

## Reproduction

After sourcing ROS 2 Humble and the workspace install, run:

```bash
ctr_run_final_system_evaluation --output-root \
  evaluation_results/final_system_<UTC timestamp> --duration 25
```

The command executes the reference, five-seed repeatability, target-source,
target-difficulty, lumen-geometry, and controller-configuration matrices sequentially.
`--resume` preserves completed cells after an interrupted batch. `--matrix` selects one
matrix for a bounded rerun. Every navigation cell uses the existing evaluator and records
the baseline and evaluated controller runs; aggregate tables use the evaluated run.

## Diagnostic data

`evaluation.diagnostic_data_collection` defaults to `false`. The paper runner enables it
explicitly and records three additional raw files:

- `tactile_safety.csv`: simulated raw and filtered tactile force, on/off thresholds,
  tactile state, source/frame validity, evidence age, safety state, fault/latch evidence,
  commanded and safe controls, and the applied command scale/gate.
- `mppi_cost_terms.csv`: unweighted terms, configured weights, weighted terms for the
  minimum-cost rollout, weighted-population means, and total-cost statistics.
- `mppi_computation.csv`: non-overlapping sampling, rollout propagation, target/control
  cost, lumen cost, tactile cost, normalization, control-update, ROS-conversion, and
  end-to-end solve durations measured with the monotonic clock.

Enabling diagnostics does not change random-number consumption, sampled controls,
importance weights, controller commands, or safety decisions. Source tests compare exact
commands with diagnostics disabled and enabled.

## Metric definitions

| Metric | Formula and sample window | Unit | Empty or invalid data |
|---|---|---:|---|
| Final target error | `||tip_N - target||_2` at the final aligned tip sample | m | unavailable without an aligned sample |
| Tip-to-target RMSE | `sqrt(mean_i(||tip_i - target||_2^2))` over valid aligned samples | m | unavailable for an empty aligned set |
| Centerline-tracking RMSE | `sqrt(mean_i(d(tip_i, analytic centerline)^2))` over valid lumen samples | m | unavailable without lumen samples |
| Maximum centerline distance | `max_i d(tip_i, analytic centerline)` with its first timestamp | m | unavailable without lumen samples |
| Minimum whole-backbone clearance | minimum physical clearance over every evaluated backbone point and sample | m | unavailable without backbone evidence |
| Effective solve frequency | `(N - 1)/(t_last - t_first)` from controller-metric timestamps | Hz | unavailable for fewer than two samples |
| Solve-time statistics | median, 95th percentile, and maximum of finite solve durations | s | unavailable for an empty solve set |
| Deadline misses | count and percentage of solve durations above the configured control period | count, % | zero count and unavailable percentage for an empty set |
| Cartesian path length | `sum_i ||tip_i - tip_(i-1)||_2` | m | zero for fewer than two samples |
| Cumulative control effort | `sum_i ||u_i||_2^2 * dt_i` for valid aligned applied safe commands | mixed command units squared second | zero for an empty set |
| Command total variation | sum of Euclidean changes, separately for insertion and rotation command groups | m/s, rad/s | zero for fewer than two commands |
| Saturation percentage | percentage of command samples with any component at its configured group limit | % | zero for an empty set |
| Event counts and durations | transition counts and interval duration in each tactile/safety state | count, s | zero when the event is absent |

Target-distance RMSE is not called reference-path tracking RMSE. Centerline tracking is a
separate geometric metric and is not represented as an optimizer cost unless the selected
controller configuration explicitly enables such a cost.

## Artifacts

Every completed navigation run retains raw CSV data, metadata, summary JSON, Markdown
report, the ten established run plots, and four diagnostic plots. Aggregate outputs include
the eleven paper figures, eight paper tables, `comparison.csv`, `comparison.json`,
`experiment_matrix.csv`, `plot_index.json`, `manifest.json`, `artifact_validation.md`, and
`paper_results.md`. `overleaf_upload/` contains only the eleven figures, tables, report, and
plot index; raw data remains referenced by provenance paths.

All evidence remains simulator-only. Timing depends on host load, and the five-seed matrix
supports descriptive repeatability analysis rather than a statistical-significance or
hardware real-time claim.
