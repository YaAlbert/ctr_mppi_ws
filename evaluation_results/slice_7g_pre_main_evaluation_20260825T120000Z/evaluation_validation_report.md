# Slice 7G pre-main evaluation validation

Validated on Ubuntu 22.04 / ROS 2 Humble from
`feature/slice-7g-functional-simulation` at starting commit
`042e70c9626facd436151431d152e508746a78ea`. These are development-simulation
results, not hardware or production-promotion evidence.

## Pipeline and completion

`ctr_evaluation.nodes.evaluation_node.EvaluationNode` owns recording through
`ExperimentRecorder`. The development orchestrator accepts the target, starts
the recorder through `/ctr/start_experiment`, runs the scheduled evaluation
window, and finalizes through `/ctr/stop_experiment`. Completion means that the
whole requested evaluation window finished; merely entering the goal tolerance
does not finalize the run early. A shutdown while recording finalizes with
`run_status.status=incomplete`, `interrupted=true`, and functional/navigation
success false.

The task-owned result root is:

```text
evaluation_results/slice_7g_pre_main_evaluation_20260825T120000Z
```

The final validated cases are `profile_e2e`, `cli_e2e`, and `rviz_e2e`.
Each candidate run contains 27 required artifacts. Raw time-series data are
`state.csv`, `tip.csv`, `reference.csv`, `command.csv`, `solve_timing.csv`,
`horizon.csv`, `reference_path.csv`, `backbone.csv`, `lumen_evaluation.csv`, and
`aligned_samples.csv`. Structured summaries are `metadata.yaml`, `summary.json`,
`orchestration.json`, `comparison.json`, `finalization_trace.json`, `report.md`,
and `comparison.md`. The ten generated PNGs show trajectory, tip trajectory,
tracking/goal error, command history, solve timing, cumulative control effort,
curved-lumen geometry, clearance, and centerline deviation. Raw result trees are
intentionally ignored by Git; this concise validation report is the committed
summary.

## Metric semantics

All position, error, and clearance quantities use metres; time uses seconds and
rates use hertz.

| Metric | Source and exact meaning | Missing/invalid handling |
| --- | --- | --- |
| Readiness time | `stability_collection_end_monotonic - monitor_created_monotonic`; readiness additionally requires stable state/tip and fault-free tactile/safety status. | Missing/nonfinite endpoints produce no value and fail development validation. |
| Effective MPPI solve frequency | For sorted solve timestamps `t`, `(N-1)/(t[-1]-t[0])`; fewer than two samples or a nonpositive span yields `0`. | Nonfinite samples are rejected/accounted by the recorder. |
| Final target error | `||tip[-1] - target||_2` over aligned evaluation samples. | No valid aligned samples makes the goal metric unavailable and the run invalid. |
| `tip_to_target_rmse_m` | `sqrt(mean(||tip_i - target||_2^2))`. The controller-owned reference has one terminal pose, repeated for time alignment. This is **not reference-path tracking RMSE**. | Missing/nonfinite aligned data makes the metric unavailable and fails data-quality/goal acceptance. |
| `centerline_tracking_rmse_m` | `sqrt(mean(radial_offset_i^2))`, where each real recorded tip is projected to the closest valid point on the analytic `CurvedLumen` centerline. It is independent of the singleton controller target. | Reported only for the curved centerline-target scenario; invalid lumen samples make the lumen result fail closed. |
| Minimum wall clearance | Minimum analytic physical clearance over every recorded backbone point and aligned lumen sample. | Missing backbone samples invalidate the required curved-lumen evaluation. |
| Collision count | Number of false-to-true transitions in the analytic whole-backbone physical-collision flag, not the number of collision samples. | Missing/invalid geometry data prevents a physical pass. |
| Runtime | Recorder stop time minus start time (`actual_duration` / `experiment_wall_duration`). Requested window and actual recording duration are reported separately. | Negative/nonfinite durations are rejected. |
| Navigation success | `completed_evaluation_window && run_valid && goal_reached && physical_safety_pass`. Goal reached requires the configured 0.003 m tolerance for the required hold interval. | Any false/missing prerequisite means false. |
| Comparison validity | Exact pair-identity compatibility plus valid baseline and candidate runs. Timing remains descriptive, not a navigation acceptance criterion. | Compatibility or run-validity failure makes comparison invalid with reasons. |
| Safety/tactile events | Counts recording-window safety fault/invalid/e-stop samples and tactile invalid/non-simulated samples. | Wrong types or invalid source values count as faults; missing required status fails readiness/data quality. |

## Independently validated cases

The runtime-published reference, accepted target, target source, projection,
seed, reference pose count, and first observed accepted-reference timestamp are
recorded in `orchestration.json` and `development_results.json`.

| Case | Target source and accepted target (m, `base_link`) | Window (s) | Tip samples | Final error (m) | Target RMSE (m) | Centerline RMSE (m) | Min clearance (m) | Solve Hz | Collisions | Result |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Profile, seed 11 | profile; `[0.0211809664, 0, 0.0847121866]` | 25 | 2347 | 0.0013361073 | 0.0023315495 | 0.0005255858 | 0.0272053347 | 1.3173857502 | 0 | completed/success |
| CLI, seed 11 | cli; `[0.0166457424, 0.00397477634, 0.102231139]` | 30 | 2770 | 0.0012555985 | 0.0123121233 | 0.0086360051 | 0.0151135792 | 1.2918651133 | 0 | completed/success |
| Automated RViz-mode PointStamped, seed 11 | rviz; raw world `[0.0192468684, 0.03, 0.0809841385]`, projected `[0.0192468684, 0, 0.0809841385]` | 25 | 2369 | 0.0009852539 | 0.0009852539 | 0.0003959126 | 0.0280081766 | 1.3999401623 | 0 | completed/success |

The RViz-mode case publishes the established deterministic point through the
real `/ctr/target_point_candidate` `PointStamped` transport. It validates the
automated evaluation path and does not replace the previously completed manual
RViz usability check. Its accepted point begins inside goal tolerance, so the
profile and CLI cases provide the movement-to-target evidence.

An independent parser loaded every JSON/YAML/CSV artifact, rejected textual
NaN/Inf, checked nonempty files, monotonic timestamps, target identity, and
sample-count consistency, and recomputed final target error, both RMSE values,
minimum clearance, runtime, solve frequency, and collision events from raw
records. All reported values matched; the maximum absolute numeric difference
was `3.65e-17`.

## Reproduction

```bash
source /opt/ros/humble/setup.bash
source install_slice7g_development/setup.bash

ros2 run ctr_evaluation ctr_run_slice_7g_development \
  --development-simulation --skip-smoke --seeds 11 --duration 25 \
  --target-source profile --output-root <new-result-root>/profile

ros2 run ctr_evaluation ctr_run_slice_7g_development \
  --development-simulation --skip-smoke --seeds 11 --duration 30 \
  --target-source cli --target-x 0.0166457424 --target-y 0.00397477634 \
  --target-z 0.102231139 --output-root <new-result-root>/cli

ros2 run ctr_evaluation ctr_run_slice_7g_development \
  --development-simulation --skip-smoke --seeds 11 --duration 25 \
  --target-source rviz --automated-rviz-evaluation --target-frame world \
  --target-x 0.01924686842428271 --target-y 0.03 \
  --target-z 0.08098413850007993 --output-root <new-result-root>/rviz
```

Open `report.md` in the candidate directory and the PNG files with any image
viewer. The top-level `development_report.md` compares attempts and links to
each candidate report.

## Validation outcome and limitations

The affected package built successfully and the complete practical functional
suite passed `930/930`. No collision, raw public exception, or persistent child
was observed in the final cases. One earlier pre-final profile attempt reported
a transient safety fault; the clean retry and all three final cases completed
without a safety/tactile event. This validation does not claim real-time,
hardware, privileged-install, or production-promotion evidence.

Verdict: `SLICE_7G_EVALUATION_END_TO_END_CONFIRMED_READY_FOR_MAIN`.
