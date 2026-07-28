# Acceptance Criteria

## Milestone 1

- colcon build succeeds;
- all packages are discoverable;
- all placeholder nodes launch;
- YAML files load successfully;
- invalid parameters are rejected.

## Milestone 2

- model accepts q with shape (6,);
- backbone output has shape (N, 3);
- tip output has shape (3,);
- tests pass;
- output contains no NaN or Inf.

## Milestone 3

- RViz2 displays the backbone;
- joint commands update the robot;
- units and axes are correct;
- state topics publish at the configured rate.

## Milestone 4

- CTR tip converges toward a reachable target;
- command remains within limits;
- controller publishes solve-time metrics;
- controller does not crash on invalid reference input.

Status: verified complete for the minimum fixed-target MPPI scope on
2026-07-21 in the current Ubuntu 22.04 / ROS2 Humble environment.

Unit-test verification:

- `ctr_bringup`: 19 focused tests passed;
- `ctr_mppi_controller`: 14 focused tests passed;
- `ctr_sim`: 4 focused tests passed.

Build verification:

- clean isolated build used `build_shutdown_final`, `install_shutdown_final`,
  and `log_shutdown_final`;
- all 11 packages finished successfully.

ROS2 runtime verification:

- foreground PTY launch used `install_shutdown_final`;
- nodes verified alive before shutdown:
  - `/parameter_validator`;
  - `/ctr_simulator`;
  - `/mppi_controller`;
- no hardware node started;
- topics verified:
  - `/ctr/state`;
  - `/ctr/mppi_command`;
  - `/ctr/safe_command`;
  - `/ctr/controller/metrics`;
- Ctrl-C was delivered to the actual foreground `ros2 launch` process group;
- `ros2 launch` exited naturally with exit code 0;
- all child nodes finished cleanly;
- no project process or zombie remained;
- no `KeyboardInterrupt` traceback, `rcl_shutdown already called`, rclpy
  exception, process death, or signal escalation occurred.

Scope exclusions still apply:

- hardware execution remains disabled and unverified;
- the CTR model is still an approximate model;
- trajectory tracking, shape control, obstacle avoidance, tactile control, and
  hardware support are not complete.

## Milestone 5

- reference path is tracked;
- tip RMSE is calculated;
- circle, ellipse and helix tests are repeatable.

Status: functionally integrated and runtime smoke verified for the
simulation-only trajectory scope on 2026-07-21 in the current Ubuntu 22.04 /
ROS2 Humble environment.

Verification state:

- functionally integrated: yes;
- runtime smoke verified: yes;
- performance verified: no;
- real-time capable: no;
- physically validated: no;
- hardware validated: no.

Implemented evidence:

- circle, ellipse, and helix trajectory generation exist;
- loop and hold-final horizon extraction exist;
- elapsed-time trajectory indexing exists;
- MPPI consumes per-step horizon reference sequences and preserves fixed-target
  compatibility;
- `/ctr/reference/path` publishes the full reference path;
- `/ctr/reference/horizon` publishes the current MPPI horizon;
- `/ctr/reference/tip` publishes the immediate target;
- `/ctr/controller/trajectory_metrics` publishes trajectory metrics;
- bounded simulation-only runtime tests executed for circle, ellipse, and helix.

Unit-test verification:

- `ctr_mppi_controller`: 77 focused tests passed;
- `ctr_bringup`: 25 focused tests passed;
- `ctr_sim`: 4 focused tests passed;
- total focused tests: 106 passed.

Build verification:

- clean isolated build used `build_m5c_verify`, `install_m5c_verify`, and
  `log_m5c_verify`;
- all 11 packages finished successfully.

ROS2 runtime smoke verification:

- circle, ellipse, and helix each ran for 12 seconds;
- no hardware node started;
- commands were finite and within configured limits;
- trajectory metrics published successfully;
- launch exited with code 0;
- no residual project process or zombie remained;
- clean Ctrl-C shutdown was verified.

Runtime smoke-test metrics:

- circle: MPPI observer RMSE approximately `4.9999495e-4 m`, zero-command
  baseline RMSE approximately `5.0000000e-4 m`, improvement approximately
  `0.0010%`, mean solve time approximately `1.141 s`, maximum solve time
  approximately `1.186 s`;
- ellipse: MPPI observer RMSE approximately `4.6773663e-4 m`, zero-command
  baseline RMSE approximately `4.8019163e-4 m`, improvement approximately
  `2.594%`, mean solve time approximately `1.165 s`, maximum solve time
  approximately `1.524 s`;
- helix: MPPI observer RMSE approximately `5.0390104e-4 m`, zero-command
  baseline RMSE approximately `5.0694540e-4 m`, improvement approximately
  `0.601%`, mean solve time approximately `1.230 s`, maximum solve time
  approximately `1.542 s`.

Performance limitations:

- the configured MPPI control period is `0.05 s`;
- observed mean solve time is approximately `1.1-1.23 s`;
- observed maximum solve time is approximately `1.19-1.54 s`;
- observed effective MPPI command publication rate is approximately
  `0.78-0.85 Hz`;
- reference-manager publication rate is approximately `20 Hz`;
- the controller significantly overruns the configured period;
- long solve time causes delayed state processing, delayed horizon processing,
  stale state/reference data at command publication, and an effective command
  rate below 1 Hz.

Performance caveats:

- RMSE values above are runtime smoke-test metrics, not rigorous
  performance-validation results;
- metric timestamps currently do not provide rigorous
  state-reference-command synchronization;
- zero-command baseline comparisons are smoke-test evidence only;
- tested trajectories are very small and near the initial tip;
- improvements over zero command are weak;
- the MPPI implementation is a simplified weighted random-shooting MPPI-style
  controller, not a theoretically complete or optimized MPPI implementation;
- the CTR model remains approximate;
- no physical tracking accuracy is verified.

### Milestone 5D: Quantitative evaluation framework

Acceptance status: satisfied for software-simulation quantitative evaluation.

Accepted implemented functions:

- observation-only evaluation node;
- experiment Start/Stop lifecycle;
- raw timestamped recording;
- state/reference/command alignment;
- alignment-gap rejection;
- tracking metrics;
- control metrics;
- timing metrics;
- data-quality metrics;
- strict JSON output;
- YAML metadata;
- CSV time-series output;
- Markdown reports;
- offline plots;
- baseline comparison;
- repeated-trial aggregation support;
- partial-directory preservation;
- atomic successful finalization;
- automatic pass/fail categories.

The evaluator acceptance boundary is observation-only:

- it creates no actuator-command publisher;
- it does not publish zero commands;
- it does not control hardware.

Accepted output categories:

- `functional_pass`;
- `numerical_safety_pass`;
- `data_quality_pass`;
- `baseline_improvement_pass`;
- `timing_pass`;
- `real_time_pass`;
- `physical_validation_pass`;
- `hardware_validation_pass`.

Accepted output structure:

- `metadata.yaml`;
- `summary.json`;
- raw CSV files;
- `aligned_samples.csv`;
- `report.md`;
- `comparison.json`;
- `comparison.md`;
- tracking and timing plots.

Verification evidence:

- strict JSON output verified;
- cumulative and summary control effort use the same integration rule;
- lifecycle and finalization guards verified;
- failed partial output is preserved;
- simulation, mock-hardware, and physical-hardware launch files keep
  evaluation disabled by default;
- physical hardware was not launched.

### Milestone 5D.1: Deterministic matched-run orchestration

Acceptance status: satisfied for deterministic software-simulation matched
baseline/candidate orchestration.

Accepted implemented functions:

- `ctr_run_evaluation` CLI;
- fresh simulator/evaluator process for each run;
- unique run identity;
- initial q and tip stability checks;
- evaluator recording before controller activity;
- scheduled reference epoch;
- pre-epoch first-reference-point behavior;
- formal evaluation window beginning at the reference epoch;
- delayed MPPI controller startup;
- zero-command baseline command guard;
- candidate first-command timing audit;
- `shared_environment_hash`;
- `controller_configuration_hash`;
- `orchestration_hash`;
- exact result-directory identity;
- canonical path containment;
- `experiment_group` path validation;
- owned process-group cleanup;
- baseline/candidate automatic comparison;
- distinct CLI exit codes for orchestration failure, invalid comparison, and
  optional required improvement.

Accepted comparison-validity checks:

- matching shared environment;
- matching trajectory geometry and timing;
- matching reference phase policy;
- matching evaluation duration;
- compatible initial q;
- compatible initial tip;
- zero prohibited baseline commands;
- candidate command after recording start;
- valid result identity and output.

Latest focused verification:

- `ctr_evaluation`: 94 tests passed;
- `ctr_mppi_controller`: 82 tests passed;
- `ctr_bringup`: 34 tests passed;
- `ctr_sim`: 4 tests passed;
- total focused tests: 214 passed;
- `git diff --check` passed.

Build evidence:

- a clean isolated 11-package build passed before the final Python-only
  experiment-group/path-containment fix;
- the final fix changed only `run_evaluation.py` and
  `test_run_evaluation.py`;
- focused tests passed after that fix.

Matched runtime evidence:

- pair 1 baseline RMSE `0.0004999606356 m`, candidate RMSE
  `0.0004999542515 m`, absolute RMSE difference approximately
  `-6.3840868e-09 m`, relative RMSE improvement approximately `0.0012769%`;
- pair 2 baseline RMSE `0.0004999584685 m`, candidate RMSE
  `0.0004999552086 m`, absolute RMSE difference approximately
  `-3.2599509e-09 m`, relative RMSE improvement approximately `0.0006520%`;
- both pairs had initial q difference `0.0`, initial tip difference `0.0`,
  initial state variation `0.0` during the accepted stability window,
  `scheduled_time` reference start, matching reference phase offset, shared
  environment compatibility passed, baseline nonzero command count `0`,
  candidate first command after recording and at or after the reference epoch,
  `comparison_valid: true`, clean process cleanup, no hardware node, and no
  orphan or zombie project process.

Acceptance caveats:

- deterministic evaluation orchestration is verified;
- baseline/candidate comparison validity is verified;
- quantitative reporting is verified;
- meaningful tracking improvement is not verified;
- performance verification is not achieved;
- timing verification failed;
- real-time capability is false;
- physical validation is false;
- hardware validation is false;
- MPPI timing remains far above the configured `0.05 s` control period;
- deadline overrun remains 100%;
- the tested circle is small and close to the initial tip;
- the approximately `0.0006520%` to `0.0012769%` RMSE improvement is
  negligible and is not accepted as meaningful controller-performance
  improvement;
- stronger nontrivial repeated experiments remain required.

Scope exclusions still apply:

- real-time trajectory control is not complete;
- hardware control is not complete;
- tactile control is not complete;
- obstacle avoidance is not complete;
- whole-body shape control is not complete;
- safety retreat is not complete;
- task management is not complete;
- learned residual dynamics are not complete;
- anatomical lumen planning is not complete.

## Milestone 6

- shape error is calculated;
- shape weight changes behavior;
- full backbone can be controlled, not only the tip.

### Milestone 6A: Straight cylindrical-lumen point-goal navigation

Acceptance status: satisfied for the verified straight-cylinder
software-simulation target matrix. Milestone 6A accepted the default target;
Milestone 6A.1 accepted repeatability for the default, axial, and lateral exact
targets with unchanged `cylinder_fast` and a `25.0 s` formal evaluation
duration. Real-time capability, physical accuracy, anatomical navigation, and
hardware deployment are not accepted as complete.

Accepted implemented functions:

- straight analytical cylindrical lumen;
- arbitrary normalized cylinder axis;
- complete-backbone radial and inlet/outlet end-cap validation;
- CTR outer-radius-aware clearance;
- safety-margin evaluation;
- invalid target rejection without silent clamping;
- whole-backbone MPPI running cost;
- whole-final-backbone terminal collision surcharge;
- cylinder visualization;
- target and closest-point visualization;
- cylinder-navigation metrics;
- automatic baseline/candidate evaluation;
- automatic CSV, strict JSON, Markdown, and plot outputs.

Accepted provisional software-simulation defaults:

- radius: `0.030 m`;
- length: `0.120 m`;
- CTR outer radius: `0.0015 m`;
- safety margin: `0.0020 m`;
- default target: `[0.015, 0.005, 0.100] m`;
- goal tolerance: `0.003 m`;
- required hold duration: `0.5 s`.

These defaults are not measured physical or anatomical parameters.

Accepted `cylinder_fast` software-simulation profile:

- samples: `36`;
- horizon: `7`;
- rollout `dt`: `0.55 s`;
- controller period: `0.10 s`;
- insertion noise: `0.003`;
- rotation noise: `0.100`;
- tip and terminal weights: `15000`;
- control weight: `0.005`;
- smoothness weight: `0.01`.

This profile is not accepted as a real-time or optimal-control certification
profile.

Accepted verification evidence:

- `git diff --check` passed;
- `ctr_mppi_controller`: 121 tests passed;
- `ctr_model`: 3 tests passed;
- `ctr_sim`: 4 tests passed;
- `ctr_evaluation`: 107 tests passed;
- `ctr_bringup`: 44 tests passed;
- total focused tests: 279 passed;
- direct cylindrical-lumen tests: 28 passed;
- clean isolated 11-package build passed before the final Python-only terminal
  whole-backbone collision-cost correction;
- terminal whole-backbone correction passed focused tests and an offline
  deterministic verification;
- strict JSON passed;
- no hardware node started;
- process cleanup was clean.

Accepted default-target runtime evidence for `[0.015, 0.005, 0.100] m`:

| Seed | Success | Final error (m) | RMSE (m) | Time to goal (s) | Hold (s) | Min clearance (m) | Collisions | Mean solve (s) | Command rate (Hz) | RMSE improvement |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 11 | true | 0.001902 | 0.012120 | 18.91 | 1.08 | 0.003319 | 0 | 0.133 | 7.41 | 42.39% |
| 22 | true | 0.002263 | 0.013147 | 19.45 | 0.54 | 0.002732 | 0 | 0.138 | 7.14 | 37.51% |
| 33 | true | 0.000222 | 0.011970 | 17.31 | 2.68 | 0.003442 | 0 | 0.139 | 7.10 | 43.11% |

Accepted aggregate default-target evidence:

- success rate: `3/3`;
- mean final error: `0.001462 m`;
- final-error standard deviation: `0.001089 m`;
- mean RMSE: `0.012412 m`;
- worst minimum clearance: `0.002732 m`;
- total collisions: `0`.

Historical additional-target diagnostics are not accepted as goal-success
completion:

- pre-6A.1 axial and lateral 20 s diagnostics were collision-free but did not
  satisfy the `0.003 m` goal tolerance;
- those diagnostics predate the target-identity hardening and are superseded by
  the accepted exact-target Milestone 6A.1 matrix below.

Accepted Milestone 6A.1 exact-target identity behavior:

- requested targets are never silently replaced;
- sampled reachability is diagnostic only;
- published reference targets are verified against requested targets;
- requested and executed targets match before evidence is accepted;
- baseline and candidate targets match;
- target identity participates in comparison compatibility.

Accepted Milestone 6A.1 repeatability evidence:

- commit `8288fce6ec443f45ab3e264b82b15efa2e1d6f75`;
- unchanged `cylinder_fast`;
- no new robust profile;
- no MPPI parameter tuning;
- formal evaluation duration `25.0 s`;
- 3 targets by 3 deterministic seeds;
- 9/9 target-reaching success;
- 9/9 valid baseline comparisons;
- 0 radial collisions;
- 0 inlet/outlet collisions;
- 0 process-cleanup failures;
- all generated JSON files strict-parsed successfully.

Accepted Milestone 6A.1 target aggregates:

| Target | Success | Mean final error (m) | Mean RMSE (m) | Worst physical clearance (m) | Safety-margin acceptance |
|---|---:|---:|---:|---:|---|
| Default `[0.015, 0.005, 0.100]` | 3/3 | 0.000328 | 0.011158 | 0.002683 | pass |
| Axial `[0.019, 0.000, 0.105]` | 3/3 | 0.000453 | 0.012965 | 0.001707 | follow-up; full margin failed in 3/3 runs |
| Lateral `[0.010, 0.012, 0.095]` | 3/3 | 0.001744 | 0.012276 | 0.003712 | pass |

Overall accepted Milestone 6A.1 evidence:

- success `9/9`;
- mean final error `0.000842 m`;
- mean RMSE `0.012133 m`;
- mean time to goal `20.40 s`;
- mean solve time `0.1339 s`;
- mean command rate `7.38 Hz`;
- no collision, invalid comparison, or process-cleanup failure.

Acceptance caveats:

- Milestone 6A is functionally integrated;
- straight-cylinder default, axial, and lateral simulation runtime is verified
  for seeds 11, 22, and 33;
- collision-free behavior is verified only for the tested target/seed matrix;
- quantitative evaluation is verified;
- axial safety-margin robustness is not accepted as complete;
- sampled reachability is only a simulation sanity check;
- timing measurements remain descriptive performance output and should continue
  to be reported;
- real-time execution is not a prerequisite for current software-simulation
  navigation acceptance;
- deadline overrun remains about `100%`;
- real-time capability is false;
- physical accuracy is not verified;
- hardware deployment is not verified.

## Milestone 7

- minimum distance is calculated;
- collision penalty activates;
- hard collision is prevented in supported scenarios.

## Milestone 8

- simulated contact is detected;
- force thresholds work;
- force cost changes controller behavior.

## Milestone 9

- hard contact blocks MPPI command;
- emergency stop latches;
- stale state triggers stop;
- retreat is bounded and low speed.

## Hardware readiness

The project is ready for physical adaptation when:

- simulation is stable;
- safety tests pass;
- all commands use standard units;
- all hardware parameters are isolated in YAML;
- the MPPI controller contains no simulator-specific code;
- mock hardware uses the same ROS2 interface as real hardware.
