# Development Milestones

## Milestone 0: Input review

- inspect existing MATLAB code;
- inspect existing Python code;
- inspect available datasets;
- identify usable model functions;
- identify missing parameters;
- update unresolved item list.

## Milestone 1: ROS2 skeleton

- create packages;
- create messages and services;
- create YAML files;
- create launch files;
- create placeholder nodes;
- verify colcon build.

## Milestone 2: CTR model

- implement model interface;
- implement temporary approximate model;
- add unit tests;
- create MATLAB comparison script;
- validate dimensions and units.

## Milestone 3: ROS2 simulation

- implement simulation state;
- implement q update;
- publish backbone and tip;
- publish joint state;
- visualize in RViz2.

## Milestone 4: Minimum MPPI

- fixed target reaching;
- tip cost;
- control cost;
- smoothness cost;
- hard joint limits;
- controller metrics.

## Milestone 5: Trajectory tracking

Status: functionally integrated and runtime smoke verified for simulation only;
performance not verified; not real-time capable; not physically or hardware
validated.

Implemented:

- reference path manager;
- horizon reference sequence;
- circle, ellipse and helix trajectory generation;
- loop and hold-final horizon extraction;
- elapsed-time trajectory indexing;
- MPPI per-step horizon reference consumption;
- fixed-target compatibility;
- `/ctr/reference/path`;
- `/ctr/reference/horizon`;
- `/ctr/reference/tip`;
- `/ctr/controller/trajectory_metrics`;
- bounded simulation-only runtime tests.

Current limitations:

- the configured MPPI control period is 0.05 s, but observed mean solve time is
  approximately 1.1-1.23 s and observed maximum solve time is approximately
  1.19-1.54 s;
- effective MPPI command publication is approximately 0.78-0.85 Hz while the
  reference manager publishes at approximately 20 Hz;
- long solves delay state and horizon processing, so commands can be published
  using stale state/reference data;
- trajectory metrics are runtime smoke-test evidence only and are not
  timestamp-synchronized strongly enough for rigorous tracking-performance
  claims;
- zero-command baseline comparisons are smoke-test evidence only;
- the tested trajectories are very small and near the initial tip;
- the MPPI core is a simplified weighted random-shooting MPPI-style controller,
  not a theoretically complete or optimized MPPI implementation;
- the CTR model remains approximate and no physical tracking accuracy is
  verified.

### Milestone 5D: Quantitative evaluation framework

Status: functionally complete for software-simulation quantitative evaluation.

Milestone 5D implements a reusable framework under `ctr_evaluation` with:

- observation-only evaluation node;
- experiment Start/Stop lifecycle;
- raw timestamped recording;
- state/reference/command alignment and alignment-gap rejection;
- tracking, control, timing, and data-quality metrics;
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

The evaluator creates no actuator-command publisher, does not publish zero
commands, does not control hardware, and remains observation-only.

Output categories are reported separately:

- `functional_pass`;
- `numerical_safety_pass`;
- `data_quality_pass`;
- `baseline_improvement_pass`;
- `timing_pass`;
- `real_time_pass`;
- `physical_validation_pass`;
- `hardware_validation_pass`.

Generated output structure includes:

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

Status: functionally complete for deterministic software-simulation matched
baseline/candidate orchestration.

Milestone 5D.1 implements:

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

Comparison validity requires matching shared environment, trajectory geometry
and timing, reference phase policy, evaluation duration, initial q, initial
tip, zero prohibited baseline commands, candidate command after recording
start, and valid result identity/output.

Verification evidence:

- latest commit: `40c659159b37f49651fa9ea05ae2c6ddf07a2deb`
  (`Milestone 05D.1: orchestrate matched evaluation runs`);
- latest focused tests: `ctr_evaluation` 94, `ctr_mppi_controller` 82,
  `ctr_bringup` 34, `ctr_sim` 4, total 214 passed;
- `git diff --check` passed;
- a clean isolated 11-package build passed before the final Python-only
  experiment-group/path-containment fix;
- the final fix changed only `run_evaluation.py` and
  `test_run_evaluation.py`;
- focused tests passed after that fix.

Two matched circle experiment pairs passed compatibility validation:

- pair 1: baseline RMSE `0.0004999606356 m`, candidate RMSE
  `0.0004999542515 m`, absolute RMSE difference approximately
  `-6.3840868e-09 m`, relative RMSE improvement approximately `0.0012769%`;
- pair 2: baseline RMSE `0.0004999584685 m`, candidate RMSE
  `0.0004999552086 m`, absolute RMSE difference approximately
  `-3.2599509e-09 m`, relative RMSE improvement approximately `0.0006520%`;
- both pairs had initial q difference `0.0`, initial tip difference `0.0`,
  initial state variation `0.0` during the accepted stability window,
  `scheduled_time` reference start, matching reference phase offset, shared
  environment compatibility passed, baseline nonzero command count `0`,
  candidate first command after recording and at or after the reference epoch,
  `comparison_valid: true`, clean process cleanup, no hardware node, and no
  orphan or zombie project process.

Current limitations:

- deterministic evaluation orchestration is verified;
- baseline/candidate comparison validity is verified;
- quantitative reporting is verified;
- meaningful tracking improvement is not verified;
- performance verification is not achieved;
- timing verification failed;
- real-time capability is false;
- physical validation is false;
- hardware validation is false;
- deadline overrun remains 100%;
- MPPI timing remains far above the configured 0.05 s control period;
- the tested circle is small and close to the initial tip;
- stronger circle, ellipse, helix, and nontrivial repeated experiment batches
  remain required.

## Milestone 6: Whole-body control

- reference backbone;
- backbone resampling;
- shape cost;
- shape-constrained reaching.

## Milestone 7: Obstacles

- static sphere;
- cylinder;
- moving obstacle;
- minimum backbone clearance;
- obstacle cost.

## Milestone 8: Tactile simulation

- simulated force;
- calibration pipeline;
- filtering;
- contact state;
- force cost.

## Milestone 9: Safety supervisor

- safety state machine;
- command limiter;
- hard-contact stop;
- emergency stop;
- retreat;
- stale-state handling.

## Milestone 10: Nonideal simulation

- actuator lag;
- acceleration limits;
- noise;
- latency;
- packet dropout;
- encoder quantization.

## Milestone 11: Mock hardware

- hardware-compatible node;
- mock motor feedback;
- watchdog;
- diagnostics;
- hardware launch mode.

## Milestone 12: Physical adaptation

- motor driver;
- encoder scaling;
- tactile driver;
- homing;
- physical limits;
- low-speed commissioning.
