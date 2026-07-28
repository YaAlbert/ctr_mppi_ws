# Test and Evaluation Plan

## Unit tests

### Model

- valid q input;
- invalid q dimension;
- NaN and Inf rejection;
- deterministic output;
- output dimension;
- zero-state behavior;
- joint-limit handling.

### MPPI

- sample tensor dimension;
- rollout dimension;
- finite cost;
- weight normalization;
- command dimension;
- deterministic seed;
- command clipping.

### Cost functions

- zero tip error;
- increasing tip error;
- zero shape error;
- obstacle collision penalty;
- control smoothness;
- tactile warning penalty;
- joint-limit penalty.

### Safety

- free-motion transition;
- soft-contact transition;
- hard-contact transition;
- emergency-stop latching;
- stale-state fault;
- tactile timeout fault;
- retreat completion.

## Integration tests

- simulation node startup;
- controller node startup;
- valid command-state loop;
- target convergence;
- safety command interception;
- simulated tactile stop;
- rosbag-compatible publishing.

## Experiment metrics

- tip RMSE;
- mean tip error;
- maximum tip error;
- shape RMSE;
- minimum obstacle clearance;
- maximum tactile force;
- control effort;
- command variation;
- settling time;
- success rate;
- collision count;
- safety-stop count;
- MPPI solve time;
- control-loop frequency.

## Milestone 5D quantitative evaluation framework

Status: implemented and verified for software-simulation quantitative
evaluation under `ctr_evaluation`.

Implemented functions:

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

The evaluator is observation-only. It creates no actuator-command publisher,
does not publish zero commands, and does not control hardware.

Output categories are recorded separately:

- `functional_pass`;
- `numerical_safety_pass`;
- `data_quality_pass`;
- `baseline_improvement_pass`;
- `timing_pass`;
- `real_time_pass`;
- `physical_validation_pass`;
- `hardware_validation_pass`.

Generated evaluation output includes:

- `metadata.yaml`;
- `summary.json`;
- raw CSV files;
- `aligned_samples.csv`;
- `report.md`;
- `comparison.json`;
- `comparison.md`;
- tracking and timing plots.

Milestone 5D verification evidence:

- strict JSON output verified;
- cumulative and summary control effort use the same timestamp-interval
  integration rule;
- lifecycle and finalization guards verified;
- failed partial output is preserved;
- simulation, mock-hardware, and physical-hardware launch files keep
  evaluation disabled by default;
- physical hardware was not launched.

## Milestone 5D.1 deterministic matched-run orchestration

Status: implemented and verified for deterministic software-simulation
matched baseline/candidate orchestration.

Implemented functions:

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

Comparison validity requires:

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

Matched circle runtime evidence:

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

Performance classification:

- deterministic evaluation orchestration: verified;
- baseline/candidate comparison validity: verified;
- quantitative reporting: verified;
- meaningful tracking improvement: not verified;
- performance verification: not achieved;
- timing verification: failed;
- real-time capability: false;
- physical validation: false;
- hardware validation: false.

The approximately `0.0006520%` to `0.0012769%` RMSE improvement in the two
matched circle pairs is negligible and must not be described as meaningful
controller-performance improvement. The tested circle is small and close to the
initial tip. MPPI timing remains far above the configured `0.05 s` control
period, deadline overrun remains 100%, and stronger nontrivial repeated
software experiments are still required.

## Milestone 6A cylindrical-lumen navigation evaluation

Status: implemented and verified for the default straight-cylinder
software-simulation point-goal target. Broad multi-target robustness, real-time
capability, physical accuracy, anatomical navigation, and hardware deployment
are not verified.

Implemented evaluation coverage:

- straight analytical cylindrical lumen with arbitrary normalized axis;
- CTR outer-radius-aware wall clearance;
- complete-backbone radial and inlet/outlet end-cap validation;
- safety-margin evaluation;
- target rejection without silent clamping;
- whole-backbone MPPI running-cost checks;
- whole-final-backbone terminal collision surcharge checks;
- cylinder, target, and closest-backbone-point visualization;
- automatic zero-command baseline and MPPI candidate orchestration;
- automatic `metadata.yaml`, CSV, strict `summary.json`, Markdown reports,
  comparison files, and plots.

Cylinder-navigation metrics include:

- initial, final, mean, minimum, and RMSE tip-to-goal error;
- goal reached, time to goal, hold duration, and time inside tolerance;
- minimum backbone wall clearance;
- mean and p05 clearance;
- safety-margin violation count and duration;
- radial, inlet, and outlet collision counts;
- collision duration and maximum penetration depth;
- closest backbone-point index over time;
- tip and joint-space path length;
- path efficiency;
- control effort split into total, insertion, and rotation effort;
- solve-time statistics, command rate, deadline-overrun percentage, and
  real-time status;
- data-quality counters, rejected samples, missing backbone samples, and
  alignment gaps.

Provisional software-simulation defaults:

- radius: `0.030 m`;
- length: `0.120 m`;
- CTR outer radius: `0.0015 m`;
- safety margin: `0.0020 m`;
- default target: `[0.015, 0.005, 0.100] m`;
- goal tolerance: `0.003 m`;
- required hold duration: `0.5 s`.

These are not measured physical or anatomical parameters.

The `cylinder_fast` profile uses 36 samples, horizon 7, rollout `dt` `0.55 s`,
controller period `0.10 s`, insertion noise `0.003`, rotation noise `0.100`,
tip and terminal weights `15000`, control weight `0.005`, and smoothness weight
`0.01`. It is a bounded software-simulation profile, not a real-time or
optimal-control certification profile.

Latest verification:

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

Default target `[0.015, 0.005, 0.100] m` quantitative evidence:

| Seed | Success | Final error (m) | RMSE (m) | Time to goal (s) | Hold (s) | Min clearance (m) | Collisions | Mean solve (s) | Command rate (Hz) | RMSE improvement |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 11 | true | 0.001902 | 0.012120 | 18.91 | 1.08 | 0.003319 | 0 | 0.133 | 7.41 | 42.39% |
| 22 | true | 0.002263 | 0.013147 | 19.45 | 0.54 | 0.002732 | 0 | 0.138 | 7.14 | 37.51% |
| 33 | true | 0.000222 | 0.011970 | 17.31 | 2.68 | 0.003442 | 0 | 0.139 | 7.10 | 43.11% |

Aggregate default-target evidence:

- success rate: `3/3`;
- mean final error: `0.001462 m`;
- final-error standard deviation: `0.001089 m`;
- mean RMSE: `0.012412 m`;
- worst minimum clearance: `0.002732 m`;
- total collisions: `0`.

Additional target evidence:

- axial target `[0.019, 0.000, 0.105] m`: valid, deterministic sampled-
  reachability check passed, collision-free, final error `0.003776 m`, did not
  meet the `0.003 m` goal tolerance within `20 s`;
- lateral target `[0.010, 0.012, 0.095] m`: valid, deterministic sampled-
  reachability check passed, collision-free, final error `0.004218 m`, did not
  meet the `0.003 m` goal tolerance within `20 s`.

Milestone 6A test interpretation:

- deterministic software-simulation orchestration is verified;
- quantitative baseline/candidate comparison is verified;
- default-target collision-free point-goal behavior is verified for the tested
  seeds;
- broad target robustness is not verified;
- sampled reachability is only a simulation sanity check;
- deadline overrun remains `100%`;
- real-time capability, physical validation, and hardware validation remain
  false.

## Experimental sequence

1. Fixed target reaching
2. Multiple random target reaching
3. Circle tracking
4. Ellipse tracking
5. Helix tracking
6. Shape-constrained reaching
7. Static obstacle avoidance
8. Dynamic obstacle avoidance
9. Soft contact
10. Hard contact
11. Retreat
12. Communication delay
13. Sensor noise
14. Command dropout
