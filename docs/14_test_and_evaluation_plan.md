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
