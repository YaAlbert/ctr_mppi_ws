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
