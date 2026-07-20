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
