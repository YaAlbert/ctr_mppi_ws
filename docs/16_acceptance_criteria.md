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