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
