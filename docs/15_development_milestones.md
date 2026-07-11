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

- reference path manager;
- horizon reference sequence;
- circle, ellipse and helix;
- trajectory metrics.

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