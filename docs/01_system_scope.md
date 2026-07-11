# System Scope

## Initial implementation scope

The initial implementation shall include:

- three-tube CTR state representation;
- CTR forward model;
- ROS2 simulation node;
- MPPI controller;
- single-target reaching;
- joint and velocity constraints;
- RViz2 backbone visualization;
- metrics publishing;
- rosbag2-compatible topics.

## Intermediate scope

The intermediate implementation shall include:

- trajectory tracking;
- whole-body shape tracking;
- static obstacle avoidance;
- moving obstacle avoidance;
- simulated tactile sensor;
- contact-aware cost;
- safety state machine;
- actuator delay and noise simulation;
- mock hardware interface.

## Final research scope

The final research system may include:

- physical motor drivers;
- encoder feedback;
- tactile sensor driver;
- force calibration;
- hardware watchdog;
- hardware-in-the-loop experiments;
- phantom-environment experiments;
- model mismatch compensation;
- learned residual dynamics.

## Out of scope for the first version

- clinical autonomous deployment;
- patient use;
- certified medical safety;
- tissue damage prediction;
- Neural ODE training;
- CT-image-based anatomical reconstruction;
- complex fluid or tissue simulation.