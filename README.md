# CTR MPPI ROS2 Project

## Project goal

This project develops a ROS2 Humble control framework for a
three-tube Concentric Tube Robot equipped with a tip tactile sensor.

The first implementation target is simulation.

The final target is to deploy the same high-level controller to a
physical CTR system by replacing the simulation actuator and sensor
interfaces with hardware drivers.

## Main capabilities

- CTR forward-kinematics simulation
- MPPI tip reaching
- Tip trajectory tracking
- Whole-body backbone tracking
- Obstacle and lumen-wall avoidance
- Tactile-contact-aware control
- Safety supervision
- RViz2 visualization
- ROS2 data logging and evaluation
- Future physical hardware deployment

## Platform

- Ubuntu 22.04
- ROS2 Humble
- Python 3
- NumPy-based initial implementation
- Optional PyTorch or GPU acceleration in later versions

## Control variables

The CTR configuration is:

q = [rho1, rho2, rho3, theta1, theta2, theta3]

where:

- rho_i is insertion of tube i in meters
- theta_i is rotation of tube i in radians

The initial MPPI control command is:

u = q_dot

## Development strategy

1. Validate the CTR model outside ROS2.
2. Implement the model in Python.
3. Build a ROS2 simulation loop.
4. Implement minimum MPPI tip reaching.
5. Add trajectory and whole-body control.
6. Add obstacle and tactile simulation.
7. Add safety supervision.
8. Add mock hardware.
9. Replace mock hardware with physical drivers.