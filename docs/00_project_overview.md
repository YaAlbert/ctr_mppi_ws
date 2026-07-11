# Project Overview

## Background

The robot is a three-tube concentric tube robot made from compliant
or soft tubular elements. The tip is equipped with a tactile sensor.

The system is intended to perform navigation through a confined
environment while controlling both:

- the tip position;
- the full backbone shape.

## Main objective

Develop a ROS2-based MPPI controller that can:

1. move the CTR tip toward a target;
2. track a three-dimensional reference trajectory;
3. regulate the entire robot backbone;
4. avoid obstacles and unsafe wall contact;
5. react to tactile contact;
6. respect physical actuator and tube constraints;
7. operate in simulation and later on physical hardware.

## Architecture principle

The MPPI controller must not directly depend on:

- the simulation engine;
- a specific motor driver;
- a specific tactile sensor;
- a specific communication protocol.

The controller receives a normalized CTR state and outputs a
normalized six-dimensional joint command.

Simulation and hardware nodes shall implement the same ROS2
interfaces.