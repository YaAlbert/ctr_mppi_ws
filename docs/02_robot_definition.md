# Robot Definition

## Robot type

The robot is a three-tube concentric tube robot.

## Generalized coordinates

q = [
    rho1,
    rho2,
    rho3,
    theta1,
    theta2,
    theta3
]

where:

- rho_i: insertion displacement of tube i, unit meter;
- theta_i: axial rotation of tube i, unit radian.

## Generalized velocity

q_dot = [
    rho1_dot,
    rho2_dot,
    rho3_dot,
    theta1_dot,
    theta2_dot,
    theta3_dot
]

## Backbone state

The CTR backbone is represented by N ordered points:

P = [p_1, p_2, ..., p_N]

where each p_i is a 3D point.

Shape vector:

x = [
    p_1x, p_1y, p_1z,
    ...
    p_Nx, p_Ny, p_Nz
]

## Tip state

The tip state shall include:

- tip position;
- optional tip orientation;
- optional linear velocity;
- optional angular velocity.

## Initial state definition

The initial state shall be loaded from configuration.

No initial insertion, rotation or tube geometry value shall be
hard-coded in source files.