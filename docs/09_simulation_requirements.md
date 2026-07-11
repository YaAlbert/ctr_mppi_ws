# Simulation Requirements

## Simulation objective

The simulation shall provide a closed-loop environment for validating:

- CTR forward motion;
- MPPI reaching;
- trajectory tracking;
- whole-body shape tracking;
- obstacle avoidance;
- tactile contact handling;
- safety-state transitions.

## Minimum simulation state

The simulation node shall maintain:

- q;
- q_dot;
- backbone points;
- tip position;
- simulated time;
- actuator state;
- simulated tactile state.

## Initial actuator model

q_dot_actual =
    clip(
        q_dot_command,
        velocity_limits
    )

q_next =
    q_current + dt * q_dot_actual

## Extended actuator model

Later versions shall support:

- first-order actuator lag;
- command delay;
- acceleration limits;
- backlash;
- dead zone;
- friction;
- noise;
- dropped commands;
- encoder quantization.

## Environment representation

The first version shall support:

- target point;
- reference path;
- sphere obstacle;
- cylinder obstacle;
- moving sphere obstacle.

Later versions may support:

- lumen centerline;
- lumen wall;
- signed distance field;
- mesh environment.

## RViz visualization

The simulation shall display:

- CTR backbone as LINE_STRIP;
- downsampled backbone points;
- tip marker;
- target marker;
- reference trajectory;
- obstacles;
- safety-distance marker;
- contact marker;
- coordinate frames.

## Simulation scenarios

1. Fixed target reaching
2. Circular trajectory
3. Elliptical trajectory
4. Helical trajectory
5. Whole-body shape matching
6. Static obstacle avoidance
7. Dynamic obstacle avoidance
8. Simulated soft contact
9. Simulated hard contact
10. Retreat and replanning