# MPPI Controller Requirements

## Controller input

The MPPI controller shall receive:

- current q;
- current q_dot;
- current backbone points;
- current tip position;
- target tip sequence;
- optional target backbone sequence;
- obstacle representation;
- tactile state;
- current safety state.

## Controller output

The initial MPPI output shall be:

q_dot_command with shape (6,)

## State propagation

For each rollout:

q_next = clip(
    q_current + dt * q_dot_command,
    q_min,
    q_max
)

backbone_next, tip_next =
    ctr_model.forward_kinematics(q_next)

## MPPI rollout

At each control cycle:

1. receive current state;
2. shift the previous nominal control sequence;
3. sample K perturbation sequences;
4. propagate each candidate sequence;
5. compute candidate cost;
6. calculate exponential importance weights;
7. update the nominal sequence;
8. publish the first command;
9. repeat at the next cycle.

## Initial controller capability

The first operational version shall only enable:

- tip target cost;
- control magnitude cost;
- control smoothness cost;
- joint limit enforcement.

The following terms shall initially be implemented but disabled:

- shape cost;
- obstacle cost;
- tactile force cost;
- stability cost.

## Implementation requirements

The MPPI core shall be independent from ROS2.

ROS2-specific code shall be limited to:

- receiving messages;
- converting messages to arrays;
- loading parameters;
- publishing commands and diagnostics.

## Performance monitoring

Each cycle shall record:

- MPPI solve time;
- minimum candidate cost;
- mean candidate cost;
- effective sample weight;
- command magnitude;
- command saturation state.