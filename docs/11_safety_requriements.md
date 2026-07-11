# Safety Requirements

## General principle

The safety supervisor shall be independent from MPPI.

A high MPPI cost is not a substitute for a hard safety stop.

## Safety states

- INITIALIZING
- READY
- FREE_MOTION
- SOFT_CONTACT
- HARD_CONTACT
- RETREATING
- REPLANNING
- EMERGENCY_STOP
- FAULT

## FREE_MOTION

Conditions:

- valid state;
- valid tactile data;
- no hard contact;
- communication healthy;
- command within limits.

Behavior:

- pass the limited MPPI command.

## SOFT_CONTACT

Condition:

warning_threshold <= force < stop_threshold

Behavior:

- reduce maximum velocity;
- increase force cost;
- increase obstacle cost;
- reduce command magnitude;
- allow controlled MPPI motion.

## HARD_CONTACT

Condition:

force >= stop_threshold

Behavior:

- reject MPPI command;
- output zero velocity;
- latch hard-contact status;
- wait for retreat or operator command.

## RETREATING

Behavior:

- execute a low-speed bounded retreat;
- monitor force continuously;
- stop retreat when force falls below release threshold;
- return to replanning or ready state.

## EMERGENCY_STOP

Behavior:

- immediately publish zero command;
- latch stop state;
- require an explicit reset;
- do not automatically restart.

## FAULT

Fault conditions include:

- stale joint state;
- stale tactile state;
- NaN or Inf values;
- communication timeout;
- invalid model output;
- invalid command dimensions;
- motor fault;
- sensor fault.

## Required safety checks

Before sending a command:

- insertion-position limits;
- rotation-position limits;
- insertion-velocity limits;
- rotation-velocity limits;
- acceleration limits;
- tactile warning and stop;
- state freshness;
- command freshness;
- communication health;
- finite numerical values.