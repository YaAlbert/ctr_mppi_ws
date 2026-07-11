# Test and Evaluation Plan

## Unit tests

### Model

- valid q input;
- invalid q dimension;
- NaN and Inf rejection;
- deterministic output;
- output dimension;
- zero-state behavior;
- joint-limit handling.

### MPPI

- sample tensor dimension;
- rollout dimension;
- finite cost;
- weight normalization;
- command dimension;
- deterministic seed;
- command clipping.

### Cost functions

- zero tip error;
- increasing tip error;
- zero shape error;
- obstacle collision penalty;
- control smoothness;
- tactile warning penalty;
- joint-limit penalty.

### Safety

- free-motion transition;
- soft-contact transition;
- hard-contact transition;
- emergency-stop latching;
- stale-state fault;
- tactile timeout fault;
- retreat completion.

## Integration tests

- simulation node startup;
- controller node startup;
- valid command-state loop;
- target convergence;
- safety command interception;
- simulated tactile stop;
- rosbag-compatible publishing.

## Experiment metrics

- tip RMSE;
- mean tip error;
- maximum tip error;
- shape RMSE;
- minimum obstacle clearance;
- maximum tactile force;
- control effort;
- command variation;
- settling time;
- success rate;
- collision count;
- safety-stop count;
- MPPI solve time;
- control-loop frequency.

## Experimental sequence

1. Fixed target reaching
2. Multiple random target reaching
3. Circle tracking
4. Ellipse tracking
5. Helix tracking
6. Shape-constrained reaching
7. Static obstacle avoidance
8. Dynamic obstacle avoidance
9. Soft contact
10. Hard contact
11. Retreat
12. Communication delay
13. Sensor noise
14. Command dropout