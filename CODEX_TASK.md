# Codex Project Instructions

You are working on a ROS2 Humble project for a three-tube
Concentric Tube Robot controlled using Model Predictive Path Integral
control.

Before changing code, read:

- README.md
- CURRENT_STATUS.md
- all files under docs/
- all YAML files under config/
- existing source code
- existing tests

## Main architecture rule

Implement simulation first, but keep all interfaces compatible with
future physical hardware.

The MPPI controller must not directly depend on the simulator.

The simulator and the physical hardware node must use the same ROS2
state and command interfaces.

## Robot definition

q = [rho1, rho2, rho3, theta1, theta2, theta3]

rho is in meters.

theta is in radians.

The initial control command is:

u = q_dot

## Development rules

1. Work on one milestone only.
2. Do not implement future milestones unless specifically requested.
3. Do not invent undocumented robot parameters.
4. Use YAML values when physical values are unavailable.
5. Mark unknown values with structured TODO IDs.
6. Update docs/18_unresolved_items.md when new unknowns are found.
7. Preserve existing working code where possible.
8. Do not overwrite verified model logic without explaining why.
9. Add tests for every new mathematical or safety component.
10. Run build and tests before reporting completion.

## Parameter rules

Do not hard-code:

- geometry;
- material parameters;
- limits;
- sensor calibration;
- motor conversion factors;
- MPPI weights;
- sample count;
- horizon;
- control frequency;
- timeouts.

## Safety rules

No raw MPPI command may be sent to hardware.

All commands must pass through:

- numerical validation;
- position limits;
- velocity limits;
- acceleration limits;
- tactile safety;
- watchdog;
- communication-health check.

Hard contact and emergency stop must be enforced outside MPPI.

## Required response after every task

Return:

1. Summary
2. Files inspected
3. Files created
4. Files modified
5. Main implementation decisions
6. Commands used to build
7. Commands used to test
8. Test results
9. Remaining TODO IDs
10. Missing parameters
11. Known limitations
12. Recommended next task

Do not state that a task is complete if the build or required tests
did not pass.