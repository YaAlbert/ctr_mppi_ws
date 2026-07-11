# Hardware Adaptation Requirements

## Hardware strategy

The hardware phase shall replace the simulation execution layer
without replacing the MPPI controller.

## Common actuator interface

The common actuator interface shall provide:

- read_joint_state()
- send_velocity_command()
- stop()
- reset()
- is_healthy()
- get_diagnostics()

## Implementations

- SimulatedCTRActuator
- MockCTRActuator
- PhysicalCTRActuator

## Physical hardware responsibilities

The physical hardware module shall handle:

- motor communication;
- encoder decoding;
- motor ID mapping;
- motor direction mapping;
- unit conversion;
- homing;
- limit switches;
- motor fault detection;
- watchdog;
- emergency stop;
- actuator diagnostics.

## Unit conversion

The hardware layer shall convert between:

- meters and motor counts;
- radians and motor counts;
- meters per second and motor velocity units;
- radians per second and motor velocity units.

## Hardware restrictions

The physical driver shall not:

- calculate MPPI rollouts;
- implement reference tracking logic;
- calculate obstacle cost;
- directly modify controller weights.

## Hardware commissioning order

1. Test communication with motors disabled.
2. Read motor status.
3. Read encoders.
4. Test emergency stop.
5. Test one motor at low speed.
6. Test all motors independently.
7. Test insertion actuators.
8. Test rotation actuators.
9. Verify signs and scales.
10. Perform homing.
11. Run open-loop low-speed commands.
12. Run closed-loop joint commands.
13. Connect the CTR model.
14. Enable low-speed MPPI.
15. Add tactile safety.