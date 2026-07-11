# Parameter Registry

## Parameter policy

All parameters shall be stored in YAML.

Source code shall not contain robot-specific numerical constants.

## Parameter categories

### Robot geometry

- tube length
- tube inner diameter
- tube outer diameter
- precurvature
- precurved-section length
- material properties
- insertion limits
- rotation limits

### Model parameters

- backbone point count
- numerical integration step
- solver type
- friction parameters
- model approximation mode

### MPPI parameters

- control frequency
- dt
- horizon
- sample count
- lambda
- noise standard deviation
- cost weights
- warm-start flag
- random seed

### Tactile parameters

- zero offset
- calibration scale
- filter cutoff
- contact threshold
- warning threshold
- stop threshold
- release threshold

### Hardware parameters

- communication protocol
- serial device
- baud rate
- CAN interface
- motor IDs
- encoder resolution
- gear ratio
- screw pitch
- direction signs
- watchdog timeout

### Safety parameters

- maximum speed
- maximum acceleration
- emergency stop timeout
- state timeout
- retreat distance
- retreat speed

## Unknown values

Unknown values shall:

1. use a conservative placeholder;
2. contain a structured TODO ID;
3. appear in unresolved_items.md;
4. be reported by Codex after each milestone.