# Tactile Sensor Requirements

## Processing pipeline

raw sensor data
    ->
calibration
    ->
filtering
    ->
force estimate
    ->
contact classification
    ->
MPPI and safety system

## Sensor abstraction

The tactile module shall not assume a fixed hardware model.

It shall support:

- scalar normal-force sensor;
- multi-channel tactile array;
- three-axis force sensor;
- simulated tactile sensor.

## Calibration

The initial generic linear calibration is:

F =
    scale * (raw_value - zero_offset) + bias

The actual calibration model may later be replaced with:

- polynomial model;
- lookup table;
- piecewise-linear model;
- matrix calibration.

## Tactile state

The tactile state shall include:

- raw values;
- filtered values;
- estimated force vector;
- force magnitude;
- contact flag;
- warning flag;
- stop flag;
- sensor health;
- last update time.

## Contact thresholds

The following thresholds shall be configurable:

- contact threshold;
- warning threshold;
- stop threshold.

## Simulation model

The simulated contact force may initially be calculated from
penetration depth:

F_contact =
    stiffness * penetration
  + damping * penetration_rate

The simulation shall support:

- Gaussian noise;
- zero drift;
- latency;
- sensor dropout.