# ROS2 Architecture

## Packages

The project shall contain:

- ctr_interfaces
- ctr_model
- ctr_mppi_controller
- ctr_sim
- ctr_safety
- ctr_tactile
- ctr_state_estimator
- ctr_hardware
- ctr_viz
- ctr_evaluation
- ctr_bringup

## Data flow

reference_manager
    |
    v
mppi_controller
    |
    v
safety_supervisor
    |
    v
simulator or hardware_driver
    |
    v
state_estimator
    |
    +------> mppi_controller
    |
    +------> visualization
    |
    +------> evaluation

## Architectural restrictions

The MPPI controller shall not:

- access a motor serial port;
- access CAN directly;
- read a tactile device directly;
- modify the simulation state directly;
- publish motor-specific packet formats.

The simulator shall not:

- contain the MPPI optimization logic;
- calculate task-level controller weights;
- bypass the safety supervisor.

## Runtime modes

The project shall support:

- simulation;
- simulation_with_sensor_noise;
- hardware_in_loop;
- mock_hardware;
- physical_hardware.

All modes shall use compatible state and command messages.