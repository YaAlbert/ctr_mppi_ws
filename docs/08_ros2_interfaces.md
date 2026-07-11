# ROS2 Interfaces

## Topics

### /ctr/joint_state

Publishes:

- insertion positions;
- rotation positions;
- joint velocities;
- timestamp;
- state validity.

### /ctr/backbone

Publishes:

- ordered backbone points;
- frame_id;
- timestamp.

### /ctr/state

Publishes the controller-ready fused state:

- q;
- q_dot;
- backbone;
- tip pose;
- tactile force;
- contact state;
- state validity.

### /ctr/reference/tip

Publishes target tip pose.

### /ctr/reference/path

Publishes the target tip trajectory.

### /ctr/reference/backbone

Publishes the target backbone shape or shape sequence.

### /ctr/mppi_command

Publishes raw MPPI q_dot command.

### /ctr/safe_command

Publishes the safety-filtered q_dot command.

### /ctr/tactile/raw

Publishes uncalibrated tactile data.

### /ctr/tactile/state

Publishes:

- calibrated force;
- force magnitude;
- contact flag;
- warning flag;
- stop flag;
- sensor validity.

### /ctr/safety/status

Publishes the safety state.

### /ctr/controller/metrics

Publishes controller and experiment metrics.

## Services

- /ctr/set_task_mode
- /ctr/set_controller_weights
- /ctr/reset_controller
- /ctr/clear_fault
- /ctr/execute_retreat
- /ctr/set_reference
- /ctr/start_experiment
- /ctr/stop_experiment

## Message requirements

All custom messages shall include:

- timestamp;
- frame information when relevant;
- validity flag;
- diagnostic status when relevant.