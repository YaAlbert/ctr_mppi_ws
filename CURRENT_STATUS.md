# Current Project Status

Last updated: 2026-07-21

Status source: current Ubuntu 22.04 repository audit, focused test results,
clean isolated build results, and Milestone 4 foreground ROS2 runtime smoke
test. Documentation only was updated for this status refresh.

The current Ubuntu repository and runtime environment are the operational
source of truth. Earlier Windows-created work is treated as historical project
assets unless it has been verified in the current Ubuntu environment.

## Ubuntu environment and build-readiness summary

- OS: Ubuntu 22.04.5 LTS.
- ROS2: Humble, sourced from `/opt/ros/humble/setup.bash`.
- Python: 3.10.12.
- `ros2`: available at `/opt/ros/humble/bin/ros2`.
- `colcon`: available at `/usr/bin/colcon`.
- `pip3` and `python3 -m pip`: not available in the current environment.
- Latest clean isolated build verification:
  `build_shutdown_final`, `install_shutdown_final`, and `log_shutdown_final`.
- Latest clean isolated build result: 11 packages finished successfully.
- Latest runtime smoke test used the isolated `install_shutdown_final`
  workspace, not older `build/` or `install/` directories.
- Conventional ROS2 Humble guarded shutdown is in use for the Milestone 4
  simulation path. No custom SIGINT handler, SIGINT masking, or forced
  `KeyboardInterrupt` remains.
- Historical build-readiness finding retained: `rosdep check --from-paths src
  --ignore-src` could not resolve the declared key `ament_python` for the
  Python packages in the audited rosdep database. The clean isolated colcon
  build succeeded despite that rosdep declaration issue.

## Existing assets

### MATLAB code

- Available: Yes.
- Folders:
  - `data/matlab/Phase3_CTR in Free Space and Confined Environment`
  - `data/matlab/CTR_org_matlab`
- Main model entry function: `Robot.fkin(q_var)`
- Backbone sampling entry function:
  `sample_ctr_centerline(robot, q_var, ptsPerLink)`
- Input:
  - `q_var = [rho1 rho2 rho3 theta1 theta2 theta3]`
  - Phase 3 MATLAB scripts use `rho` in millimeters and `theta` in degrees.
- Output:
  - `Robot.fkin(q_var)` returns a 4x4 base-to-tip transform.
  - `sample_ctr_centerline(...)` returns an ordered `N x 3` backbone array
    in meters.
- Known issues:
  - MATLAB public inputs are not in the project-standard SI interface.
  - `rho1` is treated as a reference insertion in the Phase 3 model.
  - Tube geometry/material values are hard-coded in MATLAB scripts and
    conflict with YAML placeholder values.
  - The model is a simplified piecewise constant-curvature model, not a
    validated physical CTR model.
  - MATLAB hardware reference code uses Windows COM ports (`COM5`, `COM6`,
    `COM13`) and is not a verified Ubuntu ROS2 hardware interface.
  - MATLAB `.mat` files were created on `PCWIN64`; they are offline fixtures
    until loaded and validated in the Ubuntu workflow.

### Python code

- Available: Yes.
- Folder: `src/` contains ROS2 packages and Python modules.
- Main reusable modules:
  - `ctr_bringup.parameter_validation`
  - `ctr_model.approximate_model`
  - `ctr_sim.simulation_core`
  - `ctr_mppi_controller.cost_functions`
  - `ctr_mppi_controller.mppi_core`
- Known issues:
  - The Python CTR model is an approximate scaffold, not a validated port of
    `Robot.m`, `Tube.m`, and `sample_ctr_centerline.m`.
  - The safety, tactile, hardware, visualization, state-estimator, and
    evaluation packages still contain placeholder runtime nodes.
  - Minimum fixed-target MPPI tip reaching is implemented and runtime verified
    against the approximate model.
  - MPPI shape, obstacle, tactile-force, trajectory-tracking, and stability
    behavior is not complete.
  - Package-local unit tests pass when run directly, but top-level
    `unittest discover -s src` discovers zero tests.

### ROS2 workspace

- Available: Package skeleton and scaffolding are present.
- ROS2 version: Humble in the current Ubuntu environment.
- Ubuntu version: 22.04.5 LTS in the current environment.
- Existing packages discovered by `colcon list`:
  - `ctr_bringup`
  - `ctr_evaluation`
  - `ctr_hardware`
  - `ctr_interfaces`
  - `ctr_model`
  - `ctr_mppi_controller`
  - `ctr_safety`
  - `ctr_sim`
  - `ctr_state_estimator`
  - `ctr_tactile`
  - `ctr_viz`
- Interfaces: custom `.msg` and `.srv` files are present in
  `src/ctr_interfaces`.
- Launch files: `simulation.launch.py`, `mock_hardware.launch.py`, and
  `physical_hardware.launch.py` are present in `src/ctr_bringup/launch`.
- Build status: verified by a clean isolated `colcon build` using
  `build_shutdown_final`, `install_shutdown_final`, and `log_shutdown_final`.
  All 11 packages finished successfully.
- Package runtime after install: verified for the Milestone 4 simulation launch
  path using `install_shutdown_final`.
- Rosdep status: unresolved key `ament_python` in the current rosdep database.

### Data

- q samples:
  - `WorkspaceSamples.mat`: `q_log` shape `(2500, 6)`, MATLAB units mm/deg.
  - `CurvatureFeasibilityResults.mat`: `q_log` shape `(14580, 6)`,
    MATLAB units mm/deg.
  - `FitLocalSegmentResults.mat`: `searchLog` shape `(6860, 9)`,
    MATLAB units mm/deg plus fitting metrics.
- backbone samples:
  - `WorkspaceSamples.mat` contains `P_straight_m` and `P_bent_m`.
  - `FitLocalSegmentResults.mat` contains `P_ctr_local_mm` and
    `P_ctr_aligned_mm`.
- tip samples:
  - `WorkspaceSamples.mat` contains `tip_points_m`.
- vessel data:
  - `Centerline curve_3 (0).fcsv`: 3D Slicer FCSV, LPS coordinates, mm.
  - `Centerline quantification_2.csv`: radius, length, curvature, torsion,
    tortuosity, and endpoint fields.
  - `Endpoints_2.fcsv`: two endpoint fiducials, LPS coordinates, mm.
  - `All_vessel.stl`: binary STL from 3D Slicer, LPS coordinates.
  - `Centerline model_3.stl`: binary STL header with zero triangles.
- tactile data:
  - No logged tactile calibration dataset found.
  - MATLAB GUI expects pressure channels in either `p1,p2,j1x,j1y,sw1`
    or `p1,p2,j1x,j1y,j2x,j2y,j3x,j3y,sw1,sw2,sw3` serial formats.
- motor logs:
  - No motor logs found.
- calibration data:
  - No motor, encoder, tube, or tactile calibration files found.

## Milestone verification state

| Milestone | State | Ubuntu evidence | Evidence still needed |
|---|---|---|---|
| Milestone 1: ROS2 skeleton | Build verified; runtime partially verified | 11 packages discovered by `colcon list`; clean isolated build finished all 11 packages; the simulation launch path starts installed nodes from `install_shutdown_final` | Full launch coverage for mock hardware and physical hardware modes; explicit `ros2 pkg prefix` and `ros2 interface show` audit after future source changes; resolve or document `ament_python` rosdep policy |
| Milestone 2: CTR model | Partially complete | `ApproximateCTRModel.forward_kinematics(q)` exists; package-local model tests pass; output shape and finite-value checks exist | Port or validate against MATLAB fixtures; decide source-of-truth tube parameters; add MATLAB-to-Python comparison tests and thresholds |
| Milestone 3: ROS2 simulation | Partially complete; Milestone 4 loop runtime verified | `CTRSimulationCore` and `simulator_node.py` exist; simulation core tests pass; foreground runtime smoke verified `/ctr/state`, `/ctr/mppi_command`, `/ctr/safe_command`, and `/ctr/controller/metrics` while `/ctr_simulator` remained alive | Dedicated Milestone 3 acceptance audit for topic frequencies, units, axes, RViz marker visualization, and non-MPPI command paths |
| Milestone 4: minimum MPPI fixed-target reaching | Runtime verified complete for scoped minimum | Focused tests passed: `ctr_bringup` 19, `ctr_mppi_controller` 14, `ctr_sim` 4; clean isolated build finished all 11 packages; foreground PTY smoke test verified `/parameter_validator`, `/ctr_simulator`, `/mppi_controller`, MPPI command, safe command, state, metrics, clean Ctrl-C shutdown, and no tracebacks | Hardware execution remains disabled and unverified; approximate CTR model remains unresolved by `MODEL-004`; trajectory tracking, shape control, obstacle avoidance, tactile control, and hardware support are not part of Milestone 4 completion |

## Current implemented functions

- [x] MATLAB simplified CTR forward model for offline analysis
- [x] MATLAB backbone sampling utility for offline analysis
- [x] MATLAB vessel centerline analysis utilities
- [x] MATLAB coarse workspace and local segment fitting scripts
- [x] ROS2 package skeleton present in source
- [x] Custom ROS2 message and service files present in source
- [x] YAML parameter files present in source
- [x] Approximate Python CTR model scaffold
- [x] Python parameter loader and validation scaffold
- [x] Simulation core state update scaffold
- [x] ROS2 simulator node source scaffold
- [x] Minimum MPPI core scaffold for fixed-target reaching against the
  approximate model
- [x] Minimum MPPI fixed-target ROS2 runtime integration verified for
  simulation
- [x] Ubuntu clean isolated `colcon build` verification for all 11 packages
- [x] Installed ROS2 simulation launch path verified from `install_shutdown_final`
- [x] Conventional ROS2 Humble guarded shutdown verified for
  `/parameter_validator`, `/ctr_simulator`, and `/mppi_controller`
- [ ] Validated Python CTR model against MATLAB fixtures
- [ ] RViz2 runtime verification
- [ ] Safety state machine
- [ ] Tactile processing implementation
- [ ] Mock hardware feedback implementation
- [ ] Physical hardware driver
- [ ] Trajectory tracking
- [ ] Shape tracking
- [ ] Obstacle avoidance
- [ ] Tactile simulation

## Test results in current Ubuntu audit

- `git diff --check`: clean during Milestone 4 shutdown verification.
- Focused package-local tests passed during Milestone 4 verification:
  - `ctr_bringup`: 19 passed.
  - `ctr_mppi_controller`: 14 passed.
  - `ctr_sim`: 4 passed.
- Clean isolated build passed:
  - command used `build_shutdown_final`, `install_shutdown_final`, and
    `log_shutdown_final`;
  - result: 11 packages finished successfully.
- Foreground PTY ROS2 runtime smoke test passed:
  - nodes verified: `/parameter_validator`, `/ctr_simulator`,
    `/mppi_controller`;
  - hardware nodes: none started;
  - topics verified: `/ctr/state`, `/ctr/mppi_command`, `/ctr/safe_command`,
    `/ctr/controller/metrics`;
  - Ctrl-C was delivered to the actual foreground launch process group;
  - `ros2 launch` exited naturally with exit code 0;
  - all child nodes finished cleanly;
  - no project process or zombie remained;
  - no `KeyboardInterrupt` traceback, `rcl_shutdown already called`, rclpy
    exception, process death, or signal escalation occurred.
- Historical test-discovery finding retained: top-level
  `python3 -B -m unittest discover -s src` previously discovered zero tests.

## Current priority

Milestone 5 trajectory tracking is the next milestone after the verified
Milestone 4 minimum fixed-target reaching controller. Keep hardware execution
disabled until the hardware TODOs are resolved and separately commissioned.

## Current blockers

- Critical: no MATLAB-to-Python validation fixture or test harness exists yet.
- Critical: no source-of-truth physical tube geometry/material dataset is
  available.
- Critical: tactile hardware model, output units, and calibration are
  unknown.
- Critical: motor hardware model, protocol, encoder scaling, homing, and
  direction signs are unknown.
- High: MATLAB model inputs are mm/deg and relative to `rho1`, while the
  documented controller interface is meters/radians with independent
  insertions.
- High: vessel data are in 3D Slicer LPS millimeter coordinates; the
  transform into `world` or `base_link` is not defined.
- High: config YAML files contain placeholder values without all required
  structured TODO IDs in the parameter files themselves.
- Medium: CRLF line endings remain in documentation, YAML, and selected
  MATLAB/data files.
- Medium: package maintainer and license metadata remain placeholders.
- Low: `_destroy_node()` currently catches broad `Exception` during cleanup.
  This did not block verified shutdown, but should be reviewed as a future
  code-quality item.
