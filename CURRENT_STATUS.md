# Current Project Status

Last updated: 2026-07-11

Status source: current Ubuntu 22.04 repository audit and build-readiness
diagnosis. Documentation only was updated for this status refresh.

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
- `build/`: absent.
- `install/`: absent.
- `log/`: present and generated locally on Ubuntu by colcon inspection
  commands. It is not evidence of a successful build.
- Isolated `colcon build`: not run yet. Approval was requested before running
  the proposed isolated build.
- Current build failures: no `colcon build` failure has been observed because
  no build has been run in this Ubuntu workspace.
- Current build-readiness failure: `rosdep check --from-paths src --ignore-src`
  cannot resolve the declared key `ament_python` for the Python packages.
  `python3-numpy`, `python3-yaml`, `ament_cmake`, and `rclpy` resolve or are
  installed.

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
  - MPPI shape, obstacle, tactile-force, and stability costs are present only
    as disabled interfaces.
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
- Build status: not verified by `colcon build` in Ubuntu.
- Package discoverability after install: not verified. `ros2 pkg prefix`
  cannot find source packages before a build/install workspace is sourced.
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
| Milestone 1: ROS2 skeleton | Present but unverified | 11 packages discovered by `colcon list`; package manifests, setup files, interfaces, YAML files, launch files, and placeholder nodes are present | Successful isolated `colcon build`; source isolated install; verify `ros2 pkg prefix` and `ros2 interface show`; launch smoke tests; resolve or justify `ament_python` rosdep issue |
| Milestone 2: CTR model | Partially complete | `ApproximateCTRModel.forward_kinematics(q)` exists; package-local model tests pass; output shape and finite-value checks exist | Port or validate against MATLAB fixtures; decide source-of-truth tube parameters; add MATLAB-to-Python comparison tests and thresholds |
| Milestone 3: ROS2 simulation | Partially complete | `CTRSimulationCore` and `simulator_node.py` exist; simulation core tests pass; node publishes state/backbone/tip/marker topics in source | Build and launch simulation in Ubuntu; verify topic publication frequency, units, axes, RViz markers, and command path through safety-compatible interfaces |

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
- [ ] Ubuntu `colcon build` verification
- [ ] Installed ROS2 package discoverability
- [ ] Launch-file smoke tests
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

- `env PYTHONDONTWRITEBYTECODE=1 python3 -B -m unittest discover -s src`:
  0 tests discovered.
- Package-local direct tests:
  - `src/ctr_bringup/test`: 5 passed.
  - `src/ctr_model/test`: 3 passed.
  - `src/ctr_sim/test`: 4 passed.
  - `src/ctr_mppi_controller/test`: 8 passed.
- Total direct package-local tests run in audit: 20 passed.

## Current priority

Milestone 1 build verification in an isolated Ubuntu output set, without
deleting existing `build/`, `install/`, or `log/` directories.

Recommended isolated build command, pending approval:

```bash
bash -lc 'source /opt/ros/humble/setup.bash && colcon build --base-paths src --build-base build_ubuntu_audit --install-base install_ubuntu_audit --log-base log_ubuntu_audit --event-handlers console_direct+'
```

## Current blockers

- Critical: no Ubuntu `colcon build` result exists yet for the current source
  tree.
- Critical: `rosdep` cannot resolve the declared `ament_python` key in the
  current Ubuntu environment.
- Critical: generated custom interfaces have not been built or imported from
  an installed workspace.
- Critical: launch files have not been smoke-tested in Ubuntu.
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
