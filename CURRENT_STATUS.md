# Current Project Status

Last updated: 2026-07-10

Status source: repository audit. No source code has been modified for this
status update.

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

### Python code

- Available: No.
- Folder: `src/` exists but is empty.
- Main modules: None.
- Working functions: None.
- Known issues:
  - No Python `forward_kinematics(q, params)` implementation exists.
  - No parameter loader or validation layer exists.
  - No MPPI, safety, tactile, simulation, hardware, visualization, or
    evaluation modules exist.

### ROS2 workspace

- Available: No usable ROS2 package skeleton yet.
- ROS2 version: Target is ROS2 Humble, per project documentation.
- Ubuntu version: Target is Ubuntu 22.04, per project documentation.
- Existing packages: None detected.
- Build status: Not buildable. No `package.xml`, `setup.py`,
  `CMakeLists.txt`, launch files, messages, or services are present.

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

## Current completed functions

- [x] MATLAB simplified CTR forward model for offline analysis
- [x] MATLAB backbone sampling utility for offline analysis
- [x] MATLAB vessel centerline analysis utilities
- [x] MATLAB coarse workspace and local segment fitting scripts
- [ ] Python CTR forward model
- [ ] Backbone visualization in ROS2/RViz2
- [ ] ROS2 state publisher
- [ ] MPPI core
- [ ] Tip reaching
- [ ] Trajectory tracking
- [ ] Shape tracking
- [ ] Obstacle avoidance
- [ ] Tactile simulation
- [ ] Safety state machine
- [ ] Mock hardware
- [ ] Physical hardware

## Current priority

Milestone 1: create the ROS2 skeleton packages and interfaces while keeping
simulation and hardware paths behind the same state and command contracts.

The first implementation after the skeleton should be Milestone 2: port the
simplified MATLAB CTR model to a Python `forward_kinematics(q, params)`
interface using SI units at the public boundary.

## Current blockers

- Critical: no ROS2 packages or custom interfaces exist.
- Critical: no Python model, parameter loader, or tests exist.
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
- High: no MATLAB-to-Python validation fixture or test harness exists yet.
- High: config YAML files contain placeholder values without all required
  structured TODO IDs in the parameter files themselves.
