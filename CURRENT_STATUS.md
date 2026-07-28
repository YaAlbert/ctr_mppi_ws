# Current Project Status

Last updated: 2026-07-28

Status source: current Ubuntu 22.04 repository audit, focused test results,
clean isolated build results, Milestone 4 foreground ROS2 runtime smoke test,
Milestone 5 bounded simulation-only trajectory smoke tests, committed
Milestone 5D quantitative evaluation framework, committed Milestone 5D.1
deterministic matched-run orchestration, and Milestone 6A straight cylindrical
lumen navigation verification. Documentation only was updated for this status
refresh.

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
- Latest Milestone 6A clean isolated build result: 11 packages finished
  successfully before the final Python-only terminal whole-backbone
  collision-cost correction. The final correction changed only
  `cylindrical_lumen.py` and `test_cylindrical_lumen.py`; focused tests passed
  after that correction.
- Latest focused verification: `git diff --check` passed and 279 focused
  package tests passed across `ctr_mppi_controller`, `ctr_model`, `ctr_sim`,
  `ctr_evaluation`, and `ctr_bringup`.
- Latest matched runtime evidence: Milestone 6A default-target cylinder
  navigation ran three deterministic baseline/candidate pairs with valid
  comparisons, strict JSON outputs, clean process cleanup, no hardware node,
  and no orphan or zombie project process.
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
  - The safety, tactile, hardware, visualization, and state-estimator packages
    still contain placeholder runtime nodes.
  - Minimum fixed-target MPPI tip reaching is implemented and runtime verified
    against the approximate model.
  - Minimum trajectory-mode reference generation, horizon consumption, ROS2
    reference-manager integration, and trajectory metrics are functionally
    integrated and simulation smoke verified.
  - The `ctr_evaluation` package implements a reusable observation-only
    quantitative evaluation framework for software-simulation runs.
  - The `ctr_run_evaluation` CLI implements deterministic matched
    zero-command baseline and MPPI candidate orchestration for the verified
    software-simulation workflow.
  - Milestone 6A implements straight analytical cylindrical-lumen point-goal
    navigation in simulation with whole-backbone MPPI lumen costs,
    whole-final-backbone terminal collision surcharge, cylinder visualization,
    and cylinder-navigation evaluation metrics.
  - Milestone 5 trajectory tracking is not performance verified, not real-time
    capable, not physically validated, and not hardware validated.
  - MPPI shape, obstacle, tactile-force, and stability behavior is not
    complete.
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
| Milestone 5: tip trajectory tracking | Functionally integrated and runtime smoke verified; performance not verified | Focused tests passed: `ctr_mppi_controller` 77, `ctr_bringup` 25, `ctr_sim` 4, total 106; clean isolated build finished all 11 packages; 12 s circle, ellipse, and helix simulation-only runtime paths executed; `/ctr/reference/path`, `/ctr/reference/horizon`, `/ctr/reference/tip`, and `/ctr/controller/trajectory_metrics` published; commands were finite and within configured limits; no hardware node started; launch exited with code 0 and no residual project process or zombie remained | Real-time trajectory control is not verified; controller significantly overruns the configured 0.05 s period; timestamp-aligned metrics, stronger baseline synchronization, nontrivial trajectory experiments, MPPI profiling/optimization, and physical validation remain unresolved |
| Milestone 5D: quantitative evaluation framework | Functionally complete for software-simulation quantitative evaluation | Commit `59342a3`; observation-only evaluator; Start/Stop lifecycle; timestamped recording; alignment; strict JSON/YAML/CSV/Markdown/plot outputs; baseline comparison; repeated-trial aggregation; lifecycle, finalization, strict JSON, control-effort, and launch-default guards verified | Real-time, physical, and hardware validation remain unresolved; command-application timestamp and individual horizon-point timestamps are not available |
| Milestone 5D.1: deterministic matched-run orchestration | Functionally complete for software-simulation matched baseline/candidate orchestration | Commit `40c659159b37f49651fa9ea05ae2c6ddf07a2deb`; `ctr_run_evaluation`; 94 `ctr_evaluation`, 82 `ctr_mppi_controller`, 34 `ctr_bringup`, and 4 `ctr_sim` tests passed; two matched circle pairs passed compatibility with initial q/tip differences of `0.0` and clean process cleanup | Meaningful tracking improvement is not verified; MPPI remains non-real-time with 100% deadline overrun; stronger circle/ellipse/helix repeated batches and physical/hardware validation remain required |
| Milestone 6A: straight cylindrical-lumen point-goal navigation | Functionally integrated and runtime verified for the default software-simulation target | Straight analytical cylindrical lumen with arbitrary normalized axis, CTR outer-radius-aware clearance, complete-backbone radial/end-cap validation, safety-margin evaluation, whole-backbone MPPI running cost, whole-final-backbone terminal collision surcharge, cylinder/target/closest-point visualization, cylinder-navigation metrics, and automatic baseline/candidate evaluation; latest focused tests passed: `ctr_mppi_controller` 121, `ctr_model` 3, `ctr_sim` 4, `ctr_evaluation` 107, `ctr_bringup` 44, total 279; direct cylindrical-lumen tests 28; default target succeeded for seeds 11, 22, and 33 with zero collisions and valid comparisons | Broad multi-target robustness is not verified; axial and lateral tested targets remained collision-free but did not meet the 0.003 m goal tolerance within 20 s; deadline overrun remains 100%; real-time, physical accuracy, anatomical navigation, and hardware deployment remain unverified |

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
- [x] Circle, ellipse, and helix reference trajectory generation
- [x] Loop and hold-final trajectory horizon extraction
- [x] Elapsed-time trajectory indexing
- [x] MPPI per-step horizon reference consumption with fixed-target
  compatibility
- [x] ROS2 reference-manager publications for `/ctr/reference/path`,
  `/ctr/reference/horizon`, and `/ctr/reference/tip`
- [x] Trajectory metrics publication on `/ctr/controller/trajectory_metrics`
- [x] Bounded simulation-only runtime smoke tests for circle, ellipse, and
  helix trajectory modes
- [x] Observation-only `ctr_evaluation` evaluation node with experiment
  Start/Stop lifecycle
- [x] Raw timestamped state, tip, reference, command, and timing recording
- [x] State/reference/command alignment with alignment-gap rejection
- [x] Tracking, control, timing, and data-quality metrics
- [x] Strict JSON output, YAML metadata, stable CSV time-series output,
  Markdown reports, and offline tracking/timing plots
- [x] Baseline comparison and repeated-trial aggregation support
- [x] Partial-directory preservation, atomic successful finalization, and
  automatic pass/fail categories
- [x] Separate output categories: `functional_pass`,
  `numerical_safety_pass`, `data_quality_pass`,
  `baseline_improvement_pass`, `timing_pass`, `real_time_pass`,
  `physical_validation_pass`, and `hardware_validation_pass`
- [x] Evaluation disabled by default in simulation, mock-hardware, and
  physical-hardware launch files
- [x] `ctr_run_evaluation` CLI for deterministic matched
  zero-command baseline and MPPI candidate runs
- [x] Fresh simulator/evaluator process per matched run, unique run identity,
  initial q/tip stability checks, scheduled reference epoch, formal
  evaluation window, delayed MPPI controller startup, command guards,
  command-timing audit, exact result-directory identity, canonical path
  containment, `experiment_group` validation, owned process-group cleanup,
  and automatic comparison
- [x] Straight analytical cylindrical-lumen geometry with arbitrary normalized
  axis, CTR outer-radius-aware clearance, safety-margin evaluation, and valid
  target rejection without silent clamping
- [x] Complete-backbone radial and end-cap validation for cylinder navigation
- [x] Whole-backbone MPPI running lumen cost and whole-final-backbone terminal
  collision surcharge
- [x] Cylinder, target, current backbone, closest-to-wall point, and collision
  status visualization
- [x] Cylinder-navigation goal, lumen-safety, motion, timing, and data-quality
  metrics with automatic CSV, strict JSON, Markdown, and plot outputs
- [x] Automatic zero-command baseline versus MPPI candidate evaluation for
  cylinder-navigation simulation runs
- [x] Ubuntu clean isolated `colcon build` verification for all 11 packages
- [x] Installed ROS2 simulation launch path verified from `install_m5c_verify`
- [x] Conventional ROS2 Humble guarded shutdown verified for
  `/parameter_validator`, `/ctr_simulator`, and `/mppi_controller`
- [ ] Validated Python CTR model against MATLAB fixtures
- [ ] RViz2 runtime verification
- [ ] Safety state machine
- [ ] Tactile processing implementation
- [ ] Mock hardware feedback implementation
- [ ] Physical hardware driver
- [ ] Performance-verified real-time trajectory tracking
- [ ] Shape tracking
- [ ] Obstacle avoidance
- [ ] Tactile simulation

## Test results in current Ubuntu audit

- `git diff --check`: clean during Milestone 5C verification.
- `git diff --check`: passed during Milestone 5D.1 documentation source
  verification.
- Latest focused Milestone 5D.1 tests passed:
  - `ctr_evaluation`: 94 passed.
  - `ctr_mppi_controller`: 82 passed.
  - `ctr_bringup`: 34 passed.
  - `ctr_sim`: 4 passed.
  - Total focused tests: 214 passed.
- Latest focused Milestone 6A tests passed:
  - `ctr_mppi_controller`: 121 passed.
  - `ctr_model`: 3 passed.
  - `ctr_sim`: 4 passed.
  - `ctr_evaluation`: 107 passed.
  - `ctr_bringup`: 44 passed.
  - Total focused tests: 279 passed.
  - Direct cylindrical-lumen tests: 28 passed.
  - `git diff --check`: passed.
- Milestone 6A build and safety evidence:
  - a clean isolated 11-package build passed before the final Python-only
    terminal whole-backbone collision-cost correction;
  - the terminal whole-backbone correction passed focused tests and an offline
    deterministic verification;
  - strict JSON verification passed;
  - no hardware node started;
  - process cleanup was clean.
- Milestone 6A provisional software-simulation defaults:
  - straight-cylinder radius `0.030 m`;
  - length `0.120 m`;
  - CTR outer radius `0.0015 m`;
  - safety margin `0.0020 m`;
  - default target `[0.015, 0.005, 0.100] m`;
  - goal tolerance `0.003 m`;
  - required hold duration `0.5 s`.
  These values are not measured physical CTR, anatomical, or hardware
  parameters.
- Milestone 6A `cylinder_fast` software-simulation profile:
  - samples `36`;
  - horizon `7`;
  - rollout `dt` `0.55 s`;
  - controller period `0.10 s`;
  - insertion noise `0.003`;
  - rotation noise `0.100`;
  - tip and terminal weights `15000`;
  - control weight `0.005`;
  - smoothness weight `0.01`.
  This profile is for bounded software-simulation testing, not real-time or
  optimal-control certification.
- Milestone 6A default-target runtime evidence for target
  `[0.015, 0.005, 0.100] m`:
  - seed 11: success `true`, final error `0.001902 m`, RMSE `0.012120 m`,
    time to goal `18.91 s`, hold duration `1.08 s`, minimum clearance
    `0.003319 m`, collision count `0`, mean solve time `0.133 s`, command rate
    `7.41 Hz`, RMSE improvement `42.39%`;
  - seed 22: success `true`, final error `0.002263 m`, RMSE `0.013147 m`,
    time to goal `19.45 s`, hold duration `0.54 s`, minimum clearance
    `0.002732 m`, collision count `0`, mean solve time `0.138 s`, command rate
    `7.14 Hz`, RMSE improvement `37.51%`;
  - seed 33: success `true`, final error `0.000222 m`, RMSE `0.011970 m`,
    time to goal `17.31 s`, hold duration `2.68 s`, minimum clearance
    `0.003442 m`, collision count `0`, mean solve time `0.139 s`, command rate
    `7.10 Hz`, RMSE improvement `43.11%`;
  - aggregate: success rate `3/3`, mean final error `0.001462 m`, final-error
    standard deviation `0.001089 m`, mean RMSE `0.012412 m`, worst minimum
    clearance `0.002732 m`, total collisions `0`.
- Milestone 6A additional-target runtime evidence:
  - axial target `[0.019, 0.000, 0.105] m`: valid, deterministic sampled-
    reachability check passed, collision-free, final error `0.003776 m`, did
    not meet the `0.003 m` goal tolerance within `20 s`;
  - lateral target `[0.010, 0.012, 0.095] m`: valid, deterministic sampled-
    reachability check passed, collision-free, final error `0.004218 m`, did
    not meet the `0.003 m` goal tolerance within `20 s`.
- Milestone 5D.1 build evidence:
  - a clean isolated 11-package build passed before the final Python-only
    experiment-group/path-containment fix;
  - the final fix changed only `run_evaluation.py` and
    `test_run_evaluation.py`;
  - focused tests passed after that fix.
- Milestone 5D verification evidence:
  - strict JSON output was verified;
  - cumulative and summary control effort use the same integration rule;
  - lifecycle and finalization guards were verified;
  - failed partial output is preserved;
  - generated `evaluation_results` are ignored;
  - simulation, mock-hardware, and physical-hardware launch files keep
    evaluation disabled by default;
  - physical hardware was not launched.
- Milestone 5D generated output structure:
  - `metadata.yaml`;
  - `summary.json`;
  - raw CSV files;
  - `aligned_samples.csv`;
  - `report.md`;
  - `comparison.json`;
  - `comparison.md`;
  - tracking and timing plots.
- Milestone 5D.1 matched circle runtime evidence:
  - pair 1 baseline RMSE `0.0004999606356 m`, candidate RMSE
    `0.0004999542515 m`, absolute RMSE difference approximately
    `-6.3840868e-09 m`, relative RMSE improvement approximately
    `0.0012769%`;
  - pair 2 baseline RMSE `0.0004999584685 m`, candidate RMSE
    `0.0004999552086 m`, absolute RMSE difference approximately
    `-3.2599509e-09 m`, relative RMSE improvement approximately
    `0.0006520%`;
  - both pairs had initial q difference `0.0`, initial tip difference `0.0`,
    initial state variation `0.0` during the accepted stability window,
    `scheduled_time` reference start, matching reference phase offset, shared
    environment compatibility passed, baseline nonzero command count `0`,
    candidate first command after recording and at or after the reference
    epoch, `comparison_valid: true`, clean process cleanup, no hardware node,
    and no orphan or zombie project process.
- Focused package-local tests passed during Milestone 5C verification:
  - `ctr_bringup`: 25 passed.
  - `ctr_mppi_controller`: 77 passed.
  - `ctr_sim`: 4 passed.
  - Total focused tests: 106 passed.
- Clean isolated build passed:
  - command used `build_m5c_verify`, `install_m5c_verify`, and
    `log_m5c_verify`;
  - result: 11 packages finished successfully.
- Milestone 4 foreground PTY ROS2 fixed-target runtime smoke test passed:
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
- Milestone 5 bounded simulation-only trajectory smoke tests passed for runtime
  integration:
  - circle, ellipse, and helix each ran for 12 seconds;
  - `/ctr/reference/path`, `/ctr/reference/horizon`, `/ctr/reference/tip`, and
    `/ctr/controller/trajectory_metrics` were observed;
  - commands were finite and remained within configured limits;
  - no hardware node started;
  - launch exited with code 0;
  - no residual project process or zombie remained.
- Milestone 5 runtime smoke-test metrics, not rigorous performance-validation
  results:
  - circle: MPPI observer RMSE approximately `4.9999495e-4 m`, zero-command
    baseline RMSE approximately `5.0000000e-4 m`, improvement approximately
    `0.0010%`, mean solve time approximately `1.141 s`, maximum solve time
    approximately `1.186 s`;
  - ellipse: MPPI observer RMSE approximately `4.6773663e-4 m`, zero-command
    baseline RMSE approximately `4.8019163e-4 m`, improvement approximately
    `2.594%`, mean solve time approximately `1.165 s`, maximum solve time
    approximately `1.524 s`;
  - helix: MPPI observer RMSE approximately `5.0390104e-4 m`, zero-command
    baseline RMSE approximately `5.0694540e-4 m`, improvement approximately
    `0.601%`, mean solve time approximately `1.230 s`, maximum solve time
    approximately `1.542 s`.
- Milestone 5 performance limitation:
  - configured MPPI control period is `0.05 s`;
  - observed effective MPPI command publication rate is approximately
    `0.78-0.85 Hz`;
  - reference-manager publication rate is approximately `20 Hz`;
  - long solve time causes delayed state processing, delayed horizon
    processing, stale state/reference data at command publication, and an
    effective command rate below 1 Hz.
- Historical test-discovery finding retained: top-level
  `python3 -B -m unittest discover -s src` previously discovered zero tests.

## Current priority

Milestone 6A is functionally integrated for straight cylindrical-lumen
point-goal navigation in software simulation. The default target passed three
deterministic matched baseline/candidate evaluations with valid comparisons,
zero collisions, strict JSON outputs, and clean process cleanup. Collision-free
behavior is verified only for the tested cases.

Broad multi-target robustness is not verified. The axial and lateral additional
targets were valid, sampled reachable, and collision-free, but did not reach the
`0.003 m` goal tolerance within `20 s`. MPPI remains non-real-time: mean solve
time was approximately `0.133-0.139 s` against a `0.10 s` cylinder profile
period, and deadline overrun remained `100%`. Physical accuracy, anatomical
navigation, and hardware deployment remain unverified. The next safe technical
task after documentation review is Milestone 6A.1 multi-target robustness, not
hardware work.

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
- High: MPPI solve time remains far above the configured 0.05 s control period,
  and Milestone 6A still has 100% deadline overrun against its 0.10 s cylinder
  profile period; the controller is not real-time capable.
- High: Milestone 6A broad multi-target robustness is not verified; the axial
  and lateral tested targets did not satisfy the 0.003 m goal tolerance within
  20 s.
- High: straight-cylinder dimensions, CTR outer radius, safety margin, and
  point-goal defaults are provisional software-simulation values, not measured
  physical or anatomical parameters.
- High: command/state/reference causal timing remains limited because no
  command-application timestamp exists and horizon points do not have
  individual timestamps.
- Medium: deterministic zero-command baseline synchronization is resolved only
  for the verified software-simulation matched-run workflow; hardware and
  physical experiment synchronization remain unresolved.
- Medium: tested matched trajectories are very small and near the initial tip;
  stronger circle, ellipse, helix, and nontrivial repeated batches are still
  required.
- Medium: deterministic cross-process ROS_DOMAIN_ID reservation or locking is
  not implemented for concurrent `ctr_run_evaluation` invocations.
- Medium: CRLF line endings remain in documentation, YAML, and selected
  MATLAB/data files.
- Medium: package maintainer and license metadata remain placeholders.
- Low: `_destroy_node()` currently catches broad `Exception` during cleanup.
  This did not block verified shutdown, but should be reviewed as a future
  code-quality item.
