# Unresolved Items

All defaults below are retained only as development placeholders until the
listed file, datasheet, calibration, or experiment resolves the associated
TODO ID.

## Audit-resolved items

These items were unknown in the original status documents and were clarified
by the repository audit. The IDs are retained so future implementation work
can reference the finding.

| ID | Item | Audit finding | Evidence needed to keep resolved | Module | Status |
|---|---|---|---|---|---|
| MODEL-001 | Existing MATLAB function entry point | `Robot.fkin(q_var)` for tip transform and `sample_ctr_centerline(robot, q_var, ptsPerLink)` for backbone sampling | Preserve these files or replace with a documented Python comparison fixture | Model | Resolved by audit |
| MODEL-002 | MATLAB input units | Phase 3 MATLAB public scripts use `rho` in mm and `theta` in degrees | Unit conversion test comparing MATLAB fixtures to Python SI inputs | Model | Resolved by audit |
| MODEL-003 | MATLAB backbone point ordering | `sample_ctr_centerline` emits ordered backbone points from base toward tip | Python port test against `WorkspaceSamples.mat` representative backbones | Model | Resolved by audit |

## Ubuntu milestone verification

| Milestone | Current state | Ubuntu evidence | Evidence needed for verified complete |
|---|---|---|---|
| Milestone 1: ROS2 skeleton | Build verified; runtime partially verified | 11 packages are discovered by `colcon list`; clean isolated build using `build_shutdown_final`, `install_shutdown_final`, and `log_shutdown_final` finished all 11 packages; installed simulation launch path starts the required Milestone 4 nodes | Full launch coverage for mock hardware and physical hardware modes; explicit `ros2 pkg prefix` and `ros2 interface show` audit after future source changes |
| Milestone 2: CTR model | Partially complete | `ApproximateCTRModel.forward_kinematics(q)` exists and direct package-local tests pass | MATLAB-to-Python fixture comparison; parameter source-of-truth decision; validation thresholds documented and passing |
| Milestone 3: ROS2 simulation | Partially complete; Milestone 4 loop runtime verified | `CTRSimulationCore` and `simulator_node.py` exist and direct simulation core tests pass; foreground runtime smoke verified `/ctr/state`, `/ctr/mppi_command`, `/ctr/safe_command`, and `/ctr/controller/metrics` while `/ctr_simulator` remained alive | Dedicated Milestone 3 acceptance audit for topic frequencies, units, axes, RViz marker visualization, and non-MPPI command paths |
| Milestone 4: minimum MPPI fixed-target reaching | Runtime verified complete for scoped minimum | Focused tests passed: `ctr_bringup` 19, `ctr_mppi_controller` 14, `ctr_sim` 4; clean isolated build finished all 11 packages; foreground PTY smoke test verified `/parameter_validator`, `/ctr_simulator`, `/mppi_controller`, command/state/metrics topics, clean Ctrl-C shutdown, and no tracebacks | Hardware execution remains disabled and unverified; approximate model remains unresolved by `MODEL-004`; trajectory tracking, shape control, obstacle avoidance, tactile control, and hardware support remain future milestones |

## Ubuntu build-readiness and compatibility findings

| ID | Finding | Current evidence | Evidence needed to resolve | Module | Priority |
|---|---|---|---|---|---|
| BUILD-001 | Clean Ubuntu build result exists for the current source tree | `build_shutdown_final`, `install_shutdown_final`, and `log_shutdown_final` were produced by a clean isolated build; all 11 packages finished successfully | Re-run the clean isolated build after future source changes or before release | Build | Resolved for current source |
| BUILD-002 | `rosdep` cannot resolve `ament_python` | Historical `rosdep check --from-paths src --ignore-src` reported no rosdep rule for `ament_python`; colcon build still succeeds in the current Ubuntu environment | Manifest policy decision plus clean `rosdep check`, or documented justification that buildtool dependency is acceptable despite rosdep behavior and successful isolated build | Build | Retained |
| BUILD-003 | Installed simulation launch path is verified | The Milestone 4 runtime smoke test sourced `install_shutdown_final` and launched installed project nodes successfully | Explicit `ros2 pkg prefix` for all project packages and `ros2 interface show` for generated interfaces after future source changes | Build | Partially resolved |
| TEST-003 | Top-level unittest discovery finds zero tests | `python3 -m unittest discover -s src` ran zero tests; direct package-local tests passed | Adopt a documented test invocation and verify it in CI or update test layout so top-level discovery finds intended tests | Testing | Medium |
| COMPAT-001 | CRLF line endings remain in repository assets | CRLF detected in docs, YAML, README/CODEX_TASK, and selected MATLAB/data files | Line-ending policy decision and conversion/verification where required for Ubuntu tooling | Compatibility | Medium |
| COMPAT-002 | MATLAB hardware reference code uses Windows COM ports | `Drive.m` and `CTRControlApp_PseudoPMC.m` reference `COM5`, `COM6`, and `COM13` | Ubuntu hardware interface design using ROS2-safe hardware nodes and documented device mapping; do not use COM-port MATLAB code as hardware readiness evidence | Hardware | High |
| COMPAT-003 | MATLAB `.mat` fixtures were created on Windows | `file` reports PCWIN64 MATLAB v5 files | Load and validate fixtures in Ubuntu MATLAB/Octave/Python workflow or replace with documented Ubuntu-generated fixtures | Data | Medium |
| COMPAT-004 | Legacy `log/` is inspection output, but named build logs now exist | `log/` contains earlier inspection logs; `log_shutdown_final` records the successful 11-package clean isolated build | Continue using named isolated log directories for future verification runs | Build | Resolved for current source |

## Open items

| ID | Item | Default | Required input | File or experiment needed to resolve it | Module | Priority |
|---|---|---:|---|---|---|---|
| CTR-001 | Tube lengths | 0.20/0.18/0.16 m | Measurement or CAD | Update `config/robot_params.yaml` from CAD drawing or bench measurement report | Model | High |
| CTR-002 | Outer diameters | Placeholder | Datasheet | Update `config/robot_params.yaml` from tube datasheet or micrometer measurement log | Model | High |
| CTR-003 | Inner diameters | Placeholder | Datasheet | Update `config/robot_params.yaml` from tube datasheet or micrometer measurement log | Model | High |
| CTR-004 | Precurvature | Placeholder | CAD or calibration | Tube curvature calibration experiment or CAD-derived curvature file | Model | High |
| CTR-005 | Precurved lengths | Placeholder | CAD | Tube CAD drawing or measured precurved-section length file | Model | High |
| CTR-006 | Young's modulus | 1e9 Pa | Material data | Material datasheet or bending calibration experiment | Model | Medium |
| CTR-007 | Shear modulus | 3.8e8 Pa | Material data | Material datasheet or torsion calibration experiment | Model | Medium |
| CTR-008 | Tube friction | Not modeled | Experiment | Insertion/rotation friction characterization experiment | Model | Medium |
| MODEL-004 | Python CTR model implementation | approximate scaffold exists | Ported or validated model logic | `src/ctr_model` Python port of `Robot.m`, `Tube.m`, and `sample_ctr_centerline.m`, or documented validation showing the scaffold is sufficient for the milestone | Model | Critical |
| MODEL-005 | MATLAB-to-Python validation thresholds | tip and mean backbone error below 1e-3 m | Benchmark agreement target | Python test fixture using `WorkspaceSamples.mat` and MATLAB comparison outputs | Model | High |
| MODEL-006 | Source-of-truth tube parameter set | Use YAML placeholders | Decision between YAML placeholders and Phase 3 MATLAB parameters | Parameter review comparing `config/robot_params.yaml` to Phase 3 `initialize_ctr_model()` values | Model | Critical |
| ROS-001 | ROS2 package skeleton | Package files present, build unverified | Ubuntu build verification | Isolated `colcon build`, installed package discovery, and launch smoke tests | ROS2 | Critical |
| ROS-002 | Custom ROS2 interfaces | `.msg` and `.srv` files present, generated types unverified | Generated interface validation | Build `ctr_interfaces`, source isolated install, verify `ros2 interface show` and Python imports for generated types | ROS2 | Critical |
| ROS-003 | Runtime launch modes | simulation, mock_hardware, physical_hardware launch files present | Runtime launch validation | `ctr_bringup` launch startup tests for each mode, with hardware I/O disabled unless commissioned | ROS2 | High |
| ROS-004 | `simulation_with_sensor_noise` and `hardware_in_loop` modes | Documented modes, no dedicated launch validation found | Mode design and launch coverage | Launch arguments or dedicated launch files plus smoke tests for the documented modes | ROS2 | Medium |
| TODO-OWNER-001 | Package maintainer metadata | `todo@example.com` and `TODO-OWNER-001` | Project maintainer identity | Update all package manifests and setup files after owner decision | Metadata | Medium |
| TODO-LICENSE-001 | Package license metadata | `TODO-LICENSE-001` | Project license decision | Add project license file and update all package manifests/setup files | Metadata | High |
| FRAME-001 | Slicer LPS to robot/world transform | Slicer LPS coordinates in mm | Registration transform | Vessel-to-robot registration experiment or documented transform file | Simulation | High |
| FRAME-002 | ROS frame tree | `world`, `base_link`, `ctr_tip`, `tactile_frame` | TF parent-child definition | Frame convention file and RViz verification | ROS2 | High |
| MPPI-001 | Final control frequency | 20 Hz | Benchmark | Closed-loop simulation benchmark after minimum MPPI implementation | Controller | High |
| MPPI-002 | Final sample count | 500 | Benchmark | MPPI timing and convergence benchmark on target hardware | Controller | Medium |
| MPPI-003 | Final horizon | 10 | Experiment | Reaching and trajectory-tracking benchmark sweep | Controller | Medium |
| MPPI-004 | Cost weights | Defaults | Parameter tuning | Simulation tuning report using fixed target and trajectory scenarios | Controller | Medium |
| MPPI-005 | Control noise standard deviation | insertion `[0.0005, 0.0005, 0.0005]`, rotation `[0.02, 0.02, 0.02]` | Controller tuning | MPPI rollout/timing experiment updating `config/mppi_params.yaml` | Controller | Medium |
| COST-001 | Obstacle collision distance | Not defined | Safety margin | Add value to configuration after obstacle simulation experiment | Controller | High |
| COST-002 | Obstacle safety distance | Not defined | Safety margin | Add value to configuration after lumen/obstacle clearance experiment | Controller | High |
| COST-003 | Hard collision penalty | Not defined | Cost tuning | Cost-function sweep in static obstacle simulation | Controller | High |
| COST-004 | Proximity penalty shape | Not defined | Cost tuning | Cost-function sweep in static and dynamic obstacle simulations | Controller | Medium |
| COST-005 | Shape tracking cost implementation | Disabled interface | Shape reference and resampling design | Implement and test backbone shape cost using validated backbone point alignment/resampling | Controller | High |
| COST-006 | Stability cost implementation | Disabled interface | Stability criterion | Define and test stability term, or document why it remains disabled for the current research scope | Controller | Medium |
| SNS-001 | Tactile sensor model | Unknown | Hardware info | Sensor datasheet or selected simulated sensor specification | Tactile | Critical |
| SNS-002 | Tactile output type | Unknown | Hardware info | Sensor datasheet and raw serial/data capture file | Tactile | Critical |
| SNS-003 | Zero offset | 0 | Calibration | No-load tactile calibration dataset | Tactile | Critical |
| SNS-004 | Calibration scale | 1.0 | Calibration | Known-force calibration dataset | Tactile | Critical |
| SNS-005 | Warning force | 0.30 N | Safety experiment | Soft-contact safety experiment updating `config/tactile_params.yaml` and `config/safety_params.yaml` | Safety | Critical |
| SNS-006 | Stop force | 0.50 N | Safety experiment | Hard-contact safety experiment updating `config/tactile_params.yaml` and `config/safety_params.yaml` | Safety | Critical |
| SNS-007 | Contact force threshold | 0.10 N | Contact calibration | Contact/no-contact classification experiment | Tactile | High |
| SNS-008 | Release force threshold | 0.08 N | Release calibration | Retreat/release experiment after simulated and physical contact tests | Safety | High |
| SNS-009 | Tactile sample frequency | 100 Hz | Sensor timing data | Sensor timing log or hardware datasheet | Tactile | Medium |
| HW-001 | Motor model | Unknown | Hardware info | Motor datasheet or bill of materials | Hardware | Critical |
| HW-002 | Driver model | Unknown | Hardware info | Driver manual or controller board documentation | Hardware | Critical |
| HW-003 | Communication protocol | mock | Driver manual | Hardware communication manual and bench communication test | Hardware | Critical |
| HW-004 | Motor IDs | 1-6 | Hardware setup | Motor wiring map and commissioning checklist | Hardware | High |
| HW-005 | Encoder resolution | 1.0 | Datasheet | Encoder datasheet or encoder count calibration experiment | Hardware | Critical |
| HW-006 | Insertion conversion scale | 1.0 | Calibration | Linear-stage motion/count calibration experiment | Hardware | Critical |
| HW-007 | Rotation conversion scale | 1.0 | Calibration | Rotary-stage motion/count calibration experiment | Hardware | Critical |
| HW-008 | Motor direction signs | +1 | Test | Low-speed single-axis direction test | Hardware | Critical |
| HW-009 | Homing mechanism | None | Hardware design | Limit switch, index pulse, or manual homing design document | Hardware | Critical |
| HW-010 | Serial device | `/dev/ttyUSB0` | Hardware setup | Physical connection test and udev/device mapping file | Hardware | High |
| HW-011 | Baud rate | 115200 | Driver manual | Driver manual or serial communication bench test | Hardware | High |
| HW-012 | Hardware communication timeout | 0.05 s | Bench test | Communication latency/dropout experiment | Hardware | High |
| HW-013 | Watchdog timeout | 0.10 s | Bench test | Watchdog stop experiment with simulated communication loss | Hardware | Critical |
| SAFE-001 | Maximum insertion speed | 0.002 m/s | Hardware test | Low-speed insertion safety experiment | Safety | Critical |
| SAFE-002 | Maximum rotation speed | 0.10 rad/s | Hardware test | Low-speed rotation safety experiment | Safety | Critical |
| SAFE-003 | Retreat direction | Base-axis fallback | Sensor capability | Contact-release experiment or tactile direction-estimation capability test | Safety | High |
| SAFE-004 | Maximum insertion acceleration | 0.005 m/s^2 | Hardware test | Acceleration-limited motion test | Safety | Critical |
| SAFE-005 | Maximum rotation acceleration | 0.20 rad/s^2 | Hardware test | Acceleration-limited rotation test | Safety | Critical |
| SAFE-006 | State timeout | 0.10 s | Runtime test | ROS2 state freshness/dropout test | Safety | High |
| SAFE-007 | Command timeout | 0.10 s | Runtime test | ROS2 command freshness/dropout test | Safety | High |
| SAFE-008 | Tactile timeout | 0.10 s | Runtime test | Tactile dropout simulation and hardware sensor dropout test | Safety | High |
| SAFE-009 | Retreat distance, speed, and maximum duration | 0.002 m, 0.0005 m/s, 5.0 s | Contact experiment | Retreat experiment with simulated and physical contact | Safety | High |
| SAFE-010 | Emergency-stop reset policy | latch true, manual reset true | Operator workflow | Safety supervisor state-machine test and operator reset procedure | Safety | Critical |
| SIM-001 | Simulation update frequency | 100 Hz | Runtime benchmark | Simulation loop timing benchmark | Simulation | Medium |
| SIM-002 | Actuator lag time constant | 0.05 s | Actuator characterization | Step-response experiment or simulated nonideal actuator benchmark | Simulation | Medium |
| SIM-003 | Command and state dropout probabilities | 0.0 | Fault-injection target | ROS2/simulation dropout experiment | Simulation | Medium |
| DATA-001 | Vessel centerline coordinate frame | LPS mm | Registration data | Slicer-to-robot registration transform file or phantom registration experiment | Evaluation | High |
| DATA-002 | Centerline model STL content | zero triangles | Correct export | Re-exported centerline geometry or documented decision not to use this STL | Evaluation | Medium |
| TEST-001 | Unit test coverage | None | Test implementation | Add tests under `tests/` for model, MPPI, cost, safety, and integration requirements | Testing | Critical |
| TEST-002 | Build verification | Clean isolated build passed for current source | Re-run after source changes and add CI coverage | `colcon build` using isolated build/install/log directories | Testing | Resolved for current source |
| QUALITY-001 | Broad cleanup exception handling | `_destroy_node()` catches broad `Exception` during shutdown cleanup | Code-quality review to decide whether to narrow this to known shutdown-safe exceptions without changing ROS2 signal behavior | ROS2 | Low |
