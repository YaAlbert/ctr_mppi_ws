# CTR-MPPI Workspace Guide and Current Implementation Status

## Purpose

This document is the main entry point for understanding, running, and extending
the repository. It describes the current code on `main` without relying on
temporary branch names, development snapshots, or machine-specific result
paths.

The repository currently provides a complete simulator-facing CTR control
stack, deterministic evaluation tooling, tactile and safety supervision, and
ROS 2 interfaces intended to remain stable when hardware support is added.
Evidence in this repository is simulation-only. It does not establish
real-time performance, physical-robot validation, or production readiness.

Some tracked filenames and executable names retain older internal naming for
compatibility with tests and recorded evidence. This guide uses functional
names in the prose and shows those compatibility names only where they are
required in an actual path or command.

## Recommended reading order

1. Read this file for the current implementation and normal workflows.
2. Read [README.md](README.md) for the short project introduction.
3. Read [System Scope](docs/01_system_scope.md), [Robot Definition](docs/02_robot_definition.md),
   and [Coordinate and Unit Convention](docs/03_coordinate_and_unit_convention.md)
   before changing model or hardware code.
4. Read [ROS 2 Architecture](docs/07_ros2_architecture.md) and
   [ROS 2 Interfaces](docs/08_ros2_interfaces.md) before changing topics,
   services, launch composition, or package boundaries.
5. Read [Safety Requirements](docs/11_safety_requriements.md) before changing
   command handling, timeouts, tactile behavior, geometry, or failure logic.
6. Read [Final Simulation Evidence](docs/21_final_system_paper_evaluation.md)
   before reproducing an evaluation campaign.

## Current supported scope

The implemented and tested software supports:

- a deterministic approximate three-tube CTR forward model;
- joint-velocity MPPI with fixed-target and trajectory references;
- circular, elliptical, and helical reference generation;
- straight-cylinder and curved-lumen geometry, clearance, end-cap, and
  whole-backbone checks;
- tactile-aware costs and a deterministic simulated tactile source;
- an independent safety supervisor with stale-data, invalid-data, collision,
  clearance, tactile-stop, and latched-fault handling;
- a closed-loop ROS 2 simulator with state, backbone, tip, tactile, geometry,
  diagnostic, and RViz marker publication;
- fixed profile, explicit CLI, and interactive RViz target selection;
- raw-data recording, metric reconciliation, plots, reports, and deterministic
  evaluation orchestration;
- a behavior-preserving vectorized lumen-cost path that is available through
  explicit evaluation profiles.

The accepted simulation evidence demonstrates nominal circular-arc navigation,
repeatability across five tested random seeds, and operation across the tested
controller-computation profiles. The optimized computation path reproduced the
reference commands, costs, importance weights, random-number consumption, and
diagnostics exactly in its matched regression benchmark.

Important limitations:

- the controller remains non-real-time on the evaluated general-purpose host;
- the approximate model is not a calibrated physical CTR model;
- the difficult lateral targets used by the research evaluation are outside
  the reachable set of the current approximate model under its configured
  limits and completion tolerance;
- straight-lumen runs expose an evidence-summary integration gap even when
  controller motion completes;
- the configured S-curve initial state does not satisfy the required wall
  safety margin;
- physical hardware, encoder feedback, homing, and physical tactile acquisition
  are not implemented;
- the standalone state-estimator and visualization packages are interface
  placeholders; the simulator currently produces controller-ready state and
  RViz markers directly.

## Runtime architecture

The intended command and feedback flow is:

```text
reference manager or accepted external target
                    |
                    v
              MPPI controller
                    |
                    v
              safety supervisor
                    |
                    v
        simulator or future actuator driver
                    |
                    v
       state feedback / future state estimator
              |                 |
              v                 v
          controller       evaluation and RViz
```

The controller is deliberately independent of motor buses and hardware packet
formats. Geometry and safety checks are shared through explicit interfaces
rather than being bypassed by the simulator. A future hardware driver should
replace the simulated actuator/state source while preserving the controller,
safety, message, and evaluation contracts.

## Package and code map

| Package | Current responsibility | Main implementation locations |
|---|---|---|
| `ctr_interfaces` | Custom CTR messages and services for joint state, backbone, fused state, commands, tactile state, safety state, controller metrics, references, experiment control, and fault recovery. | `src/ctr_interfaces/msg/`, `src/ctr_interfaces/srv/` |
| `ctr_model` | ROS-independent approximate CTR kinematics with input validation, joint-limit handling, backbone sampling, and tip output. | `approximate_model.py`; a higher-fidelity Cosserat implementation is a planned extension behind the same conceptual interface. |
| `ctr_mppi_controller` | MPPI sampling, rollout propagation, cost evaluation, importance weighting, command update, reference handling, lumen geometry, tactile cost, diagnostics, and the ROS controller wrapper. | `mppi_core.py`, `cost_functions.py`, `lumen_geometry.py`, `curved_lumen.py`, `cylindrical_lumen.py`, `reference_trajectory.py`, `nodes/mppi_controller_node.py` |
| `ctr_sim` | Closed-loop actuator simulation, simulated state/tactile production, lumen diagnostics, RViz markers, physical-evidence timing for evaluation, and one-shot target selection. | `simulation_core.py`, `nodes/simulator_node.py`, `nodes/development_target_selector_node.py`, `lumen_markers.py`, `lumen_diagnostics.py` |
| `ctr_safety` | Independent command supervision, state/tactile freshness checks, whole-backbone geometry checks, velocity gating, contact response, emergency stop, and fault latching. | `nodes/safety_supervisor_node.py`, `geometry_adapter.py` |
| `ctr_tactile` | Deterministic tactile signal processing and simulated contact-force utilities. The current simulation publishes tactile state through `ctr_sim`; the package node remains a future sensor-driver integration point. | `tactile_processing.py`, `simulated_tactile.py`, `nodes/tactile_placeholder_node.py` |
| `ctr_evaluation` | Observation-only recording, time alignment, metrics, plots, reports, paired-run orchestration, evidence validation, target/geometry matrices, and artifact provenance. | `experiment_recorder.py`, `metrics.py`, `lumen_metrics.py`, `run_evaluation.py`, `paper_evidence.py`, `report_generator.py` |
| `ctr_bringup` | Parameter loading and validation, runtime composition, launch files, and explicit evaluation-only evidence transport. | `parameter_validation.py`, launch files under `src/ctr_bringup/launch/` |
| `ctr_hardware` | Reserved boundary for mock and physical actuator implementations. Both current nodes validate configuration but intentionally perform no hardware I/O. | `nodes/mock_hardware_node.py`, `nodes/physical_hardware_node.py` |
| `ctr_state_estimator` | Reserved boundary for encoder/tactile fusion and publication of controller-ready `/ctr/state` when real feedback replaces simulator state. | `nodes/state_estimator_node.py` |
| `ctr_viz` | Reserved boundary for a standalone visualization node. Current development visualization is published by `ctr_sim` and displayed with RViz. | `nodes/visualization_node.py` |

Other important directories:

- `config/`: ordered YAML configuration and validated evaluation contracts;
- `docs/`: requirements, interfaces, design intent, test policy, and
  historical validation notes;
- `data/matlab/`: original MATLAB CTR and vessel-geometry studies used as
  reference material, not as the active ROS controller;
- `evaluation_results/`: generated or historical evidence. New results are
  ignored by Git and must not be treated as source code;
- `build*/`, `install*/`, and `log*/`: generated colcon output, also
  ignored by Git.

## Documentation map

### Repository-level documents

| Document | Purpose |
|---|---|
| [CURRENT_STATUS.md](CURRENT_STATUS.md) | Evergreen description of the current workspace, configuration, operation, limitations, and recommended extensions. |
| [README.md](README.md) | Short project goal, platform, control variables, and quick introduction. |
| [REALTIME_HOST_REQUIREMENTS.md](REALTIME_HOST_REQUIREMENTS.md) | Host-isolation and authority requirements for any future latency/freshness study. It is not evidence that the current controller is real-time. |
| [MATLAB geometry study report](data/matlab/Phase3_CTR%20in%20Free%20Space%20and%20Confined%20Environment/Phase3_Report.md) | Documents the simplified MATLAB CTR model, workspace sampling, vessel centerline analysis, curvature feasibility, and local segment fitting. |

### Requirements and design documents

| Document | Purpose |
|---|---|
| [Project Overview](docs/00_project_overview.md) | Research motivation, overall objective, and separation between control, safety, simulation, and hardware. |
| [System Scope](docs/01_system_scope.md) | Supported simulator scope, longer-term research scope, and explicitly excluded early capabilities. |
| [Robot Definition](docs/02_robot_definition.md) | Definition of the three-tube configuration vector, velocities, backbone, tip, and initial state. |
| [Coordinate and Unit Convention](docs/03_coordinate_and_unit_convention.md) | SI units, frame names, base orientation, and ROS coordinate conventions. |
| [CTR Model Requirements](docs/04_ctr_model_requirements.md) | Model inputs/outputs, abstraction boundary, MATLAB comparison expectations, and constraints. |
| [MPPI Controller Requirements](docs/05_mppi_controller_requirements.md) | Controller input/output, rollout algorithm, deterministic random generator, limits, and timing diagnostics. |
| [Cost Function Definition](docs/06_cost_function_definition.md) | Mathematical meaning of target, shape, control, smoothness, terminal, obstacle, tactile, joint-limit, and stability costs. A documented term may still be disabled in the active configuration. |
| [ROS 2 Architecture](docs/07_ros2_architecture.md) | Package boundaries, intended data flow, runtime modes, and prohibited coupling. |
| [ROS 2 Interfaces](docs/08_ros2_interfaces.md) | Canonical topics, services, timestamps, frames, validity fields, and diagnostic expectations. |
| [Simulation Requirements](docs/09_simulation_requirements.md) | Simulator behavior, visualization legend, target validation, CLI/RViz target selection, and scenario catalog. |
| [Tactile Sensor Requirements](docs/10_tactile_sensor_requirements.md) | Calibration, filtering, contact state, thresholds, and simulation model requirements. |
| [Safety Requirements](docs/11_safety_requriements.md) | Safety states, watchdog behavior, stop conditions, retreat behavior, and fail-closed expectations. The filename spelling is retained for compatibility. |
| [Hardware Adaptation Requirements](docs/12_hardware_adaptation_requirements.md) | Common actuator API, driver responsibilities, unit conversion, restrictions, and commissioning order. |
| [Parameter Registry](docs/13_parameter_registry.md) | Policy and categories for robot, model, MPPI, tactile, hardware, and safety parameters. |
| [Test and Evaluation Plan](docs/14_test_and_evaluation_plan.md) | Unit/integration test coverage, metrics, paired-run rules, and research experiment structure. |
| [Development Roadmap](docs/15_development_milestones.md) | Historical implementation roadmap. Use it for context; use this file for the current state. |
| [Acceptance Criteria](docs/16_acceptance_criteria.md) | Functional and quantitative acceptance conditions accumulated during implementation. |
| [Known Assumptions](docs/17_known_assumptions.md) | Modeling, sensing, control, timing, and hardware assumptions that bound interpretation. |
| [Unresolved Items](docs/18_unresolved_items.md) | Open TODO identifiers, known gaps, and items requiring measurements or hardware decisions. |
| [Historical simulation governance contract](docs/19_slice_7g_simulation_promotion_charter.md) | Immutable campaign authority, evidence, readiness, and attempt-accounting rules used for an earlier validation campaign. It is not the normal user guide. |
| [Historical coordinated-runtime implementation](docs/20_slice_7g_runtime_source_implementation.md) | Provenance for the authenticated evidence transport, readiness orchestration, cleanup authority, and runtime contracts used by that campaign. |
| [Final Simulation Evidence](docs/21_final_system_paper_evaluation.md) | Current matrix reproduction command, metric definitions, diagnostic files, and artifact layout. |

### Tracked historical evidence notes

These files document earlier simulator investigations. They should remain
unchanged; new experiments should create a new ignored result root.

| Document | Purpose |
|---|---|
| [Development simulation report](evaluation_results/slice_7g_development_20260823T005500Z/development_report.md) | Initial integrated simulator build, tests, plots, and limitations. |
| [Computation comparison](evaluation_results/slice_7g_performance_optimized_20260823T014418Z/performance_comparison.md) | Earlier offline and simulator timing comparison and visualization evidence. |
| [Visual shutdown reliability](evaluation_results/slice_7g_performance_optimized_20260823T014418Z/visual_shutdown_reliability.md) | Controlled shutdown and process-cleanup checks for visual simulation. |
| [Evaluation validation report](evaluation_results/slice_7g_pre_main_evaluation_20260825T120000Z/evaluation_validation_report.md) | Validation of paired-run artifacts and metric semantics. |
| [Target-selection report](evaluation_results/slice_7g_target_selection_20260825T103000Z/target_selection_report.md) | Tested profile, CLI, and RViz target identity behavior. |

## Configuration map

All robot-specific and experiment-specific numbers should remain in YAML or an
explicit evaluation manifest. Do not hide new physical constants in Python
source.

| File | Main parameters and meaning |
|---|---|
| `config/robot_params.yaml` | Frame names; initial insertion/rotation; tube lengths, diameters, precurvature, precurved lengths, material properties; joint position, velocity, and acceleration limits. Replace provisional geometry with measured values before physical use. |
| `config/model_params.yaml` | Active model implementation, backbone sample count, integration step, input validation, joint-limit clipping, approximate-model scale factors, and disabled higher-fidelity/residual-model placeholders. |
| `config/mppi_params.yaml` | Controller frequency, rollout step, horizon, sample count, temperature, seed, warm start, exploration noise, cost weights, tactile cost, convergence tests, lumen geometry, goals, reference trajectories, and named evaluation profiles. |
| `config/simulation_params.yaml` | Simulation update rate, actuator lag/delay/dead zone/backlash, state/tactile noise, communication dropout, marker density/rate, surface opacity, tip-history bounds, and manual-target validation tolerances. |
| `config/safety_params.yaml` | State/command/tactile freshness, watchdog period, fail-closed switches, soft-contact scaling, retreat distance/speed/duration, and emergency-stop latching. |
| `config/tactile_params.yaml` | Sensor dimensions/rate, calibration, filter selection, hysteretic contact/warning/stop thresholds, simulated contact stiffness/damping, saturation, latency, and noise. |
| `config/hardware_params.yaml` | Disabled hardware selector, transport settings, motor IDs, directions, encoder scales, unit conversion, homing, and hardware watchdog. Values are placeholders until measured and commissioned. |
| `config/evaluation_params.yaml` | Result root, labels, sample limits, alignment windows, acceptance thresholds, plot/report flags, diagnostics flag, and bounded startup/finalization/cleanup behavior. |
| `config/slice_7g_runtime_params.yaml` | Compatibility-named validated simulator runtime and acceptance profile. It also contains explicit non-production watchdog tolerances; never use those tolerances as hardware defaults. |
| `config/slice_7g_simulation_charter.json` | Historical immutable campaign contract for provenance and attempt accounting, not a normal tuning file. |
| `config/slice_7g_development.rviz` | RViz display layout, topics, marker styles, and the configured Publish Point tool. |

### MPPI parameters that may be optimized

Change these only in a named evaluation profile first, keep a baseline, use the
same seeds and targets, and validate raw evidence before considering a default
change.

| Parameter | Meaning | Main trade-off |
|---|---|---|
| `control_frequency` | Requested ROS control-loop frequency in hertz. | A higher request does not help if solve time exceeds the period. |
| `dt` | Time represented by one rollout step. | Larger values look farther ahead but make the discrete model coarser. |
| `horizon` | Number of future control steps in each rollout. | More look-ahead increases computation and may improve planning. |
| `num_samples` | Number of sampled control sequences per solve. | More samples improve exploration but usually dominate runtime. |
| `lambda` | MPPI temperature used in importance weighting. | Lower values concentrate on the best rollouts; higher values distribute weight more broadly. |
| `noise_std.insertion` | Exploration standard deviation for insertion velocities. | Too small limits exploration; too large increases saturation and variation. |
| `noise_std.rotation` | Exploration standard deviation for rotation velocities. | Same trade-off in rotational command space. |
| `weights.tip` | Stage tip-to-target penalty. | Increases target attraction throughout the rollout. |
| `weights.terminal` | Final rollout tip-to-target penalty. | Emphasizes terminal accuracy. |
| `weights.control` | Control-magnitude regularization. | Higher values reduce command effort but can slow progress. |
| `weights.smoothness` | Control-rate/change regularization. | Higher values reduce abrupt changes but can reduce responsiveness. |
| `weights.shape` | Backbone/reference-shape cost. | Currently disabled in the default configuration; enabling it requires implementation-level validation. |
| `weights.obstacle` | Generic obstacle cost. | Currently disabled; lumen collision costs are separate and must not be weakened. |
| `weights.force` | Tactile force cost weight. | Used only when tactile cost is enabled and valid tactile evidence exists. |
| `weights.joint_limit` | Soft joint-limit penalty. | Complements hard command and configuration limits. |
| `warm_start` and `shift_previous_solution` | Reuse and shift the preceding nominal sequence. | Improves continuity and convergence but must remain deterministic in tests. |
| `behavior_preserving_optimization_enabled` | Selects the validated vectorized lumen-cost implementation. | It is disabled in ordinary defaults and enabled only by explicit profiles. |
| `cost_normalization.enabled` | Enables evaluation-only scale-normalized parameterization. | Baseline-equivalent scales preserve the original algebra; changed multipliers are new controller behavior. |

The `optimization_c01` through `optimization_c11` profiles are preserved
search records. They did not establish a navigation improvement and should not
be selected as new defaults.

### Geometry, goal, and reference parameters

- `cylindrical_lumen.radius`, `length`, `ctr_outer_radius`, and
  `safety_margin` define the straight environment and usable clearance.
- `curved_lumen.type` selects `circular_arc` or `s_curve`.
- Circular-arc shape is set by inlet position, initial tangent, bend normal,
  curvature radius, and arc angle.
- S-curve shape is set by inlet position, initial tangent, bend-plane normal,
  total length, and lateral amplitude.
- `centerline_sample_spacing` controls geometric discretization and affects
  both cost and safety computation.
- `goal.position`, `goal.tolerance`, and `required_hold_duration` define
  navigation completion. Do not widen these values to relabel a failed run.
- `goal.reachability_samples` and `reachability_seed` configure the
  deterministic approximate-model sanity check used by manual target
  selection.
- `reference.mode` selects fixed target or trajectory behavior;
  `trajectory_type` selects circle, ellipse, or helix.
- Trajectory center, radii, height, angular velocity, phase, sample period,
  publication frequency, and completion behavior can be adjusted for new
  predeclared tests.

### Parameters that require safety or hardware review

Do not treat the following as ordinary performance-tuning knobs:

- insertion/rotation position, velocity, and acceleration limits;
- lumen radius, CTR radius, wall safety margin, and collision/end-cap weights;
- state, tactile, command, and watchdog timeouts;
- stop-on-timeout/invalid flags and emergency-stop latching;
- tactile warning/stop/release thresholds;
- hardware directions, encoder resolution, conversion scales, homing, and
  watchdog configuration;
- goal tolerance and evidence eligibility thresholds.

Any change to these values needs new positive and fail-closed negative tests,
an explicit configuration identity, and new simulation evidence. Hardware
values additionally require measured calibration and supervised commissioning.

## Placeholder packages and future implementation locations

### Physical actuator support

Implement the common actuator behavior in `ctr_hardware`, replacing the
current physical placeholder with a driver that:

- opens and validates the serial/CAN transport;
- maps the six logical insertion/rotation axes to motor IDs;
- converts SI commands to device units and encoder feedback back to SI;
- implements homing and limit-switch checks;
- exposes health and diagnostics;
- stops on watchdog, communication, encoder, or motor faults;
- never contains MPPI rollout or task-cost logic.

Keep `physical_hardware.launch.py` fail-closed until the driver, emergency
stop, homing, direction, scaling, and low-speed commissioning tests exist.

### State estimation

Use `ctr_state_estimator` for timestamped encoder/tactile fusion, validity
checks, optional filtering, and publication of the controller-ready CTR state.
The estimator should consume hardware feedback; it should not issue actuator
commands or change controller weights.

### Physical tactile acquisition

Keep calibration and filtering algorithms in `ctr_tactile`. Add a hardware
source that publishes the existing tactile interface with source timestamp,
frame, validity, calibrated force, contact/warning/stop state, and diagnostic
status. Safety must continue to reject stale, invalid, out-of-order, or missing
tactile data.

### Standalone visualization

Use `ctr_viz` if visualization must be separated from the simulator. Reuse
the established topics and frames; do not make safety or controller decisions
depend on RViz.

### Higher-fidelity model

Add a Cosserat or calibrated model under `ctr_model` behind the same
forward-kinematics contract. Validate it against the MATLAB data and physical
measurements before changing reachability conclusions or controller tuning.

## Build and test

### Environment

The supported software environment is Ubuntu 22.04, ROS 2 Humble, Python 3,
NumPy, colcon, and the package dependencies declared in each `package.xml`.

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Use a fresh shell or source ROS and the workspace overlay again before every
run. Do not source an old install tree from a different code version.

### Functional test suite

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

For a focused controller/simulator change:

```bash
colcon test --packages-select +  ctr_model ctr_mppi_controller ctr_sim ctr_safety ctr_evaluation ctr_bringup +  --event-handlers console_direct+
colcon test-result --verbose
```

The last accepted complete source validation passed 2,268 tests with no
failures, skips, or expected failures. That result applies only to the
validated source and environment; new source or safety-relevant configuration
changes require appropriate tests again.

## Running the simulator

### Visual nominal run

After building and sourcing the workspace:

```bash
ros2 launch ctr_bringup slice_7g_development_visual.launch.py +  development_simulation:=true +  target_source:=profile +  seed:=11
```

The launch filename is retained for compatibility. Functionally, it starts the
circular-arc simulator, tactile source, safety supervisor, MPPI controller,
reference manager, target/geometry markers, fixed `world -> base_link`
transform, and RViz.

Expected visual elements include the translucent lumen, cyan boundary rings,
centerline, current backbone, tip history, target, and controller reference.
Stop the run with Ctrl-C and confirm that all ROS processes exit.

### Manual target from launch arguments

CLI coordinates are metres in `base_link`. This example uses a previously
validated manual target:

```bash
ros2 launch ctr_bringup slice_7g_development_visual.launch.py +  development_simulation:=true +  target_source:=cli +  target_x:=0.0166457424 +  target_y:=0.00397477634 +  target_z:=0.102231139 +  seed:=11
```

For another point, replace the three coordinates. The CLI path never projects
or silently changes the requested point. It rejects non-finite, out-of-lumen,
end-cap-invalid, safety-margin-invalid, or sampled-unreachable targets. A
rejected target produces no motion; inspect:

```bash
ros2 topic echo /ctr/target_selection/status
ros2 topic echo /ctr/target_selection/record --once
```

### Interactive target with RViz

```bash
ros2 launch ctr_bringup slice_7g_development_visual.launch.py +  development_simulation:=true +  target_source:=rviz +  seed:=11
```

In RViz:

1. Select **Publish Point**.
2. Click the visible lumen centerline or safe interior.
3. Check the candidate marker and target-selection status.
4. Confirm the accepted yellow target and controller reference before motion.

The controller holds position while selection is pending. The first accepted
point is immutable for that run; restart the launch to choose another target.
RViz points use the displayed `world` frame, which is fixed to
`base_link` by an identity transform in this launch.

An RViz point close to the wall may be projected to the analytic centerline
only when it is within
`simulation.development_target_selection.projection_limit`. The accepted
coordinates and projection distance are published on
`/ctr/target_selection/record`; replay those accepted `base_link`
coordinates with `target_source:=cli` when deterministic reproduction is
needed. The standard 2D Goal Pose tool is intentionally not used for this
three-dimensional target workflow.

Set a bounded interactive wait when needed:

```bash
ros2 launch ctr_bringup slice_7g_development_visual.launch.py +  development_simulation:=true +  target_source:=rviz +  target_selection_timeout:=30.0 +  seed:=11
```

### Headless result-producing run

Run the integrated smoke check followed by a 25-second evaluated seed:

```bash
ros2 run ctr_evaluation ctr_run_slice_7g_development +  --development-simulation +  --target-source profile +  --duration 25 +  --seeds 11
```

The executable name is retained for compatibility. The runner creates a new
result root, starts processes sequentially, records raw data and provenance,
and performs bounded teardown. Do not point it at an existing historical
result directory.

Headless CLI target:

```bash
ros2 run ctr_evaluation ctr_run_slice_7g_development +  --development-simulation +  --target-source cli +  --target-x 0.0166457424 +  --target-y 0.00397477634 +  --target-z 0.102231139 +  --duration 25 +  --seeds 11
```

### Research evidence runner

The complete matrix is intentionally much larger than a normal smoke test:

```bash
ros2 run ctr_evaluation ctr_run_final_system_evaluation +  --output-root evaluation_results/final_system_<UTC> +  --duration 25
```

Use a unique, non-existing output root. For a bounded reference-only
reproduction:

```bash
ros2 run ctr_evaluation ctr_run_final_system_evaluation +  --output-root evaluation_results/reference_check_<UTC> +  --duration 25 +  --matrix reference
```

This runner is simulator-only and computationally slower than its requested
control period. Do not interpret its output as hardware or real-time evidence.
Preserve completed scientific failures and their raw data.

## Useful runtime topics

| Topic | Use |
|---|---|
| `/ctr/joint_state` | Simulated or future measured insertion/rotation state. |
| `/ctr/backbone` | Ordered backbone points. |
| `/ctr/state` | Controller-ready state including tip and validity. |
| `/ctr/tip` | Current tip pose used by visualization and evaluation. |
| `/ctr/reference/tip` | Active target pose. |
| `/ctr/reference/path` | Active reference path or one-pose fixed target. |
| `/ctr/mppi_command` | Raw controller command before safety supervision. |
| `/ctr/safe_command` | Command accepted/scaled/gated by safety. |
| `/ctr/tactile/state` | Calibrated tactile/contact state and validity. |
| `/ctr/safety/status` | Safety state, readiness, fault, and latch information. |
| `/ctr/controller/metrics` | Solve time, deadline, cost, and controller diagnostics. |
| `/ctr/target_point_candidate` | RViz Publish Point input. |
| `/ctr/target_selection/status` | Manual-target acceptance/rejection status. |
| `/ctr/target_selection/record` | Canonical raw/accepted target record for replay. |

## Recommended next changes

### 1. Close geometry/evidence integration gaps

- Make straight geometry populate the same explicit terminal navigation,
  physical-safety, safety-margin, and run-valid evidence schema as curved
  geometry.
- Correct the producer/recorder path; do not weaken the evidence validator or
  rewrite historical results.
- Define an S-curve geometry and initial state that satisfy the unchanged
  whole-backbone safety margin before controller startup.
- Add focused positive and negative tests for each geometry-specific readiness
  and evidence predicate.

### 2. Improve model fidelity before more target-weight tuning

- Implement the higher-fidelity model boundary in `ctr_model`.
- Calibrate geometry/material parameters against MATLAB and measured robot
  data.
- Add deterministic constrained reachability analysis using the same
  whole-backbone clearance and command limits as runtime safety.
- Declare target sets only after feasibility is established. Cost reweighting
  cannot make an unreachable target reachable.

### 3. Continue computation optimization with equivalence tests

- Profile rollout propagation, geometry interpolation, cost reduction, ROS
  conversion, and diagnostic serialization separately.
- Prefer vectorization, immutable geometry caches, preallocated arrays, and
  bounded numerical-library thread counts.
- Preserve random-number order and add exact or tight-tolerance command/cost
  regression tests for every new fast path.
- Treat reduced sample count, horizon, or model resolution as algorithmic
  trade-offs, not behavior-preserving optimization.

### 4. Add progress-aware navigation only for feasible targets

- Use a deterministic arc-length/reference-path subgoal that remains inside
  the safe lumen.
- Keep the final accepted target and completion criterion authoritative.
- Make the feature explicit and disabled by default until nominal,
  difficult-target, holdout, clearance, saturation, and safety tests pass.

### 5. Implement the hardware boundary incrementally

- Replace hardware placeholders only after motor mapping, SI conversion,
  encoder validation, watchdog, emergency stop, and homing are tested.
- Add state estimation and physical tactile acquisition without changing
  controller-facing interfaces.
- Follow the commissioning order in
  [Hardware Adaptation Requirements](docs/12_hardware_adaptation_requirements.md).
- Run a separate latency/scheduling study on an appropriately configured host;
  simulator timing must not be reused as physical real-time evidence.

### 6. Keep documentation and generated evidence separate

- Update this file whenever package responsibility, supported launch commands,
  configuration meaning, or known limitations change.
- Update the focused requirements document when an interface or safety contract
  changes.
- Store each experiment in a new ignored `evaluation_results/` root.
- Never commit build/install/log trees, raw generated campaigns, or report
  archives to source-control history.

## Change checklist

Before submitting a code or configuration update:

1. Identify the package and contract affected.
2. Add focused positive and negative tests.
3. Run `git diff --check`.
4. Build the affected packages in a fresh overlay.
5. Run focused tests; run the complete suite for cross-package or
   safety/runtime changes.
6. Re-run deterministic controller equivalence tests for computation changes.
7. Use a new result root for any ROS experiment.
8. Report failures and denominators without deleting or relabeling evidence.
9. Confirm zero remaining ROS processes or zombies.
10. Keep hardware and production claims separate from simulation results.
