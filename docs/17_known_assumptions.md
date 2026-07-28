# Known Assumptions

1. The CTR has six controllable degrees of actuation:
   three insertions and three axial rotations.

2. The initial control input is joint velocity.

3. The initial model is quasi-static.

4. Dynamic effects are initially ignored.

5. The backbone can be represented using a fixed number of points.

6. Obstacles can initially be represented with simple geometry.

7. Tactile force can initially be modeled as a scalar force.

8. The physical motor protocol is currently unknown.

9. Exact tube material and geometry parameters are not yet finalized.

10. Exact tactile calibration is not yet available.

11. Default parameters are placeholders for software development.

12. Physical experiments will begin at low speed.

13. The system is a research prototype and not a certified medical
    control system.

14. The available MATLAB CTR model is a simplified piecewise
    constant-curvature geometry model, not a validated physical model.

15. Phase 3 MATLAB scripts expose `q = [rho1 rho2 rho3 theta1 theta2
    theta3]` with `rho` in millimeters and `theta` in degrees.

16. The ROS2/Python public interface is assumed to use SI units:
    `rho` in meters and `theta` in radians.

17. MATLAB-to-Python validation must explicitly convert between MATLAB
    mm/deg inputs and ROS2 m/rad inputs.

18. The current MATLAB Phase 3 model treats `rho1` as an insertion
    reference and computes tube 2 and tube 3 insertions relative to it.

19. Saved MATLAB `.mat` files are treated as offline validation fixtures,
    not as runtime controller state.

20. Vessel centerline and STL data exported from 3D Slicer are assumed to
    be in LPS millimeter coordinates until a robot registration transform
    is defined.

21. The MATLAB hardware GUI and `Drive`/`Pose` classes are treated as
    reference material only. They are not assumed to satisfy the ROS2
    safety architecture.

22. A ROS2 package skeleton now exists in `src/`, but it is not yet verified
    complete until the current Ubuntu source tree builds successfully and the
    installed packages/interfaces are discoverable after sourcing the isolated
    install workspace.

23. YAML defaults are conservative software-development placeholders until
    each associated TODO ID is resolved by the named file, datasheet,
    calibration, or experiment in `docs/18_unresolved_items.md`.

24. Work created on Windows is treated as historical existing material. It is
    not considered a completed milestone unless verified against the current
    Ubuntu repository, installed ROS2 Humble environment, and build/test
    results.

25. The current `log/` directory is assumed to contain Ubuntu-generated colcon
    inspection logs only. It is not a successful build log and does not prove
    package buildability.

26. The absence of `build/` and `install/` means there is no current installed
    workspace for `ros2 pkg prefix` or generated interface import checks.

27. The `ament_python` rosdep key is currently unresolved in the Ubuntu rosdep
    database, even though colcon's `ros.ament_python` build extension is
    installed. This is treated as a build-readiness issue until resolved by a
    manifest policy decision or a successful isolated build with documented
    rationale.

28. Package-local Python tests are treated as useful unit evidence only when
    run directly from each package test directory. Top-level
    `python3 -m unittest discover -s src` currently discovers zero tests.

29. CRLF line endings in documentation, YAML, and selected MATLAB/data files
    are treated as a compatibility issue to track, not as proof of source
    invalidity.

30. MATLAB hardware GUI files using Windows COM ports are reference material
    only. They are not Ubuntu hardware drivers and must not be used to justify
    physical hardware readiness.

31. Milestone 5 trajectory defaults are software-test trajectories near the
    current approximate-model initial tip. They are not physical workspace
    limits and do not prove physical tracking accuracy.

32. Milestone 5 trajectory RMSE values are runtime smoke-test metrics. They are
    not rigorous performance-validation results because state, reference,
    command, and metric timestamps are not yet synchronized strongly enough for
    that claim.

33. The current Milestone 5 MPPI implementation is treated as a simplified
    weighted random-shooting MPPI-style controller until the complete MPPI
    formulation and cost/noise scaling are reviewed.

34. Milestone 5 runtime tests verify simulation-only integration. Hardware
    execution remains disabled and unverified.

35. Milestone 5D quantitative evaluation is treated as software-simulation
    evaluation infrastructure. It is observation-only and must not be used as
    evidence that actuator command paths or hardware behavior are validated.

36. Milestone 5D evaluator output is strict JSON/YAML/CSV/Markdown/plot
    evidence for recorded runs. Missing or non-finite values are represented by
    validity fields, warnings, counters, or JSON nulls rather than by bare NaN
    or Infinity tokens.

37. Milestone 5D.1 matched-run orchestration is verified for the
    software-simulation zero-command baseline and MPPI candidate workflow only.
    Physical and hardware experiment synchronization remain unresolved.

38. Deterministic initial q/tip matching in Milestone 5D.1 is based on the
    simulator initial state and the accepted stability window from the current
    Ubuntu ROS2 environment. It does not validate physical reset repeatability.

39. Scheduled reference behavior uses a run-relative phase policy. Absolute
    scheduled timestamps naturally differ between separate baseline and
    candidate runs and are not required to be equal for compatibility.

40. Command timing evidence distinguishes command message timestamp and
    command receive timestamp. Neither is a command-application timestamp.

41. Horizon reference samples do not currently carry individual timestamps, so
    horizon-level causal timing remains approximate.

42. The two matched circle experiment pairs demonstrate deterministic
    orchestration, repeatability, valid comparison conditions, and quantitative
    reporting. They do not demonstrate meaningful tracking improvement,
    real-time performance, physical accuracy, or hardware readiness.

43. MPPI deadline overrun remains 100% in the verified matched runs, and the
    controller remains non-real-time relative to the configured 0.05 s control
    period.

44. Concurrent `ctr_run_evaluation` invocations assume ROS_DOMAIN_ID selection
    does not collide in practice. A deterministic cross-process reservation or
    locking mechanism remains unresolved.

45. Milestone 6A straight-cylinder parameters are provisional
    software-simulation defaults: radius `0.030 m`, length `0.120 m`, CTR outer
    radius `0.0015 m`, and safety margin `0.0020 m`. They are not measured
    physical CTR, tube, anatomical, or hardware parameters.

46. The Milestone 6A default point goal `[0.015, 0.005, 0.100] m`, tolerance
    `0.003 m`, and required hold duration `0.5 s` are software-simulation
    acceptance values, not physical task requirements.

47. The `cylinder_fast` profile is a bounded software-simulation profile with
    36 samples, horizon 7, rollout `dt` `0.55 s`, controller period `0.10 s`,
    insertion noise `0.003`, rotation noise `0.100`, tip and terminal weights
    `15000`, control weight `0.005`, and smoothness weight `0.01`. It is not a
    real-time or optimal-control certification profile.

48. Milestone 6A sampled reachability is a deterministic approximate-model
    sanity check. It does not prove formal kinematic reachability or guarantee
    that the controller will reach every sampled-reachable target.

49. Milestone 6A verifies whole-backbone cylinder containment for an analytical
    straight lumen in simulation. It does not verify curved-lumen, obstacle-map,
    anatomical-mesh, tactile-contact, safety-retreat, physical-validation, or
    hardware behavior.

50. Milestone 6A default-target runs demonstrate functional integration,
    quantitative evaluation, and collision-free behavior for the tested seeds
    only. They do not demonstrate broad target robustness.

51. Milestone 6A timing remains non-real-time. The default-target runs measured
    mean solve time around `0.133-0.139 s` against a `0.10 s` controller period,
    and deadline overrun remained `100%`.
