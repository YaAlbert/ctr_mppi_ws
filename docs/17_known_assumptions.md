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
