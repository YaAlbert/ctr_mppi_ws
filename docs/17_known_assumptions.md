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

22. No ROS2 package skeleton exists yet, so package names and interfaces
    in the architecture documents are still proposed targets.

23. YAML defaults are conservative software-development placeholders until
    each associated TODO ID is resolved by the named file, datasheet,
    calibration, or experiment in `docs/18_unresolved_items.md`.
