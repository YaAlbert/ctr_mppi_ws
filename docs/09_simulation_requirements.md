# Simulation Requirements

## Simulation objective

The simulation shall provide a closed-loop environment for validating:

- CTR forward motion;
- MPPI reaching;
- trajectory tracking;
- whole-body shape tracking;
- obstacle avoidance;
- tactile contact handling;
- safety-state transitions.

## Minimum simulation state

The simulation node shall maintain:

- q;
- q_dot;
- backbone points;
- tip position;
- simulated time;
- actuator state;
- simulated tactile state.

## Initial actuator model

q_dot_actual =
    clip(
        q_dot_command,
        velocity_limits
    )

q_next =
    q_current + dt * q_dot_actual

## Extended actuator model

Later versions shall support:

- first-order actuator lag;
- command delay;
- acceleration limits;
- backlash;
- dead zone;
- friction;
- noise;
- dropped commands;
- encoder quantization.

## Environment representation

The first version shall support:

- target point;
- reference path;
- sphere obstacle;
- cylinder obstacle;
- moving sphere obstacle.

Later versions may support:

- lumen centerline;
- lumen wall;
- signed distance field;
- mesh environment.

## RViz visualization

The simulation shall display:

- CTR backbone as LINE_STRIP;
- downsampled backbone points;
- tip marker;
- target marker;
- reference trajectory;
- obstacles;
- safety-distance marker;
- contact marker;
- coordinate frames.

### Slice 7G development visualization legend

The explicit `slice_7g_development_visual.launch.py` view uses `world` as its
RViz fixed frame and publishes the physically fixed identity transform to the
model's `base_link` frame. All simulation geometry remains expressed in
metres. The displays are independently toggleable and use this legend:

- translucent gray-blue: the curved lumen wall, triangulated from the same
  `CurvedLumen` centerline samples and radius profile used by collision and
  clearance calculations;
- cyan rings: lumen wireframe (physical boundary, safety boundary, inlet, and
  outlet);
- light blue: analytic lumen centerline;
- magenta: the exact MPPI reference path received on
  `/ctr/reference/path` (a point glyph is used when fixed-target mode supplies
  one pose rather than a drawable line);
- blue: the current CTR backbone;
- bright green: bounded, time-decimated history of real `/ctr/tip` positions;
- yellow ring: target position;
- orange sphere: a pending raw RViz target candidate (green after acceptance);
- red sphere: a rejected RViz target candidate;
- red sphere and small red arrow: current tip position and the orientation
  reported by `/ctr/tip`.

The default tip arrow is 0.05 m at the shaft plus a 0.015 m head, with shaft
radius 0.0025 m and head radius 0.006 m. Those dimensions can be changed under
the RViz **Tip pose → Shape** properties. Wall transparency defaults to 0.20
and is controlled by `simulation.visualization.surface_alpha`; the wall itself
is enabled only by the explicit development visual launch through
`enable_development_visualization`. Static geometry is reliable/transient-local
and dynamic development markers are rate-limited. Actual tip history is capped
by `actual_tip_history_max_points` and never enters headless or production mode.

### Slice 7G development target selection

The visual launch has three explicit target sources. `target_source:=profile`
is the default and preserves the deterministic `[0.015, 0.005, 0.100] m`
profile goal. `target_source:=cli` reads `target_x`, `target_y`, and `target_z`
in metres in `base_link`. `target_source:=rviz` waits without publishing a
controller reference until a valid `geometry_msgs/msg/PointStamped` arrives on
`/ctr/target_point_candidate`. These overrides require
`development_simulation:=true`; production mode rejects them.

The controller objective is position-only. Reference orientation remains the
existing identity placeholder and does not affect MPPI. The accepted reference
contains exactly one real terminal pose, so RViz renders it as the existing
magenta point rather than inventing a decorative path.

The RViz **Publish Point** tool is configured for the candidate topic. Its
`world` coordinates are transformed through the established fixed identity
`world -> base_link`. Candidates must have finite coordinates, a known frame,
a zero/latest or fresh timestamp, lie within the analytic curved-lumen extent,
preserve the configured wall safety margin, and pass the existing deterministic
approximate-model reachability sanity check. A CLI coordinate is never moved.
An RViz point near, on, or inside the physical wall but outside the safe target
region may be projected to the analytic centerline; the maximum centerline
projection distance is `simulation.development_target_selection.projection_limit`
(`0.035 m`). This admits at most 5 mm beyond the current 30 mm wall radius;
farther RViz clicks and every outside-lumen CLI coordinate are rejected. The raw
and accepted coordinates and projection distance are published as canonical JSON on
`/ctr/target_selection/record`; this makes an RViz choice directly replayable
with CLI coordinates.

While RViz selection is pending, `/ctr/target_selection/status` reports
`waiting_for_target` and the external-target MPPI mode publishes no command.
The default wait is unbounded for interactive use; set
`target_selection_timeout:=<seconds>` to request a bounded timeout. The first
accepted target closes selection for that run. Later clicks report
`target_update_rejected_motion_started`; restart the launch to select another
target. Live replanning is intentionally not part of this first implementation.
The standard RViz **2D Goal Pose** tool is not enabled because its world X-Y
plane does not coherently represent the curved lumen's three-dimensional target
surface (`RVIZ_2D_GOAL_SUPPORTED=NO`).

Fixed profile target:

```bash
ros2 launch ctr_bringup slice_7g_development_visual.launch.py \
  development_simulation:=true target_source:=profile seed:=11
```

CLI target (tested development coordinate):

```bash
ros2 launch ctr_bringup slice_7g_development_visual.launch.py \
  development_simulation:=true target_source:=cli \
  target_x:=0.0166457424 target_y:=0.00397477634 target_z:=0.102231139 \
  seed:=11
```

The same coordinate can be used by the result-producing runner:

```bash
ros2 run ctr_evaluation ctr_run_slice_7g_development \
  --development-simulation --target-source cli \
  --target-x 0.0166457424 --target-y 0.00397477634 --target-z 0.102231139 \
  --seeds 11
```

Interactive RViz target:

```bash
ros2 launch ctr_bringup slice_7g_development_visual.launch.py \
  development_simulation:=true target_source:=rviz seed:=11
```

Select **Publish Point**, click the visible lumen surface or centerline, inspect
the orange candidate and status text, then confirm the accepted yellow target
and magenta controller reference. Echo `/ctr/target_selection/record` to copy
the accepted `base_link` coordinates into a later `target_source:=cli` run.

## Simulation scenarios

1. Fixed target reaching
2. Circular trajectory
3. Elliptical trajectory
4. Helical trajectory
5. Whole-body shape matching
6. Static obstacle avoidance
7. Dynamic obstacle avoidance
8. Simulated soft contact
9. Simulated hard contact
10. Retreat and replanning
