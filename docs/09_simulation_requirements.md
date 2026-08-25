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
