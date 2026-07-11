# Cost Function Definition

## Total cost

J =
    w_tip * C_tip
  + w_shape * C_shape
  + w_control * C_control
  + w_smooth * C_smooth
  + w_obstacle * C_obstacle
  + w_terminal * C_terminal
  + w_force * C_force
  + w_limit * C_limit
  + w_stability * C_stability

## Tip tracking cost

C_tip =
    ||p_tip - p_tip_ref||^2

## Shape tracking cost

C_shape =
    mean_i(
        ||p_i - p_i_ref||^2
    )

Backbone points must be aligned or resampled to the same number of
points before comparison.

## Control magnitude cost

C_control =
    ||u_t||^2

## Control smoothness cost

C_smooth =
    ||u_t - u_previous||^2

## Terminal cost

C_terminal =
    ||p_tip_H - p_tip_ref_H||^2

Optional terminal shape cost may also be added.

## Obstacle cost

For each backbone point, calculate the signed or unsigned distance to
the nearest obstacle.

A suggested initial formulation is:

if distance < collision_distance:
    add hard_collision_penalty
elif distance < safety_distance:
    add proximity penalty
else:
    add zero or small inverse-distance penalty

## Tactile force cost

C_force =
    max(
        0,
        force_norm - force_warning_threshold
    )^2

## Joint-limit cost

The initial implementation shall enforce hard clipping.

A soft pre-limit penalty may also be added near physical limits.

## Stability cost

The stability term shall initially be disabled.

Later versions may include:

- elastic stability;
- tube relative rotation constraints;
- curvature limits;
- buckling margin;
- torsional strain limits.