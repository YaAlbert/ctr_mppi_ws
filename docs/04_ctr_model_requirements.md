# CTR Model Requirements

## Required model interface

The model shall expose:

forward_kinematics(q, params)

Input:

- q: array with shape (6,)

Output:

- backbone_points: array with shape (N, 3)
- tip_position: array with shape (3,)
- optional tip_orientation
- optional diagnostic information

## Initial model strategy

The initial version may use an approximate model.

The initial approximate model is intended for:

- ROS2 integration;
- MPPI verification;
- software architecture validation;
- controller development.

It shall not be described as a fully validated physical CTR model.

## Future model strategy

The model may later be replaced by:

- the user's MATLAB model;
- a Cosserat rod model;
- a boundary-value solver;
- a precomputed surrogate model;
- a learned residual model.

## Model abstraction

All models shall derive from a common interface.

Example classes:

- CTRModelBase
- ApproximateCTRModel
- CosseratCTRModel
- LookupTableCTRModel
- LearnedResidualCTRModel

## Validation against MATLAB

For identical q values:

1. calculate MATLAB backbone and tip;
2. calculate Python backbone and tip;
3. compare tip error;
4. compare mean backbone error;
5. compare maximum backbone error.

## Initial validation thresholds

Suggested temporary thresholds:

- tip error below 1e-3 m;
- mean backbone error below 1e-3 m.

These values are placeholders until model fidelity is confirmed.

## Model constraints

The model shall check:

- q shape;
- finite values;
- insertion limits;
- rotation limits;
- invalid tube geometry;
- invalid material parameters.