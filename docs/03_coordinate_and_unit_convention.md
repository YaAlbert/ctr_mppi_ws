# Coordinate and Unit Convention

## Unit system

The entire project shall use SI units.

- length: meter
- angle: radian
- velocity: meter per second or radian per second
- acceleration: meter per second squared or radian per second squared
- force: newton
- time: second

## Coordinate frames

Recommended frames:

- world
- base_link
- ctr_tip
- tactile_frame

## Base orientation

The CTR nominal extension direction shall be +Z.

- X and Y represent transverse bending directions.
- Z represents the nominal insertion direction.

## ROS convention

All published poses, paths, point clouds and markers shall include a
valid frame_id and timestamp.

## Configuration convention

All geometry parameters in YAML shall use meters.

All angles in YAML shall use radians.

Any imported MATLAB data using millimeters or degrees must be
converted before comparison.