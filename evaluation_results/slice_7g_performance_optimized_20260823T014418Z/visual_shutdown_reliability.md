# Slice 7G Visual Shutdown Reliability

> Development-simulation result only. This is not hardware or production promotion evidence.

## Scope and correction

The bounded test launched the installed seed-11 curved-lumen RViz workflow,
confirmed the six required nodes and state/tip motion, allowed a short operating
interval, delivered one SIGINT to the launch process, and accounted for every
required exit and surviving launch-session process.

The preliminary 10-cycle run did **not** reproduce the historical SIGSEGV. It
did expose one project-owned teardown defect: after SIGINT, the safety node
leaked ROS 2 Humble's `RuntimeError: Unable to convert call argument to Python
object` from `rclpy.spin()` and exited 1. The entry point now treats this race as
normal shutdown only when the ROS context is already inactive; the same
exception is re-raised during an active context. Two focused regressions cover
both branches. One preliminary cycle also reached the graph and safety-ready
state but did not show state/tip motion within the 14-second acceptance window;
that startup behavior did not recur in the complete post-correction run.

The affected `ctr_safety` package rebuilt successfully. The complete practical
functional selection passed 884/884 tests: the accepted 882-node baseline plus
the two new shutdown regressions.

## Post-correction controlled cycles

All cycles used `development_simulation:=true seed:=11`; no shutdown required
SIGTERM or SIGKILL.

| Cycle | ROS domain | Startup/readiness/motion | Signal | RViz | Safety | Simulator | Launch | Shutdown (s) | Survivors | Crash marker |
| ---: | ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 181 | pass | SIGINT | 0 | 0 | 0 | 0 | 0.365 | 0 | no |
| 2 | 182 | pass | SIGINT | 0 | 0 | 0 | 0 | 0.315 | 0 | no |
| 3 | 183 | pass | SIGINT | 0 | 0 | 0 | 0 | 0.315 | 0 | no |
| 4 | 184 | pass | SIGINT | 0 | 0 | 0 | 0 | 0.315 | 0 | no |
| 5 | 185 | pass | SIGINT | 0 | 0 | 0 | 0 | 0.365 | 0 | no |
| 6 | 186 | pass | SIGINT | 0 | 0 | 0 | 0 | 0.365 | 0 | no |
| 7 | 187 | pass | SIGINT | 0 | 0 | 0 | 0 | 0.365 | 0 | no |
| 8 | 188 | pass | SIGINT | 0 | 0 | 0 | 0 | 0.365 | 0 | no |
| 9 | 189 | pass | SIGINT | 0 | 0 | 0 | 0 | 0.365 | 0 | no |
| 10 | 190 | pass | SIGINT | 0 | 0 | 0 | 0 | 0.314 | 0 | no |

Result: **10/10 clean controlled shutdowns**. Mean shutdown duration was
0.345 seconds (range 0.314--0.365 seconds). Every parameter validator,
simulator, safety supervisor, MPPI controller, reference manager, RViz process,
and launch process exited zero.

## Final functional smoke

A separate 5-second matched seed-11 smoke evaluation selected ROS domain 124.
It completed orchestration successfully, produced valid baseline/candidate
artifacts, reported candidate navigation success, recorded zero collision
duration, and left no owned process. Its temporary raw outputs were not
committed.

## Reproduction

```bash
source /opt/ros/humble/setup.bash
source install_slice7g_development/setup.bash
DISPLAY=:0 ROS_DOMAIN_ID=181 ros2 launch ctr_bringup \
  slice_7g_development_visual.launch.py \
  development_simulation:=true seed:=11
```

After readiness and visible motion, send one Ctrl-C to the foreground launch.
Choose a different unused domain in 100--199 for concurrent or repeated runs.

## Limitations

- The original SIGSEGV remains a historical, non-reproduced low-priority EVAL-005 observation; no core was available for attribution.
- Ten clean repetitions do not prove all RViz, graphics-driver, or ROS teardown schedules.
- Only simulator processes ran. Production defaults are unchanged and no production attempt was consumed.
