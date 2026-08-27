# Real-Time Host Requirements for Physical-Evidence Freshness Evaluation

## Status and scope

The user-level scheduling candidate is not accepted as a correction for the
unchanged 0.10-second physical-evidence freshness contract.  Under the complete
controller and diagnostic load, one genuine physical-generation interval was
0.101496045 seconds and the maximum generation callback wall duration was
0.084447459 seconds.  This result is a genuine source violation, not merely a
ROS publication or subscription delay.

No freshness threshold, source timestamp, safety decision, production default,
host scheduler setting, cgroup, CPU topology, kernel setting, user capability,
resource limit, or privilege was changed while reaching this conclusion.

This document is a provisioning proposal only.  Every command and host change
below requires separate explicit user authorization and a rollback plan before
execution.

## Minimum authority required for a new experiment

Authorize exactly one bounded mechanism for the dedicated simulator physical
source and safety processes:

- an appropriate `RLIMIT_RTPRIO` plus `SCHED_FIFO`/`SCHED_RR`; or
- narrowly scoped `CAP_SYS_NICE` on one fixed-purpose launcher/service.

Do not grant the controller, evaluator, RViz, general shells, or arbitrary
executables real-time scheduling authority.  Do not grant `CAP_SYS_ADMIN`,
unbounded `CAP_SYS_NICE`, broad `sudo`, or general systemd-manager authority.

The proposed priority order is:

1. safety watchdog/direct physical-evidence reader;
2. physical state/tactile source;
3. ROS state and tactile publication workers;
4. controller;
5. evaluator, plotting, logging, and visualization.

Exact numeric priorities must be selected only after inspecting the host's
existing real-time users and kernel-thread priorities.  The safety and source
processes need a bounded CPU-time/runtime budget and must never busy-loop.

## CPU and memory isolation requirements

- Reserve at least one complete physical core, including all SMT siblings, for
  safety and one complete physical core for the physical source.  Do not place
  controller/evaluation work on either core.
- Verify affinity after every fork/exec using `/proc/<pid>/status` and the
  authenticated process identity; inheriting a requested mask is not proof.
- Bound BLAS/OpenMP/NumExpr pools to one worker for every evaluation process.
- Pre-fault or otherwise bound memory used by the high-priority loop only after
  measuring the impact; any memory locking requires separate authorization and
  a declared limit.
- Keep plotting, JSON/CSV generation, DDS serialization, and ROS logging off the
  physical-source and safety cores.

If cgroup/cpuset isolation is selected, authorization must name the exact
cgroup hierarchy, owner, controller delegation, CPU set, memory nodes, and
cleanup behavior.  The observer/controller processes must not be able to move
themselves into the reserved boundary.

## IRQ and kernel-worker considerations

Before choosing isolated CPUs, inspect—not modify—the current IRQ affinity,
RCU/nohz configuration, softirq load, thermal throttling, frequency governor,
and per-core kernel-worker activity.  Any proposed IRQ migration,
`isolcpus`/`nohz_full`/`rcu_nocbs` kernel argument, governor change, or real-time
kernel installation requires separate explicit authorization, a reboot plan,
and a complete rollback procedure.

The acceptance test must record involuntary context switches, migrations,
run-queue delay, page faults, thermal/frequency state, and the exact scheduler
policy/priority for safety, source, publisher, controller, and evaluator.

## Watchdog and fail-closed behavior

- The timestamp-based 0.10-second freshness rule remains unchanged.
- A new sequence with an old timestamp remains stale; a repeated sequence never
  refreshes safety.
- Scheduler setup failure, priority mismatch, affinity mismatch, producer death,
  shared-memory corruption/disconnect, missed watchdog execution, or CPU-budget
  exhaustion must command the existing safe stop and mark the run failed.
- A lower-priority watchdog outside the reserved execution context must monitor
  liveness without being allowed to relabel stale evidence as fresh.
- Real-time throttling must remain enabled unless a separately reviewed bounded
  replacement and emergency rollback mechanism are approved.

## Required rollback

The experiment launcher must capture all original scheduler policies,
priorities, affinities, resource limits, cgroup membership, and relevant sysctl
values before applying a change.  Normal exit, exception, timeout, signal, and
host restart must restore the original user-level state.  Kernel boot arguments,
systemd unit changes, users/groups, file capabilities, and sysctls require an
explicit manual rollback command and post-rollback verification.

## Commands/configuration requiring approval

The following categories must not be executed without renewed authorization:

- `setcap`/`getcap` changes involving `cap_sys_nice`;
- `chrt`, `sched_setscheduler`, or launcher code selecting FIFO/RR scheduling;
- `prlimit` or limits configuration granting `RLIMIT_RTPRIO`/negative nice;
- `taskset`, cpuset/cgroup creation, or systemd `CPUAffinity`/`AllowedCPUs`
  changes that reserve host CPUs;
- writes under `/sys/fs/cgroup`, `/proc/irq`, `/sys/devices/system/cpu`, or
  `/proc/sys`;
- changes to systemd services, PAM limits, `/etc/security/limits*`, boot-loader
  arguments, CPU governors, IRQ-balancing policy, or real-time kernels;
- any command using `sudo` or any package installation.

The renewed task must provide the exact executable identities, symbolic
principals, scheduler policy, priority values, CPU sets, runtime limits,
watchdog, rollback commands, and proof-root locations before applying them.

## Required post-provisioning gate

After authorized provisioning, rerun five sequential clean-process 10-second
probes with fresh ROS domains.  Every probe must independently show:

- maximum genuine generation and shared-memory commit gaps below 0.080 seconds;
- maximum valid source age at safety below 0.080 seconds;
- contiguous generated sequence and strictly increasing source timestamps;
- zero stale/invalid/collision/unrelated safety faults;
- complete trace layers and zero persistent processes.

Only after all five pass may the two 25-second confirmations and subsequent
target/build/test/evidence gates resume.
