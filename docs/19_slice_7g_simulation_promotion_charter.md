# Slice 7G Simulation Promotion Charter

## Status and authority

The user-selected project endpoint is `simulation_only_promoted_completion`:
complete the curved-lumen CTR simulation with MPPI control, simulated tactile
feedback, and safety supervision; pass an isolated build and static gate; run
one separately authorized deterministic simulation-acceptance campaign; obtain
an independent audit; and create external promotion and final-closure records.

The operative machine-readable charter is
`config/slice_7g_simulation_charter.json`, schema
`ctr-slice-7g-charter-7`. Charter v6 and earlier remain historical inspection
material and are rejected as sufficient runtime authority. The v7 charter is declarative
and is not executable. Its status is
`APPROVED_SCOPE_NOT_RUNTIME_AUTHORIZATION`; `execution_authorized`,
`launchable`, `runtime_attempt_allocated`, `domain_allocated`, and
`output_root_allocated` are all false.

Slice 7F remains closed as approved static/offline evidence only. Slice 7G code
correction, build verification, runtime acceptance, audit, promotion, and
project closure have not started merely because this charter exists.

The canonical charter is compact UTF-8 JSON with recursively sorted object
keys, `ensure_ascii=False`, no non-finite values, and no trailing newline. Its
domain-separated logical identity is:

```text
SHA-256(
  b"ctr-slice-7g-charter-canonical-7\0" || canonical_charter_bytes
)
```

The identity is external to the canonical bytes. The charter binds its
authoring branch, HEAD, scoped pre-charter source snapshot, and the immutable
Slice 7F closure. The current dirty worktree is an authoring baseline, not an
acceptance subject. A new immutable source snapshot after implementation is a
mandatory runtime-entry gate.

## OS-principal authority contract

Runtime authority is separated from the untrusted campaign process. A future
root provisioning phase creates the non-login `ctr7g-authority` service
principal, the unprivileged `ctr7g-campaign` process principal, and the
`ctr7g-runtime` primary group. Numeric identities are deliberately null in
source and must be bound by a later root-owned bootstrap. Supplementary groups
confer no authority and the campaign supplementary-group set is exactly empty.

The source-owned fixed locators are `/etc/ctr-mppi/slice-7g-authority/bootstrap.json`,
`/usr/libexec/ctr-mppi/ctr-slice7g-authorityd`,
`/var/lib/ctr-mppi/slice-7g-authority`,
`/run/ctr-mppi/slice-7g-authority.sock`, and
`/opt/ctr-mppi/slice-7g`. No CLI, environment variable, campaign ID, callback,
or caller path may replace them. The local AF_UNIX authority protocol uses
bounded canonical framing and numeric `SO_PEERCRED`; it introduces no network
authority and accepts no caller-supplied authorization booleans.

The global budget is one authority-owned 0/1 lineage across every campaign ID,
domain, output root, authorization record, and service restart. Prepare is
in-memory and non-consuming. A durable `COMMITTED` successor is the first
permanent effect and must precede every campaign/project child. The sole
precommit ROS exception is the separately classified graph observer described
below; it is a non-consuming provider and must be fully gone before commit.
The only committed successors are `COMPLETED` and `FAILED_AFTER_COMMIT`; none
can return to `UNCONSUMED`.
Precommit failure rolls back every task-owned provisional resource. Mandatory
postcommit revocation creates a revisioned authority trigger for the fixed
root-owned systemd revocation units, which stop only the complete
`ctr-slice7g-campaign.service` control group without granting the authority
principal sudo or `CAP_KILL`.

The evidence parent remains `/home/ankid/ctr_mppi_evidence/slice_7g`. A later
provisioning phase must implement the deterministic ACL/mode policy: the
campaign principal cannot list or mutate parent entries; the authority creates
and retains the campaign-root descriptor; authority/control records remain
authority-only; and exactly 15 cell directories are narrowly writable by the
runtime group. This source task applies no ACL and allocates no root.

## Root cleanup authority and exclusive observer supervision

Charter v7 separates the unprivileged authority daemon from two fixed root
services. `ctr-slice7g-cleanup-authority.service` alone owns the root-only
cleanup ledger and appends immutable, contiguous revision/anchor/head triples.
`ctr-slice7g-observer-supervisor.service` alone receives narrowly delegated
cgroup control and creates one exclusive leaf before releasing a blocked,
deprivileged observer stub. The observer receives no delegation. Complete leaf
membership, not PGID/SID or sampled ancestry, defines containment and cleanup.

The services accept only closed AF_UNIX `SOCK_SEQPACKET` operations with
`SO_PEERCRED`, peer process/cgroup reconciliation, contiguous per-connection
sequences, nonce/token binding, and at most two sealed output memfds. No request
selects an executable, argv, environment, signal, PID, path, or cgroup. Cleanup
quarantine is durable and consumes no campaign attempt. Recovery requires a
separate future OS principal and fresh helper-owned four-source evidence; it is
unavailable until provisioned.

Privileged Python bootstrap is rooted in the fixed root-owned immutable
bootstrap-3 record, not in the authority-owned installed-runtime `root_path`.
The record binds the complete service-wrapper/module and observer
executable/interpreter inventory with physical metadata and final pathname/inode
barriers. Authority-owned runtime records remain evidence only. The two root
services own independent runtime directories and sockets. Their only DAC
exception is `CAP_DAC_READ_SEARCH` for exact read-only authority evidence;
`CAP_CHOWN` is limited to assigning the fixed socket group, and
`CAP_DAC_OVERRIDE` is prohibited. Stopping either helper cannot remove the
other helper's runtime directory or socket.

Installed-runtime-manifest v3 is the Charter-v7 authority form. It binds the
exact root owner/group of every installed member in addition to path, type,
mode, link count, size and digest. Version 2 remains available only for
historical structural inspection and cannot authorize a v7 helper or daemon.

## Narrow ROS graph observation authority

Charter v7 binds one precommit process class,
`PRECOMMIT_ROS_GRAPH_OBSERVER`, and no other precommit project or ROS child.
Its command is exactly `/opt/ros/humble/bin/ros2 node list --no-daemon` with
`shell=false`, a manifest-authenticated root-owned Python interpreter and ROS
module origins, a closed installed environment, a typed `ROS_DOMAIN_ID` in
100--199, the manifest-fixed RMW implementation and cwd, and the root
supervisor's exclusive non-delegated observer leaf cgroup.
There is no PATH lookup, daemon, descendant, retry, caller process factory, or
inherited environment.

The authority daemon opens one connection/PID-bound, non-durable,
non-consuming observation session for an authorization. Its absolute lifetime
is 1,800 seconds. A daemon-generated service nonce and an initial acyclic
session-binding identity bind the connection, authenticated peer, process,
cgroup, authorization, installed manifests, creation/deadline, and daemon
generation before any receipt exists. Candidate domains are considered once each in ascending
order; the three non-ROS sources run first, and the graph observer runs only
for a candidate still clear. At most 100 precommit observers and one concurrent
observer are allowed. Each has a 10.0-second execution bound, 1,048,576-byte
stdout and stderr bounds, and zero retries.

Successful output is strict UTF-8 with at most one terminal LF and at most
65,536 unique NFC-normalized absolute node names, each no longer than 8,192
UTF-8 bytes; stderr is empty. The receipt binds the absolute executable,
interpreter/module origins, argv, environment, cwd, exclusive leaf, PID, PGID, process
start time, timestamps, output sizes/digests, parsed node tuple, and cleanup
identity, as well as the session identity, service nonce, phase, phase-local
ordinal, transaction ordinal, candidate and daemon-reconstructed four-source
identity. Public requests cannot supply receipts, provider results, counters,
captured output, process claims, cleanup claims, or four-source evidence.
The result is unusable until the observer leaf is empty and removed, pipes and
sealed descriptors are reconciled, no daemon exists, and two identical
process/DDS-clear samples span at least 0.5 seconds within a 5.0-second ceiling.
Immediate-parent exit, `setsid()`, PGID changes, and double forks do not end or
escape complete leaf cleanup; numeric process identities without retained
start-time/leaf provenance are never signalled.

Any provider, protocol, parsing, cleanup, or reconciliation failure removes
the session from the active index before returning the primary failure.
Authoritatively clean failure permits a fresh session; uncertain process/DDS
ownership produces a root-ledger `QUARANTINED` state and rejects new work with
a stable cleanup error. Disconnect, restart, expiry, revocation, peer
replacement, and cgroup replacement apply the same invalidation rule.

The global-lease member of every four-source observation is a real, read-only,
shared-registry observation under the fixed evidence parent. It authenticates
the registry lock, sealed reservation/release/binding inventory, physical
inodes, canonical records, and stable revision identity; a derived nonce or
caller claim is never lease authority. The durable cleanup-guard revision chain
under the fixed authority state root is separate from the non-durable
observation session and from the attempt budget. `ACTIVE` or `QUARANTINED`
cleanup state blocks observation, prepare, and commit after restart.

Only the completed canonical four-source observation permits issuance of a
nonrenewable 300-second prepare token. Token expiry discards the observation
and rolls back provisional resources; a new transaction starts observation
from zero. The COMMITTED revision binds the observation session, ordered
receipts, counts, selected domain and lease, and prepare identity. Exactly one
postcommit observer then repeats the full checks for the selected domain.
The immutable counts are precommit 0--100, postcommit 0--1, and total 0--101.
No campaign child may start until the postcommit four-source recheck succeeds;
postcommit failure starts none, records `FAILED_AFTER_COMMIT`, and permanently
consumes 1/1.

All primitive mapping inputs at public governance boundaries must be exact
built-in dictionaries. Dict subclasses and arbitrary `Mapping`
implementations are rejected before any caller-controlled iteration, lookup,
length, keys, items, or conversion hook can run. Nested containers are
detached and validated recursively; module-owned immutable records are
reconstructed through a separate internal path. Expected malformed inputs are
reported only with stable `Slice7GGovernanceError` codes.

## Included and deferred scope

Current completion includes circular-arc curved-lumen geometry and target
generation, curved-lumen MPPI integration, simulated tactile generation,
tactile processing and tactile cost integration, safety-supervisor state and
gating behavior, readiness monitoring, deterministic evaluation orchestration,
clean isolated build and static tests, one governed acceptance campaign,
independent audit, promotion, and final simulation-project closure.

Physical CTR drivers, calibrated physical tube parameters, physical tactile
calibration, motor/control-board commissioning, physical safety certification,
clinical use, and autonomous clinical decision-making are deferred. Shape,
obstacle, stability, nonideal-actuator, and mock-hardware work is excluded only
while the selected profile keeps those paths disabled. The implementation gate
must prove that no unfinished cost path is reachable.

Promotion makes no physical-hardware, safety-certification, clinical, or
real-time-performance claim. Physical deployment is a future extension with
its own parameters, commissioning, timing contract, runtime authorization, and
acceptance evidence.

## Required source contracts before runtime authorization

The charter records these coordinated contracts. Their current source
implementation, including the narrow observer correction, requires independent
review and a successor immutable snapshot before it can become build authority:

1. Start and verify the safety supervisor.
2. Enable simulated tactile input.
3. Enable tactile cost.
4. Enable safety tactile handling.
5. Route controller output through the safety supervisor instead of publishing
   directly to `/ctr/safe_command`.
6. Authenticate tactile publication in readiness.
7. Authenticate safety-supervisor readiness and fault state.
8. Replace random domain selection with process-, ROS/DDS-, and ledger-aware
   collision checking.
9. Prove the selected profile cannot reach unfinished `NotImplementedError`
   cost paths.
10. Produce an immutable post-implementation source snapshot.
11. Generate and validate a repository-owned immutable campaign plan with an
    exact bijection over the 15 required cells.
12. Capture and seal one complete physical evidence package per cell;
    descriptor-authenticate its retained bytes; and use the repository-owned
    campaign-wide result reconciler to recompute all 15 cell results,
    aggregates, timing limitations, and promotion eligibility.
13. Propagate a single ledger-bound ROS domain through all cells and child
    processes, with no lower-level replacement.

Runtime authorization additionally requires independent charter review,
completed implementation, authenticated simulation parameters, an isolated
passing build and complete test matrix, a validated runtime plan and argv, a
new empty external output root, one freshly reserved domain, an unconsumed 0/1
campaign budget, and a separate explicit authorization.

## Deterministic campaign

One governed campaign contains all 15 scenario/seed run cells:

| Charter scenario | Source scenario | Geometry | Seeds | Duration per cell |
|---|---|---|---|---:|
| `centerline` | `centerline_target` | `circular_arc` | 11, 22, 33, 44, 55 | 25.0 s |
| `lateral_offset` | `lateral_offset_target` | `circular_arc` | 11, 22, 33, 44, 55 | 25.0 s |
| `near_safety_boundary` | `near_safety_boundary_target` | `circular_arc` | 11, 22, 33, 44, 55 | 25.0 s |

The task is `curved_lumen_navigation`. One campaign is the governed attempt;
the 15 deterministic cells are not 15 independent attempt allocations.
The repository-owned pure plan generator expands exactly 3 approved scenarios
by 5 approved seeds into exactly 15 unique cells. Each cell binds its stable
ID, charter and campaign identities, scenario/source pair, seed, task,
geometry, duration, simulation mode, attempt-ledger identity, single ROS
domain, campaign and cell output paths, exact argv, and metric-profile
identity. Missing, duplicate, extra, reordered, or differently bound cells are
invalid. Each exact argv uses that cell's deterministic output child beneath
the one ledger-bound campaign output root.

The prospective evaluation-runner template uses only currently supported CLI
arguments:

```text
ctr_run_evaluation
  --experiment-group <campaign_id>
  --task curved_lumen_navigation
  --curved-lumen-type circular_arc
  --scenario <source_scenario_id>
  --seed <seed>
  --duration 25.0
  --runtime-mode simulation
  --output-root <new_external_output_root>
```

The prospective environment binds `ROS_DISTRO=humble` and
`ROS_DOMAIN_ID=<allocated_domain>`. A later immutable plan must expand every
placeholder. Required effective child-launch behavior includes evaluation,
curved-lumen, tactile simulation, tactile cost, and safety supervision enabled;
hardware mode disabled; and direct simulation publication to the safe-command
topic disabled. These are implementation gates because the current runner does
not yet propagate all of them. The current `ctr_run_evaluation` CLI accepts the
per-cell options, but the runner does not yet orchestrate or authenticate the
complete 15-cell campaign. This metadata is not launchable and authorizes no
process.

## Readiness contract

Readiness must be achieved before 10.0 seconds and remain stable for at least
10 samples spanning at least 0.5 seconds. The maximum variation is `5.0e-5`
for joint state and `5.0e-5 m` for tip position. State and tip values must be
finite and correctly dimensioned, required processes/nodes and services must be
present, and required topics must publish without stale state, command,
tactile, reference, or safety data. No safety fault may be active or latched at
acceptance start.

The required topics are `/ctr/state`, `/ctr/tip`, `/ctr/reference/tip`,
`/ctr/reference/path`, `/ctr/mppi_command`, `/ctr/safe_command`,
`/ctr/tactile/state`, and `/ctr/safety/status`. The required services are
`/ctr/start_experiment` and `/ctr/stop_experiment`.

## Acceptance metrics

Every required run cell must pass every promotion-blocking row.

| Metric | Unit | Per-cell aggregation/comparison | Threshold | Promotion blocking |
|---|---|---|---:|---|
| Readiness success | boolean | all equal | true | yes |
| Steady-state error | m | maximum ≤ | 0.003 | yes |
| Final goal error | m | final ≤ | 0.003 | yes |
| Goal-hold duration | s | contiguous ≥ | 0.5 | yes |
| Minimum physical wall clearance | m | minimum ≥ | 0.0 | yes |
| Minimum safety-margin wall clearance | m | minimum ≥ | 0.002 | yes |
| Collision sample count | count | sum = | 0 | yes |
| Safety-fault count | count | sum = | 0 | yes |
| Non-finite value count | count | sum = | 0 | yes |
| Missing required-topic count | count | sum = | 0 | yes |
| Missing required-result-file count | count | sum = | 0 | yes |
| Valid aligned sample count | count | minimum ≥ | 20 | yes |
| Invalid sample percentage | percent | maximum ≤ | 10.0 | yes |
| Saturation percentage | percent | maximum ≤ | 1.0 | yes |
| Process exit status | exit code | all processes = | 0 | yes |
| Deadline-overrun percentage | percent | maximum ≤ | 5.0 | **no; diagnostic** |
| RMSE | m | report only | none | no; descriptive |
| Inside tracking-tolerance percentage | percent | report only | none | no; descriptive |

The descriptive tracking tolerance remains `0.001 m`; it is used to report the
inside-tolerance percentage but is not an independent promotion gate. RMSE is
also descriptive. Values above `0.003 m` fail the operative steady-state or
final-goal contract regardless of descriptive reporting.

Deadline-overrun percentage is recorded for every run and compared with the
configured 5.0% target. `timing_pass` must not enter functional/numerical
acceptance reasons, and timing failure alone cannot block simulation
promotion. If timing fails, the promotion report must label the campaign
non-real-time. A future physical endpoint must define a separate
promotion-blocking timing contract.

## Domain and attempt governance

No domain is allocated by this charter. A future authorized preflight must
inspect active processes, the ROS graph, DDS participants, and the external
domain ledger; choose exactly one collision-free domain in 100–199; and bind it
to an immutable external attempt ledger and the identity of the separate
runtime-authorization record. The governance API rejects an allocation
proposal without that authorization identity. This single ledger-bound ROS domain
must be propagated unchanged through all 15 cells and every campaign
subprocess. `_run_one` and other lower-level helpers must not replace a
preflight-supplied `ROS_DOMAIN_ID`. A missing, occupied, changed, or unbound
domain fails before process creation. Allocation evidence and release/accounting
evidence are required. It may not choose a second domain after any campaign
process starts.

The maximum is one acceptance campaign. Current consumption is 0/1 and retries
authorized are zero. A preflight failure before process creation leaves the
budget at 0/1. Any process start consumes 1/1. A second start or retry requires
a new user governance decision.

The operative exactly-once model is the versioned
`ctr-slice-7g-attempt-ledger-1` ledger plus
`ctr-slice-7g-attempt-event-1` events. Every event binds the charter and
campaign identities, expected ledger revision, canonical predecessor-ledger
identity, final campaign-plan identity for process start, runtime-authorization
identity for allocation, unique event ID and identity, previous/resulting counts, allocation
state, process-start consumption state, and a strictly parsed UTC timestamp.
Stale predecessors, duplicate events, a second start, and every retry fail
closed.

The governance module only derives and validates transitions; a pure function
does not enforce cross-process exactly-once behavior. The future ledger writer
must use atomic compare-and-swap against the expected predecessor, or equivalent
exclusive no-replace creation, and durably commit the `process_start_commit`
transition from 0/1 to 1/1 before process creation. If that atomic commit
fails, the process must not start. Two proposals from one predecessor are not
two committed starts.

## Campaign result reconciliation

Every planned cell must produce exactly one sealed physical evidence package.
Its `ctr-slice-7g-cell-evidence-envelope-1` envelope binds the charter,
campaign, plan, exact cell, metric profile, committed 1/1 ledger identity and
revision, unique process-start event, runtime authorization, single ROS
domain, campaign/cell output roots, exact argv, exit status, mandatory member
descriptors, and a domain-separated evidence-projection identity. The closed
mandatory physical evidence roles are:

1. `invocation_process_start_receipt`
2. `runtime_authorization_binding`
3. `readiness_trace`
4. `safety_trace`
5. `tactile_trace`
6. `cell_result`
7. `output_inventory_receipt`

The finalized package root is mode `0555`; every required, nonempty regular
member is mode `0444`, link count one, and occupies a unique inode. Complete
inventory equality is mandatory. Descriptor-relative no-follow reads reject
missing or extra files, writable paths, symlinks, hardlink aliases, unsafe
paths, unstable metadata, and content or directory-entry replacement. The
retained `evidence_projection.json` record uses
`ctr-slice-7g-cell-evidence-projection-1`; its bytes, logical identity, and the
physical package identity are recomputed from retained observations. The
projection does not contain its own digest, and the envelope is excluded from
the member projection to avoid a digest cycle.

The projection identity algorithm is
`sha256:ctr-slice-7g-cell-evidence-projection-canonical-1`; the relocatable
physical package identity algorithm is
`sha256:ctr-slice-7g-cell-evidence-package-physical-1`. The latter binds the
root mode and the exact physical envelope/projection observations, which in
turn bind every mandatory member descriptor.

Package authentication retains a private, open root descriptor throughout
physical authentication, semantic ledger/runtime/plan/cell reconciliation,
and a final physical barrier. At that barrier it re-enumerates the exact
nine-file inventory, reopens every member relative to the retained root with
no-follow semantics, rechecks device/inode/type/mode/link/size/time metadata,
and rehashes every file. It then reopens the public package pathname from an
authenticated filesystem-root descriptor component by component, rejecting
parent or root symlinks, and requires that pathname to identify the retained
root inode. Rename-away, same-path replacement (including byte-identical new
inodes), parent substitution, inventory changes, and member replacement fail
closed. The immutable observation's `package_root` is only the pathname
verified at that final barrier; it is not a claim that an external actor cannot
change the filesystem after the function returns.

The designated `ctr-slice-7g-cell-result-2` bytes bind the committed authority
and carry measured values, but caller-provided evidence identities and
authority booleans are not fields and are never proof. `authenticated=true`,
`authorized=true`, or an invented digest cannot substitute for physical
package authentication. Standalone cell/campaign result canonicalization is
structural serialization only, not an authentication or reconciliation
verdict.

The closed `ctr-slice-7g-campaign-evidence-seal-1` record is stored as
`evidence/campaign_evidence_seal.json` beneath the exact ledger-bound campaign
output root. It binds the charter, runtime authorization, committed ledger
identity and revision, process-start event, campaign plan, one ROS domain, the
campaign output root, and the exact ordered set of 15 deterministic
`evidence/packages/<cell_id>` paths and physical package identities. The seal
has mode `0444` and link count one; the campaign, evidence, and package roots
are finalized at mode `0555`. Absolute overrides, sibling-prefix matches,
traversal, parent symlinks, and packages outside that ledger-bound tree are not
accepted.

The authoritative `ctr-slice-7g-campaign-result-3` path accepts the charter,
committed ledger, immutable campaign plan, and the single ledger-bound campaign
root. It opens the campaign root, evidence root, seal, packages directory, and
all packages descriptor-relatively with no-follow semantics. It authenticates
every package, parses and recomputes each result
identity, requires an exact one-package/one-cell bijection, then recomputes all
aggregate counts, functional failure reasons, timing status, non-real-time
limitations, promotion eligibility, canonical campaign bytes, and logical
identity. It also binds the result to the domain-separated
`sha256:ctr-slice-7g-campaign-evidence-snapshot-canonical-1` identity derived
from the canonical seal and all 15 sealed package identities. Any optional
supplied campaign record must exactly equal this
recomputed record. Missing, duplicate, extra, reused, stale, forged, or
context-mismatched evidence fails closed; aggregates cannot be caller
assertions. Promotion is computed only from exactly 15 authenticated packages.

Before evidence authentication, governance acquires a nonblocking shared
`flock` on the finalized seal. It holds that lock while all 15 private package
root authorities remain live, through semantic reconciliation, aggregate and
snapshot recomputation, all individual barriers, the campaign final checks,
and immutable result construction. Every future authorized evidence writer
must acquire the corresponding exclusive lock before any mutation; a busy,
unavailable, or unsupported lock fails closed, and failed exclusive acquisition
prohibits mutation. Before return, all 15 inventories and all 135 package files
are revalidated and rehashed; public path/root identities, cross-package root
and member-inode uniqueness, package identities, the seal, and derived
cell/campaign values are reconciled again. No evidence-package read occurs after
that final barrier. A late change returns no partial authoritative campaign
result.

This cooperative seal protocol defines the accepted writer threat model. The
sequential rehashes do not claim a mathematically atomic filesystem instant.
Same-owner or privileged mutation that deliberately bypasses the mandatory
exclusive lock is a governance violation outside that writer threat model; the
external immutable evidence snapshot and later independent audit remain
required.

All public filesystem-path boundaries normalize a caller path exactly once.
They accept an exact string or a supported `PathLike` whose single
`os.fspath()` call returns an exact string, reject bytes, traversal, aliases,
NUL/control characters and unsafe components, and translate ordinary
caller-controlled conversion failures to stable governance errors. They do
not coerce with `str()` and do not catch `BaseException` control flow.

Functional promotion passes only when all 15 cells satisfy every blocking
gate. Timing remains diagnostic only: a timing-only failure leaves functional
promotion eligible but makes the non-real-time limitation mandatory.

## Build, evidence, audit, and promotion

The build/static phase uses fresh external build, install, and log directories
for `ctr_interfaces`, `ctr_bringup`, `ctr_model`, `ctr_mppi_controller`,
`ctr_tactile`, `ctr_sim`, `ctr_safety`, and `ctr_evaluation`. It requires
successful `colcon build`, `colcon test`, and zero-failure `colcon test-result`;
cache-disabled Python tests; interface/package resolution; static launch,
runtime-plan, and charter validation; no source-tree bytecode/cache; no
temporary RPATH/RUNPATH; and no unresolved project dependency.

Future campaign output is external under a dedicated Slice 7G evidence parent.
It must retain the source snapshot, build/test receipts, attempt ledger,
immutable 15-cell campaign plan, immutable argv and environment, domain
allocation, the finalized locked campaign seal, the campaign evidence snapshot
identity, sealed per-cell physical evidence packages, physical package
authentication, contextual campaign-wide reconciliation,
per-cell raw output, metrics, readiness/safety/tactile/controller traces,
process stdout/stderr, report source and final report, physical inventory,
preservation evidence, and an independent-audit target. This charter allocates
neither the parent nor an output root.

Promotion requires implementation review, build/static approval, exactly one
authorized completed campaign, campaign plan/result reconciliation, every required run cell passing, independent
read-only audit approval, an external immutable promotion record, and final
simulation-project closure. The promotion record must repeat the simulation-
only, non-real-time, non-hardware, and non-clinical limitations.

## Remaining phases

1. Clean-session independent review of this charter and governance parser.
2. Independent review of the coordinated thirteen-blocker implementation and
   the narrow ROS graph observer/observation-session correction.
3. Post-implementation immutable source snapshot and isolated build/static
   verification.
4. Separate exactly-once runtime authorization and deterministic campaign.
5. Independent read-only acceptance audit.
6. External promotion decision and final simulation-project closure.

No phase listed here is implicitly authorized by this document.
