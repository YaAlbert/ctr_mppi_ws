# Slice 7G coordinated runtime source implementation

Status: source implementation only; independent code review and the charter's
post-implementation snapshot/build/runtime gates remain mandatory.

This document records the repository source path that implements the governed
requirements in `ctr-slice-7g-charter-7`. Charter v6 and earlier are retained
only for historical inspection and are not runtime authority. This is not runtime
authorization, an attempt ledger, campaign evidence, an acceptance result, or
a promotion decision.

## Charter-v7 privileged authority source

The source adds closed standard-library AF_UNIX protocols, authority-daemon
candidate, path-independent installed-runtime authenticator, global
campaign-independent 0/1 budget state machine, closed process/environment
manifests, authority-mediated output-root policy, and four root-owned systemd
templates. Production locators are constants and test-only factories are
private. Campaign methods authenticate the exact numeric UID, effective GID,
PID, start time, executable, argv, environment, cwd, cgroup, and installed
origin through `SO_PEERCRED` plus `/proc`; supplementary groups are never
trusted.

The budget revision zero must be provisioned later by the authority principal;
runtime cannot create it. Prepare is connection-bound and non-consuming.
`COMMITTED` is durably fsynced immediately before the first campaign/project
child,
survives restart, and can transition only to `COMPLETED` or
`FAILED_AFTER_COMMIT`. A service restart with an abandoned committed revision
retains consumption and emits the fixed-unit termination trigger. Precommit
rollback attempts every owned cleanup step for `Exception` and
`BaseException`, reconciles path/inode authority before removal, preserves the
primary failure, and never restores a postcommit attempt.

The candidate systemd templates bind `ctr7g-campaign:ctr7g-runtime`, an empty
supplementary-group set, `NoNewPrivileges=yes`, no capabilities, no delegation,
and `KillMode=control-group`. Timeout values are authenticated rendering slots,
not source defaults. The revocation helper invokes only the fixed root-owned
campaign unit stop action; source tests substitute a synthetic runner and do
not call systemd.

Charter v7 additionally separates a fixed root-owned cleanup-ledger service
from a fixed root observer supervisor. The cleanup service is the only writer
of immutable revision/anchor/head triples. The supervisor creates an exclusive
cgroup-v2 leaf and places a blocked stub there before dropping credentials and
executing the fixed ROS CLI. Its bounded output is transferred as two sealed
memfds; the unprivileged daemon can only query cleanup state and request that
one observer. It cannot write the ledger, manipulate systemd/cgroups, or launch
an arbitrary command.

The helper wrappers authenticate a fixed root-owned bootstrap-3 record before
importing privileged implementation modules. That record binds complete
physical service executable, module, interpreter and ROS CLI identities; an
authority-owned installed-runtime manifest is checked only as evidence and
cannot redirect the code root. All current privileged/install nested records
use exact recursively closed dictionaries and ordered inventories. Current
installed-runtime-manifest v3 includes root owner/group identity for every
member; v2 is historical-only and is not accepted by the production v7
assembly. The
services have distinct mode-0755 runtime directories and root-owned,
authority-group-connectable mode-0660 sockets. Their unit contract grants only
`CAP_DAC_READ_SEARCH` for fixed read-only authority evidence, plus `CAP_CHOWN`
for the fixed socket group assignment, and never grants `CAP_DAC_OVERRIDE`.

No OS account, installed tree, authority state, global budget, socket, service,
domain, cgroup, observer, recovery authority, or output root is created by this
source task.

`SLICE7G-AUTH-001=SOURCE_IMPLEMENTED_AND_TESTED_PRIVILEGED_INSTALL_AND_RUNTIME_PROOF_PENDING`

## Explicit development-simulation path

The separately selected `--development-simulation` path is a practical,
user-level software-simulation workflow and does not weaken or replace the
production authority entry point. It reuses the existing curved-lumen launch,
simulator, MPPI, tactile, safety, recording, metrics, and plotting components.
It requires `runtime_mode=simulation`, creates no production budget or attempt,
uses a user-owned ROS-domain exclusion lock and validated output root, and
marks all generated reports as not production promotion evidence.

The isolated Ubuntu build and functional run on 2026-08-23 completed the
seed-11 smoke pair and seeds 11, 22, and 33. Each example reached readiness,
reported navigation success, had zero collision/safety/tactile events, and
passed child cleanup. RViz also started with the curved-lumen model and
reference displays. These results do not constitute privileged-install,
installed-runtime, ROS/DDS authority, or physical-hardware proof.

## Narrow precommit graph-observer transaction

The SLICE7G-AUTH-001 source contract is implemented and source-tested through a
single closed provider class,
`PRECOMMIT_ROS_GRAPH_OBSERVER`. Production accepts only the installed-manifest
binding for `/opt/ros/humble/bin/ros2 node list --no-daemon`, its root-owned
Python interpreter and ROS modules, `shell=false`, the supervisor-owned
exclusive observer leaf cgroup, fixed cwd, and a closed environment whose only
candidate-dependent value is the
typed domain in 100--199. Bare `ros2`, PATH resolution, caller cwd/environment,
callbacks, daemon use, descendants, and other precommit ROS/project processes
remain prohibited.

The authority daemon maintains a separate in-memory observation session bound
to one authorization, connection, peer UID/GID/PID/start time, installed
runtime, process/environment manifests, cgroup, nonce, and 1,800-second
deadline. It records candidate domains and authenticated receipts in order,
permits no candidate twice, caps precommit observers at 100, and reserves no
budget. Disconnect, restart, revocation, expiry, peer/cgroup replacement,
provider failure, protocol/parsing failure, or cleanup failure invalidates the
complete session immediately. Cleanup uncertainty additionally poisons the
authority generation so new work fails closed rather than appearing to have an
active session.

The public request schema contains candidate-operation inputs only. Daemon-owned
providers reconstruct active-process, DDS-port, the descriptor-authenticated
shared global-lease registry, and ROS-graph results;
caller-created receipts, output/process/cleanup claims, counters and
four-source observations are unknown fields. Before observation, the daemon
creates an acyclic session-binding identity from its unpredictable service
nonce, connection, authenticated peer/process/cgroup, installed authorities,
deadline and daemon generation. Receipts bind that identity and nonce plus the
candidate, phase and ordinals; the final observation binds the ordered receipt
identities, and prepare then binds the final observation. Consequently a
byte-identical receipt from another, expired, failed, revoked or restarted
session is not authority.

Each observer has a 10.0-second execution limit, 1 MiB bounds for each binary
stream, strict UTF-8/NFC absolute-node parsing, empty stderr, no retry, and an
owned PID/PGID. Its result becomes eligible only after two stable clear
process/DDS samples separated by at least 0.5 seconds within 5.0 seconds.
The receipt binds process identity, executable/interpreter/module origins,
argv/environment/cwd/cgroup, output identities, ordered parsed nodes, and the
cleanup-barrier identity.

After the blocked child enters its exclusive leaf and is released, the
supervisor requires a dedicated process session and reauthenticates the
post-exec interpreter/executable, exact argv, closed environment, credentials,
session, cgroup, cwd and retained process identity before accepting output.
The lease observer retains every final record descriptor through the last
pathname/inode/digest barrier. Privileged clients bind every receipt to the
exact operation, sequence, connection nonce, request nonce, operation token,
service generation, peer and descriptor inventory; completed tokens and
requests remain non-replayable across later connections in the same service
generation. Peer-credential and socket failures are normalized to stable
public errors.

Before each observer process, the root cleanup service appends and fsyncs an
`ACTIVE_UNBOUND` revision/anchor/head triple. The root supervisor retains the
blocked leader, places it in the exclusive leaf, captures its pidfd (or
authenticated `/proc` directory handle), PID/start time, PGID/session and cgroup, and
asks the cleanup service to append `ACTIVE_BOUND`. Only complete process and DDS clearance permits an
immutable `CLEARED` successor. Ambiguity appends `QUARANTINED`; restart,
disconnect, or a new service generation cannot clear it. This cleanup chain is
non-consuming and does not reserve the global 0/1 budget. Production recovery
remains unavailable until a separately governed OS-principal recovery record
is provisioned; source tests exercise only the private synthetic recovery seam.
Cleanup inventories the complete authenticated exclusive leaf independently
of the immediate observer's exit, SID, PGID or ancestry and continues the
bounded escalation sequence until process and DDS residual barriers pass. It
refuses to signal an identity whose retained PID/start-time/leaf provenance no
longer matches.

Prepare moved after successful observation finalization. Its lifetime is
exactly 300 seconds and it cannot be extended, renewed, or detached from the
observation session. Expiry rolls back provisional resources and makes every
old receipt unusable while leaving the budget UNCONSUMED. COMMITTED binds the
session/four-source observation, ordered receipt identities, precommit count,
domain/lease, prepare identity, authorization, installed process authority,
peer, and cgroup. Exactly one postcommit graph observation is required; the
maximum transaction count is therefore 100 + 1 = 101. Only its successful
four-source cleanup/reconciliation permits the first campaign child. Any
postcommit failure instead creates `FAILED_AFTER_COMMIT` and permanently
retains 1/1 consumption.

## Effective simulation profile

`config/slice_7g_runtime_params.yaml` and
`ctr_bringup.slice_7g_profile` define a fail-closed effective profile. The
profile selects simulation, `curved_lumen_navigation`, circular-arc geometry,
simulated tactile publication, a positive tactile force cost, tactile-aware
safety supervision, and controller-to-supervisor command routing. The legacy
controller safe-command bypass is rejected whenever the profile is active.

The profile authenticates the approved readiness and acceptance constants. It
also binds `TODO-COST-001`, `TODO-COST-005`, and `TODO-COST-006` to disabled
features with exact zero weights. Both profile validation and `MPPICore`
startup reject a contradictory nonzero advanced-cost weight, so the selected
profile cannot dispatch an unfinished cost implementation.

## Topic and command path

The Slice 7G child launch starts the simulator, evaluator and safety supervisor
with simulated tactile enabled. The MPPI controller publishes proposed commands
only on `/ctr/mppi_command`. The supervisor consumes that topic together with
state and tactile observations and is the only Slice 7G publisher on
`/ctr/safe_command`; the simulator consumes the supervised command. The
evaluator and runtime monitor authenticate `/ctr/tactile/state` and
`/ctr/safety/status`. Recording-window safety faults and invalid tactile samples
enter the retained cell summary.

The governed controller withholds proposed commands until its simulated
tactile sample is eligible. No-contact input preserves the proposed command,
the retained soft-contact warning policy scales it by the configured `0.30`
factor, and every transient fatal class (invalid/stale state, command,
geometry, tactile, safety-bound violation, or tactile STOP) produces a unified
manual-reset latched zero safe command. Restoring valid input alone cannot
resume motion; `/ctr/safety/clear_fault` succeeds only while the current state,
command, geometry, tactile, and timing inputs are all healthy. Startup
status is used for readiness but is not miscounted as a recording-window fault.

Readiness uses one bounded 10-second window and requires ten stable state/tip
samples spanning at least 0.5 seconds, q and tip variation no greater than
`5.0e-5`, fresh simulated tactile data, and fresh fault-free supervisor status.

## Historical v4 coordinator and v5 integration boundary

The pre-v5 `ctr_evaluation.slice_7g_runtime` coordinator remains available to
historical unit tests but its v1 authorization is rejected at the public
runtime boundary. The v5 authority transaction and narrow graph observer are
now represented in source; until the successor installed runtime and bootstrap
exist, the production entrypoint fails closed with stable missing-authority
status and performs no effects. Private synthetic providers are test-only and
cannot be selected through the console API. The historical source-only
coordinator remains for regression inspection and is not runtime authority.

1. Load the immutable charter and a separate canonical runtime authorization.
2. Validate the requested output root as a strict component-safe descendant of
   the charter-authenticated external parent, retain descriptors for that fixed
   parent and the requested output parent, and complete all repository-owned
   non-consuming preflight. A missing runner or invalid/replaced output parent
   creates no registry, lease, output root, control ledger, or attempt revision.
3. Open the single module-owned global lease registry beneath that fixed parent.
   Derive immutable receipts from active `/proc` processes, the ROS graph, DDS
   UDP participant ports, and the external durable lease ledger. Any positive,
   ambiguous, malformed, unavailable, or failed observation is unavailable.
   All valid Slice 7G output descendants therefore contend for the same atomic
   no-replace lease in 100–199. Only after the lease succeeds is the confined
   output root allocated.
4. Generate and validate the exact 15-cell governance plan, initialize durable
   control state, commit domain/output authority, and atomically consume 1/1
   before any process factory can be called.
5. Recheck all four domain sources after attempt consumption, durably retain
   that final observation, and bind the chosen lease to the plan and committed
   process-start event. If the domain is then occupied or uncertain, no process
   starts and the already-consumed attempt remains 1/1. Otherwise, immediately
   invoke exactly one cell process for
   each immutable argv, always with the same
   ledger-bound `ROS_DOMAIN_ID` and campaign root; no retry path exists.
6. The production runner adapter exclusively retains exact stdout and stderr
   bytes (including empty or non-UTF-8 streams), with path, size, and SHA-256 in
   a canonical process-output receipt. A private descriptor-owned output
   authority traverses the complete cell tree iteratively with no-follow
   semantics, rejects aliases and unexpected artifact namespaces, and
   stream-hashes non-semantic members without retaining their complete bytes.
   Only the closed runner receipt, process-output receipt, candidate summary and
   candidate orchestration records are cached. Semantic readiness, safety,
   tactile, metric, timing, and collision values are parsed only from those
   exact authenticated cached bytes—never from a later pathname read.
7. That output authority remains live through result and evidence-role
   construction. Its final barrier iteratively re-enumerates and stream-rehashes
   the tree without building a second byte tree, checks all
   root/directory/member metadata, and component-reopens the public root to
   prove the original inode is still named. No output-tree read follows a
   successful barrier.
8. Write one canonical, sealed nine-file evidence package per cell, create the
   final exclusively locked campaign evidence seal, and delegate final physical
   authentication and aggregate/promotion recomputation to the approved
   governance reconciler.
9. Durably retain the release-provider receipt and immutable acquisition/release
   history. Failure after process-start commitment remains a consumed 1/1
   attempt. If campaign work and release both fail, the original campaign code
   remains primary while ordered cleanup issues are retained in an immutable
   side record; a cleanup-ledger failure is also attached without authorizing a
   retry.

The ledger writer uses same-parent temporary files, complete writes, fsync,
mode `0444`, atomic no-replace publication, and parent-directory fsync. A
failed process-start commit prevents process-factory invocation.

The evidence writer creates only deterministic
`evidence/packages/<cell_id>` packages, finalizes every member to `0444` and
package directory to `0555`, and publishes packages no-replace. The campaign
seal is the final commit, is written while holding the required exclusive
nonblocking lock, and seals the evidence/campaign roots. The approved governance
reader remains the only campaign reconciliation implementation.

## Bounded cell-output authority

The immutable source-owned cell-output policy permits depth `16`, `2,048`
descendants, `67,108,864` bytes in one regular file, `8,388,608` bytes in one
semantic file, `268,435,456` aggregate regular-file bytes and `33,554,432`
cached semantic bytes. Stream hashing requests at most `1,048,576` bytes per
read. Root depth is zero; directories and files both consume the member budget.
The limits are not configurable through CLI, authorization data, callbacks or
environment variables. Their future authority is the independently approved
post-implementation source snapshot.

Every file size and aggregate addition is checked before the corresponding
content read. Non-semantic files, including exact stdout/stderr artifacts, keep
only bounded metadata, size and SHA-256. The final barrier applies the same
depth/member/file/aggregate policy and caches no semantic bytes. Deep trees,
oversized members, aggregate overflow, semantic-cache overflow, traversal
failures and stream failures produce distinct stable Slice 7G error codes.

The output-tree final barrier authenticates an observed barrier instant; it is
not a claim that an external actor cannot violate file modes or mutate the tree
afterward. Exact stream bytes and every semantic artifact remain transitively
bound by the output-inventory identity stored in the mandatory evidence role.

## Metrics and timing

The evaluator writes `slice_7g_runner_result.json` only after a successful
governed run. It binds exact argv, domain, cell, plan, runtime authorization,
attempt revision/process-start event, baseline/candidate directories and exit
status. The production adapter maps these retained bytes and the physically
derived inventory into the approved governance cell result. Promotion-blocking
fields include readiness, sample counts,
steady-state and final goal error, goal hold, both clearance measures,
collisions, safety faults, nonfinite/invalid values, missing topics/results,
saturation and process exit. The 5% deadline target remains diagnostic only;
a timing-only failure sets the mandatory non-real-time limitation without
entering functional failure reasons.

## Snapshot readiness

The source module now emits only
`ctr-slice-7g-post-implementation-source-snapshot-2` proposals. Every closed
member record contains exactly `path`, normalized integer `mode`, `size`, and
`sha256`; the logical algorithm is
`sha256:ctr-slice-7g-post-implementation-source-snapshot-canonical-2`.
Normalized mode is `stat.S_IMODE(st_mode)` in the exact integer range
`0..0o7777`, captured from the same no-follow descriptor used for streamed
size/digest authentication. Device, inode, file type, mode, link count, size,
mtime, ctime, and the directory entry are reconciled before and after hashing
and against the retained first-pass baseline at the final authentication
barrier. One private repository authority opens the absolute component chain
with no-follow semantics and retains every parent descriptor, the repository
root descriptor, independently discovered membership, directory observations,
member observations, and a descriptor-bound change monitor through canonical
construction and the final pathname/inode barrier. This detects mode/content
change-and-restore even on filesystems whose rapid changes can share one
`ctime_ns`, as well as byte-identical root, parent, directory, and member
replacement. A mode-only change therefore changes both canonical bytes and
logical identity, while any intervening mutation invalidates the operation.

Repository monitoring is live before any authoritative parent, root, or source
baseline is accepted. Parent-chain and root descriptors remain provisionally
owned until their watches are installed and pre/post-watch metadata agrees.
Source-tree bootstrap then has four closed phases. First, an iterative,
descriptor-relative/no-follow traversal captures the complete provisional
directory/member set, full directory/member metadata, and streamed file
digests without claiming build authority. Second, it reopens each provisional
directory under the retained root, reconciles it with that record, and installs
the complete source-directory watch set; no directory is enumerated as
authoritative during this installation. Third, after the complete watch set
exactly equals the provisional directory set, setup events are drained and a
second complete inventory rehashes every member and must equal the provisional
inventory. Only then is the first authoritative path/source baseline accepted
and the existing two authoritative member passes begin. Provisional continuity
therefore starts with the first captured facts, complete watch coverage starts
after the last source watch, and authoritative continuity starts only after the
post-watch inventory and setup drain agree. Pre-watch change/restore is caught
by retained metadata/digests; post-watch change is caught by inotify and full
reconciliation. The monitor remains live through both authoritative member
passes and the final path barrier, and treats EOF, queue overflow, watch
invalidation, unknown watches, malformed frames, and bounded-drain exhaustion
as fail-closed source-snapshot errors.

Ancestor path components and the authenticated repository tree use separate
authority records. An ancestor binds device, inode, directory type, normalized
mode, the exact next-component name, and descriptor-relative/no-follow identity
of that next component. Ancestor directory size, link count, mtime, and ctime
are descriptive and do not block acceptance because unrelated sibling activity
may legitimately change them. The repository root, every source-tree
directory, and every source member continue to bind the full metadata and
content baseline. Next-component identity is reconciled after watch
installation, before baseline capture, during public-path reopening, and at the
final barrier. Ancestor inotify handling rejects self/fatal events and exact
bound-component names (checked as both raw filesystem bytes and decoded text),
while ignoring unrelated sibling names; source-tree watches retain their
strict all-mutation policy.

Inotify buffers use a strict 16-byte header parser. Every declared name area
must be 16-byte aligned, fit completely in the current read, contain a NUL
terminator, and contain only zero padding after that terminator. Each returned
buffer must be consumed exactly; a valid frame followed by truncation or
unexplained bytes is rejected rather than accepted when the next nonblocking
read would block. One drain is bounded to 1,048,576 bytes, 8,192 event records,
and 8,192 read attempts.

Descriptor cleanup has explicit local-to-authority ownership transfer. A
directory remains locally owned until provisional metadata, watch installation,
post-watch reconciliation, and setup drain succeed. Cleanup visits the monitor,
all transient directories, and the retained chain in deterministic order even
after an earlier close failure or cleanup `BaseException`. Every returned file
descriptor is registered in a module-owned lifecycle before metadata or watch
work: provisional ownership, transferred authority ownership, close invoked,
closed, or terminal ambiguity. The state changes before the one permitted
`close(2)` attempt. Any exception from that attempt is terminally ambiguous:
the numeric descriptor is quarantined, never inspected with inode equality,
never retried, and never acted on by destruction. This deliberately avoids
closing an independently opened descriptor that reused the number even when it
names the same inode with identical flags and metadata. Cleanup still attempts
all later resources, records immutable issues in resource order, rethrows a
pending `BaseException` only after the loop, and cannot replace a pre-existing
validation error. Terminal ambiguity reports residual resource uncertainty; it
does not claim safe retry or zero possible leak without open-file-description
provenance.

The runtime test module restores the governance evidence-parent global in an
autouse `try/finally` fixture, including after `BaseException`. Runtime and
governance suites therefore execute in either order or one process without
retaining test-selected authority state.

The immutable 30,117-byte v1 artifact remains a parseable historical
predecessor. Because v1 does not bind member modes, the production build-gate
verifier returns `source_snapshot_schema_not_build_authoritative` for it and
never upgrades it using current filesystem modes. V2 has a closed top-level
schema, deeply immutable value records, and independent canonical round-trip
coverage. Structural inspection always reports `build_authoritative=false`.
The production constructor accepts no caller member filter, while the build
verifier independently rediscovers the complete module-owned membership and
requires exact path, mode, size, digest, inode and metadata agreement before it
returns success. A separate structural subset helper exists only for schema and
canonicalization tests; an incomplete subset fails authoritative verification
with `source_snapshot_membership_mismatch`. Repository discovery excludes only
the closed generated-tree, cache, bytecode and top-level `evaluation/`
categories. This source task creates no durable v2 artifact; a clean independent
review must approve the producer and in-memory proposal before a separate
durable-creation authorization.

## Remaining gates

The coordinated source verification baseline selects 706 distinct applicable
unit and static node IDs. V2 adds strict schema/type, normalized-mode,
mid-hash/final-barrier mutation, historical-v1 rejection, immutability, and
independent-canonicalization coverage without weakening the bounded output,
coordination, or governance suites. Those 22 v2 nodes established a 728-node
baseline. The authoritative-membership and repository-lifetime correction adds
20 stage-reaching nodes, establishing a 748-node baseline. Strict monitor
framing, ownership/cleanup, and monitor-before-baseline coverage adds 49 more
nodes, establishing the 797-node starting baseline for this correction. The
descriptor-lifecycle and governance-isolation correction adds 27 current node
IDs while superseding three inode-retry/fallback node IDs, for a net increase
of 24. The descriptor-lifecycle baseline contains 821 distinct applicable node
IDs. The prior displayed digest
`77718564a0c268fb8ae556b9a6a754c0d89a2447322a238d02090034907f9690`
was not reproducible because its production record did not retain the pure
collected node bytes, exact interface-shim setup, exact historical deselections,
and path-normalization procedure. The authoritative 17-path list, full
deselected node IDs, and deterministic shim contract are retained as immutable
test constants in `test_slice_7g_runtime.py`. The procedure installs that
pre-build `ctr_interfaces` shim before
collection, deselects exactly
`test_exact_positive_charter_and_source_snapshot` and
`test_snapshot_descriptors_close_on_success_and_failure`, takes node IDs
directly from `session.items`, normalizes them to repository-relative `/`
paths, rejects duplicates, sorts by Unicode code point, and joins their UTF-8
bytes with one LF and no trailing LF. That baseline serialization is 105,132
bytes with SHA-256
`c94cee33c3098d31949fc284aa34d20c74077e2a3082f57446c8b8097690b38a`.
The ancestor-relevance correction adds 74 distinct stage-reaching nodes. The
final 895-node serialization is 117,896 bytes with SHA-256
`88138fd91803a6cf6d26c90427cf4c5454ac0a0f6979e3c496cf177990a6ad7c`;
the complete-prebaseline-watch bootstrap renames two inaccurate stage labels
without changing their count and adds 22 stage-reaching nodes. The resulting
917-node serialization is 121,038 bytes with SHA-256
`ce8395e8a91806753e26da5cfaa049e001dc01ceb302129b7200a93987711da6`.
The authoritative sequential result has zero duplicate node IDs, skips, or
xfails.
The two historical
authoring-snapshot/current-byte checks still fail, as intended after source
changes, at `snapshot_member_mismatch`; they are not post-implementation
snapshot approval. No build or ROS/runtime test was run.

No build, ROS execution, real domain probe/allocation, real output allocation,
durable campaign ledger, campaign process, real evidence package, promotion
record, or source snapshot was produced by this source task. Unit tests used
temporary roots and lowest-level deterministic OS effects while retaining the
production providers, coordinator, adapter, evidence writer, seal writer and
governance reconciler. The next gate is an independent read-only code review.
Only after that review may a separate authorization create the
post-implementation snapshot and perform the isolated build/test gate.
