"""Repository-owned Slice 7G campaign producer and coordinator.

Nothing in this module runs automatically.  Resource discovery, allocation and
process creation are injected, so importing the module has no filesystem,
process, ROS, DDS, network or environment side effects.
"""

from __future__ import annotations

from dataclasses import dataclass, fields, replace
from datetime import datetime, timezone
import argparse
import ctypes
import errno
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import selectors
import signal
import shutil
import stat
import subprocess
import sys
import time
from typing import Any, Callable, Iterable, Mapping, Sequence
import unicodedata

import ctr_bringup.slice_7g_governance as _slice_7g_governance
from .slice_7g_authority_protocol import (
    AUTHORITY_REQUEST_SCHEMA,
    AUTHORITY_RECEIPT_SCHEMA,
    FOUR_SOURCE_OBSERVATION_SCHEMA,
    MAX_POSTCOMMIT_OBSERVERS,
    MAX_PRECOMMIT_OBSERVERS,
    MAX_TRANSACTION_OBSERVERS,
    OBSERVATION_SESSION_LIFETIME_SECONDS,
    PRECOMMIT_ROS_GRAPH_OBSERVER_ARGV,
    PRECOMMIT_ROS_GRAPH_OBSERVER_CLASS,
    PRECOMMIT_ROS_GRAPH_OBSERVER_EXECUTABLE,
    PREPARE_TOKEN_LIFETIME_SECONDS,
    ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA,
    OBSERVER_CLEANUP_MAXIMUM_WAIT_SECONDS,
    OBSERVER_CLEANUP_MINIMUM_INTERVAL_SECONDS,
    OBSERVER_CLEANUP_STABLE_SAMPLES,
    OBSERVER_STDERR_LIMIT_BYTES,
    OBSERVER_STDOUT_LIMIT_BYTES,
    OBSERVER_TIMEOUT_SECONDS,
    RUNTIME_AUTHORIZATION_SCHEMA as OS_RUNTIME_AUTHORIZATION_SCHEMA,
    Slice7GAuthorityProtocolError,
    Slice7GAuthoritySession,
    authority_record_identity,
    validate_authority_record,
)
from .slice_7g_installed_runtime import OwnedResourceRollback
from ctr_bringup.slice_7g_governance import (
    ATTEMPT_LEDGER_SCHEMA_VERSION,
    CAMPAIGN_EVIDENCE_PACKAGE_RECORD_SCHEMA_VERSION,
    CAMPAIGN_EVIDENCE_SEAL_SCHEMA_VERSION,
    CELL_EVIDENCE_ENVELOPE_SCHEMA_VERSION,
    CELL_EVIDENCE_MEMBER_SCHEMA_VERSION,
    CELL_EVIDENCE_PACKAGE_IDENTITY_DOMAIN,
    CELL_EVIDENCE_PROJECTION_IDENTITY_DOMAIN,
    CELL_EVIDENCE_PROJECTION_SCHEMA_VERSION,
    EVIDENCE_ENVELOPE_PATH,
    EVIDENCE_PROJECTION_PATH,
    MANDATORY_EVIDENCE_ROLE_PATHS,
    Slice7GAttemptLedger,
    Slice7GAttemptEvent,
    Slice7GCampaignCell,
    Slice7GCampaignPlan,
    Slice7GCellResult,
    Slice7GCharter,
    Slice7GGovernanceError,
    authenticate_slice_7g_cell_evidence_package,
    canonical_slice_7g_attempt_event_bytes,
    canonical_slice_7g_attempt_ledger_bytes,
    canonical_slice_7g_campaign_evidence_seal_bytes,
    canonical_slice_7g_cell_result_bytes,
    create_slice_7g_initial_attempt_ledger,
    generate_slice_7g_campaign_plan,
    load_slice_7g_charter,
    propose_slice_7g_attempt_event,
    reconcile_slice_7g_campaign_results,
    slice_7g_attempt_event_identity,
    slice_7g_attempt_ledger_identity,
    slice_7g_campaign_plan_identity,
    slice_7g_campaign_result_identity,
    slice_7g_charter_identity,
    validate_slice_7g_attempt_transition,
    validate_slice_7g_campaign_evidence_seal,
    validate_slice_7g_campaign_plan,
)


LEGACY_RUNTIME_AUTHORIZATION_SCHEMA = "ctr-slice-7g-runtime-authorization-1"
LEGACY_RUNTIME_AUTHORIZATION_DOMAIN = b"ctr-slice-7g-runtime-authorization-canonical-1\0"
# Kept as an import-compatible name, but v1 is never production authority.
RUNTIME_AUTHORIZATION_SCHEMA = LEGACY_RUNTIME_AUTHORIZATION_SCHEMA
RUNTIME_AUTHORIZATION_DOMAIN = LEGACY_RUNTIME_AUTHORIZATION_DOMAIN
DOMAIN_LEASE_SCHEMA = "ctr-slice-7g-domain-lease-1"
DOMAIN_LEASE_DOMAIN = b"ctr-slice-7g-domain-lease-canonical-1\0"
DOMAIN_BINDING_SCHEMA = "ctr-slice-7g-domain-binding-1"
DOMAIN_BINDING_DOMAIN = b"ctr-slice-7g-domain-binding-canonical-1\0"
LEDGER_COMMIT_SCHEMA = "ctr-slice-7g-ledger-commit-1"
POST_IMPLEMENTATION_SNAPSHOT_V1_SCHEMA = "ctr-slice-7g-post-implementation-source-snapshot-1"
POST_IMPLEMENTATION_SNAPSHOT_V1_DOMAIN = b"ctr-slice-7g-post-implementation-source-snapshot-1\0"
POST_IMPLEMENTATION_SNAPSHOT_SCHEMA = "ctr-slice-7g-post-implementation-source-snapshot-2"
POST_IMPLEMENTATION_SNAPSHOT_LOGICAL_ALGORITHM = (
    "sha256:ctr-slice-7g-post-implementation-source-snapshot-canonical-2"
)
POST_IMPLEMENTATION_SNAPSHOT_DOMAIN = (
    b"ctr-slice-7g-post-implementation-source-snapshot-canonical-2\0"
)
DOMAIN_OBSERVATION_SCHEMA = "ctr-slice-7g-domain-observation-1"
DOMAIN_OBSERVATION_DOMAIN = b"ctr-slice-7g-domain-observation-canonical-1\0"
DOMAIN_RESERVATION_SCHEMA = "ctr-slice-7g-domain-reservation-1"
DOMAIN_RESERVATION_DOMAIN = b"ctr-slice-7g-domain-reservation-canonical-1\0"
DOMAIN_COMMITTED_BINDING_SCHEMA = "ctr-slice-7g-domain-committed-binding-1"
DOMAIN_COMMITTED_BINDING_DOMAIN = b"ctr-slice-7g-domain-committed-binding-canonical-1\0"
DOMAIN_RELEASE_SCHEMA = "ctr-slice-7g-domain-release-1"
DOMAIN_RELEASE_DOMAIN = b"ctr-slice-7g-domain-release-canonical-1\0"
RUNNER_RECEIPT_SCHEMA = "ctr-slice-7g-runner-result-receipt-1"
RUNNER_RECEIPT_PATH = "slice_7g_runner_result.json"
PROCESS_OUTPUT_RECEIPT_SCHEMA = "ctr-slice-7g-process-output-receipt-1"
PROCESS_OUTPUT_RECEIPT_PATH = "slice_7g_process_output_receipt.json"
PROCESS_STDOUT_PATH = "process_stdout.bin"
PROCESS_STDERR_PATH = "process_stderr.bin"
FINAL_DOMAIN_OBSERVATION_SCHEMA = "ctr-slice-7g-final-domain-observation-1"
CLEANUP_FAILURE_SCHEMA = "ctr-slice-7g-cleanup-failure-1"
GLOBAL_DOMAIN_LEASE_REGISTRY_NAME = ".ctr_slice_7g_domain_leases"
GLOBAL_DOMAIN_LEASE_LOCK_NAME = "registry.lock"
DOMAIN_MINIMUM = 100
DOMAIN_MAXIMUM = 199
ROS_GRAPH_COMMAND = (
    PRECOMMIT_ROS_GRAPH_OBSERVER_EXECUTABLE,
    *PRECOMMIT_ROS_GRAPH_OBSERVER_ARGV,
)
REQUIRED_RUN_ARTIFACTS = (
    "summary.json", "orchestration.json", "metadata.yaml", "report.md",
    "aligned_samples.csv", "state.csv", "tip.csv", "reference.csv",
    "command.csv", "solve_timing.csv", "horizon.csv", "reference_path.csv",
    "backbone.csv", "lumen_evaluation.csv",
)
SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
DIGEST = re.compile(r"^[0-9a-f]{64}$")
_INITIAL_CELL_OUTPUT_SEMANTIC_PATHS = frozenset({
    RUNNER_RECEIPT_PATH,
    PROCESS_OUTPUT_RECEIPT_PATH,
})


class Slice7GRuntimeError(RuntimeError):
    """Stable source-runtime contract error."""

    def __init__(self, code: str, message: str, *, path: str = "$") -> None:
        self.code = code
        self.path = path
        super().__init__(f"{code}:{path}:{message}")


@dataclass(frozen=True)
class _CellOutputLimits:
    """Immutable source-owned ceilings for one finalized cell output tree."""

    maximum_depth: int
    maximum_members: int
    maximum_file_bytes: int
    maximum_semantic_file_bytes: int
    maximum_total_bytes: int
    maximum_semantic_cache_bytes: int
    stream_hash_chunk_bytes: int

    def __post_init__(self) -> None:
        values = tuple(getattr(self, item.name) for item in fields(self))
        if any(type(value) is not int or value <= 0 for value in values):
            raise Slice7GRuntimeError(
                "cell_output_limits", "cell output limits must be positive exact integers",
            )
        if self.maximum_semantic_file_bytes > self.maximum_file_bytes:
            raise Slice7GRuntimeError(
                "cell_output_limits", "semantic-file limit cannot exceed the regular-file limit",
            )
        if self.maximum_file_bytes > self.maximum_total_bytes:
            raise Slice7GRuntimeError(
                "cell_output_limits", "regular-file limit cannot exceed the aggregate limit",
            )
        if not self.maximum_semantic_file_bytes <= self.maximum_semantic_cache_bytes <= self.maximum_total_bytes:
            raise Slice7GRuntimeError(
                "cell_output_limits", "semantic-cache limit is inconsistent with file and aggregate limits",
            )
        if self.stream_hash_chunk_bytes > self.maximum_semantic_file_bytes:
            raise Slice7GRuntimeError(
                "cell_output_limits", "stream chunk cannot exceed the semantic-file limit",
            )


_CELL_OUTPUT_LIMITS = _CellOutputLimits(
    maximum_depth=16,
    maximum_members=2_048,
    maximum_file_bytes=67_108_864,
    maximum_semantic_file_bytes=8_388_608,
    maximum_total_bytes=268_435_456,
    maximum_semantic_cache_bytes=33_554_432,
    stream_hash_chunk_bytes=1_048_576,
)


@dataclass(frozen=True)
class _CellOutputAccounting:
    """Persistent checked accounting used by initial and final traversals."""

    member_count: int = 0
    total_file_bytes: int = 0
    semantic_cache_bytes: int = 0

    def __post_init__(self) -> None:
        for value in (self.member_count, self.total_file_bytes, self.semantic_cache_bytes):
            if type(value) is not int or value < 0:
                raise Slice7GRuntimeError(
                    "cell_output_limits", "cell output accounting values must be nonnegative exact integers",
                )
        if self.member_count > _CELL_OUTPUT_LIMITS.maximum_members:
            raise Slice7GRuntimeError(
                "cell_output_member_limit", "accounting exceeds the descendant-member ceiling",
            )
        if self.total_file_bytes > _CELL_OUTPUT_LIMITS.maximum_total_bytes:
            raise Slice7GRuntimeError(
                "cell_output_total_size_limit", "accounting exceeds the aggregate byte ceiling",
            )
        if (
            self.semantic_cache_bytes > _CELL_OUTPUT_LIMITS.maximum_semantic_cache_bytes
            or self.semantic_cache_bytes > self.total_file_bytes
        ):
            raise Slice7GRuntimeError(
                "cell_output_semantic_cache_limit", "semantic-cache accounting is inconsistent",
            )

    def add_directory(self, depth: int) -> _CellOutputAccounting:
        self._validate_descendant(depth)
        return _CellOutputAccounting(
            self.member_count + 1, self.total_file_bytes, self.semantic_cache_bytes,
        )

    def add_file(
        self, depth: int, size: int, *, semantic: bool, cache_semantic: bool,
    ) -> _CellOutputAccounting:
        self._validate_descendant(depth)
        if type(semantic) is not bool or type(cache_semantic) is not bool or (cache_semantic and not semantic):
            raise Slice7GRuntimeError(
                "cell_output_limits", "semantic accounting flags must be consistent exact booleans",
            )
        if type(size) is not int or size < 0:
            raise Slice7GRuntimeError(
                "cell_output_file_size_limit", "cell output size must be a nonnegative exact integer",
            )
        if size > _CELL_OUTPUT_LIMITS.maximum_file_bytes:
            raise Slice7GRuntimeError(
                "cell_output_file_size_limit", "cell output member exceeds the regular-file ceiling",
            )
        if semantic and size > _CELL_OUTPUT_LIMITS.maximum_semantic_file_bytes:
            raise Slice7GRuntimeError(
                "cell_output_semantic_size_limit", "semantic output member exceeds the semantic-file ceiling",
            )
        total = self.total_file_bytes + size
        if total > _CELL_OUTPUT_LIMITS.maximum_total_bytes:
            raise Slice7GRuntimeError(
                "cell_output_total_size_limit", "cell output tree exceeds the aggregate byte ceiling",
            )
        cached = self.semantic_cache_bytes
        if semantic and cache_semantic:
            cached += size
            if cached > _CELL_OUTPUT_LIMITS.maximum_semantic_cache_bytes:
                raise Slice7GRuntimeError(
                    "cell_output_semantic_cache_limit", "semantic output cache exceeds its byte ceiling",
                )
        return _CellOutputAccounting(self.member_count + 1, total, cached)

    def add_semantic_cache(self, size: int) -> _CellOutputAccounting:
        if type(size) is not int or size < 0:
            raise Slice7GRuntimeError(
                "cell_output_semantic_size_limit", "semantic output size must be a nonnegative exact integer",
            )
        if size > _CELL_OUTPUT_LIMITS.maximum_semantic_file_bytes:
            raise Slice7GRuntimeError(
                "cell_output_semantic_size_limit", "semantic output member exceeds the semantic-file ceiling",
            )
        cached = self.semantic_cache_bytes + size
        if cached > _CELL_OUTPUT_LIMITS.maximum_semantic_cache_bytes:
            raise Slice7GRuntimeError(
                "cell_output_semantic_cache_limit", "semantic output cache exceeds its byte ceiling",
            )
        return _CellOutputAccounting(self.member_count, self.total_file_bytes, cached)

    def _validate_descendant(self, depth: int) -> None:
        if type(depth) is not int or depth < 1 or depth > _CELL_OUTPUT_LIMITS.maximum_depth:
            raise Slice7GRuntimeError(
                "cell_output_depth_limit", "cell output descendant exceeds the depth ceiling",
            )
        if self.member_count >= _CELL_OUTPUT_LIMITS.maximum_members:
            raise Slice7GRuntimeError(
                "cell_output_member_limit", "cell output tree exceeds the descendant-member ceiling",
            )


@dataclass(frozen=True)
class Slice7GCleanupIssue:
    """One immutable cleanup failure retained beside the primary failure."""

    code: str
    detail: str

    def __post_init__(self) -> None:
        _identifier(self.code, "cleanup_issue_code")
        if type(self.detail) is not str or not self.detail:
            _fail("cleanup_issue", "cleanup issue detail must be a nonempty exact string")


class Slice7GCoordinatedFailure(Slice7GRuntimeError):
    """Stable primary error plus deterministic immutable cleanup accounting."""

    def __init__(self, primary: Exception, cleanup_issues: tuple[Slice7GCleanupIssue, ...]) -> None:
        if type(cleanup_issues) is not tuple or any(type(item) is not Slice7GCleanupIssue for item in cleanup_issues):
            _fail("cleanup_issue", "cleanup issues must be an exact immutable tuple")
        code = primary.code if isinstance(primary, Slice7GRuntimeError) else "campaign_failure"
        path = primary.path if isinstance(primary, Slice7GRuntimeError) else "$"
        self.primary_code = code
        self.primary_detail = str(primary)
        self.cleanup_issues = tuple(cleanup_issues)
        super().__init__(code, self.primary_detail, path=path)


@dataclass(frozen=True)
class Slice7GRuntimeAuthorization:
    schema_version: str
    charter_logical_identity: str
    campaign_id: str
    campaign_identity: str
    post_implementation_source_snapshot_identity: str
    campaign_output_root: str
    issued_at_utc: str
    execution_authorized: bool
    canonical_bytes: bytes
    identity: str


@dataclass(frozen=True)
class Slice7GSourceSnapshotMember:
    """One deeply immutable, mode-bearing v2 source member."""

    path: str
    mode: int
    size: int
    sha256: str

    def __post_init__(self) -> None:
        _source_snapshot_relative(self.path)
        _source_snapshot_mode(self.mode)
        if type(self.size) is not int or self.size < 0:
            _fail(
                "source_snapshot_member_schema",
                "source snapshot member size must be a nonnegative exact integer",
                self.path if type(self.path) is str else "$",
            )
        if type(self.sha256) is not str or not DIGEST.fullmatch(self.sha256):
            _fail(
                "source_snapshot_member_schema",
                "source snapshot member digest must be a lowercase SHA-256",
                self.path if type(self.path) is str else "$",
            )


@dataclass(frozen=True)
class Slice7GPostImplementationSourceSnapshot:
    """Closed structural v2 snapshot value.

    Build authority is established only by complete repository verification,
    never by construction or parsing of this detached value alone.
    """

    schema_version: str
    members: tuple[Slice7GSourceSnapshotMember, ...]

    def __post_init__(self) -> None:
        if type(self.schema_version) is not str or self.schema_version != POST_IMPLEMENTATION_SNAPSHOT_SCHEMA:
            _fail("source_snapshot_schema", "unsupported post-implementation source snapshot schema")
        if type(self.members) is not tuple or not self.members:
            _fail("source_snapshot_member_schema", "source snapshot members must be a nonempty exact tuple")
        if any(type(member) is not Slice7GSourceSnapshotMember for member in self.members):
            _fail("source_snapshot_member_schema", "source snapshot contains an unsupported member record")
        members = tuple(_validated_source_snapshot_member(member) for member in self.members)
        object.__setattr__(self, "members", members)
        paths = tuple(member.path for member in members)
        if paths != tuple(sorted(paths)):
            _fail("source_snapshot_member_schema", "source snapshot members must be ordered by path")
        if len(paths) != len(set(paths)):
            _fail("source_snapshot_member_schema", "source snapshot member paths must be unique")


@dataclass(frozen=True)
class Slice7GSourceSnapshotInspection:
    """Historical or current immutable snapshot inspection result."""

    schema_version: str
    member_count: int
    physical_sha256: str
    logical_identity: str
    build_authoritative: bool

    def __post_init__(self) -> None:
        if type(self.schema_version) is not str or self.schema_version not in {
            POST_IMPLEMENTATION_SNAPSHOT_V1_SCHEMA,
            POST_IMPLEMENTATION_SNAPSHOT_SCHEMA,
        }:
            _fail("source_snapshot_inspection", "snapshot inspection schema is unsupported")
        if type(self.member_count) is not int or self.member_count <= 0:
            _fail("source_snapshot_inspection", "snapshot inspection member count is invalid")
        _digest(self.physical_sha256, "source_snapshot_physical_sha256")
        _digest(self.logical_identity, "source_snapshot_logical_identity")
        if type(self.build_authoritative) is not bool or self.build_authoritative:
            _fail(
                "source_snapshot_inspection",
                "structural snapshot inspection cannot confer build authority",
            )


@dataclass(frozen=True)
class Slice7GDomainLease:
    schema_version: str
    charter_logical_identity: str
    runtime_authorization_identity: str
    campaign_identity: str
    domain_id: int
    occupancy_checked: bool
    collision_free: bool
    provider_receipt_identity: str
    leased_at_utc: str
    identity: str


@dataclass(frozen=True)
class Slice7GDomainOccupancy:
    domain_id: int
    active_processes_clear: bool
    ros_graph_clear: bool
    dds_participants_clear: bool
    external_ledger_clear: bool
    checked_at_utc: str
    receipt_identity: str

    def __post_init__(self) -> None:
        if type(self.domain_id) is not int or not 100 <= self.domain_id <= 199:
            _fail("domain_occupancy_record", "occupancy domain must be in 100..199")
        for field in (
            "active_processes_clear", "ros_graph_clear", "dds_participants_clear", "external_ledger_clear",
        ):
            if type(getattr(self, field)) is not bool:
                _fail("domain_occupancy_record", f"{field} must be an exact bool")
        _utc(self.checked_at_utc, "occupancy_checked_at_utc")
        _digest(self.receipt_identity, "occupancy_receipt_identity")

    @property
    def collision_free(self) -> bool:
        return all((
            self.active_processes_clear,
            self.ros_graph_clear,
            self.dds_participants_clear,
            self.external_ledger_clear,
        ))


@dataclass(frozen=True)
class Slice7GDomainRelease:
    lease_identity: str
    provider_receipt_identity: str
    released_at_utc: str


@dataclass(frozen=True)
class Slice7GDomainBinding:
    schema_version: str
    lease_identity: str
    runtime_authorization_identity: str
    campaign_identity: str
    attempt_ledger_identity: str
    attempt_ledger_revision: int
    domain_id: int
    output_root: str
    identity: str


@dataclass(frozen=True)
class Slice7GReadinessResult:
    passed: bool
    failure_code: str
    stable_sample_count: int
    stable_interval_seconds: float
    q_variation: float
    tip_variation_m: float
    tactile_age_seconds: float
    safety_age_seconds: float


@dataclass(frozen=True)
class Slice7GCellExecution:
    cell_result: Slice7GCellResult
    invocation_payload: dict[str, Any]
    readiness_payload: dict[str, Any]
    safety_payload: dict[str, Any]
    tactile_payload: dict[str, Any]
    output_inventory_payload: dict[str, Any]


@dataclass(frozen=True)
class Slice7GProcessObservation:
    """Detached result of the one production cell subprocess."""

    argv: tuple[str, ...]
    returncode: int
    stdout: bytes
    stderr: bytes


@dataclass(frozen=True)
class Slice7GROSGraphObserverContract:
    executable: str
    executable_identity: str
    interpreter: str
    interpreter_identity: str
    module_origin_identities: tuple[str, ...]
    argv: tuple[str, ...]
    environment: tuple[tuple[str, str], ...]
    environment_identity: str
    working_directory: str
    cgroup: str
    rmw_implementation: str


@dataclass(frozen=True)
class Slice7GROSGraphObserverExecution:
    pid: int
    process_group_id: int
    process_start_time_ticks: int
    started_monotonic_ns: int
    ended_monotonic_ns: int
    returncode: int
    stdout: bytes
    stderr: bytes


@dataclass(frozen=True)
class Slice7GObservationResult:
    domain_id: int
    observation_session_identity: str
    observation_session_nonce: str
    four_source_observation: dict[str, Any]
    four_source_observation_identity: str
    precommit_receipts: tuple[dict[str, Any], ...]
    precommit_receipt_identities: tuple[str, ...]
    lease_identity: str


@dataclass(frozen=True)
class Slice7GPostcommitObservationResult:
    ros_graph_observation_receipt: dict[str, Any]
    ros_graph_observation_identity: str
    four_source_observation: dict[str, Any]
    four_source_observation_identity: str


@dataclass(frozen=True)
class Slice7GDomainObservationReceipt:
    schema_version: str
    source: str
    domain_id: int
    clear: bool
    observed_at_utc: str
    observation_sha256: str
    identity: str


@dataclass(frozen=True)
class _CellOutputMember:
    path: str
    metadata: tuple[int, int, int, int, int, int, int]
    size: int
    sha256: str
    semantic_bytes: bytes | None


@dataclass(frozen=True)
class _CellOutputDirectory:
    path: str
    metadata: tuple[int, int, int, int, int, int, int]


@dataclass(frozen=True)
class _CellOutputTraversalFrame:
    path: str
    depth: int
    descriptor: int
    names: tuple[str, ...]
    next_index: int
    baseline_metadata: tuple[int, int, int, int, int, int, int]
    owns_descriptor: bool


def load_slice_7g_runtime_authorization(
    path: str | os.PathLike[str], charter: Slice7GCharter,
) -> Slice7GRuntimeAuthorization:
    _load_slice_7g_runtime_authorization_v1_for_test(path, charter)
    _fail(
        "runtime_authorization_v1_historical_only",
        "runtime-authorization-v1 is parseable historical data but is not runtime authority",
    )


def _load_slice_7g_runtime_authorization_v1_for_test(
    path: str | os.PathLike[str], charter: Slice7GCharter,
) -> Slice7GRuntimeAuthorization:
    """Parse the immutable predecessor format for regression tests only."""

    source = _path(path, "runtime_authorization_path")
    try:
        raw = _read_sealed_file_nofollow(source)
    except Slice7GRuntimeError:
        raise
    except OSError as exc:
        raise Slice7GRuntimeError("runtime_authorization_open", str(exc), path=source) from exc
    data = _parse_json(raw, "runtime_authorization_json")
    required = {
        "schema_version", "charter_logical_identity", "campaign_id", "campaign_identity",
        "post_implementation_source_snapshot_identity", "campaign_output_root", "issued_at_utc",
        "execution_authorized",
    }
    _closed(data, required, "runtime_authorization_fields")
    if data["schema_version"] != RUNTIME_AUTHORIZATION_SCHEMA:
        _fail("runtime_authorization_schema", "unsupported runtime authorization schema")
    charter_identity = slice_7g_charter_identity(charter)
    if data["charter_logical_identity"] != charter_identity:
        _fail("runtime_authorization_charter", "authorization does not bind the approved charter")
    campaign_id = _identifier(data["campaign_id"], "campaign_id")
    initial = create_slice_7g_initial_attempt_ledger(charter, campaign_id)
    if data["campaign_identity"] != initial.campaign_identity:
        _fail("runtime_authorization_campaign", "campaign identity does not derive from charter and campaign ID")
    _digest(data["post_implementation_source_snapshot_identity"], "source_snapshot_identity")
    output_root = _absolute_path_text(data["campaign_output_root"], "campaign_output_root")
    trusted_parent = _trusted_external_output_parent(charter)
    _require_strict_descendant(output_root, trusted_parent, "runtime_authorization_output_root")
    _utc(data["issued_at_utc"], "issued_at_utc")
    if data["execution_authorized"] is not True:
        _fail("runtime_not_authorized", "authorization must explicitly authorize execution")
    canonical = _canonical(data)
    if raw != canonical:
        _fail("runtime_authorization_noncanonical", "authorization bytes are not canonical")
    identity = hashlib.sha256(RUNTIME_AUTHORIZATION_DOMAIN + canonical).hexdigest()
    return Slice7GRuntimeAuthorization(
        data["schema_version"], data["charter_logical_identity"], campaign_id,
        data["campaign_identity"], data["post_implementation_source_snapshot_identity"],
        output_root, data["issued_at_utc"], True, canonical, identity,
    )


def canonical_runtime_authorization_bytes(data: dict[str, Any]) -> bytes:
    """Test/producer helper; validation remains in the path loader."""

    return _canonical(_detach(data))


class Slice7GProductionEffects:
    """Lowest-level operating-system effects used by the production assembly.

    Tests may replace this object, but production callers cannot supply domain
    decisions or authority records.  All authority records are derived by the
    repository-owned providers below from these raw observations.
    """

    def utc_now(self) -> str:
        return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

    def monotonic(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)

    def which(self, executable: str) -> str | None:
        return shutil.which(executable)

    def run_observer(
        self, argv: tuple[str, ...], env: dict[str, str], timeout_seconds: float,
    ) -> Slice7GProcessObservation:
        del argv, env, timeout_seconds
        _fail(
            "legacy_observer_prohibited",
            "bare observer execution is not charter-v5 runtime authority",
        )

    def graph_observer_contract(self, domain_id: int) -> Slice7GROSGraphObserverContract:
        del domain_id
        _fail(
            "observer_manifest_required",
            "installed-runtime graph-observer process authority is not provisioned",
        )

    def run_graph_observer(
        self, contract: Slice7GROSGraphObserverContract,
    ) -> Slice7GROSGraphObserverExecution:
        contract = _validated_graph_observer_contract(contract)
        environment = {key: value for key, value in contract.environment}
        started = time.monotonic_ns()
        try:
            process = subprocess.Popen(
                contract.argv,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=contract.working_directory,
                env=environment,
                shell=False,
                close_fds=True,
                start_new_session=True,
            )
        except OSError as exc:
            raise Slice7GRuntimeError("observer_process_failed", str(exc)) from exc
        try:
            start_ticks = _process_start_time_ticks(process.pid)
            process_group = os.getpgid(process.pid)
            if process_group != process.pid:
                _fail("observer_process_group", "observer does not own its tracked process group")
            if self.process_cgroup(process.pid) != contract.cgroup:
                _fail("observer_cgroup", "observer escaped the authenticated campaign cgroup")
            stdout, stderr = _bounded_process_streams(
                process,
                timeout_seconds=OBSERVER_TIMEOUT_SECONDS,
                stdout_limit=OBSERVER_STDOUT_LIMIT_BYTES,
                stderr_limit=OBSERVER_STDERR_LIMIT_BYTES,
                process_guard=lambda: self.guard_graph_observer_process(process.pid, contract.cgroup),
            )
            returncode = process.wait(timeout=0)
            ended = time.monotonic_ns()
            return Slice7GROSGraphObserverExecution(
                process.pid, process_group, start_ticks, started, ended,
                int(returncode), stdout, stderr,
            )
        except BaseException as primary:
            cleanup_errors: list[BaseException] = []
            group = process_group
            for sent_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                try:
                    members = tuple(
                        record for record in self.active_process_records()
                        if record.get("process_group_id") == group
                    )
                    if not members:
                        break
                    if any(
                        record.get("cgroup") != contract.cgroup
                        or type(record.get("process_start_time_ticks")) is not int
                        or record["process_start_time_ticks"] < start_ticks
                        for record in members
                    ):
                        _fail(
                            "observer_process_ownership",
                            "observer PGID was reused or contains an unauthenticated process",
                        )
                    os.killpg(group, sent_signal)
                    deadline = time.monotonic() + 1.0
                    while time.monotonic() < deadline:
                        remaining = tuple(
                            record for record in self.active_process_records()
                            if record.get("process_group_id") == group
                        )
                        if not remaining:
                            break
                        if any(
                            record.get("cgroup") != contract.cgroup
                            or type(record.get("process_start_time_ticks")) is not int
                            or record["process_start_time_ticks"] < start_ticks
                            for record in remaining
                        ):
                            _fail(
                                "observer_process_ownership",
                                "observer PGID ownership changed during cleanup",
                            )
                        time.sleep(0.02)
                except ProcessLookupError:
                    continue
                except BaseException as exc:
                    cleanup_errors.append(exc)
                    break
            for stream in (process.stdout, process.stderr):
                if stream is not None:
                    try:
                        stream.close()
                    except BaseException as exc:
                        cleanup_errors.append(exc)
            if cleanup_errors:
                try:
                    primary.add_note(
                        "Slice 7G observer cleanup issues: "
                        + repr(tuple(type(item).__name__ for item in cleanup_errors))
                    )
                except (AttributeError, TypeError):
                    pass
            raise

    def process_cgroup(self, pid: int) -> str:
        try:
            lines = Path(f"/proc/{pid}/cgroup").read_text(encoding="ascii").splitlines()
        except (OSError, UnicodeError) as exc:
            raise Slice7GRuntimeError("observer_cgroup", "cannot authenticate observer cgroup") from exc
        unified = [line.split(":", 2)[2] for line in lines if line.startswith("0::")]
        if len(unified) != 1 or not unified[0].startswith("/"):
            _fail("observer_cgroup", "observer cgroup observation is ambiguous")
        return unified[0]

    def guard_graph_observer_process(self, pid: int, cgroup: str) -> None:
        records = self.active_process_records()
        descendants = _descendant_process_records(pid, records)
        if descendants:
            _fail("observer_unexpected_descendant", "graph observer created an unexpected descendant")
        if any(
            record.get("cgroup") == cgroup
            and b"ros2 daemon" in record.get("command", b"").lower()
            for record in records
        ):
            _fail("observer_ros_daemon", "graph observer started or contacted a ROS daemon process")

    def observer_cleanup_sample(
        self,
        execution: Slice7GROSGraphObserverExecution,
        domain_id: int,
    ) -> dict[str, Any]:
        process_present = Path(f"/proc/{execution.pid}").exists()
        try:
            os.killpg(execution.process_group_id, 0)
        except ProcessLookupError:
            process_group_present = False
        except PermissionError:
            process_group_present = True
        else:
            process_group_present = True
        records = self.active_process_records()
        descendants = tuple(
            record["pid"] for record in _descendant_process_records(execution.pid, records)
        )
        daemon_pids = tuple(
            record["pid"] for record in records
            if record.get("cgroup") == "/system.slice/ctr-slice7g-campaign.service"
            and b"ros2 daemon" in record.get("command", b"").lower()
        )
        ports = _udp_ports_from_proc_tables(self.udp_socket_tables())
        base = 7400 + 250 * domain_id
        matching_ports = tuple(sorted(port for port in ports if base <= port < base + 250))
        return {
            "process_present": process_present,
            "process_group_present": process_group_present,
            "descendant_pids": descendants,
            "ros_daemon_pids": daemon_pids,
            "matching_udp_ports": matching_ports,
        }

    def run_cell(
        self, argv: tuple[str, ...], env: dict[str, str], timeout_seconds: float,
    ) -> Slice7GProcessObservation:
        environment = dict(os.environ)
        environment.update(env)
        try:
            completed = subprocess.run(
                argv, env=environment, check=False, capture_output=True, timeout=timeout_seconds,
            )
        except subprocess.TimeoutExpired as exc:
            raise Slice7GRuntimeError("cell_process_timeout", "cell process exceeded its bounded timeout") from exc
        except (OSError, subprocess.SubprocessError) as exc:
            raise Slice7GRuntimeError("cell_process_failed", str(exc)) from exc
        return Slice7GProcessObservation(
            tuple(argv), int(completed.returncode), bytes(completed.stdout), bytes(completed.stderr),
        )

    def active_process_records(self) -> tuple[dict[str, Any], ...]:
        records: list[dict[str, Any]] = []
        try:
            entries = tuple(Path("/proc").iterdir())
        except OSError as exc:
            raise Slice7GRuntimeError("active_process_observation", str(exc)) from exc
        for entry in entries:
            if not entry.name.isascii() or not entry.name.isdigit():
                continue
            try:
                command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").strip()
                stat_fields = (entry / "stat").read_text(encoding="ascii").rsplit(") ", 1)[1].split()
                parent_pid = int(stat_fields[1])
                process_group_id = int(stat_fields[2])
                process_start_time_ticks = int(stat_fields[19])
            except (FileNotFoundError, ProcessLookupError):
                continue
            except OSError as exc:
                records.append({
                    "pid": int(entry.name), "parent_pid": None, "cgroup": None,
                    "process_group_id": None, "process_start_time_ticks": None,
                    "command": b"", "environment": None, "error": type(exc).__name__,
                })
                continue
            if not command:
                continue
            try:
                environment = (entry / "environ").read_bytes()
                error = None
            except (FileNotFoundError, ProcessLookupError):
                continue
            except OSError as exc:
                environment = None
                error = type(exc).__name__
            try:
                cgroup = self.process_cgroup(int(entry.name))
            except Slice7GRuntimeError:
                cgroup = None
            records.append({
                "pid": int(entry.name), "parent_pid": parent_pid,
                "process_group_id": process_group_id,
                "process_start_time_ticks": process_start_time_ticks,
                "cgroup": cgroup, "command": command,
                "environment": environment, "error": error,
            })
        return tuple(records)

    def udp_socket_tables(self) -> tuple[bytes, bytes]:
        try:
            return Path("/proc/net/udp").read_bytes(), Path("/proc/net/udp6").read_bytes()
        except OSError as exc:
            raise Slice7GRuntimeError("dds_socket_observation", str(exc)) from exc


class _ProductionRootAuthority:
    """Descriptor-confined authority for the charter parent, output and lease registry."""

    def __init__(self, charter: Slice7GCharter, authorization: Slice7GRuntimeAuthorization) -> None:
        authorization = _validated_runtime_authorization(authorization, charter)
        self.external_parent = _trusted_external_output_parent(charter)
        self.output_root = authorization.campaign_output_root
        relative = _require_strict_descendant(
            self.output_root, self.external_parent, "runtime_authorization_output_root",
        )
        components = relative.split("/")
        if components[0] == GLOBAL_DOMAIN_LEASE_REGISTRY_NAME:
            _fail("runtime_authorization_output_root", "campaign output cannot occupy the global lease registry")
        self._external_descriptor = _open_directory_path_nofollow(
            self.external_parent, "campaign_external_parent",
        )
        self._closed = False
        self._external_metadata = _stable_metadata(os.fstat(self._external_descriptor))
        if not stat.S_ISDIR(self._external_metadata[2]):
            self.close()
            _fail("campaign_external_parent", "external parent must be a real directory")
        current = os.dup(self._external_descriptor)
        self._output_parent_components = tuple(components[:-1])
        opened = [current]
        initialized = False
        try:
            for component in components[:-1]:
                child = _open_private_directory_at(current, component, "campaign_output_parent")
                opened.append(child)
                current = child
            self._output_parent_descriptor = os.dup(current)
            self._output_parent_metadata = _stable_metadata(os.fstat(current))
            self._output_leaf = components[-1]
            initialized = True
        finally:
            for descriptor in reversed(opened):
                os.close(descriptor)
            if not initialized:
                self.close()

    def preflight(self) -> None:
        self._require_open()
        self._recheck_parent_descriptors()
        try:
            os.stat(self._output_leaf, dir_fd=self._output_parent_descriptor, follow_symlinks=False)
        except FileNotFoundError:
            return
        except OSError as exc:
            raise Slice7GRuntimeError(
                "preflight_output_stat", str(exc), path=self.output_root,
            ) from exc
        _fail("preflight_output_exists", "authorized campaign output root must not exist", self.output_root)

    def prepare_global_registry(self) -> tuple[int, int, int, int, int, int, int]:
        self._require_open()
        self._recheck_parent_descriptors()
        created = False
        try:
            os.mkdir(GLOBAL_DOMAIN_LEASE_REGISTRY_NAME, 0o700, dir_fd=self._external_descriptor)
            os.fsync(self._external_descriptor)
            created = True
        except FileExistsError:
            pass
        except OSError as exc:
            raise Slice7GRuntimeError("domain_registry_create", str(exc)) from exc
        descriptor = _open_private_directory_at(
            self._external_descriptor, GLOBAL_DOMAIN_LEASE_REGISTRY_NAME, "domain_registry",
        )
        try:
            info = os.fstat(descriptor)
            if stat.S_IMODE(info.st_mode) != 0o700:
                _fail("domain_registry_mode", "domain registry must use mode 0700")
            if created:
                lock = os.open(
                    GLOBAL_DOMAIN_LEASE_LOCK_NAME,
                    os.O_CREAT | os.O_EXCL | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW,
                    0o600, dir_fd=descriptor,
                )
                try:
                    os.fsync(lock)
                finally:
                    os.close(lock)
                os.fsync(descriptor)
            authenticated_lock = _authenticate_domain_registry_lock(descriptor)
            os.close(authenticated_lock)
            return _stable_metadata(info)
        finally:
            os.close(descriptor)

    def allocate_output_root(self, authorization: Slice7GRuntimeAuthorization) -> str:
        authorization = _validated_runtime_authorization(authorization)
        if authorization.campaign_output_root != self.output_root:
            _fail("output_allocation_binding", "output authorization differs from root authority")
        self.preflight()
        descriptor: int | None = None
        try:
            os.mkdir(self._output_leaf, 0o700, dir_fd=self._output_parent_descriptor)
            descriptor = _open_private_directory_at(
                self._output_parent_descriptor, self._output_leaf, "campaign_output_root",
            )
            if os.listdir(descriptor):
                _fail("output_root_not_empty", "new campaign output root is not empty", self.output_root)
            os.fsync(descriptor)
            os.fsync(self._output_parent_descriptor)
            return self.output_root
        except FileExistsError as exc:
            raise Slice7GRuntimeError(
                "output_root_exists", "campaign output root must be new", path=self.output_root,
            ) from exc
        except Slice7GRuntimeError:
            raise
        except OSError as exc:
            raise Slice7GRuntimeError("output_root_create", str(exc), path=self.output_root) from exc
        finally:
            if descriptor is not None:
                os.close(descriptor)

    def close(self) -> None:
        if getattr(self, "_closed", True):
            return
        self._closed = True
        for name in ("_output_parent_descriptor", "_external_descriptor"):
            descriptor = getattr(self, name, None)
            if descriptor is not None:
                try:
                    os.close(descriptor)
                finally:
                    setattr(self, name, None)

    def _require_open(self) -> None:
        if getattr(self, "_closed", True):
            _fail("root_authority_closed", "production root authority is closed")

    def _recheck_parent_descriptors(self) -> None:
        if _directory_identity(_stable_metadata(os.fstat(self._external_descriptor))) != _directory_identity(self._external_metadata):
            _fail("campaign_external_parent_changed", "external parent descriptor changed")
        if _directory_identity(_stable_metadata(os.fstat(self._output_parent_descriptor))) != _directory_identity(self._output_parent_metadata):
            _fail("campaign_output_parent_changed", "output parent descriptor changed")
        reopened = _open_directory_path_nofollow(self.external_parent, "campaign_external_parent")
        try:
            if _stable_metadata(os.fstat(reopened))[:2] != self._external_metadata[:2]:
                _fail("campaign_external_parent_changed", "external parent pathname was replaced")
            current = reopened
            opened: list[int] = []
            try:
                for component in self._output_parent_components:
                    child = _open_private_directory_at(current, component, "campaign_output_parent")
                    opened.append(child)
                    current = child
                if _stable_metadata(os.fstat(current))[:2] != self._output_parent_metadata[:2]:
                    _fail("campaign_output_parent_changed", "output parent pathname was replaced")
            finally:
                for descriptor in reversed(opened):
                    os.close(descriptor)
        finally:
            os.close(reopened)


def _registry_lock(exclusive: bool):
    def decorate(method):
        def locked(self, *args, **kwargs):
            registry = self._open_registry()
            lock = None
            try:
                lock = _authenticate_domain_registry_lock(registry)
                fcntl.flock(lock, fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH)
                return method(self, *args, **kwargs)
            finally:
                if lock is not None:
                    try:
                        fcntl.flock(lock, fcntl.LOCK_UN)
                    finally:
                        os.close(lock)
                os.close(registry)
        return locked
    return decorate


class ProductionSlice7GDomainAuthority:
    """Four-source observer plus atomic durable domain lease provider."""

    def __init__(
        self,
        lease_root: str | os.PathLike[str],
        effects: Slice7GProductionEffects,
        *,
        defer_prepare: bool = False,
    ) -> None:
        if type(effects) is not Slice7GProductionEffects and not isinstance(effects, Slice7GProductionEffects):
            _fail("production_effects", "production effects must use the repository-owned interface")
        if type(defer_prepare) is not bool:
            _fail("domain_registry", "defer_prepare must be an exact bool")
        self.effects = effects
        self.lease_root = Path(_absolute_path_text(_path(lease_root, "domain_lease_root"), "domain_lease_root"))
        self._registry_metadata: tuple[int, int, int, int, int, int, int] | None = None
        self._reservations: dict[int, str] = {}
        self._last_observations: dict[int, Slice7GDomainOccupancy] = {}
        if not defer_prepare:
            _ensure_private_directory(self.lease_root)
            descriptor = _open_directory_path_nofollow(str(self.lease_root), "domain_registry")
            try:
                try:
                    lock = os.open(
                        GLOBAL_DOMAIN_LEASE_LOCK_NAME,
                        os.O_CREAT | os.O_EXCL | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW,
                        0o600, dir_fd=descriptor,
                    )
                except FileExistsError:
                    lock = _authenticate_domain_registry_lock(descriptor)
                try:
                    os.fsync(lock)
                finally:
                    os.close(lock)
                os.fsync(descriptor)
                info = os.fstat(descriptor)
                if stat.S_IMODE(info.st_mode) != 0o700:
                    _fail("domain_registry_mode", "domain registry must use mode 0700")
                self._registry_metadata = _stable_metadata(info)
            finally:
                os.close(descriptor)

    def adopt_prepared_registry(
        self, metadata: tuple[int, int, int, int, int, int, int],
    ) -> None:
        if type(metadata) is not tuple or len(metadata) != 7 or any(type(item) is not int for item in metadata):
            _fail("domain_registry", "prepared registry metadata is malformed")
        if self._registry_metadata is not None:
            _fail("domain_registry", "domain registry was already prepared")
        descriptor = _open_directory_path_nofollow(str(self.lease_root), "domain_registry")
        try:
            observed = _stable_metadata(os.fstat(descriptor))
            if _directory_identity(observed) != _directory_identity(metadata) or stat.S_IMODE(observed[2]) != 0o700:
                _fail("domain_registry_changed", "prepared domain registry identity changed")
            self._registry_metadata = observed
        finally:
            os.close(descriptor)

    def _open_registry(self) -> int:
        if self._registry_metadata is None:
            _fail("domain_registry_unprepared", "domain registry must be prepared after preflight")
        descriptor = _open_directory_path_nofollow(str(self.lease_root), "domain_registry")
        observed = _stable_metadata(os.fstat(descriptor))
        if _directory_identity(observed) != _directory_identity(self._registry_metadata) or stat.S_IMODE(observed[2]) != 0o700:
            os.close(descriptor)
            _fail("domain_registry_changed", "domain registry pathname no longer names the authenticated directory")
        return descriptor

    @_registry_lock(False)
    def observe(self, domain: int) -> Slice7GDomainOccupancy:
        if type(domain) is not int or not DOMAIN_MINIMUM <= domain <= DOMAIN_MAXIMUM:
            _fail("domain_occupancy_record", "domain must be in 100..199")
        active = self._observe_active_processes(domain)
        dds = self._observe_dds_participants(domain)
        lease = self._observe_external_lease(domain)
        if active.clear and dds.clear and lease.clear:
            graph = self._observe_ros_graph(domain)
        else:
            graph = _make_domain_observation(
                "ros_graph", domain, False,
                {"invoked": False, "reason": "candidate_occupied_by_non_ros_source"},
                _production_timestamp(self.effects, "ros_graph_timestamp"),
            )
        receipts = (active, graph, dds, lease)
        observed_at = _production_timestamp(self.effects, "domain_observation_timestamp")
        _utc(observed_at, "occupancy_checked_at_utc")
        projection = {
            "schema_version": DOMAIN_OBSERVATION_SCHEMA,
            "domain_id": domain,
            "observed_at_utc": observed_at,
            "receipts": [_domain_observation_data(item) for item in receipts],
        }
        identity = hashlib.sha256(DOMAIN_OBSERVATION_DOMAIN + _canonical(projection)).hexdigest()
        result = Slice7GDomainOccupancy(
            domain,
            receipts[0].clear,
            receipts[1].clear,
            receipts[2].clear,
            receipts[3].clear,
            observed_at,
            identity,
        )
        self._last_observations[domain] = result
        return result

    @_registry_lock(True)
    def acquire(self, domain: int, authorization_identity: str, campaign_identity: str) -> str | None:
        _digest(authorization_identity, "runtime_authorization_identity")
        _digest(campaign_identity, "campaign_identity")
        if type(domain) is not int or not DOMAIN_MINIMUM <= domain <= DOMAIN_MAXIMUM:
            _fail("domain_reservation", "domain must be in 100..199")
        registry = self._open_registry()
        domain_descriptor: int | None = None
        payload = {
            "schema_version": DOMAIN_RESERVATION_SCHEMA,
            "domain_id": domain,
            "runtime_authorization_identity": authorization_identity,
            "campaign_identity": campaign_identity,
            "reserved_at_utc": _production_timestamp(self.effects, "domain_reservation_timestamp"),
        }
        _utc(payload["reserved_at_utc"], "reserved_at_utc")
        identity = hashlib.sha256(DOMAIN_RESERVATION_DOMAIN + _canonical(payload)).hexdigest()
        try:
            domain_descriptor = _open_or_create_private_directory_at(
                registry, f"domain_{domain:03d}", "domain_lease_domain_root",
            )
            _commit_noreplace_file_at(
                domain_descriptor, "active.json", _canonical({**payload, "identity": identity}),
                "domain_lease_conflict",
            )
        except Slice7GRuntimeError as exc:
            if exc.code == "domain_lease_conflict":
                return None
            raise
        finally:
            if domain_descriptor is not None:
                os.close(domain_descriptor)
            os.close(registry)
        self._reservations[domain] = identity
        return identity

    @_registry_lock(True)
    def commit_process_binding(
        self,
        lease: Slice7GDomainLease,
        charter: Slice7GCharter,
        ledger: Slice7GAttemptLedger,
        plan: Slice7GCampaignPlan,
        final_observation: Slice7GDomainOccupancy,
    ) -> str:
        lease = _validated_domain_lease(lease)
        validate_slice_7g_campaign_plan(plan, charter, ledger)
        if not ledger.process_start_committed or ledger.consumed_campaign_attempts != 1:
            _fail("domain_committed_binding", "domain binding requires the committed 1/1 process-start ledger")
        retained = self._reservations.get(lease.domain_id)
        if retained is None or retained != lease.provider_receipt_identity:
            _fail("domain_committed_binding", "domain reservation is not owned by this production provider")
        observed = self._last_observations.get(lease.domain_id)
        if type(final_observation) is not Slice7GDomainOccupancy or observed != final_observation:
            _fail("domain_committed_binding", "final domain observation is not provider-authenticated")
        if not final_observation.collision_free:
            _fail("domain_committed_binding", "final domain observation is not collision-free")
        payload = {
            "schema_version": DOMAIN_COMMITTED_BINDING_SCHEMA,
            "domain_lease_identity": lease.identity,
            "domain_reservation_identity": lease.provider_receipt_identity,
            "final_domain_observation_identity": final_observation.receipt_identity,
            "runtime_authorization_identity": ledger.runtime_authorization_identity,
            "campaign_identity": ledger.campaign_identity,
            "campaign_plan_identity": slice_7g_campaign_plan_identity(plan),
            "attempt_ledger_identity": slice_7g_attempt_ledger_identity(ledger),
            "attempt_ledger_revision": ledger.revision,
            "process_start_event_identity": ledger.last_event_identity,
            "domain_id": ledger.domain_id,
            "output_root": ledger.output_root,
        }
        identity = hashlib.sha256(DOMAIN_COMMITTED_BINDING_DOMAIN + _canonical(payload)).hexdigest()
        registry = self._open_registry()
        domain_descriptor: int | None = None
        try:
            domain_descriptor = _open_private_directory_at(
                registry, f"domain_{lease.domain_id:03d}", "domain_lease_domain_root",
            )
            reservation = _parse_json(
                _read_sealed_file_at(domain_descriptor, "active.json", "domain_reservation_record"),
                "domain_reservation_record",
            )
            if reservation.get("identity") != retained:
                _fail("domain_committed_binding", "active reservation identity changed")
            _commit_noreplace_file_at(
                domain_descriptor, f"binding.{retained}.json",
                _canonical({**payload, "identity": identity}), "domain_binding_conflict",
            )
        finally:
            if domain_descriptor is not None:
                os.close(domain_descriptor)
            os.close(registry)
        return identity

    @_registry_lock(True)
    def release(self, domain: int, lease_identity: str) -> str | None:
        _digest(lease_identity, "domain_lease_identity")
        retained = self._reservations.get(domain)
        if retained is None:
            _fail("domain_release_unowned", "production provider does not own the requested reservation")
        payload = {
            "schema_version": DOMAIN_RELEASE_SCHEMA,
            "domain_id": domain,
            "domain_lease_identity": lease_identity,
            "domain_reservation_identity": retained,
            "released_at_utc": _production_timestamp(self.effects, "domain_release_timestamp"),
        }
        _utc(payload["released_at_utc"], "released_at_utc")
        identity = hashlib.sha256(DOMAIN_RELEASE_DOMAIN + _canonical(payload)).hexdigest()
        registry = self._open_registry()
        domain_descriptor: int | None = None
        try:
            domain_descriptor = _open_private_directory_at(
                registry, f"domain_{domain:03d}", "domain_lease_domain_root",
            )
            active = _parse_json(
                _read_sealed_file_at(domain_descriptor, "active.json", "domain_reservation_record"),
                "domain_reservation_record",
            )
            if active.get("identity") != retained:
                _fail("domain_release_unowned", "active reservation differs from provider ownership")
            _commit_noreplace_file_at(
                domain_descriptor, f"release.{retained}.json",
                _canonical({**payload, "identity": identity}), "domain_release_conflict",
            )
            _rename_noreplace_at(
                domain_descriptor, "active.json", f"reservation.{retained}.json",
                "domain_release_history_conflict",
            )
            os.fsync(domain_descriptor)
            self._reservations.pop(domain, None)
            return identity
        finally:
            if domain_descriptor is not None:
                os.close(domain_descriptor)
            os.close(registry)

    def _observe_active_processes(self, domain: int) -> Slice7GDomainObservationReceipt:
        occupied: list[int] = []
        ambiguous: list[int] = []
        try:
            records = self.effects.active_process_records()
            if type(records) is not tuple:
                _fail("active_process_observation", "process observation must be a detached tuple")
            needle = f"ROS_DOMAIN_ID={domain}".encode("ascii")
            for record in records:
                if type(record) is not dict:
                    _fail("active_process_observation", "process record must be an exact dictionary")
                command = record.get("command")
                environment = record.get("environment")
                pid = record.get("pid")
                if type(pid) is not int or type(command) is not bytes:
                    _fail("active_process_observation", "process record fields are malformed")
                ros_like = any(token in command.lower() for token in (b"ros2", b"rclpy", b"fastdds", b"cyclonedds"))
                if environment is None:
                    if ros_like:
                        ambiguous.append(pid)
                    continue
                if type(environment) is not bytes:
                    _fail("active_process_observation", "process environment must be bytes or null")
                if needle in environment.split(b"\0"):
                    occupied.append(pid)
            return _make_domain_observation(
                "active_processes", domain, not occupied and not ambiguous,
                {"occupied_pids": occupied, "ambiguous_pids": ambiguous},
                _production_timestamp(self.effects, "active_process_timestamp"),
            )
        except Slice7GRuntimeError:
            raise
        except Exception as exc:
            raise Slice7GRuntimeError("active_process_observation", str(exc)) from exc

    def _observe_ros_graph(self, domain: int) -> Slice7GDomainObservationReceipt:
        try:
            contract = _validated_graph_observer_contract(
                self.effects.graph_observer_contract(domain),
            )
            execution = self.effects.run_graph_observer(contract)
            if type(execution) is not Slice7GROSGraphObserverExecution:
                _fail("ros_graph_observation", "observer returned an unsupported execution record")
            if execution.returncode != 0:
                _fail("ros_graph_exit", "graph observer exit status is nonzero")
            if execution.stderr != b"":
                _fail("ros_graph_stderr", "graph observer stderr must be exactly empty")
            nodes = parse_ros_graph_observer_stdout(execution.stdout)
            cleanup_identity = _observer_cleanup_barrier(self.effects, execution, domain)
            clear = not nodes
            payload = {
                "observer_class": PRECOMMIT_ROS_GRAPH_OBSERVER_CLASS,
                "executable": contract.executable,
                "argv": list(contract.argv),
                "environment_identity": contract.environment_identity,
                "working_directory": contract.working_directory,
                "cgroup": contract.cgroup,
                "pid": execution.pid,
                "process_group_id": execution.process_group_id,
                "process_start_time_ticks": execution.process_start_time_ticks,
                "started_monotonic_ns": execution.started_monotonic_ns,
                "ended_monotonic_ns": execution.ended_monotonic_ns,
                "returncode": execution.returncode,
                "stdout_size": len(execution.stdout),
                "stdout_sha256": hashlib.sha256(execution.stdout).hexdigest(),
                "stderr_size": len(execution.stderr),
                "stderr_sha256": hashlib.sha256(execution.stderr).hexdigest(),
                "node_count": len(nodes),
                "nodes": list(nodes),
                "cleanup_barrier_identity": cleanup_identity,
            }
        except Slice7GRuntimeError:
            raise
        except Exception as exc:
            raise Slice7GRuntimeError("ros_graph_observation", str(exc)) from exc
        return _make_domain_observation(
            "ros_graph", domain, clear, payload,
            _production_timestamp(self.effects, "ros_graph_timestamp"),
        )

    def _observe_dds_participants(self, domain: int) -> Slice7GDomainObservationReceipt:
        try:
            tables = self.effects.udp_socket_tables()
            if type(tables) is not tuple or len(tables) != 2 or any(type(item) is not bytes for item in tables):
                _fail("dds_socket_observation", "UDP socket observation must contain exact byte tables")
            ports = _udp_ports_from_proc_tables(tables)
            base = 7400 + 250 * domain
            occupied = sorted(port for port in ports if base <= port < base + 250)
            return _make_domain_observation(
                "dds_participants", domain, not occupied, {"matching_udp_ports": occupied},
                _production_timestamp(self.effects, "dds_timestamp"),
            )
        except Slice7GRuntimeError:
            raise
        except Exception as exc:
            raise Slice7GRuntimeError("dds_socket_observation", str(exc)) from exc

    def _observe_external_lease(self, domain: int) -> Slice7GDomainObservationReceipt:
        active: list[str] = []
        malformed: list[str] = []
        registry: int | None = None
        domain_descriptor: int | None = None
        try:
            registry = self._open_registry()
            try:
                domain_descriptor = _open_private_directory_at(
                    registry, f"domain_{domain:03d}", "domain_lease_domain_root",
                )
            except Slice7GRuntimeError as exc:
                if exc.code != "domain_lease_domain_root_missing":
                    raise
            if domain_descriptor is not None:
                names = tuple(sorted(os.listdir(domain_descriptor)))
                reservation_pattern = re.compile(r"^reservation\.([0-9a-f]{64})\.json$")
                release_pattern = re.compile(r"^release\.([0-9a-f]{64})\.json$")
                binding_pattern = re.compile(r"^binding\.([0-9a-f]{64})\.json$")
                lease_names = (["active.json"] if "active.json" in names else [])
                lease_names.extend(name for name in names if reservation_pattern.fullmatch(name))
                release_names = [name for name in names if release_pattern.fullmatch(name)]
                binding_names = [name for name in names if binding_pattern.fullmatch(name)]
                recognized = set(lease_names) | set(release_names) | set(binding_names)
                unknown = set(names) - recognized
                malformed.extend(sorted(unknown))
                for lease_name in lease_names:
                    try:
                        lease_data = _parse_json(
                            _read_sealed_file_at(domain_descriptor, lease_name, "domain_reservation_json"),
                            "domain_reservation_json",
                        )
                        _closed(lease_data, {
                            "schema_version", "domain_id", "runtime_authorization_identity", "campaign_identity",
                            "reserved_at_utc", "identity",
                        }, "domain_reservation_fields")
                        _digest(lease_data["runtime_authorization_identity"], "reservation_runtime_authorization")
                        _digest(lease_data["campaign_identity"], "reservation_campaign")
                        _utc(lease_data["reserved_at_utc"], "reservation_timestamp")
                        _digest(lease_data["identity"], "reservation_identity")
                        if (
                            lease_data["schema_version"] != DOMAIN_RESERVATION_SCHEMA
                            or type(lease_data["domain_id"]) is not int
                            or lease_data["domain_id"] != domain
                        ):
                            _fail("domain_reservation_record", "reservation context is invalid")
                        reservation_payload = {key: value for key, value in lease_data.items() if key != "identity"}
                        reservation_identity = hashlib.sha256(
                            DOMAIN_RESERVATION_DOMAIN + _canonical(reservation_payload)
                        ).hexdigest()
                        if lease_data["identity"] != reservation_identity:
                            _fail("domain_reservation_record", "reservation identity is invalid")
                        owned = self._reservations.get(domain)
                        is_active = lease_name == "active.json"
                        release_name = f"release.{lease_data['identity']}.json"
                        if is_active:
                            if owned != lease_data["identity"]:
                                active.append(lease_name)
                        elif release_name not in names:
                            malformed.append(lease_name)
                        else:
                            release_data = _parse_json(
                                _read_sealed_file_at(domain_descriptor, release_name, "domain_release_json"),
                                "domain_release_json",
                            )
                            _closed(release_data, {
                                "schema_version", "domain_id", "domain_lease_identity", "domain_reservation_identity",
                                "released_at_utc", "identity",
                            }, "domain_release_fields")
                            _digest(release_data["domain_lease_identity"], "release_domain_lease")
                            _digest(release_data["domain_reservation_identity"], "release_reservation")
                            _utc(release_data["released_at_utc"], "release_timestamp")
                            _digest(release_data["identity"], "release_identity")
                            if (
                                release_data["schema_version"] != DOMAIN_RELEASE_SCHEMA
                                or type(release_data["domain_id"]) is not int
                                or release_data["domain_id"] != domain
                                or release_data["domain_reservation_identity"] != lease_data["identity"]
                            ):
                                _fail("domain_release_record", "release does not close the reservation")
                            release_payload = {key: value for key, value in release_data.items() if key != "identity"}
                            release_identity = hashlib.sha256(
                                DOMAIN_RELEASE_DOMAIN + _canonical(release_payload)
                            ).hexdigest()
                            if release_data["identity"] != release_identity:
                                _fail("domain_release_record", "release identity is invalid")
                    except Slice7GRuntimeError:
                        malformed.append(lease_name)
                for release_name in release_names:
                    reservation_identity = release_pattern.fullmatch(release_name).group(1)
                    history_name = f"reservation.{reservation_identity}.json"
                    if history_name not in names:
                        malformed.append(release_name)
                        continue
                    try:
                        release_data = _parse_json(
                            _read_sealed_file_at(domain_descriptor, release_name, "domain_release_json"),
                            "domain_release_json",
                        )
                        _closed(release_data, {
                            "schema_version", "domain_id", "domain_lease_identity",
                            "domain_reservation_identity", "released_at_utc", "identity",
                        }, "domain_release_fields")
                        _digest(release_data["domain_lease_identity"], "release_domain_lease")
                        _digest(release_data["domain_reservation_identity"], "release_reservation")
                        _utc(release_data["released_at_utc"], "release_timestamp")
                        _digest(release_data["identity"], "release_identity")
                        release_payload = {key: value for key, value in release_data.items() if key != "identity"}
                        if (
                            release_data["schema_version"] != DOMAIN_RELEASE_SCHEMA
                            or type(release_data["domain_id"]) is not int
                            or release_data["domain_id"] != domain
                            or release_data["domain_reservation_identity"] != reservation_identity
                            or release_data["identity"] != hashlib.sha256(
                                DOMAIN_RELEASE_DOMAIN + _canonical(release_payload)
                            ).hexdigest()
                        ):
                            _fail("domain_release_record", "release history is invalid")
                    except Slice7GRuntimeError:
                        malformed.append(release_name)
                binding_fields = {
                    "schema_version", "domain_lease_identity", "domain_reservation_identity",
                    "final_domain_observation_identity", "runtime_authorization_identity",
                    "campaign_identity", "campaign_plan_identity", "attempt_ledger_identity",
                    "attempt_ledger_revision", "process_start_event_identity", "domain_id",
                    "output_root", "identity",
                }
                for binding_name in binding_names:
                    reservation_identity = binding_pattern.fullmatch(binding_name).group(1)
                    if (
                        f"reservation.{reservation_identity}.json" not in names
                        and not (
                            "active.json" in names
                            and self._reservations.get(domain) == reservation_identity
                        )
                    ):
                        malformed.append(binding_name)
                        continue
                    try:
                        binding_data = _parse_json(
                            _read_sealed_file_at(domain_descriptor, binding_name, "domain_binding_json"),
                            "domain_binding_json",
                        )
                        _closed(binding_data, binding_fields, "domain_binding_fields")
                        for digest_field in (
                            "domain_lease_identity", "domain_reservation_identity",
                            "final_domain_observation_identity", "runtime_authorization_identity",
                            "campaign_identity", "campaign_plan_identity", "attempt_ledger_identity",
                            "process_start_event_identity", "identity",
                        ):
                            _digest(binding_data[digest_field], f"binding_{digest_field}")
                        _exact_nonnegative_int(binding_data["attempt_ledger_revision"], "binding_ledger_revision")
                        _absolute_path_text(binding_data["output_root"], "binding_output_root")
                        binding_payload = {key: value for key, value in binding_data.items() if key != "identity"}
                        if (
                            binding_data["schema_version"] != DOMAIN_COMMITTED_BINDING_SCHEMA
                            or type(binding_data["domain_id"]) is not int
                            or binding_data["domain_id"] != domain
                            or binding_data["domain_reservation_identity"] != reservation_identity
                            or binding_data["identity"] != hashlib.sha256(
                                DOMAIN_COMMITTED_BINDING_DOMAIN + _canonical(binding_payload)
                            ).hexdigest()
                        ):
                            _fail("domain_binding_record", "domain binding history is invalid")
                    except Slice7GRuntimeError:
                        malformed.append(binding_name)
            return _make_domain_observation(
                "external_lease_ledger", domain, not active and not malformed,
                {"active_reservations": active, "malformed_records": malformed},
                _production_timestamp(self.effects, "external_lease_timestamp"),
            )
        except Exception as exc:
            return _make_domain_observation(
                "external_lease_ledger", domain, False,
                {"error": type(exc).__name__, "message_sha256": hashlib.sha256(str(exc).encode()).hexdigest()},
                _production_timestamp(self.effects, "external_lease_timestamp"),
            )
        finally:
            if domain_descriptor is not None:
                os.close(domain_descriptor)
            if registry is not None:
                os.close(registry)


class Slice7GDomainAllocator:
    """Select one occupancy-checked domain using injected providers."""

    def __init__(
        self,
        occupancy_provider: Callable[[int], Slice7GDomainOccupancy | None],
        lease_provider: Callable[[int, str, str], str | None],
        release_provider: Callable[[int, str], str | None],
    ) -> None:
        if not callable(occupancy_provider) or not callable(lease_provider) or not callable(release_provider):
            _fail("domain_provider", "domain providers must be callable")
        self._occupancy = occupancy_provider
        self._lease = lease_provider
        self._release = release_provider

    def allocate(
        self, charter: Slice7GCharter, authorization: Slice7GRuntimeAuthorization, timestamp_utc: str,
    ) -> Slice7GDomainLease:
        authorization = _validated_runtime_authorization(authorization, charter)
        _utc(timestamp_utc, "leased_at_utc")
        for domain in range(100, 200):
            try:
                occupancy = self._occupancy(domain)
            except Slice7GRuntimeError:
                raise
            except Exception as exc:
                raise Slice7GRuntimeError(
                    "domain_occupancy_provider_failed", "domain occupancy provider failed",
                ) from exc
            if type(occupancy) is not Slice7GDomainOccupancy or occupancy.domain_id != domain:
                _fail("domain_occupancy_unproven", "occupancy provider did not return an exact bound record")
            if not occupancy.collision_free:
                continue
            try:
                receipt = self._lease(domain, authorization.identity, authorization.campaign_identity)
            except Slice7GRuntimeError:
                raise
            except Exception as exc:
                raise Slice7GRuntimeError("domain_lease_provider_failed", "domain lease provider failed") from exc
            if receipt is None:
                continue
            _digest(receipt, "provider_receipt_identity")
            payload = {
                "schema_version": DOMAIN_LEASE_SCHEMA,
                "charter_logical_identity": slice_7g_charter_identity(charter),
                "runtime_authorization_identity": authorization.identity,
                "campaign_identity": authorization.campaign_identity,
                "domain_id": domain,
                "occupancy_checked": True,
                "collision_free": True,
                "provider_receipt_identity": receipt,
                "leased_at_utc": timestamp_utc,
            }
            identity = hashlib.sha256(DOMAIN_LEASE_DOMAIN + _canonical(payload)).hexdigest()
            return Slice7GDomainLease(identity=identity, **payload)
        _fail("domain_unavailable", "no collision-free lease was established in 100..199")

    def release(self, lease: Slice7GDomainLease, timestamp_utc: str) -> Slice7GDomainRelease:
        lease = _validated_domain_lease(lease)
        _utc(timestamp_utc, "released_at_utc")
        try:
            receipt = self._release(lease.domain_id, lease.identity)
        except Slice7GRuntimeError:
            raise
        except Exception as exc:
            raise Slice7GRuntimeError("domain_release_provider_failed", "domain release provider failed") from exc
        if receipt is None:
            _fail("domain_release_unproven", "domain release provider did not retain a receipt")
        _digest(receipt, "release_receipt_identity")
        return Slice7GDomainRelease(lease.identity, receipt, timestamp_utc)


class Slice7GReadinessTracker:
    """ROS-independent freshness and stability authenticator."""

    def __init__(self, start_timestamp: float = 0.0) -> None:
        self._start_timestamp = _finite(start_timestamp, "readiness_start_timestamp")
        self._samples: list[tuple[float, tuple[float, ...], tuple[float, ...]]] = []
        self._tactile: tuple[float, bool, str] | None = None
        self._safety: tuple[float, bool, bool, str] | None = None

    def add_state_tip(self, timestamp: float, q: Sequence[float], tip: Sequence[float]) -> None:
        t = _finite(timestamp, "timestamp")
        q_value = _finite_tuple(q, 6, "q")
        tip_value = _finite_tuple(tip, 3, "tip")
        if self._samples and t <= self._samples[-1][0]:
            _fail("readiness_timestamp_order", "state/tip timestamps must strictly increase")
        self._samples.append((t, q_value, tip_value))

    def update_tactile(self, timestamp: float, *, valid: bool, source: str) -> None:
        if type(valid) is not bool or type(source) is not str:
            _fail("readiness_tactile_type", "tactile fields have invalid types")
        self._tactile = (_finite(timestamp, "tactile_timestamp"), valid, source)

    def update_safety(self, timestamp: float, *, ready: bool, fault: bool, state_name: str) -> None:
        if type(ready) is not bool or type(fault) is not bool or type(state_name) is not str:
            _fail("readiness_safety_type", "safety fields have invalid types")
        self._safety = (_finite(timestamp, "safety_timestamp"), ready, fault, state_name)

    def evaluate(self, now: float) -> Slice7GReadinessResult:
        current = _finite(now, "now")
        if current < self._start_timestamp:
            return self._result(False, "readiness_clock_reversed", current)
        if current - self._start_timestamp > 10.0:
            return self._result(False, "readiness_timeout", current)
        if len(self._samples) < 10:
            return self._result(False, "readiness_insufficient_samples", current)
        selected = self._samples[-10:]
        interval = selected[-1][0] - selected[0][0]
        q_variation = max(
            max(abs(sample[1][index] - selected[0][1][index]) for index in range(6))
            for sample in selected
        )
        tip_variation = max(
            max(abs(sample[2][index] - selected[0][2][index]) for index in range(3))
            for sample in selected
        )
        if interval < 0.5:
            return self._result(False, "readiness_stable_interval", current, interval, q_variation, tip_variation)
        if q_variation > 5.0e-5:
            return self._result(False, "readiness_q_variation", current, interval, q_variation, tip_variation)
        if tip_variation > 5.0e-5:
            return self._result(False, "readiness_tip_variation", current, interval, q_variation, tip_variation)
        if self._tactile is None:
            return self._result(False, "readiness_tactile_missing", current, interval, q_variation, tip_variation)
        tactile_age = current - self._tactile[0]
        if tactile_age < 0.0 or tactile_age > 0.10 or not self._tactile[1] or self._tactile[2] != "simulated":
            return self._result(False, "readiness_tactile_stale_or_invalid", current, interval, q_variation, tip_variation)
        if self._safety is None:
            return self._result(False, "readiness_safety_missing", current, interval, q_variation, tip_variation)
        safety_age = current - self._safety[0]
        if safety_age < 0.0 or safety_age > 0.10:
            return self._result(False, "readiness_safety_stale", current, interval, q_variation, tip_variation)
        if not self._safety[1] or self._safety[2] or self._safety[3] not in {"ready", "warning"}:
            return self._result(False, "readiness_safety_fault", current, interval, q_variation, tip_variation)
        return Slice7GReadinessResult(True, "", 10, interval, q_variation, tip_variation, tactile_age, safety_age)

    def _result(
        self, passed: bool, code: str, now: float, interval: float = 0.0,
        q_variation: float = 0.0, tip_variation: float = 0.0,
    ) -> Slice7GReadinessResult:
        tactile_age = math.inf if self._tactile is None else now - self._tactile[0]
        safety_age = math.inf if self._safety is None else now - self._safety[0]
        return Slice7GReadinessResult(
            passed, code, min(len(self._samples), 10), interval, q_variation, tip_variation,
            tactile_age, safety_age,
        )


class AtomicSlice7GOutputAllocator:
    """Prospectively create exactly the authorization-bound empty campaign root."""

    def allocate(self, authorization: Slice7GRuntimeAuthorization) -> str:
        authorization = _validated_runtime_authorization(authorization)
        root = Path(authorization.campaign_output_root)
        _require_real_directory(root.parent, "campaign_output_parent")
        try:
            root.mkdir(mode=0o700)
        except FileExistsError as exc:
            raise Slice7GRuntimeError("output_root_exists", "campaign output root must be new", path=str(root)) from exc
        except OSError as exc:
            raise Slice7GRuntimeError("output_root_create", str(exc), path=str(root)) from exc
        try:
            _require_real_directory(root, "campaign_output_root")
            try:
                entries = tuple(root.iterdir())
            except OSError as exc:
                raise Slice7GRuntimeError("output_root_inventory", str(exc), path=str(root)) from exc
            if entries:
                _fail("output_root_not_empty", "new campaign output root is not empty", str(root))
            _fsync_directory(root)
            _fsync_directory(root.parent)
            return str(root)
        except Exception:
            try:
                root.rmdir()
            except OSError:
                pass
            raise


class AtomicSlice7GLedgerWriter:
    """Versioned no-replace ledger commit store.

    The winning revision file is the atomic commit point.  No process factory
    may be called until ``commit`` has returned the committed ledger.
    """

    def __init__(self, parent: str | os.PathLike[str]) -> None:
        self.parent = Path(_path(parent, "ledger_parent"))
        if self.parent.exists() or self.parent.is_symlink():
            _require_real_directory(self.parent, "ledger_parent")
        elif not self.parent.parent.exists():
            grandparent = self.parent.parent.parent
            _require_real_directory(grandparent, "ledger_control_parent")
        else:
            _require_real_directory(self.parent.parent, "ledger_control_parent")

    def initialize(self, ledger: Slice7GAttemptLedger) -> Slice7GAttemptLedger:
        if type(ledger) is not Slice7GAttemptLedger:
            _fail("ledger_initial_state", "initial ledger must be an exact record")
        try:
            schema_version = ledger.schema_version
            revision = ledger.revision
        except AttributeError as exc:
            raise Slice7GRuntimeError("ledger_initial_state", "initial ledger is partially initialized") from exc
        if schema_version != ATTEMPT_LEDGER_SCHEMA_VERSION or revision != 0:
            _fail("ledger_initial_state", "initial ledger must be revision zero")
        if not self.parent.exists():
            if not self.parent.parent.exists():
                _ensure_private_directory(self.parent.parent)
            _ensure_private_directory(self.parent)
        else:
            _require_real_directory(self.parent, "ledger_parent")
        self._commit_file(ledger, None)
        return ledger

    def commit(
        self, current: Slice7GAttemptLedger, event: Any, *, campaign_plan: Slice7GCampaignPlan | None = None,
    ) -> Slice7GAttemptLedger:
        if type(current) is not Slice7GAttemptLedger:
            _fail("ledger_predecessor_type", "ledger predecessor must be an exact record")
        try:
            current_revision = current.revision
        except AttributeError as exc:
            raise Slice7GRuntimeError("ledger_predecessor_type", "ledger predecessor is partial") from exc
        current_path = self._revision_path(current_revision)
        observed = self._read_commit(current_path)
        if slice_7g_attempt_ledger_identity(observed) != slice_7g_attempt_ledger_identity(current):
            _fail("ledger_stale_predecessor", "retained predecessor differs from the caller expectation")
        successor = validate_slice_7g_attempt_transition(current, event, campaign_plan=campaign_plan)
        self._commit_file(successor, event)
        return successor

    def _revision_path(self, revision: int) -> Path:
        return self.parent / f"attempt_ledger.r{revision:08d}.json"

    def commit_domain_binding(self, binding: Slice7GDomainBinding) -> Path:
        if type(binding) is not Slice7GDomainBinding:
            _fail("domain_binding_type", "domain binding must be an exact record")
        try:
            data = {
                "schema_version": binding.schema_version,
                "lease_identity": binding.lease_identity,
                "runtime_authorization_identity": binding.runtime_authorization_identity,
                "campaign_identity": binding.campaign_identity,
                "attempt_ledger_identity": binding.attempt_ledger_identity,
                "attempt_ledger_revision": binding.attempt_ledger_revision,
                "domain_id": binding.domain_id,
                "output_root": binding.output_root,
            }
            cached_identity = binding.identity
        except AttributeError as exc:
            raise Slice7GRuntimeError("domain_binding_record", "domain binding is partially initialized") from exc
        expected = hashlib.sha256(DOMAIN_BINDING_DOMAIN + _canonical(data)).hexdigest()
        if cached_identity != expected:
            _fail("domain_binding_identity", "domain binding identity is invalid")
        final = self.parent / "domain_binding.json"
        _commit_noreplace_file(self.parent, final, _canonical({**data, "identity": expected}), "domain_binding_conflict")
        return final

    def commit_domain_release(self, release: Slice7GDomainRelease) -> Path:
        """Retain the immutable release receipt without rewriting attempt history."""

        if type(release) is not Slice7GDomainRelease:
            _fail("domain_release_type", "domain release must be an exact record")
        try:
            lease_identity = _digest(release.lease_identity, "domain_lease_identity")
            provider_identity = _digest(
                release.provider_receipt_identity, "domain_release_provider_identity",
            )
            released_at = _utc(release.released_at_utc, "domain_released_at_utc")
        except AttributeError as exc:
            raise Slice7GRuntimeError(
                "domain_release_record", "domain release is partially initialized",
            ) from exc
        data = {
            "schema_version": DOMAIN_RELEASE_SCHEMA,
            "domain_lease_identity": lease_identity,
            "provider_receipt_identity": provider_identity,
            "released_at_utc": released_at,
        }
        final = self.parent / "domain_release_receipt.json"
        _commit_noreplace_file(
            self.parent, final, _canonical(data), "domain_release_receipt_conflict",
        )
        return final

    def commit_final_domain_observation(
        self,
        ledger: Slice7GAttemptLedger,
        lease: Slice7GDomainLease,
        observation: Slice7GDomainOccupancy,
    ) -> Path:
        if type(ledger) is not Slice7GAttemptLedger or type(observation) is not Slice7GDomainOccupancy:
            _fail("final_domain_observation_record", "final observation context uses unsupported records")
        lease = _validated_domain_lease(lease)
        if (
            not ledger.process_start_committed
            or ledger.consumed_campaign_attempts != 1
            or ledger.domain_id != lease.domain_id
            or observation.domain_id != lease.domain_id
        ):
            _fail("final_domain_observation_context", "final observation differs from the committed attempt")
        data = {
            "schema_version": FINAL_DOMAIN_OBSERVATION_SCHEMA,
            "attempt_ledger_identity": slice_7g_attempt_ledger_identity(ledger),
            "attempt_ledger_revision": ledger.revision,
            "process_start_event_identity": ledger.last_event_identity,
            "domain_lease_identity": lease.identity,
            "domain_id": observation.domain_id,
            "active_processes_clear": observation.active_processes_clear,
            "ros_graph_clear": observation.ros_graph_clear,
            "dds_participants_clear": observation.dds_participants_clear,
            "external_ledger_clear": observation.external_ledger_clear,
            "checked_at_utc": observation.checked_at_utc,
            "observation_receipt_identity": observation.receipt_identity,
            "collision_free": observation.collision_free,
        }
        final = self.parent / "final_domain_observation.json"
        _commit_noreplace_file(
            self.parent, final, _canonical(data), "final_domain_observation_conflict",
        )
        return final

    def commit_cleanup_failure(
        self,
        primary: Exception,
        cleanup_issues: tuple[Slice7GCleanupIssue, ...],
    ) -> Path:
        if not isinstance(primary, Exception):
            _fail("cleanup_failure_record", "primary failure must be an ordinary exception")
        if type(cleanup_issues) is not tuple or not cleanup_issues:
            _fail("cleanup_failure_record", "cleanup failure record requires an immutable nonempty tuple")
        records = []
        for issue in cleanup_issues:
            if type(issue) is not Slice7GCleanupIssue:
                _fail("cleanup_failure_record", "cleanup issue uses an unsupported record")
            records.append({"code": issue.code, "detail": issue.detail})
        data = {
            "schema_version": CLEANUP_FAILURE_SCHEMA,
            "primary_code": primary.code if isinstance(primary, Slice7GRuntimeError) else "campaign_failure",
            "primary_detail": str(primary),
            "cleanup_issues": records,
        }
        final = self.parent / "cleanup_failure.json"
        _commit_noreplace_file(self.parent, final, _canonical(data), "cleanup_failure_conflict")
        return final

    def _commit_file(self, ledger: Slice7GAttemptLedger, event: Any | None) -> None:
        final = self._revision_path(ledger.revision)
        event_bytes = None if event is None else canonical_slice_7g_attempt_event_bytes(event)
        record = {
            "schema_version": LEDGER_COMMIT_SCHEMA,
            "ledger": json.loads(canonical_slice_7g_attempt_ledger_bytes(ledger).decode("utf-8")),
            "ledger_identity": slice_7g_attempt_ledger_identity(ledger),
            "event": None if event_bytes is None else json.loads(event_bytes.decode("utf-8")),
            "event_identity": None if event is None else slice_7g_attempt_event_identity(event),
        }
        raw = _canonical(record)
        temp = self.parent / f".{final.name}.{os.getpid()}.{id(record):x}.tmp"
        descriptor = None
        try:
            descriptor = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0), 0o600)
            _write_all(descriptor, raw)
            os.fsync(descriptor)
            os.fchmod(descriptor, 0o444)
            os.fsync(descriptor)
            os.close(descriptor)
            descriptor = None
            try:
                os.link(temp, final, follow_symlinks=False)
            except FileExistsError as exc:
                raise Slice7GRuntimeError("ledger_commit_conflict", "revision already committed", path=str(final)) from exc
            os.unlink(temp)
            _fsync_directory(self.parent)
        except Exception:
            if descriptor is not None:
                os.close(descriptor)
            try:
                temp.unlink()
            except FileNotFoundError:
                pass
            raise

    def _read_commit(self, path: Path) -> Slice7GAttemptLedger:
        try:
            raw = _read_sealed_file_nofollow(str(path))
            data = _parse_json(raw, "ledger_commit_json")
        except OSError as exc:
            raise Slice7GRuntimeError("ledger_predecessor_missing", str(exc), path=str(path)) from exc
        if raw != _canonical(data):
            _fail("ledger_commit_noncanonical", "retained ledger commit is not canonical", str(path))
        _closed(data, {"schema_version", "ledger", "ledger_identity", "event", "event_identity"}, "ledger_commit_fields")
        if data["schema_version"] != LEDGER_COMMIT_SCHEMA:
            _fail("ledger_commit_schema", "unsupported ledger commit schema")
        ledger_data = data["ledger"]
        if type(ledger_data) is not dict:
            _fail("ledger_commit_record", "ledger must be an exact object")
        ledger_fields = {item.name for item in fields(Slice7GAttemptLedger)}
        _closed(ledger_data, ledger_fields, "ledger_commit_record")
        identities = ledger_data.get("applied_event_identities")
        event_ids = ledger_data.get("applied_event_ids")
        if type(identities) is not list or type(event_ids) is not list:
            _fail("ledger_commit_record", "ledger event histories must be exact arrays")
        try:
            ledger = Slice7GAttemptLedger(**{
                **ledger_data,
                "applied_event_identities": tuple(identities),
                "applied_event_ids": tuple(event_ids),
            })
        except Slice7GGovernanceError as exc:
            raise Slice7GRuntimeError("ledger_commit_record", str(exc), path=str(path)) from exc
        if data["ledger_identity"] != slice_7g_attempt_ledger_identity(ledger):
            _fail("ledger_commit_identity", "retained ledger identity is invalid")
        if ledger.revision == 0:
            if data["event"] is not None or data["event_identity"] is not None:
                _fail("ledger_commit_event", "initial ledger must not claim an event")
        else:
            event_data = data["event"]
            if type(event_data) is not dict:
                _fail("ledger_commit_event", "committed revision requires an exact event")
            _closed(event_data, {item.name for item in fields(Slice7GAttemptEvent)}, "ledger_commit_event")
            try:
                event = Slice7GAttemptEvent(**event_data)
            except Slice7GGovernanceError as exc:
                raise Slice7GRuntimeError("ledger_commit_event", str(exc), path=str(path)) from exc
            event_identity = slice_7g_attempt_event_identity(event)
            if data["event_identity"] != event_identity or ledger.last_event_identity != event_identity:
                _fail("ledger_commit_event", "retained event identity differs from the ledger")
        return ledger


class _CellOutputAuthority:
    """Own bounded output-tree descriptors and exact cached semantic bytes."""

    def __init__(self, root: str | os.PathLike[str]) -> None:
        self._root_descriptor: int | None = None
        self._closed = True
        self._finalized = False
        self._directories: dict[str, _CellOutputDirectory] = {}
        self._members: dict[str, _CellOutputMember] = {}
        self._semantic_paths = set(_INITIAL_CELL_OUTPUT_SEMANTIC_PATHS)
        self._accounting = _CellOutputAccounting()
        self.root_path = _absolute_path_text(_path(root, "cell_output_root"), "cell_output_root")
        descriptor = _open_directory_path_nofollow(self.root_path, "cell_output_root")
        self._root_descriptor = descriptor
        self._closed = False
        initialized = False
        try:
            self._root_metadata = _stable_metadata(_cell_output_fstat(descriptor, self.root_path))
            if stat.S_IMODE(self._root_metadata[2]) != 0o555:
                _fail("cell_output_root_mode", "finalized cell output root must use mode 0555")
            directories, members, accounting = self._scan_tree(
                frozenset(self._semantic_paths), cache_semantics=True,
            )
            self._directories = directories
            self._members = members
            self._accounting = accounting
            projection = {
                "schema_version": "ctr-slice-7g-output-inventory-1",
                "members": [
                    {
                        "path": item.path,
                        "size": item.size,
                        "sha256": item.sha256,
                        "mode": stat.S_IMODE(item.metadata[2]),
                    }
                    for item in self.members
                ],
            }
            self.inventory_identity = hashlib.sha256(
                b"ctr-slice-7g-output-inventory-canonical-1\0" + _canonical(projection)
            ).hexdigest()
            initialized = True
        finally:
            if not initialized:
                self.close()

    @property
    def members(self) -> tuple[_CellOutputMember, ...]:
        self._require_live()
        return tuple(self._members[path] for path in sorted(self._members))

    def member_bytes(self, relative: str) -> bytes:
        self._require_live()
        path = _safe_relative(relative, "cell_output_member")
        member = self._members.get(path)
        if member is None:
            _fail("cell_output_member_missing", "authenticated output member is missing", path)
        if member.semantic_bytes is None:
            _fail(
                "cell_output_member_not_semantic",
                "complete bytes are retained only for closed production semantic members",
                path,
            )
        return bytes(member.semantic_bytes)

    def member_observation(self, relative: str) -> _CellOutputMember:
        self._require_live()
        path = _safe_relative(relative, "cell_output_member")
        member = self._members.get(path)
        if member is None:
            _fail("cell_output_member_missing", "authenticated output member is missing", path)
        return member

    def bind_candidate_semantic_directory(self, relative: str) -> None:
        """Cache only the two candidate records parsed by the production adapter."""

        self._require_live()
        directory = _safe_relative(relative, "runner_candidate_path")
        self.require_directory(directory)
        requested = tuple(
            f"{directory}/{name}" for name in ("summary.json", "orchestration.json")
        )
        budget = self._accounting
        records: list[_CellOutputMember] = []
        for path in requested:
            record = self._members.get(path)
            if record is None:
                _fail("cell_output_member_missing", "required semantic output member is missing", path)
            if record.semantic_bytes is None:
                budget = budget.add_semantic_cache(record.size)
            records.append(record)
        cached: list[tuple[str, bytes]] = []
        for path, record in zip(requested, records):
            if record.semantic_bytes is None:
                cached.append((path, self._read_semantic_member(path, record)))
        for path, raw in cached:
            record = self._members[path]
            self._members[path] = _CellOutputMember(
                record.path, record.metadata, record.size, record.sha256, raw,
            )
        self._semantic_paths.update(requested)
        self._accounting = budget

    def require_directory(self, relative: str) -> None:
        self._require_live()
        path = _safe_relative(relative, "cell_output_directory")
        if path not in self._directories:
            _fail("cell_output_directory_missing", "authenticated output directory is missing", path)

    def inventory_payload(self, missing_required_result_file_count: int) -> dict[str, Any]:
        self._require_live()
        missing = _exact_nonnegative_int(
            missing_required_result_file_count, "missing_required_result_file_count",
        )
        return {
            "missing_required_result_file_count": missing,
            "output_tree_identity": self.inventory_identity,
            "regular_file_count": len(self._members),
            "regular_file_bytes": self._accounting.total_file_bytes,
        }

    def final_barrier(self, expected_inventory_identity: str) -> None:
        self._require_live()
        _digest(expected_inventory_identity, "output_tree_identity")
        if self._finalized:
            _fail("cell_output_authority_finalized", "cell output authority was already finalized")
        if expected_inventory_identity != self.inventory_identity:
            _fail("cell_output_inventory_identity", "evidence binding differs from authenticated output bytes")
        if _stable_metadata(_cell_output_fstat(self._root_descriptor, self.root_path)) != self._root_metadata:
            _fail("cell_output_changed", "cell output root metadata changed before final barrier")
        directories, members, _ = self._scan_tree(
            frozenset(self._semantic_paths), cache_semantics=False,
        )
        if set(directories) != set(self._directories) or set(members) != set(self._members):
            _fail("cell_output_inventory_changed", "cell output inventory changed before final barrier")
        for path, observed in directories.items():
            if observed.metadata != self._directories[path].metadata:
                _fail("cell_output_changed", "cell output directory metadata changed", path)
        for path, observed in members.items():
            expected = self._members[path]
            if (
                observed.metadata != expected.metadata
                or observed.size != expected.size
                or observed.sha256 != expected.sha256
            ):
                _fail("cell_output_changed", "cell output member changed", path)
        reopened = _open_directory_path_nofollow(self.root_path, "cell_output_root")
        try:
            reopened_metadata = _stable_metadata(_cell_output_fstat(reopened, self.root_path))
            if reopened_metadata[:2] != self._root_metadata[:2] or reopened_metadata != self._root_metadata:
                _fail("cell_output_root_replaced", "cell output pathname no longer names the authenticated root")
        finally:
            os.close(reopened)
        self._finalized = True

    def close(self) -> None:
        if getattr(self, "_closed", True):
            return
        self._closed = True
        descriptor = getattr(self, "_root_descriptor", None)
        self._root_descriptor = None
        if descriptor is not None:
            os.close(descriptor)

    def _require_live(self) -> None:
        if getattr(self, "_closed", True):
            _fail("cell_output_authority_closed", "cell output authority is closed")

    def _scan_tree(
        self, semantic_paths: frozenset[str], *, cache_semantics: bool,
    ) -> tuple[
        dict[str, _CellOutputDirectory],
        dict[str, _CellOutputMember],
        _CellOutputAccounting,
    ]:
        root_descriptor = self._root_descriptor
        if type(root_descriptor) is not int:
            _fail("cell_output_authority_closed", "cell output authority is closed")
        directories: dict[str, _CellOutputDirectory] = {}
        members: dict[str, _CellOutputMember] = {}
        inodes: set[tuple[int, int]] = set()
        accounting = _CellOutputAccounting()
        root_metadata = _stable_metadata(_cell_output_fstat(root_descriptor, self.root_path))
        root_names = self._directory_names(root_descriptor, "")
        stack = [
            _CellOutputTraversalFrame(
                "", 0, root_descriptor, root_names, 0, root_metadata, False,
            )
        ]
        try:
            while stack:
                frame = stack[-1]
                if frame.next_index == len(frame.names):
                    if _stable_metadata(
                        _cell_output_fstat(frame.descriptor, frame.path or self.root_path)
                    ) != frame.baseline_metadata:
                        _fail(
                            "cell_output_changed", "cell output directory changed while enumerating",
                            frame.path or self.root_path,
                        )
                    if self._directory_names(frame.descriptor, frame.path) != frame.names:
                        _fail(
                            "cell_output_inventory_changed", "cell output entries changed while enumerating",
                            frame.path or self.root_path,
                        )
                    stack.pop()
                    if frame.owns_descriptor:
                        os.close(frame.descriptor)
                    continue
                name = frame.names[frame.next_index]
                stack[-1] = _CellOutputTraversalFrame(
                    frame.path, frame.depth, frame.descriptor, frame.names,
                    frame.next_index + 1, frame.baseline_metadata, frame.owns_descriptor,
                )
                _safe_component(name, "cell_output_member")
                relative = f"{frame.path}/{name}" if frame.path else name
                depth = frame.depth + 1
                accounting._validate_descendant(depth)
                by_name = self._stat_at(frame.descriptor, name, relative)
                if stat.S_ISLNK(by_name.st_mode):
                    _fail("cell_output_symlink", "cell output cannot contain symlinks", relative)
                if stat.S_ISDIR(by_name.st_mode):
                    accounting = accounting.add_directory(depth)
                    child = self._open_directory_at(frame.descriptor, name, relative)
                    try:
                        child_metadata = _stable_metadata(_cell_output_fstat(child, relative))
                        if child_metadata != _stable_metadata(by_name):
                            _fail("cell_output_changed", "directory entry changed while opening", relative)
                        if stat.S_IMODE(child_metadata[2]) != 0o555:
                            _fail(
                                "cell_output_directory_mode",
                                "finalized output directory must use mode 0555",
                                relative,
                            )
                        inode = child_metadata[:2]
                        if inode in inodes:
                            _fail("cell_output_inode_alias", "cell output directory inode is aliased", relative)
                        inodes.add(inode)
                        directories[relative] = _CellOutputDirectory(relative, child_metadata)
                        child_names = self._directory_names(child, relative)
                        stack.append(_CellOutputTraversalFrame(
                            relative, depth, child, child_names, 0, child_metadata, True,
                        ))
                        child = None
                    finally:
                        if child is not None:
                            os.close(child)
                    continue
                if not stat.S_ISREG(by_name.st_mode) or by_name.st_nlink != 1:
                    _fail("cell_output_member_type", "cell output member must be a unique regular file", relative)
                if stat.S_IMODE(by_name.st_mode) != 0o444:
                    _fail("cell_output_member_mode", "finalized output member must use mode 0444", relative)
                semantic = relative in semantic_paths
                accounting = accounting.add_file(
                    depth, by_name.st_size, semantic=semantic,
                    cache_semantic=semantic and cache_semantics,
                )
                member_descriptor: int | None = None
                try:
                    member_descriptor = self._open_file_at(frame.descriptor, name, relative)
                    opened = _cell_output_fstat(member_descriptor, relative)
                    opened_metadata = _stable_metadata(opened)
                    if opened_metadata != _stable_metadata(by_name):
                        _fail("cell_output_changed", "file entry changed while opening", relative)
                    inode = opened_metadata[:2]
                    if inode in inodes:
                        _fail("cell_output_hardlink", "cell output contains an inode alias", relative)
                    inodes.add(inode)
                    digest, raw = _stream_cell_output_descriptor(
                        member_descriptor, opened.st_size,
                        capture=semantic and cache_semantics, path=relative,
                    )
                    if _stable_metadata(
                        _cell_output_fstat(member_descriptor, relative)
                    ) != opened_metadata:
                        _fail("cell_output_changed", "cell output member changed while hashing", relative)
                    current_name = self._stat_at(frame.descriptor, name, relative)
                    if _stable_metadata(current_name) != opened_metadata:
                        _fail("cell_output_changed", "file entry changed after hashing", relative)
                    members[relative] = _CellOutputMember(
                        relative, opened_metadata, opened.st_size, digest, raw,
                    )
                finally:
                    if member_descriptor is not None:
                        os.close(member_descriptor)
        finally:
            for frame in reversed(stack):
                if frame.owns_descriptor:
                    os.close(frame.descriptor)
        return directories, members, accounting

    def _read_semantic_member(self, relative: str, expected: _CellOutputMember) -> bytes:
        components = relative.split("/")
        parent = self._root_descriptor
        if type(parent) is not int:
            _fail("cell_output_authority_closed", "cell output authority is closed")
        opened_directories: list[int] = []
        member_descriptor: int | None = None
        try:
            prefix: list[str] = []
            for component in components[:-1]:
                prefix.append(component)
                path = "/".join(prefix)
                descriptor = self._open_directory_at(parent, component, path)
                opened_directories.append(descriptor)
                parent = descriptor
                retained = self._directories.get(path)
                if retained is None or _stable_metadata(
                    _cell_output_fstat(descriptor, path)
                ) != retained.metadata:
                    _fail("cell_output_changed", "semantic member parent changed", path)
            member_descriptor = self._open_file_at(parent, components[-1], relative)
            opened = _stable_metadata(_cell_output_fstat(member_descriptor, relative))
            if opened != expected.metadata:
                _fail("cell_output_changed", "semantic output member changed before caching", relative)
            digest, raw = _stream_cell_output_descriptor(
                member_descriptor, expected.size, capture=True, path=relative,
            )
            if raw is None or digest != expected.sha256:
                _fail("cell_output_changed", "semantic output member differs from authenticated inventory", relative)
            if _stable_metadata(
                _cell_output_fstat(member_descriptor, relative)
            ) != expected.metadata:
                _fail("cell_output_changed", "semantic output member changed while caching", relative)
            current = self._stat_at(parent, components[-1], relative)
            if _stable_metadata(current) != expected.metadata:
                _fail("cell_output_changed", "semantic output pathname changed while caching", relative)
            return raw
        finally:
            if member_descriptor is not None:
                os.close(member_descriptor)
            for descriptor in reversed(opened_directories):
                os.close(descriptor)

    def _directory_names(self, descriptor: int, path: str) -> tuple[str, ...]:
        return _bounded_cell_output_directory_names(descriptor, path or self.root_path)

    @staticmethod
    def _stat_at(parent: int, name: str, path: str) -> os.stat_result:
        try:
            return os.stat(name, dir_fd=parent, follow_symlinks=False)
        except Exception as exc:
            raise Slice7GRuntimeError(
                "cell_output_traversal_failed", "cell output entry observation failed", path=path,
            ) from exc

    @staticmethod
    def _open_directory_at(parent: int, name: str, path: str) -> int:
        try:
            return os.open(
                name,
                os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
                | getattr(os, "O_CLOEXEC", 0),
                dir_fd=parent,
            )
        except Exception as exc:
            raise Slice7GRuntimeError(
                "cell_output_traversal_failed", "cell output directory open failed", path=path,
            ) from exc

    @staticmethod
    def _open_file_at(parent: int, name: str, path: str) -> int:
        try:
            return os.open(
                name,
                os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0),
                dir_fd=parent,
            )
        except Exception as exc:
            raise Slice7GRuntimeError(
                "cell_output_traversal_failed", "cell output member open failed", path=path,
            ) from exc


class Slice7GEvidenceWriter:
    """Exclusive, rollback-safe producer for the governance evidence reader."""

    def __init__(self, campaign_root: str | os.PathLike[str]) -> None:
        self.campaign_root = Path(_absolute_path_text(_path(campaign_root, "campaign_root"), "campaign_root"))

    def write_cell_package(
        self,
        execution: Slice7GCellExecution,
        charter: Slice7GCharter,
        ledger: Slice7GAttemptLedger,
        plan: Slice7GCampaignPlan,
    ) -> tuple[Path, str, str]:
        return self._write_cell_package(execution, charter, ledger, plan, None)

    def _write_authenticated_cell_package(
        self,
        execution: Slice7GCellExecution,
        charter: Slice7GCharter,
        ledger: Slice7GAttemptLedger,
        plan: Slice7GCampaignPlan,
        output_authority: _CellOutputAuthority,
    ) -> tuple[Path, str, str]:
        if type(output_authority) is not _CellOutputAuthority:
            _fail("cell_output_authority_type", "production evidence requires the private output authority")
        return self._write_cell_package(execution, charter, ledger, plan, output_authority)

    def _write_cell_package(
        self,
        execution: Slice7GCellExecution,
        charter: Slice7GCharter,
        ledger: Slice7GAttemptLedger,
        plan: Slice7GCampaignPlan,
        output_authority: _CellOutputAuthority | None,
    ) -> tuple[Path, str, str]:
        self._validate_context(charter, ledger, plan)
        execution = _detached_execution(execution)
        _validate_execution_payloads(execution)
        result = execution.cell_result
        cell = _planned_cell(plan, result.cell_id)
        canonical_slice_7g_cell_result_bytes(result)
        package_parent = self.campaign_root / "evidence" / "packages"
        _ensure_owned_directory(self.campaign_root / "evidence")
        _ensure_owned_directory(package_parent)
        final = package_parent / cell.cell_id
        staging = package_parent / f".{cell.cell_id}.staging.{os.getpid()}.{id(execution):x}"
        if final.exists() or staging.exists():
            _fail("evidence_package_exists", "cell evidence path already exists", str(final))
        staging.mkdir(mode=0o700)
        published = False
        try:
            bindings = _evidence_bindings(result)
            payloads = {
                "invocation_process_start_receipt": execution.invocation_payload,
                "runtime_authorization_binding": {"runtime_authorization_identity": result.runtime_authorization_identity},
                "readiness_trace": execution.readiness_payload,
                "safety_trace": execution.safety_payload,
                "tactile_trace": execution.tactile_payload,
                "output_inventory_receipt": execution.output_inventory_payload,
            }
            role_bytes: dict[str, bytes] = {}
            for role in MANDATORY_EVIDENCE_ROLE_PATHS:
                if role == "cell_result":
                    role_bytes[role] = canonical_slice_7g_cell_result_bytes(result)
                else:
                    role_bytes[role] = _canonical({
                        "schema_version": CELL_EVIDENCE_MEMBER_SCHEMA_VERSION,
                        "role": role,
                        "bindings": bindings,
                        "payload": payloads[role],
                    })
            members = []
            for role, relative in MANDATORY_EVIDENCE_ROLE_PATHS.items():
                raw = role_bytes[role]
                _exclusive_sealed_file(staging / relative, raw)
                members.append({
                    "role": role, "path": relative, "size": len(raw),
                    "sha256": hashlib.sha256(raw).hexdigest(), "mode": 0o444,
                    "link_count": 1, "file_type": "regular_file",
                })
            projection = {"schema_version": CELL_EVIDENCE_PROJECTION_SCHEMA_VERSION, "members": members}
            projection_raw = _canonical(projection)
            projection_identity = hashlib.sha256(
                CELL_EVIDENCE_PROJECTION_IDENTITY_DOMAIN + projection_raw
            ).hexdigest()
            envelope = {
                "schema_version": CELL_EVIDENCE_ENVELOPE_SCHEMA_VERSION,
                **bindings,
                "scenario_id": result.scenario_id,
                "source_scenario_id": result.source_scenario_id,
                "seed": result.seed,
                "metric_profile_identity": result.metric_profile_identity,
                "argv": list(result.argv),
                "process_exit_status": result.process_exit_status,
                "projection_identity": projection_identity,
                "members": members,
            }
            envelope_raw = _canonical(envelope)
            if output_authority is not None:
                output_authority.final_barrier(
                    execution.output_inventory_payload["output_tree_identity"],
                )
            _exclusive_sealed_file(staging / EVIDENCE_PROJECTION_PATH, projection_raw)
            _exclusive_sealed_file(staging / EVIDENCE_ENVELOPE_PATH, envelope_raw)
            _fsync_directory(staging)
            staging.chmod(0o555)
            _rename_noreplace(staging, final)
            published = True
            _fsync_directory(package_parent)
            package_identity = _physical_package_identity(final, projection_identity)
            observed = authenticate_slice_7g_cell_evidence_package(final, charter, ledger, plan)
            if (
                observed.projection_identity != projection_identity
                or observed.package_identity != package_identity
            ):
                _fail("evidence_package_identity", "writer output disagrees with governance authentication")
            return final, projection_identity, package_identity
        except Exception:
            if staging.exists():
                _make_tree_removable(staging)
                shutil.rmtree(staging)
            if published and final.exists() and not final.is_symlink():
                _make_tree_removable(final)
                shutil.rmtree(final)
                _fsync_directory(package_parent)
            raise
        finally:
            if output_authority is not None:
                output_authority.close()

    def write_campaign_seal(
        self,
        charter: Slice7GCharter,
        ledger: Slice7GAttemptLedger,
        plan: Slice7GCampaignPlan,
        package_identities: dict[str, str],
    ) -> Path:
        self._validate_context(charter, ledger, plan)
        if type(package_identities) is not dict:
            _fail("seal_package_bijection", "package identities must be an exact dictionary")
        try:
            detached_identities = dict(package_identities)
        except RuntimeError as exc:
            raise Slice7GRuntimeError(
                "seal_package_bijection", "package identity dictionary changed during detachment",
            ) from exc
        if set(detached_identities) != {cell.cell_id for cell in plan.cells}:
            _fail("seal_package_bijection", "seal requires exactly the 15 planned package identities")
        for identity in detached_identities.values():
            _digest(identity, "seal_package_identity")
        evidence_root = self.campaign_root / "evidence"
        packages_root = evidence_root / "packages"
        seal_path = evidence_root / "campaign_evidence_seal.json"
        plan_identity = slice_7g_campaign_plan_identity(plan)
        ledger_identity = slice_7g_attempt_ledger_identity(ledger)
        for cell in plan.cells:
            observed = authenticate_slice_7g_cell_evidence_package(
                packages_root / cell.cell_id, charter, ledger, plan,
            )
            if observed.package_identity != detached_identities[cell.cell_id]:
                _fail("seal_package_identity", "package identity changed before campaign seal", cell.cell_id)
        records = [
            {
                "schema_version": CAMPAIGN_EVIDENCE_PACKAGE_RECORD_SCHEMA_VERSION,
                "cell_id": cell.cell_id,
                "relative_path": f"packages/{cell.cell_id}",
                "package_identity": detached_identities[cell.cell_id],
            }
            for cell in plan.cells
        ]
        seal = validate_slice_7g_campaign_evidence_seal({
            "schema_version": CAMPAIGN_EVIDENCE_SEAL_SCHEMA_VERSION,
            "charter_logical_identity": slice_7g_charter_identity(charter),
            "campaign_identity": plan.campaign_identity,
            "campaign_plan_identity": plan_identity,
            "runtime_authorization_identity": ledger.runtime_authorization_identity,
            "attempt_ledger_identity": ledger_identity,
            "attempt_ledger_revision": ledger.revision,
            "process_start_event_identity": ledger.last_event_identity,
            "ros_domain_id": ledger.domain_id,
            "campaign_output_root": ledger.output_root,
            "evidence_root_relative_path": "evidence",
            "packages": records,
        })
        raw = canonical_slice_7g_campaign_evidence_seal_bytes(seal)
        descriptor = None
        locked = False
        original_modes = {
            packages_root: stat.S_IMODE(packages_root.stat().st_mode),
            evidence_root: stat.S_IMODE(evidence_root.stat().st_mode),
            self.campaign_root: stat.S_IMODE(self.campaign_root.stat().st_mode),
        }
        try:
            descriptor = os.open(seal_path, os.O_RDWR | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0), 0o600)
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
                locked = True
            except OSError as exc:
                raise Slice7GRuntimeError("evidence_seal_lock", str(exc), path=str(seal_path)) from exc
            _write_all(descriptor, raw)
            os.fsync(descriptor)
            os.fchmod(descriptor, 0o444)
            os.fsync(descriptor)
            packages_root.chmod(0o555)
            evidence_root.chmod(0o555)
            self.campaign_root.chmod(0o555)
            _fsync_directory(evidence_root)
            _fsync_directory(self.campaign_root)
            return seal_path
        except Exception:
            if descriptor is not None:
                try:
                    os.unlink(seal_path)
                except FileNotFoundError:
                    pass
            for path, mode in original_modes.items():
                try:
                    path.chmod(mode)
                except OSError:
                    pass
            raise
        finally:
            if descriptor is not None:
                try:
                    if locked:
                        fcntl.flock(descriptor, fcntl.LOCK_UN)
                finally:
                    os.close(descriptor)

    def _validate_context(
        self,
        charter: Slice7GCharter,
        ledger: Slice7GAttemptLedger,
        plan: Slice7GCampaignPlan,
    ) -> None:
        validate_slice_7g_campaign_plan(plan, charter, ledger)
        try:
            output_root = ledger.output_root
            process_start_committed = ledger.process_start_committed
            consumed_attempts = ledger.consumed_campaign_attempts
        except AttributeError as exc:
            raise Slice7GRuntimeError("evidence_ledger_record", "evidence ledger is partial") from exc
        if output_root != str(self.campaign_root):
            _fail("evidence_output_root_binding", "writer root differs from the committed ledger output root")
        if not process_start_committed or consumed_attempts != 1:
            _fail("evidence_ledger_uncommitted", "evidence requires the committed 1/1 process-start ledger")
        _require_real_directory(self.campaign_root, "campaign_root")


class ProductionSlice7GRunnerAdapter:
    """Convert retained evaluator artifacts into one governed cell execution.

    The adapter accepts only the immutable argv/environment selected by the
    coordinator.  It derives all result and evidence facts from the process
    observation and the physical output inventory; callers cannot inject a
    preauthenticated cell record.
    """

    def __init__(self, effects: Slice7GProductionEffects, *, timeout_seconds: float = 300.0) -> None:
        if not isinstance(effects, Slice7GProductionEffects):
            _fail("production_effects", "runner adapter requires repository-owned production effects")
        self.effects = effects
        self.timeout_seconds = _finite(timeout_seconds, "cell_timeout_seconds")
        if self.timeout_seconds <= 25.0:
            _fail("cell_timeout_seconds", "cell timeout must exceed the governed duration")
        self._context: tuple[Slice7GCharter, Slice7GAttemptLedger, Slice7GCampaignPlan] | None = None
        self._pending_authorities: dict[str, _CellOutputAuthority] = {}

    def bind_campaign(
        self, charter: Slice7GCharter, ledger: Slice7GAttemptLedger, plan: Slice7GCampaignPlan,
    ) -> None:
        validate_slice_7g_campaign_plan(plan, charter, ledger)
        if not ledger.process_start_committed or ledger.consumed_campaign_attempts != 1:
            _fail("runner_adapter_ledger", "runner adapter requires the committed 1/1 ledger")
        self._context = (charter, ledger, plan)

    def __call__(self, argv: tuple[str, ...], env: dict[str, str]) -> Slice7GCellExecution:
        if self._context is None:
            _fail("runner_adapter_unbound", "runner adapter must be bound after process-start commitment")
        charter, ledger, plan = self._context
        if type(argv) is not tuple or any(type(item) is not str for item in argv):
            _fail("runner_argv", "runner argv must be an exact string tuple")
        if type(env) is not dict or any(type(key) is not str or type(value) is not str for key, value in env.items()):
            _fail("runner_environment", "runner environment must be an exact string dictionary")
        cells = tuple(cell for cell in plan.cells if cell.argv == argv)
        if len(cells) != 1:
            _fail("runner_argv", "runner argv does not bind exactly one planned cell")
        cell = cells[0]
        _validate_runner_environment(env, cell, ledger, plan)
        try:
            observation = self.effects.run_cell(tuple(argv), dict(env), self.timeout_seconds)
        except Slice7GRuntimeError:
            raise
        except Exception as exc:
            raise Slice7GRuntimeError(
                "cell_process_provider_failed", "production process provider failed",
            ) from exc
        observation = _validated_process_observation(observation, tuple(argv), "cell_process_observation")
        _write_process_output_artifacts(cell.cell_output_path, observation)
        _finalize_cell_output_tree(cell.cell_output_path)
        return self._execution_from_artifacts(cell, plan, ledger, observation, env)

    def take_output_authority(self, execution: Slice7GCellExecution) -> _CellOutputAuthority:
        execution = _detached_execution(execution)
        cell_id = execution.cell_result.cell_id
        authority = self._pending_authorities.pop(cell_id, None)
        if type(authority) is not _CellOutputAuthority:
            _fail("cell_output_authority_missing", "runner adapter did not retain output authority for the cell")
        return authority

    def close_pending_authorities(self) -> None:
        authorities = tuple(self._pending_authorities.values())
        self._pending_authorities.clear()
        for authority in authorities:
            authority.close()

    def _execution_from_artifacts(
        self,
        cell: Slice7GCampaignCell,
        plan: Slice7GCampaignPlan,
        ledger: Slice7GAttemptLedger,
        observation: Slice7GProcessObservation,
        env: dict[str, str],
    ) -> Slice7GCellExecution:
        cell_root = Path(cell.cell_output_path)
        authority = _CellOutputAuthority(cell_root)
        retained = False
        try:
            receipt_raw = authority.member_bytes(RUNNER_RECEIPT_PATH)
            receipt = _parse_json(receipt_raw, "runner_receipt_json")
            if receipt_raw != _canonical(receipt):
                _fail("runner_receipt_noncanonical", "runner receipt is not canonical", RUNNER_RECEIPT_PATH)
            required = {
            "schema_version", "charter_logical_identity", "runtime_authorization_identity",
            "attempt_ledger_identity", "attempt_ledger_revision", "process_start_event_identity",
            "campaign_plan_identity", "domain_lease_identity", "domain_committed_binding_identity",
            "cell_id", "campaign_id", "campaign_output_root", "cell_output_root", "ros_domain_id",
            "task", "geometry", "scenario", "seed", "duration_seconds", "runtime_mode", "argv",
            "process_exit_status", "baseline_relative_path", "candidate_relative_path",
            }
            _closed(receipt, required, "runner_receipt_fields")
            if receipt["schema_version"] != RUNNER_RECEIPT_SCHEMA:
                _fail("runner_receipt_schema", "unsupported runner receipt schema")
            expected = {
            "charter_logical_identity": plan.charter_logical_identity,
            "runtime_authorization_identity": ledger.runtime_authorization_identity,
            "attempt_ledger_identity": slice_7g_attempt_ledger_identity(ledger),
            "attempt_ledger_revision": ledger.revision,
            "process_start_event_identity": ledger.last_event_identity,
            "campaign_plan_identity": slice_7g_campaign_plan_identity(plan),
            "cell_id": cell.cell_id,
            "campaign_id": plan.campaign_id,
            "campaign_output_root": cell.campaign_output_root,
            "cell_output_root": cell.cell_output_path,
            "ros_domain_id": cell.ros_domain_id,
            "task": cell.task,
            "geometry": cell.geometry_profile,
            "scenario": cell.source_scenario_id,
            "seed": cell.seed,
            "duration_seconds": cell.duration_seconds,
            "runtime_mode": cell.runtime_mode,
            "argv": list(cell.argv),
            "process_exit_status": observation.returncode,
            "domain_lease_identity": env["CTR_SLICE_7G_DOMAIN_LEASE_IDENTITY"],
            "domain_committed_binding_identity": env[
                "CTR_SLICE_7G_DOMAIN_COMMITTED_BINDING_IDENTITY"
            ],
            }
            for name, required_value in expected.items():
                if type(receipt[name]) is not type(required_value) or receipt[name] != required_value:
                    _fail("runner_receipt_binding", f"runner receipt field {name} differs from committed authority")
            _digest(receipt["domain_lease_identity"], "runner_domain_lease_identity")
            _digest(receipt["domain_committed_binding_identity"], "runner_domain_binding_identity")
            baseline_relative = _safe_relative(receipt["baseline_relative_path"], "runner_baseline_path")
            candidate_relative = _safe_relative(receipt["candidate_relative_path"], "runner_candidate_path")
            if baseline_relative == candidate_relative:
                _fail("runner_result_paths", "baseline and candidate directories must be distinct")
            authority.require_directory(baseline_relative)
            authority.require_directory(candidate_relative)
            authority.bind_candidate_semantic_directory(candidate_relative)
            required_paths = {
                f"{directory}/{name}"
                for directory in (baseline_relative, candidate_relative)
                for name in REQUIRED_RUN_ARTIFACTS
            }
            required_paths.update({
                RUNNER_RECEIPT_PATH, PROCESS_STDOUT_PATH, PROCESS_STDERR_PATH,
                PROCESS_OUTPUT_RECEIPT_PATH,
            })
            observed_paths = {record.path for record in authority.members}
            missing = sorted(required_paths - observed_paths)
            if missing:
                _fail(
                    "cell_output_required_file_missing",
                    "cell output is missing one or more required retained artifacts",
                    missing[0],
                )
            unexpected = sorted(
                path for path in observed_paths - required_paths
                if path != "ros_log" and not path.startswith("ros_log/")
            )
            if unexpected:
                _fail(
                    "cell_output_unexpected_file",
                    "cell output contains a file outside the closed runner-artifact or ROS-log namespaces",
                    unexpected[0],
                )
            _validate_process_output_receipt(authority, observation)
            summary = _parse_json(
                authority.member_bytes(f"{candidate_relative}/summary.json"), "runner_summary_json",
            )
            orchestration = _parse_json(
                authority.member_bytes(f"{candidate_relative}/orchestration.json"),
                "runner_orchestration_json",
            )
            summary = _detach(summary)
            summary["missing_required_result_file_count"] = 0
            readiness = _readiness_from_orchestration(orchestration)
            result = cell_result_from_summary(
                cell=cell, plan=plan, ledger=ledger, summary=summary,
                readiness=readiness, process_exit_status=observation.returncode,
            )
            invocation = {
                "argv": list(cell.argv),
                "process_exit_status": observation.returncode,
            }
            readiness_payload = {
            "readiness_success": readiness.passed,
            "stable_sample_count": readiness.stable_sample_count,
            "stable_interval_seconds": readiness.stable_interval_seconds,
            "q_variation": readiness.q_variation,
            "tip_variation_m": readiness.tip_variation_m,
            }
            safety_payload = {
            "minimum_physical_wall_clearance_m": result.minimum_physical_wall_clearance_m,
            "minimum_safety_margin_wall_clearance_m": result.minimum_safety_margin_wall_clearance_m,
            "collision_sample_count": result.collision_sample_count,
            "safety_fault_count": result.safety_fault_count,
            "nonfinite_value_count": result.nonfinite_value_count,
            }
            tactile_payload = {
            "valid_aligned_sample_count": result.valid_aligned_sample_count,
            "invalid_sample_count": result.invalid_sample_count,
            "invalid_sample_percentage": result.invalid_sample_percentage,
            "saturation_percentage": result.saturation_percentage,
            "missing_required_topic_count": result.missing_required_topic_count,
            }
            output_payload = authority.inventory_payload(0)
            execution = Slice7GCellExecution(
                result, invocation, readiness_payload, safety_payload, tactile_payload, output_payload,
            )
            execution = _detached_execution(execution)
            _validate_execution_payloads(execution)
            if result.cell_id in self._pending_authorities:
                _fail("cell_output_authority_duplicate", "runner adapter already retains authority for the cell")
            self._pending_authorities[result.cell_id] = authority
            retained = True
            return execution
        finally:
            if not retained:
                authority.close()


class Slice7GCampaignCoordinator:
    """One-attempt coordinator; all effects are explicit injected dependencies."""

    def __init__(
        self,
        *,
        charter_path: str | os.PathLike[str],
        authorization_path: str | os.PathLike[str],
        ledger_writer: AtomicSlice7GLedgerWriter,
        domain_allocator: Slice7GDomainAllocator,
        output_allocator: Callable[[Slice7GRuntimeAuthorization], str],
        preflight: Callable[[Slice7GRuntimeAuthorization], None],
        process_factory: Callable[[tuple[str, ...], dict[str, str]], Slice7GCellExecution],
        evidence_writer: Slice7GEvidenceWriter,
        timestamp_factory: Callable[[], str],
        production_domain_authority: ProductionSlice7GDomainAuthority | None = None,
        production_root_authority: _ProductionRootAuthority | None = None,
        production_required: bool = False,
        _historical_test_authority: bool = True,
    ) -> None:
        self.charter_path = charter_path
        self.authorization_path = authorization_path
        self.ledger_writer = ledger_writer
        self.domain_allocator = domain_allocator
        if not callable(output_allocator):
            _fail("output_allocator", "output allocator must be callable")
        self.output_allocator = output_allocator
        self.preflight = preflight
        self.process_factory = process_factory
        self.evidence_writer = evidence_writer
        self.timestamp_factory = timestamp_factory
        if type(production_required) is not bool or type(_historical_test_authority) is not bool:
            _fail("production_coordinator", "production_required must be an exact bool")
        if production_required and (
            type(production_domain_authority) is not ProductionSlice7GDomainAuthority
            or type(process_factory) is not ProductionSlice7GRunnerAdapter
            or type(production_root_authority) is not _ProductionRootAuthority
        ):
            _fail(
                "production_coordinator",
                "production assembly requires repository-owned domain authority and runner adapter",
            )
        self.production_domain_authority = production_domain_authority
        self.production_root_authority = production_root_authority
        self.production_required = production_required
        self._historical_test_authority = _historical_test_authority

    def run(self) -> Any:
        charter = load_slice_7g_charter(self.charter_path)
        authorization = (
            _load_slice_7g_runtime_authorization_v1_for_test(self.authorization_path, charter)
            if self._historical_test_authority
            else load_slice_7g_runtime_authorization(self.authorization_path, charter)
        )
        if str(self.evidence_writer.campaign_root) != authorization.campaign_output_root:
            _fail("coordinator_output_root_binding", "evidence writer differs from runtime authorization")
        lease: Slice7GDomainLease | None = None
        primary: Exception | None = None
        result: Any = None
        try:
            self.preflight(authorization)
            if self.production_required:
                registry_metadata = self.production_root_authority.prepare_global_registry()
                self.production_domain_authority.adopt_prepared_registry(registry_metadata)
            initial = create_slice_7g_initial_attempt_ledger(charter, authorization.campaign_id)
            lease = self.domain_allocator.allocate(charter, authorization, self.timestamp_factory())
            allocated_root = self.output_allocator(authorization)
            if type(allocated_root) is not str or allocated_root != authorization.campaign_output_root:
                _fail("output_allocation_binding", "allocated output root differs from runtime authorization")
            allocation_event = propose_slice_7g_attempt_event(
                initial, "domain_and_output_allocated", "domain-output-allocation",
                self.timestamp_factory(), domain_id=lease.domain_id,
                output_root=allocated_root,
                runtime_authorization_identity=authorization.identity,
            )
            projected_allocated = validate_slice_7g_attempt_transition(initial, allocation_event)
            plan = generate_slice_7g_campaign_plan(charter, projected_allocated)
            validate_slice_7g_campaign_plan(plan, charter, projected_allocated)
            self.ledger_writer.initialize(initial)
            allocated = self.ledger_writer.commit(initial, allocation_event)
            if slice_7g_attempt_ledger_identity(allocated) != slice_7g_attempt_ledger_identity(projected_allocated):
                _fail("ledger_projection_mismatch", "durable allocation differs from the validated plan predecessor")
            domain_binding = bind_domain_lease(lease, allocated)
            self.ledger_writer.commit_domain_binding(domain_binding)
            validate_slice_7g_campaign_plan(plan, charter, allocated)
            start_event = propose_slice_7g_attempt_event(
                allocated, "process_start_commit", "campaign-process-start",
                self.timestamp_factory(), campaign_plan=plan,
            )
            committed = self.ledger_writer.commit(allocated, start_event, campaign_plan=plan)
            committed_domain_binding_identity = domain_binding.identity
            if self.production_required:
                final_occupancy = self.production_domain_authority.observe(lease.domain_id)
                self.ledger_writer.commit_final_domain_observation(committed, lease, final_occupancy)
                if not final_occupancy.collision_free:
                    _fail(
                        "domain_occupancy_changed_after_commit",
                        "domain became occupied or uncertain after the 1/1 attempt commit",
                    )
                committed_domain_binding_identity = self.production_domain_authority.commit_process_binding(
                    lease, charter, committed, plan, final_occupancy,
                )
                self.process_factory.bind_campaign(charter, committed, plan)
            package_identities: dict[str, str] = {}
            for cell in plan.cells:
                env = {
                    "ROS_DOMAIN_ID": str(lease.domain_id),
                    "ROS_DISTRO": "humble",
                    "CTR_SLICE_7G_CHARTER_IDENTITY": slice_7g_charter_identity(charter),
                    "CTR_SLICE_7G_RUNTIME_AUTHORIZATION_IDENTITY": authorization.identity,
                    "CTR_SLICE_7G_ATTEMPT_LEDGER_IDENTITY": slice_7g_attempt_ledger_identity(committed),
                    "CTR_SLICE_7G_ATTEMPT_LEDGER_REVISION": str(committed.revision),
                    "CTR_SLICE_7G_PROCESS_START_EVENT_IDENTITY": committed.last_event_identity,
                    "CTR_SLICE_7G_CAMPAIGN_PLAN_IDENTITY": slice_7g_campaign_plan_identity(plan),
                    "CTR_SLICE_7G_DOMAIN_LEASE_IDENTITY": lease.identity,
                    "CTR_SLICE_7G_DOMAIN_COMMITTED_BINDING_IDENTITY": committed_domain_binding_identity,
                    "CTR_SLICE_7G_CELL_ID": cell.cell_id,
                    "CTR_SLICE_7G_CAMPAIGN_ID": plan.campaign_id,
                    "CTR_SLICE_7G_CAMPAIGN_OUTPUT_ROOT": cell.campaign_output_root,
                    "CTR_SLICE_7G_CELL_OUTPUT_ROOT": cell.cell_output_path,
                    "ROS_LOG_DIR": f"{cell.cell_output_path}/ros_log",
                }
                execution = _detached_execution(self.process_factory(cell.argv, env))
                _validate_execution_context(execution, cell, plan, committed)
                if self.production_required:
                    output_authority = self.process_factory.take_output_authority(execution)
                    try:
                        _, _, package_identity = self.evidence_writer._write_authenticated_cell_package(
                            execution, charter, committed, plan, output_authority,
                        )
                    except Exception:
                        output_authority.close()
                        raise
                else:
                    _, _, package_identity = self.evidence_writer.write_cell_package(
                        execution, charter, committed, plan,
                    )
                package_identities[cell.cell_id] = package_identity
            self.evidence_writer.write_campaign_seal(charter, committed, plan, package_identities)
            result = reconcile_slice_7g_campaign_results(
                charter, plan, committed, authorization.campaign_output_root,
            )
        except Exception as exc:
            primary = exc
        cleanup_issues: list[Slice7GCleanupIssue] = []
        cleanup_errors: list[Exception] = []
        if self.production_required:
            try:
                self.process_factory.close_pending_authorities()
            except Exception as exc:
                cleanup_errors.append(exc)
                cleanup_issues.append(_cleanup_issue("cell_output_authority_cleanup_failed", exc))
        if lease is not None:
            try:
                release = self.domain_allocator.release(lease, self.timestamp_factory())
                if self.production_required:
                    self.ledger_writer.commit_domain_release(release)
            except Exception as exc:
                cleanup_errors.append(exc)
                cleanup_issues.append(_cleanup_issue("domain_release_cleanup_failed", exc))
        if self.production_required:
            try:
                self.production_root_authority.close()
            except Exception as exc:
                cleanup_errors.append(exc)
                cleanup_issues.append(_cleanup_issue("root_authority_cleanup_failed", exc))
        cleanup_primary = primary if primary is not None else (cleanup_errors[0] if cleanup_errors else None)
        if cleanup_primary is not None and cleanup_issues and self.ledger_writer.parent.is_dir():
            try:
                self.ledger_writer.commit_cleanup_failure(cleanup_primary, tuple(cleanup_issues))
            except Exception as exc:
                cleanup_issues.append(_cleanup_issue("cleanup_ledger_record_failed", exc))
        if primary is not None:
            if cleanup_issues:
                raise Slice7GCoordinatedFailure(primary, tuple(cleanup_issues)) from primary
            raise primary
        if cleanup_errors:
            first = cleanup_errors[0]
            secondary = tuple(cleanup_issues[1:])
            if secondary:
                raise Slice7GCoordinatedFailure(first, secondary) from first
            raise first
        return result


class ProductionSlice7GPreflight:
    """Repository-owned non-consuming production preflight."""

    def __init__(
        self,
        effects: Slice7GProductionEffects,
        control_root: Path,
        charter_path: str,
        root_authority: _ProductionRootAuthority,
    ) -> None:
        self.effects = effects
        self.control_root = control_root
        self.charter_path = charter_path
        self.root_authority = root_authority

    def __call__(self, authorization: Slice7GRuntimeAuthorization) -> None:
        authorization = _validated_runtime_authorization(authorization)
        if self.effects.which("ctr_run_evaluation") is None:
            _fail("preflight_runner_missing", "ctr_run_evaluation is not installed or discoverable")
        if self.effects.which("ros2") is None:
            _fail("preflight_domain_observer_missing", "the bounded ROS graph observer is not installed")
        try:
            process_probe = self.effects.active_process_records()
            socket_probe = self.effects.udp_socket_tables()
        except Slice7GRuntimeError:
            raise
        except Exception as exc:
            raise Slice7GRuntimeError(
                "preflight_domain_provider_failed", "a local domain-observation provider is unavailable",
            ) from exc
        if type(process_probe) is not tuple or any(type(item) is not dict for item in process_probe):
            _fail("preflight_domain_provider_failed", "active-process provider returned malformed data")
        if (
            type(socket_probe) is not tuple or len(socket_probe) != 2
            or any(type(item) is not bytes for item in socket_probe)
        ):
            _fail("preflight_domain_provider_failed", "DDS socket provider returned malformed data")
        observed_charter = load_slice_7g_charter(self.charter_path)
        if slice_7g_charter_identity(observed_charter) != authorization.charter_logical_identity:
            _fail("preflight_charter_identity", "charter changed after production assembly")
        self.root_authority.preflight()
        if self.control_root.exists() or self.control_root.is_symlink():
            _fail("production_control_exists", "campaign control root must be new", str(self.control_root))


def _assemble_slice_7g_production_coordinator(
    charter_path: str | os.PathLike[str],
    authorization_path: str | os.PathLike[str],
    effects: Slice7GProductionEffects,
    *,
    _historical_test_authority: bool = True,
) -> Slice7GCampaignCoordinator:
    """Private lowest-level-effects seam for deterministic source tests."""

    charter_text = _path(charter_path, "charter_path")
    authorization_text = _path(authorization_path, "runtime_authorization_path")
    charter = load_slice_7g_charter(charter_text)
    authorization = (
        _load_slice_7g_runtime_authorization_v1_for_test(authorization_text, charter)
        if _historical_test_authority
        else load_slice_7g_runtime_authorization(authorization_text, charter)
    )
    if not isinstance(effects, Slice7GProductionEffects):
        _fail("production_effects", "test effects must implement the repository-owned raw-effect interface")
    output = Path(authorization.campaign_output_root)
    root_authority = _ProductionRootAuthority(charter, authorization)
    assembled = False
    try:
        control_root = output.with_name(f".{output.name}.slice_7g_control")
        ledger_root = control_root / "attempt_ledger"
        lease_root = Path(root_authority.external_parent) / GLOBAL_DOMAIN_LEASE_REGISTRY_NAME
        authority = ProductionSlice7GDomainAuthority(lease_root, effects, defer_prepare=True)
        allocator = Slice7GDomainAllocator(authority.observe, authority.acquire, authority.release)
        writer = AtomicSlice7GLedgerWriter(ledger_root)
        adapter = ProductionSlice7GRunnerAdapter(effects)
        coordinator = Slice7GCampaignCoordinator(
            charter_path=charter_text,
            authorization_path=authorization_text,
            ledger_writer=writer,
            domain_allocator=allocator,
            output_allocator=root_authority.allocate_output_root,
            preflight=ProductionSlice7GPreflight(effects, control_root, charter_text, root_authority),
            process_factory=adapter,
            evidence_writer=Slice7GEvidenceWriter(authorization.campaign_output_root),
            timestamp_factory=effects.utc_now,
            production_domain_authority=authority,
            production_root_authority=root_authority,
            production_required=True,
            _historical_test_authority=_historical_test_authority,
        )
        assembled = True
        return coordinator
    finally:
        if not assembled:
            root_authority.close()


def assemble_slice_7g_production_coordinator(
    charter_path: str | os.PathLike[str], authorization_path: str | os.PathLike[str],
) -> Slice7GCampaignCoordinator:
    """Assemble production authority internally; no caller providers accepted."""

    return _assemble_slice_7g_production_coordinator(
        charter_path, authorization_path, Slice7GProductionEffects(),
        _historical_test_authority=False,
    )


def run_slice_7g_production_campaign(
    charter_path: str | os.PathLike[str], authorization_path: str | os.PathLike[str],
) -> Any:
    """Run the governed campaign; callable only after separate runtime authorization."""

    return assemble_slice_7g_production_coordinator(charter_path, authorization_path).run()


@dataclass(frozen=True)
class Slice7GAuthorityTransactionBinding:
    authorization_identity: str | None = None
    prepare_token: str | None = None
    campaign_id: str | None = None
    campaign_identity: str | None = None
    campaign_template_identity: str | None = None
    observation_session_identity: str | None = None
    observation_session_nonce: str | None = None
    four_source_observation_identity: str | None = None
    precommit_receipt_identities: tuple[str, ...] = ()
    precommit_observer_count: int = 0
    postcommit_observer_count: int = 0
    transaction_observer_count: int = 0
    lease_identity: str | None = None
    domain_id: int | None = None
    output_root_path: str | None = None
    output_root_identity: str | None = None
    process_manifest_identity: str | None = None
    process_instance_identity: str | None = None


class Slice7GAuthorityTransaction:
    """Prepare/allocate/commit barrier for the fixed OS authority service.

    The public production factory accepts no providers.  The underscored test
    factory is the only callback seam and cannot be reached from the CLI.
    """

    _TOKEN = object()

    def __init__(
        self,
        *,
        session_factory: Callable[[], Any],
        nonconsuming_preflight: Callable[[Slice7GAuthorityTransactionBinding], Slice7GObservationResult],
        process_instance_builder: Callable[[Slice7GAuthorityTransactionBinding], tuple[str, str]],
        postcommit_domain_recheck: Callable[
            [Slice7GAuthorityTransactionBinding], Slice7GPostcommitObservationResult
        ],
        execute_committed_campaign: Callable[[Slice7GAuthorityTransactionBinding, Any], Any],
        cleanup: Callable[[Slice7GAuthorityTransactionBinding | None, bool], None],
        timestamp_factory: Callable[[], str],
        _token: object | None = None,
    ) -> None:
        if _token is not self._TOKEN:
            _fail("authority_transaction_factory", "authority transaction requires an internal factory")
        callbacks = (
            session_factory, nonconsuming_preflight, process_instance_builder,
            postcommit_domain_recheck, execute_committed_campaign, cleanup, timestamp_factory,
        )
        if any(not callable(callback) for callback in callbacks):
            _fail("authority_transaction_callback", "authority transaction callback is not callable")
        self._session_factory = session_factory
        self._nonconsuming_preflight = nonconsuming_preflight
        self._process_instance_builder = process_instance_builder
        self._postcommit_domain_recheck = postcommit_domain_recheck
        self._execute_committed_campaign = execute_committed_campaign
        self._cleanup = cleanup
        self._timestamp_factory = timestamp_factory

    @classmethod
    def _for_test(cls, **kwargs: Any) -> "Slice7GAuthorityTransaction":
        return cls(_token=cls._TOKEN, **kwargs)

    def run(self) -> Any:
        binding: Slice7GAuthorityTransactionBinding | None = None
        committed = False
        primary: BaseException | None = None
        result: Any = None
        cleanup_issues: list[tuple[str, str]] = []
        session: Any = None
        try:
            session = self._session_factory()
            with session:
                try:
                    started = session.exchange(_authority_request(
                        "begin_observation", self._timestamp_factory(),
                    ))
                    binding = _binding_from_observation_start(started)
                    selected = False
                    for domain in range(DOMAIN_MINIMUM, DOMAIN_MAXIMUM + 1):
                        recorded = session.exchange(_authority_request(
                            "record_precommit_observation", self._timestamp_factory(),
                            binding=binding, domain_id=domain,
                        ))
                        if (
                            recorded.data["result"] != "OBSERVATION_RECORDED"
                            or recorded.data["domain_id"] != domain
                        ):
                            _fail("authority_observation", "authority did not record its observation")
                        binding = _binding_from_server_observation(binding, recorded)
                        if recorded.data["candidate_clear"] is True:
                            selected = True
                            break
                    if not selected:
                        _fail("domain_unavailable", "authority found no free domain in 100 through 199")
                    finalized = session.exchange(_authority_request(
                        "finalize_observation", self._timestamp_factory(), binding=binding,
                    ))
                    if finalized.data["result"] != "OBSERVATION_COMPLETE":
                        _fail("authority_observation", "authority did not finalize four-source observation")
                    binding = _binding_from_server_observation(binding, finalized)
                    prepared = session.exchange(_authority_request(
                        "prepare", self._timestamp_factory(), binding=binding,
                    ))
                    binding = _binding_from_prepare(binding, prepared)
                    domain = binding.domain_id
                    allocation = session.exchange(_authority_request(
                        "allocate_provisional", self._timestamp_factory(), binding=binding,
                        domain_id=domain,
                    ))
                    binding = _binding_from_allocation(binding, allocation, domain)
                    process_manifest_identity, process_instance_identity = self._process_instance_builder(binding)
                    _digest(process_manifest_identity, "process_manifest_identity")
                    _digest(process_instance_identity, "process_instance_identity")
                    binding = replace(
                        binding,
                        process_manifest_identity=process_manifest_identity,
                        process_instance_identity=process_instance_identity,
                    )
                    committed_receipt = session.exchange(_authority_request(
                        "commit", self._timestamp_factory(), binding=binding,
                    ))
                    if committed_receipt.data["result"] != "COMMITTED":
                        _fail("authority_commit", "authority did not return a durable COMMITTED receipt")
                    committed = True
                    postcommit_receipt = session.exchange(_authority_request(
                        "record_postcommit_observation", self._timestamp_factory(),
                        binding=binding,
                    ))
                    if postcommit_receipt.data["result"] != "POSTCOMMIT_RECORDED":
                        _fail("authority_postcommit", "authority did not retain postcommit observation")
                    binding = replace(
                        binding, postcommit_observer_count=1,
                        transaction_observer_count=binding.precommit_observer_count + 1,
                    )
                    result = self._execute_committed_campaign(binding, committed_receipt)
                    completed = session.exchange(_authority_request(
                        "complete", self._timestamp_factory(), binding=binding,
                    ))
                    if completed.data["result"] != "COMPLETED":
                        _fail("authority_complete", "authority did not finalize the global budget")
                except BaseException as exc:
                    primary = exc
                    if binding is not None and not committed:
                        try:
                            session.exchange(_authority_request(
                                "cancel", self._timestamp_factory(), binding=binding,
                            ))
                        except BaseException as cleanup_exc:
                            cleanup_issues.append(("authority_cancel", type(cleanup_exc).__name__))
                    elif binding is not None and committed:
                        try:
                            session.exchange(_authority_request(
                                "fail_after_commit", self._timestamp_factory(), binding=binding,
                            ))
                        except BaseException as cleanup_exc:
                            cleanup_issues.append(("authority_fail_after_commit", type(cleanup_exc).__name__))
        except BaseException as exc:
            if primary is None:
                primary = exc
        finally:
            try:
                self._cleanup(binding, committed)
            except BaseException as cleanup_exc:
                cleanup_issues.append(("campaign_cleanup", type(cleanup_exc).__name__))
        if primary is not None:
            if cleanup_issues:
                try:
                    primary.add_note(f"Slice 7G deterministic cleanup issues: {cleanup_issues!r}")
                except (AttributeError, TypeError):
                    pass
            raise primary
        if cleanup_issues:
            _fail("authority_cleanup", f"authority cleanup failed: {cleanup_issues!r}")
        return result


def _authority_request(
    method: str,
    timestamp: str,
    *,
    binding: Slice7GAuthorityTransactionBinding | None = None,
    domain_id: int | None = None,
) -> dict[str, Any]:
    request_id = "r" + hashlib.sha256(
        f"{method}\0{timestamp}\0{os.getpid()}\0{time_monotonic_ns()}".encode("utf-8")
    ).hexdigest()[:32]
    return {
        "schema_version": AUTHORITY_REQUEST_SCHEMA,
        "method": method,
        "request_id": request_id,
        "authorization_identity": None if binding is None else binding.authorization_identity,
        "prepare_token": None if binding is None else binding.prepare_token,
        "campaign_id": None if binding is None else binding.campaign_id,
        "campaign_identity": None if binding is None else binding.campaign_identity,
        "campaign_template_identity": None if binding is None else binding.campaign_template_identity,
        "domain_id": domain_id if domain_id is not None else (None if binding is None else binding.domain_id),
        "output_root_path": None if binding is None else binding.output_root_path,
        "output_root_identity": None if binding is None else binding.output_root_identity,
        "process_manifest_identity": None if binding is None else binding.process_manifest_identity,
        "process_instance_identity": None if binding is None else binding.process_instance_identity,
        "observation_session_identity": None if binding is None else binding.observation_session_identity,
        "observation_session_nonce": None if binding is None else binding.observation_session_nonce,
        "requested_at_utc": timestamp,
    }


def _binding_from_observation_start(receipt: Any) -> Slice7GAuthorityTransactionBinding:
    data = receipt.data
    if data["result"] != "OBSERVATION_STARTED":
        _fail("authority_observation", "authority did not open an observation session")
    for field in (
        "authorization_identity", "observation_session_identity", "observation_session_nonce",
    ):
        if data[field] is None:
            _fail("authority_observation", f"observation receipt lacks {field}")
    return Slice7GAuthorityTransactionBinding(
        authorization_identity=data["authorization_identity"],
        observation_session_identity=data["observation_session_identity"],
        observation_session_nonce=data["observation_session_nonce"],
    )


def _binding_from_server_observation(
    before: Slice7GAuthorityTransactionBinding, receipt: Any,
) -> Slice7GAuthorityTransactionBinding:
    data = receipt.data
    identities = tuple(data["precommit_receipt_identities"])
    count = data["precommit_observer_count"]
    total = data["transaction_observer_count"]
    if (
        data["observation_session_identity"] != before.observation_session_identity
        or data["observation_session_nonce"] != before.observation_session_nonce
        or len(identities) != count
        or total != count + data["postcommit_observer_count"]
    ):
        _fail("authority_observation", "server-owned observation receipt is inconsistent")
    return replace(
        before,
        domain_id=data["domain_id"],
        four_source_observation_identity=data["four_source_observation_identity"],
        precommit_receipt_identities=identities,
        precommit_observer_count=count,
        postcommit_observer_count=data["postcommit_observer_count"],
        transaction_observer_count=total,
        lease_identity=data["lease_identity"],
    )


def _validated_observation_result(
    value: Any, binding: Slice7GAuthorityTransactionBinding,
) -> Slice7GObservationResult:
    if type(value) is not Slice7GObservationResult:
        _fail("authority_observation", "preflight returned an unsupported observation result")
    if (
        value.observation_session_identity != binding.observation_session_identity
        or value.observation_session_nonce != binding.observation_session_nonce
        or type(value.domain_id) is not int
        or not DOMAIN_MINIMUM <= value.domain_id <= DOMAIN_MAXIMUM
        or not 1 <= len(value.precommit_receipts) <= MAX_PRECOMMIT_OBSERVERS
        or len(value.precommit_receipts) != len(value.precommit_receipt_identities)
        or type(value.lease_identity) is not str
    ):
        _fail("authority_observation", "preflight observation binding is malformed")
    _digest(value.lease_identity, "lease_identity")
    receipts: list[dict[str, Any]] = []
    domains: list[int] = []
    identities: list[str] = []
    for index, supplied in enumerate(value.precommit_receipts):
        record = validate_authority_record(
            supplied, expected_schema=ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA,
        )
        if record.data["phase"] != "PRECOMMIT":
            _fail("authority_observation", "preflight contains a postcommit receipt")
        if record.logical_identity != value.precommit_receipt_identities[index]:
            _fail("authority_observation", "preflight receipt identity differs")
        domains.append(record.data["domain_id"])
        identities.append(record.logical_identity)
        receipts.append(json.loads(record.canonical_bytes))
    if domains != sorted(set(domains)) or domains[-1] != value.domain_id:
        _fail("authority_observation", "preflight candidate-domain order differs")
    four = validate_authority_record(
        value.four_source_observation, expected_schema=FOUR_SOURCE_OBSERVATION_SCHEMA,
    )
    if (
        four.logical_identity != value.four_source_observation_identity
        or four.data["phase"] != "PRECOMMIT"
        or four.data["observation_session_identity"] != binding.observation_session_identity
        or four.data["domain_id"] != value.domain_id
        or four.data["ros_graph_observation_identity"] != identities[-1]
        or receipts[-1]["nodes"]
    ):
        _fail("authority_observation", "complete four-source observation differs")
    return Slice7GObservationResult(
        value.domain_id, value.observation_session_identity, value.observation_session_nonce,
        json.loads(four.canonical_bytes), four.logical_identity, tuple(receipts),
        tuple(identities), value.lease_identity,
    )


def _validated_postcommit_result(
    value: Any, binding: Slice7GAuthorityTransactionBinding,
) -> Slice7GPostcommitObservationResult:
    if type(value) is not Slice7GPostcommitObservationResult:
        _fail("authority_postcommit", "postcommit recheck returned an unsupported result")
    receipt = validate_authority_record(
        value.ros_graph_observation_receipt,
        expected_schema=ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA,
    )
    four = validate_authority_record(
        value.four_source_observation, expected_schema=FOUR_SOURCE_OBSERVATION_SCHEMA,
    )
    if (
        receipt.logical_identity != value.ros_graph_observation_identity
        or receipt.data["phase"] != "POSTCOMMIT"
        or receipt.data["domain_id"] != binding.domain_id
        or receipt.data["nodes"]
        or four.logical_identity != value.four_source_observation_identity
        or four.data["phase"] != "POSTCOMMIT"
        or four.data["observation_session_identity"] != binding.observation_session_identity
        or four.data["domain_id"] != binding.domain_id
        or four.data["ros_graph_observation_identity"] != receipt.logical_identity
    ):
        _fail("authority_postcommit", "postcommit observation binding differs")
    return Slice7GPostcommitObservationResult(
        json.loads(receipt.canonical_bytes), receipt.logical_identity,
        json.loads(four.canonical_bytes), four.logical_identity,
    )


def _binding_from_prepare(
    before: Slice7GAuthorityTransactionBinding, receipt: Any,
) -> Slice7GAuthorityTransactionBinding:
    data = receipt.data
    if data["result"] != "PREPARED":
        _fail("authority_prepare", "authority did not prepare the campaign")
    for field in (
        "authorization_identity", "prepare_token", "campaign_id", "campaign_identity",
        "campaign_template_identity",
    ):
        if data[field] is None:
            _fail("authority_prepare", f"prepare receipt lacks {field}")
    if (
        data["authorization_identity"] != before.authorization_identity
        or data["observation_session_identity"] != before.observation_session_identity
        or data["four_source_observation_identity"] != before.four_source_observation_identity
        or tuple(data["precommit_receipt_identities"]) != before.precommit_receipt_identities
        or data["precommit_observer_count"] != before.precommit_observer_count
        or data["domain_id"] != before.domain_id
        or data["lease_identity"] != before.lease_identity
    ):
        _fail("authority_prepare", "prepare receipt observation binding differs")
    return replace(
        before,
        prepare_token=data["prepare_token"], campaign_id=data["campaign_id"],
        campaign_identity=data["campaign_identity"],
        campaign_template_identity=data["campaign_template_identity"],
    )


def _binding_from_allocation(
    before: Slice7GAuthorityTransactionBinding, receipt: Any, domain_id: int,
) -> Slice7GAuthorityTransactionBinding:
    data = receipt.data
    if data["result"] != "PREPARED" or data["output_root_path"] is None or data["output_root_identity"] is None:
        _fail("authority_allocation", "authority did not return a provisional output allocation")
    expected = (
        before.authorization_identity, before.prepare_token, before.campaign_id,
        before.campaign_identity, before.campaign_template_identity,
    )
    observed = (
        data["authorization_identity"], data["prepare_token"], data["campaign_id"],
        data["campaign_identity"], data["campaign_template_identity"],
    )
    if observed != expected or data["domain_id"] != domain_id:
        _fail("authority_allocation", "provisional allocation binding differs")
    return replace(
        before, domain_id=domain_id, output_root_path=data["output_root_path"],
        output_root_identity=data["output_root_identity"],
    )


def time_monotonic_ns() -> int:
    # Kept as a small source-owned seam for stable request-ID tests; it has no
    # authority meaning and never enters a durable campaign identity.
    import time
    return time.monotonic_ns()


def main(argv: list[str] | None = None) -> int:
    arguments = [] if argv is None else argv
    if type(arguments) is not list or arguments or any(type(item) is not str for item in arguments):
        print("ctr_run_slice_7g_campaign failed: caller arguments are prohibited", file=sys.stderr)
        return 2
    # Phase A supplies the fixed protocol and transaction engine.  The future
    # installed-runtime approval binds the concrete process manifest before
    # this factory becomes runnable; source-tree execution is always rejected.
    print(
        "ctr_run_slice_7g_campaign failed: missing_authority: provisioned "
        "charter-v5 OS authority and installed-runtime approval are required",
        file=sys.stderr,
    )
    return 2


def cell_result_from_summary(
    *, cell: Slice7GCampaignCell, plan: Slice7GCampaignPlan, ledger: Slice7GAttemptLedger,
    summary: dict[str, Any], readiness: Slice7GReadinessResult, process_exit_status: int,
) -> Slice7GCellResult:
    """Translate retained evaluation summary fields into the governance result schema."""

    if type(cell) is not Slice7GCampaignCell or type(plan) is not Slice7GCampaignPlan:
        _fail("cell_summary_context", "cell and plan must be exact governance records")
    if type(ledger) is not Slice7GAttemptLedger:
        _fail("cell_summary_context", "ledger must be an exact governance record")
    if type(summary) is not dict:
        _fail("cell_summary_type", "summary must be an exact dictionary")
    readiness = _validated_readiness_result(readiness)
    tracking = _section(summary, "tracking")
    goal = _section(summary, "goal")
    control = _section(summary, "control")
    timing = _section(summary, "timing")
    numerical = _section(summary, "numerical_safety")
    quality = _section(summary, "data_quality")
    lumen = _section(summary, "lumen_evaluation")
    physical = _section(lumen, "physical_safety")
    safety_margin = _section(lumen, "safety_margin")
    safety_runtime = _section(summary, "slice_7g_safety")
    tactile_runtime = _section(summary, "slice_7g_tactile")
    if "missing_required_result_file_count" not in summary:
        _fail(
            "cell_summary_missing_results",
            "Slice 7G summaries must explicitly account for every required result file",
        )
    missing_results = _exact_nonnegative_int(
        summary["missing_required_result_file_count"],
        "missing_results",
    )
    invalid = _exact_nonnegative_int(
        _required_field(quality, "rejected_aligned_sample_count", "data_quality"),
        "invalid_samples",
    )
    valid = _exact_nonnegative_int(
        _required_field(quality, "valid_aligned_sample_count", "data_quality"),
        "valid_samples",
    )
    invalid_percentage = 0.0 if valid + invalid == 0 else 100.0 * invalid / (valid + invalid)
    deadline = _finite(
        _required_field(timing, "deadline_overrun_percentage", "timing"),
        "deadline_overrun_percentage",
    )
    result = Slice7GCellResult(
        "ctr-slice-7g-cell-result-2", cell.cell_id, plan.charter_logical_identity,
        plan.campaign_identity, slice_7g_campaign_plan_identity(plan),
        slice_7g_attempt_ledger_identity(ledger), ledger.revision,
        ledger.last_event_identity, ledger.runtime_authorization_identity, plan.metric_profile_identity,
        cell.scenario_id, cell.source_scenario_id, cell.seed, cell.duration_seconds,
        cell.runtime_mode, cell.ros_domain_id, cell.campaign_output_root, cell.cell_output_path,
        cell.argv, _exact_nonnegative_int(process_exit_status, "process_exit_status"),
        readiness.passed, readiness.stable_sample_count, readiness.stable_interval_seconds,
        readiness.q_variation, readiness.tip_variation_m, valid, invalid, invalid_percentage,
        _finite(_required_field(tracking, "steady_state_error", "tracking"), "steady_state_error"),
        _finite(_required_field(goal, "final_goal_error", "goal"), "final_goal_error"),
        _finite(_required_field(goal, "goal_hold_duration", "goal"), "goal_hold_duration"),
        _finite(
            _required_field(physical, "minimum_physical_clearance_m", "lumen_evaluation.physical_safety"),
            "physical_clearance",
        ),
        _finite(
            _required_field(safety_margin, "minimum_safety_clearance_m", "lumen_evaluation.safety_margin"),
            "safety_clearance",
        ),
        _exact_nonnegative_int(
            _required_field(physical, "collision_sample_count", "lumen_evaluation.physical_safety"),
            "collision_count",
        ),
        _exact_nonnegative_int(
            _required_field(safety_runtime, "fault_count", "slice_7g_safety"),
            "safety_fault_count",
        ),
        sum(_exact_nonnegative_int(_required_field(numerical, name, "numerical_safety"), name) for name in (
            "nonfinite_state_samples", "nonfinite_reference_samples", "nonfinite_command_samples",
        )) + _exact_nonnegative_int(
            _required_field(tactile_runtime, "invalid_sample_count", "slice_7g_tactile"),
            "tactile_invalid_samples",
        ),
        _exact_nonnegative_int(
            _required_field(numerical, "missing_required_topic_count", "numerical_safety"),
            "missing_topics",
        ),
        missing_results,
        _finite(_required_field(control, "saturation_percentage", "control"), "saturation_percentage"),
        deadline, deadline <= 5.0, deadline > 5.0,
    )
    return result


def bind_domain_lease(
    lease: Slice7GDomainLease, allocated_ledger: Slice7GAttemptLedger,
) -> Slice7GDomainBinding:
    if type(allocated_ledger) is not Slice7GAttemptLedger:
        _fail("domain_binding_type", "binding requires exact lease and ledger records")
    lease = _validated_domain_lease(lease)
    try:
        ledger_identity = slice_7g_attempt_ledger_identity(allocated_ledger)
        domain_id = allocated_ledger.domain_id
        authorization_identity = allocated_ledger.runtime_authorization_identity
        campaign_identity = allocated_ledger.campaign_identity
        process_start_committed = allocated_ledger.process_start_committed
        revision = allocated_ledger.revision
        output_root = allocated_ledger.output_root
    except Slice7GGovernanceError as exc:
        raise Slice7GRuntimeError("domain_binding_record", str(exc)) from exc
    if (
        domain_id != lease.domain_id
        or authorization_identity != lease.runtime_authorization_identity
        or campaign_identity != lease.campaign_identity
        or process_start_committed
    ):
        _fail("domain_binding_context", "lease differs from the allocated pre-start ledger")
    data = {
        "schema_version": DOMAIN_BINDING_SCHEMA,
        "lease_identity": lease.identity,
        "runtime_authorization_identity": lease.runtime_authorization_identity,
        "campaign_identity": lease.campaign_identity,
        "attempt_ledger_identity": ledger_identity,
        "attempt_ledger_revision": revision,
        "domain_id": lease.domain_id,
        "output_root": output_root,
    }
    identity = hashlib.sha256(DOMAIN_BINDING_DOMAIN + _canonical(data)).hexdigest()
    return Slice7GDomainBinding(identity=identity, **data)


def _source_snapshot_relative(value: Any) -> str:
    if type(value) is not str:
        _fail("source_snapshot_member_schema", "source snapshot member path must be an exact string")
    if not value or value != unicodedata.normalize("NFC", value):
        _fail("source_snapshot_member_schema", "source snapshot member path must be nonempty NFC")
    if value.startswith("/") or "\\" in value or any(ord(char) < 32 or ord(char) == 127 for char in value):
        _fail("source_snapshot_member_schema", "source snapshot member path is unsafe", value)
    parts = value.split("/")
    if any(not part or part in {".", ".."} for part in parts):
        _fail("source_snapshot_member_schema", "source snapshot member path is unsafe", value)
    return value


def _source_snapshot_mode(value: Any) -> int:
    if type(value) is not int or value < 0 or value > 0o7777:
        _fail(
            "source_snapshot_member_mode",
            "source snapshot member mode must be an exact integer in 0..0o7777",
        )
    return value


def _validated_source_snapshot_member(value: Any) -> Slice7GSourceSnapshotMember:
    if type(value) is not Slice7GSourceSnapshotMember:
        _fail("source_snapshot_member_schema", "source snapshot member must be an exact record")
    try:
        return Slice7GSourceSnapshotMember(value.path, value.mode, value.size, value.sha256)
    except AttributeError as exc:
        raise Slice7GRuntimeError(
            "source_snapshot_member_schema", "source snapshot member is partially initialized",
        ) from exc


def _validated_source_snapshot(value: Any) -> Slice7GPostImplementationSourceSnapshot:
    if type(value) is not Slice7GPostImplementationSourceSnapshot:
        _fail("source_snapshot_schema", "source snapshot must be an exact v2 record")
    try:
        schema_version = value.schema_version
        supplied_members = value.members
    except AttributeError as exc:
        raise Slice7GRuntimeError("source_snapshot_schema", "source snapshot is partially initialized") from exc
    if type(supplied_members) is not tuple:
        _fail("source_snapshot_member_schema", "source snapshot members must be an exact tuple")
    members = tuple(_validated_source_snapshot_member(member) for member in supplied_members)
    return Slice7GPostImplementationSourceSnapshot(schema_version, members)


def _source_snapshot_payload(snapshot: Slice7GPostImplementationSourceSnapshot) -> dict[str, Any]:
    snapshot = _validated_source_snapshot(snapshot)
    return {
        "schema_version": snapshot.schema_version,
        "members": [
            {"path": member.path, "mode": member.mode, "size": member.size, "sha256": member.sha256}
            for member in snapshot.members
        ],
    }


def canonical_post_implementation_source_snapshot_bytes(
    snapshot: Slice7GPostImplementationSourceSnapshot,
) -> bytes:
    """Return canonical compact bytes for one closed structural v2 value."""

    return _canonical(_source_snapshot_payload(snapshot))


def post_implementation_source_snapshot_identity(
    snapshot: Slice7GPostImplementationSourceSnapshot,
) -> str:
    return hashlib.sha256(
        POST_IMPLEMENTATION_SNAPSHOT_DOMAIN
        + canonical_post_implementation_source_snapshot_bytes(snapshot)
    ).hexdigest()


def _source_snapshot_member_from_mapping(value: Any) -> Slice7GSourceSnapshotMember:
    if type(value) is not dict:
        _fail("source_snapshot_member_schema", "source snapshot member must be an exact object")
    _closed(value, {"path", "mode", "size", "sha256"}, "source_snapshot_member_schema")
    return Slice7GSourceSnapshotMember(
        value["path"], value["mode"], value["size"], value["sha256"],
    )


def _source_snapshot_from_mapping(value: Any) -> Slice7GPostImplementationSourceSnapshot:
    if type(value) is not dict:
        _fail("source_snapshot_schema", "source snapshot must be an exact object")
    _closed(value, {"schema_version", "members"}, "source_snapshot_schema")
    if value["schema_version"] == POST_IMPLEMENTATION_SNAPSHOT_V1_SCHEMA:
        _fail(
            "source_snapshot_schema_not_build_authoritative",
            "v1 source snapshots do not bind member modes and are historical only",
        )
    if value["schema_version"] != POST_IMPLEMENTATION_SNAPSHOT_SCHEMA:
        _fail("source_snapshot_schema", "unsupported post-implementation source snapshot schema")
    if type(value["members"]) is not list or not value["members"]:
        _fail("source_snapshot_member_schema", "source snapshot members must be a nonempty array")
    return Slice7GPostImplementationSourceSnapshot(
        value["schema_version"],
        tuple(_source_snapshot_member_from_mapping(member) for member in value["members"]),
    )


def parse_post_implementation_source_snapshot(
    raw: bytes,
) -> Slice7GPostImplementationSourceSnapshot:
    """Parse structural v2 bytes; repository verification establishes authority."""

    if type(raw) is not bytes:
        _fail("source_snapshot_schema", "source snapshot bytes must be exact bytes")
    value = _parse_json(raw, "source_snapshot_schema")
    snapshot = _source_snapshot_from_mapping(value)
    if canonical_post_implementation_source_snapshot_bytes(snapshot) != raw:
        _fail("source_snapshot_noncanonical", "source snapshot bytes are not canonical")
    return snapshot


def _historical_v1_snapshot_identity(value: Any, raw: bytes) -> tuple[int, str]:
    if type(value) is not dict:
        _fail("source_snapshot_schema", "historical source snapshot must be an exact object")
    _closed(value, {"schema_version", "members"}, "source_snapshot_schema")
    if value["schema_version"] != POST_IMPLEMENTATION_SNAPSHOT_V1_SCHEMA:
        _fail("source_snapshot_schema", "unsupported historical source snapshot schema")
    if type(value["members"]) is not list or not value["members"]:
        _fail("source_snapshot_member_schema", "historical source snapshot members must be nonempty")
    paths: list[str] = []
    for member in value["members"]:
        if type(member) is not dict:
            _fail("source_snapshot_member_schema", "historical member must be an exact object")
        _closed(member, {"path", "size", "sha256"}, "source_snapshot_member_schema")
        paths.append(_source_snapshot_relative(member["path"]))
        if type(member["size"]) is not int or member["size"] < 0:
            _fail("source_snapshot_member_schema", "historical member size is invalid")
        if type(member["sha256"]) is not str or not DIGEST.fullmatch(member["sha256"]):
            _fail("source_snapshot_member_schema", "historical member digest is invalid")
    if paths != sorted(paths) or len(paths) != len(set(paths)):
        _fail("source_snapshot_member_schema", "historical members must be ordered and unique")
    if _canonical(value) != raw:
        _fail("source_snapshot_noncanonical", "historical source snapshot bytes are not canonical")
    return len(paths), hashlib.sha256(POST_IMPLEMENTATION_SNAPSHOT_V1_DOMAIN + raw).hexdigest()


def inspect_post_implementation_source_snapshot(raw: bytes) -> Slice7GSourceSnapshotInspection:
    """Inspect exact v1 historical or v2 current bytes without upgrading either version."""

    if type(raw) is not bytes:
        _fail("source_snapshot_schema", "source snapshot bytes must be exact bytes")
    value = _parse_json(raw, "source_snapshot_schema")
    schema_version = value.get("schema_version") if type(value) is dict else None
    if schema_version == POST_IMPLEMENTATION_SNAPSHOT_V1_SCHEMA:
        count, identity = _historical_v1_snapshot_identity(value, raw)
        authoritative = False
    elif schema_version == POST_IMPLEMENTATION_SNAPSHOT_SCHEMA:
        snapshot = _source_snapshot_from_mapping(value)
        canonical = canonical_post_implementation_source_snapshot_bytes(snapshot)
        if canonical != raw:
            _fail("source_snapshot_noncanonical", "source snapshot bytes are not canonical")
        count = len(snapshot.members)
        identity = post_implementation_source_snapshot_identity(snapshot)
        authoritative = False
    else:
        _fail("source_snapshot_schema", "unsupported post-implementation source snapshot schema")
    return Slice7GSourceSnapshotInspection(
        schema_version, count, hashlib.sha256(raw).hexdigest(), identity, authoritative,
    )


@dataclass(frozen=True)
class _SourceTreeMetadata:
    """Full continuity authority for the repository root and source tree."""

    device: int
    inode: int
    file_type: int
    mode: int
    link_count: int
    size: int
    mtime_ns: int
    ctime_ns: int

    @property
    def physical_identity(self) -> tuple[int, int]:
        return self.device, self.inode


@dataclass(frozen=True)
class _AncestorComponentIdentity:
    """Path-component authority unaffected by unrelated directory entries."""

    device: int
    inode: int
    file_type: int
    mode: int


@dataclass(frozen=True)
class _AncestorPathBinding:
    """One retained ancestor and its descriptor-relative next component."""

    component: _AncestorComponentIdentity
    next_component_name: str
    next_component: _AncestorComponentIdentity


@dataclass(frozen=True)
class _RepositoryPathBaseline:
    """Separate ancestor path identity from full repository-root metadata."""

    ancestors: tuple[_AncestorPathBinding, ...]
    root: _SourceTreeMetadata


def _source_tree_metadata(value: os.stat_result) -> _SourceTreeMetadata:
    return _SourceTreeMetadata(
        value.st_dev,
        value.st_ino,
        stat.S_IFMT(value.st_mode),
        stat.S_IMODE(value.st_mode),
        value.st_nlink,
        value.st_size,
        value.st_mtime_ns,
        value.st_ctime_ns,
    )


def _ancestor_component_identity(value: os.stat_result) -> _AncestorComponentIdentity:
    return _AncestorComponentIdentity(
        value.st_dev,
        value.st_ino,
        stat.S_IFMT(value.st_mode),
        stat.S_IMODE(value.st_mode),
    )


def _ancestor_identity_from_source(
    value: _SourceTreeMetadata,
) -> _AncestorComponentIdentity:
    return _AncestorComponentIdentity(
        value.device, value.inode, value.file_type, value.mode,
    )


@dataclass(frozen=True)
class _SourceSnapshotMemberObservation:
    path: str
    metadata: _SourceTreeMetadata
    sha256: str
    record: Slice7GSourceSnapshotMember


@dataclass(frozen=True)
class _SourceSnapshotDirectoryObservation:
    path: str
    metadata: _SourceTreeMetadata


@dataclass(frozen=True)
class _SourceSnapshotProvisionalInventory:
    """Private, non-authoritative full-tree facts retained across watch bootstrap."""

    phase: str
    directories: tuple[_SourceSnapshotDirectoryObservation, ...]
    members: tuple[_SourceSnapshotMemberObservation, ...]

    @property
    def directory_paths(self) -> tuple[str, ...]:
        return tuple(item.path for item in self.directories)

    @property
    def member_paths(self) -> tuple[str, ...]:
        return tuple(item.path for item in self.members)


@dataclass(frozen=True)
class _SourceSnapshotTraversalFrame:
    path: str
    descriptor: int
    names: tuple[str, ...]
    next_index: int
    baseline: _SourceTreeMetadata
    owns_descriptor: bool


@dataclass(frozen=True)
class _RepositoryOwnedDescriptor:
    """One descriptor governed by an explicit, single-attempt close lifecycle."""

    descriptor: int
    label: str
    resource_kind: str
    ownership_order: int
    lifecycle_state: str
    cleanup_issue: Slice7GCleanupIssue | None = None


@dataclass(frozen=True)
class _RepositoryWatch:
    """Private interpretation authority for one installed inotify watch."""

    scope: str
    expected_child_name: str | None


class _RepositorySnapshotAuthority:
    """Own repository authority from provisional capture through final barrier.

    Provisional continuity begins when the descriptor-confined full-tree
    inventory captures each directory/member fact.  Complete watch coverage
    begins only after every provisionally discovered source directory has a
    live inotify watch.  The first authoritative baseline is accepted only
    after setup-event drain and a second full metadata/digest inventory agrees
    exactly with the provisional inventory.  Before a directory watch exists,
    retained metadata and digests detect change/restore; after installation,
    both inotify and full reconciliation enforce continuity.
    """

    _ROOT_FILES = ("README.md", "CODEX_TASK.md", "CURRENT_STATUS.md")
    _ROOT_DIRECTORIES = ("config", "docs", "src")
    _EXCLUDED_DIRECTORIES = frozenset({"__pycache__", ".pytest_cache", "build", "install", "log"})
    _EXCLUDED_FILE_SUFFIXES = (".pyc", ".pyo")
    _MAX_SOURCE_DIRECTORIES = 2_048
    _MAX_SOURCE_MEMBERS = 8_192
    _MAX_SOURCE_DEPTH = 64
    _IN_MODIFY = 0x00000002
    _IN_ATTRIB = 0x00000004
    _IN_CLOSE_WRITE = 0x00000008
    _IN_MOVED_FROM = 0x00000040
    _IN_MOVED_TO = 0x00000080
    _IN_CREATE = 0x00000100
    _IN_DELETE = 0x00000200
    _IN_DELETE_SELF = 0x00000400
    _IN_MOVE_SELF = 0x00000800
    _IN_UNMOUNT = 0x00002000
    _IN_Q_OVERFLOW = 0x00004000
    _IN_IGNORED = 0x00008000
    _IN_ISDIR = 0x40000000
    _INOTIFY_EVENT_HEADER_BYTES = 16
    _INOTIFY_NAME_ALIGNMENT = 16
    _INOTIFY_MAX_DRAIN_BYTES = 1_048_576
    _INOTIFY_MAX_DRAIN_EVENTS = 8_192
    _INOTIFY_MAX_DRAIN_READS = 8_192
    _DIRECTORY_CHANGE_MASK = (
        _IN_MODIFY | _IN_ATTRIB | _IN_CLOSE_WRITE | _IN_MOVED_FROM | _IN_MOVED_TO
        | _IN_CREATE | _IN_DELETE | _IN_DELETE_SELF | _IN_MOVE_SELF
    )
    _CHAIN_CHANGE_MASK = (
        _IN_ATTRIB | _IN_MOVED_FROM | _IN_MOVED_TO | _IN_CREATE | _IN_DELETE
        | _IN_DELETE_SELF | _IN_MOVE_SELF
    )
    _DESCRIPTOR_PROVISIONAL = "provisional_owned"
    _DESCRIPTOR_AUTHORITY = "authority_owned"
    _DESCRIPTOR_CLOSE_INVOKED = "close_invoked"
    _DESCRIPTOR_CLOSED = "closed"
    _DESCRIPTOR_TERMINAL_AMBIGUITY = "terminal_ambiguity"

    def __init__(self, repository_path: str) -> None:
        if type(repository_path) is not str:
            _fail("snapshot_root", "private repository authority requires a detached exact path")
        self.repository_path = repository_path
        # The lifecycle container exists before the first descriptor acquisition.
        # A descriptor returned by an opener is registered here before metadata,
        # watch, parser, or caller-controlled work can observe it.
        self._closed = False
        self._cleanup_started = False
        self._cleanup_active = False
        self._begun = False
        self._finalized = False
        self._chain_descriptors: list[int] = []
        self._chain_metadata: _RepositoryPathBaseline | tuple[()] = ()
        self._monitor_descriptor: int | None = None
        self._monitor_scopes: dict[int, _RepositoryWatch] = {}
        self._watched_directories: dict[tuple[int, int], int] = {}
        self._source_watch_identities: set[tuple[int, int]] = set()
        self._owned_descriptors: dict[int, _RepositoryOwnedDescriptor] = {}
        self._next_ownership_order = 0
        self._chain_components = tuple(repository_path[1:].split("/"))
        self._complete_member_paths: tuple[str, ...] = ()
        self._selected_member_paths: tuple[str, ...] = ()
        self._directory_baselines: dict[str, _SourceSnapshotDirectoryObservation] = {}
        self._member_baselines: dict[str, _SourceSnapshotMemberObservation] = {}
        self._provisional_inventory: _SourceSnapshotProvisionalInventory | None = None
        self._bootstrap_phase = "chain_provisional"
        try:
            descriptors, provisional_path = self._open_initial_chain()
            self._root_descriptor = descriptors[-1]
            self._monitor_descriptor = self._acquire_descriptor(
                self._open_change_monitor,
                "change-monitor",
                "monitor",
                "source_snapshot_monitor",
            )
            try:
                self._monitor_fstat(self._monitor_descriptor)
            except Slice7GRuntimeError:
                raise
            except Exception as exc:
                raise Slice7GRuntimeError(
                    "source_snapshot_monitor", "repository change monitor stat failed",
                ) from exc
            for index, descriptor in enumerate(descriptors):
                if index == len(descriptors) - 1:
                    scope = "root"
                elif index == len(descriptors) - 2:
                    scope = "root_parent"
                else:
                    scope = "parent"
                mask = self._DIRECTORY_CHANGE_MASK if scope == "root" else self._CHAIN_CHANGE_MASK
                expected_child = (
                    self._chain_components[index]
                    if scope in {"parent", "root_parent"}
                    else None
                )
                if index == len(descriptors) - 1:
                    before: _SourceTreeMetadata | _AncestorComponentIdentity = (
                        self._source_tree_fstat(descriptor, self.repository_path)
                    )
                    expected_before: _SourceTreeMetadata | _AncestorComponentIdentity = (
                        provisional_path.root
                    )
                else:
                    before = self._ancestor_fstat(descriptor, self.repository_path)
                    expected_before = provisional_path.ancestors[index].component
                if before != expected_before:
                    self._fail_chain_setup(index, "repository pathname metadata changed before watch setup")
                self._watch_directory(descriptor, before, scope, mask, expected_child)
                after = (
                    self._source_tree_fstat(descriptor, self.repository_path)
                    if index == len(descriptors) - 1
                    else self._ancestor_fstat(descriptor, self.repository_path)
                )
                if after != before:
                    self._fail_chain_setup(index, "repository pathname metadata changed during watch setup")
                if index < len(descriptors) - 1:
                    self._reconcile_retained_ancestor(
                        index, descriptors, provisional_path.ancestors[index],
                    )
                self._transfer_descriptor(descriptor)
                self._chain_descriptors.append(descriptor)
            self._assert_no_monitored_changes()
            self._reconcile_chain_path(provisional_path, check_metadata=True)

            # Source continuity starts with a complete descriptor-authenticated
            # provisional inventory.  It is intentionally not an authoritative
            # v2 result.  Every source-directory watch is then installed and the
            # complete tree is reauthenticated before any authoritative path or
            # source baseline is accepted.
            self._bootstrap_phase = "provisional_capture"
            provisional_inventory = self._capture_bootstrap_inventory(
                require_watched=False,
            )
            self._provisional_inventory = provisional_inventory
            self._bootstrap_phase = "provisional_captured"
            self._install_complete_source_watch_set(provisional_inventory)
            self._bootstrap_phase = "source_watches_installed"
            self._after_complete_source_watch_set()
            self._assert_no_monitored_changes()
            self._bootstrap_phase = "post_watch_reconciliation"
            post_watch_inventory = self._capture_bootstrap_inventory(
                require_watched=True,
            )
            self._assert_no_monitored_changes()
            self._compare_bootstrap_inventories(
                provisional_inventory,
                post_watch_inventory,
            )
            self._reconcile_chain_path(provisional_path, check_metadata=True)
            self._assert_no_monitored_changes()

            authoritative_path = self._capture_path_baseline(descriptors)
            self._compare_path_baselines(
                provisional_path,
                authoritative_path,
                "repository pathname metadata changed before baseline",
            )
            self._assert_no_monitored_changes()
            self._chain_metadata = authoritative_path
            self._root_baseline = authoritative_path.root
            self._bootstrap_phase = "complete"
        except BaseException as primary:
            issues, _ = self._cleanup_owned_descriptors()
            _raise_repository_cleanup(primary, issues)

    @property
    def complete_member_paths(self) -> tuple[str, ...]:
        self._require_live()
        if not self._begun:
            _fail("source_snapshot_authority_state", "source membership has not been authenticated")
        return self._complete_member_paths

    def begin(
        self,
        *,
        selected_paths: tuple[str, ...] | None = None,
        expected: tuple[Slice7GSourceSnapshotMember, ...] | None = None,
    ) -> tuple[Slice7GSourceSnapshotMember, ...]:
        self._require_live()
        if self._begun:
            _fail("source_snapshot_authority_state", "source authority was already initialized")
        provisional = self._provisional_inventory
        if (
            self._bootstrap_phase != "complete"
            or type(provisional) is not _SourceSnapshotProvisionalInventory
        ):
            _fail(
                "source_snapshot_authority_state",
                "complete prebaseline watch bootstrap has not succeeded",
            )
        self._assert_no_monitored_changes()
        complete_paths, directories = self._discover_complete_membership()
        self._assert_no_monitored_changes()
        provisional_directories = {item.path: item for item in provisional.directories}
        if (
            complete_paths != provisional.member_paths
            or set(directories) != set(provisional_directories)
        ):
            _fail(
                "source_snapshot_member_changed",
                "source membership changed after watch bootstrap",
            )
        for path, observation in directories.items():
            if observation != provisional_directories[path]:
                _fail(
                    "source_snapshot_member_changed",
                    "source directory changed after watch bootstrap",
                    path or self.repository_path,
                )
        if not complete_paths:
            _fail("snapshot_members", "post-implementation snapshot cannot be empty")
        if selected_paths is None:
            selected = complete_paths
        else:
            selected = tuple(_source_snapshot_relative(path) for path in selected_paths)
            if selected != tuple(sorted(selected)) or len(selected) != len(set(selected)):
                _fail("source_snapshot_member_schema", "structural member paths must be ordered and unique")
            if not set(selected).issubset(complete_paths):
                _fail(
                    "source_snapshot_membership_mismatch",
                    "structural member selection is outside module-owned discovery",
                )
        supplied: tuple[Slice7GSourceSnapshotMember, ...] | None = None
        if expected is not None:
            supplied = tuple(_validated_source_snapshot_member(member) for member in expected)
            supplied_paths = tuple(member.path for member in supplied)
            if selected_paths is not None or supplied_paths != complete_paths:
                _fail(
                    "source_snapshot_membership_mismatch",
                    "snapshot paths differ from independently discovered complete membership",
                )
        self._complete_member_paths = complete_paths
        self._selected_member_paths = selected
        self._directory_baselines = directories
        observations = self._authenticate_initial_members(selected)
        provisional_members = {item.path: item for item in provisional.members}
        for observation in observations:
            if observation != provisional_members.get(observation.path):
                _fail(
                    "source_snapshot_member_changed",
                    "source member changed after watch bootstrap",
                    observation.path,
                )
        records = tuple(observation.record for observation in observations)
        if supplied is not None:
            for actual, claimed in zip(records, supplied):
                if actual.mode != claimed.mode:
                    _fail("source_snapshot_mode_mismatch", "source member mode differs", actual.path)
                if actual != claimed:
                    _fail(
                        "source_snapshot_member_mismatch",
                        "source member bytes or metadata differ",
                        actual.path,
                    )
        self._member_baselines = {observation.path: observation for observation in observations}
        self._assert_no_monitored_changes()
        self._begun = True
        return records

    def final_barrier(self) -> None:
        self._require_live()
        if not self._begun or self._finalized:
            _fail("source_snapshot_authority_state", "source authority is not ready for its final barrier")
        self._reconcile_public_path(check_metadata=False)
        self._assert_no_monitored_changes()
        self._authenticate_final_members()
        self._assert_no_monitored_changes()
        final_paths, final_directories = self._discover_complete_membership()
        if final_paths != self._complete_member_paths or set(final_directories) != set(self._directory_baselines):
            _fail(
                "source_snapshot_membership_mismatch",
                "complete source membership changed before the final barrier",
            )
        for path, baseline in self._directory_baselines.items():
            if final_directories[path].metadata != baseline.metadata:
                _fail(
                    "source_snapshot_member_changed",
                    "source directory metadata changed before the final barrier",
                    path or self.repository_path,
                )
        self._assert_no_monitored_changes()
        self._reconcile_public_path(check_metadata=True)
        self._assert_no_monitored_changes()
        self._finalized = True

    def close(self) -> tuple[Slice7GCleanupIssue, ...]:
        """Attempt every definitely owned close once, then quarantine ambiguity."""

        issues, pending_base = self._cleanup_owned_descriptors()
        if pending_base is not None:
            try:
                setattr(pending_base, "cleanup_issues", issues)
            except Exception:
                pass
            raise pending_base
        return issues

    @property
    def terminally_ambiguous_descriptors(self) -> tuple[_RepositoryOwnedDescriptor, ...]:
        """Immutable quarantine records; their numeric descriptors are never retried."""

        return tuple(
            sorted(
                (
                    item for item in self._owned_descriptors.values()
                    if item.lifecycle_state == self._DESCRIPTOR_TERMINAL_AMBIGUITY
                ),
                key=lambda item: item.ownership_order,
            ),
        )

    @property
    def descriptor_cleanup_status(self) -> str:
        if not getattr(self, "_closed", True):
            return "definitely_owned_resources_remain"
        if self.terminally_ambiguous_descriptors:
            return "completed_with_terminal_ambiguity"
        return "all_safely_discharged"

    def _acquire_descriptor(
        self,
        opener: Callable[[], int],
        label: str,
        resource_kind: str,
        error_code: str,
    ) -> int:
        """Open and register locally owned numeric authority before metadata access."""

        descriptor: int | None = None
        try:
            descriptor = opener()
            if type(descriptor) is not int or descriptor < 0:
                _fail(error_code, "descriptor provider returned an invalid descriptor", label)
            if descriptor in self._owned_descriptors:
                _fail("source_snapshot_cleanup", "descriptor ownership was registered twice", label)
            self._owned_descriptors[descriptor] = _RepositoryOwnedDescriptor(
                descriptor,
                label,
                resource_kind,
                self._next_ownership_order,
                self._DESCRIPTOR_PROVISIONAL,
            )
            self._next_ownership_order += 1
            return descriptor
        except BaseException as primary:
            if descriptor is not None and descriptor not in self._owned_descriptors:
                issue, pending = self._close_unregistered_descriptor_once(
                    descriptor, label, resource_kind,
                )
                if issue is not None:
                    _raise_repository_cleanup(primary, (issue,))
                if pending is not None:
                    _raise_repository_cleanup(primary, (
                        Slice7GCleanupIssue(
                            "source_snapshot_descriptor_close",
                            f"{label}:terminal_ambiguity:{type(pending).__name__}",
                        ),
                    ))
            if isinstance(primary, Exception) and not isinstance(primary, Slice7GRuntimeError):
                raise Slice7GRuntimeError(
                    error_code, "descriptor acquisition failed", path=label,
                ) from primary
            raise

    def _close_unregistered_descriptor_once(
        self, descriptor: int, label: str, resource_kind: str,
    ) -> tuple[Slice7GCleanupIssue | None, BaseException | None]:
        """Discharge the tiny pre-registration window without an unsafe retry."""

        try:
            os.close(descriptor)
        except BaseException as error:
            issue = self._descriptor_close_issue(label, resource_kind, error)
            return issue, error if not isinstance(error, Exception) else None
        return None, None

    def _transfer_descriptor(self, descriptor: int) -> None:
        resource = self._owned_descriptors.get(descriptor)
        if resource is None or resource.lifecycle_state != self._DESCRIPTOR_PROVISIONAL:
            _fail("source_snapshot_cleanup", "descriptor ownership transfer is invalid")
        self._owned_descriptors[descriptor] = replace(
            resource, lifecycle_state=self._DESCRIPTOR_AUTHORITY,
        )

    @staticmethod
    def _descriptor_close_issue(
        label: str, resource_kind: str, error: BaseException,
    ) -> Slice7GCleanupIssue:
        try:
            detail = str(error)
        except BaseException:
            detail = "<unprintable>"
        return Slice7GCleanupIssue(
            "source_snapshot_descriptor_close",
            f"{resource_kind}:{label}:close:terminal_ambiguity:{type(error).__name__}:{detail}",
        )

    def _attempt_owned_descriptor_close(
        self, resource: _RepositoryOwnedDescriptor,
    ) -> tuple[tuple[Slice7GCleanupIssue, ...], BaseException | None]:
        current = self._owned_descriptors.get(resource.descriptor)
        if current != resource:
            return (), None
        if current.lifecycle_state in {
            self._DESCRIPTOR_CLOSED,
            self._DESCRIPTOR_TERMINAL_AMBIGUITY,
            self._DESCRIPTOR_CLOSE_INVOKED,
        }:
            return (), None
        if current.lifecycle_state not in {
            self._DESCRIPTOR_PROVISIONAL, self._DESCRIPTOR_AUTHORITY,
        }:
            issue = Slice7GCleanupIssue(
                "source_snapshot_descriptor_state",
                f"{current.label}:invalid lifecycle state",
            )
            return (issue,), None

        # Transition before invoking close.  Reentrant cleanup therefore cannot
        # issue a second close for the same numeric descriptor.
        invoked = replace(current, lifecycle_state=self._DESCRIPTOR_CLOSE_INVOKED)
        self._owned_descriptors[current.descriptor] = invoked
        try:
            os.close(current.descriptor)
        except BaseException as error:
            issue = self._descriptor_close_issue(
                current.label, current.resource_kind, error,
            )
            self._owned_descriptors[current.descriptor] = replace(
                invoked,
                lifecycle_state=self._DESCRIPTOR_TERMINAL_AMBIGUITY,
                cleanup_issue=issue,
            )
            pending = error if not isinstance(error, Exception) else None
            return (issue,), pending
        self._owned_descriptors.pop(current.descriptor, None)
        return (), None

    def _ordered_owned_descriptors(self) -> tuple[_RepositoryOwnedDescriptor, ...]:
        owned = tuple(self._owned_descriptors.values())
        monitor = sorted(
            (item for item in owned if item.resource_kind == "monitor"),
            key=lambda item: item.ownership_order,
        )
        transient = sorted(
            (item for item in owned if item.resource_kind == "transient"),
            key=lambda item: item.ownership_order,
            reverse=True,
        )
        chain = sorted(
            (item for item in owned if item.resource_kind == "chain"),
            key=lambda item: item.ownership_order,
            reverse=True,
        )
        return tuple((*monitor, *transient, *chain))

    def _close_descriptor_sequence(
        self, resources: Sequence[_RepositoryOwnedDescriptor],
    ) -> tuple[tuple[Slice7GCleanupIssue, ...], BaseException | None]:
        issues: list[Slice7GCleanupIssue] = []
        pending_base: BaseException | None = None
        already_active = self._cleanup_active
        self._cleanup_active = True
        try:
            for resource in resources:
                observed, pending = self._attempt_owned_descriptor_close(resource)
                issues.extend(observed)
                if pending_base is None and pending is not None:
                    pending_base = pending
        finally:
            self._cleanup_active = already_active
        return tuple(issues), pending_base

    def _cleanup_owned_descriptors(
        self,
    ) -> tuple[tuple[Slice7GCleanupIssue, ...], BaseException | None]:
        if getattr(self, "_closed", True):
            return (), None
        if getattr(self, "_cleanup_active", False):
            return (), None
        self._cleanup_started = True
        self._cleanup_active = True
        try:
            issues, pending_base = self._close_descriptor_sequence(
                self._ordered_owned_descriptors(),
            )
        finally:
            self._cleanup_active = False
        remaining_definite = any(
            item.lifecycle_state in {
                self._DESCRIPTOR_PROVISIONAL,
                self._DESCRIPTOR_AUTHORITY,
                self._DESCRIPTOR_CLOSE_INVOKED,
            }
            for item in self._owned_descriptors.values()
        )
        if not remaining_definite:
            self._closed = True
            self._monitor_descriptor = None
            self._chain_descriptors = []
        return issues, pending_base

    def _release_transient_descriptor(
        self,
        descriptor: int,
        primary: BaseException | None = None,
    ) -> None:
        self._release_transient_descriptors((descriptor,), primary)

    def _release_transient_descriptors(
        self,
        descriptors: Sequence[int],
        primary: BaseException | None = None,
    ) -> None:
        resources = tuple(
            resource
            for descriptor in descriptors
            if (resource := self._owned_descriptors.get(descriptor)) is not None
        )
        issues, pending_base = self._close_descriptor_sequence(resources)
        if primary is not None:
            _raise_repository_cleanup(primary, issues)
        if pending_base is not None:
            try:
                setattr(pending_base, "cleanup_issues", issues)
            except Exception:
                pass
            raise pending_base
        if issues:
            _raise_repository_cleanup(
                Slice7GRuntimeError(
                    "source_snapshot_cleanup", "repository transient descriptor cleanup failed",
                ),
                issues,
            )

    def _require_live(self) -> None:
        if getattr(self, "_closed", True) or getattr(self, "_cleanup_started", False):
            _fail("source_snapshot_authority_closed", "repository snapshot authority is closed")

    def _fail_chain_setup(self, index: int, detail: str) -> None:
        code = "source_snapshot_root_replaced" if index == len(self._chain_components) else "source_snapshot_parent_replaced"
        _fail(code, detail, self.repository_path)

    def _stat_chain_component(
        self,
        parent_descriptor: int,
        name: str,
        index: int,
    ) -> os.stat_result:
        code = (
            "source_snapshot_root_replaced"
            if index == len(self._chain_components) - 1
            else "source_snapshot_parent_replaced"
        )
        try:
            return os.stat(name, dir_fd=parent_descriptor, follow_symlinks=False)
        except BaseException as exc:
            if not isinstance(exc, Exception):
                raise
            raise Slice7GRuntimeError(
                code,
                "repository pathname component is unavailable",
                path=self.repository_path,
            ) from exc

    def _capture_path_baseline(
        self,
        descriptors: Sequence[int],
    ) -> _RepositoryPathBaseline:
        if len(descriptors) != len(self._chain_components) + 1:
            _fail("source_snapshot_parent_replaced", "repository pathname chain is incomplete")
        ancestors: list[_AncestorPathBinding] = []
        for index, parent_descriptor in enumerate(descriptors[:-1]):
            next_name = _safe_component(self._chain_components[index], "snapshot_root")
            component = self._ancestor_fstat(parent_descriptor, self.repository_path)
            next_descriptor = self._ancestor_fstat(descriptors[index + 1], self.repository_path)
            next_entry = _ancestor_component_identity(
                self._stat_chain_component(parent_descriptor, next_name, index),
            )
            if next_entry != next_descriptor:
                self._fail_chain_setup(
                    index + 1,
                    "repository pathname entry differs from its retained descriptor",
                )
            ancestors.append(
                _AncestorPathBinding(component, next_name, next_descriptor),
            )
        return _RepositoryPathBaseline(
            tuple(ancestors),
            self._source_tree_fstat(descriptors[-1], self.repository_path),
        )

    def _compare_path_baselines(
        self,
        expected: _RepositoryPathBaseline,
        observed: _RepositoryPathBaseline,
        detail: str,
    ) -> None:
        if len(expected.ancestors) != len(observed.ancestors):
            _fail("source_snapshot_parent_replaced", detail, self.repository_path)
        for index, (before, after) in enumerate(zip(expected.ancestors, observed.ancestors)):
            if (
                before.component != after.component
                or before.next_component_name != after.next_component_name
            ):
                self._fail_chain_setup(index, detail)
            if before.next_component != after.next_component:
                self._fail_chain_setup(index + 1, detail)
        if expected.root != observed.root:
            self._fail_chain_setup(len(self._chain_components), detail)

    def _reconcile_retained_ancestor(
        self,
        index: int,
        descriptors: Sequence[int],
        expected: _AncestorPathBinding,
    ) -> None:
        component = self._ancestor_fstat(descriptors[index], self.repository_path)
        if component != expected.component:
            self._fail_chain_setup(index, "repository ancestor identity changed")
        if expected.next_component_name != self._chain_components[index]:
            self._fail_chain_setup(index, "repository pathname component name changed")
        next_descriptor = self._ancestor_fstat(descriptors[index + 1], self.repository_path)
        next_entry = _ancestor_component_identity(
            self._stat_chain_component(
                descriptors[index], expected.next_component_name, index,
            ),
        )
        if next_descriptor != expected.next_component or next_entry != expected.next_component:
            self._fail_chain_setup(
                index + 1,
                "repository pathname component identity changed",
            )

    def _open_initial_chain(
        self,
    ) -> tuple[list[int], _RepositoryPathBaseline]:
        descriptors: list[int] = []
        try:
            current = self._acquire_descriptor(
                lambda: os.open(
                    "/",
                    os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
                    | getattr(os, "O_CLOEXEC", 0),
                ),
                "parent-chain[0]",
                "chain",
                "snapshot_root",
            )
            descriptors.append(current)
            for index, component in enumerate(self._chain_components, start=1):
                name = _safe_component(component, "snapshot_root")
                label = (
                    "repository-root"
                    if index == len(self._chain_components)
                    else f"parent-chain[{index}]"
                )
                child = self._acquire_descriptor(
                    lambda name=name, current=current: os.open(
                        name,
                        os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
                        | getattr(os, "O_CLOEXEC", 0),
                        dir_fd=current,
                    ),
                    label,
                    "chain",
                    "snapshot_root",
                )
                entry = self._stat_chain_component(current, name, index - 1)
                opened = os.fstat(child)
                entry_identity = _ancestor_component_identity(entry)
                opened_identity = _ancestor_component_identity(opened)
                if not stat.S_ISDIR(opened.st_mode) or entry_identity != opened_identity:
                    _fail("snapshot_root", "repository path component changed while opening", self.repository_path)
                descriptors.append(child)
                current = child
            return descriptors, self._capture_path_baseline(descriptors)
        except Slice7GRuntimeError:
            raise
        except Exception as exc:
            raise Slice7GRuntimeError(
                "snapshot_root", "repository descriptor chain could not be opened", path=self.repository_path,
            ) from exc

    def _capture_bootstrap_inventory(
        self,
        *,
        require_watched: bool,
    ) -> _SourceSnapshotProvisionalInventory:
        """Stream-authenticate a complete private tree inventory for bootstrap."""

        self._require_live()
        expected_phase = (
            "post_watch_reconciliation" if require_watched else "provisional_capture"
        )
        if self._bootstrap_phase != expected_phase:
            _fail(
                "source_snapshot_authority_state",
                "source bootstrap inventory was requested outside its lifecycle phase",
            )
        if require_watched:
            paths, directories = self._discover_complete_membership()
            phase = "post_watch_reconciled"
        else:
            paths, directories = self._discover_provisional_membership()
            phase = "provisional_captured"
        if not paths:
            _fail("snapshot_members", "post-implementation snapshot cannot be empty")

        previous_directories = self._directory_baselines
        self._directory_baselines = dict(directories)
        try:
            members = self._authenticate_members(paths, final=False)
        finally:
            self._directory_baselines = previous_directories

        if require_watched:
            final_paths, final_directories = self._discover_complete_membership()
        else:
            final_paths, final_directories = self._discover_provisional_membership()
        if final_paths != paths or set(final_directories) != set(directories):
            _fail(
                "source_snapshot_member_changed",
                "source membership changed during bootstrap capture",
            )
        for path, observation in directories.items():
            if final_directories[path] != observation:
                _fail(
                    "source_snapshot_member_changed",
                    "source directory changed during bootstrap capture",
                    path or self.repository_path,
                )
        return _SourceSnapshotProvisionalInventory(
            phase,
            tuple(directories[path] for path in sorted(directories)),
            tuple(members),
        )

    def _install_complete_source_watch_set(
        self,
        provisional: _SourceSnapshotProvisionalInventory,
    ) -> None:
        """Install every source watch before any authoritative baseline exists."""

        self._require_live()
        if (
            type(provisional) is not _SourceSnapshotProvisionalInventory
            or provisional.phase != "provisional_captured"
            or provisional is not self._provisional_inventory
            or self._bootstrap_phase != "provisional_captured"
        ):
            _fail(
                "source_snapshot_authority_state",
                "source watch bootstrap requires the private provisional inventory",
            )
        directories = {item.path: item for item in provisional.directories}
        if len(directories) != len(provisional.directories) or "" not in directories:
            _fail(
                "source_snapshot_membership_mismatch",
                "provisional source directory set is invalid",
            )
        expected_identities = {
            observation.metadata.physical_identity for observation in directories.values()
        }
        if len(expected_identities) != len(directories):
            _fail(
                "source_snapshot_membership_mismatch",
                "provisional source directories are physically aliased",
            )
        root_identity = directories[""].metadata.physical_identity
        if root_identity not in self._source_watch_identities:
            _fail(
                "source_snapshot_monitor",
                "repository root watch is absent before source watch bootstrap",
            )

        paths = tuple(
            sorted(
                (path for path in directories if path),
                key=lambda item: (item.count("/") + 1, item),
            ),
        )
        total = len(paths)
        for index, path in enumerate(paths):
            self._before_source_directory_watch(path, index, total)
            parent_fd: int | None = None
            directory_fd: int | None = None
            try:
                parent_fd, directory_fd, basename = self._open_bootstrap_directory(
                    path,
                    directories,
                )
                expected = directories[path].metadata
                before = self._source_tree_fstat(directory_fd, path)
                entry_before = _source_tree_metadata(
                    self._stat_at(parent_fd, basename, path),
                )
                if before != expected or entry_before != expected:
                    _fail(
                        "source_snapshot_member_changed",
                        "source directory changed before its watch was installed",
                        path,
                    )
                self._watch_directory(
                    directory_fd,
                    before,
                    "member",
                    self._DIRECTORY_CHANGE_MASK,
                    None,
                )
                after = self._source_tree_fstat(directory_fd, path)
                entry_after = _source_tree_metadata(
                    self._stat_at(parent_fd, basename, path),
                )
                if after != expected or entry_after != expected:
                    _fail(
                        "source_snapshot_member_changed",
                        "source directory changed while its watch was installed",
                        path,
                    )
            except BaseException as primary:
                self._release_transient_descriptors(
                    tuple(
                        item for item in (directory_fd, parent_fd) if item is not None
                    ),
                    primary,
                )
            else:
                self._release_transient_descriptors(
                    tuple(
                        item for item in (directory_fd, parent_fd) if item is not None
                    ),
                )

        if self._source_watch_identities != expected_identities:
            _fail(
                "source_snapshot_monitor",
                "complete source watch set differs from provisional directories",
            )

    def _open_bootstrap_directory(
        self,
        relative: str,
        directories: Mapping[str, _SourceSnapshotDirectoryObservation],
    ) -> tuple[int, int, str]:
        """Open one provisional directory and retain its parent for entry checks."""

        components = relative.split("/")
        acquired: list[int] = []
        try:
            current = self._acquire_descriptor(
                lambda: os.dup(self._root_descriptor),
                f"{relative}:bootstrap-root",
                "transient",
                "snapshot_member_io",
            )
            acquired.append(current)
            root_expected = directories.get("")
            if (
                root_expected is None
                or self._source_tree_fstat(current, self.repository_path)
                != root_expected.metadata
            ):
                _fail(
                    "source_snapshot_member_changed",
                    "repository root changed during source watch bootstrap",
                    self.repository_path,
                )
            prefix: list[str] = []
            for index, component in enumerate(components):
                prefix.append(component)
                path = "/".join(prefix)
                child = self._acquire_descriptor(
                    lambda current=current, component=component, path=path: self._open_directory_at(
                        current,
                        component,
                        path,
                    ),
                    f"{path}:bootstrap-directory",
                    "transient",
                    "snapshot_member_io",
                )
                acquired.append(child)
                expected = directories.get(path)
                opened = self._source_tree_fstat(child, path)
                by_name = _source_tree_metadata(self._stat_at(current, component, path))
                if expected is None or opened != expected.metadata or by_name != expected.metadata:
                    _fail(
                        "source_snapshot_member_changed",
                        "source directory changed before complete watch coverage",
                        path,
                    )
                if index == len(components) - 1:
                    self._transfer_descriptor(current)
                    self._transfer_descriptor(child)
                    return current, child, component
                self._release_transient_descriptor(current)
                acquired.remove(current)
                current = child
        except BaseException as primary:
            self._release_transient_descriptors(tuple(reversed(acquired)), primary)

    def _compare_bootstrap_inventories(
        self,
        provisional: _SourceSnapshotProvisionalInventory,
        observed: _SourceSnapshotProvisionalInventory,
    ) -> None:
        """Require complete metadata and digest continuity across watch setup."""

        self._require_live()
        if (
            provisional is not self._provisional_inventory
            or self._bootstrap_phase != "post_watch_reconciliation"
            or observed.phase != "post_watch_reconciled"
        ):
            _fail(
                "source_snapshot_authority_state",
                "source bootstrap inventories are outside their lifecycle phase",
            )
        if provisional.directory_paths != observed.directory_paths:
            _fail(
                "source_snapshot_member_changed",
                "source directory membership changed across watch bootstrap",
            )
        if provisional.member_paths != observed.member_paths:
            _fail(
                "source_snapshot_member_changed",
                "source member set changed across watch bootstrap",
            )
        for before, after in zip(provisional.directories, observed.directories):
            if before != after:
                _fail(
                    "source_snapshot_member_changed",
                    "source directory changed across watch bootstrap",
                    before.path or self.repository_path,
                )
        for before, after in zip(provisional.members, observed.members):
            if before != after:
                _fail(
                    "source_snapshot_member_changed",
                    "source member changed across watch bootstrap",
                    before.path,
                )

    def _before_source_directory_watch(
        self,
        path: str,
        index: int,
        total: int,
    ) -> None:
        """Private test observation boundary; production performs no callback."""

    def _after_complete_source_watch_set(self) -> None:
        """Private test observation boundary after the final source watch."""

    def _discover_complete_membership(
        self,
    ) -> tuple[tuple[str, ...], dict[str, _SourceSnapshotDirectoryObservation]]:
        """Rediscover the complete tree only after full watch bootstrap."""

        return self._enumerate_complete_membership(require_watched=True)

    def _discover_provisional_membership(
        self,
    ) -> tuple[tuple[str, ...], dict[str, _SourceSnapshotDirectoryObservation]]:
        """Capture the candidate tree without claiming complete watch coverage."""

        return self._enumerate_complete_membership(require_watched=False)

    def _enumerate_complete_membership(
        self,
        *,
        require_watched: bool,
    ) -> tuple[tuple[str, ...], dict[str, _SourceSnapshotDirectoryObservation]]:
        self._require_live()
        root = self._root_descriptor
        root_metadata = self._source_tree_fstat(root, self.repository_path)
        members: list[str] = []
        directories: dict[str, _SourceSnapshotDirectoryObservation] = {
            "": _SourceSnapshotDirectoryObservation("", root_metadata),
        }
        if require_watched and root_metadata.physical_identity not in self._source_watch_identities:
            _fail(
                "source_snapshot_monitor",
                "repository root is outside the complete source watch set",
                self.repository_path,
            )
        physical_directories = {root_metadata.physical_identity}
        for name in self._ROOT_FILES:
            observed = self._optional_stat_at(root, name, name)
            if observed is None:
                continue
            if stat.S_ISLNK(observed.st_mode) or not stat.S_ISREG(observed.st_mode):
                _fail("source_snapshot_membership_mismatch", "required source member is not a real file", name)
            if len(members) >= self._MAX_SOURCE_MEMBERS:
                _fail(
                    "source_snapshot_membership_mismatch",
                    "source membership exceeds its module-owned bound",
                )
            members.append(name)
        stack: list[_SourceSnapshotTraversalFrame] = []
        try:
            for name in reversed(self._ROOT_DIRECTORIES):
                observed = self._optional_stat_at(root, name, name)
                if observed is None:
                    continue
                if stat.S_ISLNK(observed.st_mode) or not stat.S_ISDIR(observed.st_mode):
                    _fail("source_snapshot_membership_mismatch", "approved source root is not a real directory", name)
                if len(directories) >= self._MAX_SOURCE_DIRECTORIES:
                    _fail(
                        "source_snapshot_membership_mismatch",
                        "source directory membership exceeds its module-owned bound",
                    )
                prepare = (
                    self._prepare_watched_directory
                    if require_watched
                    else self._prepare_provisional_directory
                )
                descriptor, opened, names = prepare(
                    root, name, name, observed, physical_directories,
                )
                directories[name] = _SourceSnapshotDirectoryObservation(name, opened)
                stack.append(_SourceSnapshotTraversalFrame(name, descriptor, names, 0, opened, True))
            while stack:
                frame = stack[-1]
                if frame.next_index == len(frame.names):
                    if self._source_tree_fstat(frame.descriptor, frame.path) != frame.baseline:
                        _fail("source_snapshot_member_changed", "source directory changed while enumerating", frame.path)
                    if self._directory_names(frame.descriptor, frame.path) != frame.names:
                        _fail(
                            "source_snapshot_membership_mismatch",
                            "source directory membership changed while enumerating",
                            frame.path,
                        )
                    stack.pop()
                    if frame.owns_descriptor:
                        self._release_transient_descriptor(frame.descriptor)
                    continue
                name = frame.names[frame.next_index]
                stack[-1] = _SourceSnapshotTraversalFrame(
                    frame.path, frame.descriptor, frame.names, frame.next_index + 1,
                    frame.baseline, frame.owns_descriptor,
                )
                _safe_component(name, "snapshot_member")
                relative = f"{frame.path}/{name}"
                observed = self._stat_at(frame.descriptor, name, relative)
                if stat.S_ISLNK(observed.st_mode):
                    _fail("source_snapshot_membership_mismatch", "source membership contains a symlink", relative)
                if stat.S_ISDIR(observed.st_mode):
                    if name in self._EXCLUDED_DIRECTORIES:
                        continue
                    if relative.count("/") + 1 > self._MAX_SOURCE_DEPTH:
                        _fail(
                            "source_snapshot_membership_mismatch",
                            "source directory depth exceeds its module-owned bound",
                            relative,
                        )
                    if len(directories) >= self._MAX_SOURCE_DIRECTORIES:
                        _fail(
                            "source_snapshot_membership_mismatch",
                            "source directory membership exceeds its module-owned bound",
                        )
                    prepare = (
                        self._prepare_watched_directory
                        if require_watched
                        else self._prepare_provisional_directory
                    )
                    child, opened, names = prepare(
                        frame.descriptor, name, relative, observed, physical_directories,
                    )
                    directories[relative] = _SourceSnapshotDirectoryObservation(relative, opened)
                    stack.append(_SourceSnapshotTraversalFrame(
                        relative, child, names, 0, opened, True,
                    ))
                    continue
                if stat.S_ISREG(observed.st_mode):
                    if name.endswith(self._EXCLUDED_FILE_SUFFIXES):
                        continue
                    if len(members) >= self._MAX_SOURCE_MEMBERS:
                        _fail(
                            "source_snapshot_membership_mismatch",
                            "source membership exceeds its module-owned bound",
                        )
                    members.append(_source_snapshot_relative(relative))
                    continue
                _fail("source_snapshot_membership_mismatch", "source membership contains an unsupported type", relative)
        except BaseException as primary:
            resources: list[_RepositoryOwnedDescriptor] = []
            for frame in reversed(stack):
                if not frame.owns_descriptor:
                    continue
                resource = self._owned_descriptors.get(frame.descriptor)
                if resource is not None:
                    resources.append(resource)
            cleanup_issues, _ = self._close_descriptor_sequence(tuple(resources))
            _raise_repository_cleanup(primary, cleanup_issues)
        selected = tuple(sorted(members))
        if len(selected) != len(set(selected)):
            _fail("source_snapshot_membership_mismatch", "complete source membership contains duplicates")
        return selected, directories

    def _prepare_provisional_directory(
        self,
        parent_descriptor: int,
        name: str,
        relative: str,
        observed_entry: os.stat_result,
        physical_directories: set[tuple[int, int]],
    ) -> tuple[int, _SourceTreeMetadata, tuple[str, ...]]:
        """Capture one candidate directory without claiming it is watched."""

        descriptor = self._acquire_descriptor(
            lambda: self._open_directory_at(parent_descriptor, name, relative),
            relative,
            "transient",
            "snapshot_member_io",
        )
        try:
            provisional = self._source_tree_fstat(descriptor, relative)
            entry = _source_tree_metadata(self._stat_at(parent_descriptor, name, relative))
            if provisional != _source_tree_metadata(observed_entry) or entry != provisional:
                _fail(
                    "source_snapshot_member_changed",
                    "source directory changed during provisional capture",
                    relative,
                )
            if provisional.physical_identity in physical_directories:
                _fail(
                    "source_snapshot_membership_mismatch",
                    "source directory inode is aliased",
                    relative,
                )
            names = self._directory_names(descriptor, relative)
            after = self._source_tree_fstat(descriptor, relative)
            entry_after = _source_tree_metadata(self._stat_at(parent_descriptor, name, relative))
            if after != provisional or entry_after != provisional:
                _fail(
                    "source_snapshot_member_changed",
                    "source directory changed while provisionally enumerating",
                    relative,
                )
            physical_directories.add(provisional.physical_identity)
            self._transfer_descriptor(descriptor)
            return descriptor, provisional, names
        except BaseException as primary:
            self._release_transient_descriptor(descriptor, primary)

    def _prepare_watched_directory(
        self,
        parent_descriptor: int,
        name: str,
        relative: str,
        observed_entry: os.stat_result,
        physical_directories: set[tuple[int, int]],
    ) -> tuple[int, _SourceTreeMetadata, tuple[str, ...]]:
        """Enumerate one directory already covered by complete watch bootstrap."""

        descriptor = self._acquire_descriptor(
            lambda: self._open_directory_at(parent_descriptor, name, relative),
            relative,
            "transient",
            "snapshot_member_io",
        )
        try:
            provisional = self._source_tree_fstat(descriptor, relative)
            if provisional != _source_tree_metadata(observed_entry):
                _fail("source_snapshot_member_changed", "source directory changed while opening", relative)
            if provisional.physical_identity in physical_directories:
                _fail(
                    "source_snapshot_membership_mismatch",
                    "source directory inode is aliased",
                    relative,
                )
            if provisional.physical_identity not in self._source_watch_identities:
                _fail(
                    "source_snapshot_monitor",
                    "source directory is outside the complete watch set",
                    relative,
                )
            after_watch = self._source_tree_fstat(descriptor, relative)
            entry_after_watch = _source_tree_metadata(
                self._stat_at(parent_descriptor, name, relative)
            )
            if after_watch != provisional or entry_after_watch != provisional:
                _fail(
                    "source_snapshot_member_changed",
                    "source directory changed after watch bootstrap",
                    relative,
                )
            names = self._directory_names(descriptor, relative)
            authoritative = self._source_tree_fstat(descriptor, relative)
            entry_authoritative = _source_tree_metadata(
                self._stat_at(parent_descriptor, name, relative)
            )
            if authoritative != provisional or entry_authoritative != provisional:
                _fail(
                    "source_snapshot_member_changed",
                    "source directory changed while enumerating under its watch",
                    relative,
                )
            physical_directories.add(provisional.physical_identity)
            self._transfer_descriptor(descriptor)
            return descriptor, authoritative, names
        except BaseException as primary:
            self._release_transient_descriptor(descriptor, primary)

    def _authenticate_initial_members(
        self, paths: tuple[str, ...],
    ) -> tuple[_SourceSnapshotMemberObservation, ...]:
        return self._authenticate_members(paths, final=False)

    def _authenticate_final_members(self) -> None:
        observations = self._authenticate_members(self._selected_member_paths, final=True)
        if tuple(item.path for item in observations) != self._selected_member_paths:
            _fail("source_snapshot_membership_mismatch", "final source member ordering changed")

    def _authenticate_members(
        self, paths: tuple[str, ...], *, final: bool,
    ) -> tuple[_SourceSnapshotMemberObservation, ...]:
        observations: list[_SourceSnapshotMemberObservation] = []
        physical_members: set[tuple[int, int]] = set()
        for relative in paths:
            parent_fd: int | None = None
            member_fd: int | None = None
            try:
                parent_fd, member_fd, basename = self._open_member(relative)
                entry_before = self._stat_at(parent_fd, basename, relative)
                before_stat = self._raw_fstat(member_fd, relative)
                before = _source_tree_metadata(before_stat)
                if before != _source_tree_metadata(entry_before):
                    _fail("source_snapshot_member_changed", "member entry differs from opened descriptor", relative)
                if not stat.S_ISREG(before_stat.st_mode) or before_stat.st_nlink != 1:
                    _fail("source_snapshot_member_schema", "member must be regular and single-link", relative)
                if before.physical_identity in physical_members:
                    _fail("source_snapshot_member_schema", "source snapshot members must be physically unique", relative)
                physical_members.add(before.physical_identity)
                baseline = self._member_baselines.get(relative) if final else None
                if baseline is not None:
                    if before.mode != baseline.metadata.mode:
                        _fail("source_snapshot_mode_mismatch", "source member mode changed", relative)
                    if before != baseline.metadata:
                        _fail("source_snapshot_member_changed", "source member metadata changed", relative)
                digest = hashlib.sha256()
                observed_size = 0
                while True:
                    try:
                        chunk = os.read(member_fd, 1024 * 1024)
                    except Exception as exc:
                        raise Slice7GRuntimeError(
                            "snapshot_member_io", "source member streaming read failed", path=relative,
                        ) from exc
                    if not chunk:
                        break
                    observed_size += len(chunk)
                    digest.update(chunk)
                after = _source_tree_metadata(self._raw_fstat(member_fd, relative))
                entry_after = _source_tree_metadata(self._stat_at(parent_fd, basename, relative))
                if after.mode != before.mode:
                    _fail("source_snapshot_mode_mismatch", "source member mode changed while hashing", relative)
                if before != after or after != entry_after or observed_size != before.size:
                    _fail("source_snapshot_member_changed", "source member changed while hashing", relative)
                record = Slice7GSourceSnapshotMember(relative, before.mode, observed_size, digest.hexdigest())
                if baseline is not None and (
                    after != baseline.metadata
                    or record.sha256 != baseline.sha256
                    or record != baseline.record
                ):
                    _fail("source_snapshot_member_changed", "source member differs from initial authority", relative)
                observations.append(_SourceSnapshotMemberObservation(relative, before, record.sha256, record))
            except BaseException as primary:
                self._release_transient_descriptors(
                    tuple(item for item in (member_fd, parent_fd) if item is not None),
                    primary,
                )
            else:
                self._release_transient_descriptors(
                    tuple(item for item in (member_fd, parent_fd) if item is not None),
                )
        return tuple(observations)

    def _open_member(self, relative: str) -> tuple[int, int, str]:
        components = relative.split("/")
        acquired: list[int] = []
        try:
            current = self._acquire_descriptor(
                lambda: os.dup(self._root_descriptor),
                f"{relative}:parent-root",
                "transient",
                "snapshot_member_io",
            )
            acquired.append(current)
            prefix: list[str] = []
            for component in components[:-1]:
                prefix.append(component)
                path = "/".join(prefix)
                child = self._acquire_descriptor(
                    lambda current=current, component=component, path=path: self._open_directory_at(
                        current, component, path,
                    ),
                    f"{path}:member-parent",
                    "transient",
                    "snapshot_member_io",
                )
                acquired.append(child)
                expected = self._directory_baselines.get(path)
                opened = self._source_tree_fstat(child, path)
                by_name = _source_tree_metadata(self._stat_at(current, component, path))
                if expected is None or opened != expected.metadata or by_name != expected.metadata:
                    _fail("source_snapshot_member_changed", "source member parent directory changed", path)
                self._release_transient_descriptor(current)
                acquired.remove(current)
                current = child
            member = self._acquire_descriptor(
                lambda: self._open_file_at(current, components[-1], relative),
                relative,
                "transient",
                "snapshot_member_io",
            )
            acquired.append(member)
            self._transfer_descriptor(current)
            self._transfer_descriptor(member)
            return current, member, components[-1]
        except BaseException as primary:
            self._release_transient_descriptors(tuple(reversed(acquired)), primary)

    def _reconcile_public_path(self, *, check_metadata: bool) -> None:
        baseline = self._chain_metadata
        if type(baseline) is not _RepositoryPathBaseline:
            _fail("source_snapshot_authority_state", "repository pathname baseline is unavailable")
        self._reconcile_chain_path(baseline, check_metadata=check_metadata)

    def _reconcile_chain_path(
        self,
        expected: _RepositoryPathBaseline,
        *,
        check_metadata: bool,
    ) -> None:
        reopened: list[int] = []
        try:
            current = self._acquire_descriptor(
                lambda: os.open(
                    "/",
                    os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
                    | getattr(os, "O_CLOEXEC", 0),
                ),
                "public-path-barrier[0]",
                "transient",
                "source_snapshot_parent_replaced",
            )
            reopened.append(current)
            if self._ancestor_fstat(current, self.repository_path) != expected.ancestors[0].component:
                _fail(
                    "source_snapshot_parent_replaced",
                    "repository ancestor identity changed",
                    self.repository_path,
                )
            for index, component in enumerate(self._chain_components, start=1):
                name = _safe_component(component, "snapshot_root")
                parent = current
                try:
                    child = self._acquire_descriptor(
                        lambda name=name, parent=parent: os.open(
                            name,
                            os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
                            | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0),
                            dir_fd=parent,
                        ),
                        f"public-path-barrier[{index}]",
                        "transient",
                        (
                            "source_snapshot_root_replaced"
                            if index == len(self._chain_components)
                            else "source_snapshot_parent_replaced"
                        ),
                    )
                except Exception as exc:
                    code = "source_snapshot_root_replaced" if index == len(self._chain_components) else "source_snapshot_parent_replaced"
                    raise Slice7GRuntimeError(code, "repository pathname component is unavailable") from exc
                reopened.append(child)
                current = child
                binding = expected.ancestors[index - 1]
                if name != binding.next_component_name:
                    _fail(
                        "source_snapshot_parent_replaced",
                        "repository pathname component name changed",
                        self.repository_path,
                    )
                entry_identity = _ancestor_component_identity(
                    self._stat_chain_component(parent, name, index - 1),
                )
                opened_identity = self._ancestor_fstat(child, self.repository_path)
                if entry_identity != binding.next_component or opened_identity != binding.next_component:
                    code = (
                        "source_snapshot_root_replaced"
                        if index == len(self._chain_components)
                        else "source_snapshot_parent_replaced"
                    )
                    _fail(code, "repository pathname component identity changed", self.repository_path)
                if index < len(self._chain_components):
                    if opened_identity != expected.ancestors[index].component:
                        _fail(
                            "source_snapshot_parent_replaced",
                            "repository ancestor identity changed",
                            self.repository_path,
                        )
                elif opened_identity != _ancestor_identity_from_source(expected.root):
                    _fail(
                        "source_snapshot_root_replaced",
                        "repository root identity changed",
                        self.repository_path,
                    )
            if check_metadata:
                if self._source_tree_fstat(reopened[-1], self.repository_path) != expected.root:
                    _fail(
                        "source_snapshot_root_replaced",
                        "repository root metadata changed",
                        self.repository_path,
                    )
                if self._source_tree_fstat(
                    self._chain_descriptors[-1], self.repository_path,
                ) != expected.root:
                    _fail(
                        "source_snapshot_root_replaced",
                        "retained repository root metadata changed",
                        self.repository_path,
                    )
                for index, binding in enumerate(expected.ancestors):
                    self._reconcile_retained_ancestor(
                        index, self._chain_descriptors, binding,
                    )
        except BaseException as primary:
            self._release_transient_descriptors(tuple(reversed(reopened)), primary)
        else:
            self._release_transient_descriptors(tuple(reversed(reopened)))

    @staticmethod
    def _open_change_monitor() -> int:
        try:
            library = ctypes.CDLL(None, use_errno=True)
            initialize = library.inotify_init1
            initialize.argtypes = [ctypes.c_int]
            initialize.restype = ctypes.c_int
            descriptor = initialize(os.O_NONBLOCK | getattr(os, "O_CLOEXEC", 0))
        except Exception as exc:
            raise Slice7GRuntimeError(
                "source_snapshot_monitor", "repository change monitor is unavailable",
            ) from exc
        if descriptor < 0:
            error = ctypes.get_errno()
            raise Slice7GRuntimeError(
                "source_snapshot_monitor", os.strerror(error or errno.ENOSYS),
            )
        return descriptor

    @staticmethod
    def _monitor_fstat(descriptor: int) -> os.stat_result:
        try:
            return os.fstat(descriptor)
        except Exception as exc:
            raise Slice7GRuntimeError(
                "source_snapshot_monitor", "repository change monitor stat failed",
            ) from exc

    def _watch_directory(
        self,
        descriptor: int,
        metadata: _SourceTreeMetadata | _AncestorComponentIdentity,
        scope: str,
        mask: int,
        expected_child_name: str | None,
    ) -> None:
        identity = (metadata.device, metadata.inode)
        if identity in self._watched_directories:
            watch = self._watched_directories[identity]
            retained = self._monitor_scopes.get(watch)
            if retained is None:
                _fail(
                    "source_snapshot_monitor",
                    "repository watch registry is internally inconsistent",
                )
            if scope in {"root", "member"}:
                if retained.scope not in {"root", "member"}:
                    _fail(
                        "source_snapshot_monitor",
                        "source directory collides with an ancestor watch",
                    )
                self._source_watch_identities.add(identity)
            return
        monitor = self._monitor_descriptor
        if monitor is None:
            _fail("source_snapshot_monitor", "repository change monitor is closed")
        try:
            library = ctypes.CDLL(None, use_errno=True)
            add_watch = library.inotify_add_watch
            add_watch.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_uint32]
            add_watch.restype = ctypes.c_int
            watch = add_watch(monitor, f"/proc/self/fd/{descriptor}".encode("ascii"), mask)
        except Exception as exc:
            raise Slice7GRuntimeError(
                "source_snapshot_monitor", "repository directory watch could not be installed",
            ) from exc
        if watch < 0:
            error = ctypes.get_errno()
            raise Slice7GRuntimeError(
                "source_snapshot_monitor", os.strerror(error or errno.ENOSYS),
            )
        self._watched_directories[identity] = watch
        self._monitor_scopes[watch] = _RepositoryWatch(scope, expected_child_name)
        if scope in {"root", "member"}:
            self._source_watch_identities.add(identity)

    def _assert_no_monitored_changes(self) -> None:
        monitor = self._monitor_descriptor
        if monitor is None:
            _fail("source_snapshot_monitor", "repository change monitor is closed")
        total_bytes = 0
        total_events = 0
        total_reads = 0
        while True:
            total_reads += 1
            if total_reads > self._INOTIFY_MAX_DRAIN_READS:
                _fail("source_snapshot_monitor", "repository change monitor drain exceeded its read limit")
            try:
                raw = os.read(monitor, 65536)
            except BlockingIOError:
                return
            except InterruptedError:
                continue
            except Exception as exc:
                raise Slice7GRuntimeError(
                    "source_snapshot_monitor", "repository change monitor read failed",
                ) from exc
            if not raw:
                _fail("source_snapshot_monitor", "repository change monitor reached EOF")
            total_bytes += len(raw)
            if total_bytes > self._INOTIFY_MAX_DRAIN_BYTES:
                _fail("source_snapshot_monitor", "repository change monitor drain exceeded its byte limit")
            events = self._parse_monitor_buffer(raw)
            total_events += len(events)
            if total_events > self._INOTIFY_MAX_DRAIN_EVENTS:
                _fail("source_snapshot_monitor", "repository change monitor drain exceeded its event limit")
            for watch, mask, name in events:
                self._handle_monitor_event(watch, mask, name)

    def _parse_monitor_buffer(self, raw: bytes) -> tuple[tuple[int, int, bytes], ...]:
        if type(raw) is not bytes or not raw:
            _fail("source_snapshot_monitor", "repository change monitor returned an invalid buffer")
        header = self._INOTIFY_EVENT_HEADER_BYTES
        if len(raw) < header:
            _fail("source_snapshot_monitor", "repository change monitor returned a truncated header")
        events: list[tuple[int, int, bytes]] = []
        offset = 0
        while offset < len(raw):
            if len(raw) - offset < header:
                _fail("source_snapshot_monitor", "repository change monitor returned trailing bytes")
            watch = int.from_bytes(raw[offset:offset + 4], "little", signed=True)
            mask = int.from_bytes(raw[offset + 4:offset + 8], "little")
            length = int.from_bytes(raw[offset + 12:offset + 16], "little")
            if length % self._INOTIFY_NAME_ALIGNMENT:
                _fail("source_snapshot_monitor", "repository change monitor name area is misaligned")
            end = offset + header + length
            if end > len(raw):
                _fail("source_snapshot_monitor", "repository change monitor frame exceeds its buffer")
            name = b""
            if length:
                area = raw[offset + header:end]
                nul = area.find(b"\0")
                if nul < 0:
                    _fail("source_snapshot_monitor", "repository change monitor name lacks a terminator")
                if any(area[nul + 1:]):
                    _fail("source_snapshot_monitor", "repository change monitor name padding is nonzero")
                name = area[:nul]
                if not name:
                    _fail("source_snapshot_monitor", "repository change monitor name is empty")
            events.append((watch, mask, name))
            offset = end
        if offset != len(raw):
            _fail("source_snapshot_monitor", "repository change monitor buffer was not consumed exactly")
        return tuple(events)

    def _handle_monitor_event(self, watch: int, mask: int, name: bytes) -> None:
        permitted_mask = (
            self._DIRECTORY_CHANGE_MASK | self._IN_UNMOUNT | self._IN_Q_OVERFLOW
            | self._IN_IGNORED | self._IN_ISDIR
        )
        if mask == 0 or mask & ~permitted_mask:
            _fail("source_snapshot_monitor", "repository change monitor returned an invalid mask")
        if watch == -1:
            if mask != self._IN_Q_OVERFLOW or name:
                _fail("source_snapshot_monitor", "repository change monitor returned an invalid global event")
            _fail("source_snapshot_monitor", "repository change monitor overflowed")
        if mask & self._IN_Q_OVERFLOW:
            _fail("source_snapshot_monitor", "repository change monitor overflow used a watch descriptor")
        watched = self._monitor_scopes.get(watch)
        if watched is None:
            _fail("source_snapshot_monitor", "repository change monitor returned an unknown watch descriptor")
        self_event_mask = self._IN_DELETE_SELF | self._IN_MOVE_SELF | self._IN_IGNORED | self._IN_UNMOUNT
        entry_event_mask = (
            self._IN_MODIFY | self._IN_ATTRIB | self._IN_CLOSE_WRITE | self._IN_MOVED_FROM
            | self._IN_MOVED_TO | self._IN_CREATE | self._IN_DELETE
        )
        if name and not mask & entry_event_mask:
            _fail("source_snapshot_monitor", "repository change monitor name is invalid for its mask")
        if name and mask & self_event_mask:
            _fail("source_snapshot_monitor", "repository self-event unexpectedly included a name")
        if mask & (self._IN_IGNORED | self._IN_UNMOUNT):
            _fail("source_snapshot_monitor", "repository change-monitor watch was invalidated")
        if watched.scope in {"parent", "root_parent"}:
            if not name or watched.expected_child_name is None:
                _fail(
                    "source_snapshot_parent_replaced",
                    "repository parent changed during authentication",
                )
            expected_text = _safe_component(
                watched.expected_child_name, "snapshot monitor component",
            )
            expected_bytes = os.fsencode(expected_text)
            try:
                decoded_name = os.fsdecode(name)
            except Exception as exc:
                raise Slice7GRuntimeError(
                    "source_snapshot_monitor",
                    "repository change monitor name could not be decoded",
                ) from exc
            if name == expected_bytes and decoded_name == expected_text:
                code = (
                    "source_snapshot_root_replaced"
                    if watched.scope == "root_parent"
                    else "source_snapshot_parent_replaced"
                )
                _fail(code, "repository pathname component changed during authentication")
            return
        if watched.scope == "root" and (mask & self_event_mask or not name):
            _fail("source_snapshot_root_replaced", "repository root changed during authentication")
        if watched.scope not in {"root", "member"}:
            _fail("source_snapshot_monitor", "repository change monitor scope is invalid")
        _fail("source_snapshot_member_changed", "repository member changed during authentication")

    @staticmethod
    def _raw_fstat(descriptor: int, path: str) -> os.stat_result:
        try:
            return os.fstat(descriptor)
        except Exception as exc:
            raise Slice7GRuntimeError("snapshot_member_io", "source descriptor stat failed", path=path) from exc

    def _source_tree_fstat(
        self, descriptor: int, path: str,
    ) -> _SourceTreeMetadata:
        return _source_tree_metadata(self._raw_fstat(descriptor, path))

    def _ancestor_fstat(
        self, descriptor: int, path: str,
    ) -> _AncestorComponentIdentity:
        return _ancestor_component_identity(self._raw_fstat(descriptor, path))

    @staticmethod
    def _optional_stat_at(parent: int, name: str, path: str) -> os.stat_result | None:
        try:
            return os.stat(name, dir_fd=parent, follow_symlinks=False)
        except FileNotFoundError:
            return None
        except Exception as exc:
            raise Slice7GRuntimeError("snapshot_member_io", "source membership stat failed", path=path) from exc

    @staticmethod
    def _stat_at(parent: int, name: str, path: str) -> os.stat_result:
        try:
            return os.stat(name, dir_fd=parent, follow_symlinks=False)
        except Exception as exc:
            raise Slice7GRuntimeError("snapshot_member_io", "source membership stat failed", path=path) from exc

    @staticmethod
    def _open_directory_at(parent: int, name: str, path: str) -> int:
        try:
            return os.open(
                name,
                os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
                | getattr(os, "O_CLOEXEC", 0),
                dir_fd=parent,
            )
        except Exception as exc:
            raise Slice7GRuntimeError("snapshot_member_io", "source directory open failed", path=path) from exc

    @staticmethod
    def _open_file_at(parent: int, name: str, path: str) -> int:
        try:
            return os.open(
                name,
                os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0),
                dir_fd=parent,
            )
        except Exception as exc:
            raise Slice7GRuntimeError("snapshot_member_io", "source member open failed", path=path) from exc

    @staticmethod
    def _directory_names(descriptor: int, path: str) -> tuple[str, ...]:
        try:
            names = tuple(os.listdir(descriptor))
        except Exception as exc:
            raise Slice7GRuntimeError("snapshot_member_io", "source directory enumeration failed", path=path) from exc
        for name in names:
            _safe_component(name, "snapshot_member")
        return tuple(sorted(names))


def verify_post_implementation_source_snapshot(
    raw: bytes,
    repository_root: str | os.PathLike[str],
) -> bool:
    """Build-gate verification accepts v2 only and reauthenticates every mode and byte."""

    snapshot = parse_post_implementation_source_snapshot(raw)
    root_text = _absolute_path_text(repository_root, "snapshot_root")
    authority = _RepositorySnapshotAuthority(root_text)
    try:
        authority.begin(expected=snapshot.members)
        authority.final_barrier()
    except BaseException as primary:
        _finish_repository_authority(authority, primary)
        raise AssertionError("repository cleanup helper returned after a primary failure")
    _finish_repository_authority(authority)
    return True


def post_implementation_source_snapshot(
    repository_root: str | os.PathLike[str], member_paths: Iterable[str] | None = None,
) -> tuple[bytes, str, int]:
    """Build the complete descriptor-authenticated authoritative v2 proposal."""

    if member_paths is not None:
        if type(member_paths) not in (list, tuple):
            _fail("snapshot_members_type", "snapshot members must be an exact list or tuple")
        _fail(
            "source_snapshot_membership_mismatch",
            "authoritative snapshot construction does not accept caller-selected members",
        )
    root_text = _absolute_path_text(repository_root, "snapshot_root")
    authority = _RepositorySnapshotAuthority(root_text)
    try:
        records = authority.begin()
        snapshot = Slice7GPostImplementationSourceSnapshot(POST_IMPLEMENTATION_SNAPSHOT_SCHEMA, records)
        raw = canonical_post_implementation_source_snapshot_bytes(snapshot)
        identity = post_implementation_source_snapshot_identity(snapshot)
        authority.final_barrier()
    except BaseException as primary:
        _finish_repository_authority(authority, primary)
        raise AssertionError("repository cleanup helper returned after a primary failure")
    _finish_repository_authority(authority)
    return raw, identity, len(records)


def structural_post_implementation_source_snapshot(
    repository_root: str | os.PathLike[str], member_paths: Iterable[str],
) -> tuple[bytes, str, int]:
    """Build descriptor-authenticated v2 bytes without conferring build authority."""

    root_text = _absolute_path_text(repository_root, "snapshot_root")
    if type(member_paths) not in (list, tuple):
        _fail("snapshot_members_type", "snapshot members must be an exact list or tuple")
    detached = tuple(_source_snapshot_relative(item) for item in member_paths)
    selected = tuple(sorted(detached))
    if not selected or len(selected) != len(set(selected)):
        _fail("snapshot_members", "structural snapshot members must be nonempty and unique")
    authority = _RepositorySnapshotAuthority(root_text)
    try:
        records = authority.begin(selected_paths=selected)
        snapshot = Slice7GPostImplementationSourceSnapshot(POST_IMPLEMENTATION_SNAPSHOT_SCHEMA, records)
        raw = canonical_post_implementation_source_snapshot_bytes(snapshot)
        identity = post_implementation_source_snapshot_identity(snapshot)
        authority.final_barrier()
    except BaseException as primary:
        _finish_repository_authority(authority, primary)
        raise AssertionError("repository cleanup helper returned after a primary failure")
    _finish_repository_authority(authority)
    return raw, identity, len(records)


def discover_post_implementation_snapshot_members(
    repository_root: str | os.PathLike[str],
) -> tuple[str, ...]:
    """Select the complete source/config/doc snapshot set without persisting it.

    The top-level ``evaluation/`` evidence tree, generated build trees, caches,
    VCS metadata and bytecode are excluded by construction.
    """

    root_text = _absolute_path_text(repository_root, "snapshot_root")
    authority = _RepositorySnapshotAuthority(root_text)
    try:
        members, _ = authority._discover_complete_membership()
    except BaseException as primary:
        _finish_repository_authority(authority, primary)
        raise AssertionError("repository cleanup helper returned after a primary failure")
    _finish_repository_authority(authority)
    return members


def _evidence_bindings(result: Slice7GCellResult) -> dict[str, Any]:
    return {
        "charter_logical_identity": result.charter_logical_identity,
        "campaign_identity": result.campaign_identity,
        "campaign_plan_identity": result.campaign_plan_identity,
        "cell_id": result.cell_id,
        "attempt_ledger_identity": result.attempt_ledger_identity,
        "attempt_ledger_revision": result.attempt_ledger_revision,
        "process_start_event_identity": result.process_start_event_identity,
        "runtime_authorization_identity": result.runtime_authorization_identity,
        "ros_domain_id": result.ros_domain_id,
        "campaign_output_root": result.campaign_output_root,
        "cell_output_path": result.cell_output_path,
    }


def _validated_readiness_result(value: Any) -> Slice7GReadinessResult:
    if type(value) is not Slice7GReadinessResult:
        _fail("readiness_result_type", "readiness result must be an exact record")
    try:
        fields = (
            value.passed,
            value.failure_code,
            value.stable_sample_count,
            value.stable_interval_seconds,
            value.q_variation,
            value.tip_variation_m,
            value.tactile_age_seconds,
            value.safety_age_seconds,
        )
    except AttributeError as exc:
        raise Slice7GRuntimeError("readiness_result_record", "readiness result is partially initialized") from exc
    if type(fields[0]) is not bool or type(fields[1]) is not str:
        _fail("readiness_result_record", "readiness pass/code fields have invalid types")
    if type(fields[2]) is not int or type(fields[2]) is bool or fields[2] < 0:
        _fail("readiness_result_record", "readiness sample count is invalid")
    numeric = tuple(_finite(item, "readiness_result") for item in fields[3:6])
    ages = []
    for item in fields[6:]:
        if type(item) not in (int, float) or type(item) is bool or math.isnan(float(item)):
            _fail("readiness_result_record", "readiness ages must be numeric and not NaN")
        ages.append(float(item))
    if fields[0] and fields[1]:
        _fail("readiness_result_record", "passing readiness cannot retain a failure code")
    if not fields[0] and not fields[1]:
        _fail("readiness_result_record", "failed readiness requires a stable failure code")
    return Slice7GReadinessResult(
        fields[0], fields[1], fields[2], numeric[0], numeric[1], numeric[2], ages[0], ages[1],
    )


def _validated_runtime_authorization(
    value: Any,
    charter: Slice7GCharter | None = None,
) -> Slice7GRuntimeAuthorization:
    """Reconstruct cached authorization authority at each effect boundary."""

    if type(value) is not Slice7GRuntimeAuthorization:
        _fail("runtime_authorization_type", "runtime authorization must be an exact record")
    try:
        data = {
            "schema_version": value.schema_version,
            "charter_logical_identity": value.charter_logical_identity,
            "campaign_id": value.campaign_id,
            "campaign_identity": value.campaign_identity,
            "post_implementation_source_snapshot_identity": value.post_implementation_source_snapshot_identity,
            "campaign_output_root": value.campaign_output_root,
            "issued_at_utc": value.issued_at_utc,
            "execution_authorized": value.execution_authorized,
        }
        cached_canonical = value.canonical_bytes
        cached_identity = value.identity
    except AttributeError as exc:
        raise Slice7GRuntimeError(
            "runtime_authorization_record",
            "runtime authorization is partially initialized",
        ) from exc
    if data["schema_version"] != RUNTIME_AUTHORIZATION_SCHEMA:
        _fail("runtime_authorization_schema", "unsupported runtime authorization schema")
    _digest(data["charter_logical_identity"], "charter_logical_identity")
    _identifier(data["campaign_id"], "campaign_id")
    _digest(data["campaign_identity"], "campaign_identity")
    _digest(data["post_implementation_source_snapshot_identity"], "source_snapshot_identity")
    _absolute_path_text(data["campaign_output_root"], "campaign_output_root")
    _utc(data["issued_at_utc"], "issued_at_utc")
    if data["execution_authorized"] is not True:
        _fail("runtime_not_authorized", "authorization must explicitly authorize execution")
    canonical = _canonical(data)
    if type(cached_canonical) is not bytes or cached_canonical != canonical:
        _fail("runtime_authorization_canonical", "cached authorization bytes are inconsistent")
    identity = hashlib.sha256(RUNTIME_AUTHORIZATION_DOMAIN + canonical).hexdigest()
    if cached_identity != identity:
        _fail("runtime_authorization_identity", "cached authorization identity is inconsistent")
    if charter is not None:
        charter_identity = slice_7g_charter_identity(charter)
        if data["charter_logical_identity"] != charter_identity:
            _fail("runtime_authorization_charter", "authorization differs from the supplied charter")
        initial = create_slice_7g_initial_attempt_ledger(charter, data["campaign_id"])
        if data["campaign_identity"] != initial.campaign_identity:
            _fail("runtime_authorization_campaign", "authorization campaign identity is invalid")
    return Slice7GRuntimeAuthorization(
        data["schema_version"],
        data["charter_logical_identity"],
        data["campaign_id"],
        data["campaign_identity"],
        data["post_implementation_source_snapshot_identity"],
        data["campaign_output_root"],
        data["issued_at_utc"],
        True,
        canonical,
        identity,
    )


def _validated_domain_lease(value: Any) -> Slice7GDomainLease:
    if type(value) is not Slice7GDomainLease:
        _fail("domain_lease_type", "domain lease must be an exact record")
    try:
        data = {
            "schema_version": value.schema_version,
            "charter_logical_identity": value.charter_logical_identity,
            "runtime_authorization_identity": value.runtime_authorization_identity,
            "campaign_identity": value.campaign_identity,
            "domain_id": value.domain_id,
            "occupancy_checked": value.occupancy_checked,
            "collision_free": value.collision_free,
            "provider_receipt_identity": value.provider_receipt_identity,
            "leased_at_utc": value.leased_at_utc,
        }
        cached_identity = value.identity
    except AttributeError as exc:
        raise Slice7GRuntimeError("domain_lease_record", "domain lease is partially initialized") from exc
    if data["schema_version"] != DOMAIN_LEASE_SCHEMA:
        _fail("domain_lease_schema", "unsupported domain lease schema")
    for field in (
        "charter_logical_identity", "runtime_authorization_identity", "campaign_identity",
        "provider_receipt_identity",
    ):
        _digest(data[field], field)
    if type(data["domain_id"]) is not int or not 100 <= data["domain_id"] <= 199:
        _fail("domain_lease_domain", "leased domain must be in 100..199")
    if data["occupancy_checked"] is not True or data["collision_free"] is not True:
        _fail("domain_lease_occupancy", "lease must retain a collision-free occupancy decision")
    _utc(data["leased_at_utc"], "leased_at_utc")
    identity = hashlib.sha256(DOMAIN_LEASE_DOMAIN + _canonical(data)).hexdigest()
    if cached_identity != identity:
        _fail("domain_lease_identity", "cached domain lease identity is inconsistent")
    return Slice7GDomainLease(identity=identity, **data)


def _validate_execution_context(
    execution: Slice7GCellExecution,
    cell: Slice7GCampaignCell,
    plan: Slice7GCampaignPlan,
    ledger: Slice7GAttemptLedger,
) -> None:
    if type(execution) is not Slice7GCellExecution:
        _fail("coordinator_execution_type", "process factory must return an exact cell execution")
    result = execution.cell_result
    canonical_slice_7g_cell_result_bytes(result)
    expected = {
        "cell_id": cell.cell_id,
        "campaign_identity": plan.campaign_identity,
        "campaign_plan_identity": slice_7g_campaign_plan_identity(plan),
        "attempt_ledger_identity": slice_7g_attempt_ledger_identity(ledger),
        "attempt_ledger_revision": ledger.revision,
        "process_start_event_identity": ledger.last_event_identity,
        "runtime_authorization_identity": ledger.runtime_authorization_identity,
        "scenario_id": cell.scenario_id,
        "source_scenario_id": cell.source_scenario_id,
        "seed": cell.seed,
        "duration_seconds": cell.duration_seconds,
        "runtime_mode": cell.runtime_mode,
        "ros_domain_id": cell.ros_domain_id,
        "campaign_output_root": cell.campaign_output_root,
        "cell_output_path": cell.cell_output_path,
        "argv": cell.argv,
    }
    for field, required in expected.items():
        if getattr(result, field) != required:
            _fail("coordinator_cell_result", f"cell result field {field} differs from committed plan")


def _detached_execution(value: Any) -> Slice7GCellExecution:
    if type(value) is not Slice7GCellExecution:
        _fail("execution_record", "cell execution must use the exact supported record type")
    try:
        cell_result = value.cell_result
        source_payloads = tuple(getattr(value, name) for name in (
            "invocation_payload", "readiness_payload", "safety_payload", "tactile_payload",
            "output_inventory_payload",
        ))
    except AttributeError as exc:
        raise Slice7GRuntimeError("execution_record", "cell execution is partially initialized") from exc
    if type(cell_result) is not Slice7GCellResult:
        _fail("execution_record", "cell execution result must use the exact governance record")
    try:
        result_data = json.loads(canonical_slice_7g_cell_result_bytes(cell_result).decode("utf-8"))
        result_data["argv"] = tuple(result_data["argv"])
        detached_result = Slice7GCellResult(**result_data)
    except (Slice7GGovernanceError, KeyError, TypeError, ValueError) as exc:
        raise Slice7GRuntimeError("execution_record", "cell execution result is malformed") from exc
    payloads = []
    for name, payload in zip((
        "invocation_payload", "readiness_payload", "safety_payload", "tactile_payload",
        "output_inventory_payload",
    ), source_payloads):
        if type(payload) is not dict:
            _fail("execution_record", f"{name} must be an exact dictionary")
        try:
            payloads.append(_detach(payload))
        except Slice7GRuntimeError as exc:
            raise Slice7GRuntimeError("execution_record", f"{name} contains malformed values") from exc
    return Slice7GCellExecution(detached_result, *payloads)


def _validate_execution_payloads(execution: Slice7GCellExecution) -> None:
    if type(execution) is not Slice7GCellExecution:
        _fail("execution_record", "execution payload validation requires an exact record")
    result = execution.cell_result
    expected = {
        "invocation_payload": {
            "argv": list(result.argv), "process_exit_status": result.process_exit_status,
        },
        "readiness_payload": {
            "readiness_success": result.readiness_success,
            "stable_sample_count": result.stable_sample_count,
            "stable_interval_seconds": result.stable_interval_seconds,
            "q_variation": result.q_variation,
            "tip_variation_m": result.tip_variation_m,
        },
        "safety_payload": {
            "minimum_physical_wall_clearance_m": result.minimum_physical_wall_clearance_m,
            "minimum_safety_margin_wall_clearance_m": result.minimum_safety_margin_wall_clearance_m,
            "collision_sample_count": result.collision_sample_count,
            "safety_fault_count": result.safety_fault_count,
            "nonfinite_value_count": result.nonfinite_value_count,
        },
        "tactile_payload": {
            "valid_aligned_sample_count": result.valid_aligned_sample_count,
            "invalid_sample_count": result.invalid_sample_count,
            "invalid_sample_percentage": result.invalid_sample_percentage,
            "saturation_percentage": result.saturation_percentage,
            "missing_required_topic_count": result.missing_required_topic_count,
        },
    }
    for field, required in expected.items():
        if getattr(execution, field) != required:
            _fail("execution_record", f"{field} differs from the validated cell result")
    output = execution.output_inventory_payload
    _closed(
        output,
        {"missing_required_result_file_count", "output_tree_identity", "regular_file_count", "regular_file_bytes"},
        "execution_record",
    )
    if output["missing_required_result_file_count"] != result.missing_required_result_file_count:
        _fail("execution_record", "output inventory missing-file count differs from cell result")
    _digest(output["output_tree_identity"], "output_tree_identity")
    _exact_nonnegative_int(output["regular_file_count"], "output_regular_file_count")
    _exact_nonnegative_int(output["regular_file_bytes"], "output_regular_file_bytes")


def _validate_runner_environment(
    env: dict[str, str], cell: Slice7GCampaignCell, ledger: Slice7GAttemptLedger,
    plan: Slice7GCampaignPlan,
) -> None:
    expected = {
        "ROS_DOMAIN_ID": str(cell.ros_domain_id),
        "ROS_DISTRO": "humble",
        "CTR_SLICE_7G_CHARTER_IDENTITY": plan.charter_logical_identity,
        "CTR_SLICE_7G_RUNTIME_AUTHORIZATION_IDENTITY": ledger.runtime_authorization_identity,
        "CTR_SLICE_7G_ATTEMPT_LEDGER_IDENTITY": slice_7g_attempt_ledger_identity(ledger),
        "CTR_SLICE_7G_ATTEMPT_LEDGER_REVISION": str(ledger.revision),
        "CTR_SLICE_7G_PROCESS_START_EVENT_IDENTITY": ledger.last_event_identity,
        "CTR_SLICE_7G_CAMPAIGN_PLAN_IDENTITY": slice_7g_campaign_plan_identity(plan),
        "CTR_SLICE_7G_CELL_ID": cell.cell_id,
        "CTR_SLICE_7G_CAMPAIGN_ID": plan.campaign_id,
        "CTR_SLICE_7G_CAMPAIGN_OUTPUT_ROOT": cell.campaign_output_root,
        "CTR_SLICE_7G_CELL_OUTPUT_ROOT": cell.cell_output_path,
    }
    for key, required in expected.items():
        if env.get(key) != required:
            _fail("runner_environment_binding", f"runner environment field {key} is not committed")
    for key in ("CTR_SLICE_7G_DOMAIN_LEASE_IDENTITY", "CTR_SLICE_7G_DOMAIN_COMMITTED_BINDING_IDENTITY"):
        _digest(env.get(key), key)


def _write_process_output_artifacts(
    cell_root: str | os.PathLike[str], observation: Slice7GProcessObservation,
) -> None:
    observation = _validated_process_observation(observation, observation.argv, "cell_process_observation")
    root_text = _absolute_path_text(_path(cell_root, "cell_output_root"), "cell_output_root")
    root_descriptor = _open_directory_path_nofollow(root_text, "cell_output_root")
    created: list[str] = []
    try:
        records = []
        for relative, raw in ((PROCESS_STDOUT_PATH, observation.stdout), (PROCESS_STDERR_PATH, observation.stderr)):
            _exclusive_sealed_file_at(root_descriptor, relative, raw, "process_output_exists")
            created.append(relative)
            records.append({"path": relative, "size": len(raw), "sha256": hashlib.sha256(raw).hexdigest()})
        receipt = {
            "schema_version": PROCESS_OUTPUT_RECEIPT_SCHEMA,
            "argv": list(observation.argv),
            "process_exit_status": observation.returncode,
            "streams": records,
        }
        _exclusive_sealed_file_at(
            root_descriptor, PROCESS_OUTPUT_RECEIPT_PATH, _canonical(receipt), "process_output_exists",
        )
        created.append(PROCESS_OUTPUT_RECEIPT_PATH)
        os.fsync(root_descriptor)
    except Exception:
        for relative in reversed(created):
            try:
                os.unlink(relative, dir_fd=root_descriptor)
            except FileNotFoundError:
                pass
        try:
            os.fsync(root_descriptor)
        except OSError:
            pass
        raise
    finally:
        os.close(root_descriptor)


def _finalize_cell_output_tree(cell_root: str | os.PathLike[str]) -> None:
    root_text = _absolute_path_text(_path(cell_root, "cell_output_root"), "cell_output_root")
    root_descriptor = _open_directory_path_nofollow(root_text, "cell_output_root")

    def walk(*, apply_modes: bool) -> None:
        accounting = _CellOutputAccounting()
        inodes: set[tuple[int, int]] = set()
        root_metadata = _stable_metadata(os.fstat(root_descriptor))
        stack = [_CellOutputTraversalFrame(
            "", 0, root_descriptor,
            _bounded_cell_output_directory_names(root_descriptor, root_text),
            0, root_metadata, False,
        )]
        try:
            while stack:
                frame = stack[-1]
                if frame.next_index == len(frame.names):
                    stack.pop()
                    if apply_modes:
                        os.fchmod(frame.descriptor, 0o555)
                        os.fsync(frame.descriptor)
                    if frame.owns_descriptor:
                        os.close(frame.descriptor)
                    continue
                name = frame.names[frame.next_index]
                stack[-1] = _CellOutputTraversalFrame(
                    frame.path, frame.depth, frame.descriptor, frame.names,
                    frame.next_index + 1, frame.baseline_metadata, frame.owns_descriptor,
                )
                _safe_component(name, "cell_output_member")
                relative = f"{frame.path}/{name}" if frame.path else name
                depth = frame.depth + 1
                accounting._validate_descendant(depth)
                info = os.stat(name, dir_fd=frame.descriptor, follow_symlinks=False)
                if stat.S_ISLNK(info.st_mode):
                    _fail("cell_output_symlink", "cell output cannot contain symlinks", relative)
                inode = (info.st_dev, info.st_ino)
                if inode in inodes:
                    _fail("cell_output_inode_alias", "cell output contains a hardlink or directory alias", relative)
                inodes.add(inode)
                if stat.S_ISDIR(info.st_mode):
                    accounting = accounting.add_directory(depth)
                    child = _open_private_directory_at(frame.descriptor, name, "cell_output_traversal_failed")
                    try:
                        names = _bounded_cell_output_directory_names(child, relative)
                        stack.append(_CellOutputTraversalFrame(
                            relative, depth, child, names, 0,
                            _stable_metadata(os.fstat(child)), True,
                        ))
                        child = None
                    finally:
                        if child is not None:
                            os.close(child)
                elif stat.S_ISREG(info.st_mode) and info.st_nlink == 1:
                    accounting = accounting.add_file(
                        depth, info.st_size,
                        semantic=relative in _INITIAL_CELL_OUTPUT_SEMANTIC_PATHS,
                        cache_semantic=False,
                    )
                    if apply_modes:
                        member = os.open(
                            name,
                            os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0),
                            dir_fd=frame.descriptor,
                        )
                        try:
                            if _stable_metadata(os.fstat(member))[:2] != _stable_metadata(info)[:2]:
                                _fail("cell_output_changed", "member changed while finalizing", relative)
                            os.fchmod(member, 0o444)
                            os.fsync(member)
                        finally:
                            os.close(member)
                else:
                    _fail("cell_output_member_type", "cell output member must be a unique regular file", relative)
        finally:
            for frame in reversed(stack):
                if frame.owns_descriptor:
                    os.close(frame.descriptor)

    try:
        walk(apply_modes=False)
        walk(apply_modes=True)
    except Slice7GRuntimeError:
        raise
    except Exception as exc:
        raise Slice7GRuntimeError("cell_output_finalize", str(exc), path=root_text) from exc
    finally:
        os.close(root_descriptor)


def _validate_process_output_receipt(
    authority: _CellOutputAuthority, observation: Slice7GProcessObservation,
) -> None:
    raw = authority.member_bytes(PROCESS_OUTPUT_RECEIPT_PATH)
    receipt = _parse_json(raw, "process_output_receipt_json")
    if raw != _canonical(receipt):
        _fail("process_output_receipt_noncanonical", "process output receipt must be canonical")
    _closed(receipt, {"schema_version", "argv", "process_exit_status", "streams"}, "process_output_receipt_fields")
    if receipt["schema_version"] != PROCESS_OUTPUT_RECEIPT_SCHEMA:
        _fail("process_output_receipt_schema", "unsupported process output receipt schema")
    if receipt["argv"] != list(observation.argv) or receipt["process_exit_status"] != observation.returncode:
        _fail("process_output_receipt_binding", "process output receipt differs from the observation")
    streams = receipt["streams"]
    if type(streams) is not list or len(streams) != 2:
        _fail("process_output_receipt_streams", "process output receipt requires stdout and stderr")
    expected = ((PROCESS_STDOUT_PATH, observation.stdout), (PROCESS_STDERR_PATH, observation.stderr))
    for record, (path, expected_raw) in zip(streams, expected):
        if type(record) is not dict:
            _fail("process_output_receipt_streams", "process stream descriptor must be an exact object")
        _closed(record, {"path", "size", "sha256"}, "process_output_receipt_streams")
        retained = authority.member_observation(path)
        expected_digest = hashlib.sha256(expected_raw).hexdigest()
        if (
            record["path"] != path
            or type(record["size"]) is not int
            or record["size"] != len(expected_raw)
            or record["sha256"] != expected_digest
            or retained.size != len(expected_raw)
            or retained.sha256 != expected_digest
            or retained.semantic_bytes is not None
        ):
            _fail("process_output_receipt_binding", "retained process stream differs from captured bytes", path)


def _readiness_from_orchestration(orchestration: dict[str, Any]) -> Slice7GReadinessResult:
    if type(orchestration) is not dict:
        _fail("runner_readiness", "orchestration must be an exact object")
    diagnostics = _section(orchestration, "readiness_diagnostics")
    stability = _section(orchestration, "initial_state_stability")
    criteria = _section(diagnostics, "criteria")
    snapshot = _section(diagnostics, "slice_7g_readiness_snapshot")
    required_criteria = ("finite_values", "sample_count", "duration", "q_variation", "tip_variation")
    if any(criteria.get(name) is not True for name in required_criteria):
        _fail("runner_readiness", "runner readiness criteria did not all pass")
    if diagnostics.get("readiness_result") is not True or snapshot.get("authenticated") is not True:
        _fail("runner_readiness", "runner did not retain authenticated Slice 7G readiness")
    if snapshot.get("tactile_valid") is not True or snapshot.get("safety_ready") is not True or snapshot.get("safety_fault") is not False:
        _fail("runner_readiness", "retained tactile/safety readiness is not healthy")
    sample_count = _exact_nonnegative_int(stability.get("sample_count"), "readiness_sample_count")
    interval = _finite(stability.get("duration_s"), "readiness_interval")
    q_variation = _finite(stability.get("max_q_variation"), "readiness_q_variation")
    tip_variation = _finite(stability.get("max_tip_variation"), "readiness_tip_variation")
    tactile_age = _finite(snapshot.get("tactile_receive_age_seconds"), "readiness_tactile_age")
    safety_age = _finite(snapshot.get("safety_receive_age_seconds"), "readiness_safety_age")
    passed = (
        sample_count >= 10 and interval >= 0.5 and q_variation <= 5.0e-5
        and tip_variation <= 5.0e-5 and 0.0 <= tactile_age <= 0.10
        and 0.0 <= safety_age <= 0.10
    )
    return Slice7GReadinessResult(
        passed, "" if passed else "runner_readiness_contract", sample_count, interval,
        q_variation, tip_variation, tactile_age, safety_age,
    )


def _make_domain_observation(
    source: str,
    domain_id: int,
    clear: bool,
    observation: dict[str, Any],
    observed_at_utc: str,
) -> Slice7GDomainObservationReceipt:
    _identifier(source, "domain_observation_source")
    if type(domain_id) is not int or not DOMAIN_MINIMUM <= domain_id <= DOMAIN_MAXIMUM:
        _fail("domain_observation_record", "domain observation is outside 100..199")
    if type(clear) is not bool or type(observation) is not dict:
        _fail("domain_observation_record", "domain observation fields are malformed")
    detached = _detach(observation)
    timestamp = _utc(observed_at_utc, "domain_observed_at_utc")
    observation_sha256 = hashlib.sha256(_canonical(detached)).hexdigest()
    data = {
        "schema_version": DOMAIN_OBSERVATION_SCHEMA,
        "source": source,
        "domain_id": domain_id,
        "clear": clear,
        "observed_at_utc": timestamp,
        "observation_sha256": observation_sha256,
    }
    identity = hashlib.sha256(DOMAIN_OBSERVATION_DOMAIN + _canonical(data)).hexdigest()
    return Slice7GDomainObservationReceipt(identity=identity, **data)


def _production_timestamp(effects: Slice7GProductionEffects, code: str) -> str:
    try:
        value = effects.utc_now()
    except Slice7GRuntimeError:
        raise
    except Exception as exc:
        raise Slice7GRuntimeError(code, "production timestamp provider failed") from exc
    return _utc(value, code)


def _domain_observation_data(value: Any) -> dict[str, Any]:
    if type(value) is not Slice7GDomainObservationReceipt:
        _fail("domain_observation_record", "observation must be an exact repository record")
    try:
        data = {
            "schema_version": value.schema_version,
            "source": value.source,
            "domain_id": value.domain_id,
            "clear": value.clear,
            "observed_at_utc": value.observed_at_utc,
            "observation_sha256": value.observation_sha256,
        }
        identity = value.identity
    except AttributeError as exc:
        raise Slice7GRuntimeError(
            "domain_observation_record", "observation is partially initialized",
        ) from exc
    if data["schema_version"] != DOMAIN_OBSERVATION_SCHEMA:
        _fail("domain_observation_record", "unsupported observation schema")
    _identifier(data["source"], "domain_observation_source")
    if type(data["domain_id"]) is not int or not DOMAIN_MINIMUM <= data["domain_id"] <= DOMAIN_MAXIMUM:
        _fail("domain_observation_record", "observation domain is invalid")
    if type(data["clear"]) is not bool:
        _fail("domain_observation_record", "observation decision must be an exact bool")
    _utc(data["observed_at_utc"], "domain_observed_at_utc")
    _digest(data["observation_sha256"], "domain_observation_sha256")
    expected = hashlib.sha256(DOMAIN_OBSERVATION_DOMAIN + _canonical(data)).hexdigest()
    if identity != expected:
        _fail("domain_observation_identity", "observation identity is invalid")
    return {**data, "identity": expected}


def _validated_process_observation(
    value: Any, expected_argv: tuple[str, ...], code: str,
) -> Slice7GProcessObservation:
    if type(value) is not Slice7GProcessObservation:
        _fail(code, "process observer returned an unsupported record")
    try:
        argv = value.argv
        returncode = value.returncode
        stdout = value.stdout
        stderr = value.stderr
    except AttributeError as exc:
        raise Slice7GRuntimeError(code, "process observation is partially initialized") from exc
    if (
        type(argv) is not tuple
        or argv != expected_argv
        or any(type(item) is not str for item in argv)
        or type(returncode) is not int
        or type(returncode) is bool
        or type(stdout) is not bytes
        or type(stderr) is not bytes
    ):
        _fail(code, "process observation fields are malformed")
    return Slice7GProcessObservation(tuple(argv), returncode, bytes(stdout), bytes(stderr))


def _validated_graph_observer_contract(value: Any) -> Slice7GROSGraphObserverContract:
    if type(value) is not Slice7GROSGraphObserverContract:
        _fail("observer_contract", "graph observer requires an exact installed-process contract")
    if value.executable != PRECOMMIT_ROS_GRAPH_OBSERVER_EXECUTABLE:
        _fail("observer_executable", "graph observer executable differs")
    expected_argv = (PRECOMMIT_ROS_GRAPH_OBSERVER_EXECUTABLE, *PRECOMMIT_ROS_GRAPH_OBSERVER_ARGV)
    if type(value.argv) is not tuple or value.argv != expected_argv or any(type(item) is not str for item in value.argv):
        _fail("observer_argv", "graph observer argv differs")
    if value.interpreter != "/usr/bin/python3":
        _fail("observer_interpreter", "graph observer interpreter differs")
    for label, digest in (
        ("executable_identity", value.executable_identity),
        ("interpreter_identity", value.interpreter_identity),
        ("environment_identity", value.environment_identity),
    ):
        _digest(digest, label)
    if (
        type(value.module_origin_identities) is not tuple
        or not value.module_origin_identities
        or tuple(sorted(value.module_origin_identities)) != value.module_origin_identities
        or len(set(value.module_origin_identities)) != len(value.module_origin_identities)
    ):
        _fail("observer_module_origins", "module-origin identities must be unique and sorted")
    for item in value.module_origin_identities:
        _digest(item, "observer_module_origin_identity")
    if type(value.environment) is not tuple or any(
        type(item) is not tuple or len(item) != 2 or any(type(part) is not str for part in item)
        for item in value.environment
    ):
        _fail("observer_environment", "observer environment must be a detached tuple")
    keys = tuple(key for key, _ in value.environment)
    if keys != tuple(sorted(keys)) or len(keys) != len(set(keys)):
        _fail("observer_environment", "observer environment keys must be unique and sorted")
    environment = dict(value.environment)
    required = {
        "AMENT_PREFIX_PATH", "HOME", "LD_LIBRARY_PATH", "PATH", "PYTHONNOUSERSITE",
        "PYTHONDONTWRITEBYTECODE", "PYTHONPATH", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID",
        "ROS_HOME", "ROS_LOCALHOST_ONLY", "XDG_CACHE_HOME",
    }
    if set(environment) != required:
        _fail("observer_environment", "observer environment key set differs")
    if environment["PATH"] != "/opt/ros/humble/bin:/usr/bin" or environment["ROS_LOCALHOST_ONLY"] != "1":
        _fail("observer_environment", "observer fixed environment differs")
    if environment["PYTHONDONTWRITEBYTECODE"] != "1" or environment["PYTHONNOUSERSITE"] != "1":
        _fail("observer_environment", "observer Python isolation differs")
    if environment["RMW_IMPLEMENTATION"] != value.rmw_implementation:
        _fail("observer_environment", "observer RMW binding differs")
    try:
        domain = int(environment["ROS_DOMAIN_ID"])
    except ValueError as exc:
        raise Slice7GRuntimeError("observer_environment", "ROS_DOMAIN_ID is not canonical") from exc
    if str(domain) != environment["ROS_DOMAIN_ID"] or not DOMAIN_MINIMUM <= domain <= DOMAIN_MAXIMUM:
        _fail("observer_environment", "ROS_DOMAIN_ID is outside 100..199")
    _absolute_path_text(value.working_directory, "observer_working_directory")
    if value.cgroup != "/system.slice/ctr-slice7g-campaign.service":
        _fail("observer_cgroup", "observer must remain in the fixed campaign cgroup")
    return value


def parse_ros_graph_observer_stdout(raw: bytes) -> tuple[str, ...]:
    if type(raw) is not bytes:
        _fail("observer_stdout_type", "observer stdout must be exact bytes")
    if len(raw) > OBSERVER_STDOUT_LIMIT_BYTES:
        _fail("observer_stdout_size", "observer stdout exceeds 1048576 bytes")
    if b"\0" in raw or b"\r" in raw:
        _fail("observer_stdout_format", "observer stdout contains NUL or CR")
    try:
        text = raw.decode("utf-8", "strict")
    except UnicodeError as exc:
        raise Slice7GRuntimeError("observer_stdout_utf8", "observer stdout is not strict UTF-8") from exc
    if text.endswith("\n"):
        text = text[:-1]
    if "\n" in text and any(not line for line in text.split("\n")):
        _fail("observer_stdout_empty_line", "observer stdout contains an empty interior line")
    lines = () if text == "" else tuple(text.split("\n"))
    if len(lines) > 65_536:
        _fail("observer_node_count", "observer node count exceeds 65536")
    seen: set[str] = set()
    for index, node in enumerate(lines):
        if len(node.encode("utf-8")) > 8_192:
            _fail("observer_node_length", "observer node line exceeds 8192 bytes", str(index))
        if unicodedata.normalize("NFC", node) != node:
            _fail("observer_node_nfc", "observer node is not NFC", str(index))
        if any(ord(character) < 0x20 or ord(character) == 0x7F for character in node):
            _fail("observer_node_control", "observer node contains a control character", str(index))
        if not node.startswith("/") or node == "/":
            _fail("observer_node_absolute", "observer node is not absolute", str(index))
        components = node[1:].split("/")
        if any(not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", part) for part in components):
            _fail("observer_node_component", "observer node component is malformed", str(index))
        if node in seen:
            _fail("observer_node_duplicate", "observer node is duplicated", str(index))
        seen.add(node)
    return lines


def _observer_cleanup_barrier(
    effects: Slice7GProductionEffects,
    execution: Slice7GROSGraphObserverExecution,
    domain_id: int,
) -> str:
    start = effects.monotonic()
    clean_samples: list[dict[str, Any]] = []
    last_clean_at: float | None = None
    while effects.monotonic() - start <= OBSERVER_CLEANUP_MAXIMUM_WAIT_SECONDS:
        try:
            sample = effects.observer_cleanup_sample(execution, domain_id)
        except Slice7GRuntimeError:
            raise
        except Exception as exc:
            raise Slice7GRuntimeError("observer_cleanup_provider", str(exc)) from exc
        if type(sample) is not dict or set(sample) != {
            "process_present", "process_group_present", "descendant_pids",
            "ros_daemon_pids", "matching_udp_ports",
        }:
            _fail("observer_cleanup_provider", "observer cleanup sample is malformed")
        if (
            type(sample["process_present"]) is not bool
            or type(sample["process_group_present"]) is not bool
            or any(
            type(sample[field]) is not tuple
            or any(type(item) is not int or item < 0 for item in sample[field])
            for field in ("descendant_pids", "ros_daemon_pids", "matching_udp_ports")
            )
        ):
            _fail("observer_cleanup_provider", "observer cleanup sample fields are malformed")
        clean = (
            not sample["process_present"]
            and not sample["process_group_present"]
            and not sample["descendant_pids"]
            and not sample["ros_daemon_pids"]
            and not sample["matching_udp_ports"]
        )
        now = effects.monotonic()
        if clean:
            clean_samples.append(_detach(sample))
            if last_clean_at is None:
                last_clean_at = now
            if (
                len(clean_samples) >= OBSERVER_CLEANUP_STABLE_SAMPLES
                and now - last_clean_at >= OBSERVER_CLEANUP_MINIMUM_INTERVAL_SECONDS
            ):
                projection = {
                    "schema_version": "ctr-slice-7g-observer-cleanup-barrier-1",
                    "domain_id": domain_id,
                    "observer_pid": execution.pid,
                    "observer_start_time_ticks": execution.process_start_time_ticks,
                    "stable_samples": clean_samples[-OBSERVER_CLEANUP_STABLE_SAMPLES:],
                    "minimum_interval_seconds": OBSERVER_CLEANUP_MINIMUM_INTERVAL_SECONDS,
                }
                return hashlib.sha256(
                    b"ctr-slice-7g-observer-cleanup-barrier-canonical-1\0"
                    + _canonical(projection)
                ).hexdigest()
        else:
            clean_samples.clear()
            last_clean_at = None
        remaining = OBSERVER_CLEANUP_MAXIMUM_WAIT_SECONDS - (effects.monotonic() - start)
        if remaining <= 0:
            break
        effects.sleep(min(OBSERVER_CLEANUP_MINIMUM_INTERVAL_SECONDS, remaining))
    _fail("observer_cleanup_uncertain", "observer process/DDS residual barrier did not clear")


def _process_start_time_ticks(pid: int) -> int:
    try:
        value = Path(f"/proc/{pid}/stat").read_text(encoding="ascii").rsplit(") ", 1)[1].split()[19]
        ticks = int(value)
    except (OSError, UnicodeError, ValueError, IndexError) as exc:
        raise Slice7GRuntimeError("observer_process_identity", "cannot retain observer start time") from exc
    if ticks <= 0:
        _fail("observer_process_identity", "observer start time is not positive")
    return ticks


def _bounded_process_streams(
    process: subprocess.Popen,
    *,
    timeout_seconds: float,
    stdout_limit: int,
    stderr_limit: int,
    process_guard: Callable[[], None],
) -> tuple[bytes, bytes]:
    if process.stdout is None or process.stderr is None:
        _fail("observer_pipe", "observer pipes are unavailable")
    selector = selectors.DefaultSelector()
    buffers = {process.stdout: bytearray(), process.stderr: bytearray()}
    limits = {process.stdout: stdout_limit, process.stderr: stderr_limit}
    deadline = time.monotonic() + timeout_seconds
    try:
        for stream in buffers:
            os.set_blocking(stream.fileno(), False)
            selector.register(stream, selectors.EVENT_READ)
        while selector.get_map():
            process_guard()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise Slice7GRuntimeError("observer_timeout", "graph observer exceeded 10 seconds")
            ready = selector.select(remaining)
            if not ready and process.poll() is None:
                raise Slice7GRuntimeError("observer_timeout", "graph observer exceeded 10 seconds")
            for key, _ in ready:
                stream = key.fileobj
                chunk = os.read(stream.fileno(), 65_536)
                if not chunk:
                    selector.unregister(stream)
                    stream.close()
                    continue
                buffers[stream].extend(chunk)
                if len(buffers[stream]) > limits[stream]:
                    raise Slice7GRuntimeError("observer_output_size", "graph observer output exceeds its bound")
        return bytes(buffers[process.stdout]), bytes(buffers[process.stderr])
    finally:
        selector.close()
        for stream in buffers:
            if not stream.closed:
                stream.close()


def _descendant_process_records(
    parent_pid: int, records: tuple[dict[str, Any], ...],
) -> tuple[dict[str, Any], ...]:
    if type(parent_pid) is not int or parent_pid <= 0 or type(records) is not tuple:
        _fail("observer_process_tree", "observer process-tree input is malformed")
    descendants: list[dict[str, Any]] = []
    frontier = {parent_pid}
    remaining = list(records)
    while frontier:
        next_frontier: set[int] = set()
        retained: list[dict[str, Any]] = []
        for record in remaining:
            if type(record) is not dict:
                _fail("observer_process_tree", "process-tree record is malformed")
            if record.get("parent_pid") in frontier:
                pid = record.get("pid")
                if type(pid) is not int or pid <= 0:
                    _fail("observer_process_tree", "descendant PID is malformed")
                descendants.append(record)
                next_frontier.add(pid)
            else:
                retained.append(record)
        remaining = retained
        frontier = next_frontier
    return tuple(descendants)


def _udp_ports_from_proc_tables(tables: tuple[bytes, bytes]) -> set[int]:
    ports: set[int] = set()
    for raw in tables:
        try:
            lines = raw.decode("ascii", errors="strict").splitlines()
        except UnicodeError as exc:
            raise Slice7GRuntimeError("dds_socket_observation", "UDP table is not ASCII") from exc
        for line in lines[1:]:
            columns = line.split()
            if len(columns) < 2 or ":" not in columns[1]:
                _fail("dds_socket_observation", "UDP socket table is malformed")
            port_text = columns[1].rsplit(":", 1)[1]
            try:
                port = int(port_text, 16)
            except ValueError as exc:
                raise Slice7GRuntimeError("dds_socket_observation", "UDP port is malformed") from exc
            if not 0 <= port <= 65535:
                _fail("dds_socket_observation", "UDP port is outside the valid range")
            ports.add(port)
    return ports


def _ensure_private_directory(path: Path) -> None:
    """Create one repository-owned control directory with a stable private mode."""

    _require_real_directory(path.parent, "control_directory_parent")
    try:
        path.mkdir(mode=0o700)
    except FileExistsError:
        pass
    except OSError as exc:
        raise Slice7GRuntimeError("control_directory_create", str(exc), path=str(path)) from exc
    _require_real_directory(path, "control_directory")
    try:
        mode = stat.S_IMODE(path.stat().st_mode)
    except OSError as exc:
        raise Slice7GRuntimeError("control_directory_stat", str(exc), path=str(path)) from exc
    if mode != 0o700:
        _fail("control_directory_mode", "control directory must have mode 0700", str(path))
    _fsync_directory(path.parent)


def _physical_package_identity(root: Path, projection_identity: str) -> str:
    envelope = root / EVIDENCE_ENVELOPE_PATH
    projection = root / EVIDENCE_PROJECTION_PATH
    envelope_raw = envelope.read_bytes()
    projection_raw = projection.read_bytes()
    payload = {
        "root": {"path": ".", "type": "directory", "mode": stat.S_IMODE(root.stat().st_mode)},
        "envelope": {"path": EVIDENCE_ENVELOPE_PATH, "type": "regular_file", "mode": 0o444,
                     "link_count": 1, "size": len(envelope_raw), "sha256": hashlib.sha256(envelope_raw).hexdigest()},
        "projection": {"path": EVIDENCE_PROJECTION_PATH, "type": "regular_file", "mode": 0o444,
                       "link_count": 1, "size": len(projection_raw), "sha256": hashlib.sha256(projection_raw).hexdigest()},
        "projection_identity": projection_identity,
        "schema_version": "ctr-slice-7g-cell-evidence-package-physical-1",
    }
    return hashlib.sha256(CELL_EVIDENCE_PACKAGE_IDENTITY_DOMAIN + _canonical(payload)).hexdigest()


def _planned_cell(plan: Slice7GCampaignPlan, cell_id: str) -> Slice7GCampaignCell:
    matches = [cell for cell in plan.cells if cell.cell_id == cell_id]
    if len(matches) != 1:
        _fail("planned_cell", "result must bind exactly one planned cell")
    return matches[0]


def _exclusive_sealed_file(path: Path, raw: bytes) -> None:
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0), 0o600)
    try:
        _write_all(descriptor, raw)
        os.fsync(descriptor)
        os.fchmod(descriptor, 0o444)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _commit_noreplace_file(parent: Path, final: Path, raw: bytes, conflict_code: str) -> None:
    temp = parent / f".{final.name}.{os.getpid()}.{id(raw):x}.tmp"
    descriptor: int | None = None
    try:
        descriptor = os.open(
            temp, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0), 0o600,
        )
        _write_all(descriptor, raw)
        os.fsync(descriptor)
        os.fchmod(descriptor, 0o444)
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = None
        try:
            os.link(temp, final, follow_symlinks=False)
        except FileExistsError as exc:
            raise Slice7GRuntimeError(conflict_code, "record already committed", path=str(final)) from exc
        os.unlink(temp)
        _fsync_directory(parent)
    except Exception:
        if descriptor is not None:
            os.close(descriptor)
        try:
            temp.unlink()
        except FileNotFoundError:
            pass
        raise


def _require_real_directory(path: Path, label: str) -> None:
    try:
        info = path.lstat()
    except OSError as exc:
        raise Slice7GRuntimeError("directory_open", str(exc), path=str(path)) from exc
    if not stat.S_ISDIR(info.st_mode) or path.is_symlink():
        _fail("directory_type", f"{label} must be a real directory", str(path))


def _ensure_owned_directory(path: Path) -> None:
    """Create one producer-owned directory and reject symlink substitutions."""

    parent = path.parent
    _require_real_directory(parent, "directory_parent")
    try:
        path.mkdir(mode=0o700)
    except FileExistsError:
        pass
    except OSError as exc:
        raise Slice7GRuntimeError("directory_create", str(exc), path=str(path)) from exc
    _require_real_directory(path, "producer_directory")


def _rename_noreplace(
    source: Path, target: Path, *, conflict_code: str = "evidence_package_exists",
) -> None:
    if target.exists():
        _fail(conflict_code, "target already exists", str(target))
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        _fail("rename_noreplace_unavailable", "renameat2 is required for no-replace package publication")
    renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    renameat2.restype = ctypes.c_int
    result = renameat2(-100, os.fsencode(source), -100, os.fsencode(target), 1)
    if result != 0:
        error = ctypes.get_errno()
        raise Slice7GRuntimeError("evidence_package_commit", os.strerror(error), path=str(target))


def _make_tree_removable(root: Path) -> None:
    for path in sorted(root.rglob("*"), reverse=True):
        try:
            path.chmod(0o700 if path.is_dir() else 0o600)
        except OSError:
            pass
    try:
        root.chmod(0o700)
    except OSError:
        pass


def _section(value: dict[str, Any], name: str) -> dict[str, Any]:
    item = value.get(name)
    if type(item) is not dict:
        _fail("cell_summary_section", f"missing exact {name} section")
    return item


def _required_field(value: dict[str, Any], name: str, section: str) -> Any:
    if name not in value:
        _fail("cell_summary_field", f"{section}.{name} is required")
    return value[name]


def _parse_json(raw: bytes, code: str) -> dict[str, Any]:
    if type(raw) is not bytes:
        _fail(code, "input must be exact bytes")
    try:
        value = json.loads(raw.decode("utf-8"), object_pairs_hook=_pairs, parse_constant=lambda value: _fail(code, f"nonfinite {value}"))
    except Slice7GRuntimeError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise Slice7GRuntimeError(code, str(exc)) from exc
    if type(value) is not dict:
        _fail(code, "top-level JSON must be an object")
    return value


def _pairs(items: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in items:
        if key in value:
            _fail("duplicate_json_key", f"duplicate JSON key: {key}")
        value[key] = item
    return value


def _detach(value: Any) -> Any:
    if type(value) is dict:
        return {key: _detach(item) for key, item in dict.items(value)}
    if type(value) in (list, tuple):
        return [_detach(item) for item in value]
    if value is None or type(value) in (str, bool, int):
        return value
    if type(value) is float and math.isfinite(value):
        return value
    _fail("primitive_type", f"unsupported primitive {type(value).__name__}")


def _canonical(value: Any) -> bytes:
    try:
        return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise Slice7GRuntimeError("canonical_json", str(exc)) from exc


def _closed(value: dict[str, Any], expected: set[str], code: str) -> None:
    if type(value) is not dict or set(value) != expected:
        _fail(code, "object has missing or unknown fields")


def _identifier(value: Any, label: str) -> str:
    if type(value) is not str or not SAFE_ID.fullmatch(value):
        _fail("identifier", f"{label} is invalid")
    return value


def _digest(value: Any, label: str) -> str:
    if type(value) is not str or not DIGEST.fullmatch(value):
        _fail("digest", f"{label} must be a lowercase SHA-256")
    return value


def _utc(value: Any, label: str) -> str:
    if type(value) is not str or not value.endswith("Z"):
        _fail("utc_timestamp", f"{label} must use UTC Z form")
    try:
        parsed = datetime.fromisoformat(value[:-1] + "+00:00")
    except ValueError as exc:
        raise Slice7GRuntimeError("utc_timestamp", f"invalid {label}") from exc
    if parsed.tzinfo != timezone.utc:
        _fail("utc_timestamp", f"{label} must be UTC")
    return value


def _path(value: Any, label: str) -> str:
    try:
        raw = os.fspath(value)
    except Exception as exc:
        raise Slice7GRuntimeError("path_type", f"{label} conversion failed") from exc
    if type(raw) is not str or not raw or "\x00" in raw or "\\" in raw or unicodedata.normalize("NFC", raw) != raw:
        _fail("path_type", f"{label} is not a supported path")
    return raw


def _absolute_path_text(value: Any, label: str) -> str:
    text = _path(value, label)
    if not text.startswith("/") or "//" in text or text.endswith("/") or any(part in {"", ".", ".."} for part in text[1:].split("/")):
        _fail("absolute_path", f"{label} must be a canonical absolute path")
    return text


def _safe_relative(value: Any, label: str) -> str:
    if type(value) is not str or not value or value.startswith("/") or "\\" in value or "//" in value:
        _fail("relative_path", f"{label} must be a safe relative path")
    if any(part in {"", ".", ".."} for part in value.split("/")):
        _fail("relative_path", f"{label} contains traversal")
    return value


def _finite(value: Any, label: str) -> float:
    if type(value) not in (int, float) or type(value) is bool or not math.isfinite(float(value)):
        _fail("finite_number", f"{label} must be finite")
    return float(value)


def _finite_tuple(value: Sequence[float], size: int, label: str) -> tuple[float, ...]:
    if type(value) not in (list, tuple) or len(value) != size:
        _fail("vector", f"{label} must have {size} entries")
    return tuple(_finite(item, label) for item in value)


def _exact_nonnegative_int(value: Any, label: str) -> int:
    if type(value) is not int or value < 0:
        _fail("count", f"{label} must be a nonnegative exact integer")
    return value


def _write_all(descriptor: int, raw: bytes) -> None:
    view = memoryview(raw)
    while view:
        written = os.write(descriptor, view)
        if written <= 0:
            _fail("write_failed", "short filesystem write")
        view = view[written:]


def _read_sealed_file_nofollow(path_text: str) -> bytes:
    absolute = path_text.startswith("/")
    components = path_text[1:].split("/") if absolute else path_text.split("/")
    if not components or any(component in {"", ".", ".."} for component in components):
        _fail("sealed_path", "sealed file path is not canonical", path_text)
    directory = os.open(
        "/" if absolute else ".",
        os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_CLOEXEC", 0),
    )
    opened: list[int] = [directory]
    member: int | None = None
    try:
        for component in components[:-1]:
            child = os.open(
                component,
                os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
                | getattr(os, "O_CLOEXEC", 0),
                dir_fd=directory,
            )
            opened.append(child)
            directory = child
        member = os.open(
            components[-1],
            os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0),
            dir_fd=directory,
        )
        before = os.fstat(member)
        if (
            not stat.S_ISREG(before.st_mode)
            or stat.S_IMODE(before.st_mode) != 0o444
            or before.st_nlink != 1
        ):
            _fail("sealed_file_metadata", "sealed record must be regular mode 0444 with link count one", path_text)
        chunks: list[bytes] = []
        while True:
            chunk = os.read(member, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(member)
        stable = lambda value: (
            value.st_dev, value.st_ino, value.st_mode, value.st_nlink, value.st_size,
            value.st_mtime_ns, value.st_ctime_ns,
        )
        if stable(before) != stable(after) or sum(map(len, chunks)) != before.st_size:
            _fail("sealed_file_changed", "sealed record changed while being read", path_text)
        return b"".join(chunks)
    finally:
        if member is not None:
            os.close(member)
        for descriptor in reversed(opened):
            os.close(descriptor)


def _stable_metadata(value: os.stat_result) -> tuple[int, int, int, int, int, int, int]:
    return (
        value.st_dev, value.st_ino, value.st_mode, value.st_nlink, value.st_size,
        value.st_mtime_ns, value.st_ctime_ns,
    )


def _directory_identity(
    metadata: tuple[int, int, int, int, int, int, int],
) -> tuple[int, int, int]:
    return metadata[0], metadata[1], metadata[2]


def _safe_component(value: Any, label: str) -> str:
    if (
        type(value) is not str
        or not value
        or value in {".", ".."}
        or "/" in value
        or "\\" in value
        or "\x00" in value
        or unicodedata.normalize("NFC", value) != value
        or any(ord(character) < 0x20 or ord(character) == 0x7f for character in value)
    ):
        _fail("path_component", f"{label} contains an unsafe component")
    return value


def _open_directory_path_nofollow(path_text: str, code: str) -> int:
    text = _absolute_path_text(path_text, code)
    components = text[1:].split("/")
    current = os.open(
        "/", os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_CLOEXEC", 0),
    )
    try:
        for component in components:
            child = os.open(
                _safe_component(component, code),
                os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
                | getattr(os, "O_CLOEXEC", 0),
                dir_fd=current,
            )
            os.close(current)
            current = child
        info = os.fstat(current)
        if not stat.S_ISDIR(info.st_mode):
            _fail(code, "path does not identify a real directory", text)
        return current
    except Slice7GRuntimeError:
        os.close(current)
        raise
    except OSError as exc:
        os.close(current)
        raise Slice7GRuntimeError(code, str(exc), path=text) from exc


def _open_private_directory_at(parent: int, component: str, code: str) -> int:
    name = _safe_component(component, code)
    try:
        descriptor = os.open(
            name,
            os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
            | getattr(os, "O_CLOEXEC", 0),
            dir_fd=parent,
        )
    except FileNotFoundError as exc:
        raise Slice7GRuntimeError(f"{code}_missing", "required directory is missing", path=name) from exc
    except OSError as exc:
        raise Slice7GRuntimeError(code, str(exc), path=name) from exc
    try:
        opened = os.fstat(descriptor)
        by_name = os.stat(name, dir_fd=parent, follow_symlinks=False)
        if not stat.S_ISDIR(opened.st_mode) or _stable_metadata(opened) != _stable_metadata(by_name):
            _fail(code, "directory entry changed while opening", name)
        return descriptor
    except Exception:
        os.close(descriptor)
        raise


def _open_or_create_private_directory_at(parent: int, component: str, code: str) -> int:
    name = _safe_component(component, code)
    try:
        os.mkdir(name, 0o700, dir_fd=parent)
        os.fsync(parent)
    except FileExistsError:
        pass
    except OSError as exc:
        raise Slice7GRuntimeError(code, str(exc), path=name) from exc
    descriptor = _open_private_directory_at(parent, name, code)
    if stat.S_IMODE(os.fstat(descriptor).st_mode) != 0o700:
        os.close(descriptor)
        _fail(f"{code}_mode", "private directory must use mode 0700", name)
    return descriptor


def _authenticate_domain_registry_lock(registry: int) -> int:
    try:
        descriptor = os.open(
            GLOBAL_DOMAIN_LEASE_LOCK_NAME,
            os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW,
            dir_fd=registry,
        )
    except OSError as exc:
        raise Slice7GRuntimeError("domain_registry_lock", str(exc)) from exc
    try:
        opened = os.fstat(descriptor)
        by_name = os.stat(
            GLOBAL_DOMAIN_LEASE_LOCK_NAME, dir_fd=registry, follow_symlinks=False,
        )
        if (
            not stat.S_ISREG(opened.st_mode)
            or opened.st_nlink != 1
            or stat.S_IMODE(opened.st_mode) != 0o600
            or _stable_metadata(opened) != _stable_metadata(by_name)
        ):
            _fail("domain_registry_lock", "domain registry lock identity differs")
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def _read_all_descriptor(descriptor: int) -> bytes:
    chunks: list[bytes] = []
    while True:
        chunk = os.read(descriptor, 1024 * 1024)
        if not chunk:
            return b"".join(chunks)
        chunks.append(chunk)


def _stream_cell_output_descriptor(
    descriptor: int, expected_size: int, *, capture: bool, path: str,
) -> tuple[str, bytes | None]:
    """Hash exactly one bounded member without retaining non-semantic bytes."""

    if type(expected_size) is not int or expected_size < 0:
        raise Slice7GRuntimeError(
            "cell_output_file_size_limit", "stream size must be a nonnegative exact integer", path=path,
        )
    maximum = (
        _CELL_OUTPUT_LIMITS.maximum_semantic_file_bytes
        if capture else _CELL_OUTPUT_LIMITS.maximum_file_bytes
    )
    if expected_size > maximum:
        code = "cell_output_semantic_size_limit" if capture else "cell_output_file_size_limit"
        raise Slice7GRuntimeError(code, "cell output member exceeds its stream ceiling", path=path)
    digest = hashlib.sha256()
    retained: list[bytes] | None = [] if capture else None
    remaining = expected_size
    while remaining:
        request = min(_CELL_OUTPUT_LIMITS.stream_hash_chunk_bytes, remaining)
        try:
            chunk = os.read(descriptor, request)
        except Exception as exc:
            raise Slice7GRuntimeError(
                "cell_output_stream_read_failed", "cell output member stream read failed", path=path,
            ) from exc
        if not chunk:
            _fail("cell_output_changed", "cell output member became shorter while hashing", path)
        if len(chunk) > request:
            _fail("cell_output_changed", "cell output stream returned more bytes than requested", path)
        digest.update(chunk)
        if retained is not None:
            retained.append(chunk)
        remaining -= len(chunk)
    try:
        extra = os.read(descriptor, 1)
    except Exception as exc:
        raise Slice7GRuntimeError(
            "cell_output_stream_read_failed", "cell output member final stream read failed", path=path,
        ) from exc
    if extra:
        _fail("cell_output_changed", "cell output member became longer while hashing", path)
    try:
        cached = b"".join(retained) if retained is not None else None
    except Exception as exc:
        raise Slice7GRuntimeError(
            "cell_output_stream_read_failed", "semantic output cache allocation failed", path=path,
        ) from exc
    return digest.hexdigest(), cached


def _bounded_cell_output_directory_names(descriptor: int, path: str) -> tuple[str, ...]:
    names: list[str] = []
    try:
        with os.scandir(descriptor) as entries:
            for entry in entries:
                name = entry.name
                _safe_component(name, "cell_output_member")
                names.append(name)
                if len(names) > _CELL_OUTPUT_LIMITS.maximum_members:
                    raise Slice7GRuntimeError(
                        "cell_output_member_limit",
                        "cell output directory alone exceeds the descendant-member ceiling",
                        path=path,
                    )
    except Slice7GRuntimeError:
        raise
    except Exception as exc:
        raise Slice7GRuntimeError(
            "cell_output_traversal_failed", "cell output directory enumeration failed", path=path,
        ) from exc
    try:
        ordered = tuple(sorted(names))
    except Exception as exc:
        raise Slice7GRuntimeError(
            "cell_output_traversal_failed", "cell output entry ordering failed", path=path,
        ) from exc
    if len(ordered) != len(set(ordered)):
        _fail("cell_output_duplicate_entry", "cell output directory contains duplicate names", path)
    return ordered


def _cell_output_fstat(descriptor: int, path: str) -> os.stat_result:
    try:
        return os.fstat(descriptor)
    except Exception as exc:
        raise Slice7GRuntimeError(
            "cell_output_traversal_failed", "cell output descriptor observation failed", path=path,
        ) from exc


def _read_sealed_file_at(parent: int, name: str, code: str) -> bytes:
    component = _safe_component(name, code)
    descriptor: int | None = None
    try:
        descriptor = os.open(
            component,
            os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0),
            dir_fd=parent,
        )
        before = os.fstat(descriptor)
        by_name = os.stat(component, dir_fd=parent, follow_symlinks=False)
        if (
            not stat.S_ISREG(before.st_mode)
            or stat.S_IMODE(before.st_mode) != 0o444
            or before.st_nlink != 1
            or _stable_metadata(before) != _stable_metadata(by_name)
        ):
            _fail(code, "sealed record metadata is invalid", component)
        raw = _read_all_descriptor(descriptor)
        if _stable_metadata(os.fstat(descriptor)) != _stable_metadata(before) or len(raw) != before.st_size:
            _fail(code, "sealed record changed while reading", component)
        return raw
    except Slice7GRuntimeError:
        raise
    except OSError as exc:
        raise Slice7GRuntimeError(code, str(exc), path=component) from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)


def _exclusive_sealed_file_at(parent: int, name: str, raw: bytes, conflict_code: str) -> None:
    component = _safe_component(name, conflict_code)
    if type(raw) is not bytes:
        _fail("sealed_file_bytes", "sealed file content must be exact bytes", component)
    descriptor: int | None = None
    try:
        descriptor = os.open(
            component,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
            | getattr(os, "O_CLOEXEC", 0),
            0o600,
            dir_fd=parent,
        )
        _write_all(descriptor, raw)
        os.fsync(descriptor)
        os.fchmod(descriptor, 0o444)
        os.fsync(descriptor)
    except FileExistsError as exc:
        raise Slice7GRuntimeError(conflict_code, "sealed output path already exists", path=component) from exc
    except Slice7GRuntimeError:
        raise
    except OSError as exc:
        raise Slice7GRuntimeError("sealed_file_write", str(exc), path=component) from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)


def _commit_noreplace_file_at(parent: int, name: str, raw: bytes, conflict_code: str) -> None:
    final = _safe_component(name, conflict_code)
    temp = f".{final}.{os.getpid()}.{id(raw):x}.tmp"
    descriptor: int | None = None
    try:
        descriptor = os.open(
            temp, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
            | getattr(os, "O_CLOEXEC", 0), 0o600, dir_fd=parent,
        )
        _write_all(descriptor, raw)
        os.fsync(descriptor)
        os.fchmod(descriptor, 0o444)
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = None
        try:
            os.link(temp, final, src_dir_fd=parent, dst_dir_fd=parent, follow_symlinks=False)
        except FileExistsError as exc:
            raise Slice7GRuntimeError(conflict_code, "record already exists", path=final) from exc
        os.unlink(temp, dir_fd=parent)
        os.fsync(parent)
    except Exception:
        if descriptor is not None:
            os.close(descriptor)
        try:
            os.unlink(temp, dir_fd=parent)
        except FileNotFoundError:
            pass
        raise


def _rename_noreplace_at(parent: int, source: str, destination: str, conflict_code: str) -> None:
    source_name = _safe_component(source, conflict_code)
    destination_name = _safe_component(destination, conflict_code)
    try:
        os.link(
            source_name, destination_name, src_dir_fd=parent, dst_dir_fd=parent,
            follow_symlinks=False,
        )
    except FileExistsError as exc:
        raise Slice7GRuntimeError(conflict_code, "destination already exists", path=destination_name) from exc
    try:
        os.unlink(source_name, dir_fd=parent)
    except Exception:
        try:
            os.unlink(destination_name, dir_fd=parent)
        except FileNotFoundError:
            pass
        raise


def _trusted_external_output_parent(charter: Slice7GCharter) -> str:
    if type(charter) is not Slice7GCharter:
        _fail("charter_record", "trusted output parent requires an exact charter")
    try:
        declared = charter.data["evidence_outputs"]["external_parent"]
    except (AttributeError, KeyError, TypeError) as exc:
        raise Slice7GRuntimeError("charter_record", "charter output-parent policy is unavailable") from exc
    if declared != "/home/ankid/ctr_mppi_evidence/slice_7g":
        _fail("charter_output_parent", "authenticated charter contains an unexpected evidence parent")
    configured = _slice_7g_governance.SLICE_7G_EVIDENCE_PARENT
    return _absolute_path_text(configured, "trusted_external_output_parent")


def _require_strict_descendant(value: str, parent: str, code: str) -> str:
    path = _absolute_path_text(value, code)
    trusted = _absolute_path_text(parent, "trusted_external_output_parent")
    if path == trusted or not path.startswith(trusted + "/"):
        _fail(code, "path must be a strict descendant of the charter evidence parent", path)
    return path[len(trusted) + 1:]


def _cleanup_issue(code: str, error: Exception) -> Slice7GCleanupIssue:
    _identifier(code, "cleanup_issue_code")
    if not isinstance(error, Exception):
        _fail("cleanup_issue", "cleanup issue must originate from an ordinary exception")
    return Slice7GCleanupIssue(code, f"{type(error).__name__}:{error}")


def _raise_repository_cleanup(
    primary: BaseException,
    issues: tuple[Slice7GCleanupIssue, ...],
) -> None:
    """Preserve one primary while attaching deterministic snapshot cleanup issues."""

    if type(issues) is not tuple or any(type(item) is not Slice7GCleanupIssue for item in issues):
        _fail("cleanup_issue", "repository cleanup issues must be an exact immutable tuple")
    if not issues:
        raise primary
    if not isinstance(primary, Exception):
        try:
            setattr(primary, "cleanup_issues", issues)
        except Exception:
            pass
        raise primary
    raise Slice7GCoordinatedFailure(primary, issues) from primary


def _finish_repository_authority(
    authority: _RepositorySnapshotAuthority,
    primary: BaseException | None = None,
) -> None:
    issues, pending_base = authority._cleanup_owned_descriptors()
    if primary is not None:
        _raise_repository_cleanup(primary, issues)
    if pending_base is not None:
        try:
            setattr(pending_base, "cleanup_issues", issues)
        except Exception:
            pass
        raise pending_base
    if issues:
        _raise_repository_cleanup(
            Slice7GRuntimeError(
                "source_snapshot_cleanup", "repository snapshot authority cleanup failed",
            ),
            issues,
        )


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_CLOEXEC", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _fail(code: str, message: str, path: str = "$") -> None:
    raise Slice7GRuntimeError(code, message, path=path)
