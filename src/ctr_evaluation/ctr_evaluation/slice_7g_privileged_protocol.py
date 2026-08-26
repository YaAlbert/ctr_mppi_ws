"""Closed Slice 7G protocol shared by the two privileged helper services.

The module is deliberately standard-library only and effect free on import.
Production locators and operation semantics are source owned; alternate
locators exist only on underscored test constructors.
"""

from __future__ import annotations

from dataclasses import dataclass
import array
import fcntl
import hashlib
import json
import os
from pathlib import PurePosixPath
import re
import socket
import stat
import struct
from types import MappingProxyType
from typing import Any


CLEANUP_AUTHORITY_SERVICE = "ctr-slice7g-cleanup-authority.service"
CLEANUP_AUTHORITY_EXECUTABLE = "/usr/libexec/ctr-mppi/ctr-slice7g-cleanupd"
CLEANUP_AUTHORITY_STATE_ROOT = "/var/lib/ctr-mppi/slice-7g-cleanup-authority"
CLEANUP_AUTHORITY_RUNTIME_DIRECTORY = "/run/ctr-mppi/slice-7g-cleanup-authority"
CLEANUP_AUTHORITY_SOCKET = CLEANUP_AUTHORITY_RUNTIME_DIRECTORY + "/cleanup-authority.sock"
CLEANUP_RECOVERY_SOCKET = CLEANUP_AUTHORITY_RUNTIME_DIRECTORY + "/cleanup-recovery.sock"
OBSERVER_SUPERVISOR_SERVICE = "ctr-slice7g-observer-supervisor.service"
OBSERVER_SUPERVISOR_EXECUTABLE = "/usr/libexec/ctr-mppi/ctr-slice7g-observerd"
OBSERVER_SUPERVISOR_RUNTIME_DIRECTORY = "/run/ctr-mppi/slice-7g-observer-supervisor"
OBSERVER_SUPERVISOR_SOCKET = OBSERVER_SUPERVISOR_RUNTIME_DIRECTORY + "/observer-supervisor.sock"
OBSERVER_SUPERVISOR_CGROUP = "/system.slice/ctr-slice7g-observer-supervisor.service"
OBSERVER_LEAF_PATTERN = re.compile(
    r"^/system\.slice/ctr-slice7g-observer-supervisor\.service/"
    r"observer-[0-9]{20}-[0-9a-f]{32}$"
)

AUTHORITY_ACCOUNT = "ctr7g-authority"
CAMPAIGN_ACCOUNT = "ctr7g-campaign"
RUNTIME_GROUP = "ctr7g-runtime"
OBSERVER_ACCOUNT = "ctr7g-observer"
RECOVERY_ACCOUNT = "ctr7g-recovery"

AUTHORITY_BOOTSTRAP_PATH = "/etc/ctr-mppi/slice-7g-authority/bootstrap.json"
AUTHORITY_EXECUTABLE = "/usr/libexec/ctr-mppi/ctr-slice7g-authorityd"
AUTHORITY_STATE_ROOT = "/var/lib/ctr-mppi/slice-7g-authority"
AUTHORITY_RUNTIME_DIRECTORY = "/run/ctr-mppi/slice-7g-authority"
AUTHORITY_SOCKET = AUTHORITY_RUNTIME_DIRECTORY + "/authority.sock"
INSTALLED_RUNTIME_PARENT = "/opt/ctr-mppi/slice-7g"
CAMPAIGN_CGROUP = "/system.slice/ctr-slice7g-campaign.service"

OBSERVER_EXECUTABLE = "/opt/ros/humble/bin/ros2"
OBSERVER_ARGV = ("node", "list", "--no-daemon")
MAX_FRAME_BYTES = 262_144
MAX_TRANSFERRED_FDS = 2
MAX_CONNECTIONS = 8
MAX_FRAMES_PER_CONNECTION = 128
MAX_OUTPUT_BYTES = 1_048_576
FRAME_HEADER = struct.Struct("!I")

AUTHORITY_BOOTSTRAP_V2_SCHEMA = "ctr-slice-7g-authority-bootstrap-2"
AUTHORITY_BOOTSTRAP_V3_SCHEMA = "ctr-slice-7g-authority-bootstrap-3"
INSTALLED_RUNTIME_MANIFEST_V2_SCHEMA = "ctr-slice-7g-installed-runtime-manifest-2"
INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA = "ctr-slice-7g-installed-runtime-manifest-3"
RUNTIME_AUTHORIZATION_V3_SCHEMA = "ctr-slice-7g-runtime-authorization-3"
PROCESS_MANIFEST_V2_SCHEMA = "ctr-slice-7g-process-manifest-2"
PRIVILEGED_SERVICE_MANIFEST_SCHEMA = "ctr-slice-7g-privileged-service-manifest-1"
GLOBAL_LEASE_OBSERVATION_V2_SCHEMA = "ctr-slice-7g-global-lease-observation-2"
FOUR_SOURCE_OBSERVATION_V4_SCHEMA = "ctr-slice-7g-four-source-domain-observation-4"
ROS_GRAPH_RECEIPT_V3_SCHEMA = "ctr-slice-7g-ros-graph-observation-receipt-3"
OBSERVATION_SESSION_V3_SCHEMA = "ctr-slice-7g-observation-session-3"
GLOBAL_ATTEMPT_BUDGET_V4_SCHEMA = "ctr-slice-7g-global-attempt-budget-4"
RUNTIME_AUTHORITY_REQUEST_V4_SCHEMA = "ctr-slice-7g-runtime-authority-request-4"
RUNTIME_AUTHORITY_RECEIPT_V4_SCHEMA = "ctr-slice-7g-runtime-authority-receipt-4"
CLEANUP_REVISION_SCHEMA = "ctr-slice-7g-cleanup-authority-revision-1"
CLEANUP_ANCHOR_SCHEMA = "ctr-slice-7g-cleanup-authority-anchor-1"
CLEANUP_HEAD_SCHEMA = "ctr-slice-7g-cleanup-authority-head-1"
PRIVILEGED_REQUEST_SCHEMA = "ctr-slice-7g-privileged-helper-request-1"
PRIVILEGED_RECEIPT_SCHEMA = "ctr-slice-7g-privileged-helper-receipt-1"
OBSERVER_CONTAINMENT_RECEIPT_SCHEMA = "ctr-slice-7g-observer-containment-receipt-2"
CLEANUP_RECOVERY_AUTHORIZATION_V2_SCHEMA = "ctr-slice-7g-cleanup-recovery-authorization-2"
CLEANUP_RECOVERY_PROVIDER_RECEIPT_SCHEMA = "ctr-slice-7g-cleanup-recovery-provider-receipt-1"
CLEANUP_RECOVERY_OBSERVATION_SCHEMA = "ctr-slice-7g-cleanup-recovery-observation-1"

ALL_V7_SCHEMAS = (
    AUTHORITY_BOOTSTRAP_V3_SCHEMA,
    INSTALLED_RUNTIME_MANIFEST_V2_SCHEMA,
    INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA,
    RUNTIME_AUTHORIZATION_V3_SCHEMA,
    PROCESS_MANIFEST_V2_SCHEMA,
    PRIVILEGED_SERVICE_MANIFEST_SCHEMA,
    GLOBAL_LEASE_OBSERVATION_V2_SCHEMA,
    FOUR_SOURCE_OBSERVATION_V4_SCHEMA,
    ROS_GRAPH_RECEIPT_V3_SCHEMA,
    OBSERVATION_SESSION_V3_SCHEMA,
    GLOBAL_ATTEMPT_BUDGET_V4_SCHEMA,
    RUNTIME_AUTHORITY_REQUEST_V4_SCHEMA,
    RUNTIME_AUTHORITY_RECEIPT_V4_SCHEMA,
    CLEANUP_REVISION_SCHEMA,
    CLEANUP_ANCHOR_SCHEMA,
    CLEANUP_HEAD_SCHEMA,
    PRIVILEGED_REQUEST_SCHEMA,
    PRIVILEGED_RECEIPT_SCHEMA,
    OBSERVER_CONTAINMENT_RECEIPT_SCHEMA,
    CLEANUP_RECOVERY_AUTHORIZATION_V2_SCHEMA,
    CLEANUP_RECOVERY_PROVIDER_RECEIPT_SCHEMA,
    CLEANUP_RECOVERY_OBSERVATION_SCHEMA,
)

OPERATIONS = frozenset({
    "CLEANUP_STATE_QUERY",
    "CLEANUP_REVISION_APPEND",
    "OBSERVE_START",
    "OBSERVE_STATUS",
    "OBSERVE_CANCEL_AND_CLEANUP",
    "RECOVERY_OBSERVE",
    "RECOVERY_COMMIT",
})
LEASE_STATES = (
    "CLEAR", "RESERVED", "COMMITTED", "CONFLICTING", "STALE_INVALID",
    "INDETERMINATE",
)
CLEANUP_STATES = frozenset({
    "ACTIVE_UNBOUND", "ACTIVE_BOUND", "CLEARED", "QUARANTINED", "RECOVERED",
})
RECOVERY_PROVIDERS = ("process", "dds", "lease", "graph")

_DIGEST = re.compile(r"^[0-9a-f]{64}$")
_NONCE = re.compile(r"^[A-Za-z0-9_-]{1,128}$")


class Slice7GPrivilegedProtocolError(RuntimeError):
    """Stable public protocol failure."""

    def __init__(self, code: str, message: str, *, path: str = "$") -> None:
        self.code = code
        self.path = path
        super().__init__(f"{code}:{path}:{message}")


@dataclass(frozen=True)
class PrivilegedRecord:
    schema_version: str
    data: MappingProxyType
    canonical_bytes: bytes
    logical_identity: str


@dataclass(frozen=True)
class PeerCredentials:
    pid: int
    uid: int
    gid: int

    def __post_init__(self) -> None:
        for value in (self.pid, self.uid, self.gid):
            if type(value) is not int or value < 0:
                _fail("peer_credentials", "peer credentials must be exact nonnegative integers")
        if self.pid == 0:
            _fail("peer_credentials", "peer PID must be positive")


@dataclass(frozen=True)
class PeerProcess:
    credentials: PeerCredentials
    start_time_ticks: int
    executable: str
    argv: tuple[str, ...]
    cgroup: str


class ReplayWindow:
    """Bounded fail-closed request/response replay memory for one service generation."""

    def __init__(self, service_generation_identity: str, maximum: int = 8192) -> None:
        self.service_generation_identity = _digest(
            service_generation_identity, "$.service_generation_identity",
        )
        self.maximum = _counter(maximum, 1, 65_536, "$.replay_window_maximum")
        self._claims: set[tuple[str, str, str, str]] = set()
        self._operation_tokens: set[str] = set()
        self._request_nonces: set[str] = set()

    def claim(self, record: PrivilegedRecord) -> None:
        if type(record) is not PrivilegedRecord:
            _fail("replay_record", "replay claim requires an authenticated record")
        data = record.data
        claim = (
            data["connection_nonce"], data["request_nonce"],
            data["operation_token"], data["operation"],
        )
        if any(type(item) is not str for item in claim):
            _fail("replay_binding", "replay binding is incomplete")
        if (
            claim in self._claims
            or data["operation_token"] in self._operation_tokens
            or data["request_nonce"] in self._request_nonces
        ):
            _fail("replay", "privileged operation was already claimed")
        if len(self._claims) >= self.maximum:
            _fail("replay_capacity", "privileged replay memory is exhausted")
        self._claims.add(claim)
        self._operation_tokens.add(data["operation_token"])
        self._request_nonces.add(data["request_nonce"])


_CLIENT_RESPONSE_REPLAY: dict[str, ReplayWindow] = {}


_FIELDS: dict[str, frozenset[str]] = {
    AUTHORITY_BOOTSTRAP_V2_SCHEMA: frozenset({
        "schema_version", "authority_uid", "authority_gid", "campaign_uid", "runtime_gid",
        "observer_uid", "observer_gid", "recovery_uid", "recovery_gid", "authority_account",
        "campaign_account", "runtime_group", "observer_account", "recovery_account",
        "bootstrap_path", "authority_service_path", "authority_state_root",
        "authority_socket_path", "installed_runtime_parent", "cleanup_service_path",
        "cleanup_state_root", "cleanup_socket_path", "recovery_socket_path",
        "observer_service_path", "observer_socket_path", "service_executables",
        "record_paths", "schemas", "protocol_limits", "systemd_units",
    }),
    AUTHORITY_BOOTSTRAP_V3_SCHEMA: frozenset({
        "schema_version", "authority_uid", "authority_gid", "campaign_uid", "runtime_gid",
        "observer_uid", "observer_gid", "recovery_uid", "recovery_gid", "authority_account",
        "campaign_account", "runtime_group", "observer_account", "recovery_account",
        "bootstrap_path", "authority_service_path", "authority_state_root",
        "authority_socket_path", "installed_runtime_parent", "cleanup_service_path",
        "cleanup_state_root", "cleanup_socket_path", "recovery_socket_path",
        "observer_service_path", "observer_socket_path", "service_executables",
        "privileged_code", "record_paths", "schemas", "protocol_limits", "systemd_units",
    }),
    INSTALLED_RUNTIME_MANIFEST_V2_SCHEMA: frozenset({
        "schema_version", "identity_algorithm", "installed_runtime_identity", "root_path",
        "root_device", "root_inode", "physical_tree_identity", "member_count", "members",
        "console_entrypoints", "python_modules", "generated_interfaces", "elf_members",
        "process_manifest_identity", "environment_manifest_identity", "source_snapshot",
        "build_test_approval_identity", "privileged_service_manifest_identity",
    }),
    INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA: frozenset({
        "schema_version", "identity_algorithm", "installed_runtime_identity", "root_path",
        "root_device", "root_inode", "physical_tree_identity", "member_count", "members",
        "console_entrypoints", "python_modules", "generated_interfaces", "elf_members",
        "process_manifest_identity", "environment_manifest_identity", "source_snapshot",
        "build_test_approval_identity", "privileged_service_manifest_identity",
    }),
    RUNTIME_AUTHORIZATION_V3_SCHEMA: frozenset({
        "schema_version", "authorization_nonce", "issued_at_utc", "not_before_utc",
        "not_after_utc", "branch", "head", "tracked_diff_sha256",
        "correction_manifest_sha256", "complete_subject_manifest_sha256", "source_snapshot",
        "charter", "build_test_approval_identity", "installed_runtime_identity",
        "process_manifest_identity", "environment_manifest_identity",
        "privileged_service_manifest_identity", "applicable_test_nodes", "node_id_sha256",
        "git_command_manifest_sha256", "entrypoint_identity", "campaign",
        "readiness_acceptance_identity", "evidence_schemas", "global_budget_identity",
        "output_parent_rule", "prepare_token_lifetime_seconds", "one_shot",
    }),
    PROCESS_MANIFEST_V2_SCHEMA: frozenset({
        "schema_version", "identity_algorithm", "interpreter", "interpreter_flags",
        "entrypoint", "executables", "argv_template", "transaction_slots",
        "environment_manifest_identity", "working_directory", "shell", "systemd_unit",
        "cgroup", "allowed_descendants", "timeouts", "output_ownership",
        "required_receipts", "observer_contract", "privileged_service_manifest_identity",
    }),
    PRIVILEGED_SERVICE_MANIFEST_SCHEMA: frozenset({
        "schema_version", "cleanup_service", "observer_service", "cleanup_state_root",
        "cleanup_socket", "recovery_socket", "observer_socket", "cleanup_principal",
        "observer_supervisor_principal", "observer_principal", "recovery_principal",
        "supervisor_cgroup", "observer_leaf_grammar", "observer_executable", "observer_argv",
        "environment_manifest_identity", "working_directory", "protocol_schema",
        "containment_receipt_schema", "cleanup_schemas", "service_executable_identities",
        "systemd_unit_identities", "numeric_ids_provisioned",
    }),
    GLOBAL_LEASE_OBSERVATION_V2_SCHEMA: frozenset({
        "schema_version", "registry_identity", "registry_revision_identity",
        "physical_observation_identity", "record_physical_identities", "domain_id", "state",
        "owner_bindings", "output_root_bindings", "active_reservation_identities",
        "committed_binding_identities", "stale_invalid_identities", "clear",
        "session_binding_identity", "service_nonce", "phase", "phase_local_ordinal",
        "transaction_observer_ordinal", "observation_interval_identity",
        "observed_monotonic_ns",
    }),
    FOUR_SOURCE_OBSERVATION_V4_SCHEMA: frozenset({
        "schema_version", "session_binding_identity", "service_nonce", "phase",
        "phase_local_ordinal", "transaction_observer_ordinal", "domain_id",
        "peer_process_identity", "observation_interval_identity", "cleanup_disposition_identity",
        "cleanup_head_identity", "containment_receipt_identity", "active_process_identity",
        "dds_port_identity", "global_lease_identity", "global_lease_registry_identity",
        "global_lease_revision_identity", "global_lease_state", "global_lease_clear",
        "ros_graph_provider_identity", "all_sources_clear", "observed_monotonic_ns",
    }),
    ROS_GRAPH_RECEIPT_V3_SCHEMA: frozenset({
        "schema_version", "session_binding_identity", "service_nonce", "phase",
        "phase_local_ordinal", "transaction_observer_ordinal",
        "four_source_observation_identity", "containment_receipt_identity",
        "cleanup_head_identity", "observer_class", "executable", "executable_identity",
        "interpreter", "interpreter_identity", "module_origin_identities", "argv",
        "environment_identity", "working_directory", "cgroup", "shell", "domain_id",
        "pid", "process_group_id", "process_start_time_ticks", "started_monotonic_ns",
        "ended_monotonic_ns", "exit_status", "terminating_signal", "stdout_size",
        "stdout_sha256", "stderr_size", "stderr_sha256", "nodes",
        "parsed_node_set_identity", "cleanup_barrier_identity", "unexpected_descendants",
        "ros_daemon_started",
    }),
    OBSERVATION_SESSION_V3_SCHEMA: frozenset({
        "schema_version", "authorization_identity", "installed_runtime_identity",
        "process_manifest_identity", "environment_manifest_identity",
        "privileged_service_manifest_identity", "connection_identity", "peer_uid", "peer_gid",
        "peer_pid", "peer_start_time_ticks", "campaign_cgroup", "service_nonce",
        "daemon_generation_identity", "cleanup_head_identity", "created_monotonic_ns",
        "deadline_monotonic_ns", "domain_minimum", "domain_maximum",
        "maximum_precommit_observers", "precommit_observer_count",
        "postcommit_observer_count", "transaction_observer_count", "candidate_domains",
        "precommit_receipt_identities", "selected_domain", "lease_identity",
        "four_source_observation_identity", "state",
    }),
    GLOBAL_ATTEMPT_BUDGET_V4_SCHEMA: frozenset({
        "schema_version", "revision", "predecessor_identity", "state", "attempts_consumed",
        "attempts_maximum", "retries_authorized", "authorization_identity",
        "process_start_commitment", "observation_session_identity",
        "four_source_observation_identity", "cleanup_head_identity",
        "containment_receipt_identity", "precommit_observer_count",
        "precommit_receipt_identities", "postcommit_observer_count",
        "postcommit_receipt_identity", "postcommit_four_source_observation_identity",
        "transaction_observer_count", "updated_at_utc",
    }),
    RUNTIME_AUTHORITY_REQUEST_V4_SCHEMA: frozenset({
        "schema_version", "method", "request_id", "authorization_identity", "prepare_token",
        "campaign_id", "campaign_identity", "campaign_template_identity", "domain_id",
        "output_root_path", "output_root_identity", "process_manifest_identity",
        "process_instance_identity", "observation_session_identity",
        "observation_session_nonce", "privileged_service_manifest_identity",
        "requested_at_utc",
    }),
    RUNTIME_AUTHORITY_RECEIPT_V4_SCHEMA: frozenset({
        "schema_version", "method", "request_id", "result", "authorization_identity",
        "service_instance_identity", "service_nonce", "prepare_token",
        "previous_budget_revision", "budget_revision", "budget_identity", "campaign_id",
        "campaign_identity", "campaign_template_identity", "domain_id", "output_root_path",
        "output_root_identity", "process_manifest_identity", "process_instance_identity",
        "observation_session_identity", "observation_session_nonce",
        "observation_session_deadline_monotonic_ns", "four_source_observation_identity",
        "precommit_receipt_identities", "precommit_observer_count",
        "postcommit_observer_count", "transaction_observer_count", "lease_identity",
        "prepare_expires_monotonic_ns", "committed_at_utc", "candidate_clear", "error_code",
        "cleanup_head_identity", "containment_receipt_identity",
    }),
    CLEANUP_REVISION_SCHEMA: frozenset({
        "schema_version", "revision", "predecessor_identity", "state",
        "runtime_authorization_identity", "budget_identity", "service_generation_identity",
        "session_binding_identity", "phase", "phase_local_ordinal",
        "transaction_observer_ordinal", "domain_id", "observer_contract_identity",
        "containment_identity", "process_identity", "disposition_identity",
        "recovery_authorization_identity", "created_at_utc",
    }),
    CLEANUP_ANCHOR_SCHEMA: frozenset({
        "schema_version", "revision", "authority_root_identity", "revision_identity",
        "revision_device", "revision_inode", "revision_mode", "revision_link_count",
        "revision_size", "revision_sha256", "predecessor_anchor_identity",
    }),
    CLEANUP_HEAD_SCHEMA: frozenset({
        "schema_version", "revision", "authority_root_identity", "revision_identity",
        "anchor_identity", "anchor_device", "anchor_inode", "anchor_mode",
        "anchor_link_count", "anchor_size", "anchor_sha256", "predecessor_head_identity",
    }),
    PRIVILEGED_REQUEST_SCHEMA: frozenset({
        "schema_version", "operation", "sequence", "connection_nonce", "request_nonce",
        "operation_token", "service_generation_identity", "runtime_authorization_identity",
        "installed_runtime_identity", "budget_identity", "cleanup_head_identity",
        "session_binding_identity", "domain_id", "phase", "phase_local_ordinal",
        "transaction_observer_ordinal", "transition", "observer_contract_identity",
        "containment_identity", "process_identity", "disposition_identity",
        "recovery_authorization_identity",
    }),
    PRIVILEGED_RECEIPT_SCHEMA: frozenset({
        "schema_version", "operation", "sequence", "connection_nonce", "request_nonce",
        "operation_token", "service_generation_identity", "result", "error_code",
        "cleanup_head_identity", "containment_receipt_identity", "output_descriptor_count",
        "payload_identity", "cleanup_revision", "cleanup_anchor", "cleanup_head",
        "containment_receipt",
    }),
    OBSERVER_CONTAINMENT_RECEIPT_SCHEMA: frozenset({
        "schema_version", "operation_token", "service_generation_identity",
        "session_binding_identity", "runtime_authorization_identity", "budget_identity",
        "cleanup_active_head_identity", "cleanup_terminal_head_identity", "domain_id", "phase",
        "phase_local_ordinal", "transaction_observer_ordinal", "leaf_cgroup",
        "leaf_cgroup_identity", "pid", "process_start_time_ticks", "process_group_id",
        "session_id", "pidfd_identity", "procfd_identity", "executable_identity",
        "interpreter_identity", "argv_identity", "environment_identity",
        "postexec_identity", "working_directory_identity",
        "started_monotonic_ns", "ended_monotonic_ns", "exit_status",
        "terminating_signal", "stdout_size", "stdout_sha256", "stderr_size",
        "stderr_sha256", "cleanup_barrier_identity", "stable_empty_samples",
        "stable_empty_span_ns", "leaf_removed", "disposition",
    }),
    CLEANUP_RECOVERY_AUTHORIZATION_V2_SCHEMA: frozenset({
        "schema_version", "recovery_nonce", "quarantine_head_identity",
        "quarantine_anchor_identity", "runtime_authorization_identity",
        "installed_runtime_identity", "budget_identity", "cleanup_service_generation_identity",
        "observer_service_generation_identity", "issued_at_utc", "not_before_utc",
        "not_after_utc", "one_shot",
    }),
    CLEANUP_RECOVERY_PROVIDER_RECEIPT_SCHEMA: frozenset({
        "schema_version", "provider", "provider_identity", "recovery_nonce",
        "quarantine_head_identity", "quarantine_anchor_identity",
        "recovery_authorization_identity", "service_generation_identity", "domain_id",
        "runtime_authorization_identity", "budget_identity", "phase", "ordinal",
        "started_monotonic_ns", "ended_monotonic_ns", "evidence_identity", "clear",
        "cleanup_disposition_identity",
    }),
    CLEANUP_RECOVERY_OBSERVATION_SCHEMA: frozenset({
        "schema_version", "recovery_nonce", "quarantine_head_identity",
        "quarantine_anchor_identity", "recovery_authorization_identity",
        "runtime_authorization_identity", "budget_identity", "service_generation_identity",
        "domain_id", "provider_receipt_identities", "all_sources_clear",
        "observed_monotonic_ns",
    }),
}


def schema_names() -> tuple[str, ...]:
    return ALL_V7_SCHEMAS


def canonical_bytes(value: dict[str, Any] | bytes, *, expected_schema: str) -> bytes:
    return validate_record(value, expected_schema=expected_schema).canonical_bytes


def record_identity(value: dict[str, Any] | bytes, *, expected_schema: str) -> str:
    return validate_record(value, expected_schema=expected_schema).logical_identity


def validate_record(value: dict[str, Any] | bytes, *, expected_schema: str) -> PrivilegedRecord:
    if type(expected_schema) is not str or expected_schema not in _FIELDS:
        _fail("schema", "expected schema is not a v7 closed schema")
    if type(value) is bytes:
        data = _parse(value)
        raw = value
    elif type(value) is dict:
        data = _detach(value)
        raw = _canonical(data)
    else:
        _fail("record_type", "record must be an exact dictionary or exact bytes")
    if data.get("schema_version") != expected_schema:
        _fail("schema", "schema version differs", path="$.schema_version")
    _closed(data, _FIELDS[expected_schema])
    _validate_semantics(expected_schema, data)
    canonical = _canonical(data)
    if type(value) is bytes and raw != canonical:
        _fail("noncanonical_json", "record bytes are not canonical")
    identity = hashlib.sha256(
        (expected_schema + ":canonical-1").encode("utf-8") + b"\0" + canonical
    ).hexdigest()
    return PrivilegedRecord(expected_schema, _freeze(data), canonical, identity)


def _exact_mapping(value: Any, keys: set[str], path: str) -> dict[str, Any]:
    if type(value) is not dict:
        _fail("nested_mapping", "nested value must be an exact dictionary", path=path)
    if set(value) != keys:
        _fail("closed_schema", f"nested field set differs: {sorted(set(value) ^ keys)!r}", path=path)
    return value


def _complete_file_record(value: Any, path: str, *, absolute: bool) -> dict[str, Any]:
    item = _exact_mapping(value, {
        "device", "inode", "link_count", "mode", "owner_gid", "owner_uid",
        "path", "sha256", "size", "type",
    }, path)
    if item["type"] != "regular":
        _fail("file_record_type", "trusted file must be regular", path=f"{path}.type")
    if absolute:
        if type(item["path"]) is not str or not PurePosixPath(item["path"]).is_absolute():
            _fail("path", "trusted path must be absolute", path=f"{path}.path")
    else:
        safe_relative(item["path"])
    if type(item["mode"]) is not int or not 0 <= item["mode"] <= 0o7777 or item["mode"] & 0o222:
        _fail("file_record_mode", "trusted file mode is invalid or writable", path=f"{path}.mode")
    if type(item["link_count"]) is not int or item["link_count"] != 1:
        _fail("file_record_link", "trusted file must be single-link", path=f"{path}.link_count")
    for field, minimum in (
        ("device", 0), ("inode", 1), ("owner_gid", 0), ("owner_uid", 0),
        ("size", 1),
    ):
        _counter(item[field], minimum, 2**63 - 1, f"{path}.{field}")
    if item["owner_uid"] != 0 or item["owner_gid"] != 0:
        _fail("file_record_owner", "privileged trust member must be root owned", path=path)
    _digest(item["sha256"], f"{path}.sha256")
    return item


def _installed_member(item: dict[str, Any], path: str) -> None:
    safe_relative(item["path"])
    if item["type"] not in {"regular", "directory"}:
        _fail("installed_member_type", "installed member type differs", path=f"{path}.type")
    if type(item["mode"]) is not int or not 0 <= item["mode"] <= 0o7777:
        _fail("installed_member_mode", "installed member mode differs", path=f"{path}.mode")
    _counter(item["link_count"], 1, 2**31 - 1, f"{path}.link_count")
    if item["type"] == "regular":
        _counter(item["size"], 0, 2**63 - 1, f"{path}.size")
        _digest(item["sha256"], f"{path}.sha256")
        if item["link_count"] != 1:
            _fail("installed_hardlink", "installed file must be single-link", path=path)
    elif item["size"] is not None or item["sha256"] is not None:
        _fail("installed_directory", "directory size and digest must be null", path=path)


def _plain_string(value: Any, path: str) -> str:
    if type(value) is not str or not value or len(value.encode("utf-8")) > 8192:
        _fail("string", "expected a bounded nonempty exact string", path=path)
    return value


def _string_inventory(value: Any, path: str) -> list[str]:
    if type(value) is not list:
        _fail("string_list", "expected an exact string list", path=path)
    for index, item in enumerate(value):
        _plain_string(item, f"{path}[{index}]")
    if value != sorted(value) or len(value) != len(set(value)):
        _fail("string_list", "string inventory must be unique and sorted", path=path)
    return value


def _console_entrypoint(item: dict[str, Any], path: str) -> None:
    _exact_mapping(item, {"name", "script_path", "script_sha256", "target"}, path)
    _plain_string(item["name"], f"{path}.name")
    _plain_string(item["target"], f"{path}.target")
    safe_relative(item["script_path"])
    _digest(item["script_sha256"], f"{path}.script_sha256")


def _python_module(item: dict[str, Any], path: str) -> None:
    _exact_mapping(item, {"module", "origin", "sha256"}, path)
    _plain_string(item["module"], f"{path}.module")
    safe_relative(item["origin"])
    _digest(item["sha256"], f"{path}.sha256")


def _generated_interface(item: dict[str, Any], path: str) -> None:
    _exact_mapping(item, {"kind", "name", "origin", "sha256"}, path)
    _plain_string(item["name"], f"{path}.name")
    if item["kind"] not in {"idl", "message", "service", "python", "typesupport"}:
        _fail("interface_kind", "generated interface kind differs", path=f"{path}.kind")
    safe_relative(item["origin"])
    _digest(item["sha256"], f"{path}.sha256")


def _elf_member(item: dict[str, Any], path: str) -> None:
    _exact_mapping(item, {
        "build_id", "elf_class", "machine", "needed", "path", "rpath",
        "runpath", "unresolved",
    }, path)
    safe_relative(item["path"])
    if item["elf_class"] not in {"ELF32", "ELF64"}:
        _fail("elf_class", "ELF class differs", path=f"{path}.elf_class")
    _plain_string(item["machine"], f"{path}.machine")
    _plain_string(item["build_id"], f"{path}.build_id")
    for field in ("needed", "rpath", "runpath", "unresolved"):
        _string_inventory(item[field], f"{path}.{field}")
    if item["unresolved"]:
        _fail("elf_unresolved", "ELF dependencies must resolve", path=path)


def _ordered_records(value: Any, path: str, key: str, validator: Any) -> None:
    if type(value) is not list:
        _fail("record_list", "record inventory must be an exact list", path=path)
    identities = []
    for index, item in enumerate(value):
        if type(item) is not dict:
            _fail("record_list", "record inventory member must be an exact dictionary", path=f"{path}[{index}]")
        validator(item, f"{path}[{index}]")
        identities.append(item[key])
    if identities != sorted(identities) or len(identities) != len(set(identities)):
        _fail("record_order", "record inventory must be unique and sorted", path=path)


def _source_snapshot(value: Any, path: str) -> None:
    item = _exact_mapping(value, {
        "logical_identity", "logical_identity_algorithm", "member_count", "mode_bound",
        "path", "physical_sha256", "schema_version",
    }, path)
    _plain_string(item["schema_version"], f"{path}.schema_version")
    if type(item["path"]) is not str or not PurePosixPath(item["path"]).is_absolute():
        _fail("path", "snapshot path must be absolute", path=f"{path}.path")
    _digest(item["physical_sha256"], f"{path}.physical_sha256")
    _digest(item["logical_identity"], f"{path}.logical_identity")
    _plain_string(item["logical_identity_algorithm"], f"{path}.logical_identity_algorithm")
    _counter(item["member_count"], 1, 100_000, f"{path}.member_count")
    if type(item["mode_bound"]) is not bool or item["mode_bound"] is not True:
        _fail("snapshot_mode", "snapshot must bind modes", path=f"{path}.mode_bound")


def _charter_binding(value: Any, path: str) -> None:
    item = _exact_mapping(value, {
        "logical_identity", "logical_identity_algorithm", "path",
        "physical_sha256", "schema_version",
    }, path)
    if item["schema_version"] != "ctr-slice-7g-charter-7":
        _fail("authorization_charter", "runtime authorization must bind charter v7", path=f"{path}.schema_version")
    if type(item["path"]) is not str or not PurePosixPath(item["path"]).is_absolute():
        _fail("path", "charter path must be absolute", path=f"{path}.path")
    _digest(item["physical_sha256"], f"{path}.physical_sha256")
    _digest(item["logical_identity"], f"{path}.logical_identity")
    if item["logical_identity_algorithm"] != "sha256:ctr-slice-7g-charter-canonical-7":
        _fail("authorization_charter", "charter identity algorithm differs", path=f"{path}.logical_identity_algorithm")


def _digest_inventory(
    value: Any, path: str, *, maximum: int = 65_536,
    canonical_sort: bool = False,
) -> list[str]:
    if type(value) is not list or len(value) > maximum:
        _fail("identity_list", "identity inventory is malformed or oversized", path=path)
    for index, item in enumerate(value):
        _digest(item, f"{path}[{index}]")
    if len(value) != len(set(value)) or (canonical_sort and value != sorted(value)):
        _fail("identity_list", "identity inventory must be unique and canonically ordered", path=path)
    return value


def _process_file_identity(value: Any, path: str) -> dict[str, Any]:
    item = _exact_mapping(value, {
        "device", "inode", "link_count", "mode", "owner_gid", "owner_uid",
        "path", "sha256", "size",
    }, path)
    if type(item["path"]) is not str or not PurePosixPath(item["path"]).is_absolute():
        _fail("path", "process file path must be absolute", path=f"{path}.path")
    if type(item["mode"]) is not int or not 0 <= item["mode"] <= 0o7777 or item["mode"] & 0o222:
        _fail("process_file_mode", "process file must be immutable", path=f"{path}.mode")
    if type(item["link_count"]) is not int or item["link_count"] != 1:
        _fail("process_file_link", "process file must be single-link", path=f"{path}.link_count")
    for field, minimum in (("device", 0), ("inode", 1), ("owner_gid", 0), ("owner_uid", 0), ("size", 1)):
        _counter(item[field], minimum, 2**63 - 1, f"{path}.{field}")
    _digest(item["sha256"], f"{path}.sha256")
    return item


def _campaign_binding(value: Any, path: str) -> None:
    item = _exact_mapping(value, {
        "campaign_identity_algorithm", "domain_maximum", "domain_minimum",
        "duration_seconds", "endpoint", "plan_identity", "retries", "scenarios",
        "seeds",
    }, path)
    if item["endpoint"] != "simulation_only_promoted_completion":
        _fail("campaign", "campaign endpoint differs", path=f"{path}.endpoint")
    if item["scenarios"] != ["centerline", "lateral_offset", "near_safety_boundary"]:
        _fail("campaign", "campaign scenario inventory differs", path=f"{path}.scenarios")
    if item["seeds"] != [11, 22, 33, 44, 55] or any(type(seed) is not int for seed in item["seeds"]):
        _fail("campaign", "campaign seed inventory differs", path=f"{path}.seeds")
    if type(item["duration_seconds"]) is not float or item["duration_seconds"] != 25.0:
        _fail("campaign", "campaign duration differs", path=f"{path}.duration_seconds")
    for field, expected in (("retries", 0), ("domain_minimum", 100), ("domain_maximum", 199)):
        if type(item[field]) is not int or item[field] != expected:
            _fail("campaign", f"campaign {field} differs", path=f"{path}.{field}")
    _digest(item["plan_identity"], f"{path}.plan_identity")
    if item["campaign_identity_algorithm"] != "sha256:ctr-slice-7g-runtime-campaign-canonical-1":
        _fail("campaign", "campaign identity algorithm differs", path=f"{path}.campaign_identity_algorithm")


def _output_parent_rule(value: Any, path: str) -> None:
    item = _exact_mapping(value, {
        "acl_policy_identity", "authority_creates_root", "campaign_parent_entry_mutation",
        "campaign_parent_listing", "path",
    }, path)
    if type(item["path"]) is not str or not PurePosixPath(item["path"]).is_absolute():
        _fail("output_parent", "output parent must be absolute", path=f"{path}.path")
    for field, expected in (
        ("authority_creates_root", True),
        ("campaign_parent_entry_mutation", False),
        ("campaign_parent_listing", False),
    ):
        if type(item[field]) is not bool or item[field] is not expected:
            _fail("output_parent", f"output parent {field} differs", path=f"{path}.{field}")
    _digest(item["acl_policy_identity"], f"{path}.acl_policy_identity")


def _process_start_commitment(value: Any, path: str) -> None:
    item = _exact_mapping(value, {
        "campaign_identity", "campaign_template_identity", "cleanup_head_identity",
        "committed_at_utc", "containment_receipt_identity", "domain_id",
        "four_source_observation_identity", "lease_identity",
        "observation_session_identity", "output_root_identity", "peer_executable",
        "peer_pid", "peer_start_time_ticks", "precommit_observer_count",
        "precommit_receipt_identities", "prepare_token_identity",
        "process_instance_identity", "process_manifest_identity",
        "service_instance_identity",
    }, path)
    for field in (
        "campaign_identity", "campaign_template_identity", "cleanup_head_identity",
        "containment_receipt_identity", "four_source_observation_identity",
        "lease_identity", "observation_session_identity", "output_root_identity",
        "prepare_token_identity", "process_instance_identity", "process_manifest_identity",
        "service_instance_identity",
    ):
        _digest(item[field], f"{path}.{field}")
    _domain(item["domain_id"], f"{path}.domain_id")
    _counter(item["peer_pid"], 1, 2**63 - 1, f"{path}.peer_pid")
    _counter(item["peer_start_time_ticks"], 1, 2**63 - 1, f"{path}.peer_start_time_ticks")
    _counter(item["precommit_observer_count"], 1, 100, f"{path}.precommit_observer_count")
    receipts = _digest_inventory(item["precommit_receipt_identities"], f"{path}.precommit_receipt_identities", maximum=100)
    if len(receipts) != item["precommit_observer_count"]:
        _fail("observer_counter", "commitment receipt count differs", path=path)
    if type(item["peer_executable"]) is not str or not PurePosixPath(item["peer_executable"]).is_absolute():
        _fail("commitment_peer", "commitment peer executable differs", path=f"{path}.peer_executable")
    _plain_string(item["committed_at_utc"], f"{path}.committed_at_utc")


def _validate_semantics(schema: str, data: dict[str, Any]) -> None:
    _validate_json(data, "$", 0)
    if schema in {AUTHORITY_BOOTSTRAP_V2_SCHEMA, AUTHORITY_BOOTSTRAP_V3_SCHEMA}:
        for field in (
            "authority_uid", "authority_gid", "campaign_uid", "runtime_gid",
            "observer_uid", "observer_gid", "recovery_uid", "recovery_gid",
        ):
            _counter(data[field], 0, 2**31 - 1, f"$.{field}")
        expected = {
            "authority_account": AUTHORITY_ACCOUNT,
            "campaign_account": CAMPAIGN_ACCOUNT,
            "runtime_group": RUNTIME_GROUP,
            "observer_account": OBSERVER_ACCOUNT,
            "recovery_account": RECOVERY_ACCOUNT,
            "bootstrap_path": AUTHORITY_BOOTSTRAP_PATH,
            "authority_service_path": AUTHORITY_EXECUTABLE,
            "authority_state_root": AUTHORITY_STATE_ROOT,
            "authority_socket_path": AUTHORITY_SOCKET,
            "installed_runtime_parent": INSTALLED_RUNTIME_PARENT,
            "cleanup_service_path": CLEANUP_AUTHORITY_EXECUTABLE,
            "cleanup_state_root": CLEANUP_AUTHORITY_STATE_ROOT,
            "cleanup_socket_path": CLEANUP_AUTHORITY_SOCKET,
            "recovery_socket_path": CLEANUP_RECOVERY_SOCKET,
            "observer_service_path": OBSERVER_SUPERVISOR_EXECUTABLE,
            "observer_socket_path": OBSERVER_SUPERVISOR_SOCKET,
        }
        for field, expected_value in expected.items():
            if data[field] != expected_value:
                _fail("bootstrap_binding", f"{field} differs", path=f"$.{field}")
        if type(data["service_executables"]) is not list or len(data["service_executables"]) != 3:
            _fail("bootstrap_executables", "bootstrap must bind exactly three service executables")
        if schema == AUTHORITY_BOOTSTRAP_V2_SCHEMA:
            if any(type(item) not in (str, dict) for item in data["service_executables"]):
                _fail("bootstrap_executables", "historical bootstrap executable inventory differs")
            for field in ("record_paths", "schemas", "protocol_limits", "systemd_units"):
                if type(data[field]) is not dict:
                    _fail("bootstrap_contract", "historical bootstrap nested value differs", path=f"$.{field}")
            return
        executable_paths = []
        for index, executable in enumerate(data["service_executables"]):
            _complete_file_record(executable, f"$.service_executables[{index}]", absolute=True)
            executable_paths.append(executable["path"])
        if executable_paths != sorted({
            AUTHORITY_EXECUTABLE, CLEANUP_AUTHORITY_EXECUTABLE,
            OBSERVER_SUPERVISOR_EXECUTABLE,
        }):
            _fail("bootstrap_executables", "bootstrap executable inventory differs")
        record_paths = _exact_mapping(data["record_paths"], {
            "build_test_approval", "environment_manifest", "global_budget",
            "installed_runtime_manifest", "privileged_service_manifest",
            "process_manifest", "runtime_authorization",
        }, "$.record_paths")
        expected_record_paths = {
            "build_test_approval": AUTHORITY_STATE_ROOT + "/public/build.json",
            "environment_manifest": AUTHORITY_STATE_ROOT + "/public/environment.json",
            "global_budget": AUTHORITY_STATE_ROOT + "/global-budget/revision-00000000000000000000.json",
            "installed_runtime_manifest": AUTHORITY_STATE_ROOT + "/public/installed.json",
            "privileged_service_manifest": AUTHORITY_STATE_ROOT + "/public/privileged-services.json",
            "process_manifest": AUTHORITY_STATE_ROOT + "/public/process.json",
            "runtime_authorization": AUTHORITY_STATE_ROOT + "/public/authorization.json",
        }
        if record_paths != expected_record_paths:
            _fail("bootstrap_records", "bootstrap record paths differ")
        schemas = _exact_mapping(data["schemas"], {
            "build_test_approval", "environment_manifest", "global_budget",
            "installed_runtime_manifest", "privileged_service_manifest",
            "process_manifest", "runtime_authorization",
        }, "$.schemas")
        expected_schemas = {
            "build_test_approval": "ctr-slice-7g-isolated-build-test-approval-1",
            "environment_manifest": "ctr-slice-7g-environment-manifest-1",
            "global_budget": GLOBAL_ATTEMPT_BUDGET_V4_SCHEMA,
            "installed_runtime_manifest": INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA,
            "privileged_service_manifest": PRIVILEGED_SERVICE_MANIFEST_SCHEMA,
            "process_manifest": PROCESS_MANIFEST_V2_SCHEMA,
            "runtime_authorization": RUNTIME_AUTHORIZATION_V3_SCHEMA,
        }
        if schemas != expected_schemas:
            _fail("bootstrap_schemas", "bootstrap schema inventory differs")
        limits = _exact_mapping(data["protocol_limits"], {
            "maximum_connections", "maximum_frame_bytes", "maximum_frames_per_connection",
            "maximum_list_items", "maximum_record_depth", "maximum_transferred_fds",
        }, "$.protocol_limits")
        expected_limits = {
            "maximum_connections": MAX_CONNECTIONS,
            "maximum_frame_bytes": MAX_FRAME_BYTES,
            "maximum_frames_per_connection": MAX_FRAMES_PER_CONNECTION,
            "maximum_list_items": 65_536,
            "maximum_record_depth": 24,
            "maximum_transferred_fds": MAX_TRANSFERRED_FDS,
        }
        if limits != expected_limits or any(type(value) is not int for value in limits.values()):
            _fail("bootstrap_limits", "bootstrap protocol limits differ")
        units = _exact_mapping(data["systemd_units"], {
            "authority", "campaign", "cleanup_authority", "observer_supervisor",
            "revocation_path", "revocation_service",
        }, "$.systemd_units")
        if units != {
            "authority": "ctr-slice7g-authority.service",
            "campaign": "ctr-slice7g-campaign.service",
            "cleanup_authority": CLEANUP_AUTHORITY_SERVICE,
            "observer_supervisor": OBSERVER_SUPERVISOR_SERVICE,
            "revocation_path": "ctr-slice7g-revocation.path",
            "revocation_service": "ctr-slice7g-revocation.service",
        }:
            _fail("bootstrap_units", "bootstrap systemd-unit inventory differs")
        if schema == AUTHORITY_BOOTSTRAP_V2_SCHEMA:
            return
        code = _exact_mapping(data["privileged_code"], {
            "installed_root", "members", "observer_contract",
        }, "$.privileged_code")
        installed_root = code["installed_root"]
        if (
            type(installed_root) is not str
            or PurePosixPath(installed_root).parent.as_posix() != INSTALLED_RUNTIME_PARENT
            or not _DIGEST.fullmatch(PurePosixPath(installed_root).name)
        ):
            _fail("bootstrap_code_root", "root-owned privileged code root differs")
        members = code["members"]
        if type(members) is not list or len(members) != 4:
            _fail("bootstrap_code_members", "privileged code member inventory differs")
        member_paths = []
        for index, member in enumerate(members):
            _complete_file_record(member, f"$.privileged_code.members[{index}]", absolute=False)
            member_paths.append(member["path"])
        expected_members = sorted({
            "lib/python3.10/site-packages/ctr_evaluation/__init__.py",
            "lib/python3.10/site-packages/ctr_evaluation/slice_7g_cleanup_authority.py",
            "lib/python3.10/site-packages/ctr_evaluation/slice_7g_observer_supervisor.py",
            "lib/python3.10/site-packages/ctr_evaluation/slice_7g_privileged_protocol.py",
        })
        if member_paths != expected_members:
            _fail("bootstrap_code_members", "privileged code paths differ")
        observer = _exact_mapping(code["observer_contract"], {
            "argv", "environment", "executable", "interpreter", "working_directory",
        }, "$.privileged_code.observer_contract")
        _complete_file_record(observer["executable"], "$.privileged_code.observer_contract.executable", absolute=True)
        _complete_file_record(observer["interpreter"], "$.privileged_code.observer_contract.interpreter", absolute=True)
        if observer["executable"]["path"] != OBSERVER_EXECUTABLE:
            _fail("bootstrap_observer", "observer executable path differs")
        if observer["interpreter"]["path"] != "/usr/bin/python3.10":
            _fail("bootstrap_observer", "observer interpreter path differs")
        if observer["argv"] != [OBSERVER_EXECUTABLE, *OBSERVER_ARGV]:
            _fail("bootstrap_observer", "observer argv differs")
        environment = _exact_mapping(observer["environment"], {
            "dynamic_key", "dynamic_maximum", "dynamic_minimum", "fixed_values",
        }, "$.privileged_code.observer_contract.environment")
        if (
            environment["dynamic_key"] != "ROS_DOMAIN_ID"
            or type(environment["dynamic_minimum"]) is not int
            or type(environment["dynamic_maximum"]) is not int
            or environment["dynamic_minimum"] != 100
            or environment["dynamic_maximum"] != 199
        ):
            _fail("bootstrap_observer_environment", "dynamic observer environment differs")
        fixed = environment["fixed_values"]
        expected_environment_keys = {
            "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "HOME", "LD_LIBRARY_PATH",
            "MPLCONFIGDIR", "PATH", "PYTHONDONTWRITEBYTECODE", "PYTHONNOUSERSITE",
            "PYTHONPATH", "RMW_IMPLEMENTATION", "ROS_HOME", "ROS_LOCALHOST_ONLY",
            "ROS_DISTRO", "ROS_LOG_DIR", "XDG_CACHE_HOME",
        }
        if type(fixed) is not dict or set(fixed) != expected_environment_keys or any(
            type(key) is not str or type(value) is not str for key, value in fixed.items()
        ):
            _fail("bootstrap_observer_environment", "fixed observer environment differs")
        if type(observer["working_directory"]) is not str or observer["working_directory"] != installed_root:
            _fail("bootstrap_observer_cwd", "observer working directory differs")
    elif schema in {
        INSTALLED_RUNTIME_MANIFEST_V2_SCHEMA, INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA,
    }:
        if data["identity_algorithm"] != "sha256:ctr-slice-7g-installed-runtime-tree-canonical-1":
            _fail("installed_identity_algorithm", "installed identity algorithm differs")
        identity = _digest(data["installed_runtime_identity"], "$.installed_runtime_identity")
        if data["root_path"] != f"{INSTALLED_RUNTIME_PARENT}/{identity}":
            _fail("installed_root", "installed root is not derived from its logical identity")
        _counter(data["root_device"], 0, 2**63 - 1, "$.root_device")
        _counter(data["root_inode"], 1, 2**63 - 1, "$.root_inode")
        _digest(data["physical_tree_identity"], "$.physical_tree_identity")
        _counter(data["member_count"], 1, 100_000, "$.member_count")
        if type(data["members"]) is not list or len(data["members"]) != data["member_count"]:
            _fail("installed_members", "installed member inventory/count differs")
        paths = []
        for index, member in enumerate(data["members"]):
            member_fields = {
                "path", "type", "mode", "link_count", "size", "sha256",
            }
            if schema == INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA:
                member_fields.update({"owner_uid", "owner_gid"})
            if type(member) is not dict or set(member) != member_fields:
                _fail("installed_member", "installed member schema differs", path=f"$.members[{index}]")
            _installed_member(member, f"$.members[{index}]")
            if schema == INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA:
                _counter(member["owner_uid"], 0, 2**31 - 1, f"$.members[{index}].owner_uid")
                _counter(member["owner_gid"], 0, 2**31 - 1, f"$.members[{index}].owner_gid")
                if member["owner_uid"] != 0 or member["owner_gid"] != 0:
                    _fail("installed_member_owner", "v3 installed members must be root-owned", path=f"$.members[{index}]")
            paths.append(member["path"])
        if paths != sorted(paths) or len(paths) != len(set(paths)):
            _fail("installed_members", "installed member paths must be unique and ordered")
        for field in (
            "process_manifest_identity", "environment_manifest_identity",
            "build_test_approval_identity", "privileged_service_manifest_identity",
        ):
            _digest(data[field], f"$.{field}")
        _ordered_records(data["console_entrypoints"], "$.console_entrypoints", "name", _console_entrypoint)
        _ordered_records(data["python_modules"], "$.python_modules", "module", _python_module)
        _ordered_records(data["generated_interfaces"], "$.generated_interfaces", "name", _generated_interface)
        _ordered_records(data["elf_members"], "$.elf_members", "path", _elf_member)
        _source_snapshot(data["source_snapshot"], "$.source_snapshot")
    elif schema == RUNTIME_AUTHORIZATION_V3_SCHEMA:
        for field in (
            "tracked_diff_sha256", "correction_manifest_sha256",
            "complete_subject_manifest_sha256", "build_test_approval_identity",
            "installed_runtime_identity", "process_manifest_identity",
            "environment_manifest_identity", "privileged_service_manifest_identity",
            "node_id_sha256", "git_command_manifest_sha256", "entrypoint_identity",
            "readiness_acceptance_identity", "global_budget_identity",
        ):
            _digest(data[field], f"$.{field}")
        _nonce(data["authorization_nonce"], "$.authorization_nonce")
        for field in ("issued_at_utc", "not_before_utc", "not_after_utc", "branch"):
            _plain_string(data[field], f"$.{field}")
        if type(data["head"]) is not str or re.fullmatch(r"[0-9a-f]{40}", data["head"]) is None:
            _fail("authorization_head", "authorization HEAD differs", path="$.head")
        _source_snapshot(data["source_snapshot"], "$.source_snapshot")
        _charter_binding(data["charter"], "$.charter")
        _campaign_binding(data["campaign"], "$.campaign")
        _output_parent_rule(data["output_parent_rule"], "$.output_parent_rule")
        _counter(data["applicable_test_nodes"], 1, 1_000_000, "$.applicable_test_nodes")
        evidence = data["evidence_schemas"]
        if type(evidence) is not dict or not evidence:
            _fail("evidence_schemas", "evidence schema map must be a nonempty exact dictionary")
        for key, value in evidence.items():
            _plain_string(key, f"$.evidence_schemas.{key}")
            _plain_string(value, f"$.evidence_schemas.{key}")
        if data["prepare_token_lifetime_seconds"] != 300 or type(data["prepare_token_lifetime_seconds"]) is not int:
            _fail("prepare_lifetime", "prepare lifetime must be exactly 300 seconds")
        if data["one_shot"] is not True or type(data["one_shot"]) is not bool:
            _fail("authorization_one_shot", "runtime authorization must be one-shot")
    elif schema == PROCESS_MANIFEST_V2_SCHEMA:
        if data["identity_algorithm"] != "sha256:ctr-slice-7g-process-manifest-canonical-1":
            _fail("process_identity_algorithm", "process identity algorithm differs")
        interpreter = _process_file_identity(data["interpreter"], "$.interpreter")
        entrypoint = _process_file_identity(data["entrypoint"], "$.entrypoint")
        if type(data["executables"]) is not list or not data["executables"]:
            _fail("process_executables", "process executable inventory differs")
        executables = [
            _process_file_identity(item, f"$.executables[{index}]")
            for index, item in enumerate(data["executables"])
        ]
        files = [interpreter, entrypoint, *executables]
        if len({item["path"] for item in files}) != len(files) or len({(item["device"], item["inode"]) for item in files}) != len(files):
            _fail("process_executable_alias", "process executable inventory contains an alias")
        if data["interpreter_flags"] != ["-I"]:
            _fail("process_interpreter_flags", "Python isolated mode is required")
        if type(data["argv_template"]) is not list or any(type(item) is not str for item in data["argv_template"]):
            _fail("process_argv", "process argv template differs")
        if data["argv_template"][:3] != [interpreter["path"], "-I", entrypoint["path"]]:
            _fail("process_argv", "process argv prefix differs")
        if type(data["transaction_slots"]) is not dict or any(
            type(key) is not str or type(value) is not str
            for key, value in data["transaction_slots"].items()
        ):
            _fail("process_slots", "process transaction slots differ")
        if type(data["allowed_descendants"]) is not list:
            _fail("process_descendants", "process descendant inventory differs")
        roles = []
        for index, descendant in enumerate(data["allowed_descendants"]):
            item = _exact_mapping(descendant, {
                "executable_identity", "multiplicity", "parent_role", "role",
            }, f"$.allowed_descendants[{index}]")
            _digest(item["executable_identity"], f"$.allowed_descendants[{index}].executable_identity")
            _counter(item["multiplicity"], 1, 65_536, f"$.allowed_descendants[{index}].multiplicity")
            _plain_string(item["parent_role"], f"$.allowed_descendants[{index}].parent_role")
            roles.append(_plain_string(item["role"], f"$.allowed_descendants[{index}].role"))
        if roles != sorted(roles) or len(roles) != len(set(roles)):
            _fail("process_descendants", "descendant roles must be unique and sorted")
        timeouts = _exact_mapping(data["timeouts"], {
            "cell_seconds", "sigint_seconds", "sigkill_seconds", "sigterm_seconds",
        }, "$.timeouts")
        if any(type(value) is not float or not 0.0 < value <= 3600.0 for value in timeouts.values()):
            _fail("process_timeouts", "process timeout map differs")
        ownership = _exact_mapping(data["output_ownership"], {
            "authority_owner", "campaign_account", "cell_mode", "root_mode",
            "runtime_group", "stderr_role", "stdout_role",
        }, "$.output_ownership")
        for field in ("authority_owner", "campaign_account", "runtime_group", "stderr_role", "stdout_role"):
            _plain_string(ownership[field], f"$.output_ownership.{field}")
        for field in ("cell_mode", "root_mode"):
            if type(ownership[field]) is not int or not 0 <= ownership[field] <= 0o7777:
                _fail("process_output_mode", "output ownership mode differs", path=f"$.output_ownership.{field}")
        _string_inventory(data["required_receipts"], "$.required_receipts")
        if data["shell"] is not False or type(data["shell"]) is not bool:
            _fail("process_shell", "process manifest requires shell=false")
        if data["systemd_unit"] != "ctr-slice7g-campaign.service" or data["cgroup"] != CAMPAIGN_CGROUP:
            _fail("process_cgroup", "campaign unit/cgroup differs")
        _digest(data["environment_manifest_identity"], "$.environment_manifest_identity")
        _optional_digest(
            data["privileged_service_manifest_identity"],
            "$.privileged_service_manifest_identity",
        )
        observer = data["observer_contract"]
        if type(observer) is not dict or set(observer) != {
            "class", "executable", "argv", "shell", "timeout_seconds",
            "stdout_limit_bytes", "stderr_limit_bytes", "retries", "concurrency",
        }:
            _fail("observer_contract", "observer process contract fields differ")
        if observer != {
            "class": "PRECOMMIT_ROS_GRAPH_OBSERVER",
            "executable": OBSERVER_EXECUTABLE,
            "argv": list(OBSERVER_ARGV),
            "shell": False,
            "timeout_seconds": 10.0,
            "stdout_limit_bytes": MAX_OUTPUT_BYTES,
            "stderr_limit_bytes": MAX_OUTPUT_BYTES,
            "retries": 0,
            "concurrency": 1,
        }:
            _fail("observer_contract", "observer process contract differs")
    elif schema == OBSERVATION_SESSION_V3_SCHEMA:
        for field in (
            "authorization_identity", "installed_runtime_identity",
            "process_manifest_identity", "environment_manifest_identity",
            "privileged_service_manifest_identity", "daemon_generation_identity",
            "cleanup_head_identity",
        ):
            _digest(data[field], f"$.{field}")
        _counter(data["maximum_precommit_observers"], 100, 100, "$.maximum_precommit_observers")
        pre = _counter(data["precommit_observer_count"], 0, 100, "$.precommit_observer_count")
        post = _counter(data["postcommit_observer_count"], 0, 1, "$.postcommit_observer_count")
        total = _counter(data["transaction_observer_count"], 0, 101, "$.transaction_observer_count")
        if total != pre + post:
            _fail("observer_counter", "observer counter sum differs")
        for field in ("peer_uid", "peer_gid", "peer_pid", "peer_start_time_ticks"):
            _counter(data[field], 0, 2**63 - 1, f"$.{field}")
        if data["peer_pid"] == 0 or data["peer_start_time_ticks"] == 0:
            _fail("observation_peer", "observation peer identity differs")
        _plain_string(data["connection_identity"], "$.connection_identity")
        _plain_string(data["campaign_cgroup"], "$.campaign_cgroup")
        _nonce(data["service_nonce"], "$.service_nonce")
        for field in ("created_monotonic_ns", "deadline_monotonic_ns"):
            _counter(data[field], 0, 2**63 - 1, f"$.{field}")
        if data["deadline_monotonic_ns"] - data["created_monotonic_ns"] != 1_800_000_000_000:
            _fail("observation_lifetime", "observation lifetime differs")
        if data["domain_minimum"] != 100 or type(data["domain_minimum"]) is not int or data["domain_maximum"] != 199 or type(data["domain_maximum"]) is not int:
            _fail("observation_domain_range", "observation domain range differs")
        candidates = data["candidate_domains"]
        if type(candidates) is not list or any(type(item) is not int or not 100 <= item <= 199 for item in candidates):
            _fail("observation_candidates", "observation candidate inventory differs")
        if candidates != sorted(candidates) or len(candidates) != len(set(candidates)) or len(candidates) != pre:
            _fail("observation_candidates", "observation candidates must be unique, ordered, and counter-bound")
        receipts = _digest_inventory(data["precommit_receipt_identities"], "$.precommit_receipt_identities", maximum=100)
        if len(receipts) != pre:
            _fail("observer_counter", "receipt count differs from precommit count")
        if data["selected_domain"] is not None and (
            type(data["selected_domain"]) is not int or data["selected_domain"] not in candidates
        ):
            _fail("observation_selected_domain", "selected domain was not observed")
        for field in ("lease_identity", "four_source_observation_identity"):
            _optional_digest(data[field], f"$.{field}")
        if data["state"] not in {"OPEN", "OBSERVED", "PREPARED", "INVALIDATED"}:
            _fail("observation_state", "observation session state differs")
    elif schema == FOUR_SOURCE_OBSERVATION_V4_SCHEMA:
        if data["global_lease_state"] not in LEASE_STATES:
            _fail("lease_state", "unknown four-source lease state")
        if type(data["global_lease_clear"]) is not bool or data["global_lease_clear"] is not (data["global_lease_state"] == "CLEAR"):
            _fail("lease_clear", "four-source lease state/clearance differs")
        if type(data["all_sources_clear"]) is not bool:
            _fail("four_source_clear", "four-source clearance must be exact Boolean")
        if data["all_sources_clear"] and not data["global_lease_clear"]:
            _fail("four_source_clear", "four-source clearance requires CLEAR lease state")
        _nonce(data["service_nonce"], "$.service_nonce")
        _domain(data["domain_id"], "$.domain_id")
        for field in ("phase_local_ordinal", "transaction_observer_ordinal"):
            _counter(data[field], 1, 101, f"$.{field}")
        if data["phase"] not in {"PRECOMMIT", "POSTCOMMIT"}:
            _fail("observation_phase", "four-source phase differs")
        for field in (
            "session_binding_identity", "peer_process_identity",
            "observation_interval_identity", "cleanup_disposition_identity",
            "active_process_identity", "dds_port_identity", "global_lease_identity",
            "global_lease_registry_identity", "global_lease_revision_identity",
            "ros_graph_provider_identity", "cleanup_head_identity",
            "containment_receipt_identity",
        ):
            _digest(data[field], f"$.{field}")
        _counter(data["observed_monotonic_ns"], 0, 2**63 - 1, "$.observed_monotonic_ns")
    elif schema == ROS_GRAPH_RECEIPT_V3_SCHEMA:
        if data["executable"] != OBSERVER_EXECUTABLE or data["argv"] != [OBSERVER_EXECUTABLE, *OBSERVER_ARGV]:
            _fail("observer_command", "observer command differs")
        if data["shell"] is not False or data["stderr_size"] != 0:
            _fail("observer_result", "observer shell/stderr disposition differs")
        if data["stdout_size"] > MAX_OUTPUT_BYTES or data["stderr_size"] > MAX_OUTPUT_BYTES:
            _fail("observer_output", "observer output exceeds its bound")
        if type(data["stdout_size"]) is not int or type(data["stderr_size"]) is not int:
            _fail("observer_output", "observer output size must be exact integer")
        if data["phase"] not in {"PRECOMMIT", "POSTCOMMIT"}:
            _fail("observer_phase", "observer phase differs")
        _nonce(data["service_nonce"], "$.service_nonce")
        _domain(data["domain_id"], "$.domain_id")
        for field in (
            "phase_local_ordinal", "transaction_observer_ordinal", "pid",
            "process_group_id", "process_start_time_ticks", "started_monotonic_ns",
            "ended_monotonic_ns", "unexpected_descendants",
        ):
            _counter(data[field], 0 if field in {"started_monotonic_ns", "ended_monotonic_ns", "unexpected_descendants"} else 1, 2**63 - 1, f"$.{field}")
        if data["ended_monotonic_ns"] < data["started_monotonic_ns"]:
            _fail("observer_interval", "observer interval is reversed")
        if type(data["terminating_signal"]) not in {int, type(None)} or type(data["exit_status"]) is not int:
            _fail("observer_exit", "observer exit disposition differs")
        if type(data["ros_daemon_started"]) is not bool or type(data["shell"]) is not bool:
            _fail("observer_boolean", "observer Boolean disposition differs")
        if type(data["nodes"]) is not list or any(type(node) is not str for node in data["nodes"]):
            _fail("observer_nodes", "observer node inventory differs")
        if data["nodes"] != sorted(data["nodes"]) or len(data["nodes"]) != len(set(data["nodes"])):
            _fail("observer_nodes", "observer node inventory must be unique and ordered")
        _digest_inventory(data["module_origin_identities"], "$.module_origin_identities")
        for field in (
            "session_binding_identity", "four_source_observation_identity",
            "executable_identity", "interpreter_identity", "environment_identity",
            "parsed_node_set_identity", "cleanup_barrier_identity", "stdout_sha256",
            "stderr_sha256", "cleanup_head_identity", "containment_receipt_identity",
        ):
            _digest(data[field], f"$.{field}")
    elif schema == GLOBAL_ATTEMPT_BUDGET_V4_SCHEMA:
        revision = _counter(data["revision"], 0, 10**20 - 1, "$.revision")
        if (revision == 0) != (data["predecessor_identity"] is None):
            _fail("budget_predecessor", "budget predecessor relationship differs")
        if data["state"] not in {"UNCONSUMED", "COMMITTED", "COMPLETED", "FAILED_AFTER_COMMIT"}:
            _fail("budget_state", "unknown budget state")
        if data["attempts_maximum"] != 1 or data["retries_authorized"] != 0:
            _fail("budget_scope", "budget maximum/retries differ")
        if data["state"] == "UNCONSUMED":
            if revision != 0 or data["attempts_consumed"] != 0:
                _fail("budget_state", "UNCONSUMED is revision zero only")
        elif data["attempts_consumed"] != 1:
            _fail("budget_state", "postcommit budget must consume 1/1")
        for field in (
            "attempts_consumed", "attempts_maximum", "retries_authorized",
            "precommit_observer_count", "postcommit_observer_count",
            "transaction_observer_count",
        ):
            _counter(data[field], 0, 101, f"$.{field}")
        if data["attempts_maximum"] != 1 or data["retries_authorized"] != 0:
            _fail("budget_scope", "budget scope differs")
        if data["transaction_observer_count"] != data["precommit_observer_count"] + data["postcommit_observer_count"]:
            _fail("observer_counter", "budget observer counter sum differs")
        receipts = _digest_inventory(data["precommit_receipt_identities"], "$.precommit_receipt_identities", maximum=100)
        if len(receipts) != data["precommit_observer_count"]:
            _fail("observer_counter", "budget receipt count differs")
        for field in (
            "predecessor_identity", "authorization_identity", "observation_session_identity",
            "four_source_observation_identity", "cleanup_head_identity",
            "containment_receipt_identity", "postcommit_receipt_identity",
            "postcommit_four_source_observation_identity",
        ):
            _optional_digest(data[field], f"$.{field}")
        if data["process_start_commitment"] is not None:
            _process_start_commitment(data["process_start_commitment"], "$.process_start_commitment")
        _plain_string(data["updated_at_utc"], "$.updated_at_utc")
    elif schema == RUNTIME_AUTHORITY_REQUEST_V4_SCHEMA:
        if data["method"] not in {
            "begin_observation", "record_precommit_observation", "finalize_observation",
            "prepare", "allocate_provisional", "cancel", "commit",
            "record_postcommit_observation", "complete", "fail_after_commit",
            "status", "revoke",
        }:
            _fail("authority_method", "unknown runtime-authority method")
        _digest(data["privileged_service_manifest_identity"], "$.privileged_service_manifest_identity")
        _plain_string(data["request_id"], "$.request_id")
        _plain_string(data["requested_at_utc"], "$.requested_at_utc")
        for field in (
            "authorization_identity", "campaign_identity", "campaign_template_identity",
            "output_root_identity", "process_manifest_identity", "process_instance_identity",
            "observation_session_identity",
        ):
            _optional_digest(data[field], f"$.{field}")
        _optional_domain(data["domain_id"], "$.domain_id")
        for field in ("prepare_token", "campaign_id", "observation_session_nonce"):
            if data[field] is not None:
                _nonce(data[field], f"$.{field}")
        if data["output_root_path"] is not None and (
            type(data["output_root_path"]) is not str
            or not PurePosixPath(data["output_root_path"]).is_absolute()
        ):
            _fail("authority_path", "authority output path differs", path="$.output_root_path")
    elif schema == RUNTIME_AUTHORITY_RECEIPT_V4_SCHEMA:
        if data["result"] not in {
            "OBSERVATION_STARTED", "OBSERVATION_RECORDED", "OBSERVATION_COMPLETE",
            "PREPARED", "ALLOCATED", "CANCELLED", "COMMITTED",
            "POSTCOMMIT_RECORDED", "COMPLETED", "FAILED_AFTER_COMMIT", "REVOKED",
            "STATUS", "ERROR",
        }:
            _fail("authority_result", "unknown runtime-authority result")
        for field in ("cleanup_head_identity", "containment_receipt_identity"):
            _optional_digest(data[field], f"$.{field}")
        _plain_string(data["request_id"], "$.request_id")
        _nonce(data["service_nonce"], "$.service_nonce")
        for field in (
            "authorization_identity", "service_instance_identity", "budget_identity",
            "campaign_identity", "campaign_template_identity", "output_root_identity",
            "process_manifest_identity", "process_instance_identity",
            "observation_session_identity", "four_source_observation_identity",
            "lease_identity",
        ):
            _optional_digest(data[field], f"$.{field}")
        _digest_inventory(data["precommit_receipt_identities"], "$.precommit_receipt_identities", maximum=100)
        for field in (
            "previous_budget_revision", "budget_revision", "precommit_observer_count",
            "postcommit_observer_count", "transaction_observer_count",
            "observation_session_deadline_monotonic_ns", "prepare_expires_monotonic_ns",
        ):
            if data[field] is not None:
                _counter(data[field], 0, 2**63 - 1, f"$.{field}")
        if type(data["candidate_clear"]) not in {bool, type(None)}:
            _fail("authority_boolean", "candidate clearance type differs", path="$.candidate_clear")
    elif schema == PRIVILEGED_REQUEST_SCHEMA:
        if data["operation"] not in OPERATIONS:
            _fail("operation", "unknown privileged operation", path="$.operation")
        _counter(data["sequence"], 0, MAX_FRAMES_PER_CONNECTION - 1, "$.sequence")
        for field in ("connection_nonce", "request_nonce"):
            _nonce(data[field], f"$.{field}")
        _nonce(data["operation_token"], "$.operation_token")
        for field in (
            "service_generation_identity", "runtime_authorization_identity",
            "installed_runtime_identity", "budget_identity", "cleanup_head_identity",
            "session_binding_identity", "observer_contract_identity", "containment_identity",
            "process_identity", "disposition_identity", "recovery_authorization_identity",
        ):
            _optional_digest(data[field], f"$.{field}")
        _optional_domain(data["domain_id"], "$.domain_id")
        if data["phase"] not in (None, "PRECOMMIT", "POSTCOMMIT", "RECOVERY"):
            _fail("phase", "invalid privileged phase", path="$.phase")
        for field in ("phase_local_ordinal", "transaction_observer_ordinal"):
            if data[field] is not None:
                _counter(data[field], 1, 101, f"$.{field}")
        if data["transition"] not in (None, *CLEANUP_STATES):
            _fail("cleanup_transition", "unknown requested cleanup transition")
        forbidden = ("executable", "argv", "environment", "cwd", "signal", "pid", "pgid", "cgroup")
        if any(field in data for field in forbidden):
            _fail("caller_process_authority", "caller process authority is prohibited")
    elif schema == PRIVILEGED_RECEIPT_SCHEMA:
        if data["operation"] not in OPERATIONS:
            _fail("operation", "unknown privileged operation", path="$.operation")
        _counter(data["sequence"], 0, MAX_FRAMES_PER_CONNECTION - 1, "$.sequence")
        for field in ("connection_nonce", "request_nonce"):
            _nonce(data[field], f"$.{field}")
        _nonce(data["operation_token"], "$.operation_token")
        _digest(data["service_generation_identity"], "$.service_generation_identity")
        if data["result"] not in {"OK", "ERROR", "STARTED", "RUNNING", "CLEANED", "RECOVERED"}:
            _fail("result", "invalid privileged result", path="$.result")
        if data["result"] == "ERROR" and type(data["error_code"]) is not str:
            _fail("error_code", "error receipt requires a stable code", path="$.error_code")
        if data["result"] != "ERROR" and data["error_code"] is not None:
            _fail("error_code", "success receipt cannot carry an error", path="$.error_code")
        _counter(data["output_descriptor_count"], 0, MAX_TRANSFERRED_FDS, "$.output_descriptor_count")
        for field in ("cleanup_head_identity", "containment_receipt_identity", "payload_identity"):
            _optional_digest(data[field], f"$.{field}")
        for field, nested_schema in (
            ("cleanup_revision", CLEANUP_REVISION_SCHEMA),
            ("cleanup_anchor", CLEANUP_ANCHOR_SCHEMA),
            ("cleanup_head", CLEANUP_HEAD_SCHEMA),
            ("containment_receipt", OBSERVER_CONTAINMENT_RECEIPT_SCHEMA),
        ):
            if data[field] is not None:
                if type(data[field]) is not dict:
                    _fail("nested_record", f"{field} must be an exact record")
                validate_record(data[field], expected_schema=nested_schema)
    elif schema == CLEANUP_REVISION_SCHEMA:
        _counter(data["revision"], 0, 10**20 - 1, "$.revision")
        if data["state"] not in CLEANUP_STATES:
            _fail("cleanup_state", "unknown cleanup state", path="$.state")
        _optional_digest(data["predecessor_identity"], "$.predecessor_identity")
        if (data["revision"] == 0) != (data["predecessor_identity"] is None):
            _fail("cleanup_predecessor", "revision/predecessor relationship differs")
        for field in (
            "runtime_authorization_identity", "budget_identity", "service_generation_identity",
            "session_binding_identity", "observer_contract_identity", "containment_identity",
            "process_identity", "disposition_identity", "recovery_authorization_identity",
        ):
            _optional_digest(data[field], f"$.{field}")
        if data["state"] in {"ACTIVE_UNBOUND", "ACTIVE_BOUND", "QUARANTINED"}:
            for field in ("runtime_authorization_identity", "budget_identity", "service_generation_identity", "session_binding_identity"):
                _digest(data[field], f"$.{field}")
        if data["state"] == "ACTIVE_BOUND":
            _digest(data["containment_identity"], "$.containment_identity")
            _digest(data["process_identity"], "$.process_identity")
        if data["phase"] not in (None, "PRECOMMIT", "POSTCOMMIT", "RECOVERY"):
            _fail("cleanup_phase", "cleanup phase differs")
        _optional_domain(data["domain_id"], "$.domain_id")
        for field in ("phase_local_ordinal", "transaction_observer_ordinal"):
            if data[field] is not None:
                _counter(data[field], 1, 101, f"$.{field}")
        _plain_string(data["created_at_utc"], "$.created_at_utc")
    elif schema == CLEANUP_ANCHOR_SCHEMA:
        _physical_record(data, "revision")
        _digest(data["authority_root_identity"], "$.authority_root_identity")
        _digest(data["revision_identity"], "$.revision_identity")
        _optional_digest(data["predecessor_anchor_identity"], "$.predecessor_anchor_identity")
    elif schema == CLEANUP_HEAD_SCHEMA:
        _physical_record(data, "anchor")
        for field in ("authority_root_identity", "revision_identity", "anchor_identity"):
            _digest(data[field], f"$.{field}")
        _optional_digest(data["predecessor_head_identity"], "$.predecessor_head_identity")
    elif schema == GLOBAL_LEASE_OBSERVATION_V2_SCHEMA:
        if data["state"] not in LEASE_STATES:
            _fail("lease_state", "unknown lease state", path="$.state")
        if type(data["clear"]) is not bool or data["clear"] is not (data["state"] == "CLEAR"):
            _fail("lease_clear", "only CLEAR may promote clearance", path="$.clear")
        _domain(data["domain_id"], "$.domain_id")
        for field in (
            "registry_identity", "registry_revision_identity", "physical_observation_identity",
            "session_binding_identity", "observation_interval_identity",
        ):
            _digest(data[field], f"$.{field}")
        _nonce(data["service_nonce"], "$.service_nonce")
        if data["phase"] not in {"PRECOMMIT", "POSTCOMMIT", "RECOVERY"}:
            _fail("lease_phase", "lease observation phase differs")
        for field in ("phase_local_ordinal", "transaction_observer_ordinal"):
            _counter(data[field], 1, 101, f"$.{field}")
        _counter(data["observed_monotonic_ns"], 0, 2**63 - 1, "$.observed_monotonic_ns")
        for field in (
            "record_physical_identities", "active_reservation_identities",
            "committed_binding_identities", "stale_invalid_identities",
        ):
            _digest_inventory(data[field], f"$.{field}")
        for field in ("owner_bindings", "output_root_bindings"):
            _string_inventory(data[field], f"$.{field}")
    elif schema == OBSERVER_CONTAINMENT_RECEIPT_SCHEMA:
        if data["disposition"] not in {"CLEARED", "QUARANTINED"}:
            _fail("containment_disposition", "containment disposition differs")
        if data["leaf_cgroup"] is not None and not OBSERVER_LEAF_PATTERN.fullmatch(data["leaf_cgroup"]):
            _fail("containment_cgroup", "observer leaf cgroup differs")
        if type(data["leaf_removed"]) is not bool:
            _fail("containment_leaf", "leaf removal must be exact Boolean")
        if data["disposition"] == "CLEARED" and not data["leaf_removed"]:
            _fail("containment_clear", "CLEARED requires removed leaf")
        _counter(data["stable_empty_samples"], 0, 1000, "$.stable_empty_samples")
        for field in (
            "pid", "process_start_time_ticks", "process_group_id", "session_id",
            "started_monotonic_ns", "ended_monotonic_ns", "stdout_size",
            "stderr_size", "stable_empty_span_ns",
        ):
            _counter(data[field], 0, 2**63 - 1, f"$.{field}")
        if any(data[field] == 0 for field in ("pid", "process_start_time_ticks", "process_group_id", "session_id")):
            _fail("containment_process", "containment process identity differs")
        if data["ended_monotonic_ns"] < data["started_monotonic_ns"]:
            _fail("containment_interval", "containment interval is reversed")
        if type(data["exit_status"]) is not int or type(data["terminating_signal"]) not in {int, type(None)}:
            _fail("containment_exit", "containment exit disposition differs")
        if data["stdout_size"] > MAX_OUTPUT_BYTES or data["stderr_size"] > MAX_OUTPUT_BYTES:
            _fail("containment_output", "containment output exceeds its bound")
        _domain(data["domain_id"], "$.domain_id")
        if data["phase"] not in {"PRECOMMIT", "POSTCOMMIT", "RECOVERY"}:
            _fail("containment_phase", "containment phase differs")
        for field in ("phase_local_ordinal", "transaction_observer_ordinal"):
            _counter(data[field], 1, 101, f"$.{field}")
        _nonce(data["operation_token"], "$.operation_token")
        for field in (
            "service_generation_identity", "session_binding_identity",
            "runtime_authorization_identity", "budget_identity",
            "cleanup_active_head_identity", "cleanup_terminal_head_identity",
            "leaf_cgroup_identity", "pidfd_identity", "procfd_identity",
            "executable_identity", "interpreter_identity", "argv_identity",
            "environment_identity", "postexec_identity", "working_directory_identity",
            "stdout_sha256", "stderr_sha256", "cleanup_barrier_identity",
        ):
            _digest(data[field], f"$.{field}")
    elif schema == CLEANUP_RECOVERY_AUTHORIZATION_V2_SCHEMA:
        _nonce(data["recovery_nonce"], "$.recovery_nonce")
        for field in (
            "quarantine_head_identity", "quarantine_anchor_identity",
            "runtime_authorization_identity", "installed_runtime_identity",
            "budget_identity", "cleanup_service_generation_identity",
            "observer_service_generation_identity",
        ):
            _digest(data[field], f"$.{field}")
        for field in ("issued_at_utc", "not_before_utc", "not_after_utc"):
            if type(data[field]) is not str or len(data[field].encode("utf-8")) > 64:
                _fail("recovery_time", "recovery timestamp differs", path=f"$.{field}")
        if type(data["one_shot"]) is not bool or data["one_shot"] is not True:
            _fail("recovery_one_shot", "recovery authorization must be one-shot")
    elif schema == CLEANUP_RECOVERY_PROVIDER_RECEIPT_SCHEMA:
        if data["provider"] not in RECOVERY_PROVIDERS:
            _fail("recovery_provider", "unknown recovery provider")
        if type(data["clear"]) is not bool:
            _fail("recovery_clear", "provider result must be exact Boolean")
        for field in (
            "provider_identity", "quarantine_head_identity", "quarantine_anchor_identity",
            "recovery_authorization_identity", "service_generation_identity",
            "runtime_authorization_identity", "budget_identity", "evidence_identity",
            "cleanup_disposition_identity",
        ):
            _digest(data[field], f"$.{field}")
        _nonce(data["recovery_nonce"], "$.recovery_nonce")
        _domain(data["domain_id"], "$.domain_id")
        _counter(data["ordinal"], 1, 4, "$.ordinal")
        for field in ("started_monotonic_ns", "ended_monotonic_ns"):
            _counter(data[field], 0, 2**63 - 1, f"$.{field}")
        if data["ended_monotonic_ns"] < data["started_monotonic_ns"]:
            _fail("recovery_interval", "recovery provider interval is reversed")
        if data["phase"] != "RECOVERY":
            _fail("recovery_phase", "recovery provider phase differs")
    elif schema == CLEANUP_RECOVERY_OBSERVATION_SCHEMA:
        identities = data["provider_receipt_identities"]
        if type(identities) is not list or len(identities) != 4:
            _fail("recovery_receipts", "recovery requires exactly four provider receipts")
        for index, item in enumerate(identities):
            _digest(item, f"$.provider_receipt_identities[{index}]")
        if len(set(identities)) != 4 or type(data["all_sources_clear"]) is not bool:
            _fail("recovery_receipts", "recovery receipts are duplicate or malformed")
        _nonce(data["recovery_nonce"], "$.recovery_nonce")
        _domain(data["domain_id"], "$.domain_id")
        for field in (
            "quarantine_head_identity", "quarantine_anchor_identity",
            "recovery_authorization_identity", "runtime_authorization_identity",
            "budget_identity", "service_generation_identity",
        ):
            _digest(data[field], f"$.{field}")
        _counter(data["observed_monotonic_ns"], 0, 2**63 - 1, "$.observed_monotonic_ns")
    elif schema == PRIVILEGED_SERVICE_MANIFEST_SCHEMA:
        expected = {
            "cleanup_service": CLEANUP_AUTHORITY_EXECUTABLE,
            "observer_service": OBSERVER_SUPERVISOR_EXECUTABLE,
            "cleanup_state_root": CLEANUP_AUTHORITY_STATE_ROOT,
            "cleanup_socket": CLEANUP_AUTHORITY_SOCKET,
            "recovery_socket": CLEANUP_RECOVERY_SOCKET,
            "observer_socket": OBSERVER_SUPERVISOR_SOCKET,
            "cleanup_principal": "root",
            "observer_supervisor_principal": "root",
            "observer_principal": OBSERVER_ACCOUNT,
            "recovery_principal": RECOVERY_ACCOUNT,
            "supervisor_cgroup": OBSERVER_SUPERVISOR_CGROUP,
            "observer_leaf_grammar": OBSERVER_LEAF_PATTERN.pattern,
            "observer_executable": OBSERVER_EXECUTABLE,
            "observer_argv": list(OBSERVER_ARGV),
            "protocol_schema": PRIVILEGED_REQUEST_SCHEMA,
            "containment_receipt_schema": OBSERVER_CONTAINMENT_RECEIPT_SCHEMA,
        }
        for field, expected_value in expected.items():
            if data[field] != expected_value:
                _fail("privileged_service_manifest", f"{field} differs", path=f"$.{field}")
        if type(data["numeric_ids_provisioned"]) is not bool:
            _fail("numeric_ids", "numeric provisioning marker must be an exact Boolean")
        _digest(data["environment_manifest_identity"], "$.environment_manifest_identity")
        if (
            type(data["working_directory"]) is not str
            or PurePosixPath(data["working_directory"]).parent.as_posix()
            != INSTALLED_RUNTIME_PARENT
        ):
            _fail("privileged_service_manifest", "working directory differs")
        if data["cleanup_schemas"] != [
            CLEANUP_REVISION_SCHEMA, CLEANUP_ANCHOR_SCHEMA, CLEANUP_HEAD_SCHEMA,
        ]:
            _fail("privileged_service_manifest", "cleanup schema inventory differs")
        for field in ("service_executable_identities", "systemd_unit_identities"):
            values = data[field]
            if type(values) is not list or len(values) != 2:
                _fail("privileged_service_manifest", f"{field} inventory differs")
            for index, value in enumerate(values):
                _digest(value, f"$.{field}[{index}]")
            if values != sorted(values) or len(values) != len(set(values)):
                _fail("privileged_service_manifest", f"{field} must be unique and sorted")
    else:
        _validate_common_identity_fields(data)


def _validate_common_identity_fields(data: dict[str, Any]) -> None:
    for key, value in data.items():
        if key.endswith("_identity") and value is not None:
            _digest(value, f"$.{key}")
        elif key.endswith("_identities"):
            if type(value) is not list:
                _fail("identity_list", "identity inventory must be an exact list", path=f"$.{key}")
            for index, item in enumerate(value):
                _digest(item, f"$.{key}[{index}]")


def encode_packet(value: dict[str, Any] | bytes, *, expected_schema: str) -> bytes:
    payload = canonical_bytes(value, expected_schema=expected_schema)
    if len(payload) > MAX_FRAME_BYTES:
        _fail("frame_size", "privileged frame exceeds maximum")
    return FRAME_HEADER.pack(len(payload)) + payload


def decode_packet(packet: bytes, *, expected_schema: str) -> PrivilegedRecord:
    if type(packet) is not bytes or len(packet) < FRAME_HEADER.size:
        _fail("frame_header", "privileged packet header is truncated")
    size = FRAME_HEADER.unpack(packet[:4])[0]
    payload = packet[4:]
    if size > MAX_FRAME_BYTES:
        _fail("frame_size", "privileged packet exceeds maximum")
    if size != len(payload):
        _fail("frame_length", "privileged packet length or trailing bytes differ")
    return validate_record(payload, expected_schema=expected_schema)


def send_packet(channel: socket.socket, value: dict[str, Any] | bytes, *, expected_schema: str,
                descriptors: tuple[int, ...] = ()) -> None:
    if type(channel) is not socket.socket or channel.family != socket.AF_UNIX or channel.type & 0xF != socket.SOCK_SEQPACKET:
        _fail("socket_type", "privileged channel must be AF_UNIX SOCK_SEQPACKET")
    if type(descriptors) is not tuple or len(descriptors) > MAX_TRANSFERRED_FDS:
        _fail("descriptor_count", "privileged descriptor count differs")
    if any(type(item) is not int or item < 0 for item in descriptors):
        _fail("descriptor_type", "transferred descriptors must be exact integers")
    ancillary = []
    if descriptors:
        ancillary = [(socket.SOL_SOCKET, socket.SCM_RIGHTS, array.array("i", descriptors))]
    packet = encode_packet(value, expected_schema=expected_schema)
    try:
        written = channel.sendmsg([packet], ancillary)
    except OSError as exc:
        raise Slice7GPrivilegedProtocolError("frame_write", type(exc).__name__) from exc
    if written != len(packet):
        _fail("frame_write", "privileged packet was partially written")


def receive_packet(channel: socket.socket, *, expected_schema: str,
                   expected_descriptors: int | None = 0) -> tuple[PrivilegedRecord, tuple[int, ...]]:
    if expected_descriptors is not None and (
        type(expected_descriptors) is not int
        or not 0 <= expected_descriptors <= MAX_TRANSFERRED_FDS
    ):
        _fail("descriptor_count", "expected descriptor count differs")
    ancillary_size = socket.CMSG_SPACE(MAX_TRANSFERRED_FDS * array.array("i").itemsize)
    try:
        packet, ancillary, flags, _ = channel.recvmsg(
            4 + MAX_FRAME_BYTES + 1, ancillary_size,
        )
    except OSError as exc:
        raise Slice7GPrivilegedProtocolError("frame_read", type(exc).__name__) from exc
    received: list[int] = []
    try:
        if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
            _fail("frame_truncated", "privileged packet or descriptor control is truncated")
        for level, kind, payload in ancillary:
            if level != socket.SOL_SOCKET or kind != socket.SCM_RIGHTS:
                _fail("ancillary_type", "unknown ancillary record")
            values = array.array("i")
            usable = len(payload) - (len(payload) % values.itemsize)
            values.frombytes(payload[:usable])
            received.extend(values)
        if len(received) > MAX_TRANSFERRED_FDS or (
            expected_descriptors is not None and len(received) != expected_descriptors
        ):
            _fail("descriptor_count", "received descriptor count differs")
        return decode_packet(packet, expected_schema=expected_schema), tuple(received)
    except BaseException:
        for descriptor in received:
            try:
                os.close(descriptor)
            except OSError:
                pass
        raise


def peer_credentials(channel: socket.socket) -> PeerCredentials:
    if type(channel) is not socket.socket or channel.family != socket.AF_UNIX:
        _fail("socket_family", "peer credential channel is not AF_UNIX")
    try:
        raw = _socket_peercred(channel)
        values = struct.unpack("3i", raw)
    except (OSError, struct.error) as exc:
        raise Slice7GPrivilegedProtocolError("peer_credentials", type(exc).__name__) from exc
    return PeerCredentials(*values)


def _socket_peercred(channel: socket.socket) -> bytes:
    """Private syscall seam used only by synthetic failure-normalization tests."""
    return channel.getsockopt(
        socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i"),
    )


def verify_response_binding(
    request: PrivilegedRecord, response: PrivilegedRecord, *,
    expected_service_generation_identity: str | None,
    expected_descriptor_count: int, descriptors: tuple[int, ...],
    peer: PeerProcess | None = None,
) -> str:
    """Authenticate one helper response against its exact request and connection."""
    if type(request) is not PrivilegedRecord or request.schema_version != PRIVILEGED_REQUEST_SCHEMA:
        _fail("response_request", "response request binding is not authenticated")
    if type(response) is not PrivilegedRecord or response.schema_version != PRIVILEGED_RECEIPT_SCHEMA:
        _fail("response_schema", "response schema is not authenticated")
    if type(descriptors) is not tuple or len(descriptors) != expected_descriptor_count:
        _fail("response_descriptors", "response descriptor count differs")
    if response.data["output_descriptor_count"] != expected_descriptor_count:
        _fail("response_descriptors", "response descriptor binding differs")
    for field in (
        "operation", "sequence", "connection_nonce", "request_nonce", "operation_token",
    ):
        if response.data[field] != request.data[field]:
            _fail("response_binding", f"response {field} differs", path=f"$.{field}")
    generation = _digest(
        response.data["service_generation_identity"], "$.service_generation_identity",
    )
    if (
        expected_service_generation_identity is not None
        and generation != expected_service_generation_identity
    ):
        _fail("response_generation", "response service generation differs")
    if request.data["service_generation_identity"] not in (None, generation):
        _fail("response_generation", "request targeted a different service generation")
    if peer is not None:
        reconcile_peer(peer)
    window = _CLIENT_RESPONSE_REPLAY.setdefault(generation, ReplayWindow(generation))
    window.claim(response)
    return generation


def observe_peer(credentials: PeerCredentials) -> PeerProcess:
    root = f"/proc/{credentials.pid}"
    try:
        raw = _read_bounded(root + "/stat", 65_536).decode("ascii", "strict")
        close = raw.rfind(")")
        fields = raw[close + 2:].split()
        start = int(fields[19])
        executable = os.readlink(root + "/exe")
        argv = tuple(item.decode("utf-8", "strict") for item in _read_bounded(root + "/cmdline", MAX_FRAME_BYTES).split(b"\0") if item)
        cgroups = _read_bounded(root + "/cgroup", 65_536).decode("utf-8", "strict").splitlines()
    except (OSError, UnicodeError, ValueError, IndexError) as exc:
        raise Slice7GPrivilegedProtocolError("peer_process", type(exc).__name__) from exc
    if close < 0 or len(cgroups) != 1 or not cgroups[0].startswith("0::/"):
        _fail("peer_process", "peer process identity is malformed")
    return PeerProcess(credentials, start, executable, argv, cgroups[0][3:])


def reconcile_peer(expected: PeerProcess) -> PeerProcess:
    observed = observe_peer(expected.credentials)
    if observed != expected:
        _fail("peer_replaced", "peer PID/start/executable/argv/cgroup changed")
    return observed


def authenticate_sealed_output(descriptor: int, *, expected_size: int,
                               expected_sha256: str) -> bytes:
    if type(descriptor) is not int or descriptor < 0:
        _fail("output_descriptor", "output descriptor must be an exact integer")
    if type(expected_size) is not int or not 0 <= expected_size <= MAX_OUTPUT_BYTES:
        _fail("output_size", "output size differs")
    _digest(expected_sha256, "$.expected_sha256")
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_size != expected_size:
            _fail("output_identity", "output descriptor is not the expected regular memfd")
        required = (
            fcntl.F_SEAL_WRITE | fcntl.F_SEAL_GROW
            | fcntl.F_SEAL_SHRINK | fcntl.F_SEAL_SEAL
        )
        seals = fcntl.fcntl(descriptor, fcntl.F_GET_SEALS)
        if seals & required != required:
            _fail("output_seals", "output memfd lacks required seals")
        os.lseek(descriptor, 0, os.SEEK_SET)
        chunks: list[bytes] = []
        remaining = expected_size
        while remaining:
            chunk = os.read(descriptor, min(65_536, remaining))
            if not chunk:
                _fail("output_read", "output memfd ended early")
            chunks.append(chunk)
            remaining -= len(chunk)
        if os.read(descriptor, 1):
            _fail("output_read", "output memfd contains trailing bytes")
        raw = b"".join(chunks)
        if hashlib.sha256(raw).hexdigest() != expected_sha256 or os.fstat(descriptor) != info:
            _fail("output_identity", "output memfd changed or digest differs")
        return raw
    except Slice7GPrivilegedProtocolError:
        raise
    except OSError as exc:
        raise Slice7GPrivilegedProtocolError("output_descriptor", type(exc).__name__) from exc


def make_sealed_memfd(name: str, payload: bytes) -> int:
    """Private service helper for bounded sealed output construction."""
    if type(name) is not str or not _NONCE.fullmatch(name):
        _fail("memfd_name", "memfd name differs")
    if type(payload) is not bytes or len(payload) > MAX_OUTPUT_BYTES:
        _fail("output_size", "memfd payload exceeds its bound")
    descriptor = os.memfd_create(name, os.MFD_CLOEXEC | os.MFD_ALLOW_SEALING)
    try:
        offset = 0
        while offset < len(payload):
            offset += os.write(descriptor, payload[offset:])
        os.fsync(descriptor)
        seals = fcntl.F_SEAL_WRITE | fcntl.F_SEAL_GROW | fcntl.F_SEAL_SHRINK | fcntl.F_SEAL_SEAL
        fcntl.fcntl(descriptor, fcntl.F_ADD_SEALS, seals)
        os.lseek(descriptor, 0, os.SEEK_SET)
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def _parse(raw: bytes) -> dict[str, Any]:
    if len(raw) > MAX_FRAME_BYTES:
        _fail("record_size", "record exceeds maximum")
    try:
        value = json.loads(raw.decode("utf-8", "strict"), object_pairs_hook=_pairs)
    except (UnicodeError, ValueError, json.JSONDecodeError) as exc:
        raise Slice7GPrivilegedProtocolError("record_json", type(exc).__name__) from exc
    if type(value) is not dict:
        _fail("record_type", "record JSON must be an object")
    return value


def _pairs(items: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in items:
        if key in result:
            _fail("duplicate_key", "duplicate JSON member")
        result[key] = value
    return result


def _detach(value: Any, path: str = "$", depth: int = 0) -> Any:
    if depth > 24:
        _fail("record_depth", "record nesting exceeds maximum", path=path)
    if type(value) is dict:
        return {key: _detach(item, f"{path}.{key}", depth + 1) for key, item in value.items()}
    if type(value) is list:
        if len(value) > 65_536:
            _fail("list_size", "record list exceeds maximum", path=path)
        return [_detach(item, f"{path}[{index}]", depth + 1) for index, item in enumerate(value)]
    if type(value) in (str, int, float, bool) or value is None:
        return value
    _fail("exact_type", "record contains a non-built-in primitive", path=path)


def _validate_json(value: Any, path: str, depth: int) -> None:
    if depth > 24:
        _fail("record_depth", "record nesting exceeds maximum", path=path)
    if type(value) is dict:
        if len(value) > 65_536:
            _fail("mapping_size", "record mapping exceeds maximum", path=path)
        for key, item in value.items():
            if type(key) is not str:
                _fail("exact_type", "record key must be an exact string", path=path)
            if len(key.encode("utf-8")) > 8_192:
                _fail("string_size", "record key exceeds maximum", path=path)
            _validate_json(item, f"{path}.{key}", depth + 1)
        return
    if type(value) is list:
        if len(value) > 65_536:
            _fail("list_size", "record list exceeds maximum", path=path)
        for index, item in enumerate(value):
            _validate_json(item, f"{path}[{index}]", depth + 1)
        return
    if type(value) is str and len(value.encode("utf-8")) > 8192:
        _fail("string_size", "string exceeds maximum", path=path)
    if type(value) is float and (value != value or value in (float("inf"), float("-inf"))):
        _fail("finite_number", "number must be finite", path=path)
    if type(value) not in (str, int, float, bool) and value is not None:
        _fail("exact_type", "record contains a non-built-in primitive", path=path)


def _canonical(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _freeze(value: Any) -> Any:
    if type(value) is dict:
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if type(value) is list:
        return tuple(_freeze(item) for item in value)
    return value


def _closed(value: dict[str, Any], expected: frozenset[str]) -> None:
    if set(value) != set(expected):
        _fail("closed_schema", f"field set differs: {sorted(set(value) ^ set(expected))!r}")


def _physical_record(data: dict[str, Any], prefix: str) -> None:
    _counter(data["revision"], 0, 10**20 - 1, "$.revision")
    for suffix in ("device", "inode", "size"):
        _counter(data[f"{prefix}_{suffix}"], 0, 2**63 - 1, f"$.{prefix}_{suffix}")
    if data[f"{prefix}_inode"] == 0 or data[f"{prefix}_size"] == 0:
        _fail("physical_identity", "physical inode and size must be positive")
    if data[f"{prefix}_mode"] != 0o400 or data[f"{prefix}_link_count"] != 1:
        _fail("physical_identity", "physical mode/link count differs")
    _digest(data[f"{prefix}_sha256"], f"$.{prefix}_sha256")


def _digest(value: Any, path: str) -> str:
    if type(value) is not str or _DIGEST.fullmatch(value) is None:
        _fail("digest", "expected lowercase SHA-256", path=path)
    return value


def _optional_digest(value: Any, path: str) -> None:
    if value is not None:
        _digest(value, path)


def _nonce(value: Any, path: str) -> str:
    if type(value) is not str or _NONCE.fullmatch(value) is None:
        _fail("nonce", "nonce differs", path=path)
    return value


def _optional_nonce(value: Any, path: str) -> None:
    if value is not None:
        _nonce(value, path)


def _counter(value: Any, minimum: int, maximum: int, path: str) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        _fail("counter", "counter differs", path=path)
    return value


def _domain(value: Any, path: str) -> int:
    return _counter(value, 100, 199, path)


def _optional_domain(value: Any, path: str) -> None:
    if value is not None:
        _domain(value, path)


def _read_bounded(path: str, maximum: int) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        chunks: list[bytes] = []
        total = 0
        while True:
            chunk = os.read(descriptor, min(65_536, maximum + 1 - total))
            if not chunk:
                break
            chunks.append(chunk)
            total += len(chunk)
            if total > maximum:
                _fail("record_size", "record exceeds maximum")
        return b"".join(chunks)
    finally:
        os.close(descriptor)


def safe_relative(value: str) -> PurePosixPath:
    if type(value) is not str:
        _fail("path", "path must be exact string")
    path = PurePosixPath(value)
    if path.is_absolute() or not path.parts or any(part in ("", ".", "..") for part in path.parts):
        _fail("path", "path must be a safe relative path")
    return path


def _fail(code: str, message: str, *, path: str = "$") -> None:
    raise Slice7GPrivilegedProtocolError(code, message, path=path)


__all__ = [
    "ALL_V7_SCHEMAS", "AUTHORITY_BOOTSTRAP_V2_SCHEMA", "AUTHORITY_BOOTSTRAP_V3_SCHEMA",
    "AUTHORITY_RUNTIME_DIRECTORY",
    "CLEANUP_ANCHOR_SCHEMA", "CLEANUP_AUTHORITY_EXECUTABLE",
    "CLEANUP_AUTHORITY_SERVICE", "CLEANUP_AUTHORITY_SOCKET",
    "CLEANUP_AUTHORITY_STATE_ROOT", "CLEANUP_AUTHORITY_RUNTIME_DIRECTORY", "CLEANUP_HEAD_SCHEMA",
    "CLEANUP_RECOVERY_AUTHORIZATION_V2_SCHEMA", "CLEANUP_RECOVERY_OBSERVATION_SCHEMA",
    "CLEANUP_RECOVERY_PROVIDER_RECEIPT_SCHEMA", "CLEANUP_RECOVERY_SOCKET",
    "CLEANUP_REVISION_SCHEMA", "FOUR_SOURCE_OBSERVATION_V4_SCHEMA",
    "GLOBAL_ATTEMPT_BUDGET_V4_SCHEMA", "GLOBAL_LEASE_OBSERVATION_V2_SCHEMA",
    "INSTALLED_RUNTIME_MANIFEST_V2_SCHEMA", "INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA",
    "LEASE_STATES", "MAX_FRAME_BYTES",
    "MAX_OUTPUT_BYTES", "MAX_TRANSFERRED_FDS", "OBSERVER_ACCOUNT", "OBSERVER_ARGV",
    "OBSERVER_CONTAINMENT_RECEIPT_SCHEMA", "OBSERVER_EXECUTABLE", "OBSERVER_LEAF_PATTERN",
    "OBSERVER_SUPERVISOR_CGROUP", "OBSERVER_SUPERVISOR_EXECUTABLE",
    "OBSERVER_SUPERVISOR_RUNTIME_DIRECTORY",
    "OBSERVER_SUPERVISOR_SERVICE", "OBSERVER_SUPERVISOR_SOCKET",
    "OBSERVATION_SESSION_V3_SCHEMA", "PRIVILEGED_RECEIPT_SCHEMA",
    "PRIVILEGED_REQUEST_SCHEMA", "PRIVILEGED_SERVICE_MANIFEST_SCHEMA",
    "PROCESS_MANIFEST_V2_SCHEMA", "RECOVERY_ACCOUNT", "ROS_GRAPH_RECEIPT_V3_SCHEMA",
    "RUNTIME_AUTHORITY_RECEIPT_V4_SCHEMA", "RUNTIME_AUTHORITY_REQUEST_V4_SCHEMA",
    "RUNTIME_AUTHORIZATION_V3_SCHEMA", "PeerCredentials", "PeerProcess", "PrivilegedRecord",
    "ReplayWindow",
    "Slice7GPrivilegedProtocolError", "authenticate_sealed_output", "canonical_bytes",
    "decode_packet", "encode_packet", "make_sealed_memfd", "observe_peer", "peer_credentials",
    "receive_packet", "reconcile_peer", "record_identity", "schema_names", "send_packet",
    "verify_response_binding",
    "validate_record",
]
