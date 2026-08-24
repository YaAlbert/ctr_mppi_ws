"""Closed local authority records and AF_UNIX framing for Slice 7G.

This module deliberately contains only Python standard-library imports.  It
does not create sockets, files, accounts, budgets, or processes on import.
Production locators are module-owned constants; the only alternate locators
are underscored factories intended for isolated ``/tmp`` tests.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import PurePosixPath
import re
import socket
import stat
import struct
from types import MappingProxyType
from typing import Any
import unicodedata

AUTHORITY_BOOTSTRAP_PATH = "/etc/ctr-mppi/slice-7g-authority/bootstrap.json"
AUTHORITY_SERVICE_PATH = "/usr/libexec/ctr-mppi/ctr-slice7g-authorityd"
AUTHORITY_STATE_ROOT = "/var/lib/ctr-mppi/slice-7g-authority"
AUTHORITY_SOCKET_PATH = "/run/ctr-mppi/slice-7g-authority/authority.sock"
INSTALLED_RUNTIME_PARENT = "/opt/ctr-mppi/slice-7g"
OUTPUT_PARENT = "/home/ankid/ctr_mppi_evidence/slice_7g"

AUTHORITY_ACCOUNT = "ctr7g-authority"
CAMPAIGN_ACCOUNT = "ctr7g-campaign"
RUNTIME_GROUP = "ctr7g-runtime"

AUTHORITY_BOOTSTRAP_SCHEMA = "ctr-slice-7g-authority-bootstrap-1"
INSTALLED_RUNTIME_MANIFEST_SCHEMA = "ctr-slice-7g-installed-runtime-manifest-1"
BUILD_TEST_APPROVAL_SCHEMA = "ctr-slice-7g-isolated-build-test-approval-1"
RUNTIME_AUTHORIZATION_SCHEMA = "ctr-slice-7g-runtime-authorization-2"
PROCESS_MANIFEST_SCHEMA = "ctr-slice-7g-process-manifest-1"
ENVIRONMENT_MANIFEST_SCHEMA = "ctr-slice-7g-environment-manifest-1"
LEGACY_GLOBAL_ATTEMPT_BUDGET_SCHEMA = "ctr-slice-7g-global-attempt-budget-1"
LEGACY_AUTHORITY_REQUEST_SCHEMA = "ctr-slice-7g-runtime-authority-request-1"
LEGACY_AUTHORITY_RECEIPT_SCHEMA = "ctr-slice-7g-runtime-authority-receipt-1"
LEGACY_OBSERVATION_SESSION_SCHEMA = "ctr-slice-7g-observation-session-1"
LEGACY_ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA = "ctr-slice-7g-ros-graph-observation-receipt-1"
LEGACY_FOUR_SOURCE_OBSERVATION_SCHEMA = "ctr-slice-7g-four-source-domain-observation-1"
LEGACY_GLOBAL_ATTEMPT_BUDGET_V2_SCHEMA = "ctr-slice-7g-global-attempt-budget-2"
LEGACY_AUTHORITY_REQUEST_V2_SCHEMA = "ctr-slice-7g-runtime-authority-request-2"
LEGACY_AUTHORITY_RECEIPT_V2_SCHEMA = "ctr-slice-7g-runtime-authority-receipt-2"
OBSERVATION_SESSION_SCHEMA = "ctr-slice-7g-observation-session-2"
ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA = "ctr-slice-7g-ros-graph-observation-receipt-2"
LEGACY_FOUR_SOURCE_OBSERVATION_V2_SCHEMA = "ctr-slice-7g-four-source-domain-observation-2"
FOUR_SOURCE_OBSERVATION_SCHEMA = "ctr-slice-7g-four-source-domain-observation-3"
GLOBAL_ATTEMPT_BUDGET_SCHEMA = "ctr-slice-7g-global-attempt-budget-3"
AUTHORITY_REQUEST_SCHEMA = "ctr-slice-7g-runtime-authority-request-3"
AUTHORITY_RECEIPT_SCHEMA = "ctr-slice-7g-runtime-authority-receipt-3"
AUTHORITY_REVOCATION_SCHEMA = "ctr-slice-7g-runtime-authority-revocation-1"
GLOBAL_LEASE_OBSERVATION_SCHEMA = "ctr-slice-7g-global-lease-observation-1"
OBSERVER_CLEANUP_GUARD_SCHEMA = "ctr-slice-7g-observer-cleanup-guard-1"
OBSERVER_CLEANUP_RECOVERY_SCHEMA = "ctr-slice-7g-observer-cleanup-recovery-authorization-1"

PRECOMMIT_ROS_GRAPH_OBSERVER_CLASS = "PRECOMMIT_ROS_GRAPH_OBSERVER"
PRECOMMIT_ROS_GRAPH_OBSERVER_EXECUTABLE = "/opt/ros/humble/bin/ros2"
PRECOMMIT_ROS_GRAPH_OBSERVER_ARGV = ("node", "list", "--no-daemon")
OBSERVATION_SESSION_LIFETIME_SECONDS = 1_800
PREPARE_TOKEN_LIFETIME_SECONDS = 300
MAX_PRECOMMIT_OBSERVERS = 100
MAX_POSTCOMMIT_OBSERVERS = 1
MAX_TRANSACTION_OBSERVERS = 101
OBSERVER_TIMEOUT_SECONDS = 10.0
OBSERVER_STDOUT_LIMIT_BYTES = 1_048_576
OBSERVER_STDERR_LIMIT_BYTES = 1_048_576
OBSERVER_CLEANUP_STABLE_SAMPLES = 2
OBSERVER_CLEANUP_MINIMUM_INTERVAL_SECONDS = 0.5
OBSERVER_CLEANUP_MAXIMUM_WAIT_SECONDS = 5.0

CAMPAIGN_SYSTEMD_UNIT = "ctr-slice7g-campaign.service"
AUTHORITY_SYSTEMD_UNIT = "ctr-slice7g-authority.service"
REVOCATION_PATH_UNIT = "ctr-slice7g-revocation.path"
REVOCATION_SERVICE_UNIT = "ctr-slice7g-revocation.service"

MAX_FRAME_BYTES = 1_048_576
MAX_STRING_BYTES = 8_192
MAX_LIST_ITEMS = 65_536
MAX_RECORD_DEPTH = 24
MAX_SESSION_REQUESTS = 108
FRAME_HEADER = struct.Struct("!I")
_DIGEST = re.compile(r"^[0-9a-f]{64}$")
_NAME = re.compile(r"^[a-z][a-z0-9_-]{0,127}$")

_BOOTSTRAP_SCHEMA_ROLES = {
    "authority_bootstrap": AUTHORITY_BOOTSTRAP_SCHEMA,
    "installed_runtime_manifest": INSTALLED_RUNTIME_MANIFEST_SCHEMA,
    "isolated_build_test_approval": BUILD_TEST_APPROVAL_SCHEMA,
    "runtime_authorization": RUNTIME_AUTHORIZATION_SCHEMA,
    "process_manifest": PROCESS_MANIFEST_SCHEMA,
    "environment_manifest": ENVIRONMENT_MANIFEST_SCHEMA,
    "global_attempt_budget": GLOBAL_ATTEMPT_BUDGET_SCHEMA,
    "runtime_authority_request": AUTHORITY_REQUEST_SCHEMA,
    "runtime_authority_receipt": AUTHORITY_RECEIPT_SCHEMA,
    "runtime_authority_revocation": AUTHORITY_REVOCATION_SCHEMA,
    "observation_session": OBSERVATION_SESSION_SCHEMA,
    "ros_graph_observation_receipt": ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA,
    "four_source_domain_observation": FOUR_SOURCE_OBSERVATION_SCHEMA,
}


class Slice7GAuthorityProtocolError(RuntimeError):
    """Stable public error for authority records and local framing."""

    def __init__(self, code: str, message: str, *, path: str = "$") -> None:
        self.code = code
        self.path = path
        super().__init__(f"{code}:{path}:{message}")


@dataclass(frozen=True)
class Slice7GAuthorityRecord:
    schema_version: str
    data: MappingProxyType
    canonical_bytes: bytes
    logical_identity: str


@dataclass(frozen=True)
class Slice7GPeerCredentials:
    pid: int
    uid: int
    gid: int

    def __post_init__(self) -> None:
        if any(type(value) is not int or value < 0 for value in (self.pid, self.uid, self.gid)):
            _fail("peer_credentials", "peer credentials must be nonnegative exact integers")
        if self.pid == 0:
            _fail("peer_credentials", "peer PID must be positive")


@dataclass(frozen=True)
class Slice7GPeerProcess:
    credentials: Slice7GPeerCredentials
    start_time_ticks: int
    executable: str
    argv: tuple[str, ...]
    environment: tuple[tuple[str, str], ...]
    working_directory: str
    cgroup: str


_SCHEMA_FIELDS: dict[str, frozenset[str]] = {
    AUTHORITY_BOOTSTRAP_SCHEMA: frozenset({
        "schema_version", "authority_uid", "authority_gid", "campaign_uid", "runtime_gid",
        "authority_account", "campaign_account", "runtime_group", "bootstrap_path",
        "service_executable_path", "state_root", "socket_path", "installed_runtime_parent",
        "service_executable", "record_paths", "schemas", "protocol_limits", "systemd_units",
    }),
    INSTALLED_RUNTIME_MANIFEST_SCHEMA: frozenset({
        "schema_version", "identity_algorithm", "installed_runtime_identity", "root_path",
        "root_device", "root_inode", "physical_tree_identity",
        "member_count", "members", "console_entrypoints", "python_modules",
        "generated_interfaces", "elf_members", "process_manifest_identity",
        "environment_manifest_identity", "source_snapshot", "build_test_approval_identity",
    }),
    BUILD_TEST_APPROVAL_SCHEMA: frozenset({
        "schema_version", "source_snapshot", "branch", "head", "tracked_diff_sha256",
        "applicable_test_nodes", "node_id_sha256", "git_command_manifest_sha256",
        "packages", "packages_built", "tests_passed", "origin_violations",
        "installed_runtime_proposal_identity", "issued_at_utc",
    }),
    RUNTIME_AUTHORIZATION_SCHEMA: frozenset({
        "schema_version", "authorization_nonce", "issued_at_utc", "not_before_utc",
        "not_after_utc", "branch", "head", "tracked_diff_sha256",
        "correction_manifest_sha256", "complete_subject_manifest_sha256", "source_snapshot",
        "charter", "build_test_approval_identity", "installed_runtime_identity",
        "process_manifest_identity", "environment_manifest_identity", "applicable_test_nodes",
        "node_id_sha256", "git_command_manifest_sha256", "entrypoint_identity", "campaign",
        "readiness_acceptance_identity", "evidence_schemas", "global_budget_identity",
        "output_parent_rule", "prepare_token_lifetime_seconds", "one_shot",
    }),
    PROCESS_MANIFEST_SCHEMA: frozenset({
        "schema_version", "identity_algorithm", "interpreter", "interpreter_flags",
        "entrypoint", "executables", "argv_template",
        "transaction_slots", "environment_manifest_identity", "working_directory",
        "shell", "systemd_unit", "cgroup", "allowed_descendants", "timeouts",
        "output_ownership", "required_receipts",
    }),
    ENVIRONMENT_MANIFEST_SCHEMA: frozenset({
        "schema_version", "identity_algorithm", "allowed_keys", "required_keys",
        "required_absent_keys", "fixed_values", "transaction_values", "path_keys",
        "path_order", "inherit_parent_environment",
    }),
    LEGACY_GLOBAL_ATTEMPT_BUDGET_SCHEMA: frozenset({
        "schema_version", "revision", "predecessor_identity", "state", "attempts_consumed",
        "attempts_maximum", "retries_authorized", "authorization_identity",
        "process_start_commitment", "updated_at_utc",
    }),
    LEGACY_AUTHORITY_REQUEST_SCHEMA: frozenset({
        "schema_version", "method", "request_id", "authorization_identity", "prepare_token",
        "campaign_id", "campaign_identity", "campaign_template_identity", "domain_id",
        "output_root_path", "output_root_identity",
        "process_manifest_identity", "process_instance_identity", "requested_at_utc",
    }),
    LEGACY_AUTHORITY_RECEIPT_SCHEMA: frozenset({
        "schema_version", "method", "request_id", "result", "authorization_identity",
        "service_instance_identity", "prepare_token", "previous_budget_revision",
        "budget_revision", "budget_identity", "campaign_id", "campaign_identity",
        "campaign_template_identity", "domain_id",
        "output_root_path", "output_root_identity", "process_manifest_identity", "process_instance_identity",
        "committed_at_utc", "error_code",
    }),
    LEGACY_OBSERVATION_SESSION_SCHEMA: frozenset({
        "schema_version", "authorization_identity", "installed_runtime_identity",
        "process_manifest_identity", "environment_manifest_identity", "connection_identity",
        "peer_uid", "peer_gid", "peer_pid", "peer_start_time_ticks", "campaign_cgroup",
        "service_nonce", "created_monotonic_ns", "deadline_monotonic_ns", "domain_minimum",
        "domain_maximum", "maximum_precommit_observers", "precommit_observer_count",
        "postcommit_observer_count", "transaction_observer_count", "candidate_domains",
        "precommit_receipt_identities", "selected_domain", "lease_identity",
        "four_source_observation_identity", "state",
    }),
    LEGACY_ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA: frozenset({
        "schema_version", "phase", "observer_class", "executable", "executable_identity",
        "interpreter", "interpreter_identity", "module_origin_identities", "argv",
        "environment_identity", "working_directory", "cgroup", "shell", "domain_id", "pid",
        "process_group_id", "process_start_time_ticks", "started_monotonic_ns", "ended_monotonic_ns",
        "exit_status", "terminating_signal", "stdout_size", "stdout_sha256", "stderr_size",
        "stderr_sha256", "nodes", "cleanup_barrier_identity", "unexpected_descendants",
        "ros_daemon_started",
    }),
    LEGACY_FOUR_SOURCE_OBSERVATION_SCHEMA: frozenset({
        "schema_version", "phase", "observation_session_identity", "domain_id",
        "active_process_identity", "dds_port_identity", "global_lease_identity",
        "ros_graph_observation_identity", "all_sources_clear", "observed_monotonic_ns",
    }),
    LEGACY_FOUR_SOURCE_OBSERVATION_V2_SCHEMA: frozenset({
        "schema_version", "session_binding_identity", "service_nonce", "phase",
        "phase_local_ordinal", "transaction_observer_ordinal", "domain_id",
        "peer_process_identity", "observation_interval_identity",
        "cleanup_disposition_identity", "active_process_identity", "dds_port_identity",
        "global_lease_identity", "ros_graph_provider_identity", "all_sources_clear",
        "observed_monotonic_ns",
    }),
    LEGACY_GLOBAL_ATTEMPT_BUDGET_V2_SCHEMA: frozenset({
        "schema_version", "revision", "predecessor_identity", "state", "attempts_consumed",
        "attempts_maximum", "retries_authorized", "authorization_identity",
        "process_start_commitment", "observation_session_identity",
        "four_source_observation_identity", "precommit_observer_count",
        "precommit_receipt_identities", "postcommit_observer_count",
        "postcommit_receipt_identity", "postcommit_four_source_observation_identity",
        "transaction_observer_count", "updated_at_utc",
    }),
    LEGACY_AUTHORITY_REQUEST_V2_SCHEMA: frozenset({
        "schema_version", "method", "request_id", "authorization_identity", "prepare_token",
        "campaign_id", "campaign_identity", "campaign_template_identity", "domain_id",
        "output_root_path", "output_root_identity", "process_manifest_identity",
        "process_instance_identity", "observation_session_identity", "observation_session_nonce",
        "ros_graph_observation_receipt", "four_source_observation", "four_source_observation_identity",
        "precommit_receipt_identities", "precommit_observer_count",
        "postcommit_observer_count", "transaction_observer_count", "lease_identity",
        "requested_at_utc",
    }),
    LEGACY_AUTHORITY_RECEIPT_V2_SCHEMA: frozenset({
        "schema_version", "method", "request_id", "result", "authorization_identity",
        "service_instance_identity", "prepare_token", "previous_budget_revision",
        "budget_revision", "budget_identity", "campaign_id", "campaign_identity",
        "campaign_template_identity", "domain_id", "output_root_path", "output_root_identity",
        "process_manifest_identity", "process_instance_identity", "observation_session_identity",
        "observation_session_nonce", "observation_session_deadline_monotonic_ns",
        "four_source_observation_identity", "precommit_receipt_identities",
        "precommit_observer_count", "postcommit_observer_count", "transaction_observer_count",
        "lease_identity", "prepare_expires_monotonic_ns", "committed_at_utc", "error_code",
    }),
    OBSERVATION_SESSION_SCHEMA: frozenset({
        "schema_version", "authorization_identity", "installed_runtime_identity",
        "process_manifest_identity", "environment_manifest_identity", "connection_identity",
        "peer_uid", "peer_gid", "peer_pid", "peer_start_time_ticks", "campaign_cgroup",
        "service_nonce", "daemon_generation_identity", "created_monotonic_ns",
        "deadline_monotonic_ns", "domain_minimum", "domain_maximum",
        "maximum_precommit_observers", "precommit_observer_count",
        "postcommit_observer_count", "transaction_observer_count", "candidate_domains",
        "precommit_receipt_identities", "selected_domain", "lease_identity",
        "four_source_observation_identity", "state",
    }),
    ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA: frozenset({
        "schema_version", "session_binding_identity", "service_nonce", "phase",
        "phase_local_ordinal", "transaction_observer_ordinal",
        "four_source_observation_identity", "observer_class", "executable",
        "executable_identity", "interpreter", "interpreter_identity",
        "module_origin_identities", "argv", "environment_identity", "working_directory",
        "cgroup", "shell", "domain_id", "pid", "process_group_id",
        "process_start_time_ticks", "started_monotonic_ns", "ended_monotonic_ns",
        "exit_status", "terminating_signal", "stdout_size", "stdout_sha256",
        "stderr_size", "stderr_sha256", "nodes", "parsed_node_set_identity",
        "cleanup_barrier_identity", "unexpected_descendants", "ros_daemon_started",
    }),
    FOUR_SOURCE_OBSERVATION_SCHEMA: frozenset({
        "schema_version", "session_binding_identity", "service_nonce", "phase",
        "phase_local_ordinal", "transaction_observer_ordinal", "domain_id",
        "peer_process_identity", "observation_interval_identity",
        "cleanup_disposition_identity", "active_process_identity", "dds_port_identity",
        "global_lease_identity", "global_lease_registry_identity",
        "global_lease_revision_identity", "global_lease_state", "global_lease_clear",
        "ros_graph_provider_identity", "all_sources_clear",
        "observed_monotonic_ns",
    }),
    GLOBAL_ATTEMPT_BUDGET_SCHEMA: frozenset({
        "schema_version", "revision", "predecessor_identity", "state", "attempts_consumed",
        "attempts_maximum", "retries_authorized", "authorization_identity",
        "process_start_commitment", "observation_session_identity",
        "four_source_observation_identity", "precommit_observer_count",
        "precommit_receipt_identities", "postcommit_observer_count",
        "postcommit_receipt_identity", "postcommit_four_source_observation_identity",
        "transaction_observer_count", "updated_at_utc",
    }),
    AUTHORITY_REQUEST_SCHEMA: frozenset({
        "schema_version", "method", "request_id", "authorization_identity", "prepare_token",
        "campaign_id", "campaign_identity", "campaign_template_identity", "domain_id",
        "output_root_path", "output_root_identity", "process_manifest_identity",
        "process_instance_identity", "observation_session_identity",
        "observation_session_nonce", "requested_at_utc",
    }),
    AUTHORITY_RECEIPT_SCHEMA: frozenset({
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
    }),
    AUTHORITY_REVOCATION_SCHEMA: frozenset({
        "schema_version", "revocation_id", "authorization_identity", "budget_revision",
        "state", "requested_at_utc", "requested_by_uid", "trigger_identity",
        "processed_trigger_identity", "termination_receipt_identity",
    }),
    GLOBAL_LEASE_OBSERVATION_SCHEMA: frozenset({
        "schema_version", "registry_identity", "registry_revision_identity", "domain_id",
        "state", "active_reservation_identities", "committed_binding_identities",
        "stale_invalid_identities", "clear", "observed_monotonic_ns",
    }),
    OBSERVER_CLEANUP_GUARD_SCHEMA: frozenset({
        "schema_version", "revision", "predecessor_identity", "state",
        "authorization_identity", "budget_identity", "service_generation_identity",
        "session_binding_identity", "phase", "phase_local_ordinal",
        "transaction_observer_ordinal", "domain_id", "executable_identity",
        "argv_identity", "environment_identity", "pid", "process_start_time_ticks",
        "process_group_id", "session_id", "cgroup", "pidfd_identity",
        "disposition_identity", "recovery_authorization_identity", "updated_at_utc",
    }),
    OBSERVER_CLEANUP_RECOVERY_SCHEMA: frozenset({
        "schema_version", "recovery_nonce", "quarantine_identity",
        "authority_root_identity", "runtime_authorization_identity", "budget_identity",
        "service_generation_identity", "issued_at_utc", "not_before_utc", "not_after_utc",
        "one_shot",
    }),
}


def authority_schema_names() -> tuple[str, ...]:
    return tuple(sorted(_SCHEMA_FIELDS))


def validate_authority_record(
    value: dict[str, Any] | bytes,
    *,
    expected_schema: str | None = None,
) -> Slice7GAuthorityRecord:
    """Validate, detach, freeze, and identify one closed authority record."""

    if type(value) is bytes:
        data = _parse_canonical_json(value)
        supplied = value
    elif type(value) is dict:
        data = _detach(value)
        supplied = None
    else:
        _fail("authority_record_type", "authority record must be exact bytes or dict")
    schema = _exact_string(data.get("schema_version"), "$.schema_version")
    if expected_schema is not None and schema != expected_schema:
        _fail("authority_schema", "authority record schema differs", path="$.schema_version")
    fields = _SCHEMA_FIELDS.get(schema)
    if fields is None:
        _fail("authority_schema", "unsupported authority schema", path="$.schema_version")
    _closed(data, fields, "$")
    _validate_schema_record(schema, data)
    canonical = _canonical(data)
    if supplied is not None and supplied != canonical:
        _fail("authority_noncanonical", "authority bytes are not canonical")
    identity = hashlib.sha256((f"{schema}-canonical\0").encode("ascii") + canonical).hexdigest()
    return Slice7GAuthorityRecord(schema, _freeze(data), canonical, identity)


def canonical_authority_record_bytes(value: dict[str, Any] | bytes, *, expected_schema: str) -> bytes:
    return validate_authority_record(value, expected_schema=expected_schema).canonical_bytes


def authority_record_identity(value: dict[str, Any] | bytes, *, expected_schema: str) -> str:
    return validate_authority_record(value, expected_schema=expected_schema).logical_identity


def encode_authority_frame(value: dict[str, Any] | bytes, *, expected_schema: str) -> bytes:
    payload = canonical_authority_record_bytes(value, expected_schema=expected_schema)
    if not payload or len(payload) > MAX_FRAME_BYTES:
        _fail("authority_frame_size", "authority frame size is outside the bound")
    return FRAME_HEADER.pack(len(payload)) + payload


def decode_authority_frame(frame: bytes, *, expected_schema: str) -> Slice7GAuthorityRecord:
    if type(frame) is not bytes:
        _fail("authority_frame_type", "authority frame must be exact bytes")
    if len(frame) < FRAME_HEADER.size:
        _fail("authority_frame_truncated", "authority frame header is truncated")
    (size,) = FRAME_HEADER.unpack(frame[: FRAME_HEADER.size])
    if size == 0 or size > MAX_FRAME_BYTES:
        _fail("authority_frame_size", "authority frame declares an invalid size")
    if len(frame) < FRAME_HEADER.size + size:
        _fail("authority_frame_truncated", "authority frame payload is truncated")
    if len(frame) != FRAME_HEADER.size + size:
        _fail("authority_frame_trailing", "authority frame has trailing bytes")
    return validate_authority_record(frame[FRAME_HEADER.size :], expected_schema=expected_schema)


def receive_authority_frame(channel: socket.socket, *, expected_schema: str) -> Slice7GAuthorityRecord:
    if type(channel) is not socket.socket or channel.family != socket.AF_UNIX:
        _fail("authority_socket_type", "authority channel must be an exact AF_UNIX socket")
    header = _recv_exact(channel, FRAME_HEADER.size)
    (size,) = FRAME_HEADER.unpack(header)
    if size == 0 or size > MAX_FRAME_BYTES:
        _fail("authority_frame_size", "authority frame declares an invalid size")
    payload = _recv_exact(channel, size)
    return validate_authority_record(payload, expected_schema=expected_schema)


def send_authority_frame(
    channel: socket.socket,
    value: dict[str, Any] | bytes,
    *,
    expected_schema: str,
) -> None:
    if type(channel) is not socket.socket or channel.family != socket.AF_UNIX:
        _fail("authority_socket_type", "authority channel must be an exact AF_UNIX socket")
    view = memoryview(encode_authority_frame(value, expected_schema=expected_schema))
    while view:
        try:
            written = channel.send(view)
        except OSError as exc:
            raise Slice7GAuthorityProtocolError("authority_socket_write", str(exc)) from exc
        if written <= 0:
            _fail("authority_socket_write", "authority socket made no write progress")
        view = view[written:]


def peer_credentials(channel: socket.socket) -> Slice7GPeerCredentials:
    if type(channel) is not socket.socket or channel.family != socket.AF_UNIX:
        _fail("authority_socket_type", "peer credential channel must be AF_UNIX")
    try:
        raw = channel.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i"))
        pid, uid, gid = struct.unpack("3i", raw)
    except (AttributeError, OSError, struct.error) as exc:
        raise Slice7GAuthorityProtocolError("peer_credentials_unavailable", str(exc)) from exc
    return Slice7GPeerCredentials(pid, uid, gid)


def observe_peer_process(credentials: Slice7GPeerCredentials) -> Slice7GPeerProcess:
    """Capture stable Linux process facts for an already authenticated peer PID."""

    if type(credentials) is not Slice7GPeerCredentials:
        _fail("peer_credentials", "peer credentials must be validated")
    root = f"/proc/{credentials.pid}"
    before = _proc_start_time(root)
    executable = _absolute_normalized(os.readlink(f"{root}/exe"), "$.peer.executable")
    argv = _nul_strings(_read_bounded(f"{root}/cmdline", MAX_FRAME_BYTES), "$.peer.argv")
    environment_items = _nul_strings(_read_bounded(f"{root}/environ", MAX_FRAME_BYTES), "$.peer.environment")
    environment: list[tuple[str, str]] = []
    for index, item in enumerate(environment_items):
        if "=" not in item:
            _fail("peer_environment", "environment member lacks '='", path=f"$.peer.environment[{index}]")
        key, value = item.split("=", 1)
        _environment_key(key, f"$.peer.environment[{index}].key")
        _plain_string(value, f"$.peer.environment[{index}].value", allow_empty=True)
        environment.append((key, value))
    if len({key for key, _ in environment}) != len(environment):
        _fail("peer_environment", "peer environment contains duplicate keys")
    cwd = _absolute_normalized(os.readlink(f"{root}/cwd"), "$.peer.cwd")
    cgroup_lines = _read_bounded(f"{root}/cgroup", 65_536).decode("utf-8", "strict").splitlines()
    if len(cgroup_lines) != 1 or not cgroup_lines[0].startswith("0::/"):
        _fail("peer_cgroup", "peer must have exactly one unified cgroup membership")
    cgroup = cgroup_lines[0][3:]
    _plain_string(cgroup, "$.peer.cgroup")
    after = _proc_start_time(root)
    if before != after:
        _fail("peer_replaced", "peer start time changed during observation")
    return Slice7GPeerProcess(
        credentials, before, executable, tuple(argv), tuple(sorted(environment)), cwd, cgroup,
    )


def reconcile_peer_process(expected: Slice7GPeerProcess) -> Slice7GPeerProcess:
    if type(expected) is not Slice7GPeerProcess:
        _fail("peer_process", "peer process observation must be exact")
    observed = observe_peer_process(expected.credentials)
    if observed != expected:
        _fail("peer_replaced", "peer process identity changed")
    return observed


class Slice7GAuthorityClient:
    """Fixed-path one-request/one-response production client."""

    __slots__ = ()

    @staticmethod
    def exchange(request: dict[str, Any]) -> Slice7GAuthorityRecord:
        if type(request) is not dict:
            _fail("authority_request_type", "authority request must be an exact dictionary")
        if request.get("method") in {"prepare", "cancel", "commit", "complete", "fail_after_commit"}:
            _fail("authority_session_required", "stateful campaign methods require a bound authority session")
        with Slice7GAuthoritySession() as session:
            return session.exchange(request)


class Slice7GAuthoritySession:
    """Bootstrap-authenticated bounded session used by prepare/commit transactions."""

    __slots__ = ("_channel", "_closed", "_requests")

    def __init__(self) -> None:
        bootstrap = load_production_bootstrap()
        channel = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            channel.connect(AUTHORITY_SOCKET_PATH)
            observed = peer_credentials(channel)
            if observed.uid != bootstrap.data["authority_uid"] or observed.gid != bootstrap.data["authority_gid"]:
                _fail("authority_server_peer", "authority server numeric credentials differ")
            self._channel = channel
            self._closed = False
            self._requests = 0
        except Slice7GAuthorityProtocolError:
            channel.close()
            raise
        except OSError as exc:
            channel.close()
            raise Slice7GAuthorityProtocolError("authority_connection", str(exc)) from exc

    def __enter__(self) -> "Slice7GAuthoritySession":
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()

    def exchange(self, request: dict[str, Any]) -> Slice7GAuthorityRecord:
        if self._closed:
            _fail("authority_session_closed", "authority session is closed")
        if self._requests >= MAX_SESSION_REQUESTS:
            _fail("authority_session_bound", "authority session request bound exceeded")
        if type(request) is not dict:
            _fail("authority_request_type", "authority request must be an exact dictionary")
        validated = validate_authority_record(request, expected_schema=AUTHORITY_REQUEST_SCHEMA)
        send_authority_frame(self._channel, validated.canonical_bytes, expected_schema=AUTHORITY_REQUEST_SCHEMA)
        response = receive_authority_frame(self._channel, expected_schema=AUTHORITY_RECEIPT_SCHEMA)
        self._requests += 1
        return response

    def close(self) -> None:
        if getattr(self, "_closed", True):
            return
        self._closed = True
        try:
            self._channel.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._channel.close()


def load_production_bootstrap() -> Slice7GAuthorityRecord:
    """Authenticate the fixed root-owned bootstrap without following links."""

    _authenticate_root_owned_parent_chain(AUTHORITY_BOOTSTRAP_PATH)
    _authenticate_root_owned_parent_chain(AUTHORITY_SERVICE_PATH)
    raw, observed = _read_fixed_file(
        AUTHORITY_BOOTSTRAP_PATH,
        maximum=MAX_FRAME_BYTES,
        owner_uid=0,
        owner_gid=0,
        mode=0o444,
    )
    record = validate_authority_record(raw, expected_schema=AUTHORITY_BOOTSTRAP_SCHEMA)
    identity = record.data["service_executable"]
    if identity["path"] != AUTHORITY_SERVICE_PATH:
        _fail("bootstrap_service", "service executable path differs")
    authenticate_file_identity(
        dict(identity), expected_mode=0o555,
        expected_owner_uid=0, expected_owner_gid=0,
    )
    after = os.stat(AUTHORITY_BOOTSTRAP_PATH, follow_symlinks=False)
    if _stat_tuple(after) != observed:
        _fail("bootstrap_replaced", "bootstrap identity changed after authentication")
    return record


def _authenticate_root_owned_parent_chain(path: str) -> None:
    normalized = _absolute_normalized(path, "$.fixed_path")
    descriptor = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        for part in PurePosixPath(normalized).parts[1:-1]:
            child = os.open(
                part, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW,
                dir_fd=descriptor,
            )
            info = os.fstat(child)
            if info.st_uid != 0 or stat.S_IMODE(info.st_mode) & 0o022:
                os.close(child)
                _fail("fixed_parent_authority", "fixed authority parent is caller/service writable")
            os.close(descriptor)
            descriptor = child
    except OSError as exc:
        raise Slice7GAuthorityProtocolError("fixed_parent_read", str(exc)) from exc
    finally:
        os.close(descriptor)


def authenticate_file_identity(
    identity: dict[str, Any],
    *,
    expected_mode: int | None = None,
    expected_owner_uid: int | None = None,
    expected_owner_gid: int | None = None,
) -> None:
    """Reconcile a closed physical file identity with a no-follow read."""

    item = _validate_file_identity(identity, "$.file_identity")
    path = item["path"]
    raw, observed = _read_fixed_file(
        path,
        maximum=max(item["size"], 1),
        owner_uid=expected_owner_uid,
        owner_gid=expected_owner_gid,
        mode=expected_mode,
    )
    if observed != (
        item["device"], item["inode"], item["mode"], item["link_count"], item["size"],
        item["owner_uid"], item["owner_gid"],
    ):
        _fail("file_identity", "physical file metadata differs", path=path)
    if hashlib.sha256(raw).hexdigest() != item["sha256"]:
        _fail("file_identity", "physical file digest differs", path=path)


def _validate_schema_record(schema: str, data: dict[str, Any]) -> None:
    if schema == AUTHORITY_BOOTSTRAP_SCHEMA:
        _validate_bootstrap(data)
    elif schema == INSTALLED_RUNTIME_MANIFEST_SCHEMA:
        _validate_installed_manifest(data)
    elif schema == BUILD_TEST_APPROVAL_SCHEMA:
        _validate_build_approval(data)
    elif schema == RUNTIME_AUTHORIZATION_SCHEMA:
        _validate_runtime_authorization(data)
    elif schema == PROCESS_MANIFEST_SCHEMA:
        _validate_process_manifest(data)
    elif schema == ENVIRONMENT_MANIFEST_SCHEMA:
        _validate_environment_manifest(data)
    elif schema == LEGACY_GLOBAL_ATTEMPT_BUDGET_SCHEMA:
        _validate_legacy_budget(data)
    elif schema == LEGACY_AUTHORITY_REQUEST_SCHEMA:
        _validate_legacy_request(data)
    elif schema == LEGACY_AUTHORITY_RECEIPT_SCHEMA:
        _validate_legacy_receipt(data)
    elif schema in {LEGACY_OBSERVATION_SESSION_SCHEMA, OBSERVATION_SESSION_SCHEMA}:
        _validate_observation_session(data)
    elif schema in {
        LEGACY_ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA,
        ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA,
    }:
        _validate_ros_graph_observation_receipt(data)
    elif schema in {
        LEGACY_FOUR_SOURCE_OBSERVATION_SCHEMA,
        LEGACY_FOUR_SOURCE_OBSERVATION_V2_SCHEMA,
        FOUR_SOURCE_OBSERVATION_SCHEMA,
    }:
        _validate_four_source_observation(data)
    elif schema in {LEGACY_GLOBAL_ATTEMPT_BUDGET_V2_SCHEMA, GLOBAL_ATTEMPT_BUDGET_SCHEMA}:
        _validate_budget(data)
    elif schema == LEGACY_AUTHORITY_REQUEST_V2_SCHEMA:
        _validate_request_v2(data)
    elif schema == LEGACY_AUTHORITY_RECEIPT_V2_SCHEMA:
        _validate_receipt_v2(data)
    elif schema == AUTHORITY_REQUEST_SCHEMA:
        _validate_request(data)
    elif schema == AUTHORITY_RECEIPT_SCHEMA:
        _validate_receipt(data)
    elif schema == AUTHORITY_REVOCATION_SCHEMA:
        _validate_revocation(data)
    elif schema == GLOBAL_LEASE_OBSERVATION_SCHEMA:
        _validate_global_lease_observation(data)
    elif schema == OBSERVER_CLEANUP_GUARD_SCHEMA:
        _validate_cleanup_guard(data)
    elif schema == OBSERVER_CLEANUP_RECOVERY_SCHEMA:
        _validate_cleanup_recovery(data)


def _validate_bootstrap(data: dict[str, Any]) -> None:
    for field in ("authority_uid", "authority_gid", "campaign_uid", "runtime_gid"):
        _nonnegative_int(data[field], f"$.{field}")
    _exact(data["authority_account"], AUTHORITY_ACCOUNT, "$.authority_account")
    _exact(data["campaign_account"], CAMPAIGN_ACCOUNT, "$.campaign_account")
    _exact(data["runtime_group"], RUNTIME_GROUP, "$.runtime_group")
    for field, expected in (
        ("bootstrap_path", AUTHORITY_BOOTSTRAP_PATH),
        ("service_executable_path", AUTHORITY_SERVICE_PATH),
        ("state_root", AUTHORITY_STATE_ROOT),
        ("socket_path", AUTHORITY_SOCKET_PATH),
        ("installed_runtime_parent", INSTALLED_RUNTIME_PARENT),
    ):
        _exact(data[field], expected, f"$.{field}")
    _validate_file_identity(data["service_executable"], "$.service_executable")
    record_paths = _exact_dict(data["record_paths"], "$.record_paths")
    _closed(record_paths, {
        "installed_runtime_manifest", "build_test_approval", "runtime_authorization",
        "process_manifest", "environment_manifest", "budget_root", "revocation_root",
    }, "$.record_paths")
    for key, value in record_paths.items():
        path = _absolute_normalized(value, f"$.record_paths.{key}")
        if not _strict_descendant(path, AUTHORITY_STATE_ROOT):
            _fail("authority_path", "authority record path escapes state root", path=f"$.record_paths.{key}")
    schemas = _exact_dict(data["schemas"], "$.schemas")
    if schemas != _BOOTSTRAP_SCHEMA_ROLES:
        _fail("bootstrap_schemas", "bootstrap schema inventory differs", path="$.schemas")
    limits = _exact_dict(data["protocol_limits"], "$.protocol_limits")
    _closed(limits, {"maximum_frame_bytes", "maximum_string_bytes", "maximum_list_items", "maximum_record_depth", "maximum_session_requests"}, "$.protocol_limits")
    observed_limits = (
        limits["maximum_frame_bytes"],
        limits["maximum_string_bytes"],
        limits["maximum_list_items"],
        limits["maximum_record_depth"],
        limits["maximum_session_requests"],
    )
    expected_limits = (MAX_FRAME_BYTES, MAX_STRING_BYTES, MAX_LIST_ITEMS, MAX_RECORD_DEPTH, MAX_SESSION_REQUESTS)
    if observed_limits != expected_limits or any(type(value) is not int for value in observed_limits):
        _fail("protocol_limits", "bootstrap protocol limits differ")
    units = _exact_dict(data["systemd_units"], "$.systemd_units")
    _closed(units, {"authority", "campaign", "revocation_path", "revocation_service"}, "$.systemd_units")
    if units != {"authority": AUTHORITY_SYSTEMD_UNIT, "campaign": CAMPAIGN_SYSTEMD_UNIT, "revocation_path": REVOCATION_PATH_UNIT, "revocation_service": REVOCATION_SERVICE_UNIT}:
        _fail("systemd_units", "bootstrap systemd unit names differ")


def _validate_installed_manifest(data: dict[str, Any]) -> None:
    _exact(data["identity_algorithm"], "sha256:ctr-slice-7g-installed-runtime-tree-canonical-1", "$.identity_algorithm")
    identity = _digest_value(data["installed_runtime_identity"], "$.installed_runtime_identity")
    _exact(data["root_path"], f"{INSTALLED_RUNTIME_PARENT}/{identity}", "$.root_path")
    _nonnegative_int(data["root_device"], "$.root_device")
    _positive_int(data["root_inode"], "$.root_inode")
    _digest_value(data["physical_tree_identity"], "$.physical_tree_identity")
    members = _record_list(data["members"], "$.members", _validate_installed_member)
    _positive_int(data["member_count"], "$.member_count")
    if data["member_count"] != len(members):
        _fail("installed_member_count", "installed member count differs")
    paths = [item["path"] for item in members]
    if paths != sorted(paths) or len(paths) != len(set(paths)):
        _fail("installed_member_order", "installed members must have unique sorted paths")
    _ordered_unique_records(data["console_entrypoints"], "$.console_entrypoints", _validate_console_entrypoint, "name")
    _ordered_unique_records(data["python_modules"], "$.python_modules", _validate_python_module, "module")
    _ordered_unique_records(data["generated_interfaces"], "$.generated_interfaces", _validate_generated_interface, "name")
    _ordered_unique_records(data["elf_members"], "$.elf_members", _validate_elf_member, "path")
    for field in ("process_manifest_identity", "environment_manifest_identity", "build_test_approval_identity"):
        _digest_value(data[field], f"$.{field}")
    _validate_source_snapshot(data["source_snapshot"], "$.source_snapshot")


def _validate_build_approval(data: dict[str, Any]) -> None:
    _validate_source_snapshot(data["source_snapshot"], "$.source_snapshot")
    _plain_string(data["branch"], "$.branch")
    _hex(data["head"], 40, "$.head")
    for field in ("tracked_diff_sha256", "node_id_sha256", "git_command_manifest_sha256", "installed_runtime_proposal_identity"):
        _digest_value(data[field], f"$.{field}")
    _positive_int(data["applicable_test_nodes"], "$.applicable_test_nodes")
    packages = _string_list(data["packages"], "$.packages")
    if packages != sorted(packages) or len(packages) != len(set(packages)):
        _fail("build_packages", "build packages must be unique and sorted")
    _positive_int(data["packages_built"], "$.packages_built")
    _positive_int(data["tests_passed"], "$.tests_passed")
    _nonnegative_int(data["origin_violations"], "$.origin_violations")
    if data["packages_built"] != len(packages) or data["origin_violations"] != 0:
        _fail("build_approval", "build approval does not represent a complete isolated result")
    _utc(data["issued_at_utc"], "$.issued_at_utc")


def _validate_runtime_authorization(data: dict[str, Any]) -> None:
    _bounded_identifier(data["authorization_nonce"], "$.authorization_nonce")
    issued = _utc(data["issued_at_utc"], "$.issued_at_utc")
    before = _utc(data["not_before_utc"], "$.not_before_utc")
    after = _utc(data["not_after_utc"], "$.not_after_utc")
    if not before <= issued < after:
        _fail("authorization_validity", "authorization validity interval is inconsistent")
    _plain_string(data["branch"], "$.branch")
    _hex(data["head"], 40, "$.head")
    for field in (
        "tracked_diff_sha256", "correction_manifest_sha256", "complete_subject_manifest_sha256",
        "build_test_approval_identity", "installed_runtime_identity", "process_manifest_identity",
        "environment_manifest_identity", "node_id_sha256", "git_command_manifest_sha256",
        "entrypoint_identity", "readiness_acceptance_identity", "global_budget_identity",
    ):
        _digest_value(data[field], f"$.{field}")
    _validate_source_snapshot(data["source_snapshot"], "$.source_snapshot")
    _validate_charter_binding(data["charter"], "$.charter")
    _positive_int(data["applicable_test_nodes"], "$.applicable_test_nodes")
    _validate_campaign(data["campaign"], "$.campaign")
    evidence = _exact_dict(data["evidence_schemas"], "$.evidence_schemas")
    if not evidence:
        _fail("evidence_schemas", "evidence schema bindings must not be empty")
    for key, value in evidence.items():
        _bounded_identifier(key, f"$.evidence_schemas.{key}")
        _plain_string(value, f"$.evidence_schemas.{key}")
    _validate_output_parent_rule(data["output_parent_rule"], "$.output_parent_rule")
    lifetime = _positive_int(data["prepare_token_lifetime_seconds"], "$.prepare_token_lifetime_seconds")
    if lifetime != PREPARE_TOKEN_LIFETIME_SECONDS:
        _fail("prepare_token_lifetime", "prepare token lifetime must be exactly 300 seconds")
    _exact_bool(data["one_shot"], True, "$.one_shot")


def _validate_process_manifest(data: dict[str, Any]) -> None:
    _exact(data["identity_algorithm"], "sha256:ctr-slice-7g-process-manifest-canonical-1", "$.identity_algorithm")
    interpreter = _validate_file_identity(data["interpreter"], "$.interpreter")
    entrypoint = _validate_file_identity(data["entrypoint"], "$.entrypoint")
    executables = _record_list(data["executables"], "$.executables", _validate_file_identity)
    if not executables:
        _fail("process_executables", "process executable inventory must not be empty")
    all_files = [interpreter, entrypoint, *executables]
    identities = {(item["device"], item["inode"]) for item in all_files}
    paths = [item["path"] for item in all_files]
    if len(identities) != len(all_files) or len(set(paths)) != len(paths):
        _fail("process_executable_alias", "process executable inventory contains an inode alias")
    for index, item in enumerate(all_files):
        if item["mode"] & 0o222:
            _fail("process_executable_writable", "process executable is writable", path=f"$.executables[{index}]")
        if (
            not item["path"].startswith(INSTALLED_RUNTIME_PARENT + "/")
            and not item["path"].startswith("/opt/ros/humble/")
            and item["path"] not in {"/usr/bin/python3", "/usr/bin/python3.10", "/usr/bin/systemctl"}
        ):
            _fail("process_executable_origin", "process executable origin is not authorized", path=f"$.executables[{index}]")
    flags = _string_list(data["interpreter_flags"], "$.interpreter_flags")
    if flags != ["-I"]:
        _fail("process_interpreter_flags", "Python isolated mode is required")
    argv = _string_list(data["argv_template"], "$.argv_template")
    prefix = [interpreter["path"], *flags, entrypoint["path"]]
    if len(argv) < len(prefix) or argv[:len(prefix)] != prefix:
        _fail("process_argv", "argv must begin with the authenticated interpreter and entrypoint")
    slots = _exact_dict(data["transaction_slots"], "$.transaction_slots")
    for key, value in slots.items():
        _bounded_identifier(key, f"$.transaction_slots.{key}")
        if value not in {
            "campaign_id", "cell_id", "domain_id", "output_root", "campaign_output_root",
            "cell_output_root", "authorization_identity", "receipt_identity", "charter_identity",
            "ledger_identity", "ledger_revision", "process_start_identity", "plan_identity",
            "lease_identity", "domain_binding_identity", "working_directory",
        }:
            _fail("process_slot", "unsupported process transaction slot", path=f"$.transaction_slots.{key}")
    _digest_value(data["environment_manifest_identity"], "$.environment_manifest_identity")
    _absolute_normalized(data["working_directory"], "$.working_directory")
    _exact_bool(data["shell"], False, "$.shell")
    _exact(data["systemd_unit"], CAMPAIGN_SYSTEMD_UNIT, "$.systemd_unit")
    _plain_string(data["cgroup"], "$.cgroup")
    descendants = _record_list(data["allowed_descendants"], "$.allowed_descendants", _validate_descendant)
    roles = [item["role"] for item in descendants]
    if roles != sorted(roles) or len(roles) != len(set(roles)):
        _fail("process_descendants", "descendant roles must be unique and sorted")
    timeouts = _exact_dict(data["timeouts"], "$.timeouts")
    _closed(timeouts, {"sigint_seconds", "sigterm_seconds", "sigkill_seconds", "cell_seconds"}, "$.timeouts")
    for key, value in timeouts.items():
        _positive_number(value, f"$.timeouts.{key}")
    _validate_output_ownership(data["output_ownership"], "$.output_ownership")
    receipts = _string_list(data["required_receipts"], "$.required_receipts")
    if receipts != sorted(receipts) or len(receipts) != len(set(receipts)):
        _fail("process_receipts", "required receipts must be unique and sorted")


def _validate_environment_manifest(data: dict[str, Any]) -> None:
    _exact(data["identity_algorithm"], "sha256:ctr-slice-7g-environment-manifest-canonical-1", "$.identity_algorithm")
    allowed = _string_list(data["allowed_keys"], "$.allowed_keys")
    required = _string_list(data["required_keys"], "$.required_keys")
    absent = _string_list(data["required_absent_keys"], "$.required_absent_keys")
    for keys, path in ((allowed, "allowed_keys"), (required, "required_keys"), (absent, "required_absent_keys")):
        if keys != sorted(keys) or len(keys) != len(set(keys)):
            _fail("environment_keys", f"{path} must be unique and sorted")
        for key in keys:
            _environment_key(key, f"$.{path}")
    if not set(required) <= set(allowed) or set(allowed) & set(absent):
        _fail("environment_keys", "environment key sets are inconsistent")
    governed = {
        "PATH", "PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "LD_LIBRARY_PATH",
        "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_HOME", "ROS_LOG_DIR",
        "ROS_LOCALHOST_ONLY", "HOME", "XDG_CACHE_HOME",
        "MPLCONFIGDIR", "PYTHONDONTWRITEBYTECODE", "PYTHONNOUSERSITE",
    }
    if not governed <= set(allowed) | set(absent):
        _fail("environment_governance", "minimum environment key set is not governed")
    fixed = _exact_dict(data["fixed_values"], "$.fixed_values")
    generated = _exact_dict(data["transaction_values"], "$.transaction_values")
    if set(fixed) | set(generated) != set(required) or set(fixed) & set(generated):
        _fail("environment_values", "required environment keys must have one source")
    for key, value in fixed.items():
        _environment_key(key, f"$.fixed_values.{key}")
        _plain_string(value, f"$.fixed_values.{key}", allow_empty=True)
    for key, value in generated.items():
        _environment_key(key, f"$.transaction_values.{key}")
        if value not in {
            "domain_id", "campaign_id", "cell_id", "output_root", "campaign_output_root",
            "cell_output_root", "authorization_identity", "receipt_identity", "charter_identity",
            "ledger_identity", "ledger_revision", "process_start_identity", "plan_identity",
            "lease_identity", "domain_binding_identity", "working_directory",
        }:
            _fail("environment_slot", "unsupported transaction environment slot")
    path_keys = _string_list(data["path_keys"], "$.path_keys")
    if not set(path_keys) <= set(required):
        _fail("environment_paths", "path keys must be required")
    order = _string_list(data["path_order"], "$.path_order")
    if order != path_keys:
        _fail("environment_paths", "path order must exactly match path keys")
    _exact_bool(data["inherit_parent_environment"], False, "$.inherit_parent_environment")


def _validate_legacy_budget(data: dict[str, Any]) -> None:
    revision = _nonnegative_int(data["revision"], "$.revision")
    predecessor = data["predecessor_identity"]
    if revision == 0:
        if predecessor is not None:
            _fail("budget_predecessor", "revision zero cannot have a predecessor")
    else:
        _digest_value(predecessor, "$.predecessor_identity")
    if data["state"] not in {"UNCONSUMED", "COMMITTED", "COMPLETED", "FAILED_AFTER_COMMIT"}:
        _fail("budget_state", "unsupported global budget state")
    if revision == 0 and data["state"] != "UNCONSUMED":
        _fail("budget_state", "externally provisioned revision zero must be UNCONSUMED")
    if revision > 0 and data["state"] == "UNCONSUMED":
        _fail("budget_state", "UNCONSUMED cannot be recreated after revision zero")
    _nonnegative_int(data["attempts_consumed"], "$.attempts_consumed")
    _exact(data["attempts_maximum"], 1, "$.attempts_maximum")
    _exact(data["retries_authorized"], 0, "$.retries_authorized")
    if data["state"] == "UNCONSUMED":
        if data["attempts_consumed"] != 0 or data["authorization_identity"] is not None or data["process_start_commitment"] is not None:
            _fail("budget_state", "unconsumed state contains consuming bindings")
    else:
        if data["attempts_consumed"] != 1:
            _fail("budget_state", "post-commit state must consume one attempt")
        _digest_value(data["authorization_identity"], "$.authorization_identity")
        _validate_named_record(data["process_start_commitment"], "$.process_start_commitment")
    _utc(data["updated_at_utc"], "$.updated_at_utc")


def _validate_legacy_request(data: dict[str, Any]) -> None:
    if data["method"] not in {"prepare", "allocate_provisional", "cancel", "commit", "complete", "fail_after_commit", "status", "revoke"}:
        _fail("authority_method", "unsupported authority request method")
    _bounded_identifier(data["request_id"], "$.request_id")
    for field in ("authorization_identity", "campaign_identity", "campaign_template_identity", "output_root_identity", "process_manifest_identity", "process_instance_identity"):
        if data[field] is not None:
            _digest_value(data[field], f"$.{field}")
    if data["prepare_token"] is not None:
        _bounded_identifier(data["prepare_token"], "$.prepare_token")
    if data["campaign_id"] is not None:
        _bounded_identifier(data["campaign_id"], "$.campaign_id")
    if data["campaign_template_identity"] is not None:
        _digest_value(data["campaign_template_identity"], "$.campaign_template_identity")
    if data["domain_id"] is not None and (type(data["domain_id"]) is not int or not 100 <= data["domain_id"] <= 199):
        _fail("domain_id", "domain ID must be 100 through 199")
    if data["output_root_path"] is not None:
        output = _absolute_normalized(data["output_root_path"], "$.output_root_path")
        if not _strict_descendant(output, OUTPUT_PARENT):
            _fail("output_root_path", "output root escapes fixed parent")
    _utc(data["requested_at_utc"], "$.requested_at_utc")


def _validate_legacy_receipt(data: dict[str, Any]) -> None:
    _plain_string(data["method"], "$.method")
    _bounded_identifier(data["request_id"], "$.request_id")
    if data["result"] not in {"PREPARED", "CANCELLED", "COMMITTED", "COMPLETED", "FAILED_AFTER_COMMIT", "REVOKED", "STATUS", "ERROR"}:
        _fail("authority_result", "unsupported authority result")
    for field in ("authorization_identity", "service_instance_identity", "budget_identity", "campaign_identity", "campaign_template_identity", "output_root_identity", "process_manifest_identity", "process_instance_identity"):
        if data[field] is not None:
            _digest_value(data[field], f"$.{field}")
    if data["prepare_token"] is not None:
        _bounded_identifier(data["prepare_token"], "$.prepare_token")
    if data["campaign_id"] is not None:
        _bounded_identifier(data["campaign_id"], "$.campaign_id")
    for field in ("previous_budget_revision", "budget_revision"):
        if data[field] is not None:
            _nonnegative_int(data[field], f"$.{field}")
    if data["domain_id"] is not None and (type(data["domain_id"]) is not int or not 100 <= data["domain_id"] <= 199):
        _fail("domain_id", "domain ID must be 100 through 199")
    if data["output_root_path"] is not None:
        output = _absolute_normalized(data["output_root_path"], "$.output_root_path")
        if not _strict_descendant(output, OUTPUT_PARENT):
            _fail("output_root_path", "output root escapes fixed parent")
    if data["committed_at_utc"] is not None:
        _utc(data["committed_at_utc"], "$.committed_at_utc")
    if data["error_code"] is not None:
        _bounded_identifier(data["error_code"], "$.error_code")


def _validate_observation_session(data: dict[str, Any]) -> None:
    for field in (
        "authorization_identity", "installed_runtime_identity", "process_manifest_identity",
        "environment_manifest_identity", "four_source_observation_identity",
    ):
        if data[field] is not None:
            _digest_value(data[field], f"$.{field}")
    for field in ("connection_identity", "service_nonce"):
        _bounded_identifier(data[field], f"$.{field}")
    if data["schema_version"] == OBSERVATION_SESSION_SCHEMA:
        _digest_value(data["daemon_generation_identity"], "$.daemon_generation_identity")
    for field in ("peer_uid", "peer_gid", "peer_pid", "peer_start_time_ticks"):
        _nonnegative_int(data[field], f"$.{field}")
    if data["peer_pid"] == 0 or data["peer_start_time_ticks"] == 0:
        _fail("observation_peer", "observation peer PID and start time must be positive")
    _plain_string(data["campaign_cgroup"], "$.campaign_cgroup")
    created = _nonnegative_int(data["created_monotonic_ns"], "$.created_monotonic_ns")
    deadline = _positive_int(data["deadline_monotonic_ns"], "$.deadline_monotonic_ns")
    if deadline - created != OBSERVATION_SESSION_LIFETIME_SECONDS * 1_000_000_000:
        _fail("observation_session_lifetime", "observation-session lifetime must be exactly 1800 seconds")
    _exact(data["domain_minimum"], 100, "$.domain_minimum")
    _exact(data["domain_maximum"], 199, "$.domain_maximum")
    _exact(data["maximum_precommit_observers"], MAX_PRECOMMIT_OBSERVERS, "$.maximum_precommit_observers")
    precommit = _bounded_counter(data["precommit_observer_count"], MAX_PRECOMMIT_OBSERVERS, "$.precommit_observer_count")
    postcommit = _bounded_counter(data["postcommit_observer_count"], MAX_POSTCOMMIT_OBSERVERS, "$.postcommit_observer_count")
    total = _bounded_counter(data["transaction_observer_count"], MAX_TRANSACTION_OBSERVERS, "$.transaction_observer_count")
    if total != precommit + postcommit:
        _fail("observer_counter", "transaction observer count is inconsistent")
    candidates = _domain_list(data["candidate_domains"], "$.candidate_domains")
    receipts = _digest_list(data["precommit_receipt_identities"], "$.precommit_receipt_identities")
    if len(candidates) != precommit or len(receipts) != precommit:
        _fail("observer_counter", "precommit candidates, receipts, and counter differ")
    if postcommit != 0:
        _fail("observer_counter", "observation session cannot contain a postcommit observer")
    if data["selected_domain"] is not None:
        selected = _domain_id(data["selected_domain"], "$.selected_domain")
        if selected not in candidates:
            _fail("observation_selected_domain", "selected domain was not observed")
    if data["lease_identity"] is not None:
        _digest_value(data["lease_identity"], "$.lease_identity")
    if data["state"] not in {"OPEN", "OBSERVED", "PREPARED", "INVALIDATED"}:
        _fail("observation_session_state", "observation-session state differs")
    if data["state"] == "OPEN":
        if any(data[field] is not None for field in (
            "selected_domain", "lease_identity", "four_source_observation_identity",
        )):
            _fail("observation_session_state", "open session contains finalized authority")
    elif data["state"] in {"OBSERVED", "PREPARED"}:
        if precommit < 1 or any(data[field] is None for field in (
            "selected_domain", "lease_identity", "four_source_observation_identity",
        )):
            _fail("observation_session_state", "finalized session lacks observation authority")


def _validate_ros_graph_observation_receipt(data: dict[str, Any]) -> None:
    current = data["schema_version"] == ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA
    if current:
        _digest_value(data["session_binding_identity"], "$.session_binding_identity")
        _bounded_identifier(data["service_nonce"], "$.service_nonce")
        phase_ordinal = _positive_int(data["phase_local_ordinal"], "$.phase_local_ordinal")
        transaction_ordinal = _positive_int(
            data["transaction_observer_ordinal"], "$.transaction_observer_ordinal",
        )
        if data["phase"] == "PRECOMMIT" and phase_ordinal != transaction_ordinal:
            _fail("observer_ordinal", "precommit observer ordinals differ")
        if data["phase"] == "POSTCOMMIT" and phase_ordinal != 1:
            _fail("observer_ordinal", "postcommit phase ordinal must equal one")
        _digest_value(
            data["four_source_observation_identity"],
            "$.four_source_observation_identity",
        )
    if data["phase"] not in {"PRECOMMIT", "POSTCOMMIT"}:
        _fail("observer_phase", "observer phase differs")
    _exact(data["observer_class"], PRECOMMIT_ROS_GRAPH_OBSERVER_CLASS, "$.observer_class")
    _exact(data["executable"], PRECOMMIT_ROS_GRAPH_OBSERVER_EXECUTABLE, "$.executable")
    _digest_value(data["executable_identity"], "$.executable_identity")
    _exact(data["interpreter"], "/usr/bin/python3", "$.interpreter")
    _digest_value(data["interpreter_identity"], "$.interpreter_identity")
    origins = _digest_list(data["module_origin_identities"], "$.module_origin_identities")
    if not origins or origins != sorted(origins) or len(origins) != len(set(origins)):
        _fail("observer_module_origins", "observer module-origin identities must be unique and sorted")
    argv = _string_list(data["argv"], "$.argv")
    expected_argv = [PRECOMMIT_ROS_GRAPH_OBSERVER_EXECUTABLE, *PRECOMMIT_ROS_GRAPH_OBSERVER_ARGV]
    if argv != expected_argv:
        _fail("observer_argv", "ROS graph observer argv differs")
    _digest_value(data["environment_identity"], "$.environment_identity")
    _absolute_normalized(data["working_directory"], "$.working_directory")
    _plain_string(data["cgroup"], "$.cgroup")
    _exact_bool(data["shell"], False, "$.shell")
    _domain_id(data["domain_id"], "$.domain_id")
    pid = _positive_int(data["pid"], "$.pid")
    process_group = _positive_int(data["process_group_id"], "$.process_group_id")
    if process_group != pid:
        _fail("observer_process_group", "observer must own its authenticated process group")
    _positive_int(data["process_start_time_ticks"], "$.process_start_time_ticks")
    started = _nonnegative_int(data["started_monotonic_ns"], "$.started_monotonic_ns")
    ended = _nonnegative_int(data["ended_monotonic_ns"], "$.ended_monotonic_ns")
    if ended < started:
        _fail("observer_timing", "observer end precedes start")
    if data["exit_status"] is not None:
        _nonnegative_int(data["exit_status"], "$.exit_status")
    if data["terminating_signal"] is not None:
        _positive_int(data["terminating_signal"], "$.terminating_signal")
    if (data["exit_status"] is None) == (data["terminating_signal"] is None):
        _fail("observer_exit", "observer must bind exactly one exit status or signal")
    stdout_size = _nonnegative_int(data["stdout_size"], "$.stdout_size")
    stderr_size = _nonnegative_int(data["stderr_size"], "$.stderr_size")
    if stdout_size > OBSERVER_STDOUT_LIMIT_BYTES or stderr_size > OBSERVER_STDERR_LIMIT_BYTES:
        _fail("observer_output_size", "observer output exceeds its bound")
    _digest_value(data["stdout_sha256"], "$.stdout_sha256")
    _digest_value(data["stderr_sha256"], "$.stderr_sha256")
    nodes = _string_list(data["nodes"], "$.nodes")
    if len(nodes) > MAX_LIST_ITEMS:
        _fail("observer_node_count", "observer node count exceeds its bound")
    for index, node in enumerate(nodes):
        _ros_node_name(node, f"$.nodes[{index}]")
    if len(nodes) != len(set(nodes)):
        _fail("observer_duplicate_node", "observer node names must be unique")
    if current:
        _digest_value(data["parsed_node_set_identity"], "$.parsed_node_set_identity")
    _digest_value(data["cleanup_barrier_identity"], "$.cleanup_barrier_identity")
    _exact(data["unexpected_descendants"], 0, "$.unexpected_descendants")
    _exact_bool(data["ros_daemon_started"], False, "$.ros_daemon_started")


def _validate_four_source_observation(data: dict[str, Any]) -> None:
    if data["phase"] not in {"PRECOMMIT", "POSTCOMMIT"}:
        _fail("four_source_phase", "four-source phase differs")
    session_bound = data["schema_version"] in {
        LEGACY_FOUR_SOURCE_OBSERVATION_V2_SCHEMA, FOUR_SOURCE_OBSERVATION_SCHEMA,
    }
    if session_bound:
        _digest_value(data["session_binding_identity"], "$.session_binding_identity")
        _bounded_identifier(data["service_nonce"], "$.service_nonce")
        phase_ordinal = _positive_int(data["phase_local_ordinal"], "$.phase_local_ordinal")
        transaction_ordinal = _positive_int(
            data["transaction_observer_ordinal"], "$.transaction_observer_ordinal",
        )
        if data["phase"] == "PRECOMMIT" and phase_ordinal != transaction_ordinal:
            _fail("observer_ordinal", "precommit observer ordinals differ")
        if data["phase"] == "POSTCOMMIT" and phase_ordinal != 1:
            _fail("observer_ordinal", "postcommit phase ordinal must equal one")
        identity_fields = [
            "peer_process_identity", "observation_interval_identity",
            "cleanup_disposition_identity", "active_process_identity",
            "dds_port_identity", "global_lease_identity", "ros_graph_provider_identity",
        ]
        if data["schema_version"] == FOUR_SOURCE_OBSERVATION_SCHEMA:
            identity_fields.extend(("global_lease_registry_identity", "global_lease_revision_identity"))
            if data["global_lease_state"] not in {"CLEAR", "RESERVED", "COMMITTED"}:
                _fail("global_lease_state", "global lease observation state differs")
            if type(data["global_lease_clear"]) is not bool:
                _fail("global_lease_clear", "global lease clearance must be an exact Boolean")
            if data["global_lease_clear"] != (data["global_lease_state"] == "CLEAR"):
                _fail("global_lease_clear", "global lease state and clearance disagree")
        for field in identity_fields:
            _digest_value(data[field], f"$.{field}")
    else:
        _digest_value(data["observation_session_identity"], "$.observation_session_identity")
        for field in (
            "active_process_identity", "dds_port_identity", "global_lease_identity",
            "ros_graph_observation_identity",
        ):
            _digest_value(data[field], f"$.{field}")
    _domain_id(data["domain_id"], "$.domain_id")
    if type(data["all_sources_clear"]) is not bool:
        _fail("four_source_clear", "four-source clearance must be an exact Boolean")
    if (
        data["schema_version"] == FOUR_SOURCE_OBSERVATION_SCHEMA
        and data["all_sources_clear"]
        and not data["global_lease_clear"]
    ):
        _fail("four_source_clear", "four-source clearance requires a clear global lease source")
    _nonnegative_int(data["observed_monotonic_ns"], "$.observed_monotonic_ns")


def _validate_global_lease_observation(data: dict[str, Any]) -> None:
    _digest_value(data["registry_identity"], "$.registry_identity")
    _digest_value(data["registry_revision_identity"], "$.registry_revision_identity")
    _domain_id(data["domain_id"], "$.domain_id")
    if data["state"] not in {
        "CLEAR", "RESERVED", "COMMITTED", "CONFLICTING",
        "STALE_INVALID", "INDETERMINATE",
    }:
        _fail("global_lease_state", "global lease observation state differs")
    active = _digest_list(data["active_reservation_identities"], "$.active_reservation_identities")
    committed = _digest_list(data["committed_binding_identities"], "$.committed_binding_identities")
    stale = _digest_list(data["stale_invalid_identities"], "$.stale_invalid_identities")
    for values, path in ((active, "active"), (committed, "committed"), (stale, "stale")):
        if values != sorted(values) or len(values) != len(set(values)):
            _fail("global_lease_inventory", f"{path} lease identities must be unique and sorted")
    if type(data["clear"]) is not bool or data["clear"] != (data["state"] == "CLEAR"):
        _fail("global_lease_clear", "global lease state and clearance disagree")
    if data["state"] == "CLEAR" and (active or committed or stale):
        _fail("global_lease_clear", "clear lease observation contains active state")
    if data["state"] in {"RESERVED", "COMMITTED"} and not active:
        _fail("global_lease_state", "occupied lease observation lacks an active reservation")
    _nonnegative_int(data["observed_monotonic_ns"], "$.observed_monotonic_ns")


def _validate_cleanup_guard(data: dict[str, Any]) -> None:
    revision = _nonnegative_int(data["revision"], "$.revision")
    if revision == 0:
        if data["predecessor_identity"] is not None or data["state"] != "CLEARED":
            _fail("cleanup_guard_initial", "provisioned cleanup guard revision zero differs")
    else:
        _digest_value(data["predecessor_identity"], "$.predecessor_identity")
    if data["state"] not in {"CLEARED", "ACTIVE_UNBOUND", "ACTIVE_BOUND", "QUARANTINED", "RECOVERED"}:
        _fail("cleanup_guard_state", "cleanup guard state differs")
    for field in (
        "authorization_identity", "budget_identity", "service_generation_identity",
        "session_binding_identity", "executable_identity", "argv_identity",
        "environment_identity", "pidfd_identity", "disposition_identity",
        "recovery_authorization_identity",
    ):
        if data[field] is not None:
            _digest_value(data[field], f"$.{field}")
    if data["phase"] is not None and data["phase"] not in {"PRECOMMIT", "POSTCOMMIT", "RECOVERY"}:
        _fail("cleanup_guard_phase", "cleanup guard phase differs")
    for field in (
        "phase_local_ordinal", "transaction_observer_ordinal", "domain_id", "pid",
        "process_start_time_ticks", "process_group_id", "session_id",
    ):
        if data[field] is not None:
            _positive_int(data[field], f"$.{field}")
    if data["domain_id"] is not None:
        _domain_id(data["domain_id"], "$.domain_id")
    if data["cgroup"] is not None:
        _plain_string(data["cgroup"], "$.cgroup")
    context = (
        "authorization_identity", "budget_identity", "service_generation_identity",
        "session_binding_identity", "phase", "phase_local_ordinal",
        "transaction_observer_ordinal", "domain_id", "executable_identity",
        "argv_identity", "environment_identity",
    )
    process = ("pid", "process_start_time_ticks", "process_group_id", "session_id", "cgroup")
    if data["state"] == "CLEARED" and revision == 0:
        if any(data[field] is not None for field in (*context, *process, "pidfd_identity", "disposition_identity", "recovery_authorization_identity")):
            _fail("cleanup_guard_initial", "revision zero contains observer authority")
    elif data["state"] == "ACTIVE_UNBOUND":
        if any(data[field] is None for field in context) or any(data[field] is not None for field in (*process, "pidfd_identity", "disposition_identity", "recovery_authorization_identity")):
            _fail("cleanup_guard_state", "unbound active cleanup guard differs")
    elif data["state"] == "ACTIVE_BOUND":
        if any(data[field] is None for field in (*context, *process, "pidfd_identity")) or data["disposition_identity"] is not None:
            _fail("cleanup_guard_state", "bound active cleanup guard differs")
    elif data["state"] == "CLEARED":
        if any(data[field] is None for field in (*context, *process, "pidfd_identity", "disposition_identity")):
            _fail("cleanup_guard_state", "cleared cleanup guard lacks observer binding")
        if data["recovery_authorization_identity"] is not None:
            _fail("cleanup_guard_state", "ordinary terminal guard contains recovery authority")
    elif data["state"] == "QUARANTINED":
        bound = [data[field] is not None for field in process]
        if (
            any(data[field] is None for field in (*context, "disposition_identity"))
            or (any(bound) and not all(bound))
            or (all(bound) and data["pidfd_identity"] is None)
            or (not any(bound) and data["pidfd_identity"] is not None)
        ):
            _fail("cleanup_guard_state", "quarantined cleanup guard binding is inconsistent")
        if data["recovery_authorization_identity"] is not None:
            _fail("cleanup_guard_state", "ordinary terminal guard contains recovery authority")
    else:
        if data["disposition_identity"] is None or data["recovery_authorization_identity"] is None:
            _fail("cleanup_guard_state", "recovered guard lacks recovery authority")
    _utc(data["updated_at_utc"], "$.updated_at_utc")


def _validate_cleanup_recovery(data: dict[str, Any]) -> None:
    _bounded_identifier(data["recovery_nonce"], "$.recovery_nonce")
    for field in (
        "quarantine_identity", "authority_root_identity", "runtime_authorization_identity",
        "budget_identity", "service_generation_identity",
    ):
        _digest_value(data[field], f"$.{field}")
    issued = _utc(data["issued_at_utc"], "$.issued_at_utc")
    before = _utc(data["not_before_utc"], "$.not_before_utc")
    after = _utc(data["not_after_utc"], "$.not_after_utc")
    if not before <= issued < after:
        _fail("cleanup_recovery_validity", "cleanup recovery validity interval differs")
    _exact_bool(data["one_shot"], True, "$.one_shot")


def _validate_budget(data: dict[str, Any]) -> None:
    revision = _nonnegative_int(data["revision"], "$.revision")
    if revision == 0:
        if data["predecessor_identity"] is not None:
            _fail("budget_predecessor", "revision zero cannot have a predecessor")
    else:
        _digest_value(data["predecessor_identity"], "$.predecessor_identity")
    state = data["state"]
    if state not in {"UNCONSUMED", "COMMITTED", "COMPLETED", "FAILED_AFTER_COMMIT"}:
        _fail("budget_state", "unsupported global budget state")
    if revision == 0 and state != "UNCONSUMED":
        _fail("budget_state", "externally provisioned revision zero must be UNCONSUMED")
    if revision > 0 and state == "UNCONSUMED":
        _fail("budget_state", "UNCONSUMED cannot be recreated after revision zero")
    consumed = _nonnegative_int(data["attempts_consumed"], "$.attempts_consumed")
    _exact(data["attempts_maximum"], 1, "$.attempts_maximum")
    _exact(data["retries_authorized"], 0, "$.retries_authorized")
    precommit = _bounded_counter(data["precommit_observer_count"], MAX_PRECOMMIT_OBSERVERS, "$.precommit_observer_count")
    postcommit = _bounded_counter(data["postcommit_observer_count"], MAX_POSTCOMMIT_OBSERVERS, "$.postcommit_observer_count")
    total = _bounded_counter(data["transaction_observer_count"], MAX_TRANSACTION_OBSERVERS, "$.transaction_observer_count")
    receipts = _digest_list(data["precommit_receipt_identities"], "$.precommit_receipt_identities")
    if len(receipts) != precommit or total != precommit + postcommit:
        _fail("observer_counter", "budget observer counters are inconsistent")
    for field in (
        "authorization_identity", "observation_session_identity", "four_source_observation_identity",
        "postcommit_receipt_identity", "postcommit_four_source_observation_identity",
    ):
        if data[field] is not None:
            _digest_value(data[field], f"$.{field}")
    if state == "UNCONSUMED":
        if consumed != 0 or data["process_start_commitment"] is not None or receipts or total != 0:
            _fail("budget_state", "unconsumed state contains consuming bindings")
        if any(data[field] is not None for field in (
            "authorization_identity", "observation_session_identity", "four_source_observation_identity",
            "postcommit_receipt_identity", "postcommit_four_source_observation_identity",
        )):
            _fail("budget_state", "unconsumed state contains authority identities")
    else:
        if consumed != 1 or precommit < 1:
            _fail("budget_state", "post-commit state must consume one attempt and bind precommit observation")
        for field in ("authorization_identity", "observation_session_identity", "four_source_observation_identity"):
            if data[field] is None:
                _fail("budget_state", "post-commit state lacks observation authority", path=f"$.{field}")
        _validate_named_record(data["process_start_commitment"], "$.process_start_commitment")
        if postcommit == 0:
            if data["postcommit_receipt_identity"] is not None or data["postcommit_four_source_observation_identity"] is not None:
                _fail("budget_state", "zero postcommit count contains postcommit identities")
        elif data["postcommit_receipt_identity"] is None or data["postcommit_four_source_observation_identity"] is None:
            _fail("budget_state", "postcommit count lacks postcommit identities")
        if state == "COMPLETED" and postcommit != 1:
            _fail("budget_state", "completed campaign lacks the mandatory postcommit observation")
    _utc(data["updated_at_utc"], "$.updated_at_utc")


def _validate_request_v2(data: dict[str, Any]) -> None:
    methods = {
        "begin_observation", "record_precommit_observation", "finalize_observation",
        "prepare", "allocate_provisional", "cancel", "commit", "record_postcommit_observation",
        "complete", "fail_after_commit", "status", "revoke",
    }
    if data["method"] not in methods:
        _fail("authority_method", "unsupported authority request method")
    _bounded_identifier(data["request_id"], "$.request_id")
    for field in (
        "authorization_identity", "campaign_identity", "campaign_template_identity",
        "output_root_identity", "process_manifest_identity", "process_instance_identity",
        "observation_session_identity", "four_source_observation_identity", "lease_identity",
    ):
        if field in data and data[field] is not None:
            _digest_value(data[field], f"$.{field}")
    for field in ("prepare_token", "campaign_id", "observation_session_nonce"):
        if data[field] is not None:
            _bounded_identifier(data[field], f"$.{field}")
    if data["domain_id"] is not None:
        _domain_id(data["domain_id"], "$.domain_id")
    if data["output_root_path"] is not None:
        output = _absolute_normalized(data["output_root_path"], "$.output_root_path")
        if not _strict_descendant(output, OUTPUT_PARENT):
            _fail("output_root_path", "output root escapes fixed parent")
    if data["ros_graph_observation_receipt"] is not None:
        validate_authority_record(
            _exact_dict(data["ros_graph_observation_receipt"], "$.ros_graph_observation_receipt"),
            expected_schema=ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA,
        )
    if data["four_source_observation"] is not None:
        validate_authority_record(
            _exact_dict(data["four_source_observation"], "$.four_source_observation"),
            expected_schema=FOUR_SOURCE_OBSERVATION_SCHEMA,
        )
    receipts = _digest_list(data["precommit_receipt_identities"], "$.precommit_receipt_identities")
    precommit = _bounded_counter(data["precommit_observer_count"], MAX_PRECOMMIT_OBSERVERS, "$.precommit_observer_count")
    postcommit = _bounded_counter(data["postcommit_observer_count"], MAX_POSTCOMMIT_OBSERVERS, "$.postcommit_observer_count")
    total = _bounded_counter(data["transaction_observer_count"], MAX_TRANSACTION_OBSERVERS, "$.transaction_observer_count")
    if len(receipts) != precommit or total != precommit + postcommit:
        _fail("observer_counter", "authority request observer counters are inconsistent")
    if data["method"] not in {
        "record_postcommit_observation", "complete", "fail_after_commit",
    } and postcommit != 0:
        _fail(
            "observer_counter",
            "postcommit count is only valid for recording or final disposition",
        )
    _utc(data["requested_at_utc"], "$.requested_at_utc")


def _validate_receipt_v2(data: dict[str, Any]) -> None:
    _plain_string(data["method"], "$.method")
    _bounded_identifier(data["request_id"], "$.request_id")
    if data["result"] not in {
        "OBSERVATION_STARTED", "OBSERVATION_RECORDED", "OBSERVATION_COMPLETE", "PREPARED",
        "CANCELLED", "COMMITTED", "POSTCOMMIT_RECORDED", "COMPLETED",
        "FAILED_AFTER_COMMIT", "REVOKED", "STATUS", "ERROR",
    }:
        _fail("authority_result", "unsupported authority result")
    for field in (
        "authorization_identity", "service_instance_identity", "budget_identity", "campaign_identity",
        "campaign_template_identity", "output_root_identity", "process_manifest_identity",
        "process_instance_identity", "observation_session_identity",
        "four_source_observation_identity", "lease_identity",
    ):
        if data[field] is not None:
            _digest_value(data[field], f"$.{field}")
    for field in ("prepare_token", "campaign_id", "observation_session_nonce"):
        if data[field] is not None:
            _bounded_identifier(data[field], f"$.{field}")
    for field in (
        "previous_budget_revision", "budget_revision", "observation_session_deadline_monotonic_ns",
        "prepare_expires_monotonic_ns",
    ):
        if data[field] is not None:
            _nonnegative_int(data[field], f"$.{field}")
    if data["domain_id"] is not None:
        _domain_id(data["domain_id"], "$.domain_id")
    if data["output_root_path"] is not None:
        output = _absolute_normalized(data["output_root_path"], "$.output_root_path")
        if not _strict_descendant(output, OUTPUT_PARENT):
            _fail("output_root_path", "output root escapes fixed parent")
    receipts = _digest_list(data["precommit_receipt_identities"], "$.precommit_receipt_identities")
    precommit = _bounded_counter(data["precommit_observer_count"], MAX_PRECOMMIT_OBSERVERS, "$.precommit_observer_count")
    postcommit = _bounded_counter(data["postcommit_observer_count"], MAX_POSTCOMMIT_OBSERVERS, "$.postcommit_observer_count")
    total = _bounded_counter(data["transaction_observer_count"], MAX_TRANSACTION_OBSERVERS, "$.transaction_observer_count")
    if len(receipts) != precommit or total != precommit + postcommit:
        _fail("observer_counter", "authority receipt observer counters are inconsistent")
    if data["committed_at_utc"] is not None:
        _utc(data["committed_at_utc"], "$.committed_at_utc")
    if data["error_code"] is not None:
        _bounded_identifier(data["error_code"], "$.error_code")


def _validate_request(data: dict[str, Any]) -> None:
    methods = {
        "begin_observation", "record_precommit_observation", "finalize_observation",
        "prepare", "allocate_provisional", "cancel", "commit",
        "record_postcommit_observation", "complete", "fail_after_commit", "status", "revoke",
    }
    if data["method"] not in methods:
        _fail("authority_method", "unsupported authority request method")
    _bounded_identifier(data["request_id"], "$.request_id")
    for field in (
        "authorization_identity", "campaign_identity", "campaign_template_identity",
        "output_root_identity", "process_manifest_identity", "process_instance_identity",
        "observation_session_identity",
    ):
        if data[field] is not None:
            _digest_value(data[field], f"$.{field}")
    for field in ("prepare_token", "campaign_id", "observation_session_nonce"):
        if data[field] is not None:
            _bounded_identifier(data[field], f"$.{field}")
    if data["domain_id"] is not None:
        _domain_id(data["domain_id"], "$.domain_id")
    if data["output_root_path"] is not None:
        output = _absolute_normalized(data["output_root_path"], "$.output_root_path")
        if not _strict_descendant(output, OUTPUT_PARENT):
            _fail("output_root_path", "output root escapes fixed parent")
    _utc(data["requested_at_utc"], "$.requested_at_utc")


def _validate_receipt(data: dict[str, Any]) -> None:
    _validate_receipt_v2(data)
    if data["service_nonce"] is not None:
        _bounded_identifier(data["service_nonce"], "$.service_nonce")
    if data["candidate_clear"] is not None and type(data["candidate_clear"]) is not bool:
        _fail("candidate_clear", "candidate clearance must be an exact Boolean")


def _validate_revocation(data: dict[str, Any]) -> None:
    _bounded_identifier(data["revocation_id"], "$.revocation_id")
    _digest_value(data["authorization_identity"], "$.authorization_identity")
    if data["budget_revision"] is not None:
        _nonnegative_int(data["budget_revision"], "$.budget_revision")
    if data["state"] not in {"REQUESTED_PRECOMMIT", "TRIGGERED_POSTCOMMIT", "ENFORCED_POSTCOMMIT"}:
        _fail("revocation_state", "unsupported revocation state")
    _utc(data["requested_at_utc"], "$.requested_at_utc")
    _nonnegative_int(data["requested_by_uid"], "$.requested_by_uid")
    for field in ("trigger_identity", "processed_trigger_identity", "termination_receipt_identity"):
        if data[field] is not None:
            _digest_value(data[field], f"$.{field}")
    state = data["state"]
    if state == "REQUESTED_PRECOMMIT":
        if any(data[field] is not None for field in (
            "trigger_identity", "processed_trigger_identity", "termination_receipt_identity",
        )):
            _fail("revocation_binding", "pre-commit revocation cannot contain enforcement bindings")
    elif state == "TRIGGERED_POSTCOMMIT":
        if (
            data["trigger_identity"] is None
            or data["processed_trigger_identity"] is not None
            or data["termination_receipt_identity"] is not None
        ):
            _fail("revocation_binding", "pending post-commit revocation bindings differ")
    elif any(data[field] is None for field in (
        "trigger_identity", "processed_trigger_identity", "termination_receipt_identity",
    )):
        _fail("revocation_binding", "enforced post-commit revocation lacks a termination binding")


def _validate_installed_member(value: Any, path: str) -> dict[str, Any]:
    item = _exact_dict(value, path)
    _closed(item, {"path", "type", "mode", "link_count", "size", "sha256"}, path)
    _safe_relative(item["path"], f"{path}.path")
    if item["type"] not in {"regular", "directory"}:
        _fail("installed_member_type", "unsupported installed member type", path=f"{path}.type")
    _mode(item["mode"], f"{path}.mode")
    _positive_int(item["link_count"], f"{path}.link_count")
    if item["type"] == "regular":
        _nonnegative_int(item["size"], f"{path}.size")
        _digest_value(item["sha256"], f"{path}.sha256")
        if item["link_count"] != 1:
            _fail("installed_hardlink", "regular installed members must be single-link", path=path)
    elif item["size"] is not None or item["sha256"] is not None:
        _fail("installed_directory", "directory size and digest must be null", path=path)
    return item


def _validate_file_identity(value: Any, path: str) -> dict[str, Any]:
    item = _exact_dict(value, path)
    _closed(item, {"path", "mode", "link_count", "device", "inode", "size", "sha256", "owner_uid", "owner_gid"}, path)
    _absolute_normalized(item["path"], f"{path}.path")
    _mode(item["mode"], f"{path}.mode")
    _exact(item["link_count"], 1, f"{path}.link_count")
    for field in ("device", "inode", "size", "owner_uid", "owner_gid"):
        _nonnegative_int(item[field], f"{path}.{field}")
    _digest_value(item["sha256"], f"{path}.sha256")
    return item


def _validate_named_record(value: Any, path: str) -> dict[str, Any]:
    item = _exact_dict(value, path)
    if not item:
        _fail("authority_nested_record", "nested record must not be empty", path=path)
    for key, member in item.items():
        _bounded_identifier(key, f"{path}.{key}")
        _validate_json_value(member, f"{path}.{key}", 1)
    return item


def _validate_console_entrypoint(value: Any, path: str) -> dict[str, Any]:
    item = _exact_dict(value, path)
    _closed(item, {"name", "target", "script_path", "script_sha256"}, path)
    _bounded_identifier(item["name"], f"{path}.name")
    _plain_string(item["target"], f"{path}.target")
    _safe_relative(item["script_path"], f"{path}.script_path")
    _digest_value(item["script_sha256"], f"{path}.script_sha256")
    return item


def _validate_python_module(value: Any, path: str) -> dict[str, Any]:
    item = _exact_dict(value, path)
    _closed(item, {"module", "origin", "sha256"}, path)
    _plain_string(item["module"], f"{path}.module")
    _safe_relative(item["origin"], f"{path}.origin")
    _digest_value(item["sha256"], f"{path}.sha256")
    return item


def _validate_generated_interface(value: Any, path: str) -> dict[str, Any]:
    item = _exact_dict(value, path)
    _closed(item, {"name", "kind", "origin", "sha256"}, path)
    _plain_string(item["name"], f"{path}.name")
    if item["kind"] not in {"idl", "message", "service", "python", "typesupport"}:
        _fail("generated_interface_kind", "generated interface kind differs", path=f"{path}.kind")
    _safe_relative(item["origin"], f"{path}.origin")
    _digest_value(item["sha256"], f"{path}.sha256")
    return item


def _validate_elf_member(value: Any, path: str) -> dict[str, Any]:
    item = _exact_dict(value, path)
    _closed(item, {"path", "elf_class", "machine", "needed", "build_id", "rpath", "runpath", "unresolved"}, path)
    _safe_relative(item["path"], f"{path}.path")
    if item["elf_class"] not in {"ELF32", "ELF64"}:
        _fail("elf_class", "ELF class differs", path=f"{path}.elf_class")
    _plain_string(item["machine"], f"{path}.machine")
    for field in ("needed", "rpath", "runpath", "unresolved"):
        members = _string_list(item[field], f"{path}.{field}")
        if members != sorted(members) or len(members) != len(set(members)):
            _fail("elf_inventory", "ELF string inventory must be unique and sorted", path=f"{path}.{field}")
    if item["unresolved"]:
        _fail("elf_unresolved", "installed-runtime manifest contains an unresolved ELF dependency", path=path)
    _plain_string(item["build_id"], f"{path}.build_id")
    return item


def _validate_descendant(value: Any, path: str) -> dict[str, Any]:
    item = _exact_dict(value, path)
    _closed(item, {"role", "executable_identity", "parent_role", "multiplicity"}, path)
    _bounded_identifier(item["role"], f"{path}.role")
    _digest_value(item["executable_identity"], f"{path}.executable_identity")
    if item["parent_role"] is not None:
        _bounded_identifier(item["parent_role"], f"{path}.parent_role")
    _positive_int(item["multiplicity"], f"{path}.multiplicity")
    return item


def _validate_output_ownership(value: Any, path: str) -> dict[str, Any]:
    item = _exact_dict(value, path)
    _closed(item, {"authority_owner", "runtime_group", "campaign_account", "root_mode", "cell_mode", "stdout_role", "stderr_role"}, path)
    _exact(item["authority_owner"], AUTHORITY_ACCOUNT, f"{path}.authority_owner")
    _exact(item["runtime_group"], RUNTIME_GROUP, f"{path}.runtime_group")
    _exact(item["campaign_account"], CAMPAIGN_ACCOUNT, f"{path}.campaign_account")
    _exact(item["root_mode"], 0o750, f"{path}.root_mode")
    _exact(item["cell_mode"], 0o770, f"{path}.cell_mode")
    _safe_relative(item["stdout_role"], f"{path}.stdout_role")
    _safe_relative(item["stderr_role"], f"{path}.stderr_role")
    return item


def _ordered_unique_records(value: Any, path: str, validator: Any, key: str) -> list[dict[str, Any]]:
    records = _record_list(value, path, validator)
    identities = [record[key] for record in records]
    if identities != sorted(identities) or len(identities) != len(set(identities)):
        _fail("authority_record_order", "record inventory must be unique and sorted", path=path)
    return records


def _validate_source_snapshot(value: Any, path: str) -> None:
    item = _exact_dict(value, path)
    _closed(item, {
        "schema_version", "path", "physical_sha256", "logical_identity",
        "logical_identity_algorithm", "member_count", "mode_bound",
    }, path)
    _plain_string(item["schema_version"], f"{path}.schema_version")
    _absolute_normalized(item["path"], f"{path}.path")
    _digest_value(item["physical_sha256"], f"{path}.physical_sha256")
    _digest_value(item["logical_identity"], f"{path}.logical_identity")
    _identity_algorithm(item["logical_identity_algorithm"], f"{path}.logical_identity_algorithm")
    _positive_int(item["member_count"], f"{path}.member_count")
    _exact_bool(item["mode_bound"], True, f"{path}.mode_bound")


def _validate_charter_binding(value: Any, path: str) -> None:
    item = _exact_dict(value, path)
    _closed(item, {
        "schema_version", "path", "physical_sha256", "logical_identity",
        "logical_identity_algorithm",
    }, path)
    _exact(item["schema_version"], "ctr-slice-7g-charter-6", f"{path}.schema_version")
    _absolute_normalized(item["path"], f"{path}.path")
    _digest_value(item["physical_sha256"], f"{path}.physical_sha256")
    _digest_value(item["logical_identity"], f"{path}.logical_identity")
    _exact(
        item["logical_identity_algorithm"], "sha256:ctr-slice-7g-charter-canonical-6",
        f"{path}.logical_identity_algorithm",
    )


def _validate_campaign(value: Any, path: str) -> None:
    item = _exact_dict(value, path)
    _closed(item, {"endpoint", "scenarios", "seeds", "duration_seconds", "retries", "domain_minimum", "domain_maximum", "plan_identity", "campaign_identity_algorithm"}, path)
    _exact(item["endpoint"], "simulation_only_promoted_completion", f"{path}.endpoint")
    if _string_list(item["scenarios"], f"{path}.scenarios") != ["centerline", "lateral_offset", "near_safety_boundary"]:
        _fail("campaign_scenarios", "campaign scenarios differ", path=f"{path}.scenarios")
    if type(item["seeds"]) is not list or item["seeds"] != [11, 22, 33, 44, 55] or any(type(v) is not int for v in item["seeds"]):
        _fail("campaign_seeds", "campaign seeds differ", path=f"{path}.seeds")
    _exact_number(item["duration_seconds"], 25.0, f"{path}.duration_seconds")
    _exact(item["retries"], 0, f"{path}.retries")
    _exact(item["domain_minimum"], 100, f"{path}.domain_minimum")
    _exact(item["domain_maximum"], 199, f"{path}.domain_maximum")
    _digest_value(item["plan_identity"], f"{path}.plan_identity")
    _exact(item["campaign_identity_algorithm"], "sha256:ctr-slice-7g-runtime-campaign-canonical-1", f"{path}.campaign_identity_algorithm")


def _validate_output_parent_rule(value: Any, path: str) -> None:
    item = _exact_dict(value, path)
    _closed(item, {"path", "authority_creates_root", "campaign_parent_entry_mutation", "campaign_parent_listing", "acl_policy_identity"}, path)
    _exact(item["path"], OUTPUT_PARENT, f"{path}.path")
    _exact_bool(item["authority_creates_root"], True, f"{path}.authority_creates_root")
    _exact_bool(item["campaign_parent_entry_mutation"], False, f"{path}.campaign_parent_entry_mutation")
    _exact_bool(item["campaign_parent_listing"], False, f"{path}.campaign_parent_listing")
    _digest_value(item["acl_policy_identity"], f"{path}.acl_policy_identity")


def _record_list(value: Any, path: str, validator: Any) -> list[dict[str, Any]]:
    if type(value) is not list or len(value) > MAX_LIST_ITEMS:
        _fail("authority_list", "record list is invalid or oversized", path=path)
    return [validator(item, f"{path}[{index}]") for index, item in enumerate(value)]


def _string_list(value: Any, path: str) -> list[str]:
    if type(value) is not list or len(value) > MAX_LIST_ITEMS:
        _fail("authority_list", "string list is invalid or oversized", path=path)
    return [_plain_string(item, f"{path}[{index}]") for index, item in enumerate(value)]


def _parse_canonical_json(raw: bytes) -> dict[str, Any]:
    if not raw or len(raw) > MAX_FRAME_BYTES or raw.endswith((b" ", b"\t", b"\r", b"\n")):
        _fail("authority_json_size", "authority JSON is empty, oversized, or has trailing whitespace")
    try:
        text = raw.decode("utf-8", "strict")
        value = json.loads(text, object_pairs_hook=_pairs, parse_constant=lambda value: _fail("authority_json_number", f"invalid number {value}"))
    except Slice7GAuthorityProtocolError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise Slice7GAuthorityProtocolError("authority_json", str(exc)) from exc
    if type(value) is not dict:
        _fail("authority_json_type", "authority JSON must contain one object")
    data = _detach(value)
    if raw != _canonical(data):
        _fail("authority_noncanonical", "authority JSON is not canonical")
    return data


def _pairs(items: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in items:
        if type(key) is not str or key in result:
            _fail("authority_json_key", "JSON keys must be unique exact strings")
        result[key] = value
    return result


def _detach(value: Any, path: str = "$", depth: int = 0) -> Any:
    if depth > MAX_RECORD_DEPTH:
        _fail("authority_depth", "authority record exceeds nesting bound", path=path)
    if value is None or type(value) in (bool, int, str):
        if type(value) is str:
            _plain_string(value, path, allow_empty=True)
        return value
    if type(value) is float:
        if not math.isfinite(value):
            _fail("authority_number", "authority numbers must be finite", path=path)
        return value
    if type(value) is list:
        if len(value) > MAX_LIST_ITEMS:
            _fail("authority_list", "authority list is oversized", path=path)
        return [_detach(item, f"{path}[{index}]", depth + 1) for index, item in enumerate(value)]
    if type(value) is dict:
        if len(value) > MAX_LIST_ITEMS:
            _fail("authority_object", "authority object is oversized", path=path)
        result: dict[str, Any] = {}
        for key, member in value.items():
            if type(key) is not str:
                _fail("authority_json_key", "authority keys must be exact strings", path=path)
            _plain_string(key, f"{path}.<key>")
            result[key] = _detach(member, f"{path}.{key}", depth + 1)
        return result
    _fail("authority_primitive", "authority values must be exact built-in JSON primitives", path=path)


def _freeze(value: Any) -> Any:
    if type(value) is dict:
        return MappingProxyType({key: _freeze(member) for key, member in value.items()})
    if type(value) is list:
        return tuple(_freeze(member) for member in value)
    return value


def _canonical(value: dict[str, Any]) -> bytes:
    return json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _closed(value: dict[str, Any], expected: set[str] | frozenset[str], path: str) -> None:
    if set(value) != set(expected):
        _fail("authority_fields", "authority record has missing or unknown fields", path=path)


def _exact_dict(value: Any, path: str) -> dict[str, Any]:
    if type(value) is not dict:
        _fail("authority_object", "value must be an exact object", path=path)
    return value


def _validate_json_value(value: Any, path: str, depth: int) -> None:
    _detach(value, path, depth)


def _plain_string(value: Any, path: str, *, allow_empty: bool = False) -> str:
    if type(value) is not str or (not allow_empty and not value):
        _fail("authority_string", "value must be an exact string", path=path)
    if len(value.encode("utf-8")) > MAX_STRING_BYTES or unicodedata.normalize("NFC", value) != value:
        _fail("authority_string", "string is oversized or non-NFC", path=path)
    if any(ord(char) < 0x20 or ord(char) == 0x7F for char in value):
        _fail("authority_string", "string contains control characters", path=path)
    return value


def _exact_string(value: Any, path: str) -> str:
    return _plain_string(value, path)


def _bounded_identifier(value: Any, path: str) -> str:
    text = _plain_string(value, path)
    if _NAME.fullmatch(text) is None:
        _fail("authority_identifier", "identifier is unsafe", path=path)
    return text


def _environment_key(value: Any, path: str) -> str:
    text = _plain_string(value, path)
    if re.fullmatch(r"[A-Z][A-Z0-9_]{0,127}", text) is None:
        _fail("environment_key", "environment key is unsafe", path=path)
    return text


def _exact(value: Any, expected: Any, path: str) -> Any:
    if type(value) is not type(expected) or value != expected:
        _fail("authority_value", f"value must equal {expected!r}", path=path)
    return value


def _exact_bool(value: Any, expected: bool, path: str) -> bool:
    return _exact(value, expected, path)


def _nonnegative_int(value: Any, path: str) -> int:
    if type(value) is not int or value < 0:
        _fail("authority_integer", "value must be a nonnegative exact integer", path=path)
    return value


def _bounded_counter(value: Any, maximum: int, path: str) -> int:
    observed = _nonnegative_int(value, path)
    if observed > maximum:
        _fail("observer_counter", f"observer counter exceeds {maximum}", path=path)
    return observed


def _domain_id(value: Any, path: str) -> int:
    if type(value) is not int or not 100 <= value <= 199:
        _fail("domain_id", "domain ID must be an exact integer from 100 through 199", path=path)
    return value


def _domain_list(value: Any, path: str) -> list[int]:
    if type(value) is not list:
        _fail("observer_candidates", "candidate domains must be an exact list", path=path)
    if len(value) > MAX_PRECOMMIT_OBSERVERS:
        _fail("observer_candidates", "candidate domain count exceeds 100", path=path)
    result = [_domain_id(item, f"{path}[{index}]") for index, item in enumerate(value)]
    if result != sorted(result) or len(result) != len(set(result)):
        _fail("observer_candidates", "candidate domains must be unique and ascending", path=path)
    return result


def _digest_list(value: Any, path: str) -> list[str]:
    values = _string_list(value, path)
    for index, item in enumerate(values):
        _digest_value(item, f"{path}[{index}]")
    return values


def _ros_node_name(value: Any, path: str) -> str:
    node = _plain_string(value, path)
    if unicodedata.normalize("NFC", node) != node or not node.startswith("/") or node == "/":
        _fail("observer_node_name", "ROS node name must be NFC and absolute", path=path)
    encoded = node.encode("utf-8")
    if len(encoded) > MAX_STRING_BYTES:
        _fail("observer_node_name", "ROS node name exceeds its byte bound", path=path)
    if any(ord(character) < 0x20 or ord(character) == 0x7F for character in node):
        _fail("observer_node_name", "ROS node name contains a control character", path=path)
    components = node[1:].split("/")
    if any(not component or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", component) for component in components):
        _fail("observer_node_name", "ROS node name contains a malformed component", path=path)
    return node


def _positive_int(value: Any, path: str) -> int:
    result = _nonnegative_int(value, path)
    if result == 0:
        _fail("authority_integer", "value must be positive", path=path)
    return result


def _positive_number(value: Any, path: str) -> float:
    if type(value) not in (int, float) or type(value) is bool or not math.isfinite(float(value)) or value <= 0:
        _fail("authority_number", "value must be a positive finite exact number", path=path)
    return float(value)


def _exact_number(value: Any, expected: float, path: str) -> None:
    if type(value) not in (int, float) or type(value) is bool or float(value) != expected:
        _fail("authority_number", f"value must equal {expected}", path=path)


def _mode(value: Any, path: str) -> int:
    if type(value) is not int or not 0 <= value <= 0o7777:
        _fail("authority_mode", "mode must be an exact permission integer", path=path)
    return value


def _digest_value(value: Any, path: str) -> str:
    text = _plain_string(value, path)
    if _DIGEST.fullmatch(text) is None:
        _fail("authority_digest", "digest must be 64 lowercase hexadecimal characters", path=path)
    return text


def _identity_algorithm(value: Any, path: str) -> str:
    text = _plain_string(value, path)
    if re.fullmatch(r"sha256:[a-z0-9][a-z0-9-]{0,191}", text) is None:
        _fail("authority_identity_algorithm", "identity algorithm is unsupported", path=path)
    return text


def _hex(value: Any, length: int, path: str) -> str:
    text = _plain_string(value, path)
    if len(text) != length or re.fullmatch(r"[0-9a-f]+", text) is None:
        _fail("authority_hex", "value has invalid hexadecimal form", path=path)
    return text


def _safe_relative(value: Any, path: str) -> str:
    text = _plain_string(value, path)
    candidate = PurePosixPath(text)
    if candidate.is_absolute() or text.startswith("./") or "\\" in text or "//" in text or any(part in ("", ".", "..") for part in candidate.parts):
        _fail("authority_relative_path", "relative path is unsafe", path=path)
    return text


def _absolute_normalized(value: Any, path: str) -> str:
    text = _plain_string(value, path)
    candidate = PurePosixPath(text)
    if not candidate.is_absolute() or "\\" in text or "//" in text or any(part in (".", "..") for part in candidate.parts):
        _fail("authority_absolute_path", "absolute path is not normalized", path=path)
    return text


def _strict_descendant(path: str, parent: str) -> bool:
    candidate = PurePosixPath(path)
    base = PurePosixPath(parent)
    return candidate != base and base in candidate.parents


def _utc(value: Any, path: str) -> datetime:
    text = _plain_string(value, path)
    if re.fullmatch(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z", text) is None:
        _fail("authority_timestamp", "timestamp must be canonical whole-second UTC", path=path)
    try:
        observed = datetime.strptime(text, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
    except ValueError as exc:
        raise Slice7GAuthorityProtocolError("authority_timestamp", str(exc), path=path) from exc
    return observed


def _recv_exact(channel: socket.socket, size: int) -> bytes:
    result = bytearray()
    while len(result) < size:
        try:
            chunk = channel.recv(size - len(result))
        except OSError as exc:
            raise Slice7GAuthorityProtocolError("authority_socket_read", str(exc)) from exc
        if not chunk:
            _fail("authority_frame_truncated", "authority socket reached EOF within a frame")
        result.extend(chunk)
    return bytes(result)


def _read_bounded(path: str, maximum: int) -> bytes:
    try:
        with open(path, "rb", buffering=0) as stream:
            value = stream.read(maximum + 1)
    except OSError as exc:
        raise Slice7GAuthorityProtocolError("peer_process_read", str(exc), path=path) from exc
    if len(value) > maximum:
        _fail("peer_process_size", "peer process observation exceeds bound", path=path)
    return value


def _read_fixed_file(
    path: str,
    *,
    maximum: int,
    owner_uid: int | None,
    owner_gid: int | None,
    mode: int | None,
) -> tuple[bytes, tuple[int, int, int, int, int, int, int]]:
    """Read an absolute file through no-follow descriptor traversal."""

    normalized = _absolute_normalized(path, "$.fixed_path")
    parts = PurePosixPath(normalized).parts[1:]
    if not parts:
        _fail("fixed_file", "fixed file path cannot be root")
    descriptor = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        for part in parts[:-1]:
            next_descriptor = os.open(
                part,
                os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW,
                dir_fd=descriptor,
            )
            os.close(descriptor)
            descriptor = next_descriptor
        file_descriptor = os.open(
            parts[-1], os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=descriptor,
        )
        try:
            before = os.fstat(file_descriptor)
            if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
                _fail("fixed_file", "fixed authority file must be regular and single-link", path=normalized)
            if owner_uid is not None and before.st_uid != owner_uid:
                _fail("fixed_file_owner", "fixed authority file owner differs", path=normalized)
            if owner_gid is not None and before.st_gid != owner_gid:
                _fail("fixed_file_group", "fixed authority file group differs", path=normalized)
            if mode is not None and stat.S_IMODE(before.st_mode) != mode:
                _fail("fixed_file_mode", "fixed authority file mode differs", path=normalized)
            chunks: list[bytes] = []
            total = 0
            while True:
                chunk = os.read(file_descriptor, min(1_048_576, maximum + 1 - total))
                if not chunk:
                    break
                chunks.append(chunk)
                total += len(chunk)
                if total > maximum:
                    _fail("fixed_file_size", "fixed authority file exceeds bound", path=normalized)
            after = os.fstat(file_descriptor)
            if _stat_tuple(before) != _stat_tuple(after):
                _fail("fixed_file_replaced", "fixed authority file changed during read", path=normalized)
            return b"".join(chunks), _stat_tuple(after)
        finally:
            os.close(file_descriptor)
    except OSError as exc:
        raise Slice7GAuthorityProtocolError("fixed_file_read", str(exc), path=normalized) from exc
    finally:
        os.close(descriptor)


def _stat_tuple(observed: os.stat_result) -> tuple[int, int, int, int, int, int, int]:
    return (
        observed.st_dev,
        observed.st_ino,
        stat.S_IMODE(observed.st_mode),
        observed.st_nlink,
        observed.st_size,
        observed.st_uid,
        observed.st_gid,
    )


def _proc_start_time(root: str) -> int:
    raw = _read_bounded(f"{root}/stat", 65_536).decode("ascii", "strict")
    close = raw.rfind(")")
    if close < 0:
        _fail("peer_process_stat", "peer stat record is malformed")
    fields = raw[close + 2 :].split()
    if len(fields) < 20 or not fields[19].isdigit():
        _fail("peer_process_stat", "peer start time is unavailable")
    return int(fields[19])


def _nul_strings(raw: bytes, path: str) -> list[str]:
    if not raw or raw[-1:] != b"\0":
        _fail("peer_process_record", "NUL-separated process record is malformed", path=path)
    try:
        values = [item.decode("utf-8", "strict") for item in raw[:-1].split(b"\0")]
    except UnicodeDecodeError as exc:
        raise Slice7GAuthorityProtocolError("peer_process_utf8", str(exc), path=path) from exc
    for index, value in enumerate(values):
        _plain_string(value, f"{path}[{index}]", allow_empty=False)
    return values


def _fail(code: str, message: str, *, path: str = "$") -> None:
    raise Slice7GAuthorityProtocolError(code, message, path=path)


__all__ = [
    "AUTHORITY_ACCOUNT", "AUTHORITY_BOOTSTRAP_PATH", "AUTHORITY_BOOTSTRAP_SCHEMA",
    "AUTHORITY_REQUEST_SCHEMA", "AUTHORITY_RECEIPT_SCHEMA", "AUTHORITY_REVOCATION_SCHEMA",
    "AUTHORITY_SERVICE_PATH", "AUTHORITY_SOCKET_PATH", "AUTHORITY_STATE_ROOT",
    "BUILD_TEST_APPROVAL_SCHEMA", "CAMPAIGN_ACCOUNT", "CAMPAIGN_SYSTEMD_UNIT",
    "ENVIRONMENT_MANIFEST_SCHEMA", "GLOBAL_ATTEMPT_BUDGET_SCHEMA",
    "INSTALLED_RUNTIME_MANIFEST_SCHEMA", "INSTALLED_RUNTIME_PARENT", "MAX_FRAME_BYTES",
    "MAX_SESSION_REQUESTS",
    "OUTPUT_PARENT", "PROCESS_MANIFEST_SCHEMA", "REVOCATION_PATH_UNIT",
    "REVOCATION_SERVICE_UNIT", "RUNTIME_AUTHORIZATION_SCHEMA", "RUNTIME_GROUP",
    "Slice7GAuthorityClient", "Slice7GAuthorityProtocolError", "Slice7GAuthorityRecord",
    "Slice7GAuthoritySession",
    "Slice7GPeerCredentials", "Slice7GPeerProcess", "authority_record_identity",
    "authenticate_file_identity", "load_production_bootstrap",
    "authority_schema_names", "canonical_authority_record_bytes", "decode_authority_frame",
    "encode_authority_frame", "observe_peer_process", "peer_credentials",
    "receive_authority_frame", "reconcile_peer_process", "send_authority_frame",
    "validate_authority_record",
]
