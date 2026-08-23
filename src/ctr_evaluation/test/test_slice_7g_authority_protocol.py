import hashlib
import json
import socket

import pytest

from ctr_evaluation import slice_7g_authority_protocol as protocol


DIGEST = "0" * 64
OTHER_DIGEST = "1" * 64


def file_identity(path="/usr/bin/python3", inode=2):
    return {
        "path": path, "mode": 0o555, "link_count": 1, "device": 1, "inode": inode,
        "size": 3, "sha256": DIGEST, "owner_uid": 0, "owner_gid": 0,
    }


def source_snapshot():
    return {
        "schema_version": "ctr-slice-7g-post-implementation-source-snapshot-3",
        "path": "/var/lib/ctr-mppi/slice-7g-authority/public/source.json",
        "physical_sha256": DIGEST, "logical_identity": OTHER_DIGEST,
        "logical_identity_algorithm": "sha256:ctr-slice-7g-post-implementation-source-snapshot-canonical-3",
        "member_count": 1, "mode_bound": True,
    }


def bootstrap():
    return {
        "schema_version": protocol.AUTHORITY_BOOTSTRAP_SCHEMA,
        "authority_uid": 101, "authority_gid": 102, "campaign_uid": 103, "runtime_gid": 104,
        "authority_account": "ctr7g-authority", "campaign_account": "ctr7g-campaign",
        "runtime_group": "ctr7g-runtime", "bootstrap_path": protocol.AUTHORITY_BOOTSTRAP_PATH,
        "service_executable_path": protocol.AUTHORITY_SERVICE_PATH,
        "state_root": protocol.AUTHORITY_STATE_ROOT, "socket_path": protocol.AUTHORITY_SOCKET_PATH,
        "installed_runtime_parent": protocol.INSTALLED_RUNTIME_PARENT,
        "service_executable": file_identity(protocol.AUTHORITY_SERVICE_PATH),
        "record_paths": {
            "installed_runtime_manifest": protocol.AUTHORITY_STATE_ROOT + "/public/installed.json",
            "build_test_approval": protocol.AUTHORITY_STATE_ROOT + "/public/build.json",
            "runtime_authorization": protocol.AUTHORITY_STATE_ROOT + "/public/authorization.json",
            "process_manifest": protocol.AUTHORITY_STATE_ROOT + "/public/process.json",
            "environment_manifest": protocol.AUTHORITY_STATE_ROOT + "/public/environment.json",
            "budget_root": protocol.AUTHORITY_STATE_ROOT + "/global-budget",
            "revocation_root": protocol.AUTHORITY_STATE_ROOT + "/revocation",
        },
        "schemas": {
            "authority_bootstrap": protocol.AUTHORITY_BOOTSTRAP_SCHEMA,
            "installed_runtime_manifest": protocol.INSTALLED_RUNTIME_MANIFEST_SCHEMA,
            "isolated_build_test_approval": protocol.BUILD_TEST_APPROVAL_SCHEMA,
            "runtime_authorization": protocol.RUNTIME_AUTHORIZATION_SCHEMA,
            "process_manifest": protocol.PROCESS_MANIFEST_SCHEMA,
            "environment_manifest": protocol.ENVIRONMENT_MANIFEST_SCHEMA,
            "global_attempt_budget": protocol.GLOBAL_ATTEMPT_BUDGET_SCHEMA,
            "runtime_authority_request": protocol.AUTHORITY_REQUEST_SCHEMA,
            "runtime_authority_receipt": protocol.AUTHORITY_RECEIPT_SCHEMA,
            "runtime_authority_revocation": protocol.AUTHORITY_REVOCATION_SCHEMA,
            "observation_session": protocol.OBSERVATION_SESSION_SCHEMA,
            "ros_graph_observation_receipt": protocol.ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA,
            "four_source_domain_observation": protocol.FOUR_SOURCE_OBSERVATION_SCHEMA,
        },
        "protocol_limits": {
            "maximum_frame_bytes": protocol.MAX_FRAME_BYTES,
            "maximum_string_bytes": protocol.MAX_STRING_BYTES,
            "maximum_list_items": protocol.MAX_LIST_ITEMS,
            "maximum_record_depth": protocol.MAX_RECORD_DEPTH,
            "maximum_session_requests": protocol.MAX_SESSION_REQUESTS,
        },
        "systemd_units": {
            "authority": protocol.AUTHORITY_SYSTEMD_UNIT,
            "campaign": protocol.CAMPAIGN_SYSTEMD_UNIT,
            "revocation_path": protocol.REVOCATION_PATH_UNIT,
            "revocation_service": protocol.REVOCATION_SERVICE_UNIT,
        },
    }


def environment_manifest():
    keys = sorted({
        "PATH", "PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "LD_LIBRARY_PATH",
        "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_HOME", "HOME", "XDG_CACHE_HOME",
        "ROS_LOG_DIR", "ROS_LOCALHOST_ONLY", "MPLCONFIGDIR", "PYTHONDONTWRITEBYTECODE",
        "PYTHONNOUSERSITE",
    })
    fixed = {key: "/opt/ctr-mppi/slice-7g/fixed" for key in keys}
    fixed.update({
        "PYTHONDONTWRITEBYTECODE": "1", "PYTHONNOUSERSITE": "1", "ROS_DOMAIN_ID": "100",
        "ROS_LOCALHOST_ONLY": "1",
    })
    return {
        "schema_version": protocol.ENVIRONMENT_MANIFEST_SCHEMA,
        "identity_algorithm": "sha256:ctr-slice-7g-environment-manifest-canonical-1",
        "allowed_keys": keys, "required_keys": keys,
        "required_absent_keys": ["PYTHONINSPECT"], "fixed_values": fixed,
        "transaction_values": {},
        "path_keys": ["PATH", "PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "LD_LIBRARY_PATH"],
        "path_order": ["PATH", "PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "LD_LIBRARY_PATH"],
        "inherit_parent_environment": False,
    }


def process_manifest():
    env_identity = protocol.authority_record_identity(
        environment_manifest(), expected_schema=protocol.ENVIRONMENT_MANIFEST_SCHEMA,
    )
    return {
        "schema_version": protocol.PROCESS_MANIFEST_SCHEMA,
        "identity_algorithm": "sha256:ctr-slice-7g-process-manifest-canonical-1",
        "interpreter": file_identity("/usr/bin/python3.10", 2), "interpreter_flags": ["-I"],
        "entrypoint": file_identity("/opt/ctr-mppi/slice-7g/" + DIGEST + "/campaign.py", 3),
        "executables": [file_identity("/opt/ros/humble/bin/ros2", 4)],
        "argv_template": ["/usr/bin/python3.10", "-I", "/opt/ctr-mppi/slice-7g/" + DIGEST + "/campaign.py"],
        "transaction_slots": {}, "environment_manifest_identity": env_identity,
        "working_directory": "/opt/ctr-mppi/slice-7g/" + DIGEST,
        "shell": False, "systemd_unit": protocol.CAMPAIGN_SYSTEMD_UNIT,
        "cgroup": "/system.slice/ctr-slice7g-campaign.service",
        "allowed_descendants": [{
            "role": "ros_launch", "executable_identity": DIGEST,
            "parent_role": "coordinator", "multiplicity": 1,
        }],
        "timeouts": {"sigint_seconds": 1.0, "sigterm_seconds": 1.0, "sigkill_seconds": 1.0, "cell_seconds": 25.0},
        "output_ownership": {
            "authority_owner": "ctr7g-authority", "runtime_group": "ctr7g-runtime",
            "campaign_account": "ctr7g-campaign", "root_mode": 0o750, "cell_mode": 0o770,
            "stdout_role": "authority/process_stdout.bin", "stderr_role": "authority/process_stderr.bin",
        },
        "required_receipts": ["cleanup", "process_start"],
    }


def installed_manifest():
    return {
        "schema_version": protocol.INSTALLED_RUNTIME_MANIFEST_SCHEMA,
        "identity_algorithm": "sha256:ctr-slice-7g-installed-runtime-tree-canonical-1",
        "installed_runtime_identity": DIGEST,
        "root_path": protocol.INSTALLED_RUNTIME_PARENT + "/" + DIGEST,
        "root_device": 1, "root_inode": 2, "physical_tree_identity": OTHER_DIGEST,
        "member_count": 1,
        "members": [{"path": "bin/tool", "type": "regular", "mode": 0o555, "link_count": 1, "size": 3, "sha256": DIGEST}],
        "console_entrypoints": [{
            "name": "ctr_run_slice_7g_campaign", "target": "ctr_evaluation.slice_7g_runtime:main",
            "script_path": "lib/ctr_evaluation/ctr_run_slice_7g_campaign", "script_sha256": DIGEST,
        }],
        "python_modules": [{
            "module": "ctr_evaluation.slice_7g_runtime", "origin": "lib/python3.10/site-packages/ctr_evaluation/slice_7g_runtime.py",
            "sha256": DIGEST,
        }],
        "generated_interfaces": [{
            "name": "ctr_interfaces.msg", "kind": "python",
            "origin": "lib/python3.10/site-packages/ctr_interfaces/msg/__init__.py", "sha256": DIGEST,
        }],
        "elf_members": [{
            "path": "lib/typesupport.so", "elf_class": "ELF64", "machine": "Advanced Micro Devices X86-64",
            "needed": ["libc.so.6"], "build_id": "00", "rpath": [], "runpath": [], "unresolved": [],
        }],
        "process_manifest_identity": OTHER_DIGEST,
        "environment_manifest_identity": DIGEST,
        "source_snapshot": source_snapshot(),
        "build_test_approval_identity": OTHER_DIGEST,
    }


def build_approval():
    return {
        "schema_version": protocol.BUILD_TEST_APPROVAL_SCHEMA, "source_snapshot": source_snapshot(),
        "branch": "milestone/06b-curved-lumen-sim", "head": "a" * 40,
        "tracked_diff_sha256": DIGEST, "applicable_test_nodes": 1,
        "node_id_sha256": DIGEST, "git_command_manifest_sha256": DIGEST,
        "packages": ["ctr_evaluation"], "packages_built": 1, "tests_passed": 1,
        "origin_violations": 0, "installed_runtime_proposal_identity": DIGEST,
        "issued_at_utc": "2026-08-22T00:00:00Z",
    }


def runtime_authorization():
    return {
        "schema_version": protocol.RUNTIME_AUTHORIZATION_SCHEMA,
        "authorization_nonce": "authority1", "issued_at_utc": "2026-08-22T00:00:00Z",
        "not_before_utc": "2026-08-22T00:00:00Z", "not_after_utc": "2026-08-23T00:00:00Z",
        "branch": "milestone/06b-curved-lumen-sim", "head": "a" * 40,
        "tracked_diff_sha256": DIGEST, "correction_manifest_sha256": DIGEST,
        "complete_subject_manifest_sha256": DIGEST, "source_snapshot": source_snapshot(),
        "charter": {"schema_version": "ctr-slice-7g-charter-6", "path": "/opt/ctr-mppi/slice-7g/charter.json", "physical_sha256": DIGEST, "logical_identity": DIGEST, "logical_identity_algorithm": "sha256:ctr-slice-7g-charter-canonical-6"},
        "build_test_approval_identity": DIGEST, "installed_runtime_identity": DIGEST,
        "process_manifest_identity": DIGEST, "environment_manifest_identity": DIGEST,
        "applicable_test_nodes": 1, "node_id_sha256": DIGEST,
        "git_command_manifest_sha256": DIGEST, "entrypoint_identity": DIGEST,
        "campaign": {
            "endpoint": "simulation_only_promoted_completion",
            "scenarios": ["centerline", "lateral_offset", "near_safety_boundary"],
            "seeds": [11, 22, 33, 44, 55], "duration_seconds": 25.0,
            "retries": 0, "domain_minimum": 100, "domain_maximum": 199,
            "plan_identity": DIGEST,
            "campaign_identity_algorithm": "sha256:ctr-slice-7g-runtime-campaign-canonical-1",
        },
        "readiness_acceptance_identity": DIGEST,
        "evidence_schemas": {"seal": "ctr-slice-7g-campaign-evidence-seal-1"},
        "global_budget_identity": DIGEST,
        "output_parent_rule": {
            "path": protocol.OUTPUT_PARENT, "authority_creates_root": True,
            "campaign_parent_entry_mutation": False, "campaign_parent_listing": False,
            "acl_policy_identity": DIGEST,
        },
        "prepare_token_lifetime_seconds": 300, "one_shot": True,
    }


def request():
    return {
        "schema_version": protocol.AUTHORITY_REQUEST_SCHEMA, "method": "prepare",
        "request_id": "request1", "authorization_identity": None, "prepare_token": None,
        "campaign_id": None, "campaign_identity": None, "campaign_template_identity": None,
        "domain_id": None, "output_root_path": None, "output_root_identity": None,
        "process_manifest_identity": None, "process_instance_identity": None,
        "observation_session_identity": None, "observation_session_nonce": None,
        "requested_at_utc": "2026-08-22T00:00:00Z",
    }


def receipt():
    return {
        "schema_version": protocol.AUTHORITY_RECEIPT_SCHEMA, "method": "prepare",
        "request_id": "request1", "result": "PREPARED", "authorization_identity": DIGEST,
        "service_instance_identity": DIGEST, "service_nonce": "observe1",
        "prepare_token": "prepare1",
        "previous_budget_revision": 0, "budget_revision": 0, "budget_identity": DIGEST,
        "campaign_id": "campaign1", "campaign_identity": DIGEST,
        "campaign_template_identity": DIGEST, "domain_id": None,
        "output_root_path": None, "output_root_identity": None,
        "process_manifest_identity": None, "process_instance_identity": None,
        "observation_session_identity": DIGEST, "observation_session_nonce": "observe1",
        "observation_session_deadline_monotonic_ns": 1_800_000_000_000,
        "four_source_observation_identity": None, "precommit_receipt_identities": [],
        "precommit_observer_count": 0, "postcommit_observer_count": 0,
        "transaction_observer_count": 0, "lease_identity": None,
        "prepare_expires_monotonic_ns": 300_000_000_000,
        "committed_at_utc": None, "candidate_clear": None, "error_code": None,
    }


def budget():
    return {
        "schema_version": protocol.GLOBAL_ATTEMPT_BUDGET_SCHEMA, "revision": 0,
        "predecessor_identity": None, "state": "UNCONSUMED", "attempts_consumed": 0,
        "attempts_maximum": 1, "retries_authorized": 0,
        "authorization_identity": None, "process_start_commitment": None,
        "observation_session_identity": None, "four_source_observation_identity": None,
        "precommit_observer_count": 0, "precommit_receipt_identities": [],
        "postcommit_observer_count": 0, "postcommit_receipt_identity": None,
        "postcommit_four_source_observation_identity": None,
        "transaction_observer_count": 0,
        "updated_at_utc": "2026-08-22T00:00:00Z",
    }


def observation_session():
    return {
        "schema_version": protocol.OBSERVATION_SESSION_SCHEMA,
        "authorization_identity": DIGEST, "installed_runtime_identity": DIGEST,
        "process_manifest_identity": DIGEST, "environment_manifest_identity": DIGEST,
        "connection_identity": "connection1", "peer_uid": 103, "peer_gid": 104,
        "peer_pid": 1234, "peer_start_time_ticks": 55,
        "campaign_cgroup": "/system.slice/ctr-slice7g-campaign.service",
        "service_nonce": "observe1", "daemon_generation_identity": DIGEST,
        "created_monotonic_ns": 0,
        "deadline_monotonic_ns": 1_800_000_000_000,
        "domain_minimum": 100, "domain_maximum": 199,
        "maximum_precommit_observers": 100, "precommit_observer_count": 0,
        "postcommit_observer_count": 0, "transaction_observer_count": 0,
        "candidate_domains": [], "precommit_receipt_identities": [],
        "selected_domain": None, "lease_identity": None,
        "four_source_observation_identity": None, "state": "OPEN",
    }


def graph_receipt(phase="PRECOMMIT", domain=100, nodes=None):
    return {
        "schema_version": protocol.ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA,
        "session_binding_identity": DIGEST, "service_nonce": "observe1",
        "phase": phase, "phase_local_ordinal": 1,
        "transaction_observer_ordinal": 1,
        "four_source_observation_identity": OTHER_DIGEST,
        "observer_class": protocol.PRECOMMIT_ROS_GRAPH_OBSERVER_CLASS,
        "executable": protocol.PRECOMMIT_ROS_GRAPH_OBSERVER_EXECUTABLE,
        "executable_identity": DIGEST, "interpreter": "/usr/bin/python3",
        "interpreter_identity": DIGEST, "module_origin_identities": [DIGEST],
        "argv": [protocol.PRECOMMIT_ROS_GRAPH_OBSERVER_EXECUTABLE, "node", "list", "--no-daemon"],
        "environment_identity": DIGEST, "working_directory": "/opt/ctr-mppi/slice-7g/fixed",
        "cgroup": "/system.slice/ctr-slice7g-campaign.service", "shell": False,
        "domain_id": domain, "pid": 1234, "process_group_id": 1234,
        "process_start_time_ticks": 55,
        "started_monotonic_ns": 1, "ended_monotonic_ns": 2,
        "exit_status": 0, "terminating_signal": None,
        "stdout_size": 0, "stdout_sha256": hashlib.sha256(b"").hexdigest(),
        "stderr_size": 0, "stderr_sha256": hashlib.sha256(b"").hexdigest(),
        "nodes": [] if nodes is None else nodes, "parsed_node_set_identity": DIGEST,
        "cleanup_barrier_identity": DIGEST,
        "unexpected_descendants": 0, "ros_daemon_started": False,
    }


def four_source():
    return {
        "schema_version": protocol.FOUR_SOURCE_OBSERVATION_SCHEMA,
        "session_binding_identity": DIGEST, "service_nonce": "observe1",
        "phase": "PRECOMMIT", "phase_local_ordinal": 1,
        "transaction_observer_ordinal": 1, "domain_id": 100,
        "peer_process_identity": DIGEST, "observation_interval_identity": DIGEST,
        "cleanup_disposition_identity": DIGEST,
        "active_process_identity": DIGEST, "dds_port_identity": DIGEST,
        "global_lease_identity": DIGEST, "global_lease_registry_identity": DIGEST,
        "global_lease_revision_identity": DIGEST, "global_lease_state": "CLEAR",
        "global_lease_clear": True, "ros_graph_provider_identity": DIGEST,
        "all_sources_clear": True, "observed_monotonic_ns": 3,
    }


def global_lease_observation():
    return {
        "schema_version": protocol.GLOBAL_LEASE_OBSERVATION_SCHEMA,
        "registry_identity": DIGEST, "registry_revision_identity": "1" * 64,
        "domain_id": 100, "state": "CLEAR",
        "active_reservation_identities": [], "committed_binding_identities": [],
        "stale_invalid_identities": [], "clear": True, "observed_monotonic_ns": 1,
    }


def cleanup_guard():
    return {
        "schema_version": protocol.OBSERVER_CLEANUP_GUARD_SCHEMA,
        "revision": 0, "predecessor_identity": None, "state": "CLEARED",
        "authorization_identity": None, "budget_identity": None,
        "service_generation_identity": None, "session_binding_identity": None,
        "phase": None, "phase_local_ordinal": None,
        "transaction_observer_ordinal": None, "domain_id": None,
        "executable_identity": None, "argv_identity": None,
        "environment_identity": None, "pid": None,
        "process_start_time_ticks": None, "process_group_id": None,
        "session_id": None, "cgroup": None, "pidfd_identity": None,
        "disposition_identity": None, "recovery_authorization_identity": None,
        "updated_at_utc": "2026-08-22T00:00:00Z",
    }


def cleanup_recovery():
    return {
        "schema_version": protocol.OBSERVER_CLEANUP_RECOVERY_SCHEMA,
        "recovery_nonce": "recovery1", "quarantine_identity": DIGEST,
        "authority_root_identity": DIGEST,
        "runtime_authorization_identity": DIGEST, "budget_identity": DIGEST,
        "service_generation_identity": DIGEST,
        "issued_at_utc": "2026-08-22T00:00:00Z",
        "not_before_utc": "2026-08-22T00:00:00Z",
        "not_after_utc": "2026-08-22T01:00:00Z", "one_shot": True,
    }


def test_global_lease_and_four_source_clearance_are_closed_and_consistent():
    observed = protocol.validate_authority_record(
        global_lease_observation(), expected_schema=protocol.GLOBAL_LEASE_OBSERVATION_SCHEMA,
    )
    assert observed.data["clear"] is True
    invalid = four_source()
    invalid["global_lease_state"] = "RESERVED"
    invalid["global_lease_clear"] = False
    with pytest.raises(protocol.Slice7GAuthorityProtocolError, match="four_source_clear"):
        protocol.validate_authority_record(
            invalid, expected_schema=protocol.FOUR_SOURCE_OBSERVATION_SCHEMA,
        )


def test_public_request_rejects_caller_lease_identity_and_clearance():
    value = request()
    value["method"] = "record_precommit_observation"
    value["global_lease_identity"] = DIGEST
    value["global_lease_clear"] = True
    with pytest.raises(protocol.Slice7GAuthorityProtocolError, match="authority_fields"):
        protocol.validate_authority_record(value, expected_schema=protocol.AUTHORITY_REQUEST_SCHEMA)


def revocation():
    return {
        "schema_version": protocol.AUTHORITY_REVOCATION_SCHEMA, "revocation_id": "revoke1",
        "authorization_identity": DIGEST, "budget_revision": None,
        "state": "REQUESTED_PRECOMMIT", "requested_at_utc": "2026-08-22T00:00:00Z",
        "requested_by_uid": 0, "trigger_identity": None,
        "processed_trigger_identity": None, "termination_receipt_identity": None,
    }


def legacy_budget():
    value = budget()
    value["schema_version"] = protocol.LEGACY_GLOBAL_ATTEMPT_BUDGET_SCHEMA
    for field in (
        "observation_session_identity", "four_source_observation_identity",
        "precommit_observer_count", "precommit_receipt_identities",
        "postcommit_observer_count", "postcommit_receipt_identity",
        "postcommit_four_source_observation_identity", "transaction_observer_count",
    ):
        value.pop(field)
    return value


def legacy_request():
    value = request()
    value["schema_version"] = protocol.LEGACY_AUTHORITY_REQUEST_SCHEMA
    for field in ("observation_session_identity", "observation_session_nonce"):
        value.pop(field)
    return value


def legacy_receipt():
    value = receipt()
    value["schema_version"] = protocol.LEGACY_AUTHORITY_RECEIPT_SCHEMA
    for field in (
        "observation_session_identity", "observation_session_nonce",
        "observation_session_deadline_monotonic_ns", "four_source_observation_identity",
        "precommit_receipt_identities", "precommit_observer_count",
        "postcommit_observer_count", "transaction_observer_count", "lease_identity",
        "prepare_expires_monotonic_ns", "service_nonce", "candidate_clear",
    ):
        value.pop(field)
    return value


@pytest.mark.parametrize("schema,factory", [
    (protocol.AUTHORITY_BOOTSTRAP_SCHEMA, bootstrap),
    (protocol.INSTALLED_RUNTIME_MANIFEST_SCHEMA, installed_manifest),
    (protocol.BUILD_TEST_APPROVAL_SCHEMA, build_approval),
    (protocol.RUNTIME_AUTHORIZATION_SCHEMA, runtime_authorization),
    (protocol.PROCESS_MANIFEST_SCHEMA, process_manifest),
    (protocol.ENVIRONMENT_MANIFEST_SCHEMA, environment_manifest),
    (protocol.GLOBAL_ATTEMPT_BUDGET_SCHEMA, budget),
    (protocol.AUTHORITY_REQUEST_SCHEMA, request),
    (protocol.AUTHORITY_RECEIPT_SCHEMA, receipt),
    (protocol.AUTHORITY_REVOCATION_SCHEMA, revocation),
    (protocol.OBSERVATION_SESSION_SCHEMA, observation_session),
    (protocol.ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA, graph_receipt),
    (protocol.FOUR_SOURCE_OBSERVATION_SCHEMA, four_source),
    (protocol.GLOBAL_LEASE_OBSERVATION_SCHEMA, global_lease_observation),
    (protocol.OBSERVER_CLEANUP_GUARD_SCHEMA, cleanup_guard),
    (protocol.OBSERVER_CLEANUP_RECOVERY_SCHEMA, cleanup_recovery),
])
def test_every_closed_authority_schema_round_trips(schema, factory):
    value = factory()
    first = protocol.validate_authority_record(value, expected_schema=schema)
    second = protocol.validate_authority_record(first.canonical_bytes, expected_schema=schema)
    assert first.canonical_bytes == second.canonical_bytes
    assert first.logical_identity == second.logical_identity
    assert not first.canonical_bytes.endswith(b"\n")


@pytest.mark.parametrize("schema,factory", [
    (protocol.LEGACY_GLOBAL_ATTEMPT_BUDGET_SCHEMA, legacy_budget),
    (protocol.LEGACY_AUTHORITY_REQUEST_SCHEMA, legacy_request),
    (protocol.LEGACY_AUTHORITY_RECEIPT_SCHEMA, legacy_receipt),
])
def test_legacy_v1_records_are_structurally_parseable_but_not_operative(schema, factory):
    record = protocol.validate_authority_record(factory(), expected_schema=schema)
    assert record.schema_version.endswith("-1")
    with pytest.raises(protocol.Slice7GAuthorityProtocolError, match="authority_schema"):
        protocol.validate_authority_record(record.canonical_bytes, expected_schema={
            protocol.LEGACY_GLOBAL_ATTEMPT_BUDGET_SCHEMA: protocol.GLOBAL_ATTEMPT_BUDGET_SCHEMA,
            protocol.LEGACY_AUTHORITY_REQUEST_SCHEMA: protocol.AUTHORITY_REQUEST_SCHEMA,
            protocol.LEGACY_AUTHORITY_RECEIPT_SCHEMA: protocol.AUTHORITY_RECEIPT_SCHEMA,
        }[schema])


def test_unknown_fields_subclasses_and_noncanonical_bytes_are_rejected():
    class HostileDict(dict):
        pass

    with pytest.raises(protocol.Slice7GAuthorityProtocolError):
        protocol.validate_authority_record(HostileDict(request()), expected_schema=protocol.AUTHORITY_REQUEST_SCHEMA)
    extra = request()
    extra["execution_authorized"] = True
    with pytest.raises(protocol.Slice7GAuthorityProtocolError):
        protocol.validate_authority_record(extra, expected_schema=protocol.AUTHORITY_REQUEST_SCHEMA)
    raw = json.dumps(request(), sort_keys=True).encode() + b"\n"
    with pytest.raises(protocol.Slice7GAuthorityProtocolError):
        protocol.validate_authority_record(raw, expected_schema=protocol.AUTHORITY_REQUEST_SCHEMA)


def test_observation_session_and_counter_boundaries_are_exact():
    value = observation_session()
    value.update({
        "precommit_observer_count": 100,
        "transaction_observer_count": 100,
        "candidate_domains": list(range(100, 200)),
        "precommit_receipt_identities": [f"{index:064x}" for index in range(100)],
        "selected_domain": 199,
        "lease_identity": OTHER_DIGEST,
        "four_source_observation_identity": OTHER_DIGEST,
        "state": "OBSERVED",
    })
    protocol.validate_authority_record(value, expected_schema=protocol.OBSERVATION_SESSION_SCHEMA)

    for field, replacement in (
        ("precommit_observer_count", 101),
        ("postcommit_observer_count", 1),
        ("transaction_observer_count", 101),
        ("precommit_observer_count", True),
        ("precommit_observer_count", 100.0),
    ):
        invalid = dict(value)
        invalid[field] = replacement
        with pytest.raises(protocol.Slice7GAuthorityProtocolError):
            protocol.validate_authority_record(
                invalid, expected_schema=protocol.OBSERVATION_SESSION_SCHEMA,
            )


def test_observation_session_lifetime_is_exactly_1800_seconds():
    exact = observation_session()
    protocol.validate_authority_record(exact, expected_schema=protocol.OBSERVATION_SESSION_SCHEMA)
    for deadline in (1_799_999_999_999, 1_800_000_000_001):
        invalid = dict(exact)
        invalid["deadline_monotonic_ns"] = deadline
        with pytest.raises(protocol.Slice7GAuthorityProtocolError, match="observation_session_lifetime"):
            protocol.validate_authority_record(
                invalid, expected_schema=protocol.OBSERVATION_SESSION_SCHEMA,
            )


def test_public_request_rejects_caller_observation_authority_fields():
    for field, supplied in (
        ("ros_graph_observation_receipt", graph_receipt()),
        ("four_source_observation", four_source()),
        ("precommit_observer_count", 1),
        ("precommit_receipt_identities", [DIGEST]),
        ("lease_identity", DIGEST),
    ):
        invalid = request()
        invalid[field] = supplied
        with pytest.raises(protocol.Slice7GAuthorityProtocolError, match="authority_fields"):
            protocol.validate_authority_record(
                invalid, expected_schema=protocol.AUTHORITY_REQUEST_SCHEMA,
            )


@pytest.mark.parametrize("field", ["stdout_size", "stderr_size"])
def test_graph_observation_output_size_boundary(field):
    exact = graph_receipt()
    exact[field] = 1_048_576
    protocol.validate_authority_record(
        exact, expected_schema=protocol.ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA,
    )
    exact[field] += 1
    with pytest.raises(protocol.Slice7GAuthorityProtocolError, match="observer_output_size"):
        protocol.validate_authority_record(
            exact, expected_schema=protocol.ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA,
        )


def test_graph_observation_requires_exact_absolute_no_daemon_command():
    for field, value in (
        ("executable", "ros2"),
        ("argv", [protocol.PRECOMMIT_ROS_GRAPH_OBSERVER_EXECUTABLE, "node", "list"]),
        ("shell", True),
        ("ros_daemon_started", True),
        ("unexpected_descendants", 1),
    ):
        invalid = graph_receipt()
        invalid[field] = value
        with pytest.raises(protocol.Slice7GAuthorityProtocolError):
            protocol.validate_authority_record(
                invalid, expected_schema=protocol.ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA,
            )


def test_bounded_frame_rejects_truncation_trailing_and_oversize():
    frame = protocol.encode_authority_frame(request(), expected_schema=protocol.AUTHORITY_REQUEST_SCHEMA)
    assert protocol.decode_authority_frame(frame, expected_schema=protocol.AUTHORITY_REQUEST_SCHEMA).schema_version == protocol.AUTHORITY_REQUEST_SCHEMA
    for malformed in (frame[:-1], frame + b"x", (protocol.MAX_FRAME_BYTES + 1).to_bytes(4, "big")):
        with pytest.raises(protocol.Slice7GAuthorityProtocolError):
            protocol.decode_authority_frame(malformed, expected_schema=protocol.AUTHORITY_REQUEST_SCHEMA)


def test_public_client_rejects_hostile_mapping_before_invoking_mapping_hooks():
    class Hostile(dict):
        def get(self, *args, **kwargs):
            raise AssertionError("caller hook invoked")

    with pytest.raises(protocol.Slice7GAuthorityProtocolError, match="authority_request_type"):
        protocol.Slice7GAuthorityClient.exchange(Hostile())


def test_unix_peer_credentials_are_numeric_and_no_network_socket_is_accepted():
    left, right = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        observed = protocol.peer_credentials(left)
        assert observed.pid > 0
        assert observed.uid >= 0
        assert observed.gid >= 0
    finally:
        left.close()
        right.close()
    internet = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        with pytest.raises(protocol.Slice7GAuthorityProtocolError):
            protocol.peer_credentials(internet)
    finally:
        internet.close()
