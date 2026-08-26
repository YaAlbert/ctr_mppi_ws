from __future__ import annotations

import hashlib
import json
import os
import socket
import copy

import pytest

from ctr_evaluation import slice_7g_privileged_protocol as protocol


HEX = "a" * 64
NONCE = "b" * 32


def _request(**updates):
    value = {
        "schema_version": protocol.PRIVILEGED_REQUEST_SCHEMA,
        "operation": "CLEANUP_STATE_QUERY",
        "sequence": 0,
        "connection_nonce": NONCE,
        "request_nonce": "c" * 32,
        "operation_token": "d" * 32,
        "service_generation_identity": HEX,
        "runtime_authorization_identity": HEX,
        "installed_runtime_identity": HEX,
        "budget_identity": HEX,
        "cleanup_head_identity": HEX,
        "session_binding_identity": HEX,
        "domain_id": 100,
        "phase": "PRECOMMIT",
        "phase_local_ordinal": 1,
        "transaction_observer_ordinal": 1,
        "transition": None,
        "observer_contract_identity": None,
        "containment_identity": None,
        "process_identity": None,
        "disposition_identity": None,
        "recovery_authorization_identity": None,
    }
    value.update(updates)
    return value


def _lease(state="CLEAR", clear=True):
    return {
        "schema_version": protocol.GLOBAL_LEASE_OBSERVATION_V2_SCHEMA,
        "registry_identity": HEX,
        "registry_revision_identity": "b" * 64,
        "physical_observation_identity": "c" * 64,
        "record_physical_identities": [],
        "domain_id": 100,
        "state": state,
        "owner_bindings": [],
        "output_root_bindings": [],
        "active_reservation_identities": [],
        "committed_binding_identities": [],
        "stale_invalid_identities": [],
        "clear": clear,
        "session_binding_identity": "d" * 64,
        "service_nonce": NONCE,
        "phase": "PRECOMMIT",
        "phase_local_ordinal": 1,
        "transaction_observer_ordinal": 1,
        "observation_interval_identity": "e" * 64,
        "observed_monotonic_ns": 1,
    }


def _trusted_file(path, inode, mode=0o444, *, relative=False):
    return {
        "device": 1, "inode": inode, "link_count": 1, "mode": mode,
        "owner_gid": 0, "owner_uid": 0, "path": path,
        "sha256": f"{inode:064x}"[-64:], "size": 10, "type": "regular",
    }


def _bootstrap_v3():
    root = protocol.INSTALLED_RUNTIME_PARENT + "/" + "1" * 64
    member_paths = [
        "lib/python3.10/site-packages/ctr_evaluation/__init__.py",
        "lib/python3.10/site-packages/ctr_evaluation/slice_7g_cleanup_authority.py",
        "lib/python3.10/site-packages/ctr_evaluation/slice_7g_observer_supervisor.py",
        "lib/python3.10/site-packages/ctr_evaluation/slice_7g_privileged_protocol.py",
    ]
    fixed_keys = {
        "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "HOME", "LD_LIBRARY_PATH",
        "MPLCONFIGDIR", "PATH", "PYTHONDONTWRITEBYTECODE", "PYTHONNOUSERSITE",
        "PYTHONPATH", "RMW_IMPLEMENTATION", "ROS_DISTRO", "ROS_HOME",
        "ROS_LOCALHOST_ONLY", "ROS_LOG_DIR", "XDG_CACHE_HOME",
    }
    return {
        "schema_version": protocol.AUTHORITY_BOOTSTRAP_V3_SCHEMA,
        "authority_uid": 101, "authority_gid": 102, "campaign_uid": 103,
        "runtime_gid": 104, "observer_uid": 105, "observer_gid": 106,
        "recovery_uid": 107, "recovery_gid": 108,
        "authority_account": protocol.AUTHORITY_ACCOUNT,
        "campaign_account": protocol.CAMPAIGN_ACCOUNT,
        "runtime_group": protocol.RUNTIME_GROUP,
        "observer_account": protocol.OBSERVER_ACCOUNT,
        "recovery_account": protocol.RECOVERY_ACCOUNT,
        "bootstrap_path": protocol.AUTHORITY_BOOTSTRAP_PATH,
        "authority_service_path": protocol.AUTHORITY_EXECUTABLE,
        "authority_state_root": protocol.AUTHORITY_STATE_ROOT,
        "authority_socket_path": protocol.AUTHORITY_SOCKET,
        "installed_runtime_parent": protocol.INSTALLED_RUNTIME_PARENT,
        "cleanup_service_path": protocol.CLEANUP_AUTHORITY_EXECUTABLE,
        "cleanup_state_root": protocol.CLEANUP_AUTHORITY_STATE_ROOT,
        "cleanup_socket_path": protocol.CLEANUP_AUTHORITY_SOCKET,
        "recovery_socket_path": protocol.CLEANUP_RECOVERY_SOCKET,
        "observer_service_path": protocol.OBSERVER_SUPERVISOR_EXECUTABLE,
        "observer_socket_path": protocol.OBSERVER_SUPERVISOR_SOCKET,
        "service_executables": [
            _trusted_file(protocol.AUTHORITY_EXECUTABLE, 1, 0o555),
            _trusted_file(protocol.CLEANUP_AUTHORITY_EXECUTABLE, 2, 0o555),
            _trusted_file(protocol.OBSERVER_SUPERVISOR_EXECUTABLE, 3, 0o555),
        ],
        "privileged_code": {
            "installed_root": root,
            "members": [
                _trusted_file(path, 10 + index, relative=True)
                for index, path in enumerate(member_paths)
            ],
            "observer_contract": {
                "argv": [protocol.OBSERVER_EXECUTABLE, *protocol.OBSERVER_ARGV],
                "environment": {
                    "dynamic_key": "ROS_DOMAIN_ID", "dynamic_minimum": 100,
                    "dynamic_maximum": 199,
                    "fixed_values": {key: "/fixed/" + key.lower() for key in sorted(fixed_keys)},
                },
                "executable": _trusted_file(protocol.OBSERVER_EXECUTABLE, 20, 0o555),
                "interpreter": _trusted_file("/usr/bin/python3.10", 21, 0o555),
                "working_directory": root,
            },
        },
        "record_paths": {
            "build_test_approval": protocol.AUTHORITY_STATE_ROOT + "/public/build.json",
            "environment_manifest": protocol.AUTHORITY_STATE_ROOT + "/public/environment.json",
            "global_budget": protocol.AUTHORITY_STATE_ROOT + "/global-budget/revision-00000000000000000000.json",
            "installed_runtime_manifest": protocol.AUTHORITY_STATE_ROOT + "/public/installed.json",
            "privileged_service_manifest": protocol.AUTHORITY_STATE_ROOT + "/public/privileged-services.json",
            "process_manifest": protocol.AUTHORITY_STATE_ROOT + "/public/process.json",
            "runtime_authorization": protocol.AUTHORITY_STATE_ROOT + "/public/authorization.json",
        },
        "schemas": {
            "build_test_approval": "ctr-slice-7g-isolated-build-test-approval-1",
            "environment_manifest": "ctr-slice-7g-environment-manifest-1",
            "global_budget": protocol.GLOBAL_ATTEMPT_BUDGET_V4_SCHEMA,
            "installed_runtime_manifest": protocol.INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA,
            "privileged_service_manifest": protocol.PRIVILEGED_SERVICE_MANIFEST_SCHEMA,
            "process_manifest": protocol.PROCESS_MANIFEST_V2_SCHEMA,
            "runtime_authorization": protocol.RUNTIME_AUTHORIZATION_V3_SCHEMA,
        },
        "protocol_limits": {
            "maximum_connections": 8, "maximum_frame_bytes": 262_144,
            "maximum_frames_per_connection": 128, "maximum_list_items": 65_536,
            "maximum_record_depth": 24, "maximum_transferred_fds": 2,
        },
        "systemd_units": {
            "authority": "ctr-slice7g-authority.service",
            "campaign": "ctr-slice7g-campaign.service",
            "cleanup_authority": protocol.CLEANUP_AUTHORITY_SERVICE,
            "observer_supervisor": protocol.OBSERVER_SUPERVISOR_SERVICE,
            "revocation_path": "ctr-slice7g-revocation.path",
            "revocation_service": "ctr-slice7g-revocation.service",
        },
    }


def _installed_v3():
    identity = "1" * 64
    return {
        "schema_version": protocol.INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA,
        "identity_algorithm": "sha256:ctr-slice-7g-installed-runtime-tree-canonical-1",
        "installed_runtime_identity": identity,
        "root_path": protocol.INSTALLED_RUNTIME_PARENT + "/" + identity,
        "root_device": 1, "root_inode": 2, "physical_tree_identity": "2" * 64,
        "member_count": 1,
        "members": [{
            "path": "bin/tool", "type": "regular", "mode": 0o555,
            "owner_uid": 0, "owner_gid": 0, "link_count": 1,
            "size": 1, "sha256": "3" * 64,
        }],
        "console_entrypoints": [], "python_modules": [],
        "generated_interfaces": [], "elf_members": [],
        "process_manifest_identity": "4" * 64,
        "environment_manifest_identity": "5" * 64,
        "source_snapshot": {
            "schema_version": "ctr-slice-7g-post-implementation-source-snapshot-3",
            "path": "/var/lib/ctr-mppi/snapshot.json", "physical_sha256": "6" * 64,
            "logical_identity": "7" * 64,
            "logical_identity_algorithm": "sha256:ctr-slice-7g-post-implementation-source-snapshot-canonical-3",
            "member_count": 1, "mode_bound": True,
        },
        "build_test_approval_identity": "8" * 64,
        "privileged_service_manifest_identity": "9" * 64,
    }


def test_all_twenty_one_v7_schemas_are_closed_and_versioned():
    # Keep the historical node id stable for canonical collection accounting:
    # v7 now has 22 closed schema versions across the original 21 families
    # because installed-runtime v2 remains inspection-only beside authority v3.
    assert len(protocol.ALL_V7_SCHEMAS) == 22
    assert len(set(protocol.ALL_V7_SCHEMAS)) == 22
    assert all(item.startswith("ctr-slice-7g-") for item in protocol.ALL_V7_SCHEMAS)


def test_privileged_identity_uses_schema_domain_and_canonical_bytes():
    value = _request()
    record = protocol.validate_record(value, expected_schema=protocol.PRIVILEGED_REQUEST_SCHEMA)
    expected = hashlib.sha256(
        (protocol.PRIVILEGED_REQUEST_SCHEMA + ":canonical-1").encode()
        + b"\0" + record.canonical_bytes
    ).hexdigest()
    assert record.logical_identity == expected
    assert record.canonical_bytes == json.dumps(
        value, sort_keys=True, ensure_ascii=False, allow_nan=False,
        separators=(",", ":"),
    ).encode()


@pytest.mark.parametrize("state", protocol.LEASE_STATES)
def test_exact_six_state_lease_model(state):
    record = protocol.validate_record(
        _lease(state, state == "CLEAR"),
        expected_schema=protocol.GLOBAL_LEASE_OBSERVATION_V2_SCHEMA,
    )
    assert record.data["clear"] is (state == "CLEAR")


@pytest.mark.parametrize("state", ["", "clear", "UNKNOWN", "STALE"])
def test_unknown_lease_state_is_rejected(state):
    with pytest.raises(protocol.Slice7GPrivilegedProtocolError):
        protocol.validate_record(
            _lease(state, False),
            expected_schema=protocol.GLOBAL_LEASE_OBSERVATION_V2_SCHEMA,
        )


def test_non_clear_lease_cannot_claim_clear():
    with pytest.raises(protocol.Slice7GPrivilegedProtocolError) as caught:
        protocol.validate_record(
            _lease("CONFLICTING", True),
            expected_schema=protocol.GLOBAL_LEASE_OBSERVATION_V2_SCHEMA,
        )
    assert caught.value.code == "lease_clear"


@pytest.mark.parametrize(
    "field,value",
    [
        ("executable", "/bin/sh"),
        ("argv", ["sh"]),
        ("environment", {"PATH": "/tmp"}),
        ("signal", 9),
        ("pid", 123),
        ("cgroup", "/other"),
        ("lease_clear", True),
        ("recovery_clear", True),
    ],
)
def test_public_request_rejects_arbitrary_authority_fields(field, value):
    request = _request()
    request[field] = value
    with pytest.raises(protocol.Slice7GPrivilegedProtocolError) as caught:
        protocol.validate_record(request, expected_schema=protocol.PRIVILEGED_REQUEST_SCHEMA)
    assert caught.value.code == "closed_schema"


def test_exact_dict_boundary_rejects_subclass():
    class Hostile(dict):
        pass

    with pytest.raises(protocol.Slice7GPrivilegedProtocolError):
        protocol.validate_record(Hostile(_request()), expected_schema=protocol.PRIVILEGED_REQUEST_SCHEMA)


@pytest.mark.parametrize("sequence", [True, 1.0, -1, 128])
def test_sequence_exact_range(sequence):
    with pytest.raises(protocol.Slice7GPrivilegedProtocolError):
        protocol.validate_record(
            _request(sequence=sequence), expected_schema=protocol.PRIVILEGED_REQUEST_SCHEMA,
        )


def test_seqpacket_round_trip_and_peer_credentials():
    left, right = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    try:
        protocol.send_packet(left, _request(), expected_schema=protocol.PRIVILEGED_REQUEST_SCHEMA)
        record, descriptors = protocol.receive_packet(
            right, expected_schema=protocol.PRIVILEGED_REQUEST_SCHEMA,
        )
        assert record.data["operation"] == "CLEANUP_STATE_QUERY"
        assert descriptors == ()
        credentials = protocol.peer_credentials(right)
        assert (credentials.uid, credentials.gid, credentials.pid) == (
            os.geteuid(), os.getegid(), os.getpid(),
        )
    finally:
        left.close()
        right.close()


@pytest.mark.parametrize(
    "packet,code",
    [
        (b"", "frame_header"),
        (b"\x00\x00\x00", "frame_header"),
        (b"\x00\x00\x00\x02{}x", "frame_length"),
        ((262145).to_bytes(4, "big") + b"{}", "frame_size"),
    ],
)
def test_malformed_packets_fail_closed(packet, code):
    with pytest.raises(protocol.Slice7GPrivilegedProtocolError) as caught:
        protocol.decode_packet(packet, expected_schema=protocol.PRIVILEGED_REQUEST_SCHEMA)
    assert caught.value.code == code


def test_noncanonical_and_duplicate_json_are_rejected():
    canonical = protocol.canonical_bytes(
        _request(), expected_schema=protocol.PRIVILEGED_REQUEST_SCHEMA,
    )
    with pytest.raises(protocol.Slice7GPrivilegedProtocolError):
        protocol.validate_record(b" " + canonical, expected_schema=protocol.PRIVILEGED_REQUEST_SCHEMA)
    duplicate = b'{"schema_version":"x","schema_version":"x"}'
    with pytest.raises(protocol.Slice7GPrivilegedProtocolError):
        protocol.validate_record(duplicate, expected_schema=protocol.PRIVILEGED_REQUEST_SCHEMA)


def test_sealed_memfd_round_trip_and_substitution_rejection():
    payload = b"bounded output\n"
    descriptor = protocol.make_sealed_memfd("slice7g-test", payload)
    try:
        assert protocol.authenticate_sealed_output(
            descriptor, expected_size=len(payload),
            expected_sha256=hashlib.sha256(payload).hexdigest(),
        ) == payload
        with pytest.raises(protocol.Slice7GPrivilegedProtocolError):
            protocol.authenticate_sealed_output(
                descriptor, expected_size=len(payload), expected_sha256="0" * 64,
            )
    finally:
        os.close(descriptor)


def test_unsealed_regular_descriptor_is_rejected(tmp_path):
    path = tmp_path / "not-memfd"
    path.write_bytes(b"x")
    descriptor = os.open(path, os.O_RDONLY)
    try:
        with pytest.raises(protocol.Slice7GPrivilegedProtocolError):
            protocol.authenticate_sealed_output(
                descriptor, expected_size=1,
                expected_sha256=hashlib.sha256(b"x").hexdigest(),
            )
    finally:
        os.close(descriptor)


def test_root_bootstrap_v3_binds_complete_privileged_code_and_services():
    record = protocol.validate_record(
        _bootstrap_v3(), expected_schema=protocol.AUTHORITY_BOOTSTRAP_V3_SCHEMA,
    )
    assert len(record.data["service_executables"]) == 3
    assert len(record.data["privileged_code"]["members"]) == 4
    assert record.data["privileged_code"]["installed_root"].startswith(
        protocol.INSTALLED_RUNTIME_PARENT + "/"
    )


@pytest.mark.parametrize(
    "defect",
    [
        "boolean_integer", "unknown_nested", "missing_nested", "mapping_subclass",
        "malformed_array", "duplicate_path", "reordered_collection", "invalid_mode",
        "attacker_environment_mapping",
    ],
)
def test_root_bootstrap_v3_nested_boundaries_fail_closed(defect):
    value = _bootstrap_v3()
    if defect == "boolean_integer":
        value["protocol_limits"]["maximum_connections"] = True
    elif defect == "unknown_nested":
        value["record_paths"]["attacker"] = "/tmp/attacker"
    elif defect == "missing_nested":
        del value["schemas"]["process_manifest"]
    elif defect == "mapping_subclass":
        class Hostile(dict):
            pass
        value["privileged_code"] = Hostile(value["privileged_code"])
    elif defect == "malformed_array":
        value["service_executables"] = {"path": protocol.AUTHORITY_EXECUTABLE}
    elif defect == "duplicate_path":
        value["privileged_code"]["members"][1]["path"] = value["privileged_code"]["members"][0]["path"]
    elif defect == "reordered_collection":
        value["privileged_code"]["members"].reverse()
    elif defect == "invalid_mode":
        value["service_executables"][0]["mode"] = 0o755
    else:
        class Hostile(dict):
            pass
        environment = value["privileged_code"]["observer_contract"]["environment"]
        environment["fixed_values"] = Hostile(environment["fixed_values"])
    with pytest.raises(protocol.Slice7GPrivilegedProtocolError):
        protocol.validate_record(value, expected_schema=protocol.AUTHORITY_BOOTSTRAP_V3_SCHEMA)


@pytest.mark.parametrize(
    "defect", ["boolean_mode", "unknown_member", "duplicate_member", "reordered_members"],
)
def test_installed_runtime_v3_nested_member_boundaries_fail_closed(defect):
    value = _installed_v3()
    if defect == "boolean_mode":
        value["members"][0]["mode"] = True
    elif defect == "unknown_member":
        value["members"][0]["attacker"] = True
    else:
        other = dict(value["members"][0])
        other["path"] = "lib/other"
        other["sha256"] = "a" * 64
        value["members"].append(other)
        value["member_count"] = 2
        if defect == "duplicate_member":
            value["members"][1]["path"] = value["members"][0]["path"]
        else:
            value["members"].reverse()
    with pytest.raises(protocol.Slice7GPrivilegedProtocolError):
        protocol.validate_record(
            value, expected_schema=protocol.INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA,
        )


def _response(request, generation, **updates):
    value = {
        "schema_version": protocol.PRIVILEGED_RECEIPT_SCHEMA,
        "operation": request["operation"], "sequence": request["sequence"],
        "connection_nonce": request["connection_nonce"],
        "request_nonce": request["request_nonce"],
        "operation_token": request["operation_token"],
        "service_generation_identity": generation, "result": "OK",
        "error_code": None, "cleanup_head_identity": None,
        "containment_receipt_identity": None, "output_descriptor_count": 0,
        "payload_identity": None, "cleanup_revision": None,
        "cleanup_anchor": None, "cleanup_head": None, "containment_receipt": None,
    }
    value.update(updates)
    return value


@pytest.mark.parametrize(
    "field,replacement",
    [
        ("operation", "OBSERVE_START"), ("sequence", 1),
        ("connection_nonce", "e" * 32), ("request_nonce", "f" * 32),
        ("operation_token", "g" * 32), ("service_generation_identity", "f" * 64),
    ],
)
def test_privileged_response_exact_request_bindings_are_required(field, replacement):
    request_value = _request()
    generation = hashlib.sha256((field + "-generation").encode()).hexdigest()
    response_value = _response(request_value, generation)
    response_value[field] = replacement
    request = protocol.validate_record(
        request_value, expected_schema=protocol.PRIVILEGED_REQUEST_SCHEMA,
    )
    response = protocol.validate_record(
        response_value, expected_schema=protocol.PRIVILEGED_RECEIPT_SCHEMA,
    )
    with pytest.raises(protocol.Slice7GPrivilegedProtocolError):
        protocol.verify_response_binding(
            request, response, expected_service_generation_identity=generation,
            expected_descriptor_count=0, descriptors=(),
        )


def test_completed_and_cross_connection_operation_token_replay_is_rejected():
    generation = hashlib.sha256(b"completed-operation-replay").hexdigest()
    first_value = _request(
        operation_token="replay-token", request_nonce="first-request",
        service_generation_identity=generation,
    )
    first = protocol.validate_record(first_value, expected_schema=protocol.PRIVILEGED_REQUEST_SCHEMA)
    first_response = protocol.validate_record(
        _response(first_value, generation), expected_schema=protocol.PRIVILEGED_RECEIPT_SCHEMA,
    )
    protocol.verify_response_binding(
        first, first_response, expected_service_generation_identity=generation,
        expected_descriptor_count=0, descriptors=(),
    )
    second_value = _request(
        connection_nonce="other-connection", request_nonce="second-request",
        operation_token="replay-token", service_generation_identity=generation,
    )
    second = protocol.validate_record(second_value, expected_schema=protocol.PRIVILEGED_REQUEST_SCHEMA)
    second_response = protocol.validate_record(
        _response(second_value, generation), expected_schema=protocol.PRIVILEGED_RECEIPT_SCHEMA,
    )
    with pytest.raises(protocol.Slice7GPrivilegedProtocolError) as caught:
        protocol.verify_response_binding(
            second, second_response, expected_service_generation_identity=generation,
            expected_descriptor_count=0, descriptors=(),
        )
    assert caught.value.code == "replay"


def test_peer_credential_syscall_failure_is_publicly_normalized(monkeypatch):
    left, right = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    try:
        monkeypatch.setattr(
            protocol, "_socket_peercred",
            lambda channel: (_ for _ in ()).throw(PermissionError("denied")),
        )
        with pytest.raises(protocol.Slice7GPrivilegedProtocolError) as caught:
            protocol.peer_credentials(right)
        assert caught.value.code == "peer_credentials"
        assert "PermissionError" in str(caught.value)
    finally:
        left.close()
        right.close()
