import hashlib
import os
from pathlib import Path

import pytest

from ctr_evaluation.slice_7g_authority_protocol import (
    Slice7GPeerCredentials,
    Slice7GPeerProcess,
    authority_record_identity,
    ENVIRONMENT_MANIFEST_SCHEMA,
)
from ctr_evaluation.slice_7g_installed_runtime import (
    CAMPAIGN_CGROUP,
    EXPECTED_ROS_NODES,
    OwnedResourceRollback,
    RosNodeObservation,
    Slice7GInstalledRuntimeError,
    authenticate_ros_node_authority,
    inspect_installed_runtime_candidate,
    instantiate_closed_environment,
    make_environment_manifest,
    output_parent_acl_policy,
    render_and_verify_systemd_templates,
    validate_output_parent_acl_policy,
    validate_privileged_service_manifest,
)
from ctr_evaluation import slice_7g_privileged_protocol as privileged


def environment_manifest():
    fixed = {
        "PATH": "/usr/bin:/opt/ros/humble/bin",
        "PYTHONPATH": "/opt/ctr-mppi/slice-7g/fixed/lib/python3.10/site-packages",
        "AMENT_PREFIX_PATH": "/opt/ctr-mppi/slice-7g/fixed:/opt/ros/humble",
        "CMAKE_PREFIX_PATH": "/opt/ctr-mppi/slice-7g/fixed:/opt/ros/humble",
        "LD_LIBRARY_PATH": "/opt/ctr-mppi/slice-7g/fixed/lib:/opt/ros/humble/lib",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "ROS_HOME": "/run/ctr-slice7g-campaign/ros",
        "ROS_LOG_DIR": "/run/ctr-slice7g-campaign/ros/log",
        "ROS_LOCALHOST_ONLY": "1",
        "ROS_DISTRO": "humble",
        "HOME": "/run/ctr-slice7g-campaign/home",
        "XDG_CACHE_HOME": "/run/ctr-slice7g-campaign/cache",
        "MPLCONFIGDIR": "/run/ctr-slice7g-campaign/matplotlib",
        "PYTHONDONTWRITEBYTECODE": "1",
        "PYTHONNOUSERSITE": "1",
    }
    generated = {
        "ROS_DOMAIN_ID": "domain_id",
        "CTR_SLICE_7G_CAMPAIGN_ID": "campaign_id",
        "CTR_SLICE_7G_CELL_ID": "cell_id",
        "CTR_SLICE_7G_OUTPUT_ROOT": "output_root",
        "CTR_SLICE_7G_AUTHORIZATION_IDENTITY": "authorization_identity",
        "CTR_SLICE_7G_RECEIPT_IDENTITY": "receipt_identity",
        "CTR_SLICE_7G_CHARTER_IDENTITY": "charter_identity",
        "CTR_SLICE_7G_ATTEMPT_LEDGER_IDENTITY": "ledger_identity",
        "CTR_SLICE_7G_ATTEMPT_LEDGER_REVISION": "ledger_revision",
        "CTR_SLICE_7G_PROCESS_START_EVENT_IDENTITY": "process_start_identity",
        "CTR_SLICE_7G_CAMPAIGN_PLAN_IDENTITY": "plan_identity",
        "CTR_SLICE_7G_DOMAIN_LEASE_IDENTITY": "lease_identity",
        "CTR_SLICE_7G_DOMAIN_COMMITTED_BINDING_IDENTITY": "domain_binding_identity",
        "CTR_SLICE_7G_CAMPAIGN_OUTPUT_ROOT": "campaign_output_root",
        "CTR_SLICE_7G_CELL_OUTPUT_ROOT": "cell_output_root",
        "CTR_SLICE_7G_WORKING_DIRECTORY": "working_directory",
    }
    return make_environment_manifest(
        fixed_values=fixed,
        transaction_values=generated,
        required_absent_keys=[],
        path_keys=["PATH", "PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "LD_LIBRARY_PATH"],
    )


def peer(node):
    pid = 1000 + sorted(EXPECTED_ROS_NODES | {"/unexpected"}).index(node)
    process = Slice7GPeerProcess(
        Slice7GPeerCredentials(pid, 200, 300), 10,
        "/opt/ctr-mppi/slice-7g/fixed/bin/node", ("node",), (),
        "/opt/ctr-mppi/slice-7g/fixed", CAMPAIGN_CGROUP,
    )
    publishers = ("/ctr/safe_command",) if node == "/safety_supervisor" else ()
    subscribers = (
        ("/ctr/mppi_command",) if node == "/safety_supervisor" else
        (("/ctr/safe_command",) if node == "/ctr_simulator" else ())
    )
    return RosNodeObservation(node, process, process, True, True, publishers, subscribers)


def test_path_independent_tree_identity_is_stable_and_rejects_aliases(tmp_path):
    first = tmp_path / "first"
    second = tmp_path / "second"
    for root in (first, second):
        (root / "lib").mkdir(parents=True)
        (root / "lib" / "module.py").write_text("value = 1\n", encoding="utf-8")
        os.chmod(root / "lib" / "module.py", 0o444)
    one = inspect_installed_runtime_candidate(str(first))
    two = inspect_installed_runtime_candidate(str(second))
    assert one.logical_identity == two.logical_identity
    assert str(first) not in one.canonical_projection.decode()
    alias_root = tmp_path / "alias"
    alias_root.mkdir()
    (alias_root / "a").write_bytes(b"x")
    os.link(alias_root / "a", alias_root / "b")
    with pytest.raises(Slice7GInstalledRuntimeError, match="installed_hardlink|installed_inode_alias"):
        inspect_installed_runtime_candidate(str(alias_root))


def test_environment_is_closed_and_path_poisoning_is_rejected():
    manifest = environment_manifest()
    transaction = {
        "domain_id": "100", "campaign_id": "campaign1", "cell_id": "cell1",
        "output_root": "/home/ankid/ctr_mppi_evidence/slice_7g/campaign1",
        "authorization_identity": "0" * 64, "receipt_identity": "1" * 64,
        "charter_identity": "2" * 64, "ledger_identity": "3" * 64,
        "ledger_revision": "2", "process_start_identity": "4" * 64,
        "plan_identity": "5" * 64, "lease_identity": "6" * 64,
        "domain_binding_identity": "7" * 64,
        "campaign_output_root": "/home/ankid/ctr_mppi_evidence/slice_7g/campaign1",
        "cell_output_root": "/home/ankid/ctr_mppi_evidence/slice_7g/campaign1/cells/cell1",
        "working_directory": "/opt/ctr-mppi/slice-7g/fixed",
    }
    observed = instantiate_closed_environment(manifest, transaction)
    assert "PYTHONINSPECT" not in observed
    assert observed["ROS_DOMAIN_ID"] == "100"
    def thaw(value):
        if hasattr(value, "items"):
            return {key: thaw(member) for key, member in value.items()}
        if type(value) is tuple:
            return [thaw(member) for member in value]
        return value

    hostile = thaw(manifest)
    hostile["fixed_values"]["PATH"] = ":/usr/bin"
    with pytest.raises(Slice7GInstalledRuntimeError):
        instantiate_closed_environment(hostile, transaction)


def test_acl_policy_is_deterministic_and_forbids_parent_mutation():
    first = output_parent_acl_policy()
    second = output_parent_acl_policy()
    assert first.canonical_bytes == second.canonical_bytes
    assert first.logical_identity == "e66b7103b47263c91f94a79db381fdeafbb96439f008c4ab8d7f0b8845ca12fb"
    assert first.data["campaign_parent_create_remove_list"] is False
    validate_output_parent_acl_policy(dict(first.data))
    hostile = dict(first.data)
    hostile["campaign_parent_create_remove_list"] = True
    with pytest.raises(Slice7GInstalledRuntimeError):
        validate_output_parent_acl_policy(hostile)


def test_systemd_templates_render_closed_campaign_cgroup():
    resource = Path(__file__).parents[1] / "resource" / "systemd"
    templates = {path.name: path.read_text(encoding="utf-8") for path in resource.glob("*.in")}
    root = "/opt/ctr-mppi/slice-7g/" + "a" * 64
    rendered = render_and_verify_systemd_templates(
        templates,
        installed_runtime_root=root,
        working_directory=root,
        environment_values={
            "PATH": "/usr/bin:/opt/ros/humble/bin",
            "PYTHONPATH": root + "/lib/python3.10/site-packages",
            "AMENT_PREFIX_PATH": root + ":/opt/ros/humble",
            "CMAKE_PREFIX_PATH": root + ":/opt/ros/humble",
            "LD_LIBRARY_PATH": root + "/lib:/opt/ros/humble/lib",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "ROS_HOME": "/run/ctr-slice7g-campaign/ros",
            "ROS_LOG_DIR": "/run/ctr-slice7g-campaign/ros/log",
            "ROS_LOCALHOST_ONLY": "1",
        },
        timeouts={"sigint_seconds": 2.0, "sigterm_seconds": 3.0, "sigkill_seconds": 1.0, "cell_seconds": 25.0},
    )
    campaign = rendered["ctr-slice7g-campaign.service"]
    assert "KillMode=control-group" in campaign
    assert "Delegate=no" in campaign
    assert "TimeoutStopSec=6s" in campaign
    assert "EnvironmentFile=" not in campaign
    assert "systemctl" not in campaign


def test_privileged_service_manifest_binds_both_helpers_and_units():
    value = {
        "schema_version": privileged.PRIVILEGED_SERVICE_MANIFEST_SCHEMA,
        "cleanup_service": privileged.CLEANUP_AUTHORITY_EXECUTABLE,
        "observer_service": privileged.OBSERVER_SUPERVISOR_EXECUTABLE,
        "cleanup_state_root": privileged.CLEANUP_AUTHORITY_STATE_ROOT,
        "cleanup_socket": privileged.CLEANUP_AUTHORITY_SOCKET,
        "recovery_socket": privileged.CLEANUP_RECOVERY_SOCKET,
        "observer_socket": privileged.OBSERVER_SUPERVISOR_SOCKET,
        "cleanup_principal": "root", "observer_supervisor_principal": "root",
        "observer_principal": privileged.OBSERVER_ACCOUNT,
        "recovery_principal": privileged.RECOVERY_ACCOUNT,
        "supervisor_cgroup": privileged.OBSERVER_SUPERVISOR_CGROUP,
        "observer_leaf_grammar": privileged.OBSERVER_LEAF_PATTERN.pattern,
        "observer_executable": privileged.OBSERVER_EXECUTABLE,
        "observer_argv": list(privileged.OBSERVER_ARGV),
        "environment_manifest_identity": "a" * 64,
        "working_directory": "/opt/ctr-mppi/slice-7g/" + "b" * 64,
        "protocol_schema": privileged.PRIVILEGED_REQUEST_SCHEMA,
        "containment_receipt_schema": privileged.OBSERVER_CONTAINMENT_RECEIPT_SCHEMA,
        "cleanup_schemas": [
            privileged.CLEANUP_REVISION_SCHEMA,
            privileged.CLEANUP_ANCHOR_SCHEMA,
            privileged.CLEANUP_HEAD_SCHEMA,
        ],
        "service_executable_identities": ["c" * 64, "d" * 64],
        "systemd_unit_identities": ["e" * 64, "f" * 64],
        "numeric_ids_provisioned": False,
    }
    observed = validate_privileged_service_manifest(value)
    assert observed["numeric_ids_provisioned"] is False
    assert observed["observer_argv"] == privileged.OBSERVER_ARGV


def test_exact_seven_node_and_supervisor_child_authority():
    valid = [peer(name) for name in sorted(EXPECTED_ROS_NODES)]
    assert len(authenticate_ros_node_authority(valid)) == 7
    for missing in EXPECTED_ROS_NODES:
        with pytest.raises(Slice7GInstalledRuntimeError, match="ros_node_set"):
            authenticate_ros_node_authority([item for item in valid if item.node_name != missing])
    with pytest.raises(Slice7GInstalledRuntimeError, match="ros_node_set"):
        authenticate_ros_node_authority(valid + [peer("/unexpected")])
    supervisor = next(index for index, item in enumerate(valid) if item.node_name == "/safety_supervisor")
    bad = list(valid)
    original = bad[supervisor]
    bad[supervisor] = RosNodeObservation(
        original.node_name, original.process, original.reconciled_process, False, True,
        original.publishers, original.subscribers,
    )
    with pytest.raises(Slice7GInstalledRuntimeError, match="safety_supervisor_state"):
        authenticate_ros_node_authority(bad)
    replacement = list(valid)
    original = replacement[supervisor]
    changed = Slice7GPeerProcess(
        original.process.credentials, original.process.start_time_ticks + 1,
        original.process.executable, original.process.argv, original.process.environment,
        original.process.working_directory, original.process.cgroup,
    )
    replacement[supervisor] = RosNodeObservation(
        original.node_name, original.process, changed, True, True,
        original.publishers, original.subscribers,
    )
    with pytest.raises(Slice7GInstalledRuntimeError, match="ros_node_process_replaced"):
        authenticate_ros_node_authority(replacement)


def test_baseexception_rollback_attempts_all_steps_and_preserves_primary():
    observed = []
    rollback = OwnedResourceRollback()
    rollback.own("first", lambda: observed.append("first"))

    def fail():
        observed.append("second")
        raise RuntimeError("cleanup")

    rollback.own("second", fail)
    primary = KeyboardInterrupt()
    with pytest.raises(KeyboardInterrupt) as captured:
        rollback.rollback(primary)
    assert captured.value is primary
    assert observed == ["second", "first"]
    assert rollback.rollback() == ()
