import ast
import fcntl
import hashlib
import inspect
import json
import os
from pathlib import Path
import subprocess
import threading
import time
from types import SimpleNamespace

import pytest

from ctr_evaluation import slice_7g_authority_daemon as daemon
from ctr_evaluation.slice_7g_authority_daemon import (
    DaemonObservationEvidence,
    GlobalAttemptBudgetStore,
    GlobalLeaseStateObserver,
    ObserverCleanupGuardStore,
    ProvisionalAllocation,
    RuntimeAuthorityStateMachine,
    Slice7GAuthorityDaemonError,
    _cleanup_server_observer_group,
    _enforce_pending_revocations_for_test,
    _run_server_owned_graph_observer,
    _server_group_members,
)
from ctr_evaluation import slice_7g_authority_protocol as protocol


TIMESTAMP = "2026-08-22T00:00:00Z"
DIGEST = "a" * 64


def test_authority_daemon_dependency_surface_is_standard_library_plus_local_modules():
    source = (
        Path(__file__).parents[1]
        / "ctr_evaluation"
        / "slice_7g_authority_daemon.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)
    absolute_roots = set()
    relative_imports = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            absolute_roots.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                relative_imports.add(node.module)
            elif node.module:
                absolute_roots.add(node.module.split(".")[0])
    assert absolute_roots <= {
            "__future__", "ctypes", "dataclasses", "datetime", "fcntl", "hashlib", "json",
            "os", "pathlib", "re", "selectors", "secrets", "signal", "socket", "stat",
            "struct", "subprocess", "time", "types", "typing", "unicodedata",
    }
    assert relative_imports == {
        "slice_7g_authority_protocol", "slice_7g_cleanup_authority",
        "slice_7g_installed_runtime", "slice_7g_observer_supervisor",
        "slice_7g_privileged_protocol",
    }


def test_v7_production_daemon_uses_privileged_clients_not_local_observer_launch():
    source = inspect.getsource(daemon.Slice7GAuthorityDaemon._observe_domain)
    assert "self.observer_client.observe(" in source
    assert "self.cleanup_guard.require_clear()" in source
    assert "_run_server_owned_graph_observer(" not in source
    assert "subprocess.Popen" not in source


def test_v7_unprivileged_daemon_has_query_only_cleanup_authority():
    source = inspect.getsource(daemon.PrivilegedCleanupGuardView)
    assert "def require_clear" in source
    assert "def begin_unbound" not in source
    assert "def bind(" not in source
    assert "def terminate(" not in source


def provision(tmp_path):
    root = tmp_path / "authority"
    GlobalAttemptBudgetStore._provision_test_root(str(root), TIMESTAMP)
    for relative in ("revocation/records", "revocation/pending", "revocation/processed", "receipts"):
        (root / relative).mkdir(mode=0o700, parents=True, exist_ok=True)
    return root


def commitment(campaign="campaign1", output="output1"):
    return {
        "campaign_identity": DIGEST,
        "campaign_template_identity": "b" * 64,
        "domain_id": 100,
        "output_root_identity": output,
        "process_manifest_identity": "c" * 64,
        "process_instance_identity": "d" * 64,
        "peer_pid": 10,
        "peer_start_time_ticks": 20,
        "peer_executable": "/usr/bin/python3",
        "committed_at_utc": TIMESTAMP,
        "service_instance_identity": "e" * 64,
        "campaign_id": campaign,
        "observation_session_identity": "8" * 64,
        "four_source_observation_identity": "9" * 64,
        "precommit_receipt_identities": ["f" * 64],
        "precommit_observer_count": 1,
        "prepare_token_identity": "1" * 64,
        "lease_identity": "2" * 64,
    }


def test_revision_zero_is_external_and_commit_is_permanent(tmp_path):
    root = provision(tmp_path)
    store = GlobalAttemptBudgetStore._for_test(str(root))
    try:
        assert store.observe().record.data["state"] == "UNCONSUMED"
        committed = store.commit(
            authorization_identity=DIGEST, commitment=commitment(), timestamp=TIMESTAMP,
        )
        assert committed.record.data["state"] == "COMMITTED"
        assert committed.record.data["attempts_consumed"] == 1
        failed = store.finalize("FAILED_AFTER_COMMIT", timestamp=TIMESTAMP)
        assert failed.record.data["state"] == "FAILED_AFTER_COMMIT"
        with pytest.raises(Slice7GAuthorityDaemonError, match="budget_consumed"):
            store.commit(
                authorization_identity=DIGEST,
                commitment=commitment("different-campaign", "different-output"),
                timestamp=TIMESTAMP,
            )
    finally:
        store.close()
    restarted = GlobalAttemptBudgetStore._for_test(str(root))
    try:
        assert restarted.observe().record.data["state"] == "FAILED_AFTER_COMMIT"
        assert restarted.observe().record.data["attempts_consumed"] == 1
    finally:
        restarted.close()


def test_concurrent_campaign_and_output_contenders_have_one_winner(tmp_path):
    root = provision(tmp_path)
    barrier = threading.Barrier(8)
    results = []
    lock = threading.Lock()

    def contender(index):
        store = GlobalAttemptBudgetStore._for_test(str(root))
        try:
            barrier.wait()
            store.commit(
                authorization_identity=DIGEST,
                commitment=commitment(f"campaign{index}", f"output{index}"),
                timestamp=TIMESTAMP,
            )
            result = "winner"
        except Slice7GAuthorityDaemonError:
            result = "rejected"
        finally:
            store.close()
        with lock:
            results.append(result)

    threads = [threading.Thread(target=contender, args=(index,)) for index in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=10)
    assert results.count("winner") == 1
    assert results.count("rejected") == 7


def test_gap_corruption_alias_and_writable_lock_fail_closed(tmp_path):
    root = provision(tmp_path)
    budget = root / "global-budget"
    gap = budget / "revision-00000000000000000002.json"
    gap.write_text("{}", encoding="utf-8")
    os.chmod(gap, 0o600)
    store = GlobalAttemptBudgetStore._for_test(str(root))
    try:
        with pytest.raises(Slice7GAuthorityDaemonError, match="budget_revision_history"):
            store.observe()
    finally:
        store.close()
    gap.unlink()
    revision = budget / "revision-00000000000000000000.json"
    revision.write_bytes(revision.read_bytes() + b"\n")
    store = GlobalAttemptBudgetStore._for_test(str(root))
    try:
        with pytest.raises(Exception):
            store.observe()
    finally:
        store.close()


def test_runtime_cannot_initialize_revision_zero(tmp_path):
    root = tmp_path / "missing"
    with pytest.raises((FileNotFoundError, Slice7GAuthorityDaemonError)):
        GlobalAttemptBudgetStore._for_test(str(root))
    assert not root.exists()


def _sealed_lease_record(directory, name="active.json", *, domain=100):
    projection = {
        "schema_version": "ctr-slice-7g-domain-reservation-1",
        "domain_id": domain,
        "runtime_authorization_identity": "1" * 64,
        "campaign_identity": "2" * 64,
        "reserved_at_utc": TIMESTAMP,
    }
    identity = hashlib.sha256(
        b"ctr-slice-7g-domain-reservation-canonical-1\0"
        + json.dumps(projection, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    path = directory / name
    path.write_bytes(json.dumps(
        {**projection, "identity": identity}, sort_keys=True, separators=(",", ":"),
    ).encode())
    os.chmod(path, 0o444)
    return path, identity


def _sealed_binding_record(directory, reservation_identity, *, output_root):
    projection = {
        "schema_version": "ctr-slice-7g-domain-committed-binding-1",
        "domain_lease_identity": "3" * 64,
        "domain_reservation_identity": reservation_identity,
        "final_domain_observation_identity": "4" * 64,
        "runtime_authorization_identity": "1" * 64,
        "campaign_identity": "2" * 64,
        "campaign_plan_identity": "5" * 64,
        "attempt_ledger_identity": "6" * 64,
        "attempt_ledger_revision": 1,
        "process_start_event_identity": "7" * 64,
        "domain_id": 100,
        "output_root": output_root,
    }
    identity = hashlib.sha256(
        b"ctr-slice-7g-domain-committed-binding-canonical-1\0"
        + json.dumps(projection, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    path = directory / f"binding.{reservation_identity}.json"
    path.write_bytes(json.dumps(
        {**projection, "identity": identity}, sort_keys=True, separators=(",", ":"),
    ).encode())
    os.chmod(path, 0o444)
    return path, identity


def test_daemon_owned_global_lease_observer_reads_clear_and_conflicting_shared_state(tmp_path):
    registry = tmp_path / "lease-registry"
    GlobalLeaseStateObserver._provision_test_registry(str(registry))
    observer = GlobalLeaseStateObserver._for_test(str(registry))
    try:
        clear = observer.observe(100, 1)
        assert clear.clear is True
        assert clear.record.data["state"] == "CLEAR"
        domain = registry / "domain_100"
        domain.mkdir(mode=0o700)
        _, identity = _sealed_lease_record(domain)
        occupied = observer.observe(100, 2)
        assert occupied.clear is False
        assert occupied.record.data["state"] == "RESERVED"
        assert occupied.record.data["active_reservation_identities"] == (identity,)
        assert occupied.record.data["registry_revision_identity"] != clear.record.data["registry_revision_identity"]
    finally:
        observer.close()


@pytest.mark.parametrize("defect", [
    "missing", "malformed", "writable", "symlink", "hardlink", "replaced", "busy",
])
def test_global_lease_observer_physical_and_lock_defects_fail_closed(tmp_path, defect):
    registry = tmp_path / "lease-registry"
    if defect == "missing":
        with pytest.raises((FileNotFoundError, Slice7GAuthorityDaemonError)):
            GlobalLeaseStateObserver._for_test(str(registry))
        return
    GlobalLeaseStateObserver._provision_test_registry(str(registry))
    observer = GlobalLeaseStateObserver._for_test(str(registry))
    lock = None
    try:
        if defect == "replaced":
            moved = tmp_path / "moved-registry"
            registry.rename(moved)
            GlobalLeaseStateObserver._provision_test_registry(str(registry))
        elif defect == "busy":
            lock = os.open(registry / "registry.lock", os.O_RDWR | os.O_CLOEXEC)
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        else:
            domain = registry / "domain_100"
            domain.mkdir(mode=0o700)
            path, _ = _sealed_lease_record(domain)
            if defect == "malformed":
                os.chmod(path, 0o644)
                path.write_bytes(b"{}")
                os.chmod(path, 0o444)
            elif defect == "writable":
                os.chmod(path, 0o644)
            elif defect == "symlink":
                path.unlink()
                path.symlink_to("elsewhere")
            elif defect == "hardlink":
                os.link(path, domain / ("reservation." + ("3" * 64) + ".json"))
        with pytest.raises((OSError, Slice7GAuthorityDaemonError)):
            observer.observe(100, 1)
    finally:
        if lock is not None:
            fcntl.flock(lock, fcntl.LOCK_UN)
            os.close(lock)
        observer.close()


def test_global_lease_revision_change_during_observation_is_rejected(tmp_path, monkeypatch):
    registry = tmp_path / "lease-registry"
    GlobalLeaseStateObserver._provision_test_registry(str(registry))
    observer = GlobalLeaseStateObserver._for_test(str(registry))
    original = observer._inventory
    calls = [0]

    def changing(domain):
        result = original(domain)
        calls[0] += 1
        if calls[0] == 1:
            target = registry / "domain_100"
            target.mkdir(mode=0o700)
            _sealed_lease_record(target)
        return result

    monkeypatch.setattr(observer, "_inventory", changing)
    try:
        with pytest.raises(Slice7GAuthorityDaemonError, match="global_lease_changed"):
            observer.observe(100, 1)
    finally:
        observer.close()


def test_released_global_lease_history_is_clear_and_shared_across_output_roots(tmp_path):
    registry = tmp_path / "lease-registry"
    GlobalLeaseStateObserver._provision_test_registry(str(registry))
    domain = registry / "domain_100"
    domain.mkdir(mode=0o700)
    active, reservation_identity = _sealed_lease_record(domain)
    reservation_history = domain / f"reservation.{reservation_identity}.json"
    active.rename(reservation_history)
    release_projection = {
        "schema_version": "ctr-slice-7g-domain-release-1",
        "domain_id": 100,
        "domain_lease_identity": "4" * 64,
        "domain_reservation_identity": reservation_identity,
        "released_at_utc": TIMESTAMP,
    }
    release_identity = hashlib.sha256(
        b"ctr-slice-7g-domain-release-canonical-1\0"
        + json.dumps(release_projection, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    release = domain / f"release.{reservation_identity}.json"
    release.write_bytes(json.dumps(
        {**release_projection, "identity": release_identity},
        sort_keys=True, separators=(",", ":"),
    ).encode())
    os.chmod(release, 0o444)
    observer = GlobalLeaseStateObserver._for_test(str(registry))
    try:
        observed = observer.observe(100, 1)
        assert observed.clear is True
        assert observed.record.data["state"] == "CLEAR"
    finally:
        observer.close()


def test_committed_lease_for_another_output_root_is_globally_occupied(tmp_path):
    registry = tmp_path / "lease-registry"
    GlobalLeaseStateObserver._provision_test_registry(str(registry))
    domain = registry / "domain_100"
    domain.mkdir(mode=0o700)
    _, reservation_identity = _sealed_lease_record(domain)
    _, binding_identity = _sealed_binding_record(
        domain, reservation_identity,
        output_root="/home/ankid/ctr_mppi_evidence/slice_7g/another-campaign",
    )
    observer = GlobalLeaseStateObserver._for_test(str(registry))
    try:
        observed = observer.observe(100, 1)
        assert observed.clear is False
        assert observed.record.data["state"] == "COMMITTED"
        assert observed.record.data["committed_binding_identities"] == (
            binding_identity,
        )
    finally:
        observer.close()


def test_same_size_lease_record_replacement_during_observation_is_rejected(tmp_path, monkeypatch):
    registry = tmp_path / "lease-registry"
    GlobalLeaseStateObserver._provision_test_registry(str(registry))
    domain = registry / "domain_100"
    domain.mkdir(mode=0o700)
    active, _ = _sealed_lease_record(domain)
    observer = GlobalLeaseStateObserver._for_test(str(registry))
    original = observer._inventory
    calls = [0]

    def mutate(domain_id):
        result = original(domain_id)
        calls[0] += 1
        if calls[0] == 1:
            raw = json.loads(active.read_bytes())
            raw["campaign_identity"] = "8" * 64
            projection = {key: value for key, value in raw.items() if key != "identity"}
            raw["identity"] = hashlib.sha256(
                b"ctr-slice-7g-domain-reservation-canonical-1\0"
                + json.dumps(projection, sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest()
            replacement = json.dumps(raw, sort_keys=True, separators=(",", ":")).encode()
            assert len(replacement) == active.stat().st_size
            os.chmod(active, 0o644)
            active.write_bytes(replacement)
            os.chmod(active, 0o444)
        return result

    monkeypatch.setattr(observer, "_inventory", mutate)
    try:
        with pytest.raises(Slice7GAuthorityDaemonError, match="global_lease_changed"):
            observer.observe(100, 1)
    finally:
        observer.close()


def test_byte_identical_lease_record_inode_replacement_is_rejected(tmp_path, monkeypatch):
    registry = tmp_path / "lease-registry"
    GlobalLeaseStateObserver._provision_test_registry(str(registry))
    domain = registry / "domain_100"
    domain.mkdir(mode=0o700)
    active, _ = _sealed_lease_record(domain)
    observer = GlobalLeaseStateObserver._for_test(str(registry))
    original = observer._inventory
    calls = [0]

    def replace(domain_id):
        result = original(domain_id)
        calls[0] += 1
        if calls[0] == 1:
            replacement = active.with_name("replacement.json")
            replacement.write_bytes(active.read_bytes())
            replacement.chmod(0o444)
            os.replace(replacement, active)
        return result

    monkeypatch.setattr(observer, "_inventory", replace)
    try:
        with pytest.raises(Slice7GAuthorityDaemonError, match="global_lease_changed"):
            observer.observe(100, 1)
    finally:
        observer.close()


def test_final_retained_lease_descriptor_barrier_rejects_last_moment_same_byte_replacement(
    tmp_path, monkeypatch,
):
    registry = tmp_path / "lease-registry"
    GlobalLeaseStateObserver._provision_test_registry(str(registry))
    domain = registry / "domain_100"
    domain.mkdir(mode=0o700)
    active, _ = _sealed_lease_record(domain)
    observer = GlobalLeaseStateObserver._for_test(str(registry))
    original = observer._final_inventory_barrier
    replaced = False

    def final_barrier(domain_id, authenticated, retained):
        nonlocal replaced
        if not replaced:
            replaced = True
            replacement = active.with_name("late-replacement.json")
            replacement.write_bytes(active.read_bytes())
            replacement.chmod(0o444)
            os.replace(replacement, active)
        return original(domain_id, authenticated, retained)

    monkeypatch.setattr(observer, "_final_inventory_barrier", final_barrier)
    try:
        with pytest.raises(Slice7GAuthorityDaemonError) as caught:
            observer.observe(100, 1)
        assert caught.value.code == "global_lease_record_replaced"
        assert replaced is True
    finally:
        observer.close()


def _begin_cleanup_guard(guard, store, *, service="2" * 64):
    return guard.begin(
        authorization_identity="1" * 64,
        budget_identity=store.observe().record.logical_identity,
        service_generation_identity=service,
        session_binding_identity="3" * 64,
        phase="PRECOMMIT", phase_local_ordinal=1,
        transaction_observer_ordinal=1, domain_id=100,
        executable_identity="4" * 64, argv_identity="5" * 64,
        environment_identity="6" * 64, timestamp=TIMESTAMP,
    )


def test_cleanup_quarantine_survives_store_and_state_machine_reconstruction(tmp_path):
    root = provision(tmp_path)
    store = GlobalAttemptBudgetStore._for_test(str(root))
    guard = ObserverCleanupGuardStore._for_test(str(root))
    active = _begin_cleanup_guard(guard, store)
    quarantined = guard.quarantine(active.record.logical_identity, "7" * 64, TIMESTAMP)
    assert quarantined.record.data["state"] == "QUARANTINED"
    guard.close()
    restarted = ObserverCleanupGuardStore._for_test(str(root))
    try:
        assert restarted.observe().record.logical_identity == quarantined.record.logical_identity
        with pytest.raises(Slice7GAuthorityDaemonError, match="observation_cleanup_uncertain"):
            RuntimeAuthorityStateMachine._for_test(
                bootstrap=_bootstrap(), authorization=_authorization(store), budget=store,
                cleanup_guard=restarted, service_instance_identity="8" * 64,
                peer_matcher=lambda peer, request: None,
            )
    finally:
        restarted.close()
        store.close()


def test_cleanup_recovery_is_separate_fresh_one_shot_authority(tmp_path):
    root = provision(tmp_path)
    store = GlobalAttemptBudgetStore._for_test(str(root))
    guard = ObserverCleanupGuardStore._for_test(str(root))
    active = _begin_cleanup_guard(guard, store)
    quarantined = guard.quarantine(active.record.logical_identity, "7" * 64, TIMESTAMP)
    recovery = {
        "schema_version": protocol.OBSERVER_CLEANUP_RECOVERY_SCHEMA,
        "recovery_nonce": "recovery1",
        "quarantine_identity": quarantined.record.logical_identity,
        "authority_root_identity": guard._authority_root_identity,
        "runtime_authorization_identity": "1" * 64,
        "budget_identity": store.observe().record.logical_identity,
        "service_generation_identity": "9" * 64,
        "issued_at_utc": TIMESTAMP, "not_before_utc": TIMESTAMP,
        "not_after_utc": "2026-08-22T01:00:00Z", "one_shot": True,
    }
    with pytest.raises(Slice7GAuthorityDaemonError, match="cleanup_recovery_residual"):
        guard.recover_for_test(
            recovery, process_clear=True, dds_clear=True, lease_clear=False,
            graph_clear=True, disposition_identity="a" * 64, timestamp=TIMESTAMP,
            current_service_generation_identity="9" * 64,
        )
    recovered = guard.recover_for_test(
        recovery, process_clear=True, dds_clear=True, lease_clear=True,
        graph_clear=True, disposition_identity="a" * 64, timestamp=TIMESTAMP,
        current_service_generation_identity="9" * 64,
    )
    assert recovered.record.data["state"] == "RECOVERED"
    with pytest.raises(Slice7GAuthorityDaemonError, match="cleanup_recovery_state"):
        guard.recover_for_test(
            recovery, process_clear=True, dds_clear=True, lease_clear=True,
            graph_clear=True, disposition_identity="b" * 64, timestamp=TIMESTAMP,
            current_service_generation_identity="9" * 64,
        )
    assert store.observe().record.data["attempts_consumed"] == 0
    guard.close()
    store.close()


@pytest.mark.parametrize("defect", [
    "missing", "writable", "symlink", "hardlink", "replaced", "unknown",
])
def test_cleanup_guard_physical_authority_defects_fail_closed(tmp_path, defect):
    root = provision(tmp_path)
    guard_root = root / "observer-cleanup-guard"
    revision = guard_root / "revision-00000000000000000000.json"
    if defect == "missing":
        revision.unlink()
        guard = ObserverCleanupGuardStore._for_test(str(root))
        try:
            with pytest.raises(Slice7GAuthorityDaemonError):
                guard.observe()
        finally:
            guard.close()
        return
    guard = ObserverCleanupGuardStore._for_test(str(root))
    try:
        if defect == "writable":
            os.chmod(revision, 0o644)
        elif defect == "symlink":
            revision.unlink()
            revision.symlink_to("guard.lock")
        elif defect == "hardlink":
            os.link(revision, guard_root / "revision-00000000000000000001.json")
        elif defect == "replaced":
            moved = root / "moved-guard"
            guard_root.rename(moved)
            guard_root.mkdir(mode=0o700)
            (guard_root / "guard.lock").write_bytes(b"")
            os.chmod(guard_root / "guard.lock", 0o600)
        else:
            (guard_root / "caller-clear.json").write_bytes(b"{}")
        with pytest.raises((OSError, Slice7GAuthorityDaemonError)):
            guard.observe()
    finally:
        guard.close()


def test_cleanup_guard_concurrent_begin_has_exactly_one_winner(tmp_path):
    root = provision(tmp_path)
    budget = GlobalAttemptBudgetStore._for_test(str(root))
    budget_identity = budget.observe().record.logical_identity
    budget.close()
    barrier = threading.Barrier(6)
    results = []
    result_lock = threading.Lock()

    def contender(index):
        guard = ObserverCleanupGuardStore._for_test(str(root))
        try:
            barrier.wait()
            guard.begin(
                authorization_identity="1" * 64, budget_identity=budget_identity,
                service_generation_identity="2" * 64,
                session_binding_identity=f"{index + 10:064x}",
                phase="PRECOMMIT", phase_local_ordinal=1,
                transaction_observer_ordinal=1, domain_id=100,
                executable_identity="4" * 64, argv_identity="5" * 64,
                environment_identity="6" * 64, timestamp=TIMESTAMP,
            )
            result = "winner"
        except Slice7GAuthorityDaemonError:
            result = "rejected"
        finally:
            guard.close()
        with result_lock:
            results.append(result)

    threads = [threading.Thread(target=contender, args=(index,)) for index in range(6)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=10)
    assert results.count("winner") == 1
    assert results.count("rejected") == 5


def test_cleanup_recovery_rejects_wrong_binding_generation_and_expiry(tmp_path):
    root = provision(tmp_path)
    store = GlobalAttemptBudgetStore._for_test(str(root))
    guard = ObserverCleanupGuardStore._for_test(str(root))
    active = _begin_cleanup_guard(guard, store)
    quarantined = guard.quarantine(active.record.logical_identity, "7" * 64, TIMESTAMP)
    valid = {
        "schema_version": protocol.OBSERVER_CLEANUP_RECOVERY_SCHEMA,
        "recovery_nonce": "recovery2",
        "quarantine_identity": quarantined.record.logical_identity,
        "authority_root_identity": guard._authority_root_identity,
        "runtime_authorization_identity": "1" * 64,
        "budget_identity": store.observe().record.logical_identity,
        "service_generation_identity": "9" * 64,
        "issued_at_utc": TIMESTAMP, "not_before_utc": TIMESTAMP,
        "not_after_utc": "2026-08-22T01:00:00Z", "one_shot": True,
    }
    cases = (
        ({**valid, "quarantine_identity": "a" * 64}, "9" * 64, TIMESTAMP),
        ({**valid, "authority_root_identity": "b" * 64}, "9" * 64, TIMESTAMP),
        (valid, "c" * 64, TIMESTAMP),
        (valid, "9" * 64, "2026-08-22T01:00:00Z"),
    )
    try:
        for recovery, generation, now in cases:
            with pytest.raises(Slice7GAuthorityDaemonError):
                guard.recover_for_test(
                    recovery, process_clear=True, dds_clear=True,
                    lease_clear=True, graph_clear=True,
                    disposition_identity="d" * 64, timestamp=now,
                    current_service_generation_identity=generation,
                )
        assert guard.observe().record.data["state"] == "QUARANTINED"
        assert store.observe().record.data["attempts_consumed"] == 0
    finally:
        guard.close()
        store.close()


@pytest.mark.parametrize("unclear", ["process", "dds", "lease", "graph"])
def test_cleanup_recovery_requires_every_fresh_daemon_owned_source_clear(tmp_path, unclear):
    root = provision(tmp_path)
    store = GlobalAttemptBudgetStore._for_test(str(root))
    guard = ObserverCleanupGuardStore._for_test(str(root))
    active = _begin_cleanup_guard(guard, store)
    quarantined = guard.quarantine(active.record.logical_identity, "7" * 64, TIMESTAMP)
    recovery = {
        "schema_version": protocol.OBSERVER_CLEANUP_RECOVERY_SCHEMA,
        "recovery_nonce": "recovery3",
        "quarantine_identity": quarantined.record.logical_identity,
        "authority_root_identity": guard._authority_root_identity,
        "runtime_authorization_identity": "1" * 64,
        "budget_identity": store.observe().record.logical_identity,
        "service_generation_identity": "9" * 64,
        "issued_at_utc": TIMESTAMP, "not_before_utc": TIMESTAMP,
        "not_after_utc": "2026-08-22T01:00:00Z", "one_shot": True,
    }
    clearances = {name: name != unclear for name in ("process", "dds", "lease", "graph")}
    try:
        with pytest.raises(Slice7GAuthorityDaemonError, match="cleanup_recovery_residual"):
            guard.recover_for_test(
                recovery, process_clear=clearances["process"],
                dds_clear=clearances["dds"], lease_clear=clearances["lease"],
                graph_clear=clearances["graph"], disposition_identity="a" * 64,
                timestamp=TIMESTAMP, current_service_generation_identity="9" * 64,
            )
        assert guard.observe().record.data["state"] == "QUARANTINED"
        assert store.observe().record.data["attempts_consumed"] == 0
    finally:
        guard.close()
        store.close()


def test_failure_before_leader_provenance_is_durably_quarantined(tmp_path, monkeypatch):
    root = provision(tmp_path)
    guard = ObserverCleanupGuardStore._for_test(str(root))
    monkeypatch.setattr(
        daemon, "_set_child_subreaper",
        lambda enabled: (_ for _ in ()).throw(RuntimeError("subreaper unavailable")),
    )
    context = {
        "authorization_identity": "1" * 64, "budget_identity": "2" * 64,
        "service_generation_identity": "3" * 64,
        "session_binding_identity": "4" * 64, "phase": "PRECOMMIT",
        "phase_local_ordinal": 1, "transaction_observer_ordinal": 1,
        "domain_id": 100, "executable_identity": "5" * 64,
        "argv_identity": "6" * 64, "environment_identity": "7" * 64,
    }
    try:
        with pytest.raises(RuntimeError, match="subreaper unavailable"):
            _run_server_owned_graph_observer(
                ("/usr/bin/python3", "-I", "-c", "pass"),
                {"ROS_DOMAIN_ID": "100"}, str(tmp_path), "/",
                cleanup_guard=guard, guard_context=context,
                utc_now=lambda: TIMESTAMP, port_observer=lambda domain: (),
            )
        identity = guard.observe().record.logical_identity
        assert guard.observe().record.data["state"] == "QUARANTINED"
        guard.close()
        guard = ObserverCleanupGuardStore._for_test(str(root))
        assert guard.observe().record.logical_identity == identity
        assert guard.observe().record.data["state"] == "QUARANTINED"
    finally:
        guard.close()


def test_fast_exit_leader_descendant_is_cleaned_by_product_before_reap(tmp_path):
    root = provision(tmp_path)
    guard = ObserverCleanupGuardStore._for_test(str(root))
    child_pid_path = tmp_path / "child.pid"
    helper = (
        "import os,subprocess;"
        f"p=subprocess.Popen(['/usr/bin/python3','-I','-c','import time;time.sleep(60)'],"
        "stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL);"
        f"open({str(child_pid_path)!r},'w').write(str(p.pid));os._exit(0)"
    )
    cgroup_line = Path("/proc/self/cgroup").read_text(encoding="utf-8").strip()
    assert cgroup_line.startswith("0::/")
    context = {
        "authorization_identity": "1" * 64,
        "budget_identity": "2" * 64,
        "service_generation_identity": "3" * 64,
        "session_binding_identity": "4" * 64,
        "phase": "PRECOMMIT", "phase_local_ordinal": 1,
        "transaction_observer_ordinal": 1, "domain_id": 100,
        "executable_identity": "5" * 64, "argv_identity": "6" * 64,
        "environment_identity": "7" * 64,
    }
    with pytest.raises(Slice7GAuthorityDaemonError, match="observer_unexpected_descendant"):
        _run_server_owned_graph_observer(
            ("/usr/bin/python3", "-I", "-c", helper), {"ROS_DOMAIN_ID": "100"},
            str(tmp_path), cgroup_line[3:], cleanup_guard=guard,
            guard_context=context, utc_now=lambda: TIMESTAMP,
            port_observer=lambda domain: (),
        )
    child_pid = int(child_pid_path.read_text(encoding="ascii"))
    deadline = time.monotonic() + 2.0
    while Path(f"/proc/{child_pid}").exists() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not Path(f"/proc/{child_pid}").exists()
    assert guard.observe().record.data["state"] == "CLEARED"
    guard.close()


def test_fast_exit_leader_multiple_descendants_are_all_cleaned_by_product(tmp_path):
    root = provision(tmp_path)
    guard = ObserverCleanupGuardStore._for_test(str(root))
    child_pid_path = tmp_path / "children.pid"
    helper = (
        "import os,subprocess;ps=[subprocess.Popen(['/usr/bin/python3','-I','-c',"
        "'import time;time.sleep(60)'],stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,"
        "stderr=subprocess.DEVNULL) for _ in range(3)];"
        f"open({str(child_pid_path)!r},'w').write(','.join(str(p.pid) for p in ps));"
        "os._exit(0)"
    )
    cgroup_line = Path("/proc/self/cgroup").read_text(encoding="utf-8").strip()
    context = {
        "authorization_identity": "1" * 64, "budget_identity": "2" * 64,
        "service_generation_identity": "3" * 64,
        "session_binding_identity": "4" * 64, "phase": "PRECOMMIT",
        "phase_local_ordinal": 1, "transaction_observer_ordinal": 1,
        "domain_id": 100, "executable_identity": "5" * 64,
        "argv_identity": "6" * 64, "environment_identity": "7" * 64,
    }
    try:
        with pytest.raises(Slice7GAuthorityDaemonError, match="observer_unexpected_descendant"):
            _run_server_owned_graph_observer(
                ("/usr/bin/python3", "-I", "-c", helper),
                {"ROS_DOMAIN_ID": "100"}, str(tmp_path), cgroup_line[3:],
                cleanup_guard=guard, guard_context=context,
                utc_now=lambda: TIMESTAMP, port_observer=lambda domain: (),
            )
        children = [int(value) for value in child_pid_path.read_text(encoding="ascii").split(",")]
        assert len(children) == 3
        assert all(not Path(f"/proc/{pid}").exists() for pid in children)
        assert guard.observe().record.data["state"] == "CLEARED"
    finally:
        guard.close()


def test_restart_recovery_retains_consumption_and_requests_cgroup_termination(tmp_path):
    root = provision(tmp_path)
    store = GlobalAttemptBudgetStore._for_test(str(root))
    authorization = _authorization(store)
    store.commit(
        authorization_identity=protocol.authority_record_identity(
            authorization, expected_schema=protocol.RUNTIME_AUTHORIZATION_SCHEMA,
        ),
        commitment=commitment(),
        timestamp=TIMESTAMP,
    )
    machine = RuntimeAuthorityStateMachine._for_test(
        bootstrap=_bootstrap(), authorization=authorization, budget=store,
        service_instance_identity="6" * 64, peer_matcher=lambda peer, request: None,
        peer_reconciler=lambda peer: peer, monotonic=lambda: 10.0,
        utc_now=lambda: TIMESTAMP,
    )
    recovered = machine.recover_abandoned_commit()
    assert recovered.record.data["state"] == "FAILED_AFTER_COMMIT"
    assert recovered.record.data["attempts_consumed"] == 1
    pending = list((root / "revocation" / "pending").iterdir())
    assert [item.name for item in pending] == [
        "service-restart-revision-00000000000000000001.json",
    ]
    trigger = protocol.validate_authority_record(
        pending[0].read_bytes(), expected_schema=protocol.AUTHORITY_REVOCATION_SCHEMA,
    )
    assert trigger.data["state"] == "TRIGGERED_POSTCOMMIT"
    store.close()


def _file(path, inode):
    return {
        "path": path, "mode": 0o555, "link_count": 1, "device": 1,
        "inode": inode, "size": 3, "sha256": "0" * 64,
        "owner_uid": 0, "owner_gid": 0,
    }


def _bootstrap():
    return {
        "schema_version": protocol.AUTHORITY_BOOTSTRAP_SCHEMA,
        "authority_uid": 101, "authority_gid": 102,
        "campaign_uid": 103, "runtime_gid": 104,
        "authority_account": protocol.AUTHORITY_ACCOUNT,
        "campaign_account": protocol.CAMPAIGN_ACCOUNT,
        "runtime_group": protocol.RUNTIME_GROUP,
        "bootstrap_path": protocol.AUTHORITY_BOOTSTRAP_PATH,
        "service_executable_path": protocol.AUTHORITY_SERVICE_PATH,
        "state_root": protocol.AUTHORITY_STATE_ROOT,
        "socket_path": protocol.AUTHORITY_SOCKET_PATH,
        "installed_runtime_parent": protocol.INSTALLED_RUNTIME_PARENT,
        "service_executable": _file(protocol.AUTHORITY_SERVICE_PATH, 1),
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


def _authorization(store=None):
    snapshot = {
        "schema_version": "ctr-slice-7g-post-implementation-source-snapshot-3",
        "path": protocol.AUTHORITY_STATE_ROOT + "/public/source.json",
        "physical_sha256": "0" * 64, "logical_identity": "1" * 64,
        "logical_identity_algorithm": "sha256:ctr-slice-7g-post-implementation-source-snapshot-canonical-3",
        "member_count": 1, "mode_bound": True,
    }
    return {
        "schema_version": protocol.RUNTIME_AUTHORIZATION_SCHEMA,
        "authorization_nonce": "authority1", "issued_at_utc": TIMESTAMP,
        "not_before_utc": TIMESTAMP, "not_after_utc": "2026-08-23T00:00:00Z",
        "branch": "milestone/06b-curved-lumen-sim", "head": "a" * 40,
        "tracked_diff_sha256": "0" * 64, "correction_manifest_sha256": "0" * 64,
        "complete_subject_manifest_sha256": "0" * 64, "source_snapshot": snapshot,
        "charter": {
            "schema_version": "ctr-slice-7g-charter-6",
            "path": "/opt/ctr-mppi/slice-7g/charter.json",
            "physical_sha256": "0" * 64, "logical_identity": "0" * 64,
            "logical_identity_algorithm": "sha256:ctr-slice-7g-charter-canonical-6",
        },
        "build_test_approval_identity": "0" * 64,
        "installed_runtime_identity": "0" * 64,
        "process_manifest_identity": "2" * 64,
        "environment_manifest_identity": "3" * 64,
        "applicable_test_nodes": 1, "node_id_sha256": "0" * 64,
        "git_command_manifest_sha256": "0" * 64,
        "entrypoint_identity": "0" * 64,
        "campaign": {
            "endpoint": "simulation_only_promoted_completion",
            "scenarios": ["centerline", "lateral_offset", "near_safety_boundary"],
            "seeds": [11, 22, 33, 44, 55], "duration_seconds": 25.0,
            "retries": 0, "domain_minimum": 100, "domain_maximum": 199,
            "plan_identity": "4" * 64,
            "campaign_identity_algorithm": "sha256:ctr-slice-7g-runtime-campaign-canonical-1",
        },
        "readiness_acceptance_identity": "0" * 64,
        "evidence_schemas": {"seal": "ctr-slice-7g-campaign-evidence-seal-1"},
        "global_budget_identity": (
            "0" * 64 if store is None else store.observe().record.logical_identity
        ),
        "output_parent_rule": {
            "path": protocol.OUTPUT_PARENT, "authority_creates_root": True,
            "campaign_parent_entry_mutation": False, "campaign_parent_listing": False,
            "acl_policy_identity": "0" * 64,
        },
        "prepare_token_lifetime_seconds": 300, "one_shot": True,
    }


def _peer(uid=103, gid=104):
    credentials = protocol.Slice7GPeerCredentials(1234, uid, gid)
    return protocol.Slice7GPeerProcess(
        credentials, 55, "/usr/bin/python3", ("/usr/bin/python3", "/opt/ctr-mppi/campaign.py"),
        (), "/opt/ctr-mppi/slice-7g/fixed", "/system.slice/ctr-slice7g-campaign.service",
    )


def _request(method, **updates):
    value = {
        "schema_version": protocol.AUTHORITY_REQUEST_SCHEMA, "method": method,
        "request_id": "request" + method.replace("_", ""),
        "authorization_identity": None, "prepare_token": None,
        "campaign_id": None, "campaign_identity": None,
        "campaign_template_identity": None, "domain_id": None,
        "output_root_path": None, "output_root_identity": None,
        "process_manifest_identity": None, "process_instance_identity": None,
        "observation_session_identity": None, "observation_session_nonce": None,
        "requested_at_utc": TIMESTAMP,
    }
    value.update(updates)
    return value


def _evidence(domain=100, phase="PRECOMMIT", nodes=None):
    del domain, phase
    return DaemonObservationEvidence(
        active_process_identity="5" * 64, active_process_clear=True,
        dds_port_identity="6" * 64, dds_port_clear=True,
        global_lease_identity="7" * 64,
        global_lease_registry_identity="8" * 64,
        global_lease_revision_identity="9" * 64,
        global_lease_state="CLEAR", global_lease_clear=True,
        peer_process_identity="8" * 64, observation_interval_identity="9" * 64,
        graph_provider_identity="a" * 64,
        executable=protocol.PRECOMMIT_ROS_GRAPH_OBSERVER_EXECUTABLE,
        executable_identity="0" * 64, interpreter="/usr/bin/python3",
        interpreter_identity="1" * 64, module_origin_identities=("2" * 64,),
        argv=(protocol.PRECOMMIT_ROS_GRAPH_OBSERVER_EXECUTABLE, "node", "list", "--no-daemon"),
        environment_identity="3" * 64, working_directory="/opt/ctr-mppi/slice-7g/fixed",
        cgroup="/system.slice/ctr-slice7g-campaign.service", pid=4321,
        process_group_id=4321, process_start_time_ticks=66,
        started_monotonic_ns=1, ended_monotonic_ns=2, exit_status=0,
        terminating_signal=None, stdout=b"", stderr=b"",
        nodes=tuple(() if nodes is None else nodes), cleanup_barrier_identity="4" * 64,
        unexpected_descendants=0, ros_daemon_started=False, observed_monotonic_ns=3,
    )


def _observe_and_prepare(machine, peer, connection="connection1", domain=100):
    started = machine.handle(_request("begin_observation"), peer, connection)
    session = {
        "authorization_identity": started["authorization_identity"],
        "observation_session_identity": started["observation_session_identity"],
        "observation_session_nonce": started["observation_session_nonce"],
    }
    observed = machine.handle(_request(
        "record_precommit_observation", **session, domain_id=domain,
    ), peer, connection)
    assert observed["result"] == "OBSERVATION_RECORDED"
    finalized_bind = {
        **session, "domain_id": domain,
        "process_manifest_identity": "2" * 64,
    }
    finalized = machine.handle(_request(
        "finalize_observation", **finalized_bind,
    ), peer, connection)
    assert finalized["result"] == "OBSERVATION_COMPLETE"
    finalized_bind.update({
        "domain_id": finalized["domain_id"],
    })
    prepared = machine.handle(_request("prepare", **finalized_bind), peer, connection)
    bind = {
        **finalized_bind,
        "prepare_token": prepared["prepare_token"],
        "campaign_id": prepared["campaign_id"],
        "campaign_identity": prepared["campaign_identity"],
        "campaign_template_identity": prepared["campaign_template_identity"],
    }
    return prepared, bind


def _record_postcommit(machine, peer, bind, connection="connection1"):
    response = machine.handle(_request(
        "record_postcommit_observation", **bind,
    ), peer, connection)
    assert response["result"] == "POSTCOMMIT_RECORDED"
    return {
        **bind,
    }


def _observation_machine(tmp_path, clock, provider=None):
    root = provision(tmp_path)
    store = GlobalAttemptBudgetStore._for_test(str(root))
    machine = RuntimeAuthorityStateMachine._for_test(
        bootstrap=_bootstrap(), authorization=_authorization(store), budget=store,
        service_instance_identity="6" * 64, peer_matcher=lambda peer, request: None,
        peer_reconciler=lambda peer: peer,
        observation_provider=provider or (
            lambda session, phase, domain, phase_ordinal, transaction_ordinal, peer:
            _evidence(domain, phase)
        ),
        monotonic=lambda: clock[0], utc_now=lambda: TIMESTAMP,
    )
    return machine, store


def test_observation_session_is_single_connection_bound_and_expires_at_1800(tmp_path):
    clock = [0.0]
    machine, store = _observation_machine(tmp_path, clock)
    peer = _peer()
    started = machine.handle(_request("begin_observation"), peer, "connection1")
    bind = {
        "authorization_identity": started["authorization_identity"],
        "observation_session_identity": started["observation_session_identity"],
        "observation_session_nonce": started["observation_session_nonce"],
    }
    with pytest.raises(Slice7GAuthorityDaemonError, match="observation_session_active"):
        machine.handle(_request("begin_observation"), peer, "connection1")
    assert not machine._observation_sessions
    started = machine.handle(_request("begin_observation"), peer, "connection1")
    bind = {
        "authorization_identity": started["authorization_identity"],
        "observation_session_identity": started["observation_session_identity"],
        "observation_session_nonce": started["observation_session_nonce"],
    }
    clock[0] = 1_799.999
    machine.handle(_request(
        "record_precommit_observation", **bind, domain_id=100,
    ), peer, "connection1")
    clock[0] = 1_800.0
    with pytest.raises(Slice7GAuthorityDaemonError, match="observation_session"):
        machine.handle(_request(
            "record_precommit_observation", **bind, domain_id=101,
        ), peer, "connection1")
    assert store.observe().record.data["state"] == "UNCONSUMED"
    store.close()


def test_prepare_requires_final_observation_and_expires_at_exact_300_seconds(tmp_path):
    clock = [0.0]
    machine, store = _observation_machine(tmp_path, clock)
    peer = _peer()
    with pytest.raises(Slice7GAuthorityDaemonError, match="observation_session"):
        machine.handle(_request("prepare"), peer, "connection1")
    prepared, bind = _observe_and_prepare(machine, peer)
    clock[0] = 300.0
    with pytest.raises(Slice7GAuthorityDaemonError, match="prepare_token"):
        machine.handle(_request("allocate_provisional", **bind), peer, "connection1")
    assert store.observe().record.data["state"] == "UNCONSUMED"
    assert not machine._observation_sessions
    store.close()


def test_observation_session_accepts_exactly_100_ordered_precommit_candidates(tmp_path):
    clock = [0.0]
    machine, store = _observation_machine(
        tmp_path, clock,
        provider=lambda session, phase, domain, phase_ordinal, transaction_ordinal, peer:
        _evidence(domain, phase, [] if domain == 199 else [f"/occupied_{domain}"]),
    )
    peer = _peer()
    started = machine.handle(_request("begin_observation"), peer, "connection1")
    bind = {
        "authorization_identity": started["authorization_identity"],
        "observation_session_identity": started["observation_session_identity"],
        "observation_session_nonce": started["observation_session_nonce"],
    }
    for count, domain in enumerate(range(100, 200), start=1):
        response = machine.handle(_request(
            "record_precommit_observation", **bind, domain_id=domain,
        ), peer, "connection1")
        assert response["precommit_observer_count"] == count
    final_bind = {
        **bind, "domain_id": 199,
        "process_manifest_identity": "2" * 64,
    }
    machine.handle(_request(
        "finalize_observation", **final_bind,
    ), peer, "connection1")
    prepared = machine.handle(_request("prepare", **final_bind), peer, "connection1")
    assert prepared["precommit_observer_count"] == 100
    assert prepared["transaction_observer_count"] == 100
    machine.disconnect("connection1")
    assert store.observe().record.data["state"] == "UNCONSUMED"
    store.close()


def test_daemon_constructs_session_bound_receipts_and_cross_session_replay_is_impossible(tmp_path):
    clock = [0.0]
    machine, store = _observation_machine(tmp_path, clock)
    peer = _peer()
    first = machine.handle(_request("begin_observation"), peer, "connection1")
    first_bind = {
        "authorization_identity": first["authorization_identity"],
        "observation_session_identity": first["observation_session_identity"],
        "observation_session_nonce": first["observation_session_nonce"],
    }
    machine.handle(
        _request("record_precommit_observation", **first_bind, domain_id=100),
        peer, "connection1",
    )
    first_session = machine._observation_sessions[first["observation_session_nonce"]]
    first_receipt = first_session.receipt_records[0]
    assert first_receipt.data["session_binding_identity"] == first_session.identity
    assert first_receipt.data["service_nonce"] == first_session.nonce
    machine.disconnect("connection1")

    second = machine.handle(_request("begin_observation"), peer, "connection2")
    second_bind = {
        "authorization_identity": second["authorization_identity"],
        "observation_session_identity": second["observation_session_identity"],
        "observation_session_nonce": second["observation_session_nonce"],
    }
    machine.handle(
        _request("record_precommit_observation", **second_bind, domain_id=100),
        peer, "connection2",
    )
    second_session = machine._observation_sessions[second["observation_session_nonce"]]
    second_receipt = second_session.receipt_records[0]
    assert second_receipt.logical_identity != first_receipt.logical_identity
    assert second_receipt.data["session_binding_identity"] != first_receipt.data["session_binding_identity"]
    store.close()


def test_provider_protocol_and_cleanup_failures_invalidate_active_session(tmp_path):
    cases = (
        (RuntimeError("provider"), "observation_provider_failed", False),
        (Slice7GAuthorityDaemonError("observer_cleanup_uncertain", "cleanup"),
         "observer_cleanup_uncertain", True),
    )
    for index, (failure, code, poisoned) in enumerate(cases):
        root = tmp_path / f"case-{index}"
        root.mkdir()
        clock = [0.0]
        machine, store = _observation_machine(
            root, clock,
            provider=lambda *args, failure=failure: (_ for _ in ()).throw(failure),
        )
        peer = _peer()
        started = machine.handle(_request("begin_observation"), peer, "connection1")
        bind = {
            "authorization_identity": started["authorization_identity"],
            "observation_session_identity": started["observation_session_identity"],
            "observation_session_nonce": started["observation_session_nonce"],
        }
        with pytest.raises(Slice7GAuthorityDaemonError, match=code):
            machine.handle(
                _request("record_precommit_observation", **bind, domain_id=100),
                peer, "connection1",
            )
        assert not machine._observation_sessions
        if poisoned:
            with pytest.raises(Slice7GAuthorityDaemonError, match="observation_cleanup_uncertain"):
                machine.handle(_request("begin_observation"), peer, "connection2")
        else:
            assert machine.handle(
                _request("begin_observation"), peer, "connection2",
            )["result"] == "OBSERVATION_STARTED"
        store.close()


def test_protocol_failure_invalidates_session_and_fabricated_authority_is_unknown(tmp_path):
    clock = [0.0]
    machine, store = _observation_machine(tmp_path, clock)
    peer = _peer()
    machine.handle(_request("begin_observation"), peer, "connection1")
    fabricated = _request("record_precommit_observation", domain_id=100)
    fabricated["ros_graph_observation_receipt"] = {"invented": True}
    with pytest.raises(protocol.Slice7GAuthorityProtocolError, match="authority_fields"):
        machine.handle(fabricated, peer, "connection1")
    assert not machine._observation_sessions
    store.close()


def test_provider_baseexception_is_preserved_after_session_invalidation(tmp_path):
    primary = KeyboardInterrupt()
    machine, store = _observation_machine(
        tmp_path, [0.0],
        provider=lambda *args: (_ for _ in ()).throw(primary),
    )
    peer = _peer()
    started = machine.handle(_request("begin_observation"), peer, "connection1")
    bind = {
        "authorization_identity": started["authorization_identity"],
        "observation_session_identity": started["observation_session_identity"],
        "observation_session_nonce": started["observation_session_nonce"],
    }
    with pytest.raises(KeyboardInterrupt) as raised:
        machine.handle(
            _request("record_precommit_observation", **bind, domain_id=100),
            peer, "connection1",
        )
    assert raised.value is primary
    assert not machine._observation_sessions
    store.close()


def test_immediate_observer_parent_exit_does_not_abandon_surviving_pgid_descendant():
    helper = subprocess.Popen(
        [
            "/usr/bin/python3", "-I", "-c",
            "import os,time; child=os.fork(); "
            "time.sleep(0.2) if child else time.sleep(60); "
            "os._exit(0)",
        ],
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, shell=False, close_fds=True,
        start_new_session=True,
    )
    pgid = helper.pid
    stat_fields = Path(f"/proc/{helper.pid}/stat").read_text(
        encoding="ascii",
    ).rsplit(") ", 1)[1].split()
    start_ticks = int(stat_fields[19])
    cgroup_lines = Path(f"/proc/{helper.pid}/cgroup").read_text(
        encoding="ascii",
    ).splitlines()
    assert len(cgroup_lines) == 1 and cgroup_lines[0].startswith("0::/")
    cgroup = cgroup_lines[0][3:]
    try:
        deadline = time.monotonic() + 2.0
        authenticated_members = ()
        while time.monotonic() < deadline:
            authenticated_members = _server_group_members(
                pgid, cgroup, start_ticks,
            )
            if len(authenticated_members) >= 2:
                break
            time.sleep(0.01)
        assert len(authenticated_members) >= 2
        helper.wait(timeout=2.0)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                continue
            break
        identity = _cleanup_server_observer_group(
            pgid, cgroup, start_ticks, 100, (), authenticated_members,
            port_observer=lambda domain: (),
        )
        assert len(identity) == 64
        with pytest.raises(ProcessLookupError):
            os.killpg(pgid, 0)
    finally:
        try:
            os.killpg(pgid, 9)
        except ProcessLookupError:
            pass


def test_cleanup_never_signals_a_numeric_pgid_with_wrong_provenance():
    helper = subprocess.Popen(
        ["/usr/bin/python3", "-I", "-c", "import time; time.sleep(60)"],
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, shell=False, close_fds=True,
        start_new_session=True,
    )
    try:
        stat_fields = Path(f"/proc/{helper.pid}/stat").read_text(
            encoding="ascii",
        ).rsplit(") ", 1)[1].split()
        start_ticks = int(stat_fields[19])
        cgroup_line = Path(f"/proc/{helper.pid}/cgroup").read_text(
            encoding="ascii",
        ).strip()
        assert cgroup_line.startswith("0::/")
        with pytest.raises(
            Slice7GAuthorityDaemonError, match="observer_process_ownership",
        ):
            _cleanup_server_observer_group(
                helper.pid, cgroup_line[3:], start_ticks, 100, (), (),
                port_observer=lambda domain: (),
            )
        assert helper.poll() is None
    finally:
        try:
            os.killpg(helper.pid, 9)
        except ProcessLookupError:
            pass
        helper.wait(timeout=2.0)


def test_prepare_allocation_commit_and_finalization_are_connection_bound(tmp_path):
    root = provision(tmp_path)
    store = GlobalAttemptBudgetStore._for_test(str(root))
    events = []

    def provisioner(prepared, domain):
        return ProvisionalAllocation(
            prepared.campaign_id, prepared.campaign_identity, domain,
            protocol.OUTPUT_PARENT + "/campaign-" + prepared.campaign_id, "5" * 64,
            lambda: events.append("rollback"), lambda: events.append("barrier"),
            lambda: events.append("close"),
        )

    machine = RuntimeAuthorityStateMachine._for_test(
        bootstrap=_bootstrap(), authorization=_authorization(store), budget=store,
        service_instance_identity="6" * 64, peer_matcher=lambda peer, request: None,
        peer_reconciler=lambda peer: peer, provisioner=provisioner,
        process_instance_validator=lambda prepared, allocation, peer, identity: (
            None if identity == "7" * 64 else (_ for _ in ()).throw(AssertionError(identity))
        ),
        observation_provider=lambda session, phase, domain, phase_ordinal, transaction_ordinal, peer: _evidence(domain, phase),
        monotonic=lambda: 10.0, utc_now=lambda: TIMESTAMP,
    )
    peer = _peer()
    try:
        prepared, bind = _observe_and_prepare(machine, peer)
        allocated = machine.handle(
            _request("allocate_provisional", **bind), peer, "connection1",
        )
        commit_bind = {
            **bind, "output_root_path": allocated["output_root_path"],
            "output_root_identity": allocated["output_root_identity"],
            "process_manifest_identity": "2" * 64, "process_instance_identity": "7" * 64,
        }
        with pytest.raises(Slice7GAuthorityDaemonError, match="prepare_binding"):
            machine.handle(_request("commit", **commit_bind), peer, "connection2")
        committed = machine.handle(_request("commit", **commit_bind), peer, "connection1")
        assert committed["result"] == "COMMITTED"
        assert store.observe().record.data["attempts_consumed"] == 1
        postcommit_bind = _record_postcommit(machine, peer, commit_bind)
        completed = machine.handle(_request("complete", **postcommit_bind), peer, "connection1")
        assert completed["result"] == "COMPLETED"
        assert events == ["barrier", "close"]
    finally:
        store.close()


def test_precommit_disconnect_and_revocation_rollback_owned_allocation(tmp_path):
    for method in ("disconnect", "revoke"):
        root = tmp_path / method
        GlobalAttemptBudgetStore._provision_test_root(str(root), TIMESTAMP)
        for relative in ("revocation/records", "revocation/pending", "revocation/processed", "receipts"):
            (root / relative).mkdir(mode=0o700, parents=True, exist_ok=True)
        store = GlobalAttemptBudgetStore._for_test(str(root))
        events = []
        machine = RuntimeAuthorityStateMachine._for_test(
            bootstrap=_bootstrap(), authorization=_authorization(store), budget=store,
            service_instance_identity="6" * 64, peer_matcher=lambda peer, request: None,
            peer_reconciler=lambda peer: peer,
            provisioner=lambda prepared, domain: ProvisionalAllocation(
                prepared.campaign_id, prepared.campaign_identity, domain,
                protocol.OUTPUT_PARENT + "/campaign-" + prepared.campaign_id, "5" * 64,
                lambda: events.append("rollback"), lambda: None, lambda: events.append("close"),
            ),
            observation_provider=lambda session, phase, domain, phase_ordinal, transaction_ordinal, peer: _evidence(domain, phase),
            monotonic=lambda: 10.0, utc_now=lambda: TIMESTAMP,
        )
        peer = _peer()
        prepared, bind = _observe_and_prepare(machine, peer)
        machine.handle(_request("allocate_provisional", **bind), peer, "connection1")
        if method == "disconnect":
            machine.disconnect("connection1")
        else:
            machine.handle(_request("revoke", authorization_identity=prepared["authorization_identity"]), _peer(0, 0), "root")
        assert events == ["rollback", "close"]
        assert store.observe().record.data["state"] == "UNCONSUMED"
        store.close()


def _process_manifest():
    return {
        "schema_version": protocol.PROCESS_MANIFEST_SCHEMA,
        "identity_algorithm": "sha256:ctr-slice-7g-process-manifest-canonical-1",
        "interpreter": _file("/usr/bin/python3.10", 201), "interpreter_flags": ["-I"],
        "entrypoint": _file("/opt/ctr-mppi/slice-7g/" + "a" * 64 + "/campaign.py", 202),
        "executables": [_file("/opt/ros/humble/bin/ros2", 203)],
        "argv_template": [
            "/usr/bin/python3.10", "-I", "/opt/ctr-mppi/slice-7g/" + "a" * 64 + "/campaign.py",
        ],
        "transaction_slots": {}, "environment_manifest_identity": "b" * 64,
        "working_directory": "/opt/ctr-mppi/slice-7g/" + "a" * 64,
        "shell": False, "systemd_unit": protocol.CAMPAIGN_SYSTEMD_UNIT,
        "cgroup": "/system.slice/ctr-slice7g-campaign.service",
        "allowed_descendants": [{
            "role": "ros_launch", "executable_identity": "c" * 64,
            "parent_role": "coordinator", "multiplicity": 1,
        }],
        "timeouts": {
            "sigint_seconds": 1.0, "sigterm_seconds": 2.0,
            "sigkill_seconds": 3.0, "cell_seconds": 25.0,
        },
        "output_ownership": {
            "authority_owner": protocol.AUTHORITY_ACCOUNT,
            "runtime_group": protocol.RUNTIME_GROUP,
            "campaign_account": protocol.CAMPAIGN_ACCOUNT,
            "root_mode": 0o750, "cell_mode": 0o770,
            "stdout_role": "authority/stdout.bin", "stderr_role": "authority/stderr.bin",
        },
        "required_receipts": ["cleanup", "process_start"],
    }


def test_postcommit_revocation_is_processed_once_with_durable_termination_identity(tmp_path):
    root = provision(tmp_path)
    store = GlobalAttemptBudgetStore._for_test(str(root))
    machine = RuntimeAuthorityStateMachine._for_test(
        bootstrap=_bootstrap(), authorization=_authorization(store), budget=store,
        service_instance_identity="6" * 64, peer_matcher=lambda peer, request: None,
        peer_reconciler=lambda peer: peer,
        provisioner=lambda prepared, domain: ProvisionalAllocation(
            prepared.campaign_id, prepared.campaign_identity, domain,
            protocol.OUTPUT_PARENT + "/campaign-" + prepared.campaign_id, "5" * 64,
            lambda: None, lambda: None, lambda: None,
        ),
        process_instance_validator=lambda *args: None,
        observation_provider=lambda session, phase, domain, phase_ordinal, transaction_ordinal, peer: _evidence(domain, phase),
        monotonic=lambda: 10.0, utc_now=lambda: TIMESTAMP,
    )
    peer = _peer()
    prepared, bind = _observe_and_prepare(machine, peer)
    allocated = machine.handle(
        _request("allocate_provisional", **bind), peer, "connection1",
    )
    committed_binding = {
        **bind, "output_root_path": allocated["output_root_path"],
        "output_root_identity": allocated["output_root_identity"],
        "process_manifest_identity": "2" * 64,
        "process_instance_identity": "7" * 64,
    }
    machine.handle(_request("commit", **committed_binding), peer, "connection1")
    machine.handle(
        _request("revoke", authorization_identity=prepared["authorization_identity"]),
        _peer(0, 0), "root",
    )
    calls = []

    def runner(argv, **kwargs):
        calls.append((argv, kwargs))
        return SimpleNamespace(returncode=0, stdout=b"", stderr=b"")

    processed = _enforce_pending_revocations_for_test(
        str(root), os.geteuid(), _process_manifest(), runner, lambda: TIMESTAMP,
    )
    assert processed == 1
    assert len(calls) == 1
    assert calls[0][0] == ["/usr/bin/systemctl", "stop", "ctr-slice7g-campaign.service"]
    assert calls[0][1]["timeout"] == 6.0
    assert not list((root / "revocation/pending").iterdir())
    records = list((root / "revocation/processed").iterdir())
    assert len(records) == 1
    record = protocol.validate_authority_record(
        records[0].read_bytes(), expected_schema=protocol.AUTHORITY_REVOCATION_SCHEMA,
    )
    assert record.data["state"] == "ENFORCED_POSTCOMMIT"
    assert record.data["processed_trigger_identity"] is not None
    assert record.data["termination_receipt_identity"] is not None
    assert store.observe().record.data["state"] == "COMMITTED"
    machine.disconnect("connection1")
    assert store.observe().record.data["state"] == "FAILED_AFTER_COMMIT"
    store.close()


class _TransactionSession:
    def __init__(self, events, fail_method=None, failure=None):
        self.events = events
        self.fail_method = fail_method
        self.failure = failure

    def __enter__(self):
        self.events.append("session_enter")
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.events.append("session_exit")

    def exchange(self, request):
        method = request["method"]
        self.events.append(method)
        if method == self.fail_method:
            raise self.failure
        observed = method not in {"begin_observation"}
        postcommit = method in {"record_postcommit_observation", "complete"}
        common = {
            "authorization_identity": "a" * 64, "prepare_token": "prepare1",
            "campaign_id": "campaign1", "campaign_identity": "b" * 64,
            "campaign_template_identity": "c" * 64,
            "observation_session_identity": "8" * 64,
            "observation_session_nonce": "observationnonce1",
            "four_source_observation_identity": "9" * 64 if observed else None,
            "precommit_receipt_identities": ["f" * 64] if observed else [],
            "precommit_observer_count": 1 if observed else 0,
            "postcommit_observer_count": 1 if postcommit else 0,
            "transaction_observer_count": (2 if postcommit else (1 if observed else 0)),
            "lease_identity": "7" * 64 if observed else None,
            "domain_id": request["domain_id"] if request["domain_id"] is not None else (100 if observed else None),
            "output_root_path": request["output_root_path"],
            "output_root_identity": request["output_root_identity"],
            "candidate_clear": True if method == "record_precommit_observation" else None,
        }
        if method == "begin_observation":
            return SimpleNamespace(data={
                "result": "OBSERVATION_STARTED", **common,
                "observation_session_identity": "8" * 64,
                "observation_session_nonce": "observationnonce1",
            })
        if method == "record_precommit_observation":
            return SimpleNamespace(data={"result": "OBSERVATION_RECORDED", **common})
        if method == "finalize_observation":
            return SimpleNamespace(data={"result": "OBSERVATION_COMPLETE", **common})
        if method == "prepare":
            return SimpleNamespace(data={"result": "PREPARED", **common})
        if method == "allocate_provisional":
            return SimpleNamespace(data={
                "result": "PREPARED", **common,
                "output_root_path": protocol.OUTPUT_PARENT + "/campaign-campaign1",
                "output_root_identity": "d" * 64,
            })
        results = {
            "commit": "COMMITTED", "complete": "COMPLETED",
            "cancel": "CANCELLED", "fail_after_commit": "FAILED_AFTER_COMMIT",
            "record_postcommit_observation": "POSTCOMMIT_RECORDED",
        }
        return SimpleNamespace(data={"result": results[method]})


def test_authority_transaction_precommit_baseexception_cancels_and_preserves_primary():
    from ctr_evaluation.slice_7g_runtime import Slice7GAuthorityTransaction

    events = []
    primary = KeyboardInterrupt()
    transaction = Slice7GAuthorityTransaction._for_test(
        session_factory=lambda: _TransactionSession(
            events, "record_precommit_observation", primary,
        ),
        nonconsuming_preflight=lambda binding: None,
        process_instance_builder=lambda binding: ("e" * 64, "f" * 64),
        postcommit_domain_recheck=lambda binding: events.append("recheck"),
        execute_committed_campaign=lambda binding, receipt: events.append("execute"),
        cleanup=lambda binding, committed: events.append(("cleanup", committed)),
        timestamp_factory=lambda: TIMESTAMP,
    )
    with pytest.raises(KeyboardInterrupt) as observed:
        transaction.run()
    assert observed.value is primary
    assert events == [
        "session_enter", "begin_observation", "record_precommit_observation",
        "cancel", "session_exit",
        ("cleanup", False),
    ]


def test_authority_transaction_postcommit_failure_stays_consumed_and_runs_cleanup():
    from ctr_evaluation.slice_7g_runtime import Slice7GAuthorityTransaction

    events = []
    transaction = Slice7GAuthorityTransaction._for_test(
        session_factory=lambda: _TransactionSession(
            events, "record_postcommit_observation", RuntimeError("occupied"),
        ),
        nonconsuming_preflight=lambda binding: None,
        process_instance_builder=lambda binding: ("e" * 64, "f" * 64),
        postcommit_domain_recheck=lambda binding: None,
        execute_committed_campaign=lambda binding, receipt: events.append("execute"),
        cleanup=lambda binding, committed: events.append(("cleanup", committed)),
        timestamp_factory=lambda: TIMESTAMP,
    )
    with pytest.raises(RuntimeError, match="occupied"):
        transaction.run()
    assert events == [
        "session_enter", "begin_observation", "record_precommit_observation",
        "finalize_observation", "prepare", "allocate_provisional", "commit",
        "record_postcommit_observation", "fail_after_commit", "session_exit",
        ("cleanup", True),
    ]


def test_authority_transaction_orders_postobservation_prepare_and_postcommit_recheck():
    from ctr_evaluation.slice_7g_runtime import Slice7GAuthorityTransaction

    events = []
    transaction = Slice7GAuthorityTransaction._for_test(
        session_factory=lambda: _TransactionSession(events),
        nonconsuming_preflight=lambda binding: None,
        process_instance_builder=lambda binding: ("e" * 64, "f" * 64),
        postcommit_domain_recheck=lambda binding: None,
        execute_committed_campaign=lambda binding, receipt: events.append("campaign_child"),
        cleanup=lambda binding, committed: events.append(("cleanup", committed)),
        timestamp_factory=lambda: TIMESTAMP,
    )
    transaction.run()
    assert events == [
        "session_enter", "begin_observation", "record_precommit_observation",
        "finalize_observation", "prepare", "allocate_provisional", "commit",
        "record_postcommit_observation", "campaign_child", "complete", "session_exit",
        ("cleanup", True),
    ]
