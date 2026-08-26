from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path

import pytest

from ctr_evaluation.slice_7g_cleanup_authority import (
    CleanupAuthorityLedger,
    CleanupRecoveryController,
    RecoveryProviderEvidence,
    Slice7GCleanupAuthorityError,
)
from ctr_evaluation.slice_7g_privileged_protocol import (
    CLEANUP_RECOVERY_AUTHORIZATION_V2_SCHEMA,
    PRIVILEGED_REQUEST_SCHEMA,
    record_identity,
    validate_record,
)


HEX = "a" * 64


@pytest.fixture
def ledger_root(tmp_path):
    root = tmp_path / "cleanup"
    CleanupAuthorityLedger._provision_test_root(str(root))
    return root


def _begin(ledger):
    return ledger.begin_unbound(
        runtime_authorization_identity=HEX,
        budget_identity="b" * 64,
        service_generation_identity="c" * 64,
        session_binding_identity="d" * 64,
        phase="PRECOMMIT",
        phase_local_ordinal=1,
        transaction_observer_ordinal=1,
        domain_id=100,
        observer_contract_identity="e" * 64,
        timestamp="2026-08-23T00:00:01Z",
    )


def test_cleanup_triples_are_root_contract_modes_and_contiguous(ledger_root):
    with CleanupAuthorityLedger._for_test(str(ledger_root)) as ledger:
        initial = ledger.reconstruct()
        assert initial.state == "CLEARED"
        active = _begin(ledger)
        bound = ledger.bind(
            active, containment_identity="f" * 64,
            process_identity="1" * 64, timestamp="2026-08-23T00:00:02Z",
        )
        cleared = ledger.terminate(
            bound, state="CLEARED", disposition_identity="2" * 64,
            timestamp="2026-08-23T00:00:03Z",
        )
        assert cleared.revision.data["revision"] == 3
    for directory, prefix in (
        ("revisions", "revision"), ("anchors", "anchor"), ("heads", "head"),
    ):
        names = sorted((ledger_root / directory).iterdir())
        assert [item.name for item in names] == [
            f"{prefix}-{number:020d}.json" for number in range(4)
        ]
        assert all((item.stat().st_mode & 0o777) == 0o400 for item in names)


def test_same_byte_revision_replacement_is_rejected(ledger_root):
    ledger = CleanupAuthorityLedger._for_test(str(ledger_root))
    path = ledger_root / "revisions" / "revision-00000000000000000000.json"
    raw = path.read_bytes()
    replacement = ledger_root / "revisions" / "replacement"
    replacement.write_bytes(raw)
    replacement.chmod(0o400)
    os.replace(replacement, path)
    try:
        with pytest.raises(Slice7GCleanupAuthorityError) as caught:
            ledger.reconstruct()
        assert caught.value.code in {"cleanup_same_byte_replacement", "cleanup_anchor"}
    finally:
        ledger.close()


@pytest.mark.parametrize("kind", ["anchor", "head"])
def test_same_byte_anchor_or_head_replacement_is_rejected_while_retained(ledger_root, kind):
    ledger = CleanupAuthorityLedger._for_test(str(ledger_root))
    directory = ledger_root / (kind + "s")
    path = directory / f"{kind}-00000000000000000000.json"
    raw = path.read_bytes()
    replacement = directory / "replacement"
    replacement.write_bytes(raw)
    replacement.chmod(0o400)
    os.replace(replacement, path)
    try:
        with pytest.raises(Slice7GCleanupAuthorityError):
            ledger.reconstruct()
    finally:
        ledger.close()


def test_revision_gap_and_extra_inventory_fail_closed(ledger_root):
    extra = ledger_root / "heads" / "head-00000000000000000002.json"
    extra.write_bytes((ledger_root / "heads" / "head-00000000000000000000.json").read_bytes())
    extra.chmod(0o400)
    with pytest.raises(Slice7GCleanupAuthorityError) as caught:
        CleanupAuthorityLedger._for_test(str(ledger_root))
    assert caught.value.code == "cleanup_inventory"


def test_symlink_and_hardlink_alias_fail_closed(ledger_root):
    source = ledger_root / "heads" / "head-00000000000000000000.json"
    os.link(source, ledger_root / "heads" / "head-00000000000000000001.json")
    with pytest.raises(Slice7GCleanupAuthorityError):
        CleanupAuthorityLedger._for_test(str(ledger_root))


def test_busy_lock_fails_closed(ledger_root):
    descriptor = os.open(ledger_root / "ledger.lock", os.O_RDWR)
    ledger = CleanupAuthorityLedger._for_test(str(ledger_root))
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(Slice7GCleanupAuthorityError) as caught:
            ledger.reconstruct()
        assert caught.value.code == "cleanup_busy"
    finally:
        fcntl.flock(descriptor, fcntl.LOCK_UN)
        os.close(descriptor)
        ledger.close()


def test_active_and_quarantined_survive_reconstruction_and_block(ledger_root):
    with CleanupAuthorityLedger._for_test(str(ledger_root)) as first:
        active = _begin(first)
        first.terminate(
            active, state="QUARANTINED", disposition_identity="f" * 64,
            timestamp="2026-08-23T00:00:02Z",
        )
    with CleanupAuthorityLedger._for_test(str(ledger_root)) as second:
        assert second.reconstruct().state == "QUARANTINED"
        with pytest.raises(Slice7GCleanupAuthorityError) as caught:
            second.require_clear()
        assert caught.value.code == "cleanup_blocked"


def test_cleanup_guard_is_non_budget_and_has_no_consumption_field(ledger_root):
    with CleanupAuthorityLedger._for_test(str(ledger_root)) as ledger:
        fields = set(ledger.reconstruct().revision.data)
    assert "attempts_consumed" not in fields
    assert "attempts_maximum" not in fields


def test_predecessor_fork_is_rejected(ledger_root):
    with CleanupAuthorityLedger._for_test(str(ledger_root)) as ledger:
        current = ledger.reconstruct()
        value = dict(current.revision.data)
        value.update({
            "revision": 1,
            "predecessor_identity": "f" * 64,
            "state": "ACTIVE_UNBOUND",
        })
        with pytest.raises(Slice7GCleanupAuthorityError) as caught:
            ledger.append(value)
        assert caught.value.code == "cleanup_predecessor"


def _quarantine(ledger):
    active = _begin(ledger)
    return ledger.terminate(
        active, state="QUARANTINED", disposition_identity="f" * 64,
        timestamp="2026-08-23T00:00:02Z",
    )


def _recovery_request(identity, operation, token="e" * 32):
    return validate_record({
        "schema_version": PRIVILEGED_REQUEST_SCHEMA,
        "operation": operation,
        "sequence": 0,
        "connection_nonce": "a" * 32,
        "request_nonce": "b" * 32,
        "operation_token": token,
        "service_generation_identity": "c" * 64,
        "runtime_authorization_identity": HEX,
        "installed_runtime_identity": "d" * 64,
        "budget_identity": "b" * 64,
        "cleanup_head_identity": None,
        "session_binding_identity": None,
        "domain_id": 100,
        "phase": "RECOVERY",
        "phase_local_ordinal": 1,
        "transaction_observer_ordinal": 1,
        "transition": None,
        "observer_contract_identity": None,
        "containment_identity": None,
        "process_identity": None,
        "disposition_identity": None,
        "recovery_authorization_identity": identity,
    }, expected_schema=PRIVILEGED_REQUEST_SCHEMA)


def _controller(ledger, quarantine, residual_provider=None):
    auth = {
        "schema_version": CLEANUP_RECOVERY_AUTHORIZATION_V2_SCHEMA,
        "recovery_nonce": "f" * 32,
        "quarantine_head_identity": quarantine.head.logical_identity,
        "quarantine_anchor_identity": quarantine.anchor.logical_identity,
        "runtime_authorization_identity": HEX,
        "installed_runtime_identity": "d" * 64,
        "budget_identity": "b" * 64,
        "cleanup_service_generation_identity": "c" * 64,
        "observer_service_generation_identity": "e" * 64,
        "issued_at_utc": "2026-08-23T00:00:03Z",
        "not_before_utc": "2026-08-23T00:00:00Z",
        "not_after_utc": "2026-08-24T00:00:00Z",
        "one_shot": True,
    }
    identity = record_identity(auth, expected_schema=CLEANUP_RECOVERY_AUTHORIZATION_V2_SCHEMA)

    def result(name):
        residual = 1 if residual_provider == name else 0
        evidence_identities = {
            "process": "1" * 64,
            "dds": "2" * 64,
            "lease": "3" * 64,
            "graph": "4" * 64,
        }
        return RecoveryProviderEvidence(
            name, evidence_identities[name], residual,
            "CLEAR" if name == "lease" else None,
            1, 2, "9" * 64,
        )

    providers = {
        name: (lambda _authorization, name=name: result(name))
        for name in ("process", "dds", "lease", "graph")
    }
    controller = CleanupRecoveryController(
        ledger, authorization_loader=lambda requested: dict(auth),
        providers=providers, service_generation_identity="c" * 64,
        utc_now=lambda: "2026-08-23T00:00:04Z",
    )
    return controller, identity


def test_daemon_owned_four_source_recovery_creates_one_successor(ledger_root):
    with CleanupAuthorityLedger._for_test(str(ledger_root)) as ledger:
        quarantine = _quarantine(ledger)
        controller, identity = _controller(ledger, quarantine)
        controller.observe(_recovery_request(identity, "RECOVERY_OBSERVE"))
        recovered = controller.commit(_recovery_request(identity, "RECOVERY_COMMIT"))
        assert recovered.state == "RECOVERED"
        assert "attempts_consumed" not in recovered.revision.data


@pytest.mark.parametrize("provider", ["process", "dds", "lease", "graph"])
def test_each_recovery_residual_source_blocks(ledger_root, provider):
    with CleanupAuthorityLedger._for_test(str(ledger_root)) as ledger:
        quarantine = _quarantine(ledger)
        controller, identity = _controller(ledger, quarantine, residual_provider=provider)
        with pytest.raises(Slice7GCleanupAuthorityError) as caught:
            controller.observe(_recovery_request(identity, "RECOVERY_OBSERVE"))
        assert caught.value.code == "recovery_residual"


def test_recovery_commit_replay_is_rejected(ledger_root):
    with CleanupAuthorityLedger._for_test(str(ledger_root)) as ledger:
        quarantine = _quarantine(ledger)
        controller, identity = _controller(ledger, quarantine)
        observed = _recovery_request(identity, "RECOVERY_OBSERVE")
        committed = _recovery_request(identity, "RECOVERY_COMMIT")
        controller.observe(observed)
        controller.commit(committed)
        with pytest.raises(Slice7GCleanupAuthorityError) as caught:
            controller.commit(committed)
        assert caught.value.code == "recovery_replay"


def test_boolean_only_recovery_has_no_public_schema_field():
    request = dict(_recovery_request("a" * 64, "RECOVERY_OBSERVE").data)
    request["process_clear"] = True
    with pytest.raises(Exception):
        validate_record(request, expected_schema=PRIVILEGED_REQUEST_SCHEMA)


def test_direct_structural_recovery_cannot_confer_authority(ledger_root):
    with CleanupAuthorityLedger._for_test(str(ledger_root)) as ledger:
        _quarantine(ledger)
        with pytest.raises(TypeError):
            ledger._recover_authenticated({}, [], {}, timestamp="2026-08-23T00:00:04Z")
