import ast
import gc
import hashlib
import inspect
import json
import os
from pathlib import Path
import shutil
import stat
import struct
import sys
import threading
from dataclasses import replace
from unittest import mock

import pytest


REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "ctr_bringup"))
sys.path.insert(0, str(REPO / "src" / "ctr_evaluation"))

from ctr_bringup.slice_7g_governance import (  # noqa: E402
    create_slice_7g_initial_attempt_ledger,
    generate_slice_7g_campaign_plan,
    load_slice_7g_charter,
    propose_slice_7g_attempt_event,
    slice_7g_attempt_ledger_identity,
    slice_7g_campaign_plan_identity,
    slice_7g_charter_identity,
    validate_slice_7g_attempt_transition,
)
import ctr_bringup.slice_7g_governance as governance  # noqa: E402
import ctr_evaluation.slice_7g_runtime as runtime_module  # noqa: E402
from ctr_evaluation.slice_7g_runtime import (  # noqa: E402
    AtomicSlice7GLedgerWriter,
    AtomicSlice7GOutputAllocator,
    ProductionSlice7GDomainAuthority,
    ProductionSlice7GRunnerAdapter,
    Slice7GROSGraphObserverContract,
    Slice7GROSGraphObserverExecution,
    Slice7GProcessObservation,
    Slice7GProductionEffects,
    Slice7GCellExecution,
    Slice7GCampaignCoordinator,
    Slice7GDomainAllocator,
    Slice7GDomainOccupancy,
    Slice7GEvidenceWriter,
    Slice7GReadinessResult,
    Slice7GReadinessTracker,
    Slice7GRuntimeError,
    Slice7GCoordinatedFailure,
    Slice7GPostImplementationSourceSnapshot,
    Slice7GSourceSnapshotInspection,
    Slice7GSourceSnapshotMember,
    PROCESS_OUTPUT_RECEIPT_PATH,
    PROCESS_STDERR_PATH,
    PROCESS_STDOUT_PATH,
    POST_IMPLEMENTATION_SNAPSHOT_LOGICAL_ALGORITHM,
    POST_IMPLEMENTATION_SNAPSHOT_SCHEMA,
    POST_IMPLEMENTATION_SNAPSHOT_V1_SCHEMA,
    GLOBAL_DOMAIN_LEASE_REGISTRY_NAME,
    RUNNER_RECEIPT_PATH,
    _CELL_OUTPUT_LIMITS,
    _CellOutputAccounting,
    _CellOutputAuthority,
    _CellOutputLimits,
    _stream_cell_output_descriptor,
    _assemble_slice_7g_production_coordinator,
    _load_slice_7g_runtime_authorization_v1_for_test,
    assemble_slice_7g_production_coordinator,
    canonical_post_implementation_source_snapshot_bytes,
    canonical_runtime_authorization_bytes,
    cell_result_from_summary,
    inspect_post_implementation_source_snapshot,
    load_slice_7g_runtime_authorization,
    parse_post_implementation_source_snapshot,
    post_implementation_source_snapshot,
    post_implementation_source_snapshot_identity,
    structural_post_implementation_source_snapshot,
    discover_post_implementation_snapshot_members,
    verify_post_implementation_source_snapshot,
)
from ctr_evaluation.run_evaluation import (  # noqa: E402
    OrchestrationError,
    build_base_simulation_command,
    run_environment,
    parse_args,
    unexpected_command_publishers,
    validate_slice_7g_runtime_binding,
    write_slice_7g_runner_receipt,
)


CHARTER_PATH = REPO / "config/slice_7g_simulation_charter.json"
STAMP = "2026-08-19T00:00:00Z"


@pytest.fixture(autouse=True)
def _restore_slice_7g_governance_globals():
    """Make every runtime test hermetic with respect to governance authority."""

    event_globals = propose_slice_7g_attempt_event.__globals__
    original_module_parent = governance.SLICE_7G_EVIDENCE_PARENT
    original_event_parent = event_globals["SLICE_7G_EVIDENCE_PARENT"]
    try:
        yield
    finally:
        governance.SLICE_7G_EVIDENCE_PARENT = original_module_parent
        event_globals["SLICE_7G_EVIDENCE_PARENT"] = original_event_parent


def test_runtime_governance_global_restoration_fixture_restores_normal_exit(tmp_path):
    original = governance.SLICE_7G_EVIDENCE_PARENT
    fixture = _restore_slice_7g_governance_globals.__wrapped__()
    next(fixture)
    governance.SLICE_7G_EVIDENCE_PARENT = str(tmp_path)
    propose_slice_7g_attempt_event.__globals__["SLICE_7G_EVIDENCE_PARENT"] = str(tmp_path)
    with pytest.raises(StopIteration):
        next(fixture)
    assert governance.SLICE_7G_EVIDENCE_PARENT == original
    assert propose_slice_7g_attempt_event.__globals__["SLICE_7G_EVIDENCE_PARENT"] == original


def test_runtime_governance_global_restoration_fixture_restores_baseexception(tmp_path):
    original = governance.SLICE_7G_EVIDENCE_PARENT
    fixture = _restore_slice_7g_governance_globals.__wrapped__()
    next(fixture)
    governance.SLICE_7G_EVIDENCE_PARENT = str(tmp_path)
    propose_slice_7g_attempt_event.__globals__["SLICE_7G_EVIDENCE_PARENT"] = str(tmp_path)

    class TestAbort(BaseException):
        pass

    with pytest.raises(TestAbort):
        fixture.throw(TestAbort())
    assert governance.SLICE_7G_EVIDENCE_PARENT == original
    assert propose_slice_7g_attempt_event.__globals__["SLICE_7G_EVIDENCE_PARENT"] == original


def _occupancy(domain, *, clear=True):
    return Slice7GDomainOccupancy(
        domain, clear, clear, clear, clear, STAMP,
        hashlib.sha256(f"occupancy:{domain}:{clear}".encode()).hexdigest(),
    )


def _charter():
    return load_slice_7g_charter(CHARTER_PATH)


def _authorization(tmp_path, charter=None):
    governance.SLICE_7G_EVIDENCE_PARENT = str(tmp_path)
    propose_slice_7g_attempt_event.__globals__["SLICE_7G_EVIDENCE_PARENT"] = str(tmp_path)
    charter = charter or _charter()
    campaign_id = "campaign-001"
    initial = create_slice_7g_initial_attempt_ledger(charter, campaign_id)
    data = {
        "schema_version": "ctr-slice-7g-runtime-authorization-1",
        "charter_logical_identity": slice_7g_charter_identity(charter),
        "campaign_id": campaign_id,
        "campaign_identity": initial.campaign_identity,
        "post_implementation_source_snapshot_identity": "a" * 64,
        "campaign_output_root": str(tmp_path / "campaign"),
        "issued_at_utc": STAMP,
        "execution_authorized": True,
    }
    path = tmp_path / "authorization.json"
    path.write_bytes(canonical_runtime_authorization_bytes(data))
    path.chmod(0o444)
    return _load_slice_7g_runtime_authorization_v1_for_test(path, charter), initial


def _committed_context(tmp_path):
    governance.SLICE_7G_EVIDENCE_PARENT = str(tmp_path)
    propose_slice_7g_attempt_event.__globals__["SLICE_7G_EVIDENCE_PARENT"] = str(tmp_path)
    charter = _charter()
    authorization, initial = _authorization(tmp_path, charter)
    allocation = propose_slice_7g_attempt_event(
        initial, "domain_and_output_allocated", "allocation", STAMP,
        domain_id=117,
        output_root=authorization.campaign_output_root,
        runtime_authorization_identity=authorization.identity,
    )
    allocated = validate_slice_7g_attempt_transition(initial, allocation)
    plan = generate_slice_7g_campaign_plan(charter, allocated)
    start = propose_slice_7g_attempt_event(
        allocated, "process_start_commit", "process-start", STAMP, campaign_plan=plan,
    )
    committed = validate_slice_7g_attempt_transition(allocated, start, campaign_plan=plan)
    return charter, authorization, initial, allocated, plan, committed


def test_v1_authorization_is_historical_only_and_public_runtime_rejects_it(tmp_path):
    charter = _charter()
    _authorization(tmp_path, charter)
    with pytest.raises(Slice7GRuntimeError) as raised:
        load_slice_7g_runtime_authorization(tmp_path / "authorization.json", charter)
    assert raised.value.code == "runtime_authorization_v1_historical_only"


def _summary(*, deadline=0.0, safety_faults=0, collision=0):
    return {
        "tracking": {"steady_state_error": 0.001},
        "goal": {"final_goal_error": 0.001, "goal_hold_duration": 0.6},
        "control": {"saturation_percentage": 0.0},
        "timing": {"deadline_overrun_percentage": deadline},
        "numerical_safety": {
            "nonfinite_state_samples": 0,
            "nonfinite_reference_samples": 0,
            "nonfinite_command_samples": 0,
            "missing_required_topic_count": 0,
        },
        "data_quality": {"valid_aligned_sample_count": 20, "rejected_aligned_sample_count": 0},
        "lumen_evaluation": {
            "physical_safety": {
                "minimum_physical_clearance_m": 0.003,
                "collision_sample_count": collision,
            },
            "safety_margin": {"minimum_safety_clearance_m": 0.002},
        },
        "slice_7g_safety": {"fault_count": safety_faults},
        "slice_7g_tactile": {"invalid_sample_count": 0},
        "missing_required_result_file_count": 0,
    }


def _execution(cell, plan, ledger, *, deadline=0.0, collision=0):
    readiness = Slice7GReadinessResult(True, "", 10, 0.5, 0.0, 0.0, 0.01, 0.01)
    result = cell_result_from_summary(
        cell=cell, plan=plan, ledger=ledger, summary=_summary(deadline=deadline, collision=collision),
        readiness=readiness, process_exit_status=0,
    )
    return Slice7GCellExecution(
        result,
        {"argv": list(cell.argv), "process_exit_status": 0},
        {
            "readiness_success": True, "stable_sample_count": 10,
            "stable_interval_seconds": 0.5, "q_variation": 0.0, "tip_variation_m": 0.0,
        },
        {
            "minimum_physical_wall_clearance_m": 0.003,
            "minimum_safety_margin_wall_clearance_m": 0.002,
            "collision_sample_count": collision, "safety_fault_count": 0, "nonfinite_value_count": 0,
        },
        {
            "valid_aligned_sample_count": 20, "invalid_sample_count": 0,
            "invalid_sample_percentage": 0.0, "saturation_percentage": 0.0,
            "missing_required_topic_count": 0,
        },
        {
            "missing_required_result_file_count": 0, "output_tree_identity": "b" * 64,
            "regular_file_count": 1, "regular_file_bytes": 1,
        },
    )


def test_cell_summary_requires_explicit_missing_result_accounting(tmp_path):
    _, _, _, _, plan, committed = _committed_context(tmp_path)
    summary = _summary()
    del summary["missing_required_result_file_count"]
    readiness = Slice7GReadinessResult(True, "", 10, 0.5, 0.0, 0.0, 0.01, 0.01)
    with pytest.raises(Slice7GRuntimeError) as raised:
        cell_result_from_summary(
            cell=plan.cells[0],
            plan=plan,
            ledger=committed,
            summary=summary,
            readiness=readiness,
            process_exit_status=0,
        )
    assert raised.value.code == "cell_summary_missing_results"


def test_cell_summary_missing_nested_metric_has_stable_error(tmp_path):
    _, _, _, _, plan, committed = _committed_context(tmp_path)
    summary = _summary()
    del summary["goal"]["final_goal_error"]
    readiness = Slice7GReadinessResult(True, "", 10, 0.5, 0.0, 0.0, 0.01, 0.01)
    with pytest.raises(Slice7GRuntimeError) as raised:
        cell_result_from_summary(
            cell=plan.cells[0],
            plan=plan,
            ledger=committed,
            summary=summary,
            readiness=readiness,
            process_exit_status=0,
        )
    assert raised.value.code == "cell_summary_field"


def test_cell_summary_rejects_partial_readiness_record(tmp_path):
    _, _, _, _, plan, committed = _committed_context(tmp_path)
    with pytest.raises(Slice7GRuntimeError) as raised:
        cell_result_from_summary(
            cell=plan.cells[0],
            plan=plan,
            ledger=committed,
            summary=_summary(),
            readiness=object.__new__(Slice7GReadinessResult),
            process_exit_status=0,
        )
    assert raised.value.code == "readiness_result_record"


def test_domain_allocator_is_collision_aware_and_binds_authority(tmp_path):
    charter = _charter()
    authorization, _ = _authorization(tmp_path, charter)
    occupied = []
    allocator = Slice7GDomainAllocator(
        lambda domain: (occupied.append(domain), _occupancy(domain, clear=domain >= 103))[1],
        lambda domain, auth, campaign: hashlib.sha256(f"{domain}:{auth}:{campaign}".encode()).hexdigest(),
        lambda domain, lease: hashlib.sha256(f"release:{domain}:{lease}".encode()).hexdigest(),
    )
    lease = allocator.allocate(charter, authorization, STAMP)
    assert occupied == [100, 101, 102, 103]
    assert lease.domain_id == 103
    assert lease.runtime_authorization_identity == authorization.identity
    assert lease.occupancy_checked is True and lease.collision_free is True


def test_output_allocator_creates_only_new_empty_authorized_root(tmp_path):
    charter = _charter()
    authorization, _ = _authorization(tmp_path, charter)
    allocator = AtomicSlice7GOutputAllocator()
    assert allocator.allocate(authorization) == authorization.campaign_output_root
    root = Path(authorization.campaign_output_root)
    assert root.is_dir() and list(root.iterdir()) == []
    with pytest.raises(Slice7GRuntimeError) as raised:
        allocator.allocate(authorization)
    assert raised.value.code == "output_root_exists"


def test_effect_boundaries_reject_forged_authorization_and_domain_lease(tmp_path):
    charter = _charter()
    authorization, _ = _authorization(tmp_path, charter)
    allocator = Slice7GDomainAllocator(
        lambda domain: _occupancy(domain),
        lambda *args: "a" * 64,
        lambda *args: "b" * 64,
    )
    forged_authorization = replace(authorization, identity="0" * 64)
    with pytest.raises(Slice7GRuntimeError) as raised:
        allocator.allocate(charter, forged_authorization, STAMP)
    assert raised.value.code == "runtime_authorization_identity"
    with pytest.raises(Slice7GRuntimeError) as raised:
        AtomicSlice7GOutputAllocator().allocate(object.__new__(type(authorization)))
    assert raised.value.code == "runtime_authorization_record"

    lease = allocator.allocate(charter, authorization, STAMP)
    with pytest.raises(Slice7GRuntimeError) as raised:
        allocator.release(replace(lease, identity="0" * 64), STAMP)
    assert raised.value.code == "domain_lease_identity"


def test_runtime_authorization_must_be_sealed_and_nofollow(tmp_path):
    charter = _charter()
    authorization, _ = _authorization(tmp_path, charter)
    path = tmp_path / "authorization.json"
    path.chmod(0o644)
    with pytest.raises(Slice7GRuntimeError) as raised:
        _load_slice_7g_runtime_authorization_v1_for_test(path, charter)
    assert raised.value.code == "sealed_file_metadata"
    path.chmod(0o444)
    link = tmp_path / "authorization-link.json"
    link.symlink_to(path)
    with pytest.raises(Slice7GRuntimeError):
        _load_slice_7g_runtime_authorization_v1_for_test(link, charter)


def test_domain_allocator_fails_closed_when_occupancy_is_unknown(tmp_path):
    charter = _charter()
    authorization, _ = _authorization(tmp_path, charter)
    with pytest.raises(Slice7GRuntimeError) as raised:
        Slice7GDomainAllocator(
            lambda domain: None, lambda *args: "a" * 64, lambda *args: "b" * 64,
        ).allocate(charter, authorization, STAMP)
    assert raised.value.code == "domain_occupancy_unproven"


def test_domain_release_is_receipted(tmp_path):
    charter = _charter()
    authorization, _ = _authorization(tmp_path, charter)
    released = []
    allocator = Slice7GDomainAllocator(
        lambda domain: _occupancy(domain),
        lambda *args: "a" * 64,
        lambda domain, lease: released.append((domain, lease)) or "b" * 64,
    )
    lease = allocator.allocate(charter, authorization, STAMP)
    receipt = allocator.release(lease, STAMP)
    assert released == [(100, lease.identity)]
    assert receipt.lease_identity == lease.identity
    assert receipt.provider_receipt_identity == "b" * 64


def test_readiness_authenticates_stability_tactile_and_safety():
    tracker = Slice7GReadinessTracker()
    for index in range(10):
        tracker.add_state_tip(index / 18.0, [0.0] * 6, [0.0, 0.0, 0.08])
    tracker.update_tactile(0.51, valid=True, source="simulated")
    tracker.update_safety(0.51, ready=True, fault=False, state_name="ready")
    result = tracker.evaluate(0.55)
    assert result.passed is True
    assert result.stable_sample_count == 10
    tracker.update_safety(0.56, ready=False, fault=True, state_name="fault")
    assert tracker.evaluate(0.56).failure_code == "readiness_safety_fault"


def test_readiness_timeout_is_bounded_and_stable():
    tracker = Slice7GReadinessTracker(start_timestamp=5.0)
    assert tracker.evaluate(15.000001).failure_code == "readiness_timeout"


def test_atomic_ledger_commit_is_noreplace_and_stale_safe(tmp_path):
    charter, _, initial, _, _, _ = _committed_context(tmp_path)
    store = tmp_path / "ledger"
    store.mkdir()
    writer = AtomicSlice7GLedgerWriter(store)
    writer.initialize(initial)
    event = propose_slice_7g_attempt_event(
        initial, "domain_and_output_allocated", "allocation-a", STAMP,
        domain_id=117, output_root=str(tmp_path / "campaign"),
        runtime_authorization_identity="c" * 64,
    )
    successor = writer.commit(initial, event)
    assert successor.revision == 1 and successor.consumed_campaign_attempts == 0
    assert (store / "attempt_ledger.r00000001.json").stat().st_mode & 0o777 == 0o444
    competing = propose_slice_7g_attempt_event(
        initial, "domain_and_output_allocated", "allocation-b", STAMP,
        domain_id=118, output_root=str(tmp_path / "campaign"),
        runtime_authorization_identity="c" * 64,
    )
    with pytest.raises(Slice7GRuntimeError) as raised:
        writer.commit(initial, competing)
    assert raised.value.code == "ledger_commit_conflict"


def test_atomic_ledger_concurrency_has_exactly_one_revision_winner(tmp_path):
    charter, _, initial, _, _, _ = _committed_context(tmp_path)
    store = tmp_path / "ledger"
    store.mkdir()
    AtomicSlice7GLedgerWriter(store).initialize(initial)
    barrier = threading.Barrier(2)

    class SynchronizedWriter(AtomicSlice7GLedgerWriter):
        def _commit_file(self, ledger, event):
            if ledger.revision == 1:
                barrier.wait(timeout=2.0)
            return super()._commit_file(ledger, event)

    outcomes = []

    def commit(event_id, domain):
        event = propose_slice_7g_attempt_event(
            initial,
            "domain_and_output_allocated",
            event_id,
            STAMP,
            domain_id=domain,
            output_root=str(tmp_path / "campaign"),
            runtime_authorization_identity="c" * 64,
        )
        try:
            SynchronizedWriter(store).commit(initial, event)
            outcomes.append("committed")
        except Slice7GRuntimeError as exc:
            outcomes.append(exc.code)

    threads = [
        threading.Thread(target=commit, args=("concurrent-a", 117)),
        threading.Thread(target=commit, args=("concurrent-b", 118)),
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=3.0)
    assert not any(thread.is_alive() for thread in threads)
    assert sorted(outcomes) == ["committed", "ledger_commit_conflict"]
    assert len(list(store.glob("attempt_ledger.r00000001.json"))) == 1


def test_plan_is_exact_deterministic_15_cell_single_domain(tmp_path):
    _, _, _, _, plan, _ = _committed_context(tmp_path)
    assert len(plan.cells) == 15
    assert len({(cell.scenario_id, cell.seed) for cell in plan.cells}) == 15
    assert {cell.ros_domain_id for cell in plan.cells} == {117}
    assert {cell.campaign_output_root for cell in plan.cells} == {str(tmp_path / "campaign")}
    assert [cell.seed for cell in plan.cells[:5]] == [11, 22, 33, 44, 55]
    assert slice_7g_campaign_plan_identity(plan) == slice_7g_campaign_plan_identity(plan)


def test_governed_child_launch_enables_safety_tactile_and_no_bypass():
    command = build_base_simulation_command(
        experiment_group="campaign-001", controller_label="mppi", baseline_dir=None,
        task="curved_lumen_navigation", target_position=[0.0, 0.0, 0.08],
        curved_lumen_type="circular_arc", random_seed=11, slice_7g_profile=True,
    )
    assert "slice_7g_profile:=true" in command
    assert "tactile_enabled:=true" in command
    assert "start_safety_supervisor:=true" in command
    assert "mppi_publish_safe_for_simulation:=true" not in command
    assert unexpected_command_publishers(
        {"/ctr/mppi_command": 0, "/ctr/safe_command": 1}, slice_7g_governed=True,
    ) == {}


def test_lower_level_environment_rejects_domain_overwrite(monkeypatch):
    monkeypatch.setenv("CTR_SLICE_7G_RUNTIME_AUTHORIZATION_IDENTITY", "a" * 64)
    monkeypatch.setenv("CTR_SLICE_7G_ATTEMPT_LEDGER_IDENTITY", "b" * 64)
    monkeypatch.setenv("ROS_DOMAIN_ID", "117")
    with pytest.raises(OrchestrationError, match="cannot replace"):
        run_environment(118)


def test_cli_cell_binding_rejects_mutated_output_and_accepts_exact_plan(monkeypatch, tmp_path):
    _, authorization, _, _, plan, committed = _committed_context(tmp_path)
    cell = plan.cells[0]
    env = {
        "CTR_SLICE_7G_RUNTIME_AUTHORIZATION_IDENTITY": authorization.identity,
        "CTR_SLICE_7G_ATTEMPT_LEDGER_IDENTITY": slice_7g_attempt_ledger_identity(committed),
        "CTR_SLICE_7G_CAMPAIGN_PLAN_IDENTITY": slice_7g_campaign_plan_identity(plan),
        "CTR_SLICE_7G_CELL_ID": cell.cell_id,
        "CTR_SLICE_7G_CAMPAIGN_ID": plan.campaign_id,
        "CTR_SLICE_7G_CAMPAIGN_OUTPUT_ROOT": cell.campaign_output_root,
        "CTR_SLICE_7G_CELL_OUTPUT_ROOT": cell.cell_output_path,
        "ROS_DOMAIN_ID": str(cell.ros_domain_id),
    }
    for name, value in env.items():
        monkeypatch.setenv(name, value)
    args = parse_args(list(cell.argv[1:]))
    validate_slice_7g_runtime_binding(args, Path(cell.cell_output_path))
    with pytest.raises(OrchestrationError, match="output root"):
        validate_slice_7g_runtime_binding(args, Path(cell.cell_output_path + "-mutated"))
    reordered = list(cell.argv[1:])
    reordered[0:4] = reordered[2:4] + reordered[0:2]
    with pytest.raises(OrchestrationError, match="raw argv"):
        validate_slice_7g_runtime_binding(parse_args(reordered), Path(cell.cell_output_path))


def test_cell_result_translation_preserves_diagnostic_only_timing(tmp_path):
    _, _, _, _, plan, committed = _committed_context(tmp_path)
    execution = _execution(plan.cells[0], plan, committed, deadline=6.0)
    assert execution.cell_result.timing_pass is False
    assert execution.cell_result.non_real_time_label is True
    assert execution.cell_result.steady_state_error_m == 0.001


def test_evidence_writer_and_governance_reconcile_exact_fifteen_packages(tmp_path):
    charter, _, _, _, plan, committed = _committed_context(tmp_path)
    root = tmp_path / "campaign"
    root.mkdir()
    writer = Slice7GEvidenceWriter(root)
    identities = {}
    for cell in plan.cells:
        package, _, identity = writer.write_cell_package(_execution(cell, plan, committed), charter, committed, plan)
        assert package == root / "evidence" / "packages" / cell.cell_id
        assert package.stat().st_mode & 0o777 == 0o555
        identities[cell.cell_id] = identity
    seal = writer.write_campaign_seal(charter, committed, plan, identities)
    assert seal.stat().st_mode & 0o777 == 0o444
    from ctr_bringup.slice_7g_governance import reconcile_slice_7g_campaign_results
    result = reconcile_slice_7g_campaign_results(charter, plan, committed, root)
    assert result.total_result_count == 15
    assert result.functional_promotion_pass is True
    assert result.total_collision_samples == 0


def test_authenticated_functional_failure_blocks_campaign_promotion(tmp_path):
    charter, _, _, _, plan, committed = _committed_context(tmp_path)
    root = tmp_path / "campaign"
    root.mkdir()
    writer = Slice7GEvidenceWriter(root)
    identities = {}
    for index, cell in enumerate(plan.cells):
        _, _, identity = writer.write_cell_package(
            _execution(cell, plan, committed, collision=1 if index == 0 else 0),
            charter, committed, plan,
        )
        identities[cell.cell_id] = identity
    writer.write_campaign_seal(charter, committed, plan, identities)
    from ctr_bringup.slice_7g_governance import reconcile_slice_7g_campaign_results
    result = reconcile_slice_7g_campaign_results(charter, plan, committed, root)
    assert result.functional_promotion_pass is False
    assert result.total_collision_samples == 1
    assert any("collision_sample_count" in reason for reason in result.functional_failure_reasons)


def test_evidence_package_rolls_back_on_member_failure(tmp_path):
    charter, _, _, _, plan, committed = _committed_context(tmp_path)
    root = tmp_path / "campaign"
    root.mkdir()
    writer = Slice7GEvidenceWriter(root)
    with mock.patch("ctr_evaluation.slice_7g_runtime._exclusive_sealed_file", side_effect=OSError("injected")):
        with pytest.raises(OSError, match="injected"):
            writer.write_cell_package(_execution(plan.cells[0], plan, committed), charter, committed, plan)
    parent = root / "evidence" / "packages"
    assert list(parent.iterdir()) == []


def test_evidence_writer_rejects_root_not_bound_to_ledger(tmp_path):
    charter, _, _, _, plan, committed = _committed_context(tmp_path)
    wrong = tmp_path / "wrong"
    wrong.mkdir()
    with pytest.raises(Slice7GRuntimeError) as raised:
        Slice7GEvidenceWriter(wrong).write_cell_package(
            _execution(plan.cells[0], plan, committed), charter, committed, plan,
        )
    assert raised.value.code == "evidence_output_root_binding"


def test_campaign_seal_exclusive_lock_failure_is_closed_and_released(tmp_path):
    charter, _, _, _, plan, committed = _committed_context(tmp_path)
    root = tmp_path / "campaign"
    root.mkdir()
    original_root_mode = root.stat().st_mode & 0o777
    writer = Slice7GEvidenceWriter(root)
    identities = {}
    for cell in plan.cells:
        _, _, identity = writer.write_cell_package(_execution(cell, plan, committed), charter, committed, plan)
        identities[cell.cell_id] = identity
    with mock.patch("ctr_evaluation.slice_7g_runtime.fcntl.flock", side_effect=BlockingIOError("busy")) as lock:
        with pytest.raises(Slice7GRuntimeError) as raised:
            writer.write_campaign_seal(charter, committed, plan, identities)
    assert raised.value.code == "evidence_seal_lock"
    assert lock.call_count == 1
    assert not (root / "evidence/campaign_evidence_seal.json").exists()
    assert root.stat().st_mode & 0o777 == original_root_mode


def test_post_implementation_snapshot_proposal_is_dynamic_and_not_persisted(tmp_path):
    root = tmp_path / "repo"
    (root / "src/pkg").mkdir(parents=True)
    (root / "src/pkg/module.py").write_text("value = 1\n", encoding="utf-8")
    raw, identity, count = post_implementation_source_snapshot(root)
    assert count == 1 and len(identity) == 64
    assert json.loads(raw)["members"][0]["path"] == "src/pkg/module.py"
    assert list(root.rglob("*")) == [root / "src", root / "src/pkg", root / "src/pkg/module.py"]


def test_post_implementation_snapshot_discovery_excludes_generated_and_top_level_evaluation(tmp_path):
    root = tmp_path / "repo"
    (root / "src/pkg").mkdir(parents=True)
    (root / "src/pkg/module.py").write_text("x = 1\n", encoding="utf-8")
    (root / "src/pkg/__pycache__").mkdir()
    (root / "src/pkg/__pycache__/module.pyc").write_bytes(b"cache")
    (root / "evaluation").mkdir()
    (root / "evaluation/secret.json").write_text("{}", encoding="utf-8")
    assert discover_post_implementation_snapshot_members(root) == ("src/pkg/module.py",)


def _independent_snapshot_json(value):
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False,
    ).encode("utf-8")


def _snapshot_member_value(*, path="src/pkg/module.py", mode=0o644, raw=b"value = 1\n"):
    return {"path": path, "mode": mode, "size": len(raw), "sha256": hashlib.sha256(raw).hexdigest()}


def _one_member_v2(value=None):
    return _independent_snapshot_json({
        "schema_version": POST_IMPLEMENTATION_SNAPSHOT_SCHEMA,
        "members": [value or _snapshot_member_value()],
    })


def test_source_snapshot_v2_exact_schema_and_descriptor_mode(tmp_path):
    root = tmp_path / "repo"
    member = root / "src/pkg/module.py"
    member.parent.mkdir(parents=True)
    member.write_bytes(b"value = 1\n")
    member.chmod(0o640)
    raw, identity, count = post_implementation_source_snapshot(root)
    parsed = parse_post_implementation_source_snapshot(raw)
    assert parsed.schema_version == "ctr-slice-7g-post-implementation-source-snapshot-2"
    assert POST_IMPLEMENTATION_SNAPSHOT_LOGICAL_ALGORITHM == (
        "sha256:ctr-slice-7g-post-implementation-source-snapshot-canonical-2"
    )
    assert count == 1 and parsed.members[0].mode == 0o640
    assert set(json.loads(raw)["members"][0]) == {"path", "mode", "size", "sha256"}
    assert identity == post_implementation_source_snapshot_identity(parsed)
    assert inspect_post_implementation_source_snapshot(raw).build_authoritative is False
    assert verify_post_implementation_source_snapshot(raw, root) is True


@pytest.mark.parametrize(
    "mode",
    ["420", 420.0, True, -1, 0o10000],
    ids=("string", "float", "boolean", "negative", "above-07777"),
)
def test_source_snapshot_v2_rejects_invalid_mode_types_and_range(mode):
    with pytest.raises(Slice7GRuntimeError) as raised:
        parse_post_implementation_source_snapshot(_one_member_v2(_snapshot_member_value(mode=mode)))
    assert raised.value.code == "source_snapshot_member_mode"


@pytest.mark.parametrize(
    ("mutation", "expected_code"),
    [
        (lambda value: value["members"][0].pop("mode"), "source_snapshot_member_schema"),
        (lambda value: value["members"][0].update({"owner": "caller"}), "source_snapshot_member_schema"),
    ],
    ids=("missing-mode", "unknown-member-field"),
)
def test_source_snapshot_v2_member_schema_is_closed(mutation, expected_code):
    value = {"schema_version": POST_IMPLEMENTATION_SNAPSHOT_SCHEMA, "members": [_snapshot_member_value()]}
    mutation(value)
    with pytest.raises(Slice7GRuntimeError) as raised:
        parse_post_implementation_source_snapshot(_independent_snapshot_json(value))
    assert raised.value.code == expected_code


@pytest.mark.parametrize(
    "value",
    [
        {"members": [_snapshot_member_value()]},
        {
            "schema_version": POST_IMPLEMENTATION_SNAPSHOT_SCHEMA,
            "members": [_snapshot_member_value()],
            "authority": True,
        },
    ],
    ids=("missing-schema", "unknown-top-level-field"),
)
def test_source_snapshot_v2_top_level_schema_is_closed(value):
    with pytest.raises(Slice7GRuntimeError) as raised:
        parse_post_implementation_source_snapshot(_independent_snapshot_json(value))
    assert raised.value.code == "source_snapshot_schema"


def test_source_snapshot_v2_rejects_member_reordering_and_duplicate_paths():
    first = _snapshot_member_value(path="src/a.py")
    second = _snapshot_member_value(path="src/b.py")
    for members in ([second, first], [first, dict(first)]):
        raw = _independent_snapshot_json({
            "schema_version": POST_IMPLEMENTATION_SNAPSHOT_SCHEMA, "members": members,
        })
        with pytest.raises(Slice7GRuntimeError) as raised:
            parse_post_implementation_source_snapshot(raw)
        assert raised.value.code == "source_snapshot_member_schema"


def test_source_snapshot_mode_only_mutation_changes_both_identities_and_mismatches(tmp_path):
    root = tmp_path / "repo"
    member = root / "src/pkg/module.py"
    member.parent.mkdir(parents=True)
    member.write_bytes(b"fixed bytes\n")
    member.chmod(0o644)
    raw_a, logical_a, _ = post_implementation_source_snapshot(root)
    member.chmod(0o664)
    raw_b, logical_b, _ = post_implementation_source_snapshot(root)
    value_a, value_b = json.loads(raw_a), json.loads(raw_b)
    for field in ("path", "size", "sha256"):
        assert value_a["members"][0][field] == value_b["members"][0][field]
    assert value_a["members"][0]["mode"] == 0o644
    assert value_b["members"][0]["mode"] == 0o664
    assert hashlib.sha256(raw_a).hexdigest() != hashlib.sha256(raw_b).hexdigest()
    assert logical_a != logical_b
    with pytest.raises(Slice7GRuntimeError) as raised:
        verify_post_implementation_source_snapshot(raw_a, root)
    assert raised.value.code == "source_snapshot_mode_mismatch"


@pytest.mark.parametrize(("first_mode", "second_mode"), [(0o644, 0o744), (0o644, 0o664)])
def test_source_snapshot_executable_and_write_bits_are_bound(tmp_path, first_mode, second_mode):
    root = tmp_path / "repo"
    member = root / "src/pkg/module.py"
    member.parent.mkdir(parents=True)
    member.write_bytes(b"same\n")
    member.chmod(first_mode)
    raw_a, identity_a, _ = post_implementation_source_snapshot(root)
    member.chmod(second_mode)
    raw_b, identity_b, _ = post_implementation_source_snapshot(root)
    assert raw_a != raw_b and identity_a != identity_b


def test_source_snapshot_restored_mode_authenticates(tmp_path):
    root = tmp_path / "repo"
    member = root / "src/pkg/module.py"
    member.parent.mkdir(parents=True)
    member.write_bytes(b"same\n")
    member.chmod(0o644)
    raw, _, _ = post_implementation_source_snapshot(root)
    member.chmod(0o664)
    member.chmod(0o644)
    assert verify_post_implementation_source_snapshot(raw, root) is True


def test_source_snapshot_mode_mutation_during_hashing_is_rejected(tmp_path, monkeypatch):
    root = tmp_path / "repo"
    member = root / "src/pkg/module.py"
    member.parent.mkdir(parents=True)
    member.write_bytes(b"content\n")
    member.chmod(0o644)
    original_read = runtime_module.os.read
    changed = False

    def mutating_read(descriptor, count):
        nonlocal changed
        chunk = original_read(descriptor, count)
        if chunk and not changed:
            changed = True
            member.chmod(0o664)
        return chunk

    monkeypatch.setattr(runtime_module.os, "read", mutating_read)
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    assert raised.value.code == "source_snapshot_mode_mismatch"


def test_source_snapshot_mode_mutation_before_final_barrier_is_rejected(tmp_path, monkeypatch):
    root = tmp_path / "repo"
    member = root / "src/pkg/module.py"
    member.parent.mkdir(parents=True)
    member.write_bytes(b"content\n")
    member.chmod(0o644)
    original = runtime_module._RepositorySnapshotAuthority._authenticate_final_members

    def mutate_after_initial(authority):
        member.chmod(0o664)
        return original(authority)

    monkeypatch.setattr(
        runtime_module._RepositorySnapshotAuthority,
        "_authenticate_final_members",
        mutate_after_initial,
    )
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    assert raised.value.code == "source_snapshot_mode_mismatch"


def test_source_snapshot_stale_caller_mode_and_descriptor_cleanup(tmp_path):
    root = tmp_path / "repo"
    member = root / "src/pkg/module.py"
    member.parent.mkdir(parents=True)
    raw_bytes = b"content\n"
    member.write_bytes(raw_bytes)
    member.chmod(0o664)
    stale = _one_member_v2(_snapshot_member_value(mode=0o644, raw=raw_bytes))
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    with pytest.raises(Slice7GRuntimeError) as raised:
        verify_post_implementation_source_snapshot(stale, root)
    gc.collect()
    assert raised.value.code == "source_snapshot_mode_mismatch"
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before


def test_source_snapshot_values_are_deeply_immutable_and_reject_subclasses():
    member = Slice7GSourceSnapshotMember("src/pkg/module.py", 0o644, 1, hashlib.sha256(b"x").hexdigest())
    supplied = [member]
    snapshot = Slice7GPostImplementationSourceSnapshot(POST_IMPLEMENTATION_SNAPSHOT_SCHEMA, tuple(supplied))
    raw = canonical_post_implementation_source_snapshot_bytes(snapshot)
    supplied.clear()
    assert canonical_post_implementation_source_snapshot_bytes(snapshot) == raw
    assert snapshot.members == (member,)

    class MemberSubclass(Slice7GSourceSnapshotMember):
        pass

    with pytest.raises(Slice7GRuntimeError) as raised:
        Slice7GPostImplementationSourceSnapshot(
            POST_IMPLEMENTATION_SNAPSHOT_SCHEMA,
            (MemberSubclass("src/pkg/other.py", 0o644, 1, hashlib.sha256(b"x").hexdigest()),),
        )
    assert raised.value.code == "source_snapshot_member_schema"
    partial = object.__new__(Slice7GSourceSnapshotMember)
    with pytest.raises(Slice7GRuntimeError) as partial_error:
        Slice7GPostImplementationSourceSnapshot(POST_IMPLEMENTATION_SNAPSHOT_SCHEMA, (partial,))
    assert partial_error.value.code == "source_snapshot_member_schema"


def test_source_snapshot_v1_is_historical_only_and_cannot_be_upgraded_in_place():
    payload = {
        "schema_version": POST_IMPLEMENTATION_SNAPSHOT_V1_SCHEMA,
        "members": [{
            "path": "src/pkg/module.py", "size": 1, "sha256": hashlib.sha256(b"x").hexdigest(),
        }],
    }
    raw = _independent_snapshot_json(payload)
    inspection = inspect_post_implementation_source_snapshot(raw)
    assert inspection.schema_version == POST_IMPLEMENTATION_SNAPSHOT_V1_SCHEMA
    assert inspection.build_authoritative is False
    assert inspection.member_count == 1
    with pytest.raises(Slice7GRuntimeError) as raised:
        parse_post_implementation_source_snapshot(raw)
    assert raised.value.code == "source_snapshot_schema_not_build_authoritative"
    assert _independent_snapshot_json(payload) == raw


def test_source_snapshot_v1_and_v2_versioned_identities_do_not_collide():
    v1 = _independent_snapshot_json({
        "schema_version": POST_IMPLEMENTATION_SNAPSHOT_V1_SCHEMA,
        "members": [{"path": "src/pkg/module.py", "size": 1, "sha256": hashlib.sha256(b"x").hexdigest()}],
    })
    v2 = _one_member_v2(_snapshot_member_value(raw=b"x"))
    v1_inspection = inspect_post_implementation_source_snapshot(v1)
    v2_inspection = inspect_post_implementation_source_snapshot(v2)
    assert v1_inspection.logical_identity != v2_inspection.logical_identity
    assert v1_inspection.physical_sha256 != v2_inspection.physical_sha256


def test_source_snapshot_v2_independent_serializers_round_trip_exactly():
    raw = _one_member_v2()
    snapshot = parse_post_implementation_source_snapshot(raw)
    production = canonical_post_implementation_source_snapshot_bytes(snapshot)
    independent = _independent_snapshot_json(json.loads(raw))
    assert production == independent == raw
    assert not raw.endswith(b"\n")
    reparsed = parse_post_implementation_source_snapshot(production)
    assert canonical_post_implementation_source_snapshot_bytes(reparsed) == raw
    assert post_implementation_source_snapshot_identity(reparsed) == hashlib.sha256(
        b"ctr-slice-7g-post-implementation-source-snapshot-canonical-2\0" + raw,
    ).hexdigest()


def _complete_snapshot_fixture(tmp_path, *, parent=""):
    base = tmp_path / parent if parent else tmp_path
    root = base / "repo"
    first = root / "src/pkg/a.py"
    second = root / "src/pkg/b.py"
    first.parent.mkdir(parents=True)
    first.write_bytes(b"a = 1\n")
    second.write_bytes(b"b = 2\n")
    first.chmod(0o644)
    second.chmod(0o644)
    return root, first, second


def _complete_bootstrap_snapshot_fixture(tmp_path):
    root = tmp_path / "repo"
    files = {
        "config/settings.py": b"setting = 1\n",
        "docs/note.md": b"source note\n",
        "src/pkg/a.py": b"a = 1\n",
        "src/pkg/b.py": b"b = 2\n",
        "src/pkg/deep/c.py": b"c = 3\n",
    }
    paths = {}
    for relative, content in files.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
        path.chmod(0o644)
        paths[relative] = path
    return root, paths


def _install_pre_child_watch_mutation(monkeypatch, target_path, mutation):
    authority_type = runtime_module._RepositorySnapshotAuthority
    original = authority_type._before_source_directory_watch
    reached = []

    def before_watch(authority, path, index, total):
        original(authority, path, index, total)
        if path != target_path or reached:
            return
        inventory = authority._provisional_inventory
        assert authority._bootstrap_phase == "provisional_captured"
        assert type(inventory) is runtime_module._SourceSnapshotProvisionalInventory
        directories = {item.path: item for item in inventory.directories}
        assert target_path in directories
        assert directories[target_path].metadata.physical_identity not in authority._source_watch_identities
        assert tuple(item.path for item in inventory.members)
        reached.append((index, total))
        mutation()

    monkeypatch.setattr(authority_type, "_before_source_directory_watch", before_watch)
    return reached


def _before_snapshot_final(monkeypatch, mutation):
    original = runtime_module._RepositorySnapshotAuthority._authenticate_final_members
    invoked = False

    def wrapped(authority):
        nonlocal invoked
        if not invoked:
            invoked = True
            mutation()
        return original(authority)

    monkeypatch.setattr(
        runtime_module._RepositorySnapshotAuthority,
        "_authenticate_final_members",
        wrapped,
    )


def _exercise_unrelated_sibling(path, kind):
    renamed = path.with_name(f"{path.name}-renamed")
    assert not path.exists() and not renamed.exists()
    try:
        if kind == "file":
            path.write_bytes(b"created\n")
            path.write_bytes(b"modified\n")
            os.rename(path, renamed)
            renamed.unlink()
        else:
            path.mkdir()
            child = path / "entry"
            child.write_bytes(b"modified\n")
            os.rename(path, renamed)
            (renamed / "entry").unlink()
            renamed.rmdir()
    finally:
        if path.is_dir():
            shutil.rmtree(path)
        elif path.exists():
            path.unlink()
        if renamed.is_dir():
            shutil.rmtree(renamed)
        elif renamed.exists():
            renamed.unlink()


def _install_snapshot_stage_mutation(monkeypatch, root, stage, mutation, watched_parent=None):
    authority_type = runtime_module._RepositorySnapshotAuthority
    if stage == "monitor-setup":
        original = authority_type._watch_directory
        invoked = False

        def wrapped(authority, descriptor, metadata, scope, mask, expected):
            nonlocal invoked
            result = original(authority, descriptor, metadata, scope, mask, expected)
            observed = os.fstat(descriptor)
            target = watched_parent.stat()
            if not invoked and (observed.st_dev, observed.st_ino) == (target.st_dev, target.st_ino):
                invoked = True
                mutation()
            return result

        monkeypatch.setattr(authority_type, "_watch_directory", wrapped)
        return
    if stage == "first-pass":
        original = authority_type._authenticate_members
        invoked = False

        def wrapped(authority, paths, *, final):
            nonlocal invoked
            result = original(authority, paths, final=final)
            if (
                not final
                and authority._bootstrap_phase == "complete"
                and not invoked
            ):
                invoked = True
                mutation()
            return result

        monkeypatch.setattr(authority_type, "_authenticate_members", wrapped)
        return
    if stage == "between-passes":
        _before_snapshot_final(monkeypatch, mutation)
        return
    if stage == "final-rediscovery":
        original = authority_type._discover_complete_membership
        invoked = False

        def wrapped(authority):
            nonlocal invoked
            result = original(authority)
            if authority._begun and not invoked:
                invoked = True
                mutation()
            return result

        monkeypatch.setattr(authority_type, "_discover_complete_membership", wrapped)
        return
    if stage == "final-drain":
        original = authority_type._reconcile_public_path
        invoked = False

        def wrapped(authority, *, check_metadata):
            nonlocal invoked
            result = original(authority, check_metadata=check_metadata)
            if check_metadata and not invoked:
                invoked = True
                mutation()
            return result

        monkeypatch.setattr(authority_type, "_reconcile_public_path", wrapped)
        return
    raise AssertionError(f"unsupported mutation stage: {stage}")


_APPLICABLE_NODE_ACCOUNTING_PATHS = (
    "src/ctr_evaluation/test/test_slice_7g_runtime.py",
    "src/ctr_evaluation/test/test_run_evaluation.py",
    "src/ctr_evaluation/test/test_experiment_recorder.py",
    "src/ctr_bringup/test/test_parameter_validation.py",
    "src/ctr_bringup/test/test_slice_7g_profile.py",
    "src/ctr_bringup/test/test_simulation_launch_config_paths.py",
    "src/ctr_mppi_controller/test/test_mppi_controller_node.py",
    "src/ctr_safety/test/test_safety_supervisor.py",
    "src/ctr_sim/test/test_simulator_node.py",
    "src/ctr_evaluation/test/test_evaluation_node.py",
    "src/ctr_evaluation/test/test_curved_lumen_compatibility.py",
    "src/ctr_evaluation/test/test_curved_lumen_runner.py",
    "src/ctr_evaluation/test/test_curved_lumen_scenarios.py",
    "src/ctr_mppi_controller/test/test_tactile_cost.py",
    "src/ctr_safety/test/test_safety_ros.py",
    "src/ctr_bringup/test/test_slice_7g_governance.py",
    "src/ctr_bringup/test/test_slice_7f_authority_contract.py",
)
_SUPERSEDED_AUTHORING_SNAPSHOT_NODE_IDS = (
    "src/ctr_bringup/test/test_slice_7g_governance.py::test_exact_positive_charter_and_source_snapshot",
    "src/ctr_bringup/test/test_slice_7g_governance.py::test_snapshot_descriptors_close_on_success_and_failure",
)
_PREBUILD_INTERFACE_SHIM_CONTRACT = (
    "Header/Pose/PoseStamped/Vector3 from installed ROS message packages",
    "CtrBackbone/CtrControllerMetrics/CtrJointCommand/CtrJointState/"
    "CtrSafetyStatus/CtrState/CtrTactileState deterministic classes",
    "ClearFault/ExecuteRetreat/ResetController/SetControllerWeights/"
    "SetReference/SetTaskMode/StartExperiment/StopExperiment deterministic services",
    "six-element zero q/q_dot and no generated-package import",
)


def _canonical_node_id_bytes(node_ids):
    """Serialize a pure repository-relative pytest node list without report text."""

    if type(node_ids) not in (list, tuple):
        raise TypeError("node IDs must be an exact list or tuple")
    normalized = []
    for item in node_ids:
        if type(item) is not str:
            raise TypeError("node IDs must be exact strings")
        item = item.replace("\\", "/")
        if (
            not item
            or item.startswith("/")
            or "\x1b" in item
            or "\n" in item
            or "\r" in item
            or "::" not in item
        ):
            raise ValueError("node ID is not a pure repository-relative pytest node")
        normalized.append(item)
    if len(normalized) != len(set(normalized)):
        raise ValueError("duplicate node IDs")
    return b"\n".join(item.encode("utf-8") for item in sorted(normalized))


def test_source_snapshot_structural_subset_is_never_build_authoritative(tmp_path):
    root, first, _ = _complete_snapshot_fixture(tmp_path)
    raw, _, count = structural_post_implementation_source_snapshot(
        root, [first.relative_to(root).as_posix()],
    )
    assert count == 1
    assert inspect_post_implementation_source_snapshot(raw).build_authoritative is False
    with pytest.raises(Slice7GRuntimeError) as raised:
        verify_post_implementation_source_snapshot(raw, root)
    assert raised.value.code == "source_snapshot_membership_mismatch"
    with pytest.raises(Slice7GRuntimeError) as production_error:
        post_implementation_source_snapshot(root, [first.relative_to(root).as_posix()])
    assert production_error.value.code == "source_snapshot_membership_mismatch"


def test_source_snapshot_authoritative_verifier_rejects_omitted_member(tmp_path):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    raw, _, _ = post_implementation_source_snapshot(root)
    value = json.loads(raw)
    value["members"].pop()
    with pytest.raises(Slice7GRuntimeError) as raised:
        verify_post_implementation_source_snapshot(_independent_snapshot_json(value), root)
    assert raised.value.code == "source_snapshot_membership_mismatch"


def test_source_snapshot_authoritative_verifier_rejects_extra_member(tmp_path):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    raw, _, _ = post_implementation_source_snapshot(root)
    value = json.loads(raw)
    value["members"].append(_snapshot_member_value(path="src/pkg/extra.py", raw=b"extra\n"))
    value["members"].sort(key=lambda item: item["path"])
    with pytest.raises(Slice7GRuntimeError) as raised:
        verify_post_implementation_source_snapshot(_independent_snapshot_json(value), root)
    assert raised.value.code == "source_snapshot_membership_mismatch"


def test_source_snapshot_exact_complete_membership_is_authoritatively_verified(tmp_path):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    raw, _, count = post_implementation_source_snapshot(root)
    assert count == 2
    assert verify_post_implementation_source_snapshot(raw, root) is True
    assert inspect_post_implementation_source_snapshot(raw).build_authoritative is False


def test_source_snapshot_newly_discovered_member_invalidates_older_snapshot(tmp_path):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    raw, _, _ = post_implementation_source_snapshot(root)
    added = root / "src/pkg/new_dependency.py"
    added.write_bytes(b"new = True\n")
    added.chmod(0o644)
    with pytest.raises(Slice7GRuntimeError) as raised:
        verify_post_implementation_source_snapshot(raw, root)
    assert raised.value.code == "source_snapshot_membership_mismatch"


def test_source_snapshot_caller_authority_claim_and_forged_results_are_rejected(tmp_path):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    raw, _, _ = post_implementation_source_snapshot(root)
    parsed = inspect_post_implementation_source_snapshot(raw)
    with pytest.raises(Slice7GRuntimeError) as claimed:
        Slice7GSourceSnapshotInspection(
            parsed.schema_version, parsed.member_count, parsed.physical_sha256,
            parsed.logical_identity, True,
        )
    assert claimed.value.code == "source_snapshot_inspection"
    with pytest.raises(Slice7GRuntimeError) as structural:
        verify_post_implementation_source_snapshot(parsed, root)
    assert structural.value.code == "source_snapshot_schema"
    partial = object.__new__(Slice7GSourceSnapshotInspection)
    with pytest.raises(Slice7GRuntimeError) as forged:
        verify_post_implementation_source_snapshot(partial, root)
    assert forged.value.code == "source_snapshot_schema"


def test_source_snapshot_structural_member_boundary_rejects_hostile_collections(tmp_path):
    root, first, _ = _complete_snapshot_fixture(tmp_path)

    class ListSubclass(list):
        pass

    class Hostile:
        invoked = False

        def __iter__(self):
            self.invoked = True
            raise RuntimeError("caller hook")

    for supplied in (ListSubclass([first.relative_to(root).as_posix()]), Hostile()):
        with pytest.raises(Slice7GRuntimeError) as raised:
            structural_post_implementation_source_snapshot(root, supplied)
        assert raised.value.code == "snapshot_members_type"
        assert not getattr(supplied, "invoked", False)


def test_source_snapshot_verifier_missing_root_is_stable(tmp_path):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    raw, _, _ = post_implementation_source_snapshot(root)
    with pytest.raises(Slice7GRuntimeError) as raised:
        verify_post_implementation_source_snapshot(raw, tmp_path / "missing")
    assert raised.value.code == "snapshot_root"


def test_source_snapshot_byte_identical_root_replacement_is_rejected(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    old_root = root.with_name("repo-old")

    def replace_root():
        os.rename(root, old_root)
        shutil.copytree(old_root, root, copy_function=shutil.copy2)

    _before_snapshot_final(monkeypatch, replace_root)
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert raised.value.code == "source_snapshot_root_replaced"
    assert root.stat().st_ino != old_root.stat().st_ino
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before


def test_source_snapshot_verifier_rejects_root_replacement_during_final_pass(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    raw, _, _ = post_implementation_source_snapshot(root)
    old_root = root.with_name("repo-old")

    def replace_root():
        os.rename(root, old_root)
        shutil.copytree(old_root, root, copy_function=shutil.copy2)

    _before_snapshot_final(monkeypatch, replace_root)
    with pytest.raises(Slice7GRuntimeError) as raised:
        verify_post_implementation_source_snapshot(raw, root)
    assert raised.value.code == "source_snapshot_root_replaced"


def test_source_snapshot_parent_component_replacement_is_rejected(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path, parent="outer")
    parent = root.parent
    old_parent = tmp_path / "outer-old"

    def replace_parent():
        os.rename(parent, old_parent)
        shutil.copytree(old_parent, parent, copy_function=shutil.copy2)

    _before_snapshot_final(monkeypatch, replace_parent)
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert raised.value.code == "source_snapshot_parent_replaced"
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before


def test_source_snapshot_nested_directory_replacement_is_rejected(tmp_path, monkeypatch):
    root, first, _ = _complete_snapshot_fixture(tmp_path)
    directory = first.parent
    old_directory = tmp_path / "pkg-old"

    def replace_directory():
        os.rename(directory, old_directory)
        shutil.copytree(old_directory, directory, copy_function=shutil.copy2)

    _before_snapshot_final(monkeypatch, replace_directory)
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    assert raised.value.code == "source_snapshot_member_changed"


@pytest.mark.parametrize(
    "mutation",
    ("mode", "executable", "content", "inode", "rename-restore", "add-remove"),
)
def test_source_snapshot_change_and_restore_between_passes_is_rejected(tmp_path, monkeypatch, mutation):
    root, first, _ = _complete_snapshot_fixture(tmp_path)
    original_bytes = first.read_bytes()
    original_mode = first.stat().st_mode & 0o7777

    def mutate_and_restore():
        if mutation == "mode":
            first.chmod(0o664)
            first.chmod(original_mode)
        elif mutation == "executable":
            first.chmod(original_mode | 0o100)
            first.chmod(original_mode)
        elif mutation == "content":
            first.write_bytes(b"z = 9\n")
            first.write_bytes(original_bytes)
        elif mutation == "inode":
            retained = tmp_path / "retained-a.py"
            os.rename(first, retained)
            first.write_bytes(original_bytes)
            first.chmod(original_mode)
        elif mutation == "rename-restore":
            retained = tmp_path / "retained-a.py"
            os.rename(first, retained)
            os.rename(retained, first)
        else:
            transient = first.parent / "transient.py"
            transient.write_bytes(b"transient = True\n")
            transient.unlink()

    _before_snapshot_final(monkeypatch, mutate_and_restore)
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert raised.value.code == "source_snapshot_member_changed"
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before


def test_source_snapshot_unchanged_authority_closes_every_descriptor(tmp_path):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    raw, _, count = post_implementation_source_snapshot(root)
    assert count == 2
    assert verify_post_implementation_source_snapshot(raw, root) is True
    gc.collect()
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before


def test_source_snapshot_partial_authority_cleanup_and_idempotent_close(tmp_path):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    with mock.patch.object(
        runtime_module._RepositorySnapshotAuthority,
        "_open_change_monitor",
        side_effect=Slice7GRuntimeError("source_snapshot_monitor", "unavailable"),
    ):
        with pytest.raises(Slice7GRuntimeError) as raised:
            post_implementation_source_snapshot(root)
    assert raised.value.code == "source_snapshot_monitor"
    gc.collect()
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before


def _repository_fd_count():
    return len(tuple(Path("/proc/self/fd").iterdir()))


def _inotify_frame(watch, mask, name=b""):
    if name:
        area = name + b"\0"
        area += b"\0" * ((16 - len(area) % 16) % 16)
    else:
        area = b""
    return struct.pack("iIII", watch, mask, 0, len(area)) + area


def _monitor_parser_authority(*, scope="member", expected=None):
    authority = object.__new__(runtime_module._RepositorySnapshotAuthority)
    authority._monitor_descriptor = 901
    authority._monitor_scopes = {
        7: runtime_module._RepositoryWatch(scope, expected),
    }
    return authority


@pytest.mark.parametrize("length", range(1, 16), ids=lambda value: f"bytes-{value}")
def test_source_snapshot_monitor_rejects_every_truncated_header_length(length):
    authority = _monitor_parser_authority()
    with pytest.raises(Slice7GRuntimeError) as raised:
        authority._parse_monitor_buffer(b"x" * length)
    assert raised.value.code == "source_snapshot_monitor"
    assert "truncated header" in str(raised.value)


@pytest.mark.parametrize(
    ("payload", "detail"),
    [
        (struct.pack("iIII", 7, 0x100, 0, 16) + b"short", "exceeds its buffer"),
        (_inotify_frame(7, 0x100, b"name") + b"x", "trailing bytes"),
        (struct.pack("iIII", 7, 0x100, 0, 3) + b"a\0\0", "misaligned"),
        (struct.pack("iIII", 7, 0x100, 0, 16) + b"a" * 16, "lacks a terminator"),
        (
            struct.pack("iIII", 7, 0x100, 0, 16) + b"a\0x" + b"\0" * 13,
            "padding is nonzero",
        ),
        (_inotify_frame(7, 0x100, b"valid") + b"truncated", "trailing bytes"),
    ],
    ids=("declared-overrun", "trailing-garbage", "misaligned-name", "missing-nul", "nonzero-padding", "valid-then-truncated"),
)
def test_source_snapshot_monitor_rejects_malformed_frame_boundaries(payload, detail):
    authority = _monitor_parser_authority()
    with pytest.raises(Slice7GRuntimeError) as raised:
        authority._parse_monitor_buffer(payload)
    assert raised.value.code == "source_snapshot_monitor"
    assert detail in str(raised.value)


def test_source_snapshot_monitor_accepts_multiple_complete_concatenated_frames():
    authority = _monitor_parser_authority()
    first = _inotify_frame(7, authority._IN_CREATE | authority._IN_ISDIR, b"alpha")
    second = _inotify_frame(7, authority._IN_DELETE, b"beta")
    events = authority._parse_monitor_buffer(first + second)
    assert events == (
        (7, authority._IN_CREATE | authority._IN_ISDIR, b"alpha"),
        (7, authority._IN_DELETE, b"beta"),
    )


@pytest.mark.parametrize(
    ("watch", "mask", "name", "detail"),
    [
        (99, 0x100, b"member", "unknown watch descriptor"),
        (-1, 0x100, b"member", "invalid global event"),
        (-1, 0x4000, b"", "overflowed"),
        (7, 0x8000, b"", "watch was invalidated"),
    ],
    ids=("unknown-watch", "invalid-global-watch", "queue-overflow", "watch-invalidation"),
)
def test_source_snapshot_monitor_rejects_invalid_watch_semantics(watch, mask, name, detail):
    authority = _monitor_parser_authority()
    with pytest.raises(Slice7GRuntimeError) as raised:
        authority._handle_monitor_event(watch, mask, name)
    assert raised.value.code == "source_snapshot_monitor"
    assert detail in str(raised.value)


def test_source_snapshot_monitor_empty_read_is_failure(monkeypatch):
    authority = _monitor_parser_authority()
    monkeypatch.setattr(runtime_module.os, "read", lambda descriptor, size: b"")
    with pytest.raises(Slice7GRuntimeError) as raised:
        authority._assert_no_monitored_changes()
    assert raised.value.code == "source_snapshot_monitor"
    assert "EOF" in str(raised.value)


def test_source_snapshot_monitor_drains_until_would_block(monkeypatch):
    authority = _monitor_parser_authority(scope="parent", expected="repo")
    returned = iter((
        _inotify_frame(7, authority._IN_CREATE, b"unrelated-a"),
        _inotify_frame(7, authority._IN_DELETE, b"unrelated-b"),
    ))
    calls = 0

    def read_monitor(descriptor, size):
        nonlocal calls
        calls += 1
        try:
            return next(returned)
        except StopIteration:
            raise BlockingIOError

    monkeypatch.setattr(runtime_module.os, "read", read_monitor)
    authority._assert_no_monitored_changes()
    assert calls == 3


@pytest.mark.parametrize("resource", ("bytes", "events"))
def test_source_snapshot_monitor_drain_is_bounded(monkeypatch, resource):
    authority = _monitor_parser_authority(scope="parent", expected="repo")
    frame = _inotify_frame(7, authority._IN_CREATE, b"unrelated")
    if resource == "bytes":
        authority._INOTIFY_MAX_DRAIN_BYTES = len(frame) - 1
    else:
        authority._INOTIFY_MAX_DRAIN_EVENTS = 0
    monkeypatch.setattr(runtime_module.os, "read", lambda descriptor, size: frame)
    with pytest.raises(Slice7GRuntimeError) as raised:
        authority._assert_no_monitored_changes()
    assert raised.value.code == "source_snapshot_monitor"
    assert "limit" in str(raised.value)


def test_source_snapshot_monitor_provider_oserror_is_normalized(monkeypatch):
    authority = _monitor_parser_authority()
    monkeypatch.setattr(runtime_module.os, "read", mock.Mock(side_effect=OSError("provider")))
    with pytest.raises(Slice7GRuntimeError) as raised:
        authority._assert_no_monitored_changes()
    assert raised.value.code == "source_snapshot_monitor"
    assert isinstance(raised.value.__cause__, OSError)


def test_source_snapshot_monitor_provider_baseexception_is_preserved(monkeypatch):
    authority = _monitor_parser_authority()

    class MonitorAbort(BaseException):
        pass

    monkeypatch.setattr(runtime_module.os, "read", mock.Mock(side_effect=MonitorAbort("stop")))
    with pytest.raises(MonitorAbort, match="stop"):
        authority._assert_no_monitored_changes()


@pytest.mark.parametrize("failing_call", (1, 3), ids=("first-watch", "later-chain-watch"))
def test_source_snapshot_watch_install_failure_closes_all_descriptors(tmp_path, monkeypatch, failing_call):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    authority_type = runtime_module._RepositorySnapshotAuthority
    original = authority_type._watch_directory
    calls = 0

    def fail_watch(authority, *args):
        nonlocal calls
        calls += 1
        if calls == failing_call:
            raise Slice7GRuntimeError("source_snapshot_monitor", "injected watch failure")
        return original(authority, *args)

    before = _repository_fd_count()
    monkeypatch.setattr(authority_type, "_watch_directory", fail_watch)
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert raised.value.code == "source_snapshot_monitor"
    assert _repository_fd_count() == before


@pytest.mark.parametrize("failing_open", (1, 2, 3), ids=("first", "second", "third"))
def test_source_snapshot_provisional_chain_open_failure_has_no_descriptor_gap(
    tmp_path, monkeypatch, failing_open,
):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    original = runtime_module.os.open
    calls = 0

    def fail_open(*args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == failing_open:
            raise OSError("injected provisional open failure")
        return original(*args, **kwargs)

    before = _repository_fd_count()
    monkeypatch.setattr(runtime_module.os, "open", fail_open)
    with pytest.raises(Slice7GRuntimeError) as raised:
        runtime_module._RepositorySnapshotAuthority(str(root))
    assert raised.value.code == "snapshot_root"
    assert _repository_fd_count() == before


@pytest.mark.parametrize("failing_open", (1, 2, 3), ids=("first", "second", "third"))
def test_source_snapshot_provisional_chain_baseexception_cleans_prior_descriptors(
    tmp_path, monkeypatch, failing_open,
):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    original = runtime_module.os.open
    calls = 0

    class OpenAbort(BaseException):
        pass

    def abort_open(*args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == failing_open:
            raise OpenAbort(f"open-{failing_open}")
        return original(*args, **kwargs)

    before = _repository_fd_count()
    monkeypatch.setattr(runtime_module.os, "open", abort_open)
    with pytest.raises(OpenAbort, match=f"open-{failing_open}"):
        runtime_module._RepositorySnapshotAuthority(str(root))
    assert _repository_fd_count() == before


@pytest.mark.parametrize("failing_fstat", (1, 2, 3), ids=("first", "second", "third"))
def test_source_snapshot_provisional_chain_fstat_failure_closes_registered_descriptors(
    tmp_path, monkeypatch, failing_fstat,
):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    original = runtime_module.os.fstat
    calls = 0

    def fail_fstat(descriptor):
        nonlocal calls
        calls += 1
        if calls == failing_fstat:
            raise OSError("injected provisional stat failure")
        return original(descriptor)

    before = _repository_fd_count()
    monkeypatch.setattr(runtime_module.os, "fstat", fail_fstat)
    with pytest.raises(Slice7GRuntimeError) as raised:
        runtime_module._RepositorySnapshotAuthority(str(root))
    assert raised.value.code == "snapshot_root"
    assert _repository_fd_count() == before


def test_source_snapshot_provisional_chain_fstat_baseexception_closes_registered_descriptors(
    tmp_path, monkeypatch,
):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    original = runtime_module.os.fstat
    calls = 0

    class StatAbort(BaseException):
        pass

    def abort_fstat(descriptor):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise StatAbort("provisional stat abort")
        return original(descriptor)

    before = _repository_fd_count()
    monkeypatch.setattr(runtime_module.os, "fstat", abort_fstat)
    with pytest.raises(StatAbort, match="provisional stat abort"):
        runtime_module._RepositorySnapshotAuthority(str(root))
    assert _repository_fd_count() == before


@pytest.mark.parametrize("failure_kind", ("ordinary", "baseexception"))
def test_source_snapshot_monitor_registered_before_first_fstat_failure(
    tmp_path, monkeypatch, failure_kind,
):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    authority_type = runtime_module._RepositorySnapshotAuthority

    class MonitorAbort(BaseException):
        pass

    def fail_monitor_fstat(descriptor):
        if failure_kind == "ordinary":
            raise OSError("monitor fstat")
        raise MonitorAbort("monitor fstat abort")

    before = _repository_fd_count()
    monkeypatch.setattr(authority_type, "_monitor_fstat", staticmethod(fail_monitor_fstat))
    if failure_kind == "ordinary":
        with pytest.raises(Slice7GRuntimeError) as raised:
            authority_type(str(root))
        assert raised.value.code == "source_snapshot_monitor"
    else:
        with pytest.raises(MonitorAbort, match="monitor fstat abort"):
            authority_type(str(root))
    assert _repository_fd_count() == before


@pytest.mark.parametrize("failing_call", (1, 3), ids=("first", "later"))
def test_source_snapshot_watch_baseexception_attempts_complete_constructor_cleanup(
    tmp_path, monkeypatch, failing_call,
):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    authority_type = runtime_module._RepositorySnapshotAuthority
    original = authority_type._watch_directory
    calls = 0

    class WatchAbort(BaseException):
        pass

    def abort_watch(authority, *args):
        nonlocal calls
        calls += 1
        if calls == failing_call:
            raise WatchAbort(f"watch-{failing_call}")
        return original(authority, *args)

    before = _repository_fd_count()
    monkeypatch.setattr(authority_type, "_watch_directory", abort_watch)
    with pytest.raises(WatchAbort, match=f"watch-{failing_call}"):
        authority_type(str(root))
    assert _repository_fd_count() == before


def test_source_snapshot_child_watch_failure_closes_locally_owned_descriptor(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    authority_type = runtime_module._RepositorySnapshotAuthority
    original = authority_type._watch_directory

    def fail_member_watch(authority, descriptor, metadata, scope, mask, expected):
        if scope == "member":
            raise Slice7GRuntimeError("source_snapshot_monitor", "injected member watch failure")
        return original(authority, descriptor, metadata, scope, mask, expected)

    before = _repository_fd_count()
    monkeypatch.setattr(authority_type, "_watch_directory", fail_member_watch)
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert raised.value.code == "source_snapshot_monitor"
    assert _repository_fd_count() == before


def test_source_snapshot_failure_before_child_transfer_closes_descriptor(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    authority_type = runtime_module._RepositorySnapshotAuthority
    original = authority_type._directory_names

    def fail_enumeration(descriptor, path):
        if path == "src":
            raise Slice7GRuntimeError("snapshot_member_io", "injected enumeration failure", path=path)
        return original(descriptor, path)

    before = _repository_fd_count()
    monkeypatch.setattr(authority_type, "_directory_names", staticmethod(fail_enumeration))
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert raised.value.code == "snapshot_member_io"
    assert _repository_fd_count() == before


def test_source_snapshot_close_failure_is_terminal_and_later_resources_close(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    before = _repository_fd_count()
    authority = runtime_module._RepositorySnapshotAuthority(str(root))
    target = authority._monitor_descriptor
    original = runtime_module.os.close
    failed = False

    def fail_once(descriptor):
        nonlocal failed
        if descriptor == target and not failed:
            failed = True
            raise OSError("initial close failed")
        return original(descriptor)

    monkeypatch.setattr(runtime_module.os, "close", fail_once)
    issues = authority.close()
    assert tuple(issue.code for issue in issues) == ("source_snapshot_descriptor_close",)
    assert issues[0].detail.startswith("monitor:change-monitor:close:terminal_ambiguity")
    assert authority._closed is True
    assert len(authority.terminally_ambiguous_descriptors) == 1
    assert authority.descriptor_cleanup_status == "completed_with_terminal_ambiguity"
    assert _repository_fd_count() == before + 1
    monkeypatch.setattr(runtime_module.os, "close", original)
    original(target)
    assert _repository_fd_count() == before


def test_source_snapshot_destruction_never_retries_terminal_ambiguity(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    before = _repository_fd_count()
    authority = runtime_module._RepositorySnapshotAuthority(str(root))
    target = authority._monitor_descriptor
    original = runtime_module.os.close
    target_calls = 0

    def fail_target(descriptor):
        nonlocal target_calls
        if descriptor == target:
            target_calls += 1
            raise OSError("terminal ambiguity")
        return original(descriptor)

    monkeypatch.setattr(runtime_module.os, "close", fail_target)
    authority.close()
    assert target_calls == 1
    del authority
    gc.collect()
    assert target_calls == 1
    monkeypatch.setattr(runtime_module.os, "close", original)
    original(target)
    assert _repository_fd_count() == before


def test_source_snapshot_multiple_close_failures_have_deterministic_resource_order(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    before = _repository_fd_count()
    authority = runtime_module._RepositorySnapshotAuthority(str(root))
    targets = {authority._monitor_descriptor, authority._root_descriptor}
    original = runtime_module.os.close
    failed = set()

    def fail_first_close(descriptor):
        if descriptor in targets and descriptor not in failed:
            failed.add(descriptor)
            raise OSError(f"close-{descriptor}")
        return original(descriptor)

    monkeypatch.setattr(runtime_module.os, "close", fail_first_close)
    issues = authority.close()
    assert len(issues) == 2
    assert issues[0].detail.startswith("monitor:change-monitor:close:terminal_ambiguity")
    assert issues[1].detail.startswith("chain:repository-root:close:terminal_ambiguity")
    assert authority._closed is True
    assert len(authority.terminally_ambiguous_descriptors) == 2
    monkeypatch.setattr(runtime_module.os, "close", original)
    for descriptor in targets:
        original(descriptor)
    assert _repository_fd_count() == before


def test_source_snapshot_ambiguous_descriptor_is_terminal_and_never_retried(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    before = _repository_fd_count()
    authority = runtime_module._RepositorySnapshotAuthority(str(root))
    target = authority._monitor_descriptor
    original = runtime_module.os.close

    def fail_target(descriptor):
        if descriptor == target:
            raise OSError("still open")
        return original(descriptor)

    monkeypatch.setattr(runtime_module.os, "close", fail_target)
    target_calls = 0

    def counted_failure(descriptor):
        nonlocal target_calls
        if descriptor == target:
            target_calls += 1
        return fail_target(descriptor)

    monkeypatch.setattr(runtime_module.os, "close", counted_failure)
    first = authority.close()
    assert len(first) == 1
    assert authority._closed is True
    assert authority.terminally_ambiguous_descriptors[0].descriptor == target
    assert target_calls == 1
    assert authority.close() == ()
    assert target_calls == 1
    monkeypatch.setattr(runtime_module.os, "close", original)
    original(target)
    assert _repository_fd_count() == before


@pytest.mark.parametrize(
    "replacement_kind",
    ("different-inode", "same-inode-independent", "duplicate-other-owner"),
)
def test_source_snapshot_uncertain_close_never_closes_reused_descriptor_number(
    tmp_path, monkeypatch, replacement_kind,
):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    before = _repository_fd_count()
    authority = runtime_module._RepositorySnapshotAuthority(str(root))
    target = authority._monitor_descriptor
    original_close = runtime_module.os.close
    replacement_descriptors = []
    donor = None
    if replacement_kind == "duplicate-other-owner":
        donor = os.open(root, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    target_calls = 0

    def fail_target(descriptor):
        nonlocal target_calls
        if descriptor == target:
            target_calls += 1
            original_close(descriptor)
            if replacement_kind == "duplicate-other-owner":
                os.dup2(donor, target)
                replacement_descriptors.append(target)
            else:
                replacement_path = "/dev/null" if replacement_kind == "different-inode" else str(root)
                flags = os.O_RDONLY
                if replacement_kind == "same-inode-independent":
                    flags |= getattr(os, "O_DIRECTORY", 0)
                while target not in replacement_descriptors:
                    replacement_descriptors.append(os.open(replacement_path, flags))
            raise OSError("ambiguous close after numeric reuse")
        return original_close(descriptor)

    monkeypatch.setattr(runtime_module.os, "close", fail_target)
    try:
        authority.close()
        retained = authority._owned_descriptors[target]
        assert retained.lifecycle_state == authority._DESCRIPTOR_TERMINAL_AMBIGUITY
        monkeypatch.setattr(runtime_module.os, "close", original_close)
        assert authority.close() == ()
        assert authority._closed is True
        assert target_calls == 1
        assert os.fstat(target).st_ino > 0
        if replacement_kind != "different-inode":
            assert (os.fstat(target).st_dev, os.fstat(target).st_ino) == (
                root.stat().st_dev, root.stat().st_ino,
            )
    finally:
        monkeypatch.setattr(runtime_module.os, "close", original_close)
        for descriptor in reversed(replacement_descriptors):
            try:
                original_close(descriptor)
            except OSError:
                pass
        if donor is not None:
            original_close(donor)
    assert _repository_fd_count() == before


def test_source_snapshot_confirmed_closed_descriptor_is_never_closed_twice(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    before = _repository_fd_count()
    authority = runtime_module._RepositorySnapshotAuthority(str(root))
    target = authority._monitor_descriptor
    original = runtime_module.os.close
    target_calls = 0

    def close_then_report_failure(descriptor):
        nonlocal target_calls
        if descriptor == target:
            target_calls += 1
            original(descriptor)
            raise OSError("reported after close")
        return original(descriptor)

    monkeypatch.setattr(runtime_module.os, "close", close_then_report_failure)
    issues = authority.close()
    assert len(issues) == 1
    assert target_calls == 1
    assert authority._closed is True
    assert authority.close() == ()
    assert target_calls == 1
    assert _repository_fd_count() == before


def test_source_snapshot_primary_failure_survives_cleanup_failure(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    authority_type = runtime_module._RepositorySnapshotAuthority
    original_close = runtime_module.os.close
    failed = False

    def primary_failure(authority):
        raise Slice7GRuntimeError("source_snapshot_member_changed", "primary validation failure")

    def fail_monitor_close_once(descriptor):
        nonlocal failed
        try:
            label = os.readlink(f"/proc/self/fd/{descriptor}")
        except OSError:
            label = ""
        if not failed and "inotify" in label:
            failed = True
            original_close(descriptor)
            raise OSError("cleanup close failure after close")
        return original_close(descriptor)

    before = _repository_fd_count()
    monkeypatch.setattr(authority_type, "_authenticate_final_members", primary_failure)
    monkeypatch.setattr(runtime_module.os, "close", fail_monitor_close_once)
    with pytest.raises(Slice7GCoordinatedFailure) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert raised.value.code == "source_snapshot_member_changed"
    assert raised.value.primary_code == "source_snapshot_member_changed"
    assert tuple(issue.code for issue in raised.value.cleanup_issues) == (
        "source_snapshot_descriptor_close",
    )
    assert _repository_fd_count() == before


def test_source_snapshot_baseexception_still_closes_authority(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)

    class SnapshotAbort(BaseException):
        pass

    def abort_final(authority):
        raise SnapshotAbort("abort")

    before = _repository_fd_count()
    monkeypatch.setattr(
        runtime_module._RepositorySnapshotAuthority,
        "_authenticate_final_members",
        abort_final,
    )
    with pytest.raises(SnapshotAbort, match="abort"):
        post_implementation_source_snapshot(root)
    gc.collect()
    assert _repository_fd_count() == before


def test_source_snapshot_cleanup_baseexception_attempts_every_resource_before_reraise(
    tmp_path, monkeypatch,
):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    before = _repository_fd_count()
    authority = runtime_module._RepositorySnapshotAuthority(str(root))
    expected = len(authority._owned_descriptors)
    target = authority._monitor_descriptor
    original = runtime_module.os.close
    calls = []

    class CleanupAbort(BaseException):
        pass

    def close_and_abort(descriptor):
        calls.append(descriptor)
        original(descriptor)
        if descriptor == target:
            raise CleanupAbort("cleanup abort")

    monkeypatch.setattr(runtime_module.os, "close", close_and_abort)
    with pytest.raises(CleanupAbort, match="cleanup abort") as raised:
        authority.close()
    assert len(calls) == expected
    assert tuple(issue.code for issue in raised.value.cleanup_issues) == (
        "source_snapshot_descriptor_close",
    )
    assert authority._closed is True
    assert authority.close() == ()
    assert _repository_fd_count() == before


@pytest.mark.parametrize("primary_kind", ("ordinary", "baseexception"))
def test_source_snapshot_primary_survives_cleanup_baseexception_after_all_resources(
    tmp_path, monkeypatch, primary_kind,
):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    before = _repository_fd_count()
    authority = runtime_module._RepositorySnapshotAuthority(str(root))
    expected = len(authority._owned_descriptors)
    target = authority._monitor_descriptor
    original = runtime_module.os.close
    calls = []

    class PrimaryAbort(BaseException):
        pass

    class CleanupAbort(BaseException):
        pass

    def close_and_abort(descriptor):
        calls.append(descriptor)
        original(descriptor)
        if descriptor == target:
            raise CleanupAbort("cleanup abort")

    primary = (
        Slice7GRuntimeError("source_snapshot_member_changed", "primary validation")
        if primary_kind == "ordinary"
        else PrimaryAbort("primary abort")
    )
    monkeypatch.setattr(runtime_module.os, "close", close_and_abort)
    if primary_kind == "ordinary":
        with pytest.raises(Slice7GCoordinatedFailure) as raised:
            runtime_module._finish_repository_authority(authority, primary)
        assert raised.value.primary_code == "source_snapshot_member_changed"
        issues = raised.value.cleanup_issues
    else:
        with pytest.raises(PrimaryAbort, match="primary abort") as raised:
            runtime_module._finish_repository_authority(authority, primary)
        issues = raised.value.cleanup_issues
    assert len(calls) == expected
    assert tuple(issue.code for issue in issues) == ("source_snapshot_descriptor_close",)
    assert _repository_fd_count() == before


def test_source_snapshot_cleanup_reentrant_close_never_double_closes(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    before = _repository_fd_count()
    authority = runtime_module._RepositorySnapshotAuthority(str(root))
    target = authority._monitor_descriptor
    original = runtime_module.os.close
    calls = []

    def reentrant_close(descriptor):
        calls.append(descriptor)
        if descriptor == target:
            assert authority.close() == ()
        return original(descriptor)

    monkeypatch.setattr(runtime_module.os, "close", reentrant_close)
    assert authority.close() == ()
    assert len(calls) == len(set(calls))
    assert authority.close() == ()
    assert _repository_fd_count() == before


def test_source_snapshot_cleanup_only_failure_has_stable_public_boundary(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    before = _repository_fd_count()
    authority = runtime_module._RepositorySnapshotAuthority(str(root))
    target = authority._monitor_descriptor
    original = runtime_module.os.close

    def close_then_fail(descriptor):
        original(descriptor)
        if descriptor == target:
            raise OSError("reported close failure")

    monkeypatch.setattr(runtime_module.os, "close", close_then_fail)
    with pytest.raises(Slice7GCoordinatedFailure) as raised:
        runtime_module._finish_repository_authority(authority)
    assert raised.value.code == "source_snapshot_cleanup"
    assert raised.value.cleanup_issues[0].code == "source_snapshot_descriptor_close"
    assert _repository_fd_count() == before


def test_source_snapshot_complete_watch_bootstrap_precedes_authoritative_baselines(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    authority_type = runtime_module._RepositorySnapshotAuthority
    original_open_chain = authority_type._open_initial_chain
    original_monitor = authority_type._open_change_monitor
    original_watch = authority_type._watch_directory
    original_drain = authority_type._assert_no_monitored_changes
    original_provisional = authority_type._discover_provisional_membership
    original_discover = authority_type._discover_complete_membership
    events = []

    def open_chain(authority):
        events.append("provisional-open")
        return original_open_chain(authority)

    def open_monitor():
        events.append("monitor-open")
        return original_monitor()

    def watch(authority, descriptor, metadata, scope, mask, expected):
        if scope in {"parent", "root_parent", "root"} and not authority._chain_metadata:
            assert authority._chain_metadata == ()
            events.append(f"watch-{scope}")
        return original_watch(authority, descriptor, metadata, scope, mask, expected)

    def drain(authority):
        events.append("setup-drain" if not authority._chain_metadata else "active-drain")
        return original_drain(authority)

    def provisional(authority):
        events.append("provisional-discovery")
        assert not authority._chain_metadata
        return original_provisional(authority)

    def discover(authority):
        events.append(f"watched-discovery:{authority._bootstrap_phase}")
        return original_discover(authority)

    monkeypatch.setattr(authority_type, "_open_initial_chain", open_chain)
    monkeypatch.setattr(authority_type, "_open_change_monitor", staticmethod(open_monitor))
    monkeypatch.setattr(authority_type, "_watch_directory", watch)
    monkeypatch.setattr(authority_type, "_assert_no_monitored_changes", drain)
    monkeypatch.setattr(authority_type, "_discover_provisional_membership", provisional)
    monkeypatch.setattr(authority_type, "_discover_complete_membership", discover)
    post_implementation_source_snapshot(root)
    assert events.index("provisional-open") < events.index("monitor-open")
    assert events.index("monitor-open") < min(
        index for index, value in enumerate(events) if value.startswith("watch-")
    )
    assert events.index("setup-drain") < events.index("provisional-discovery")
    assert events.index("provisional-discovery") < events.index(
        "watched-discovery:post_watch_reconciliation"
    )
    assert "watched-discovery:complete" in events


def test_source_snapshot_provisional_enumeration_precedes_complete_watched_enumeration(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    authority_type = runtime_module._RepositorySnapshotAuthority
    original = authority_type._enumerate_complete_membership
    phases = []

    def enumerate_membership(authority, *, require_watched):
        result = original(authority, require_watched=require_watched)
        identities = {
            observation.metadata.physical_identity
            for observation in result[1].values()
        }
        if require_watched:
            assert identities == authority._source_watch_identities
        phases.append((require_watched, tuple(sorted(result[1]))))
        return result

    monkeypatch.setattr(authority_type, "_enumerate_complete_membership", enumerate_membership)
    post_implementation_source_snapshot(root)
    assert phases[0] == (False, ("", "src", "src/pkg"))
    assert any(require_watched for require_watched, _ in phases)


@pytest.mark.parametrize(
    "mutation",
    (
        "create-remove-file",
        "create-remove-directory",
        "content-restore",
        "mode-restore",
        "same-size-content-restore",
        "inode-replace-restore",
        "rename-away-back",
        "symlink-add-remove",
    ),
)
def test_source_snapshot_pre_child_watch_mutation_is_rejected_from_provisional_facts(
    tmp_path, monkeypatch, mutation,
):
    root, files = _complete_bootstrap_snapshot_fixture(tmp_path)
    target = root / "config"
    member = files["config/settings.py"]
    original_bytes = member.read_bytes()
    original_mode = stat.S_IMODE(member.stat().st_mode)

    def mutate():
        if mutation == "create-remove-file":
            transient = target / "transient.py"
            transient.write_bytes(b"transient = True\n")
            transient.unlink()
        elif mutation == "create-remove-directory":
            transient = target / "transient"
            transient.mkdir()
            transient.rmdir()
        elif mutation == "content-restore":
            member.write_bytes(b"changed = 2\n")
            member.write_bytes(original_bytes)
        elif mutation == "mode-restore":
            member.chmod(original_mode ^ 0o020)
            member.chmod(original_mode)
        elif mutation == "same-size-content-restore":
            member.write_bytes(b"X" * len(original_bytes))
            member.write_bytes(original_bytes)
        elif mutation == "inode-replace-restore":
            displaced = member.with_name("settings-original.py")
            os.rename(member, displaced)
            shutil.copy2(displaced, member)
            member.unlink()
            os.rename(displaced, member)
        elif mutation == "rename-away-back":
            displaced = member.with_name("settings-away.py")
            os.rename(member, displaced)
            os.rename(displaced, member)
        else:
            link = target / "transient-link"
            link.symlink_to(member.name)
            link.unlink()

    reached = _install_pre_child_watch_mutation(
        monkeypatch,
        "config",
        mutate,
    )
    before = _repository_fd_count()
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert reached == [(0, 5)]
    assert raised.value.code == "source_snapshot_member_changed"
    assert member.read_bytes() == original_bytes
    assert stat.S_IMODE(member.stat().st_mode) == original_mode
    assert _repository_fd_count() == before


@pytest.mark.parametrize(
    ("ordinal", "target_path"),
    (("first", "config"), ("middle", "src"), ("last", "src/pkg/deep")),
)
def test_source_snapshot_first_middle_last_watch_gap_rejects_nested_create_delete(
    tmp_path, monkeypatch, ordinal, target_path,
):
    root, _ = _complete_bootstrap_snapshot_fixture(tmp_path)
    target = root / target_path

    def mutate():
        transient = target / f"{ordinal}-transient.py"
        transient.write_bytes(b"temporary\n")
        transient.unlink()

    reached = _install_pre_child_watch_mutation(monkeypatch, target_path, mutate)
    before = _repository_fd_count()
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert len(reached) == 1
    index, total = reached[0]
    assert total == 5
    assert index == {"first": 0, "middle": 2, "last": 4}[ordinal]
    assert raised.value.code == "source_snapshot_member_changed"
    assert _repository_fd_count() == before


@pytest.mark.parametrize(
    ("stage", "target_path"),
    (
        ("during-watch-install", "config"),
        ("after-final-watch-before-drain", "src/pkg/deep"),
        ("during-setup-drain", "src"),
        ("post-watch-reconciliation", "src/pkg"),
    ),
)
def test_source_snapshot_complete_watch_bootstrap_races_fail_closed(
    tmp_path, monkeypatch, stage, target_path,
):
    root, files = _complete_bootstrap_snapshot_fixture(tmp_path)
    member = (
        files["config/settings.py"]
        if target_path == "config"
        else files["src/pkg/a.py"]
    )
    original_bytes = member.read_bytes()
    authority_type = runtime_module._RepositorySnapshotAuthority
    invoked = []

    def mutate():
        if invoked:
            return
        invoked.append(stage)
        member.write_bytes(b"X" * len(original_bytes))
        member.write_bytes(original_bytes)

    if stage == "during-watch-install":
        original = authority_type._watch_directory
        target_identity = (root / target_path).stat()

        def watch(authority, descriptor, metadata, scope, mask, expected):
            result = original(authority, descriptor, metadata, scope, mask, expected)
            if (
                scope == "member"
                and (metadata.device, metadata.inode)
                == (target_identity.st_dev, target_identity.st_ino)
            ):
                mutate()
            return result

        monkeypatch.setattr(authority_type, "_watch_directory", watch)
    elif stage == "after-final-watch-before-drain":
        original = authority_type._after_complete_source_watch_set

        def after(authority):
            original(authority)
            mutate()

        monkeypatch.setattr(authority_type, "_after_complete_source_watch_set", after)
    elif stage == "during-setup-drain":
        original = authority_type._assert_no_monitored_changes

        def drain(authority):
            if authority._bootstrap_phase == "source_watches_installed":
                mutate()
            return original(authority)

        monkeypatch.setattr(authority_type, "_assert_no_monitored_changes", drain)
    else:
        original = authority_type._capture_bootstrap_inventory

        def capture(authority, *, require_watched):
            if require_watched:
                mutate()
            return original(authority, require_watched=require_watched)

        monkeypatch.setattr(authority_type, "_capture_bootstrap_inventory", capture)

    before = _repository_fd_count()
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert invoked == [stage]
    assert raised.value.code == "source_snapshot_member_changed"
    assert member.read_bytes() == original_bytes
    assert _repository_fd_count() == before


def test_source_snapshot_pre_child_watch_unrelated_ancestor_activity_is_accepted(
    tmp_path, monkeypatch,
):
    root, _ = _complete_bootstrap_snapshot_fixture(tmp_path)
    sibling = root.parent / "unrelated-bootstrap-sibling"
    reached = _install_pre_child_watch_mutation(
        monkeypatch,
        "config",
        lambda: _exercise_unrelated_sibling(sibling, "directory"),
    )
    before = _repository_fd_count()
    raw, _, count = post_implementation_source_snapshot(root)
    assert reached == [(0, 5)]
    assert count == 5
    assert inspect_post_implementation_source_snapshot(raw).member_count == 5
    gc.collect()
    assert _repository_fd_count() == before


def test_source_snapshot_complete_watch_set_matches_private_provisional_directories(
    tmp_path, monkeypatch,
):
    root, _ = _complete_bootstrap_snapshot_fixture(tmp_path)
    authority_type = runtime_module._RepositorySnapshotAuthority
    original = authority_type._after_complete_source_watch_set
    observed = []

    def after(authority):
        inventory = authority._provisional_inventory
        assert inventory.phase == "provisional_captured"
        expected = {
            item.metadata.physical_identity for item in inventory.directories
        }
        assert expected == authority._source_watch_identities
        assert authority._chain_metadata == ()
        observed.append((len(inventory.directories), len(inventory.members)))
        return original(authority)

    monkeypatch.setattr(authority_type, "_after_complete_source_watch_set", after)
    raw, _, count = post_implementation_source_snapshot(root)
    assert observed == [(6, 5)]
    assert count == 5
    assert inspect_post_implementation_source_snapshot(raw).build_authoritative is False


def test_source_snapshot_private_provisional_inventory_cannot_be_reused_or_confer_authority(
    tmp_path,
):
    root, _ = _complete_bootstrap_snapshot_fixture(tmp_path)
    authority = runtime_module._RepositorySnapshotAuthority(str(root))
    inventory = authority._provisional_inventory
    assert type(inventory) is runtime_module._SourceSnapshotProvisionalInventory
    assert inventory.phase == "provisional_captured"
    assert len(inventory.directories) == 6
    assert len(inventory.members) == 5
    with pytest.raises(Exception):
        inventory.phase = "complete"
    with pytest.raises(Slice7GRuntimeError) as replay:
        authority._install_complete_source_watch_set(inventory)
    assert replay.value.code == "source_snapshot_authority_state"
    assert authority.close() == ()
    with pytest.raises(Slice7GRuntimeError) as closed:
        authority._install_complete_source_watch_set(inventory)
    assert closed.value.code == "source_snapshot_authority_closed"


@pytest.mark.parametrize("member_watch", (1, 3, 5), ids=("first", "middle", "last"))
def test_source_snapshot_bootstrap_watch_failure_cleans_all_descriptors(
    tmp_path, monkeypatch, member_watch,
):
    root, _ = _complete_bootstrap_snapshot_fixture(tmp_path)
    authority_type = runtime_module._RepositorySnapshotAuthority
    original = authority_type._watch_directory
    calls = 0

    def watch(authority, descriptor, metadata, scope, mask, expected):
        nonlocal calls
        if scope == "member":
            calls += 1
            if calls == member_watch:
                raise Slice7GRuntimeError(
                    "source_snapshot_monitor",
                    "injected bootstrap watch failure",
                )
        return original(authority, descriptor, metadata, scope, mask, expected)

    before = _repository_fd_count()
    monkeypatch.setattr(authority_type, "_watch_directory", watch)
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert calls == member_watch
    assert raised.value.code == "source_snapshot_monitor"
    assert _repository_fd_count() == before


def test_source_snapshot_bootstrap_watch_baseexception_attempts_complete_cleanup(
    tmp_path, monkeypatch,
):
    root, _ = _complete_bootstrap_snapshot_fixture(tmp_path)
    authority_type = runtime_module._RepositorySnapshotAuthority
    original = authority_type._watch_directory

    class BootstrapAbort(BaseException):
        pass

    def watch(authority, descriptor, metadata, scope, mask, expected):
        if scope == "member":
            raise BootstrapAbort("bootstrap watch abort")
        return original(authority, descriptor, metadata, scope, mask, expected)

    before = _repository_fd_count()
    monkeypatch.setattr(authority_type, "_watch_directory", watch)
    with pytest.raises(BootstrapAbort, match="bootstrap watch abort"):
        post_implementation_source_snapshot(root)
    gc.collect()
    assert _repository_fd_count() == before


def test_source_snapshot_root_replacement_during_monitor_setup_is_rejected(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path)
    displaced = tmp_path / "repo-displaced"
    authority_type = runtime_module._RepositorySnapshotAuthority
    original_watch = authority_type._watch_directory
    replaced = False

    def replace_after_root_watch(authority, descriptor, metadata, scope, mask, expected):
        nonlocal replaced
        result = original_watch(authority, descriptor, metadata, scope, mask, expected)
        if scope == "root" and not replaced:
            replaced = True
            os.rename(root, displaced)
            shutil.copytree(displaced, root, copy_function=shutil.copy2)
        return result

    before = _repository_fd_count()
    monkeypatch.setattr(authority_type, "_watch_directory", replace_after_root_watch)
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert raised.value.code == "source_snapshot_root_replaced"
    assert _repository_fd_count() == before


def test_source_snapshot_parent_replacement_during_monitor_setup_is_rejected(tmp_path, monkeypatch):
    root, _, _ = _complete_snapshot_fixture(tmp_path, parent="outer")
    parent = root.parent
    displaced = tmp_path / "outer-displaced"
    authority_type = runtime_module._RepositorySnapshotAuthority
    original_watch = authority_type._watch_directory
    replaced = False

    def replace_after_parent_watch(authority, descriptor, metadata, scope, mask, expected):
        nonlocal replaced
        result = original_watch(authority, descriptor, metadata, scope, mask, expected)
        if scope == "parent" and expected == "outer" and not replaced:
            replaced = True
            os.rename(parent, displaced)
            shutil.copytree(displaced, parent, copy_function=shutil.copy2)
        return result

    before = _repository_fd_count()
    monkeypatch.setattr(authority_type, "_watch_directory", replace_after_parent_watch)
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert raised.value.code == "source_snapshot_parent_replaced"
    assert _repository_fd_count() == before


@pytest.mark.parametrize(
    ("parent", "target_scope", "expected_code"),
    [
        ("", "root", "source_snapshot_root_replaced"),
        ("outer", "root_parent", "source_snapshot_parent_replaced"),
    ],
    ids=("root-metadata-restored", "parent-metadata-restored"),
)
def test_source_snapshot_setup_change_and_restore_is_rejected_before_baseline(
    tmp_path, monkeypatch, parent, target_scope, expected_code,
):
    root, _, _ = _complete_snapshot_fixture(tmp_path, parent=parent)
    target = root if target_scope == "root" else root.parent
    authority_type = runtime_module._RepositorySnapshotAuthority
    original_watch = authority_type._watch_directory
    mutated = False

    def mutate_after_watch(authority, descriptor, metadata, scope, mask, expected):
        nonlocal mutated
        result = original_watch(authority, descriptor, metadata, scope, mask, expected)
        if scope == target_scope and not mutated:
            mutated = True
            initial = target.stat().st_mode & 0o7777
            target.chmod(initial ^ 0o020)
            target.chmod(initial)
        return result

    before = _repository_fd_count()
    monkeypatch.setattr(authority_type, "_watch_directory", mutate_after_watch)
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert raised.value.code == expected_code
    assert _repository_fd_count() == before

    authority = runtime_module._RepositorySnapshotAuthority(str(root))
    authority.close()
    authority.close()
    with pytest.raises(Slice7GRuntimeError) as closed:
        authority._discover_complete_membership()
    assert closed.value.code == "source_snapshot_authority_closed"
    gc.collect()
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before


@pytest.mark.parametrize(
    "stage",
    ("monitor-setup", "first-pass", "between-passes", "final-rediscovery", "final-drain"),
)
@pytest.mark.parametrize("kind", ("file", "directory"))
@pytest.mark.parametrize("level", ("tmp", "intermediate-parent", "root-parent"))
def test_source_snapshot_unrelated_ancestor_sibling_activity_is_accepted(
    tmp_path, monkeypatch, stage, kind, level,
):
    root, _, _ = _complete_snapshot_fixture(tmp_path, parent="outer/middle")
    suffix = f"slice7g-{tmp_path.name}-{level}-{kind}-{stage}"
    if level == "tmp":
        sibling = Path("/tmp") / suffix
    elif level == "intermediate-parent":
        sibling = tmp_path / suffix
    else:
        sibling = root.parent / suffix

    _install_snapshot_stage_mutation(
        monkeypatch,
        root,
        stage,
        lambda: _exercise_unrelated_sibling(sibling, kind),
        watched_parent=sibling.parent,
    )
    before = _repository_fd_count()
    raw, _, count = post_implementation_source_snapshot(root)
    assert count == 2
    assert inspect_post_implementation_source_snapshot(raw).member_count == 2
    gc.collect()
    assert _repository_fd_count() == before


@pytest.mark.parametrize("kind", ("file", "directory"))
@pytest.mark.parametrize("level", ("tmp", "intermediate-parent", "root-parent"))
def test_source_snapshot_concurrent_unrelated_ancestor_sibling_activity_is_accepted(
    tmp_path, monkeypatch, kind, level,
):
    root, _, _ = _complete_snapshot_fixture(tmp_path, parent="outer/middle")
    suffix = f"slice7g-concurrent-{tmp_path.name}-{level}-{kind}"
    if level == "tmp":
        sibling = Path("/tmp") / suffix
    elif level == "intermediate-parent":
        sibling = tmp_path / suffix
    else:
        sibling = root.parent / suffix
    authority_type = runtime_module._RepositorySnapshotAuthority
    original = authority_type._authenticate_members
    invoked = False

    def wrapped(authority, paths, *, final):
        nonlocal invoked
        if final or invoked:
            return original(authority, paths, final=final)
        invoked = True
        errors = []

        def worker():
            try:
                for _ in range(4):
                    _exercise_unrelated_sibling(sibling, kind)
            except BaseException as exc:  # captured and asserted in the test thread owner
                errors.append(exc)

        thread = threading.Thread(target=worker)
        thread.start()
        try:
            result = original(authority, paths, final=final)
        finally:
            thread.join()
        assert not errors
        return result

    monkeypatch.setattr(authority_type, "_authenticate_members", wrapped)
    before = _repository_fd_count()
    raw, _, count = post_implementation_source_snapshot(root)
    assert count == 2
    assert inspect_post_implementation_source_snapshot(raw).member_count == 2
    gc.collect()
    assert _repository_fd_count() == before


@pytest.mark.parametrize(
    "stage",
    ("monitor-setup", "first-pass", "between-passes", "final-rediscovery", "final-drain"),
)
@pytest.mark.parametrize(
    ("component", "expected_code"),
    (
        ("intermediate-parent", "source_snapshot_parent_replaced"),
        ("repository-root", "source_snapshot_root_replaced"),
    ),
)
def test_source_snapshot_bound_component_change_restore_remains_rejected(
    tmp_path, monkeypatch, stage, component, expected_code,
):
    root, _, _ = _complete_snapshot_fixture(tmp_path, parent="outer/middle")
    target = tmp_path / "outer" if component == "intermediate-parent" else root
    original_mode = stat.S_IMODE(target.stat().st_mode)

    def mutate_and_restore():
        target.chmod(original_mode ^ 0o020)
        target.chmod(original_mode)

    _install_snapshot_stage_mutation(
        monkeypatch,
        root,
        stage,
        mutate_and_restore,
        watched_parent=target.parent,
    )
    before = _repository_fd_count()
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert raised.value.code == expected_code
    assert _repository_fd_count() == before


@pytest.mark.parametrize(
    "stage",
    ("monitor-setup", "first-pass", "between-passes", "final-rediscovery", "final-drain"),
)
@pytest.mark.parametrize(
    ("mutation", "expected_code"),
    (
        ("parent-rename-restore", "source_snapshot_parent_replaced"),
        ("parent-byte-identical-replacement", "source_snapshot_parent_replaced"),
        ("parent-symlink-substitution", "source_snapshot_parent_replaced"),
        ("root-byte-identical-replacement", "source_snapshot_root_replaced"),
        ("source-member-add-remove", "source_snapshot_member_changed"),
    ),
)
def test_source_snapshot_relevant_component_and_tree_controls_fail_at_each_stage(
    tmp_path, monkeypatch, stage, mutation, expected_code,
):
    root, first, _ = _complete_snapshot_fixture(tmp_path, parent="outer/middle")
    parent = tmp_path / "outer"
    displaced_parent = tmp_path / "outer-displaced"
    displaced_root = root.with_name("repo-displaced")

    def mutate():
        if mutation == "parent-rename-restore":
            os.rename(parent, displaced_parent)
            os.rename(displaced_parent, parent)
        elif mutation == "parent-byte-identical-replacement":
            os.rename(parent, displaced_parent)
            shutil.copytree(displaced_parent, parent, copy_function=shutil.copy2)
        elif mutation == "parent-symlink-substitution":
            os.rename(parent, displaced_parent)
            parent.symlink_to(displaced_parent, target_is_directory=True)
        elif mutation == "root-byte-identical-replacement":
            os.rename(root, displaced_root)
            shutil.copytree(displaced_root, root, copy_function=shutil.copy2)
        else:
            transient = first.parent / "transient.py"
            transient.write_bytes(b"transient = True\n")
            transient.unlink()

    if mutation.startswith("parent-"):
        watched_parent = parent.parent
    elif mutation.startswith("root-"):
        watched_parent = root.parent
    else:
        watched_parent = first.parent
    _install_snapshot_stage_mutation(
        monkeypatch,
        root,
        stage,
        mutate,
        watched_parent=watched_parent,
    )
    before = _repository_fd_count()
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(root)
    gc.collect()
    assert raised.value.code == expected_code
    assert _repository_fd_count() == before


def test_source_snapshot_ancestor_identity_excludes_incidental_directory_metadata(tmp_path):
    directory = tmp_path / "ancestor"
    directory.mkdir()
    before = runtime_module._ancestor_component_identity(directory.stat())
    sibling = directory / "unrelated"
    sibling.mkdir()
    after = runtime_module._ancestor_component_identity(directory.stat())
    assert before == after
    assert tuple(before.__dataclass_fields__) == ("device", "inode", "file_type", "mode")
    full_before = runtime_module._source_tree_metadata(os.stat(directory))
    sibling.rmdir()
    full_after = runtime_module._source_tree_metadata(os.stat(directory))
    assert full_before != full_after


def test_source_snapshot_ancestor_monitor_compares_bound_unicode_name_as_exact_bytes():
    authority = _monitor_parser_authority(scope="parent", expected="caf\N{LATIN SMALL LETTER E WITH ACUTE}")
    with pytest.raises(Slice7GRuntimeError) as raised:
        authority._handle_monitor_event(7, authority._IN_ATTRIB, "caf\N{LATIN SMALL LETTER E WITH ACUTE}".encode())
    assert raised.value.code == "source_snapshot_parent_replaced"
    authority._handle_monitor_event(
        7,
        authority._IN_ATTRIB,
        "cafe\N{COMBINING ACUTE ACCENT}".encode(),
    )


def test_canonical_node_id_serialization_is_pure_sorted_utf8_without_trailing_lf():
    assert len(_APPLICABLE_NODE_ACCOUNTING_PATHS) == 17
    assert len(set(_APPLICABLE_NODE_ACCOUNTING_PATHS)) == 17
    assert _SUPERSEDED_AUTHORING_SNAPSHOT_NODE_IDS == (
        "src/ctr_bringup/test/test_slice_7g_governance.py::test_exact_positive_charter_and_source_snapshot",
        "src/ctr_bringup/test/test_slice_7g_governance.py::test_snapshot_descriptors_close_on_success_and_failure",
    )
    assert len(_PREBUILD_INTERFACE_SHIM_CONTRACT) == 4
    collected = [
        "src/z/test_b.py::test_\N{GREEK SMALL LETTER BETA}",
        "src/a/test_a.py::test_alpha[param]",
    ]
    expected = (
        "src/a/test_a.py::test_alpha[param]\n"
        "src/z/test_b.py::test_\N{GREEK SMALL LETTER BETA}"
    ).encode("utf-8")
    assert _canonical_node_id_bytes(collected) == expected
    assert not expected.endswith(b"\n")
    assert hashlib.sha256(_canonical_node_id_bytes(tuple(reversed(collected)))).digest() == hashlib.sha256(expected).digest()
    with pytest.raises(ValueError, match="duplicate"):
        _canonical_node_id_bytes([collected[0], collected[0]])


def test_coordinator_commits_attempt_before_any_process_and_runs_exact_plan(tmp_path):
    charter = _charter()
    authorization, initial = _authorization(tmp_path, charter)
    root = tmp_path / "campaign"
    ledger_parent = tmp_path / "ledger"
    ledger_parent.mkdir()
    ledger_writer = AtomicSlice7GLedgerWriter(ledger_parent)
    releases = []
    allocator = Slice7GDomainAllocator(
        lambda domain: _occupancy(domain, clear=domain == 117),
        lambda domain, *args: "c" * 64,
        lambda domain, lease: releases.append((domain, lease)) or "d" * 64,
    )

    class FakeEvidenceWriter:
        campaign_root = root

        def __init__(self):
            self.cells = []
            self.sealed = False

        def write_cell_package(self, execution, charter_arg, ledger, plan):
            self.cells.append(execution.cell_result.cell_id)
            return root / "evidence" / execution.cell_result.cell_id, "e" * 64, hashlib.sha256(
                execution.cell_result.cell_id.encode()
            ).hexdigest()

        def write_campaign_seal(self, charter_arg, ledger, plan, identities):
            assert len(identities) == 15
            self.sealed = True

    evidence = FakeEvidenceWriter()
    process_calls = []

    def process(argv, env):
        assert (ledger_parent / "attempt_ledger.r00000002.json").is_file()
        allocated = ledger_writer._read_commit(ledger_parent / "attempt_ledger.r00000001.json")
        committed = ledger_writer._read_commit(ledger_parent / "attempt_ledger.r00000002.json")
        plan = generate_slice_7g_campaign_plan(charter, allocated)
        cell = next(item for item in plan.cells if item.argv == tuple(argv))
        assert env["ROS_DOMAIN_ID"] == "117"
        assert env["CTR_SLICE_7G_ATTEMPT_LEDGER_IDENTITY"] == slice_7g_attempt_ledger_identity(committed)
        assert len(env["CTR_SLICE_7G_DOMAIN_LEASE_IDENTITY"]) == 64
        process_calls.append(cell.cell_id)
        return _execution(cell, plan, committed)

    coordinator = Slice7GCampaignCoordinator(
        charter_path=CHARTER_PATH,
        authorization_path=tmp_path / "authorization.json",
        ledger_writer=ledger_writer,
        domain_allocator=allocator,
        output_allocator=lambda auth: (root.mkdir(), auth.campaign_output_root)[1],
        preflight=lambda auth: None,
        process_factory=process,
        evidence_writer=evidence,
        timestamp_factory=lambda: STAMP,
    )
    sentinel = object()
    with mock.patch("ctr_evaluation.slice_7g_runtime.reconcile_slice_7g_campaign_results", return_value=sentinel):
        assert coordinator.run() is sentinel
    assert len(process_calls) == 15 and len(set(process_calls)) == 15
    assert evidence.sealed is True
    assert releases and releases[0][0] == 117
    committed = ledger_writer._read_commit(ledger_parent / "attempt_ledger.r00000002.json")
    assert committed.consumed_campaign_attempts == 1
    assert committed.process_start_committed is True
    assert (ledger_parent / "domain_binding.json").stat().st_mode & 0o777 == 0o444


def test_coordinator_commit_failure_prohibits_process_creation(tmp_path):
    charter = _charter()
    authorization, initial = _authorization(tmp_path, charter)
    root = tmp_path / "campaign"
    processes = []

    class FailingWriter:
        def initialize(self, ledger):
            return ledger

        def commit(self, current, event, **kwargs):
            if event.event_kind == "process_start_commit":
                raise Slice7GRuntimeError("injected_commit_failure", "failed")
            return validate_slice_7g_attempt_transition(current, event, **kwargs)

        def commit_domain_binding(self, binding):
            return None

    evidence = type("Evidence", (), {"campaign_root": root})()
    allocator = Slice7GDomainAllocator(
        lambda domain: _occupancy(domain), lambda *args: "a" * 64, lambda *args: "b" * 64,
    )
    coordinator = Slice7GCampaignCoordinator(
        charter_path=CHARTER_PATH,
        authorization_path=tmp_path / "authorization.json",
        ledger_writer=FailingWriter(),
        domain_allocator=allocator,
        output_allocator=lambda auth: (root.mkdir(), auth.campaign_output_root)[1],
        preflight=lambda auth: None,
        process_factory=lambda argv, env: processes.append((argv, env)),
        evidence_writer=evidence,
        timestamp_factory=lambda: STAMP,
    )
    with pytest.raises(Slice7GRuntimeError) as raised:
        coordinator.run()
    assert raised.value.code == "injected_commit_failure"
    assert processes == []


def test_coordinator_never_retries_after_process_factory_failure(tmp_path):
    charter = _charter()
    authorization, _ = _authorization(tmp_path, charter)
    root = tmp_path / "campaign"
    ledger_parent = tmp_path / "ledger"
    ledger_parent.mkdir()
    calls = []
    releases = []
    evidence = type("Evidence", (), {"campaign_root": root})()
    coordinator = Slice7GCampaignCoordinator(
        charter_path=CHARTER_PATH,
        authorization_path=tmp_path / "authorization.json",
        ledger_writer=AtomicSlice7GLedgerWriter(ledger_parent),
        domain_allocator=Slice7GDomainAllocator(
            lambda domain: _occupancy(domain),
            lambda *args: "a" * 64,
            lambda domain, lease: releases.append((domain, lease)) or "b" * 64,
        ),
        output_allocator=lambda auth: (root.mkdir(), auth.campaign_output_root)[1],
        preflight=lambda auth: None,
        process_factory=lambda argv, env: calls.append((argv, env)) or (_ for _ in ()).throw(
            Slice7GRuntimeError("cell_failed", "injected")
        ),
        evidence_writer=evidence,
        timestamp_factory=lambda: STAMP,
    )
    with pytest.raises(Slice7GRuntimeError) as raised:
        coordinator.run()
    assert raised.value.code == "cell_failed"
    assert len(calls) == 1
    assert len(releases) == 1
    committed = coordinator.ledger_writer._read_commit(ledger_parent / "attempt_ledger.r00000002.json")
    assert committed.consumed_campaign_attempts == 1 and committed.retry_count == 0


class _ProductionEffects(Slice7GProductionEffects):
    def __init__(self, *, collision=0, stdout=b"ok", stderr=b"", returncode=0):
        self.cells = []
        self.collision = collision
        self.stdout = stdout
        self.stderr = stderr
        self.returncode = returncode
        self._monotonic = 0.0

    def utc_now(self):
        return STAMP

    def which(self, executable):
        assert executable in {"ctr_run_evaluation", "ros2"}
        return f"/usr/bin/{executable}"

    def active_process_records(self):
        return ()

    def udp_socket_tables(self):
        header = b"sl local_address rem_address st tx_queue rx_queue tr tm->when retrnsmt uid timeout inode\n"
        return header, header

    def run_observer(self, argv, env, timeout_seconds):
        raise AssertionError("legacy bare observer must not be used")

    def monotonic(self):
        return self._monotonic

    def sleep(self, seconds):
        self._monotonic += seconds

    def graph_observer_contract(self, domain_id):
        environment = {
            "AMENT_PREFIX_PATH": "/opt/ros/humble",
            "HOME": "/opt/ctr-mppi/runtime-home",
            "LD_LIBRARY_PATH": "/opt/ros/humble/lib",
            "PATH": "/opt/ros/humble/bin:/usr/bin",
            "PYTHONNOUSERSITE": "1",
            "PYTHONDONTWRITEBYTECODE": "1",
            "PYTHONPATH": "/opt/ros/humble/lib/python3.10/site-packages",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "ROS_DOMAIN_ID": str(domain_id),
            "ROS_HOME": "/opt/ctr-mppi/runtime-home/ros",
            "ROS_LOCALHOST_ONLY": "1",
            "XDG_CACHE_HOME": "/opt/ctr-mppi/runtime-home/cache",
        }
        return Slice7GROSGraphObserverContract(
            "/opt/ros/humble/bin/ros2", "0" * 64,
            "/usr/bin/python3", "1" * 64, ("2" * 64,),
            ("/opt/ros/humble/bin/ros2", "node", "list", "--no-daemon"),
            tuple(sorted(environment.items())), "3" * 64,
            "/opt/ctr-mppi/slice-7g/fixed",
            "/system.slice/ctr-slice7g-campaign.service",
            "rmw_fastrtps_cpp",
        )

    def run_graph_observer(self, contract):
        return Slice7GROSGraphObserverExecution(
            4321, 4321, 66, 1, 2, 0, b"", b"",
        )

    def observer_cleanup_sample(self, execution, domain_id):
        return {
            "process_present": False,
            "process_group_present": False,
            "descendant_pids": (),
            "ros_daemon_pids": (),
            "matching_udp_ports": (),
        }

    @staticmethod
    def _option(argv, name):
        return argv[argv.index(name) + 1]

    def run_cell(self, argv, env, timeout_seconds):
        del timeout_seconds
        root = Path(env["CTR_SLICE_7G_CELL_OUTPUT_ROOT"])
        campaign_root = Path(env["CTR_SLICE_7G_CAMPAIGN_OUTPUT_ROOT"])
        committed = campaign_root.with_name(f".{campaign_root.name}.slice_7g_control") / "attempt_ledger" / "attempt_ledger.r00000002.json"
        assert committed.is_file()
        root.mkdir(mode=0o700, parents=True)
        baseline = root / "baseline"
        candidate = root / "candidate"
        baseline.mkdir()
        candidate.mkdir()
        summary = _summary(collision=self.collision)
        orchestration = {
            "initial_state_stability": {
                "sample_count": 10,
                "duration_s": 0.5,
                "max_q_variation": 0.0,
                "max_tip_variation": 0.0,
            },
            "readiness_diagnostics": {
                "criteria": {
                    "finite_values": True,
                    "sample_count": True,
                    "duration": True,
                    "q_variation": True,
                    "tip_variation": True,
                },
                "readiness_result": True,
                "slice_7g_readiness_snapshot": {
                    "authenticated": True,
                    "tactile_receive_age_seconds": 0.01,
                    "tactile_valid": True,
                    "safety_receive_age_seconds": 0.01,
                    "safety_ready": True,
                    "safety_fault": False,
                },
            },
        }
        runtime_module = __import__("ctr_evaluation.slice_7g_runtime", fromlist=["REQUIRED_RUN_ARTIFACTS"])
        for directory in (baseline, candidate):
            for name in runtime_module.REQUIRED_RUN_ARTIFACTS:
                path = directory / name
                if name == "summary.json":
                    path.write_text(json.dumps(summary, sort_keys=True, separators=(",", ":")), encoding="utf-8")
                elif name == "orchestration.json":
                    path.write_text(json.dumps(orchestration, sort_keys=True, separators=(",", ":")), encoding="utf-8")
                else:
                    path.write_text("retained\n", encoding="utf-8")
        receipt = {
            "schema_version": "ctr-slice-7g-runner-result-receipt-1",
            "charter_logical_identity": env["CTR_SLICE_7G_CHARTER_IDENTITY"],
            "runtime_authorization_identity": env["CTR_SLICE_7G_RUNTIME_AUTHORIZATION_IDENTITY"],
            "attempt_ledger_identity": env["CTR_SLICE_7G_ATTEMPT_LEDGER_IDENTITY"],
            "attempt_ledger_revision": int(env["CTR_SLICE_7G_ATTEMPT_LEDGER_REVISION"]),
            "process_start_event_identity": env["CTR_SLICE_7G_PROCESS_START_EVENT_IDENTITY"],
            "campaign_plan_identity": env["CTR_SLICE_7G_CAMPAIGN_PLAN_IDENTITY"],
            "domain_lease_identity": env["CTR_SLICE_7G_DOMAIN_LEASE_IDENTITY"],
            "domain_committed_binding_identity": env["CTR_SLICE_7G_DOMAIN_COMMITTED_BINDING_IDENTITY"],
            "cell_id": env["CTR_SLICE_7G_CELL_ID"],
            "campaign_id": env["CTR_SLICE_7G_CAMPAIGN_ID"],
            "campaign_output_root": env["CTR_SLICE_7G_CAMPAIGN_OUTPUT_ROOT"],
            "cell_output_root": env["CTR_SLICE_7G_CELL_OUTPUT_ROOT"],
            "ros_domain_id": int(env["ROS_DOMAIN_ID"]),
            "task": self._option(argv, "--task"),
            "geometry": self._option(argv, "--curved-lumen-type"),
            "scenario": self._option(argv, "--scenario"),
            "seed": int(self._option(argv, "--seed")),
            "duration_seconds": float(self._option(argv, "--duration")),
            "runtime_mode": self._option(argv, "--runtime-mode"),
            "argv": list(argv),
            "process_exit_status": self.returncode,
            "baseline_relative_path": "baseline",
            "candidate_relative_path": "candidate",
        }
        receipt_path = root / "slice_7g_runner_result.json"
        receipt_path.write_bytes(json.dumps(receipt, sort_keys=True, separators=(",", ":")).encode())
        receipt_path.chmod(0o444)
        self.cells.append(env["CTR_SLICE_7G_CELL_ID"])
        return Slice7GProcessObservation(
            tuple(argv), self.returncode, self.stdout, self.stderr,
        )


def _adapter_execution(tmp_path, effects=None):
    charter, authorization, _, _, plan, committed = _committed_context(tmp_path)
    cell = plan.cells[0]
    control = Path(authorization.campaign_output_root).with_name(
        f".{Path(authorization.campaign_output_root).name}.slice_7g_control"
    ) / "attempt_ledger"
    control.mkdir(parents=True)
    (control / "attempt_ledger.r00000002.json").write_text("retained", encoding="utf-8")
    env = {
        "ROS_DOMAIN_ID": str(cell.ros_domain_id),
        "ROS_DISTRO": "humble",
        "CTR_SLICE_7G_CHARTER_IDENTITY": plan.charter_logical_identity,
        "CTR_SLICE_7G_RUNTIME_AUTHORIZATION_IDENTITY": committed.runtime_authorization_identity,
        "CTR_SLICE_7G_ATTEMPT_LEDGER_IDENTITY": slice_7g_attempt_ledger_identity(committed),
        "CTR_SLICE_7G_ATTEMPT_LEDGER_REVISION": str(committed.revision),
        "CTR_SLICE_7G_PROCESS_START_EVENT_IDENTITY": committed.last_event_identity,
        "CTR_SLICE_7G_CAMPAIGN_PLAN_IDENTITY": slice_7g_campaign_plan_identity(plan),
        "CTR_SLICE_7G_DOMAIN_LEASE_IDENTITY": "6" * 64,
        "CTR_SLICE_7G_DOMAIN_COMMITTED_BINDING_IDENTITY": "7" * 64,
        "CTR_SLICE_7G_CELL_ID": cell.cell_id,
        "CTR_SLICE_7G_CAMPAIGN_ID": plan.campaign_id,
        "CTR_SLICE_7G_CAMPAIGN_OUTPUT_ROOT": cell.campaign_output_root,
        "CTR_SLICE_7G_CELL_OUTPUT_ROOT": cell.cell_output_path,
        "ROS_LOG_DIR": f"{cell.cell_output_path}/ros_log",
    }
    adapter = ProductionSlice7GRunnerAdapter(effects or _ProductionEffects())
    adapter.bind_campaign(charter, committed, plan)
    execution = adapter(cell.argv, env)
    authority = adapter.take_output_authority(execution)
    return charter, plan, committed, cell, execution, authority


def test_charter_v7_binds_exact_lease_guard_and_ros_graph_observer_policy():
    data = json.loads(CHARTER_PATH.read_bytes())
    policy = data["runtime_authority_contract"]["observation_policy"]
    expected = {
        "authority_owner": "root_observer_supervisor",
        "cleanup_anchor_owner": "root_cleanup_authority",
        "observer_class": "PRECOMMIT_ROS_GRAPH_OBSERVER",
        "executable": "/opt/ros/humble/bin/ros2",
        "argv": ["node", "list", "--no-daemon"],
        "shell": False, "timeout_seconds": 10.0,
        "maximum_stdout_bytes": 1_048_576,
        "maximum_stderr_bytes": 1_048_576,
        "maximum_precommit_observers": 100,
        "maximum_postcommit_observers": 1,
        "maximum_transaction_observers": 101,
        "concurrency": 1, "retries": 0, "unexpected_descendants": 0,
        "ros_daemon_allowed": False,
        "observation_session_lifetime_seconds": 1_800,
        "prepare_token_lifetime_seconds": 300,
        "cleanup_stable_samples": 2,
        "cleanup_minimum_interval_seconds": 0.5,
        "cleanup_maximum_wait_seconds": 5.0,
        "failure_invalidates_session": True,
        "receipt_replay_across_sessions": False,
        "server_owned_four_sources": True,
        "surviving_pgid_cleanup_required": True,
        "global_lease_observer_daemon_owned": True,
        "global_lease_registry": "/home/ankid/ctr_mppi_evidence/slice_7g/.ctr_slice_7g_domain_leases",
        "global_lease_lock": "registry.lock", "global_lease_clear_required": True,
        "cleanup_guard_durable_nonconsuming": True,
        "cleanup_guard_created_before_process": True,
        "cleanup_quarantine_survives_restart": True,
        "cleanup_recovery_production_available": False,
        "dedicated_process_session_required": True,
        "postexec_identity_reconciliation_required": True,
        "leader_reaped_after_provenance_and_cleanup": True,
        "exclusive_cgroup_before_exec": True,
        "setsid_and_double_fork_escape_prevented_by_cgroup": True,
        "sealed_output_memfd_count": 2,
        "leaf_cgroup_grammar": (
            "/system.slice/ctr-slice7g-observer-supervisor.service/"
            "observer-[0-9]{20}-[0-9a-f]{32}"
        ),
    }
    assert policy == expected
    assert data["runtime_authority_contract"]["global_budget"][
        "other_precommit_project_ros_children"
    ] == 0


@pytest.mark.parametrize("raw,code", [
    (b"relative\n", "observer_node_absolute"),
    (b"/valid\n\n/also_valid\n", "observer_stdout_empty_line"),
    (b"/duplicate\n/duplicate\n", "observer_node_duplicate"),
    (b"/has-dash\n", "observer_node_component"),
    (b"/valid\r\n", "observer_stdout_format"),
    (b"/valid\x00\n", "observer_stdout_format"),
    (b"\xff", "observer_stdout_utf8"),
])
def test_ros_graph_stdout_parser_is_strict(raw, code):
    with pytest.raises(Slice7GRuntimeError) as raised:
        runtime_module.parse_ros_graph_observer_stdout(raw)
    assert raised.value.code == code


def test_ros_graph_stdout_parser_accepts_empty_and_nfc_absolute_names():
    assert runtime_module.parse_ros_graph_observer_stdout(b"") == ()
    assert runtime_module.parse_ros_graph_observer_stdout(
        b"/ctr/simulator\n/ctr/safety_supervisor\n"
    ) == ("/ctr/simulator", "/ctr/safety_supervisor")


@pytest.mark.parametrize("mutation,code", [
    ({"executable": "ros2"}, "observer_executable"),
    ({"argv": ("/opt/ros/humble/bin/ros2", "node", "list")}, "observer_argv"),
    ({"working_directory": "relative"}, "absolute_path"),
    ({"cgroup": "/other.slice/escaped.scope"}, "observer_cgroup"),
])
def test_graph_observer_contract_rejects_path_argv_cwd_and_cgroup_substitution(mutation, code):
    contract = _ProductionEffects().graph_observer_contract(100)
    with pytest.raises(Slice7GRuntimeError) as raised:
        runtime_module._validated_graph_observer_contract(replace(contract, **mutation))
    assert raised.value.code == code


def test_graph_observer_contract_rejects_environment_inheritance_and_domain_substitution():
    contract = _ProductionEffects().graph_observer_contract(100)
    for environment in (
        (*contract.environment, ("CALLER_EXTRA", "hostile")),
        tuple((key, "200" if key == "ROS_DOMAIN_ID" else value) for key, value in contract.environment),
        tuple((key, "/hostile/bin" if key == "PATH" else value) for key, value in contract.environment),
    ):
        with pytest.raises(Slice7GRuntimeError, match="observer_environment"):
            runtime_module._validated_graph_observer_contract(
                replace(contract, environment=environment),
            )


def test_unprovisioned_public_entrypoint_is_effect_free_and_stably_missing_authority(capsys):
    assert runtime_module.main([]) == 2
    captured = capsys.readouterr()
    assert captured.out == ""
    assert "missing_authority" in captured.err


@pytest.mark.parametrize("residual_field", [
    "process_present", "process_group_present", "descendant_pids",
    "ros_daemon_pids", "matching_udp_ports",
])
def test_graph_observer_residual_barrier_fails_closed(residual_field, tmp_path):
    class Residual(_ProductionEffects):
        def observer_cleanup_sample(self, execution, domain_id):
            sample = super().observer_cleanup_sample(execution, domain_id)
            sample[residual_field] = (
                True if residual_field in {"process_present", "process_group_present"}
                else (999,)
            )
            return sample

    with pytest.raises(Slice7GRuntimeError) as raised:
        ProductionSlice7GDomainAuthority(tmp_path / "leases", Residual()).observe(100)
    assert raised.value.code == "observer_cleanup_uncertain"


def _write_authorization(tmp_path, name, campaign_id, output_root, charter=None):
    charter = charter or _charter()
    initial = create_slice_7g_initial_attempt_ledger(charter, campaign_id)
    data = {
        "schema_version": "ctr-slice-7g-runtime-authorization-1",
        "charter_logical_identity": slice_7g_charter_identity(charter),
        "campaign_id": campaign_id,
        "campaign_identity": initial.campaign_identity,
        "post_implementation_source_snapshot_identity": "a" * 64,
        "campaign_output_root": str(output_root),
        "issued_at_utc": STAMP,
        "execution_authorized": True,
    }
    path = tmp_path / name
    path.write_bytes(canonical_runtime_authorization_bytes(data))
    path.chmod(0o444)
    return path


def test_production_assembly_runs_real_adapter_writer_seal_and_reconciler(tmp_path):
    charter = _charter()
    authorization, _ = _authorization(tmp_path, charter)
    effects = _ProductionEffects()
    descriptors_before = len(tuple(Path("/proc/self/fd").iterdir()))
    coordinator = _assemble_slice_7g_production_coordinator(
        CHARTER_PATH, tmp_path / "authorization.json", effects,
    )
    assert isinstance(coordinator.production_domain_authority, ProductionSlice7GDomainAuthority)
    assert isinstance(coordinator.process_factory, ProductionSlice7GRunnerAdapter)
    result = coordinator.run()
    assert result.functional_promotion_pass is True
    assert result.timing_all_pass is True
    assert len(effects.cells) == 15 and len(set(effects.cells)) == 15
    assert authorization.campaign_output_root == str(tmp_path / "campaign")
    assert len(tuple((tmp_path / "campaign" / "evidence" / "packages").iterdir())) == 15
    control = tmp_path / ".campaign.slice_7g_control" / "attempt_ledger"
    assert (control / "domain_release_receipt.json").is_file()
    gc.collect()
    assert len(tuple(Path("/proc/self/fd").iterdir())) == descriptors_before


def test_domain_provider_exception_is_stable(tmp_path):
    charter = _charter()
    authorization, _ = _authorization(tmp_path, charter)

    def hostile(_domain):
        raise RuntimeError("caller controlled")

    allocator = Slice7GDomainAllocator(hostile, lambda *args: "a" * 64, lambda *args: "b" * 64)
    with pytest.raises(Slice7GRuntimeError) as raised:
        allocator.allocate(charter, authorization, STAMP)
    assert raised.value.code == "domain_occupancy_provider_failed"


def test_snapshot_rejects_hostile_iterable_before_hook(tmp_path):
    class Hostile:
        invoked = False

        def __iter__(self):
            self.invoked = True
            raise RuntimeError("caller controlled")

    hostile = Hostile()
    with pytest.raises(Slice7GRuntimeError) as raised:
        post_implementation_source_snapshot(tmp_path, hostile)
    assert raised.value.code == "snapshot_members_type"
    assert hostile.invoked is False


def test_malformed_retained_ledger_missing_history_has_stable_error(tmp_path):
    charter = _charter()
    _, initial = _authorization(tmp_path, charter)
    ledger_parent = tmp_path / "ledger"
    ledger_parent.mkdir()
    raw_ledger = json.loads(governance.canonical_slice_7g_attempt_ledger_bytes(initial))
    del raw_ledger["applied_event_identities"]
    record = {
        "schema_version": "ctr-slice-7g-ledger-commit-1",
        "ledger": raw_ledger,
        "ledger_identity": slice_7g_attempt_ledger_identity(initial),
        "event": None,
        "event_identity": None,
    }
    path = ledger_parent / "attempt_ledger.r00000000.json"
    path.write_bytes(json.dumps(record, sort_keys=True, separators=(",", ":")).encode())
    path.chmod(0o444)
    writer = AtomicSlice7GLedgerWriter(ledger_parent)
    event = propose_slice_7g_attempt_event(
        initial, "preflight_failed_before_process_creation", "preflight", STAMP,
    )
    with pytest.raises(Slice7GRuntimeError) as raised:
        writer.commit(initial, event)
    assert raised.value.code == "ledger_commit_record"


def test_production_domain_authority_requires_all_four_raw_observations(tmp_path):
    class Active(_ProductionEffects):
        def active_process_records(self):
            return ({
                "pid": 42,
                "command": b"ros2 node",
                "environment": b"ROS_DOMAIN_ID=100\0",
                "error": None,
            },)

    active = ProductionSlice7GDomainAuthority(tmp_path / "active", Active()).observe(100)
    assert active.active_processes_clear is False and active.collision_free is False

    class Graph(_ProductionEffects):
        def run_graph_observer(self, contract):
            return Slice7GROSGraphObserverExecution(
                4321, 4321, 66, 1, 2, 0, b"/occupied\n", b"",
            )

    graph = ProductionSlice7GDomainAuthority(tmp_path / "graph", Graph()).observe(100)
    assert graph.ros_graph_clear is False and graph.collision_free is False

    class Dds(_ProductionEffects):
        def udp_socket_tables(self):
            header = b"sl local_address rem_address st\n"
            occupied = b"0: 00000000:7E90 00000000:0000 07\n"
            return header + occupied, header

    dds = ProductionSlice7GDomainAuthority(tmp_path / "dds", Dds()).observe(100)
    assert dds.dds_participants_clear is False and dds.collision_free is False

    lease_root = tmp_path / "lease"
    authority = ProductionSlice7GDomainAuthority(lease_root, _ProductionEffects())
    assert authority.acquire(100, "a" * 64, "b" * 64) is not None
    external = ProductionSlice7GDomainAuthority(lease_root, _ProductionEffects()).observe(100)
    assert external.external_ledger_clear is False and external.collision_free is False


def test_production_domain_lease_is_atomic_no_replace(tmp_path):
    first = ProductionSlice7GDomainAuthority(tmp_path / "leases", _ProductionEffects())
    second = ProductionSlice7GDomainAuthority(tmp_path / "leases", _ProductionEffects())
    first_identity = first.acquire(100, "a" * 64, "b" * 64)
    second_identity = second.acquire(100, "a" * 64, "b" * 64)
    assert first_identity is not None
    assert second_identity is None
    assert first.release(100, "c" * 64) is not None
    assert second.acquire(100, "d" * 64, "e" * 64) is not None


def test_production_release_failure_retains_consumed_attempt(tmp_path):
    charter = _charter()
    _authorization(tmp_path, charter)
    coordinator = _assemble_slice_7g_production_coordinator(
        CHARTER_PATH, tmp_path / "authorization.json", _ProductionEffects(),
    )

    def fail_release(*_args):
        raise RuntimeError("release failed")

    coordinator.domain_allocator._release = fail_release
    with pytest.raises(Slice7GRuntimeError) as raised:
        coordinator.run()
    assert raised.value.code == "domain_release_provider_failed"
    ledger_root = tmp_path / ".campaign.slice_7g_control" / "attempt_ledger"
    committed = coordinator.ledger_writer._read_commit(
        ledger_root / "attempt_ledger.r00000002.json",
    )
    assert committed.consumed_campaign_attempts == 1
    assert committed.process_start_committed is True
    assert not (ledger_root / "domain_release_receipt.json").exists()


def test_real_runner_receipt_retains_exact_committed_bindings(monkeypatch, tmp_path):
    cell_root = tmp_path / "campaign" / "cells" / "cell"
    baseline = cell_root / "baseline"
    candidate = cell_root / "candidate"
    baseline.mkdir(parents=True)
    candidate.mkdir()
    bindings = {
        "CTR_SLICE_7G_CHARTER_IDENTITY": "1" * 64,
        "CTR_SLICE_7G_RUNTIME_AUTHORIZATION_IDENTITY": "2" * 64,
        "CTR_SLICE_7G_ATTEMPT_LEDGER_IDENTITY": "3" * 64,
        "CTR_SLICE_7G_ATTEMPT_LEDGER_REVISION": "2",
        "CTR_SLICE_7G_PROCESS_START_EVENT_IDENTITY": "4" * 64,
        "CTR_SLICE_7G_CAMPAIGN_PLAN_IDENTITY": "5" * 64,
        "CTR_SLICE_7G_DOMAIN_LEASE_IDENTITY": "6" * 64,
        "CTR_SLICE_7G_DOMAIN_COMMITTED_BINDING_IDENTITY": "7" * 64,
        "CTR_SLICE_7G_CELL_ID": "centerline.seed_0000000011",
        "CTR_SLICE_7G_CAMPAIGN_ID": "campaign-001",
        "CTR_SLICE_7G_CAMPAIGN_OUTPUT_ROOT": str(tmp_path / "campaign"),
        "CTR_SLICE_7G_CELL_OUTPUT_ROOT": str(cell_root),
        "ROS_DOMAIN_ID": "117",
    }
    for name, value in bindings.items():
        monkeypatch.setenv(name, value)
    argv = [
        "--experiment-group", "campaign-001", "--task", "curved_lumen_navigation",
        "--curved-lumen-type", "circular_arc", "--scenario", "centerline_target",
        "--seed", "11", "--duration", "25.0", "--runtime-mode", "simulation",
        "--output-root", str(cell_root),
    ]
    args = parse_args(argv)
    receipt_path = write_slice_7g_runner_receipt(args, {
        "baseline_dir": str(baseline), "candidate_dir": str(candidate),
    })
    receipt = json.loads(receipt_path.read_bytes())
    assert receipt["argv"] == ["ctr_run_evaluation", *argv]
    assert receipt["attempt_ledger_revision"] == 2
    assert receipt["ros_domain_id"] == 117
    assert receipt["baseline_relative_path"] == "baseline"
    assert receipt["candidate_relative_path"] == "candidate"
    assert receipt_path.stat().st_mode & 0o777 == 0o444


def test_production_console_mapping_targets_internal_assembly():
    signature = inspect.signature(assemble_slice_7g_production_coordinator)
    assert tuple(signature.parameters) == ("charter_path", "authorization_path")
    tree = ast.parse((REPO / "src" / "ctr_evaluation" / "setup.py").read_text(encoding="utf-8"))
    literals = {
        node.value for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }
    assert "ctr_run_slice_7g_campaign = ctr_evaluation.slice_7g_runtime:main" in literals


def test_descriptor_authority_semantics_ignore_substituted_path_read(monkeypatch, tmp_path):
    passing = json.dumps(_summary(collision=0), sort_keys=True, separators=(",", ":")).encode()
    original = Path.read_bytes

    def substituted(path):
        if path.name == "summary.json" and path.parent.name == "candidate":
            return passing
        return original(path)

    monkeypatch.setattr(Path, "read_bytes", substituted)
    _, _, _, _, execution, authority = _adapter_execution(
        tmp_path, _ProductionEffects(collision=1),
    )
    try:
        assert execution.cell_result.collision_sample_count == 1
        retained = json.loads(authority.member_bytes("candidate/summary.json"))
        assert retained["lumen_evaluation"]["physical_safety"]["collision_sample_count"] == 1
    finally:
        authority.close()


def _restore_finalized_modes(root):
    for current, directories, files in os.walk(root, topdown=False):
        for name in files:
            (Path(current) / name).chmod(0o444)
        for name in directories:
            (Path(current) / name).chmod(0o555)
    root.chmod(0o555)


@pytest.mark.parametrize(
    ("mutation", "expected_code"),
    (
        ("same_size_rewrite", "cell_output_changed"),
        ("restored_bytes", "cell_output_changed"),
        ("member_inode", "cell_output_changed"),
        ("directory_inode", "cell_output_changed"),
        ("root_inode", "cell_output_changed"),
        ("addition", "cell_output_changed"),
        ("removal", "cell_output_inventory_changed"),
        ("symlink", "cell_output_symlink"),
        ("hardlink", "cell_output_member_type"),
    ),
)
def test_cell_output_final_barrier_rejects_late_mutation(tmp_path, mutation, expected_code):
    descriptors_before = len(tuple(Path("/proc/self/fd").iterdir()))
    _, _, _, cell, execution, authority = _adapter_execution(tmp_path)
    root = Path(cell.cell_output_path)
    summary = root / "candidate" / "summary.json"
    original = summary.read_bytes()
    if mutation == "same_size_rewrite":
        summary.chmod(0o644)
        summary.write_bytes(bytes([original[0] ^ 1]) + original[1:])
        summary.chmod(0o444)
    elif mutation == "restored_bytes":
        summary.chmod(0o644)
        summary.write_bytes(original)
        summary.chmod(0o444)
    elif mutation in {"member_inode", "symlink", "hardlink"}:
        summary.parent.chmod(0o755)
        summary.unlink()
        if mutation == "member_inode":
            summary.write_bytes(original)
            summary.chmod(0o444)
        elif mutation == "symlink":
            summary.symlink_to(root / "baseline" / "summary.json")
        else:
            os.link(root / "baseline" / "summary.json", summary)
        summary.parent.chmod(0o555)
    elif mutation == "directory_inode":
        root.chmod(0o755)
        old = root / "candidate-old"
        (root / "candidate").rename(old)
        import shutil
        shutil.copytree(old, root / "candidate")
        _restore_finalized_modes(root / "candidate")
        _restore_finalized_modes(old)
        root.chmod(0o555)
    elif mutation == "root_inode":
        old = root.with_name(f"{root.name}.old")
        root.rename(old)
        import shutil
        shutil.copytree(old, root)
        _restore_finalized_modes(root)
    elif mutation == "addition":
        root.chmod(0o755)
        (root / "unexpected.bin").write_bytes(b"unexpected")
        (root / "unexpected.bin").chmod(0o444)
        root.chmod(0o555)
    elif mutation == "removal":
        summary.parent.chmod(0o755)
        summary.unlink()
        summary.parent.chmod(0o555)
    with pytest.raises(Slice7GRuntimeError) as raised:
        authority.final_barrier(execution.output_inventory_payload["output_tree_identity"])
    assert raised.value.code == expected_code
    authority.close()
    gc.collect()
    assert len(tuple(Path("/proc/self/fd").iterdir())) == descriptors_before


def test_cell_output_unchanged_tree_binds_evidence_with_zero_descriptor_delta(tmp_path):
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    charter, plan, committed, _, execution, authority = _adapter_execution(tmp_path)
    writer = Slice7GEvidenceWriter(plan.cells[0].campaign_output_root)
    final, _, _ = writer._write_authenticated_cell_package(
        execution, charter, committed, plan, authority,
    )
    assert final.is_dir()
    gc.collect()
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before


def test_invalid_output_authorization_is_rejected_without_effects(tmp_path):
    governance.SLICE_7G_EVIDENCE_PARENT = str(tmp_path / "trusted")
    propose_slice_7g_attempt_event.__globals__["SLICE_7G_EVIDENCE_PARENT"] = str(tmp_path / "trusted")
    (tmp_path / "trusted").mkdir()
    outside = tmp_path / "outside" / "campaign"
    path = _write_authorization(tmp_path, "outside.json", "campaign-outside", outside)
    with pytest.raises(Slice7GRuntimeError) as raised:
        _load_slice_7g_runtime_authorization_v1_for_test(path, _charter())
    assert raised.value.code == "runtime_authorization_output_root"
    assert not outside.exists()
    assert not (tmp_path / "trusted" / GLOBAL_DOMAIN_LEASE_REGISTRY_NAME).exists()


def test_missing_runner_preflight_creates_no_control_lease_output_or_ledger(tmp_path):
    charter = _charter()
    authorization, _ = _authorization(tmp_path, charter)

    class MissingRunner(_ProductionEffects):
        def which(self, executable):
            return None

    coordinator = _assemble_slice_7g_production_coordinator(
        CHARTER_PATH, tmp_path / "authorization.json", MissingRunner(),
    )
    with pytest.raises(Slice7GRuntimeError) as raised:
        coordinator.run()
    assert raised.value.code == "preflight_runner_missing"
    assert not Path(authorization.campaign_output_root).exists()
    assert not (tmp_path / ".campaign.slice_7g_control").exists()
    assert not (tmp_path / GLOBAL_DOMAIN_LEASE_REGISTRY_NAME).exists()


def test_missing_domain_observer_preflight_creates_no_durable_state(tmp_path):
    charter = _charter()
    authorization, _ = _authorization(tmp_path, charter)

    class MissingObserver(_ProductionEffects):
        def which(self, executable):
            return None if executable == "ros2" else super().which(executable)

    coordinator = _assemble_slice_7g_production_coordinator(
        CHARTER_PATH, tmp_path / "authorization.json", MissingObserver(),
    )
    with pytest.raises(Slice7GRuntimeError) as raised:
        coordinator.run()
    assert raised.value.code == "preflight_domain_observer_missing"
    assert not Path(authorization.campaign_output_root).exists()
    assert not (tmp_path / ".campaign.slice_7g_control").exists()
    assert not (tmp_path / GLOBAL_DOMAIN_LEASE_REGISTRY_NAME).exists()


@pytest.mark.parametrize("provider", ("active_process", "dds_socket"))
def test_unavailable_local_domain_provider_preflight_is_side_effect_free(tmp_path, provider):
    charter = _charter()
    authorization, _ = _authorization(tmp_path, charter)

    class UnavailableProvider(_ProductionEffects):
        def active_process_records(self):
            if provider == "active_process":
                raise RuntimeError("unavailable")
            return super().active_process_records()

        def udp_socket_tables(self):
            if provider == "dds_socket":
                raise RuntimeError("unavailable")
            return super().udp_socket_tables()

    coordinator = _assemble_slice_7g_production_coordinator(
        CHARTER_PATH, tmp_path / "authorization.json", UnavailableProvider(),
    )
    with pytest.raises(Slice7GRuntimeError) as raised:
        coordinator.run()
    assert raised.value.code == "preflight_domain_provider_failed"
    assert not Path(authorization.campaign_output_root).exists()
    assert not (tmp_path / ".campaign.slice_7g_control").exists()
    assert not (tmp_path / GLOBAL_DOMAIN_LEASE_REGISTRY_NAME).exists()


def test_global_registry_serializes_same_domain_across_distinct_output_roots(tmp_path):
    governance.SLICE_7G_EVIDENCE_PARENT = str(tmp_path)
    propose_slice_7g_attempt_event.__globals__["SLICE_7G_EVIDENCE_PARENT"] = str(tmp_path)
    first_path = _write_authorization(tmp_path, "first.json", "campaign-first", tmp_path / "runs" / "first")
    second_path = _write_authorization(tmp_path, "second.json", "campaign-second", tmp_path / "runs" / "second")
    (tmp_path / "runs").mkdir()
    effects = _ProductionEffects()
    first = _assemble_slice_7g_production_coordinator(CHARTER_PATH, first_path, effects)
    second = _assemble_slice_7g_production_coordinator(CHARTER_PATH, second_path, effects)
    first.preflight(_load_slice_7g_runtime_authorization_v1_for_test(first_path, _charter()))
    second.preflight(_load_slice_7g_runtime_authorization_v1_for_test(second_path, _charter()))
    first.production_domain_authority.adopt_prepared_registry(
        first.production_root_authority.prepare_global_registry()
    )
    second.production_domain_authority.adopt_prepared_registry(
        second.production_root_authority.prepare_global_registry()
    )
    first_auth = _load_slice_7g_runtime_authorization_v1_for_test(first_path, _charter())
    second_auth = _load_slice_7g_runtime_authorization_v1_for_test(second_path, _charter())
    first_receipt = first.production_domain_authority.acquire(
        100, first_auth.identity, first_auth.campaign_identity,
    )
    second_receipt = second.production_domain_authority.acquire(
        100, second_auth.identity, second_auth.campaign_identity,
    )
    assert first_receipt is not None and second_receipt is None
    release_identity = first.production_domain_authority.release(100, "f" * 64)
    assert release_identity is not None
    assert second.production_domain_authority.acquire(
        100, second_auth.identity, second_auth.campaign_identity,
    ) is not None
    domain_root = tmp_path / GLOBAL_DOMAIN_LEASE_REGISTRY_NAME / "domain_100"
    assert tuple(domain_root.glob("reservation.*.json"))
    assert tuple(domain_root.glob("release.*.json"))
    first.production_root_authority.close()
    second.production_root_authority.close()


def test_post_commit_domain_change_is_durable_and_prohibits_process(tmp_path):
    charter = _charter()
    _authorization(tmp_path, charter)

    class OccupiedAfterCommit(_ProductionEffects):
        def __init__(self):
            super().__init__()
            self.graph_calls = 0
            self.process_calls = 0

        def run_graph_observer(self, contract):
            self.graph_calls += 1
            occupied = self.graph_calls >= 2
            return Slice7GROSGraphObserverExecution(
                4321, 4321, 66, 1, 2, 0,
                b"/late\n" if occupied else b"", b"",
            )

        def run_cell(self, argv, env, timeout_seconds):
            self.process_calls += 1
            return super().run_cell(argv, env, timeout_seconds)

    effects = OccupiedAfterCommit()
    coordinator = _assemble_slice_7g_production_coordinator(
        CHARTER_PATH, tmp_path / "authorization.json", effects,
    )
    with pytest.raises(Slice7GRuntimeError) as raised:
        coordinator.run()
    assert raised.value.code == "domain_occupancy_changed_after_commit"
    ledger = tmp_path / ".campaign.slice_7g_control" / "attempt_ledger"
    assert (ledger / "attempt_ledger.r00000002.json").is_file()
    assert (ledger / "final_domain_observation.json").is_file()
    assert effects.process_calls == 0


def test_primary_failure_survives_release_and_cleanup_record_failure(tmp_path):
    charter = _charter()
    _authorization(tmp_path, charter)

    class CellFailure(_ProductionEffects):
        def run_cell(self, argv, env, timeout_seconds):
            raise Slice7GRuntimeError("primary_cell_failure", "injected")

    coordinator = _assemble_slice_7g_production_coordinator(
        CHARTER_PATH, tmp_path / "authorization.json", CellFailure(),
    )
    coordinator.domain_allocator._release = lambda *_: (_ for _ in ()).throw(RuntimeError("release failed"))
    coordinator.ledger_writer.commit_cleanup_failure = lambda *_: (_ for _ in ()).throw(RuntimeError("ledger failed"))
    with pytest.raises(Slice7GCoordinatedFailure) as raised:
        coordinator.run()
    assert raised.value.code == "primary_cell_failure"
    assert tuple(item.code for item in raised.value.cleanup_issues) == (
        "domain_release_cleanup_failed", "cleanup_ledger_record_failed",
    )


def test_primary_failure_and_release_failure_are_durably_accounted(tmp_path):
    charter = _charter()
    _authorization(tmp_path, charter)

    class CellFailure(_ProductionEffects):
        def run_cell(self, argv, env, timeout_seconds):
            raise Slice7GRuntimeError("primary_cell_failure", "injected")

    coordinator = _assemble_slice_7g_production_coordinator(
        CHARTER_PATH, tmp_path / "authorization.json", CellFailure(),
    )
    coordinator.domain_allocator._release = lambda *_: (_ for _ in ()).throw(RuntimeError("release failed"))
    with pytest.raises(Slice7GCoordinatedFailure) as raised:
        coordinator.run()
    assert raised.value.code == "primary_cell_failure"
    assert tuple(item.code for item in raised.value.cleanup_issues) == ("domain_release_cleanup_failed",)
    cleanup = tmp_path / ".campaign.slice_7g_control" / "attempt_ledger" / "cleanup_failure.json"
    assert json.loads(cleanup.read_bytes())["primary_code"] == "primary_cell_failure"


def test_forged_execution_record_has_stable_public_error(tmp_path):
    charter, _, _, _, plan, committed = _committed_context(tmp_path)
    Path(plan.cells[0].campaign_output_root).mkdir()
    writer = Slice7GEvidenceWriter(plan.cells[0].campaign_output_root)
    with pytest.raises(Slice7GRuntimeError) as raised:
        writer.write_cell_package(object.__new__(Slice7GCellExecution), charter, committed, plan)
    assert raised.value.code == "execution_record"


def test_execution_record_rejects_subclass_and_cross_field_mismatch(tmp_path):
    charter, _, _, _, plan, committed = _committed_context(tmp_path)
    Path(plan.cells[0].campaign_output_root).mkdir()
    writer = Slice7GEvidenceWriter(plan.cells[0].campaign_output_root)
    valid = _execution(plan.cells[0], plan, committed)

    class Derived(Slice7GCellExecution):
        pass

    with pytest.raises(Slice7GRuntimeError) as subclassed:
        writer.write_cell_package(Derived(**valid.__dict__), charter, committed, plan)
    assert subclassed.value.code == "execution_record"
    invalid = replace(valid, safety_payload={**valid.safety_payload, "collision_sample_count": 1})
    with pytest.raises(Slice7GRuntimeError) as mismatched:
        writer.write_cell_package(invalid, charter, committed, plan)
    assert mismatched.value.code == "execution_record"


def test_binary_process_streams_are_exact_authenticated_inventory_members(tmp_path):
    stdout = b"stdout\xff\x00bytes"
    stderr = b""
    _, _, _, _, execution, authority = _adapter_execution(
        tmp_path, _ProductionEffects(stdout=stdout, stderr=stderr),
    )
    try:
        stdout_record = authority.member_observation(PROCESS_STDOUT_PATH)
        stderr_record = authority.member_observation(PROCESS_STDERR_PATH)
        assert (stdout_record.size, stdout_record.sha256, stdout_record.semantic_bytes) == (
            len(stdout), hashlib.sha256(stdout).hexdigest(), None,
        )
        assert (stderr_record.size, stderr_record.sha256, stderr_record.semantic_bytes) == (
            0, hashlib.sha256(b"").hexdigest(), None,
        )
        receipt = json.loads(authority.member_bytes(PROCESS_OUTPUT_RECEIPT_PATH))
        assert receipt["streams"] == [
            {"path": PROCESS_STDOUT_PATH, "size": len(stdout), "sha256": hashlib.sha256(stdout).hexdigest()},
            {"path": PROCESS_STDERR_PATH, "size": 0, "sha256": hashlib.sha256(b"").hexdigest()},
        ]
        assert authority.member_observation("baseline/summary.json").semantic_bytes is None
        assert authority.member_observation("candidate/summary.json").semantic_bytes is not None
        assert execution.output_inventory_payload["regular_file_count"] >= 3
    finally:
        authority.close()


def test_preexisting_process_output_artifact_fails_without_overwrite(tmp_path):
    class StaleLog(_ProductionEffects):
        def run_cell(self, argv, env, timeout_seconds):
            observation = super().run_cell(argv, env, timeout_seconds)
            stale = Path(env["CTR_SLICE_7G_CELL_OUTPUT_ROOT"]) / PROCESS_STDOUT_PATH
            stale.write_bytes(b"stale")
            return observation

    with pytest.raises(Slice7GRuntimeError) as raised:
        _adapter_execution(tmp_path, StaleLog())
    assert raised.value.code == "process_output_exists"


@pytest.mark.parametrize("field", ("size", "sha256"))
def test_process_output_receipt_size_or_digest_mismatch_is_rejected(tmp_path, field):
    original = runtime_module._write_process_output_artifacts

    def corrupt(root, observation):
        original(root, observation)
        receipt_path = Path(root) / PROCESS_OUTPUT_RECEIPT_PATH
        data = json.loads(receipt_path.read_bytes())
        data["streams"][0][field] = (
            data["streams"][0][field] + 1 if field == "size" else "0" * 64
        )
        receipt_path.chmod(0o644)
        receipt_path.write_bytes(json.dumps(data, sort_keys=True, separators=(",", ":")).encode())
        receipt_path.chmod(0o444)

    with mock.patch.object(runtime_module, "_write_process_output_artifacts", corrupt):
        with pytest.raises(Slice7GRuntimeError) as raised:
            _adapter_execution(tmp_path)
    assert raised.value.code == "process_output_receipt_binding"


def test_failed_process_retains_exact_logs_in_cell_execution(tmp_path):
    _, _, _, _, execution, authority = _adapter_execution(
        tmp_path, _ProductionEffects(stdout=b"failure-out", stderr=b"failure-err", returncode=7),
    )
    try:
        assert execution.cell_result.process_exit_status == 7
        assert authority.member_observation(PROCESS_STDOUT_PATH).sha256 == hashlib.sha256(
            b"failure-out"
        ).hexdigest()
        assert authority.member_observation(PROCESS_STDERR_PATH).sha256 == hashlib.sha256(
            b"failure-err"
        ).hexdigest()
        assert authority.member_observation(PROCESS_STDOUT_PATH).semantic_bytes is None
        assert authority.member_observation(PROCESS_STDERR_PATH).semantic_bytes is None
    finally:
        authority.close()


@pytest.mark.parametrize("attack", ("sibling_prefix", "traversal", "symlink_parent"))
def test_output_authorization_confinement_attacks_have_no_effects(tmp_path, attack):
    trusted = tmp_path / "trusted"
    trusted.mkdir()
    governance.SLICE_7G_EVIDENCE_PARENT = str(trusted)
    propose_slice_7g_attempt_event.__globals__["SLICE_7G_EVIDENCE_PARENT"] = str(trusted)
    if attack == "sibling_prefix":
        output = tmp_path / "trusted-escape" / "campaign"
    elif attack == "traversal":
        output = f"{trusted}/runs/../campaign"
    else:
        external = tmp_path / "external"
        external.mkdir()
        (trusted / "linked").symlink_to(external, target_is_directory=True)
        output = trusted / "linked" / "campaign"
    path = _write_authorization(tmp_path, f"{attack}.json", f"campaign-{attack}", output)
    with pytest.raises(Slice7GRuntimeError) as raised:
        _assemble_slice_7g_production_coordinator(CHARTER_PATH, path, _ProductionEffects())
    assert raised.value.code in {"runtime_authorization_output_root", "absolute_path", "campaign_output_parent"}
    assert not Path(output).exists()
    assert not (trusted / GLOBAL_DOMAIN_LEASE_REGISTRY_NAME).exists()


def test_root_authority_rejects_output_parent_replacement_before_effects(tmp_path):
    governance.SLICE_7G_EVIDENCE_PARENT = str(tmp_path)
    propose_slice_7g_attempt_event.__globals__["SLICE_7G_EVIDENCE_PARENT"] = str(tmp_path)
    runs = tmp_path / "runs"
    runs.mkdir()
    auth_path = _write_authorization(
        tmp_path, "replacement.json", "campaign-replacement", runs / "campaign",
    )
    coordinator = _assemble_slice_7g_production_coordinator(
        CHARTER_PATH, auth_path, _ProductionEffects(),
    )
    runs.rename(tmp_path / "runs.old")
    runs.mkdir()
    with pytest.raises(Slice7GRuntimeError) as raised:
        coordinator.run()
    assert raised.value.code == "campaign_output_parent_changed"
    assert not (tmp_path / GLOBAL_DOMAIN_LEASE_REGISTRY_NAME).exists()
    assert not (runs / ".campaign.slice_7g_control").exists()


def test_global_registry_malformed_history_fails_closed(tmp_path):
    authority = ProductionSlice7GDomainAuthority(tmp_path / "leases", _ProductionEffects())
    domain_root = tmp_path / "leases" / "domain_100"
    domain_root.mkdir(mode=0o700)
    malformed = domain_root / f"release.{'a' * 64}.json"
    malformed.write_bytes(b"{}")
    malformed.chmod(0o444)
    observation = authority.observe(100)
    assert observation.external_ledger_clear is False
    assert observation.collision_free is False


def test_output_authority_rejects_unexpected_runner_file(tmp_path):
    class ExtraArtifact(_ProductionEffects):
        def run_cell(self, argv, env, timeout_seconds):
            observation = super().run_cell(argv, env, timeout_seconds)
            (Path(env["CTR_SLICE_7G_CELL_OUTPUT_ROOT"]) / "unbound.json").write_bytes(b"{}")
            return observation

    with pytest.raises(Slice7GRuntimeError) as raised:
        _adapter_execution(tmp_path, ExtraArtifact())
    assert raised.value.code == "cell_output_unexpected_file"


def test_process_log_replacement_is_rejected_by_output_final_barrier(tmp_path):
    _, _, _, _, execution, authority = _adapter_execution(
        tmp_path, _ProductionEffects(stdout=b"original-log"),
    )
    log = Path(authority.root_path) / PROCESS_STDOUT_PATH
    log.parent.chmod(0o755)
    log.chmod(0o644)
    log.unlink()
    log.write_bytes(b"replacement!")
    log.chmod(0o444)
    log.parent.chmod(0o555)
    with pytest.raises(Slice7GRuntimeError) as raised:
        authority.final_barrier(execution.output_inventory_payload["output_tree_identity"])
    assert raised.value.code == "cell_output_changed"
    authority.close()


def test_primary_cell_failure_retains_successful_release_receipt(tmp_path):
    charter = _charter()
    _authorization(tmp_path, charter)

    class CellFailure(_ProductionEffects):
        def run_cell(self, argv, env, timeout_seconds):
            raise Slice7GRuntimeError("primary_cell_failure", "injected")

    coordinator = _assemble_slice_7g_production_coordinator(
        CHARTER_PATH, tmp_path / "authorization.json", CellFailure(),
    )
    with pytest.raises(Slice7GRuntimeError) as raised:
        coordinator.run()
    assert raised.value.code == "primary_cell_failure"
    ledger = tmp_path / ".campaign.slice_7g_control" / "attempt_ledger"
    assert (ledger / "domain_release_receipt.json").is_file()
    history = tmp_path / GLOBAL_DOMAIN_LEASE_REGISTRY_NAME / "domain_100"
    assert tuple(history.glob("release.*.json"))
    assert tuple(history.glob("reservation.*.json"))


def test_production_order_records_preflight_lease_commit_recheck_then_process(tmp_path):
    charter = _charter()
    _authorization(tmp_path, charter)
    coordinator = _assemble_slice_7g_production_coordinator(
        CHARTER_PATH, tmp_path / "authorization.json", _ProductionEffects(),
    )
    events = []

    def wrap(target, name, event):
        original = getattr(target, name)

        def observed(*args, **kwargs):
            events.append(event)
            return original(*args, **kwargs)

        setattr(target, name, observed)

    # Special methods resolve on the class, so wrap the explicit coordinator field.
    original_preflight = coordinator.preflight
    coordinator.preflight = lambda authorization: (events.append("preflight"), original_preflight(authorization))[1]
    wrap(coordinator.production_root_authority, "prepare_global_registry", "registry")
    original_allocate = coordinator.domain_allocator.allocate

    def allocate(*args, **kwargs):
        value = original_allocate(*args, **kwargs)
        events.append("lease")
        return value

    coordinator.domain_allocator.allocate = allocate
    original_output = coordinator.output_allocator
    coordinator.output_allocator = lambda authorization: (events.append("output"), original_output(authorization))[1]
    wrap(coordinator.ledger_writer, "initialize", "ledger_initialize")
    original_commit = coordinator.ledger_writer.commit

    def commit(*args, **kwargs):
        event = args[1]
        events.append("attempt_commit" if event.event_kind == "process_start_commit" else "allocation_commit")
        return original_commit(*args, **kwargs)

    coordinator.ledger_writer.commit = commit
    original_observe = coordinator.production_domain_authority.observe
    observation_count = {"value": 0}

    def observe(domain):
        observation_count["value"] += 1
        events.append("initial_observation" if observation_count["value"] == 1 else "final_observation")
        return original_observe(domain)

    coordinator.production_domain_authority.observe = observe
    coordinator.domain_allocator._occupancy = observe
    original_run = coordinator.process_factory.effects.run_cell
    coordinator.process_factory.effects.run_cell = lambda *args, **kwargs: (
        events.append("process"), original_run(*args, **kwargs)
    )[1]
    coordinator.run()
    assert events.index("preflight") < events.index("registry")
    assert events.index("initial_observation") < events.index("lease") < events.index("output")
    assert events.index("output") < events.index("ledger_initialize") < events.index("attempt_commit")
    assert events.index("attempt_commit") < events.index("final_observation") < events.index("process")


def _sealed_nested_output(root, depth):
    root.mkdir()
    directories = []
    current = root
    for index in range(depth):
        current = current / f"d{index}"
        current.mkdir()
        directories.append(current)
    for directory in directories:
        directory.chmod(0o555)
    root.chmod(0o555)
    return directories


def _make_nested_output_writable(root, directories):
    root.chmod(0o755)
    for directory in directories:
        directory.chmod(0o755)


def test_cell_output_limits_are_exact_immutable_and_cross_validated():
    limits = _CELL_OUTPUT_LIMITS
    assert limits == _CellOutputLimits(
        16, 2_048, 67_108_864, 8_388_608, 268_435_456, 33_554_432, 1_048_576,
    )
    invalid = (
        (True, 2_048, 67_108_864, 8_388_608, 268_435_456, 33_554_432, 1_048_576),
        (16, 2_048, 8_388_608, 67_108_864, 268_435_456, 33_554_432, 1_048_576),
        (16, 2_048, 67_108_864, 8_388_608, 1, 33_554_432, 1_048_576),
        (16, 2_048, 67_108_864, 8_388_608, 268_435_456, 1, 1_048_576),
        (16, 2_048, 67_108_864, 8_388_608, 268_435_456, 33_554_432, 9_000_000),
    )
    for values in invalid:
        with pytest.raises(Slice7GRuntimeError) as raised:
            _CellOutputLimits(*values)
        assert raised.value.code == "cell_output_limits"


def test_iterative_cell_output_depth_16_is_accepted(tmp_path):
    root = tmp_path / "depth16"
    directories = _sealed_nested_output(root, 16)
    authority = None
    try:
        authority = _CellOutputAuthority(root)
        assert len(authority._directories) == 16
    finally:
        if authority is not None:
            authority.close()
        _make_nested_output_writable(root, directories)


def test_iterative_cell_output_depth_17_has_stable_limit(tmp_path):
    root = tmp_path / "depth17"
    directories = _sealed_nested_output(root, 17)
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    try:
        with pytest.raises(Slice7GRuntimeError) as raised:
            _CellOutputAuthority(root)
        assert raised.value.code == "cell_output_depth_limit"
        gc.collect()
        assert len(tuple(Path("/proc/self/fd").iterdir())) == before
    finally:
        _make_nested_output_writable(root, directories)


def test_deep_tree_and_lowered_recursion_limit_never_raise_recursion_error(tmp_path):
    root = tmp_path / "depth180"
    directories = _sealed_nested_output(root, 180)
    old_limit = sys.getrecursionlimit()
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    unraisable = []
    old_hook = sys.unraisablehook
    sys.unraisablehook = lambda value: unraisable.append(value)
    try:
        sys.setrecursionlimit(80)
        with pytest.raises(Slice7GRuntimeError) as raised:
            _CellOutputAuthority(root)
        assert raised.value.code == "cell_output_depth_limit"
        assert not isinstance(raised.value.__cause__, RecursionError)
    finally:
        sys.setrecursionlimit(old_limit)
        sys.unraisablehook = old_hook
        _make_nested_output_writable(root, directories)
    gc.collect()
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before
    assert unraisable == []


def test_real_iterative_traversal_accepts_2048_and_rejects_2049_members(tmp_path):
    root = tmp_path / "members"
    root.mkdir()
    for index in range(_CELL_OUTPUT_LIMITS.maximum_members):
        member = root / f"m{index:04d}"
        member.touch()
        member.chmod(0o444)
    root.chmod(0o555)
    authority = _CellOutputAuthority(root)
    try:
        assert len(authority.members) == 2_048
    finally:
        authority.close()
    root.chmod(0o755)
    extra = root / "overflow"
    extra.touch()
    extra.chmod(0o444)
    root.chmod(0o555)
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    with pytest.raises(Slice7GRuntimeError) as raised:
        _CellOutputAuthority(root)
    assert raised.value.code == "cell_output_member_limit"
    gc.collect()
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before
    root.chmod(0o755)


def test_directory_members_participate_in_the_same_member_budget():
    accounting = _CellOutputAccounting(member_count=2_047)
    exact = accounting.add_directory(1)
    assert exact.member_count == 2_048
    with pytest.raises(Slice7GRuntimeError) as raised:
        exact.add_directory(1)
    assert raised.value.code == "cell_output_member_limit"


def test_file_and_semantic_size_boundaries_are_exact():
    base = _CellOutputAccounting()
    exact_file = base.add_file(
        1, 67_108_864, semantic=False, cache_semantic=False,
    )
    assert exact_file.total_file_bytes == 67_108_864
    with pytest.raises(Slice7GRuntimeError) as file_error:
        base.add_file(1, 67_108_865, semantic=False, cache_semantic=False)
    assert file_error.value.code == "cell_output_file_size_limit"
    exact_semantic = base.add_file(
        1, 8_388_608, semantic=True, cache_semantic=True,
    )
    assert exact_semantic.semantic_cache_bytes == 8_388_608
    with pytest.raises(Slice7GRuntimeError) as semantic_error:
        base.add_file(1, 8_388_609, semantic=True, cache_semantic=True)
    assert semantic_error.value.code == "cell_output_semantic_size_limit"


def test_aggregate_and_semantic_cache_boundaries_use_checked_accounting():
    aggregate = _CellOutputAccounting(member_count=4, total_file_bytes=268_435_455)
    assert aggregate.add_file(
        1, 1, semantic=False, cache_semantic=False,
    ).total_file_bytes == 268_435_456
    full = _CellOutputAccounting(member_count=4, total_file_bytes=268_435_456)
    with pytest.raises(Slice7GRuntimeError) as aggregate_error:
        full.add_file(1, 1, semantic=False, cache_semantic=False)
    assert aggregate_error.value.code == "cell_output_total_size_limit"
    semantic = _CellOutputAccounting(
        member_count=4, total_file_bytes=40_000_000, semantic_cache_bytes=25_165_824,
    )
    assert semantic.add_semantic_cache(8_388_608).semantic_cache_bytes == 33_554_432
    full_cache = _CellOutputAccounting(
        member_count=4, total_file_bytes=40_000_000, semantic_cache_bytes=33_554_432,
    )
    with pytest.raises(Slice7GRuntimeError) as cache_error:
        full_cache.add_semantic_cache(1)
    assert cache_error.value.code == "cell_output_semantic_cache_limit"
    for invalid in (-1, True, 10**200):
        with pytest.raises(Slice7GRuntimeError):
            full.add_file(1, invalid, semantic=False, cache_semantic=False)


def test_sparse_oversized_file_is_rejected_before_any_content_read(monkeypatch, tmp_path):
    root = tmp_path / "oversized"
    root.mkdir()
    member = root / "large.bin"
    with member.open("wb") as stream:
        stream.truncate(_CELL_OUTPUT_LIMITS.maximum_file_bytes + 1)
    member.chmod(0o444)
    root.chmod(0o555)
    reads = []
    original = runtime_module.os.read

    def observed(descriptor, size):
        reads.append((descriptor, size))
        return original(descriptor, size)

    monkeypatch.setattr(runtime_module.os, "read", observed)
    with pytest.raises(Slice7GRuntimeError) as raised:
        _CellOutputAuthority(root)
    assert raised.value.code == "cell_output_file_size_limit"
    assert reads == []
    root.chmod(0o755)


def test_sparse_oversized_semantic_file_is_rejected_before_allocation(monkeypatch, tmp_path):
    root = tmp_path / "semantic-oversized"
    root.mkdir()
    member = root / RUNNER_RECEIPT_PATH
    with member.open("wb") as stream:
        stream.truncate(_CELL_OUTPUT_LIMITS.maximum_semantic_file_bytes + 1)
    member.chmod(0o444)
    root.chmod(0o555)
    reads = []
    monkeypatch.setattr(runtime_module.os, "read", lambda *args: reads.append(args) or b"")
    with pytest.raises(Slice7GRuntimeError) as raised:
        _CellOutputAuthority(root)
    assert raised.value.code == "cell_output_semantic_size_limit"
    assert reads == []
    root.chmod(0o755)


def test_stream_hash_is_chunk_bounded_and_matches_independent_sha256(monkeypatch, tmp_path):
    raw = (b"0123456789abcdef" * 170_000) + b"tail"
    path = tmp_path / "stream.bin"
    path.write_bytes(raw)
    descriptor = os.open(path, os.O_RDONLY)
    requests = []
    original = runtime_module.os.read

    def observed(fd, size):
        requests.append(size)
        return original(fd, size)

    monkeypatch.setattr(runtime_module.os, "read", observed)
    try:
        digest, retained = _stream_cell_output_descriptor(
            descriptor, len(raw), capture=False, path="stream.bin",
        )
    finally:
        os.close(descriptor)
    assert digest == hashlib.sha256(raw).hexdigest()
    assert retained is None
    assert requests and max(requests) <= 1_048_576
    assert len(requests) > 2


def test_nonsemantic_bytes_are_never_retained_and_do_not_scale_authority_memory(tmp_path):
    root = tmp_path / "nonsemantic"
    root.mkdir()
    member = root / "retained.bin"
    member.write_bytes(b"x" * (4 * 1024 * 1024))
    member.chmod(0o444)
    root.chmod(0o555)
    authority = _CellOutputAuthority(root)
    try:
        record = authority.member_observation("retained.bin")
        assert record.size == 4 * 1024 * 1024
        assert record.semantic_bytes is None
        assert not any(item.semantic_bytes is not None for item in authority.members)
    finally:
        authority.close()
        root.chmod(0o755)


def test_semantic_cache_is_detached_immutable_and_exactly_authenticated(tmp_path):
    raw = b'{"schema_version":"test"}'
    root = tmp_path / "semantic"
    root.mkdir()
    member = root / RUNNER_RECEIPT_PATH
    member.write_bytes(raw)
    member.chmod(0o444)
    root.chmod(0o555)
    authority = _CellOutputAuthority(root)
    try:
        cached = authority.member_bytes(RUNNER_RECEIPT_PATH)
        assert cached == raw
        assert authority.member_observation(RUNNER_RECEIPT_PATH).sha256 == hashlib.sha256(raw).hexdigest()
        with pytest.raises(TypeError):
            memoryview(cached)[0] = 0
        assert authority.member_bytes(RUNNER_RECEIPT_PATH) == raw
    finally:
        authority.close()
        root.chmod(0o755)


def test_final_barrier_streams_without_a_second_semantic_byte_tree(monkeypatch, tmp_path):
    root = tmp_path / "barrier-stream"
    root.mkdir()
    for name in (RUNNER_RECEIPT_PATH, PROCESS_OUTPUT_RECEIPT_PATH):
        member = root / name
        member.write_bytes(b"{}")
        member.chmod(0o444)
    root.chmod(0o555)
    captures = []
    original = runtime_module._stream_cell_output_descriptor

    def observed(*args, **kwargs):
        captures.append(kwargs["capture"])
        return original(*args, **kwargs)

    monkeypatch.setattr(runtime_module, "_stream_cell_output_descriptor", observed)
    authority = _CellOutputAuthority(root)
    try:
        assert captures == [True, True]
        authority.final_barrier(authority.inventory_identity)
        assert captures == [True, True, False, False]
    finally:
        authority.close()
        root.chmod(0o755)


def test_traversal_and_stream_provider_failures_have_stable_codes(monkeypatch, tmp_path):
    root = tmp_path / "provider-errors"
    root.mkdir()
    member = root / "member.bin"
    member.write_bytes(b"content")
    member.chmod(0o444)
    root.chmod(0o555)
    original_scandir = runtime_module.os.scandir
    monkeypatch.setattr(
        runtime_module.os, "scandir", lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("hostile")),
    )
    with pytest.raises(Slice7GRuntimeError) as traversal:
        _CellOutputAuthority(root)
    assert traversal.value.code == "cell_output_traversal_failed"
    monkeypatch.setattr(runtime_module.os, "scandir", original_scandir)
    monkeypatch.setattr(
        runtime_module.os, "read", lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("hostile")),
    )
    with pytest.raises(Slice7GRuntimeError) as stream:
        _CellOutputAuthority(root)
    assert stream.value.code == "cell_output_stream_read_failed"
    monkeypatch.setattr(
        runtime_module.os, "read", lambda *_args, **_kwargs: (_ for _ in ()).throw(MemoryError("hostile")),
    )
    with pytest.raises(Slice7GRuntimeError) as memory:
        _CellOutputAuthority(root)
    assert memory.value.code == "cell_output_stream_read_failed"
    root.chmod(0o755)


def test_production_adapter_resource_failure_is_stable_consumed_and_released(tmp_path):
    charter = _charter()
    _authorization(tmp_path, charter)

    class OversizedOutput(_ProductionEffects):
        def run_cell(self, argv, env, timeout_seconds):
            observation = super().run_cell(argv, env, timeout_seconds)
            log_root = Path(env["CTR_SLICE_7G_CELL_OUTPUT_ROOT"]) / "ros_log"
            log_root.mkdir()
            with (log_root / "oversized.bin").open("wb") as stream:
                stream.truncate(_CELL_OUTPUT_LIMITS.maximum_file_bytes + 1)
            return observation

    effects = OversizedOutput()
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    coordinator = _assemble_slice_7g_production_coordinator(
        CHARTER_PATH, tmp_path / "authorization.json", effects,
    )
    with pytest.raises(Slice7GRuntimeError) as raised:
        coordinator.run()
    assert raised.value.code == "cell_output_file_size_limit"
    assert len(effects.cells) == 1
    ledger_root = tmp_path / ".campaign.slice_7g_control" / "attempt_ledger"
    committed = coordinator.ledger_writer._read_commit(ledger_root / "attempt_ledger.r00000002.json")
    assert committed.consumed_campaign_attempts == 1 and committed.retry_count == 0
    assert (ledger_root / "domain_release_receipt.json").is_file()
    gc.collect()
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before


def test_production_adapter_deep_output_never_leaks_recursion_error(tmp_path):
    charter = _charter()
    _authorization(tmp_path, charter)

    class DeepOutput(_ProductionEffects):
        def run_cell(self, argv, env, timeout_seconds):
            observation = super().run_cell(argv, env, timeout_seconds)
            current = Path(env["CTR_SLICE_7G_CELL_OUTPUT_ROOT"]) / "ros_log"
            current.mkdir()
            for index in range(180):
                current = current / f"d{index}"
                current.mkdir()
            return observation

    effects = DeepOutput()
    coordinator = _assemble_slice_7g_production_coordinator(
        CHARTER_PATH, tmp_path / "authorization.json", effects,
    )
    old_limit = sys.getrecursionlimit()
    try:
        sys.setrecursionlimit(80)
        with pytest.raises(Slice7GRuntimeError) as raised:
            coordinator.run()
    finally:
        sys.setrecursionlimit(old_limit)
    assert raised.value.code == "cell_output_depth_limit"
    assert not isinstance(raised.value.__cause__, RecursionError)
    assert len(effects.cells) == 1
    ledger_root = tmp_path / ".campaign.slice_7g_control" / "attempt_ledger"
    committed = coordinator.ledger_writer._read_commit(ledger_root / "attempt_ledger.r00000002.json")
    assert committed.consumed_campaign_attempts == 1
    assert (ledger_root / "domain_release_receipt.json").is_file()
