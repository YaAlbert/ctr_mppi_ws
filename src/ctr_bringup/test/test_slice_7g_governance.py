from __future__ import annotations

import ast
from collections.abc import Mapping
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import replace
import gc
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile

import pytest
import ctr_bringup.slice_7g_governance as governance

from ctr_bringup.slice_7g_governance import (
    ATTEMPT_EVENT_SCHEMA_VERSION,
    ATTEMPT_LEDGER_SCHEMA_VERSION,
    CAMPAIGN_PLAN_SCHEMA_VERSION,
    CAMPAIGN_RESULT_SCHEMA_VERSION,
    CAMPAIGN_EVIDENCE_PACKAGE_RECORD_SCHEMA_VERSION,
    CAMPAIGN_EVIDENCE_SEAL_PATH,
    CAMPAIGN_EVIDENCE_SEAL_SCHEMA_VERSION,
    CELL_EVIDENCE_ENVELOPE_SCHEMA_VERSION,
    CELL_EVIDENCE_MEMBER_SCHEMA_VERSION,
    CELL_EVIDENCE_PROJECTION_SCHEMA_VERSION,
    CELL_RESULT_SCHEMA_VERSION,
    CHARTER_IDENTITY_ALGORITHM,
    MANDATORY_EVIDENCE_ROLE_PATHS,
    Slice7GAttemptBudget,
    Slice7GAttemptEvent,
    Slice7GAttemptLedger,
    Slice7GCampaignCell,
    Slice7GCampaignEvidencePackage,
    Slice7GCampaignEvidenceSeal,
    Slice7GCampaignPlan,
    Slice7GCampaignResult,
    Slice7GCellResult,
    Slice7GCharter,
    Slice7GDomainPolicy,
    Slice7GGovernanceError,
    Slice7GMetric,
    Slice7GScenario,
    authenticate_slice_7g_cell_evidence_package,
    canonical_slice_7g_attempt_event_bytes,
    canonical_slice_7g_attempt_ledger_bytes,
    canonical_slice_7g_campaign_plan_bytes,
    canonical_slice_7g_campaign_evidence_seal_bytes,
    canonical_slice_7g_campaign_result_bytes,
    canonical_slice_7g_cell_result_bytes,
    canonical_slice_7g_charter_bytes,
    create_slice_7g_initial_attempt_ledger,
    generate_slice_7g_campaign_plan,
    load_slice_7g_charter,
    propose_slice_7g_attempt_event,
    reconcile_slice_7g_campaign_results,
    slice_7g_attempt_event_identity,
    slice_7g_attempt_ledger_identity,
    slice_7g_campaign_plan_identity,
    slice_7g_campaign_evidence_snapshot_identity,
    slice_7g_campaign_result_identity,
    slice_7g_cell_result_identity,
    slice_7g_charter_identity,
    slice_7g_metric_profile_identity,
    validate_slice_7g_attempt_budget,
    validate_slice_7g_attempt_transition,
    validate_slice_7g_charter,
    validate_slice_7g_campaign_plan,
    validate_slice_7g_campaign_evidence_seal,
    validate_slice_7g_domain_policy,
    validate_slice_7g_metric,
    validate_slice_7g_scenario,
    verify_authoring_source_snapshot,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
CHARTER_PATH = REPOSITORY_ROOT / "config" / "slice_7g_simulation_charter.json"
MODULE_PATH = REPOSITORY_ROOT / "src" / "ctr_bringup" / "ctr_bringup" / "slice_7g_governance.py"
DOC_PATH = REPOSITORY_ROOT / "docs" / "19_slice_7g_simulation_promotion_charter.md"
IDENTITY_DOMAIN = b"ctr-slice-7g-charter-canonical-7\0"
EVIDENCE_PROJECTION_DOMAIN = b"ctr-slice-7g-cell-evidence-projection-canonical-1\0"
RUNTIME_AUTHORIZATION_IDENTITY = hashlib.sha256(b"separate-runtime-authorization").hexdigest()


def charter_data() -> dict:
    return json.loads(CHARTER_PATH.read_bytes())


def metric(data: dict, name: str) -> dict:
    return next(item for item in data["acceptance_contract"]["metrics"] if item["name"] == name)


def assert_code(code: str, callable_, *args, **kwargs) -> Slice7GGovernanceError:
    with pytest.raises(Slice7GGovernanceError) as captured:
        callable_(*args, **kwargs)
    assert captured.value.code == code
    return captured.value


def allocated_ledger(charter=None, output_root=None):
    charter = charter or load_slice_7g_charter(CHARTER_PATH)
    output_root = output_root or "/home/ankid/ctr_mppi_evidence/slice_7g/campaign-001"
    initial = create_slice_7g_initial_attempt_ledger(charter, "campaign-001")
    event = propose_slice_7g_attempt_event(
        initial,
        "domain_and_output_allocated",
        "allocate-001",
        "2026-08-19T00:00:00Z",
        domain_id=145,
        output_root=output_root,
        runtime_authorization_identity=RUNTIME_AUTHORIZATION_IDENTITY,
    )
    return validate_slice_7g_attempt_transition(initial, event)


def plan_and_ledger(output_root=None):
    charter = load_slice_7g_charter(CHARTER_PATH)
    ledger = allocated_ledger(charter, output_root)
    return charter, ledger, generate_slice_7g_campaign_plan(charter, ledger)


def process_start_proposal(ledger, event_id="start-001", timestamp="2026-08-19T00:00:01Z"):
    charter = load_slice_7g_charter(CHARTER_PATH)
    plan = generate_slice_7g_campaign_plan(charter, ledger)
    event = propose_slice_7g_attempt_event(
        ledger, "process_start_commit", event_id, timestamp, campaign_plan=plan
    )
    return plan, event


def committed_context(output_root=None):
    charter, allocated, plan = plan_and_ledger(output_root)
    event = propose_slice_7g_attempt_event(
        allocated, "process_start_commit", "start-001", "2026-08-19T00:00:01Z", campaign_plan=plan
    )
    committed = validate_slice_7g_attempt_transition(allocated, event, campaign_plan=plan)
    return charter, committed, plan


def passing_result(cell, plan, committed):
    return Slice7GCellResult(
        schema_version=CELL_RESULT_SCHEMA_VERSION,
        cell_id=cell.cell_id,
        charter_logical_identity=cell.charter_logical_identity,
        campaign_identity=cell.campaign_identity,
        campaign_plan_identity=slice_7g_campaign_plan_identity(plan),
        attempt_ledger_identity=slice_7g_attempt_ledger_identity(committed),
        attempt_ledger_revision=committed.revision,
        process_start_event_identity=committed.last_event_identity,
        runtime_authorization_identity=committed.runtime_authorization_identity,
        metric_profile_identity=cell.metric_profile_identity,
        scenario_id=cell.scenario_id,
        source_scenario_id=cell.source_scenario_id,
        seed=cell.seed,
        duration_seconds=25.0,
        runtime_mode="simulation",
        ros_domain_id=cell.ros_domain_id,
        campaign_output_root=cell.campaign_output_root,
        cell_output_path=cell.cell_output_path,
        argv=cell.argv,
        process_exit_status=0,
        readiness_success=True,
        stable_sample_count=10,
        stable_interval_seconds=0.5,
        q_variation=5.0e-5,
        tip_variation_m=5.0e-5,
        valid_aligned_sample_count=20,
        invalid_sample_count=0,
        invalid_sample_percentage=0.0,
        steady_state_error_m=0.003,
        final_goal_error_m=0.003,
        goal_hold_duration_seconds=0.5,
        minimum_physical_wall_clearance_m=0.0,
        minimum_safety_margin_wall_clearance_m=0.002,
        collision_sample_count=0,
        safety_fault_count=0,
        nonfinite_value_count=0,
        missing_required_topic_count=0,
        missing_required_result_file_count=0,
        saturation_percentage=1.0,
        deadline_overrun_percentage=5.0,
        timing_pass=True,
        non_real_time_label=False,
    )


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode()


def cell_result_data(result):
    data = {name: getattr(result, name) for name in result.__dataclass_fields__}
    data["argv"] = list(data["argv"])
    return data


def write_evidence_package(parent, cell, plan, committed, *, result_changes=None):
    result = passing_result(cell, plan, committed)
    if result_changes:
        result = replace(result, **result_changes)
    bindings = {
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
    payloads = {
        "invocation_process_start_receipt": {"argv": list(result.argv), "process_exit_status": result.process_exit_status},
        "runtime_authorization_binding": {"runtime_authorization_identity": result.runtime_authorization_identity},
        "readiness_trace": {
            "readiness_success": result.readiness_success, "stable_sample_count": result.stable_sample_count,
            "stable_interval_seconds": result.stable_interval_seconds, "q_variation": result.q_variation,
            "tip_variation_m": result.tip_variation_m,
        },
        "safety_trace": {
            "minimum_physical_wall_clearance_m": result.minimum_physical_wall_clearance_m,
            "minimum_safety_margin_wall_clearance_m": result.minimum_safety_margin_wall_clearance_m,
            "collision_sample_count": result.collision_sample_count, "safety_fault_count": result.safety_fault_count,
            "nonfinite_value_count": result.nonfinite_value_count,
        },
        "tactile_trace": {
            "valid_aligned_sample_count": result.valid_aligned_sample_count,
            "invalid_sample_count": result.invalid_sample_count,
            "invalid_sample_percentage": result.invalid_sample_percentage,
            "saturation_percentage": result.saturation_percentage,
            "missing_required_topic_count": result.missing_required_topic_count,
        },
        "output_inventory_receipt": {
            "missing_required_result_file_count": result.missing_required_result_file_count,
            "output_tree_identity": hashlib.sha256(("output:" + cell.cell_id).encode()).hexdigest(),
            "regular_file_count": 7, "regular_file_bytes": 4096,
        },
    }
    package = parent / cell.cell_id
    package.mkdir()
    members = []
    for role, relative in MANDATORY_EVIDENCE_ROLE_PATHS.items():
        if role == "cell_result":
            raw = canonical(cell_result_data(result))
        else:
            raw = canonical({
                "schema_version": CELL_EVIDENCE_MEMBER_SCHEMA_VERSION,
                "role": role,
                "bindings": bindings,
                "payload": payloads[role],
            })
        path = package / relative
        path.write_bytes(raw)
        path.chmod(0o444)
        members.append({
            "role": role, "path": relative, "size": len(raw),
            "sha256": hashlib.sha256(raw).hexdigest(), "mode": 0o444,
            "link_count": 1, "file_type": "regular_file",
        })
    projection = {"schema_version": CELL_EVIDENCE_PROJECTION_SCHEMA_VERSION, "members": members}
    projection_raw = canonical(projection)
    projection_identity = hashlib.sha256(EVIDENCE_PROJECTION_DOMAIN + projection_raw).hexdigest()
    projection_path = package / "evidence_projection.json"
    projection_path.write_bytes(projection_raw)
    projection_path.chmod(0o444)
    envelope = {
        "schema_version": CELL_EVIDENCE_ENVELOPE_SCHEMA_VERSION,
        "charter_logical_identity": result.charter_logical_identity,
        "campaign_identity": result.campaign_identity,
        "campaign_plan_identity": result.campaign_plan_identity,
        "cell_id": result.cell_id,
        "scenario_id": result.scenario_id,
        "source_scenario_id": result.source_scenario_id,
        "seed": result.seed,
        "metric_profile_identity": result.metric_profile_identity,
        "attempt_ledger_identity": result.attempt_ledger_identity,
        "attempt_ledger_revision": result.attempt_ledger_revision,
        "process_start_event_identity": result.process_start_event_identity,
        "runtime_authorization_identity": result.runtime_authorization_identity,
        "ros_domain_id": result.ros_domain_id,
        "campaign_output_root": result.campaign_output_root,
        "cell_output_path": result.cell_output_path,
        "argv": list(result.argv),
        "process_exit_status": result.process_exit_status,
        "projection_identity": projection_identity,
        "members": members,
    }
    envelope_path = package / "evidence_envelope.json"
    envelope_path.write_bytes(canonical(envelope))
    envelope_path.chmod(0o444)
    package.chmod(0o555)
    return package


def write_campaign_packages(parent, plan, committed, first_changes=None):
    return [
        write_evidence_package(parent, cell, plan, committed, result_changes=first_changes if index == 0 else None)
        for index, cell in enumerate(plan.cells)
    ]


def campaign_seal_data(charter, plan, committed, packages):
    records = []
    for cell, package in zip(plan.cells, packages, strict=True):
        observed = authenticate_slice_7g_cell_evidence_package(package, charter, committed, plan)
        records.append({
            "schema_version": CAMPAIGN_EVIDENCE_PACKAGE_RECORD_SCHEMA_VERSION,
            "cell_id": cell.cell_id,
            "relative_path": f"packages/{cell.cell_id}",
            "package_identity": observed.package_identity,
        })
    return {
        "schema_version": CAMPAIGN_EVIDENCE_SEAL_SCHEMA_VERSION,
        "charter_logical_identity": slice_7g_charter_identity(charter),
        "campaign_identity": plan.campaign_identity,
        "campaign_plan_identity": slice_7g_campaign_plan_identity(plan),
        "runtime_authorization_identity": committed.runtime_authorization_identity,
        "attempt_ledger_identity": slice_7g_attempt_ledger_identity(committed),
        "attempt_ledger_revision": committed.revision,
        "process_start_event_identity": committed.last_event_identity,
        "ros_domain_id": committed.domain_id,
        "campaign_output_root": committed.output_root,
        "evidence_root_relative_path": "evidence",
        "packages": records,
    }


def write_campaign_seal(evidence_root, data):
    path = evidence_root / CAMPAIGN_EVIDENCE_SEAL_PATH
    path.write_bytes(canonical(data))
    path.chmod(0o444)
    return path


def rewrite_campaign_seal(path, data):
    path.chmod(0o644)
    path.write_bytes(canonical(data))
    path.chmod(0o444)


def _make_fixture_tree_writable(root):
    if not root.exists() or root.is_symlink():
        return
    for current, directories, files in os.walk(root, topdown=False, followlinks=False):
        for name in files:
            path = Path(current) / name
            if not path.is_symlink():
                path.chmod(0o644)
        for name in directories:
            path = Path(current) / name
            if not path.is_symlink():
                path.chmod(0o755)
        Path(current).chmod(0o755)


@contextmanager
def sealed_campaign(*, first_changes=None):
    temporary = tempfile.TemporaryDirectory()
    parent = Path(temporary.name)
    old_parent = governance.SLICE_7G_EVIDENCE_PARENT
    governance.SLICE_7G_EVIDENCE_PARENT = str(parent)
    campaign_root = parent / "campaign-001"
    try:
        charter, committed, plan = committed_context(str(campaign_root))
        evidence_root = campaign_root / "evidence"
        packages_root = evidence_root / "packages"
        packages_root.mkdir(parents=True)
        packages = write_campaign_packages(packages_root, plan, committed, first_changes)
        seal_data = campaign_seal_data(charter, plan, committed, packages)
        seal_path = write_campaign_seal(evidence_root, seal_data)
        packages_root.chmod(0o555)
        evidence_root.chmod(0o555)
        campaign_root.chmod(0o555)
        yield charter, committed, plan, campaign_root, evidence_root, seal_path, packages, seal_data
    finally:
        _make_fixture_tree_writable(parent)
        governance.SLICE_7G_EVIDENCE_PARENT = old_parent
        temporary.cleanup()


def clone_sealed_package(source: Path, destination: Path) -> Path:
    """Create byte-identical contents with distinct root and member inodes."""

    shutil.copytree(source, destination, copy_function=shutil.copy2)
    for member in destination.iterdir():
        member.chmod(0o444)
    destination.chmod(0o555)
    return destination


class HostilePathLike:
    def __init__(self, behavior):
        self.behavior = behavior
        self.calls = 0

    def __fspath__(self):
        self.calls += 1
        if isinstance(self.behavior, BaseException):
            raise self.behavior
        if callable(self.behavior):
            return self.behavior(self.calls)
        return self.behavior


class HostileMapping(Mapping):
    def __init__(self, attack):
        self.attack = attack
        self.calls = []

    def _called(self, name):
        self.calls.append(name)
        if self.attack == name:
            if name == "__iter__":
                raise TypeError("attacker-controlled mapping hook")
            raise RuntimeError("attacker-controlled mapping hook")

    def __iter__(self):
        self._called("__iter__")
        return iter(())

    def __getitem__(self, key):
        self._called("__getitem__")
        raise KeyError(key)

    def __len__(self):
        self._called("__len__")
        return 0

    def items(self):
        self._called("items")
        return ()

    def keys(self):
        self._called("keys")
        return ()


class InconsistentMapping(Mapping):
    def __init__(self):
        self.calls = 0

    def __iter__(self):
        self.calls += 1
        return iter(("first", "second"))

    def __getitem__(self, key):
        self.calls += 1
        return key

    def __len__(self):
        self.calls += 1
        return 1


class MutatingMapping(Mapping):
    def __init__(self):
        self.data = {"first": 1}
        self.calls = 0

    def __iter__(self):
        self.calls += 1
        self.data["new"] = 2
        return iter(self.data)

    def __getitem__(self, key):
        self.calls += 1
        self.data.pop("first", None)
        return self.data[key]

    def __len__(self):
        self.calls += 1
        return len(self.data)


def mutate_envelope(package, mutator):
    package.chmod(0o755)
    path = package / "evidence_envelope.json"
    path.chmod(0o644)
    data = json.loads(path.read_bytes())
    mutator(data)
    if "members" in data:
        projection = {"schema_version": CELL_EVIDENCE_PROJECTION_SCHEMA_VERSION, "members": data["members"]}
        projection_raw = canonical(projection)
        projection_path = package / "evidence_projection.json"
        projection_path.chmod(0o644)
        projection_path.write_bytes(projection_raw)
        projection_path.chmod(0o444)
        data["projection_identity"] = hashlib.sha256(EVIDENCE_PROJECTION_DOMAIN + projection_raw).hexdigest()
    path.write_bytes(canonical(data))
    path.chmod(0o444)
    package.chmod(0o555)


def mutate_member_and_reseal(package, role, mutator):
    package.chmod(0o755)
    relative = MANDATORY_EVIDENCE_ROLE_PATHS[role]
    member_path = package / relative
    member_path.chmod(0o644)
    data = json.loads(member_path.read_bytes())
    mutator(data)
    raw = canonical(data)
    member_path.write_bytes(raw)
    member_path.chmod(0o444)
    envelope_path = package / "evidence_envelope.json"
    envelope_path.chmod(0o644)
    envelope = json.loads(envelope_path.read_bytes())
    descriptor = next(item for item in envelope["members"] if item["role"] == role)
    descriptor["size"] = len(raw)
    descriptor["sha256"] = hashlib.sha256(raw).hexdigest()
    projection = {"schema_version": CELL_EVIDENCE_PROJECTION_SCHEMA_VERSION, "members": envelope["members"]}
    projection_raw = canonical(projection)
    projection_path = package / "evidence_projection.json"
    projection_path.chmod(0o644)
    projection_path.write_bytes(projection_raw)
    projection_path.chmod(0o444)
    envelope["projection_identity"] = hashlib.sha256(EVIDENCE_PROJECTION_DOMAIN + projection_raw).hexdigest()
    envelope_path.write_bytes(canonical(envelope))
    envelope_path.chmod(0o444)
    package.chmod(0o555)


def charter_with_snapshot_members(members):
    data = charter_data()
    projection = {"schema_version": "ctr-scoped-source-snapshot-1", "members": members}
    identity = hashlib.sha256(
        b"ctr-slice-7g-authoring-source-snapshot-1\0"
        + json.dumps(projection, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    ).hexdigest()
    data["authoring"]["scoped_source_snapshot"]["members"] = members
    data["authoring"]["scoped_source_snapshot"]["identity"] = identity
    return data


def test_exact_positive_charter_and_successor_snapshot_gate():
    charter = load_slice_7g_charter(CHARTER_PATH)
    assert charter.data["endpoint"] == "simulation_only_promoted_completion"
    assert charter.data["governance"]["execution_authorized"] is False
    assert charter.data["campaign"]["run_cell_count"] == 15
    # Charter-v5 is a source correction after the immutable v2 predecessor.
    # A successor snapshot is deliberately a later independently reviewed phase.
    assert charter.data["authoring"]["post_implementation_snapshot_required"] is True
    assert_code(
        "snapshot_member_mismatch",
        verify_authoring_source_snapshot,
        charter,
        REPOSITORY_ROOT,
    )


def test_canonical_round_trip_has_no_newline():
    raw = CHARTER_PATH.read_bytes()
    charter = load_slice_7g_charter(CHARTER_PATH)
    assert raw == canonical_slice_7g_charter_bytes(charter)
    assert not raw.endswith(b"\n")
    assert raw.decode("utf-8").encode("utf-8") == raw


def test_independent_recursive_serializer_matches_canonical_bytes():
    def independent(value):
        if type(value) is dict:
            return b"{" + b",".join(
                json.dumps(key, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
                + b":" + independent(value[key])
                for key in sorted(value)
            ) + b"}"
        if type(value) is list:
            return b"[" + b",".join(independent(item) for item in value) + b"]"
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")

    assert independent(charter_data()) == CHARTER_PATH.read_bytes()
    assert independent(charter_data()) == canonical_slice_7g_charter_bytes(charter_data())


def test_deep_immutability_and_caller_alias_detachment():
    source = charter_data()
    charter = validate_slice_7g_charter(source)
    original_identity = slice_7g_charter_identity(charter)
    source["campaign"]["seeds"][0] = 999
    source["readiness"]["required_topics"].append("/mutated")
    assert charter.data["campaign"]["seeds"][0] == 11
    assert "/mutated" not in charter.data["readiness"]["required_topics"]
    assert slice_7g_charter_identity(charter) == original_identity
    with pytest.raises(TypeError):
        charter.data["endpoint"] = "mutated"


def test_direct_construction_rejected():
    assert_code("direct_construction", Slice7GCharter)


def test_duplicate_json_key_rejected():
    raw = CHARTER_PATH.read_bytes()
    duplicate = b'{"schema_version":"ctr-slice-7g-charter-4",' + raw[1:]
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "duplicate.json"
        path.write_bytes(duplicate)
        assert_code("duplicate_json_key", load_slice_7g_charter, path)


def test_noncanonical_bytes_rejected():
    pretty = json.dumps(charter_data(), indent=2, ensure_ascii=False).encode("utf-8")
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "pretty.json"
        path.write_bytes(pretty)
        assert_code("noncanonical_json", load_slice_7g_charter, path)


def test_unknown_top_level_field_rejected():
    data = charter_data()
    data["unknown"] = 1
    assert_code("unknown_field", validate_slice_7g_charter, data)


def test_missing_top_level_field_rejected():
    data = charter_data()
    del data["objective"]
    assert_code("missing_field", validate_slice_7g_charter, data)


def test_boolean_as_number_rejected():
    data = charter_data()
    data["campaign"]["duration_seconds"] = True
    assert_code("campaign_duration", validate_slice_7g_charter, data)


def test_integer_as_float_rejected():
    data = charter_data()
    data["campaign"]["duration_seconds"] = 25
    assert_code("campaign_duration", validate_slice_7g_charter, data)


def test_nonfinite_json_rejected():
    raw = CHARTER_PATH.read_bytes().replace(b'"duration_seconds":25.0', b'"duration_seconds":NaN')
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "nonfinite.json"
        path.write_bytes(raw)
        assert_code("nonfinite_json", load_slice_7g_charter, path)


def test_unsafe_snapshot_path_rejected():
    data = charter_data()
    data["authoring"]["scoped_source_snapshot"]["members"][0]["path"] = "../escape"
    assert_code("unsafe_path", validate_slice_7g_charter, data)


def test_invalid_metric_unit_rejected():
    data = charter_data()
    metric(data, "steady_state_error")["unit"] = "millimeters"
    assert_code("metric_unit", validate_slice_7g_charter, data)


def test_missing_metric_unit_rejected():
    data = charter_data()
    del metric(data, "steady_state_error")["unit"]
    assert_code("missing_field", validate_slice_7g_charter, data)


def test_duplicate_seed_rejected():
    data = charter_data()
    data["campaign"]["seeds"][-1] = 44
    assert_code("duplicate_seed", validate_slice_7g_charter, data)


def test_duplicate_scenario_rejected():
    data = charter_data()
    data["campaign"]["scenarios"].append(deepcopy(data["campaign"]["scenarios"][0]))
    assert_code("duplicate_scenario", validate_slice_7g_charter, data)


def test_duplicate_metric_rejected():
    data = charter_data()
    data["acceptance_contract"]["metrics"].append(deepcopy(data["acceptance_contract"]["metrics"][0]))
    assert_code("duplicate_metric", validate_slice_7g_charter, data)


@pytest.mark.parametrize(
    ("name", "field", "value"),
    [
        ("steady_state_error", "threshold", 0.0031),
        ("final_goal_error", "threshold", 0.0031),
        ("goal_hold_duration", "threshold", 0.49),
    ],
)
def test_tracking_threshold_contradictions_rejected(name, field, value):
    data = charter_data()
    metric(data, name)[field] = value
    assert_code("tracking_threshold", validate_slice_7g_charter, data)


def test_timing_cannot_be_promotion_blocking():
    data = charter_data()
    data["acceptance_contract"]["timing_policy"]["promotion_blocking"] = True
    assert_code("timing_promotion_blocking", validate_slice_7g_charter, data)


def test_timing_metric_cannot_be_promotion_blocking():
    data = charter_data()
    metric(data, "deadline_overrun_percentage")["promotion_blocking"] = True
    assert_code("timing_promotion_blocking", validate_slice_7g_charter, data)


def test_real_time_claim_rejected():
    data = charter_data()
    data["governance"]["real_time_performance_claim"] = True
    assert_code("real_time_claim", validate_slice_7g_charter, data)


def test_missing_non_real_time_limitation_rejected():
    data = charter_data()
    data["promotion_contract"]["limitations"].remove("NO_REAL_TIME_PERFORMANCE_CLAIM")
    assert_code("missing_non_real_time_limitation", validate_slice_7g_charter, data)


def test_timing_cannot_enter_functional_reasons():
    data = charter_data()
    data["acceptance_contract"]["timing_policy"]["failure_in_functional_reasons"] = True
    assert_code("timing_functional_reason", validate_slice_7g_charter, data)


def test_runtime_authorization_true_rejected():
    data = charter_data()
    data["governance"]["execution_authorized"] = True
    assert_code("runtime_authorized", validate_slice_7g_charter, data)


def test_launchable_true_rejected():
    data = charter_data()
    data["governance"]["launchable"] = True
    assert_code("launchable", validate_slice_7g_charter, data)


def test_runtime_argv_reordering_rejected():
    data = charter_data()
    argv = data["runtime_template"]["argv_template"]
    argv[1], argv[3] = argv[3], argv[1]
    assert_code("runtime_argv", validate_slice_7g_charter, data)


def test_domain_allocated_prematurely_rejected():
    data = charter_data()
    data["domain_policy"]["domain_allocated"] = True
    data["domain_policy"]["selected_domain_id"] = 100
    assert_code("domain_preallocated", validate_slice_7g_charter, data)


def test_domain_outside_policy_rejected():
    data = charter_data()
    data["domain_policy"]["minimum_domain_id"] = 99
    assert_code("domain_range", validate_slice_7g_charter, data)


def test_allocation_proposal_requires_separate_runtime_authorization_identity():
    ledger = create_slice_7g_initial_attempt_ledger(load_slice_7g_charter(CHARTER_PATH), "campaign-001")
    assert_code(
        "runtime_authorization_required",
        propose_slice_7g_attempt_event,
        ledger,
        "domain_and_output_allocated",
        "allocate-001",
        "2026-08-19T00:00:00Z",
        domain_id=145,
        output_root="/home/ankid/ctr_mppi_evidence/slice_7g/campaign-001",
    )


def test_attempt_above_one_rejected():
    data = charter_data()
    data["attempt_budget"]["consumed_campaigns"] = 2
    assert_code("attempt_count", validate_slice_7g_charter, data)


def test_retry_above_zero_rejected():
    data = charter_data()
    data["attempt_budget"]["retries_authorized"] = 1
    assert_code("retry_count", validate_slice_7g_charter, data)


def test_preflight_failure_leaves_attempt_unconsumed():
    charter = load_slice_7g_charter(CHARTER_PATH)
    ledger = create_slice_7g_initial_attempt_ledger(charter, "campaign-001")
    event = propose_slice_7g_attempt_event(
        ledger, "preflight_failed_before_process_creation", "preflight-failed-001", "2026-08-19T00:00:00Z"
    )
    result = validate_slice_7g_attempt_transition(ledger, event)
    assert result.consumed_campaign_attempts == 0
    assert result.revision == 1
    assert result.process_start_committed is False


def test_process_start_consumes_one_of_one():
    ledger = allocated_ledger()
    plan, event = process_start_proposal(ledger)
    result = validate_slice_7g_attempt_transition(ledger, event, campaign_plan=plan)
    assert result.consumed_campaign_attempts == 1
    assert result.process_start_committed is True


def test_process_start_requires_final_campaign_plan_binding():
    ledger = allocated_ledger()
    assert_code(
        "process_start_plan",
        propose_slice_7g_attempt_event,
        ledger,
        "process_start_commit",
        "start-001",
        "2026-08-19T00:00:01Z",
    )


def test_second_process_start_rejected():
    ledger = allocated_ledger()
    plan, first = process_start_proposal(ledger)
    committed = validate_slice_7g_attempt_transition(ledger, first, campaign_plan=plan)
    assert_code("attempt_exhausted", propose_slice_7g_attempt_event, committed, "process_start_commit", "start-002", "2026-08-19T00:00:02Z")


def test_retry_after_process_start_rejected():
    ledger = allocated_ledger()
    assert_code("retry_not_authorized", propose_slice_7g_attempt_event, ledger, "retry_requested", "retry-001", "2026-08-19T00:00:01Z")


def test_missing_implementation_gate_rejected():
    data = charter_data()
    data["implementation_requirements"].pop()
    assert_code("missing_implementation_gate", validate_slice_7g_charter, data)


def test_missing_readiness_timeout_rejected():
    data = charter_data()
    del data["readiness"]["timeout_seconds"]
    assert_code("missing_field", validate_slice_7g_charter, data)


def test_missing_readiness_stable_sample_count_rejected():
    data = charter_data()
    del data["readiness"]["minimum_stable_samples"]
    assert_code("missing_field", validate_slice_7g_charter, data)


def test_missing_tactile_readiness_topic_rejected():
    data = charter_data()
    data["readiness"]["required_topics"].remove("/ctr/tactile/state")
    assert_code("readiness_tactile_topic", validate_slice_7g_charter, data)


def test_missing_scope_exclusion_rejected():
    data = charter_data()
    data["completion_scope"]["excluded"].remove("Physical CTR drivers")
    assert_code("completion_scope", validate_slice_7g_charter, data)


def test_missing_post_implementation_snapshot_gate_rejected():
    data = charter_data()
    data["authoring"]["post_implementation_snapshot_required"] = False
    assert_code("source_snapshot_gate", validate_slice_7g_charter, data)


def test_missing_build_entry_gate_rejected():
    data = charter_data()
    data["entry_criteria"] = [item for item in data["entry_criteria"] if item["id"] != "ISOLATED_BUILD_PASSED"]
    assert_code("missing_entry_gate", validate_slice_7g_charter, data)


def test_build_command_template_mutation_rejected():
    data = charter_data()
    data["build_test_gate"]["command_templates"][0] = "colcon build"
    assert_code("build_commands", validate_slice_7g_charter, data)


def test_missing_independent_audit_gate_rejected():
    data = charter_data()
    data["promotion_contract"]["independent_audit_required"] = False
    assert_code("independent_audit_gate", validate_slice_7g_charter, data)


def test_missing_promotion_gate_rejected():
    data = charter_data()
    data["promotion_contract"]["required_gates"].remove("EXTERNAL_IMMUTABLE_PROMOTION_RECORD_CREATED")
    assert_code("missing_promotion_gate", validate_slice_7g_charter, data)


def test_physical_hardware_claim_rejected():
    data = charter_data()
    data["governance"]["physical_hardware_claim"] = True
    assert_code("physical_hardware_claim", validate_slice_7g_charter, data)


def test_deterministic_domain_separated_logical_identity():
    data = charter_data()
    canonical = canonical_slice_7g_charter_bytes(data)
    expected = hashlib.sha256(IDENTITY_DOMAIN + canonical).hexdigest()
    assert CHARTER_IDENTITY_ALGORITHM == "sha256:ctr-slice-7g-charter-canonical-7"
    assert slice_7g_charter_identity(data) == expected
    assert slice_7g_charter_identity(validate_slice_7g_charter(deepcopy(data))) == expected
    assert hashlib.sha256(canonical).hexdigest() != expected


def test_forged_charter_instance_is_reconstructed_and_rejected():
    forged = object.__new__(Slice7GCharter)
    object.__setattr__(forged, "data", charter_data())
    object.__setattr__(forged, "canonical_bytes", b'{"forged":true}')
    assert_code("charter_canonical_mismatch", validate_slice_7g_charter, forged)


def test_partially_initialized_charter_is_rejected():
    forged = object.__new__(Slice7GCharter)
    assert_code("invalid_charter_record", validate_slice_7g_charter, forged)


def test_charter_subclass_is_rejected_even_with_valid_parent_state():
    class ForgedCharter(Slice7GCharter):
        pass

    forged = object.__new__(ForgedCharter)
    object.__setattr__(forged, "data", validate_slice_7g_charter(charter_data()).data)
    object.__setattr__(forged, "canonical_bytes", CHARTER_PATH.read_bytes())
    assert_code("charter_exact_type", validate_slice_7g_charter, forged)


@pytest.mark.parametrize(
    ("factory", "validator", "code"),
    [
        (
            lambda: Slice7GScenario("", "centerline_target", "circular_arc"),
            validate_slice_7g_scenario,
            "scenario_id",
        ),
        (
            lambda: Slice7GMetric("unknown_metric", "m", "maximum_per_cell", "less_than_or_equal", 1.0, True, "x"),
            validate_slice_7g_metric,
            "metric_name",
        ),
        (lambda: Slice7GDomainPolicy(99, 199, False, None), validate_slice_7g_domain_policy, "domain_range"),
        (lambda: Slice7GAttemptBudget(1, -1, 0), validate_slice_7g_attempt_budget, "attempt_count"),
    ],
)
def test_invalid_direct_leaf_construction_is_fail_closed(factory, validator, code):
    assert_code(code, factory)


@pytest.mark.parametrize(
    ("record_type", "validator", "fields", "code"),
    [
        (Slice7GScenario, validate_slice_7g_scenario, {"scenario_id": "", "source_scenario_id": "centerline_target", "geometry_profile": "circular_arc"}, "scenario_id"),
        (Slice7GMetric, validate_slice_7g_metric, {"name": "unknown_metric", "unit": "m", "aggregation": "x", "comparison": "equal", "threshold": 0, "promotion_blocking": True, "rationale": "x"}, "metric_name"),
        (Slice7GDomainPolicy, validate_slice_7g_domain_policy, {"minimum_domain_id": 100, "maximum_domain_id": 199, "domain_allocated": True, "selected_domain_id": 145}, "domain_preallocated"),
        (Slice7GAttemptBudget, validate_slice_7g_attempt_budget, {"maximum_campaigns": 1, "consumed_campaigns": -1, "retries_authorized": 0}, "attempt_count"),
    ],
)
def test_forged_leaf_records_are_revalidated(record_type, validator, fields, code):
    forged = object.__new__(record_type)
    for name, value in fields.items():
        object.__setattr__(forged, name, value)
    assert_code(code, validator, forged)


def test_campaign_cell_detaches_caller_owned_argv():
    _, _, plan = plan_and_ledger()
    cell = plan.cells[0]
    caller_argv = list(cell.argv)
    detached = replace(cell, argv=caller_argv)
    caller_argv[0] = "mutated"
    assert detached.argv[0] == "ctr_run_evaluation"
    assert type(detached.argv) is tuple


@pytest.mark.parametrize(
    ("callable_", "args", "code"),
    [
        (load_slice_7g_charter, (object(),), "charter_path_type"),
        (load_slice_7g_charter, ("bad\x00path",), "charter_open"),
        (verify_authoring_source_snapshot, (charter_data(), object()), "snapshot_root_type"),
        (verify_authoring_source_snapshot, (charter_data(), "/definitely/missing/slice7g-root"), "snapshot_root_missing"),
        (verify_authoring_source_snapshot, (charter_data(), "bad\x00root"), "snapshot_root_open"),
    ],
)
def test_public_path_errors_are_normalized(callable_, args, code):
    assert_code(code, callable_, *args)


def test_snapshot_non_directory_root_is_normalized(tmp_path):
    root = tmp_path / "not-a-directory"
    root.write_text("x", encoding="utf-8")
    assert_code("snapshot_root_not_directory", verify_authoring_source_snapshot, charter_data(), root)


def test_invalid_real_utc_timestamp_rejected():
    ledger = create_slice_7g_initial_attempt_ledger(load_slice_7g_charter(CHARTER_PATH), "campaign-001")
    assert_code(
        "event_timestamp",
        propose_slice_7g_attempt_event,
        ledger,
        "preflight_failed_before_process_creation",
        "event-001",
        "2026-99-99T99:99:99Z",
    )


def test_intermediate_symlink_snapshot_escape_is_rejected(tmp_path):
    external = tmp_path / "external"
    external.mkdir()
    member = external / "member.txt"
    member.write_bytes(b"outside")
    repository = tmp_path / "repository"
    repository.mkdir()
    (repository / "link").symlink_to(external, target_is_directory=True)
    members = [{"path": "link/member.txt", "size": 7, "sha256": hashlib.sha256(b"outside").hexdigest()}]
    assert_code("snapshot_component_not_directory", verify_authoring_source_snapshot, charter_with_snapshot_members(members), repository)


def test_final_snapshot_symlink_is_rejected(tmp_path):
    external = tmp_path / "external.txt"
    external.write_bytes(b"outside")
    repository = tmp_path / "repository"
    repository.mkdir()
    (repository / "member.txt").symlink_to(external)
    members = [{"path": "member.txt", "size": 7, "sha256": hashlib.sha256(b"outside").hexdigest()}]
    assert_code("snapshot_member_open", verify_authoring_source_snapshot, charter_with_snapshot_members(members), repository)


def test_snapshot_hardlink_alias_is_rejected(tmp_path):
    repository = tmp_path / "repository"
    repository.mkdir()
    first = repository / "first.txt"
    first.write_bytes(b"same inode")
    os.link(first, repository / "second.txt")
    digest = hashlib.sha256(b"same inode").hexdigest()
    members = [
        {"path": "first.txt", "size": 10, "sha256": digest},
        {"path": "second.txt", "size": 10, "sha256": digest},
    ]
    assert_code("snapshot_hardlink_alias", verify_authoring_source_snapshot, charter_with_snapshot_members(members), repository)


def test_snapshot_descriptors_close_on_success_and_failure(tmp_path):
    before = len(os.listdir("/proc/self/fd"))
    with pytest.raises(Slice7GGovernanceError, match="snapshot_member_mismatch"):
        verify_authoring_source_snapshot(charter_data(), REPOSITORY_ROOT)
    assert len(os.listdir("/proc/self/fd")) == before
    missing = tmp_path / "missing"
    assert_code("snapshot_root_missing", verify_authoring_source_snapshot, charter_data(), missing)
    assert len(os.listdir("/proc/self/fd")) == before


def test_ledger_canonical_identity_is_deterministic_and_domain_separated():
    ledger = create_slice_7g_initial_attempt_ledger(load_slice_7g_charter(CHARTER_PATH), "campaign-001")
    canonical = canonical_slice_7g_attempt_ledger_bytes(ledger)
    assert ledger.schema_version == ATTEMPT_LEDGER_SCHEMA_VERSION
    assert slice_7g_attempt_ledger_identity(ledger) == slice_7g_attempt_ledger_identity(json.loads(canonical))
    assert hashlib.sha256(canonical).hexdigest() != slice_7g_attempt_ledger_identity(ledger)


def test_stale_ledger_predecessor_rejected_after_first_commit():
    ledger = allocated_ledger()
    plan, first = process_start_proposal(ledger, "start-a", "2026-08-19T00:00:01Z")
    _, second = process_start_proposal(ledger, "start-b", "2026-08-19T00:00:02Z")
    advanced = validate_slice_7g_attempt_transition(ledger, first, campaign_plan=plan)
    assert_code("stale_ledger_predecessor", validate_slice_7g_attempt_transition, advanced, second, campaign_plan=plan)


def test_duplicate_ledger_event_id_rejected():
    ledger = create_slice_7g_initial_attempt_ledger(load_slice_7g_charter(CHARTER_PATH), "campaign-001")
    first = propose_slice_7g_attempt_event(ledger, "preflight_failed_before_process_creation", "same-event", "2026-08-19T00:00:00Z")
    advanced = validate_slice_7g_attempt_transition(ledger, first)
    duplicate = propose_slice_7g_attempt_event(advanced, "preflight_failed_before_process_creation", "same-event", "2026-08-19T00:00:01Z")
    assert_code("duplicate_ledger_event", validate_slice_7g_attempt_transition, advanced, duplicate)


def test_attempt_event_semantic_forgery_rejected_before_transition():
    ledger = allocated_ledger()
    _, event = process_start_proposal(ledger)
    assert_code("event_semantics", replace, event, resulting_attempt_count=0, process_start_consumed=False)


def test_attempt_event_identity_is_recomputed():
    ledger = allocated_ledger()
    _, event = process_start_proposal(ledger)
    canonical = canonical_slice_7g_attempt_event_bytes(event)
    assert event.schema_version == ATTEMPT_EVENT_SCHEMA_VERSION
    assert slice_7g_attempt_event_identity(event) != hashlib.sha256(canonical).hexdigest()


@pytest.mark.parametrize(
    ("record_type", "public_api", "code"),
    [
        (Slice7GAttemptLedger, canonical_slice_7g_attempt_ledger_bytes, "attempt_ledger_type"),
        (Slice7GAttemptEvent, canonical_slice_7g_attempt_event_bytes, "attempt_event_type"),
        (Slice7GCampaignPlan, canonical_slice_7g_campaign_plan_bytes, "campaign_plan_type"),
    ],
)
def test_partially_initialized_exported_records_are_rejected(record_type, public_api, code):
    assert_code(code, public_api, object.__new__(record_type))


def test_exact_fifteen_cell_cartesian_campaign_plan():
    _, _, plan = plan_and_ledger()
    assert plan.schema_version == CAMPAIGN_PLAN_SCHEMA_VERSION
    assert len(plan.cells) == 15
    assert len({(cell.scenario_id, cell.seed) for cell in plan.cells}) == 15
    assert {cell.ros_domain_id for cell in plan.cells} == {145}
    assert {cell.campaign_output_root for cell in plan.cells} == {"/home/ankid/ctr_mppi_evidence/slice_7g/campaign-001"}
    assert all(cell.domain_allocation_requested is False for cell in plan.cells)


@pytest.mark.parametrize("mutation", ["missing", "extra"])
def test_campaign_plan_wrong_cell_count_rejected(mutation):
    _, _, plan = plan_and_ledger()
    cells = plan.cells[:-1] if mutation == "missing" else plan.cells + (plan.cells[-1],)
    assert_code("campaign_cell_count", replace, plan, cells=cells)


def test_duplicate_campaign_cell_rejected_at_bijection_stage():
    _, _, plan = plan_and_ledger()
    cells = list(plan.cells)
    cells[-1] = replace(cells[-1], cell_id=cells[0].cell_id, scenario_id=cells[0].scenario_id, source_scenario_id=cells[0].source_scenario_id, seed=cells[0].seed, cell_output_path=cells[0].cell_output_path, argv=cells[0].argv)
    assert_code("duplicate_campaign_cell", replace, plan, cells=tuple(cells))


def test_campaign_scenario_source_mismatch_rejected():
    _, _, plan = plan_and_ledger()
    assert_code("scenario_source_mismatch", replace, plan.cells[0], source_scenario_id="lateral_offset_target")


def test_campaign_per_cell_domain_mismatch_rejected():
    _, _, plan = plan_and_ledger()
    cells = (replace(plan.cells[0], ros_domain_id=146),) + plan.cells[1:]
    assert_code("cell_binding_mismatch", replace, plan, cells=cells)


def test_campaign_output_root_escape_rejected():
    _, _, plan = plan_and_ledger()
    assert_code(
        "cell_output_escape",
        replace,
        plan.cells[0],
        cell_output_path="/home/ankid/ctr_mppi_evidence/slice_7g/another-campaign/cell",
    )


def test_campaign_different_valid_output_root_rejected_by_plan_binding():
    _, _, plan = plan_and_ledger()
    cell = plan.cells[0]
    other_root = "/home/ankid/ctr_mppi_evidence/slice_7g/other-campaign"
    other_output = f"{other_root}/cells/{cell.cell_id}"
    argv = cell.argv[:-1] + (other_output,)
    changed = replace(cell, campaign_output_root=other_root, cell_output_path=other_output, argv=argv)
    assert_code("cell_binding_mismatch", replace, plan, cells=(changed,) + plan.cells[1:])


def test_campaign_cell_cannot_request_domain_allocation():
    _, _, plan = plan_and_ledger()
    assert_code("cell_domain_allocation", replace, plan.cells[0], domain_allocation_requested=True)


def test_campaign_semantic_argv_reordering_rejected():
    _, _, plan = plan_and_ledger()
    argv = list(plan.cells[0].argv)
    argv[1], argv[3] = argv[3], argv[1]
    assert_code("cell_argv_mismatch", replace, plan.cells[0], argv=argv)


def test_valid_sealed_cell_evidence_and_exact_campaign_reconciliation():
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, _, packages, _):
        observed = authenticate_slice_7g_cell_evidence_package(packages[0], charter, ledger, plan)
        assert observed.cell_result.cell_id == plan.cells[0].cell_id
        assert observed.package_identity != observed.projection_identity
        aggregate = reconcile_slice_7g_campaign_results(charter, plan, ledger, campaign_root)
        assert aggregate.total_result_count == 15
        assert len(aggregate.evidence_package_identities) == 15
        assert aggregate.campaign_evidence_snapshot_identity != aggregate.evidence_package_identities[0]
        assert aggregate.functional_promotion_pass is True
        assert aggregate.total_valid_aligned_samples == 300
        assert aggregate.total_collision_samples == 0


def test_campaign_seal_is_closed_canonical_and_snapshot_bound():
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, seal_path, _, seal_data):
        seal = validate_slice_7g_campaign_evidence_seal(seal_data)
        assert seal_path.read_bytes() == canonical_slice_7g_campaign_evidence_seal_bytes(seal)
        expected_snapshot = slice_7g_campaign_evidence_snapshot_identity(seal)
        result = reconcile_slice_7g_campaign_results(charter, plan, ledger, campaign_root)
        assert result.campaign_evidence_snapshot_identity == expected_snapshot
        assert len(seal.packages) == 15


@pytest.mark.parametrize(
    ("mutation", "code"),
    [
        ("missing", "campaign_seal_open"),
        ("malformed", "invalid_json"),
        ("writable", "campaign_seal_mode"),
        ("symlink", "campaign_seal_open"),
        ("hardlink", "campaign_seal_hardlink"),
    ],
)
def test_campaign_seal_physical_failures_reach_named_stage(mutation, code):
    with sealed_campaign() as (charter, ledger, plan, campaign_root, evidence_root, seal_path, _, _):
        if mutation == "missing":
            evidence_root.chmod(0o755)
            seal_path.unlink()
            evidence_root.chmod(0o555)
        elif mutation == "malformed":
            seal_path.chmod(0o644)
            seal_path.write_bytes(b"{")
            seal_path.chmod(0o444)
        elif mutation == "writable":
            seal_path.chmod(0o644)
        elif mutation == "symlink":
            evidence_root.chmod(0o755)
            campaign_root.chmod(0o755)
            retained = campaign_root / "retained-seal"
            seal_path.rename(retained)
            seal_path.symlink_to("../retained-seal")
            campaign_root.chmod(0o555)
            evidence_root.chmod(0o555)
        else:
            campaign_root.chmod(0o755)
            os.link(seal_path, campaign_root / "seal-hardlink-alias")
            campaign_root.chmod(0o555)
        assert_code(code, reconcile_slice_7g_campaign_results, charter, plan, ledger, campaign_root)


@pytest.mark.parametrize(
    ("field", "value", "code"),
    [
        ("charter_logical_identity", "a" * 64, "campaign_seal_charter_logical_identity_mismatch"),
        ("runtime_authorization_identity", "b" * 64, "campaign_seal_runtime_authorization_identity_mismatch"),
        ("attempt_ledger_identity", "c" * 64, "campaign_seal_attempt_ledger_identity_mismatch"),
        ("attempt_ledger_revision", 99, "campaign_seal_attempt_ledger_revision_mismatch"),
        ("process_start_event_identity", "d" * 64, "campaign_seal_process_start_event_identity_mismatch"),
        ("campaign_plan_identity", "e" * 64, "campaign_seal_campaign_plan_identity_mismatch"),
        ("ros_domain_id", 146, "campaign_seal_ros_domain_id_mismatch"),
    ],
)
def test_campaign_seal_committed_authority_bindings_are_enforced(field, value, code):
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, seal_path, _, seal_data):
        seal_data[field] = value
        rewrite_campaign_seal(seal_path, seal_data)
        assert_code(code, reconcile_slice_7g_campaign_results, charter, plan, ledger, campaign_root)


@pytest.mark.parametrize(
    ("relative_path", "code"),
    [
        ("/tmp/external-package", "unsafe_path"),
        ("packages/../external-package", "unsafe_path"),
        ("packages-sibling/centerline.seed_0000000011", "campaign_evidence_package_path"),
    ],
)
def test_campaign_seal_rejects_absolute_traversal_and_sibling_prefix_package_paths(relative_path, code):
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, seal_path, _, seal_data):
        seal_data["packages"][0]["relative_path"] = relative_path
        rewrite_campaign_seal(seal_path, seal_data)
        assert_code(code, reconcile_slice_7g_campaign_results, charter, plan, ledger, campaign_root)


def test_campaign_api_rejects_root_outside_the_ledger_binding():
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, _, _, _):
        assert_code(
            "campaign_root_ledger_mismatch",
            reconcile_slice_7g_campaign_results,
            charter,
            plan,
            ledger,
            str(campaign_root) + "-sibling",
        )


@pytest.mark.parametrize("mutation", ["missing", "extra"])
def test_campaign_physical_package_inventory_is_exact(mutation):
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, _, packages, _):
        packages_root = packages[0].parent
        packages_root.chmod(0o755)
        if mutation == "missing":
            packages[0].rename(packages_root / "removed-package")
        else:
            extra = packages_root / "extra-package"
            extra.mkdir()
            extra.chmod(0o555)
        packages_root.chmod(0o555)
        assert_code(
            "campaign_package_inventory",
            reconcile_slice_7g_campaign_results,
            charter,
            plan,
            ledger,
            campaign_root,
        )


def test_campaign_seal_output_root_binding_is_exact():
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, seal_path, _, seal_data):
        seal_data["campaign_output_root"] = str(campaign_root.parent / "other-campaign")
        rewrite_campaign_seal(seal_path, seal_data)
        assert_code(
            "campaign_seal_campaign_output_root_mismatch",
            reconcile_slice_7g_campaign_results,
            charter,
            plan,
            ledger,
            campaign_root,
        )


def test_campaign_root_parent_symlink_is_rejected_component_by_component():
    temporary = tempfile.TemporaryDirectory()
    parent = Path(temporary.name)
    old_parent = governance.SLICE_7G_EVIDENCE_PARENT
    governance.SLICE_7G_EVIDENCE_PARENT = str(parent)
    try:
        real_parent = parent / "real"
        real_parent.mkdir()
        link_parent = parent / "link"
        link_parent.symlink_to(real_parent, target_is_directory=True)
        ledger_root = link_parent / "campaign-001"
        real_campaign_root = real_parent / "campaign-001"
        charter, ledger, plan = committed_context(str(ledger_root))
        evidence_root = real_campaign_root / "evidence"
        packages_root = evidence_root / "packages"
        packages_root.mkdir(parents=True)
        packages = write_campaign_packages(packages_root, plan, ledger)
        seal_data = campaign_seal_data(charter, plan, ledger, packages)
        write_campaign_seal(evidence_root, seal_data)
        packages_root.chmod(0o555)
        evidence_root.chmod(0o555)
        real_campaign_root.chmod(0o555)
        assert_code(
            "campaign_root_open",
            reconcile_slice_7g_campaign_results,
            charter,
            plan,
            ledger,
            ledger_root,
        )
    finally:
        _make_fixture_tree_writable(parent)
        governance.SLICE_7G_EVIDENCE_PARENT = old_parent
        temporary.cleanup()


def test_campaign_shared_lock_blocks_writer_and_releases_after_success(monkeypatch):
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, seal_path, _, _):
        original = governance._open_campaign_package_authority
        probed = False

        def probe_writer(campaign, package, *args):
            nonlocal probed
            if not probed:
                probed = True
                descriptor = os.open(seal_path, os.O_RDONLY)
                try:
                    with pytest.raises(BlockingIOError):
                        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
                finally:
                    os.close(descriptor)
            return original(campaign, package, *args)

        monkeypatch.setattr(governance, "_open_campaign_package_authority", probe_writer)
        result = reconcile_slice_7g_campaign_results(charter, plan, ledger, campaign_root)
        assert result.total_result_count == 15
        assert probed is True
        descriptor = os.open(seal_path, os.O_RDONLY)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)


def test_existing_exclusive_seal_lock_fails_immediately_and_reader_releases_cleanly():
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, seal_path, _, _):
        before = len(os.listdir("/proc/self/fd"))
        descriptor = os.open(seal_path, os.O_RDONLY)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            assert_code(
                "campaign_seal_lock_busy",
                reconcile_slice_7g_campaign_results,
                charter,
                plan,
                ledger,
                campaign_root,
            )
        finally:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
            os.close(descriptor)
        assert len(os.listdir("/proc/self/fd")) == before


def test_campaign_lock_primitive_unavailable_fails_closed_without_descriptor_leak(monkeypatch):
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, _, _, _):
        real_flock = governance.fcntl.flock

        def unavailable(descriptor, operation):
            if operation & fcntl.LOCK_SH:
                raise OSError(governance.errno.ENOSYS, "unsupported")
            return real_flock(descriptor, operation)

        before = len(os.listdir("/proc/self/fd"))
        monkeypatch.setattr(governance.fcntl, "flock", unavailable)
        assert_code(
            "campaign_seal_lock_unavailable",
            reconcile_slice_7g_campaign_results,
            charter,
            plan,
            ledger,
            campaign_root,
        )
        assert len(os.listdir("/proc/self/fd")) == before


def test_campaign_lock_releases_after_post_lock_package_identity_failure():
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, seal_path, _, seal_data):
        seal_data["packages"][0]["package_identity"] = "0" * 64
        rewrite_campaign_seal(seal_path, seal_data)
        assert_code(
            "campaign_seal_package_identity_mismatch",
            reconcile_slice_7g_campaign_results,
            charter,
            plan,
            ledger,
            campaign_root,
        )
        descriptor = os.open(seal_path, os.O_RDONLY)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)


def test_campaign_replaced_seal_is_rejected_at_final_barrier(monkeypatch):
    with sealed_campaign() as (charter, ledger, plan, campaign_root, evidence_root, seal_path, _, _):
        original = governance._open_campaign_package_authority
        calls = 0

        def replace_seal(campaign, package, *args):
            nonlocal calls
            calls += 1
            if calls == 15:
                raw = seal_path.read_bytes()
                evidence_root.chmod(0o755)
                seal_path.rename(evidence_root / "retained-seal")
                seal_path.write_bytes(raw)
                seal_path.chmod(0o444)
                evidence_root.chmod(0o555)
            return original(campaign, package, *args)

        monkeypatch.setattr(governance, "_open_campaign_package_authority", replace_seal)
        assert_code(
            "campaign_evidence_late_change",
            reconcile_slice_7g_campaign_results,
            charter,
            plan,
            ledger,
            campaign_root,
        )
        assert calls == 15


@pytest.mark.parametrize(
    ("field", "value", "code"),
    [
        ("charter_logical_identity", "a" * 64, "evidence_charter_logical_identity_mismatch"),
        ("campaign_identity", "9" * 64, "evidence_campaign_identity_mismatch"),
        ("attempt_ledger_identity", "f" * 64, "evidence_attempt_ledger_identity_mismatch"),
        ("attempt_ledger_revision", 999, "evidence_attempt_ledger_revision_mismatch"),
        ("process_start_event_identity", "e" * 64, "evidence_process_start_event_identity_mismatch"),
        ("runtime_authorization_identity", "d" * 64, "evidence_runtime_authorization_identity_mismatch"),
        ("campaign_plan_identity", "c" * 64, "evidence_campaign_plan_identity_mismatch"),
        ("metric_profile_identity", "b" * 64, "evidence_metric_profile_identity_mismatch"),
        ("ros_domain_id", 146, "evidence_ros_domain_id_mismatch"),
    ],
)
def test_evidence_context_mismatch_is_rejected(field, value, code):
    charter, ledger, plan = committed_context()
    with tempfile.TemporaryDirectory() as directory:
        package = write_evidence_package(Path(directory), plan.cells[0], plan, ledger, result_changes={field: value})
        assert_code(code, authenticate_slice_7g_cell_evidence_package, package, charter, ledger, plan)


def test_preflight_only_ledger_cannot_authenticate_evidence():
    charter, committed, plan = committed_context()
    allocated = allocated_ledger(charter)
    with tempfile.TemporaryDirectory() as directory:
        package = write_evidence_package(Path(directory), plan.cells[0], plan, committed)
        assert_code("evidence_process_start_uncommitted", authenticate_slice_7g_cell_evidence_package, package, charter, allocated, plan)


@pytest.mark.parametrize(
    ("root", "code"),
    [(object(), "evidence_root_type"), ("/definitely/missing/slice-7g-package", "evidence_root_open"), ("bad\x00path", "evidence_root_open")],
)
def test_evidence_root_public_errors_are_normalized(root, code):
    charter, ledger, plan = committed_context()
    assert_code(code, authenticate_slice_7g_cell_evidence_package, root, charter, ledger, plan)


@pytest.mark.parametrize("field", ["evidence_identity", "authenticated", "authorized"])
def test_caller_evidence_identity_and_authority_booleans_are_not_schema_fields(field):
    charter, ledger, plan = committed_context()
    with tempfile.TemporaryDirectory() as directory:
        package = write_evidence_package(Path(directory), plan.cells[0], plan, ledger)
        mutate_member_and_reseal(package, "cell_result", lambda data: data.__setitem__(field, True if field != "evidence_identity" else "0" * 64))
        assert_code("unknown_field", authenticate_slice_7g_cell_evidence_package, package, charter, ledger, plan)


def test_wrong_argv_and_output_root_are_rejected_after_physical_authentication():
    charter, ledger, plan = committed_context()
    with tempfile.TemporaryDirectory() as directory:
        cell = plan.cells[0]
        wrong_root = "/home/ankid/ctr_mppi_evidence/slice_7g/wrong-campaign"
        wrong_output = f"{wrong_root}/cells/{cell.cell_id}"
        wrong_argv = cell.argv[:-1] + (wrong_output,)
        package = write_evidence_package(
            Path(directory), cell, plan, ledger,
            result_changes={"campaign_output_root": wrong_root, "cell_output_path": wrong_output, "argv": wrong_argv},
        )
        assert_code("evidence_campaign_output_root_mismatch", authenticate_slice_7g_cell_evidence_package, package, charter, ledger, plan)


def test_wrong_argv_is_rejected_after_physical_authentication():
    charter, ledger, plan = committed_context()
    with tempfile.TemporaryDirectory() as directory:
        argv = list(plan.cells[0].argv)
        argv[1], argv[3] = argv[3], argv[1]
        package = write_evidence_package(
            Path(directory), plan.cells[0], plan, ledger, result_changes={"argv": tuple(argv)}
        )
        assert_code("evidence_argv_mismatch", authenticate_slice_7g_cell_evidence_package, package, charter, ledger, plan)


@pytest.mark.parametrize("kind", ["cell", "scenario", "seed"])
def test_relabelled_cell_scenario_or_seed_cannot_reuse_evidence(kind):
    charter, ledger, plan = committed_context()
    with tempfile.TemporaryDirectory() as directory:
        package = write_evidence_package(Path(directory), plan.cells[0], plan, ledger)

        def relabel(data):
            if kind in ("cell", "scenario"):
                replacement = plan.cells[5]
            else:
                replacement = plan.cells[1]
            data["cell_id"] = replacement.cell_id
            if kind == "scenario":
                data["scenario_id"] = replacement.scenario_id
                data["source_scenario_id"] = replacement.source_scenario_id
            if kind == "seed":
                data["seed"] = replacement.seed

        mutate_envelope(package, relabel)
        expected = "evidence_cell_id" if kind == "cell" else "evidence_cell_output_path_mismatch"
        assert_code(expected, authenticate_slice_7g_cell_evidence_package, package, charter, ledger, plan)


@pytest.mark.parametrize(
    ("mutation", "code"),
    [
        ("extra", "evidence_inventory_mismatch"),
        ("missing", "evidence_inventory_mismatch"),
        ("digest", "evidence_member_descriptor_mismatch"),
        ("size", "evidence_member_descriptor_mismatch"),
        ("projection", "evidence_projection_mismatch"),
        ("writable_root", "evidence_root_mode"),
        ("writable_member", "evidence_member_mode"),
        ("symlink", "evidence_member_open"),
        ("hardlink", "evidence_hardlink_alias"),
    ],
)
def test_physical_evidence_package_failures_reach_intended_stage(mutation, code):
    charter, ledger, plan = committed_context()
    with tempfile.TemporaryDirectory() as directory:
        package = write_evidence_package(Path(directory), plan.cells[0], plan, ledger)
        package.chmod(0o755)
        target = package / MANDATORY_EVIDENCE_ROLE_PATHS["tactile_trace"]
        if mutation == "extra":
            extra = package / "extra.json"
            extra.write_bytes(b"{}")
            extra.chmod(0o444)
            package.chmod(0o555)
        elif mutation == "missing":
            target.unlink()
            package.chmod(0o555)
        elif mutation == "digest":
            target.chmod(0o644)
            target.write_bytes(target.read_bytes() + b" ")
            target.chmod(0o444)
            package.chmod(0o555)
        elif mutation == "size":
            package.chmod(0o555)
            mutate_envelope(
                package,
                lambda data: next(item for item in data["members"] if item["role"] == "tactile_trace").__setitem__("size", 1),
            )
        elif mutation == "projection":
            projection = package / "evidence_projection.json"
            projection.chmod(0o644)
            projection.write_bytes(projection.read_bytes() + b" ")
            projection.chmod(0o444)
            package.chmod(0o555)
        elif mutation == "writable_root":
            pass
        elif mutation == "writable_member":
            target.chmod(0o644)
            package.chmod(0o555)
        elif mutation == "symlink":
            target.unlink()
            target.symlink_to("safety_trace.json")
            package.chmod(0o555)
        else:
            target.unlink()
            os.link(package / "safety_trace.json", target)
            package.chmod(0o555)
        assert_code(code, authenticate_slice_7g_cell_evidence_package, package, charter, ledger, plan)


def test_missing_and_duplicate_mandatory_roles_are_rejected_from_canonical_envelope():
    charter, ledger, plan = committed_context()
    for kind, code in (("missing", "evidence_role_set"), ("duplicate", "duplicate_evidence_role")):
        with tempfile.TemporaryDirectory() as directory:
            package = write_evidence_package(Path(directory), plan.cells[0], plan, ledger)
            if kind == "missing":
                mutate_envelope(package, lambda data: data["members"].pop())
            else:
                def duplicate(data):
                    data["members"][-1] = deepcopy(data["members"][0])
                mutate_envelope(package, duplicate)
            assert_code(code, authenticate_slice_7g_cell_evidence_package, package, charter, ledger, plan)


def test_campaign_aggregate_claims_must_equal_authenticated_recomputation():
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, _, _, _):
        recomputed = reconcile_slice_7g_campaign_results(charter, plan, ledger, campaign_root)
        assert_code(
            "campaign_result_mismatch", reconcile_slice_7g_campaign_results,
            charter, plan, ledger, campaign_root, replace(recomputed, total_collision_samples=999),
        )
        assert_code(
            "campaign_result_mismatch", reconcile_slice_7g_campaign_results,
            charter, plan, ledger, campaign_root, replace(recomputed, total_valid_aligned_samples=1),
        )
        identities = list(recomputed.result_identities)
        identities[0] = "0" * 64
        assert_code(
            "campaign_result_mismatch", reconcile_slice_7g_campaign_results,
            charter, plan, ledger, campaign_root, replace(recomputed, result_identities=tuple(identities)),
        )
        assert_code(
            "campaign_result_mismatch", reconcile_slice_7g_campaign_results,
            charter, plan, ledger, campaign_root,
            replace(recomputed, campaign_evidence_snapshot_identity="0" * 64),
        )


def test_standalone_result_objects_cannot_enter_authoritative_reconciler():
    charter, ledger, plan = committed_context()
    results = [passing_result(cell, plan, ledger) for cell in plan.cells]
    assert_code("campaign_root_type", reconcile_slice_7g_campaign_results, charter, plan, ledger, results)


def test_evidence_package_descriptors_close_on_success_and_failure():
    charter, ledger, plan = committed_context()
    with tempfile.TemporaryDirectory() as directory:
        package = write_evidence_package(Path(directory), plan.cells[0], plan, ledger)
        before = len(os.listdir("/proc/self/fd"))
        authenticate_slice_7g_cell_evidence_package(package, charter, ledger, plan)
        assert len(os.listdir("/proc/self/fd")) == before
        package.chmod(0o755)
        (package / "tactile_trace.json").unlink()
        package.chmod(0o555)
        assert_code("evidence_inventory_mismatch", authenticate_slice_7g_cell_evidence_package, package, charter, ledger, plan)
        assert len(os.listdir("/proc/self/fd")) == before


@pytest.mark.parametrize(
    "mutation",
    [
        "rename_away",
        "byte_identical_root_replacement",
        "changed_root_replacement",
        "root_symlink_replacement",
        "root_mode_change",
        "file_added",
        "file_removed",
        "member_inode_replacement",
    ],
)
def test_individual_package_final_barrier_rejects_late_root_and_member_changes(monkeypatch, mutation):
    charter, ledger, plan = committed_context()
    with tempfile.TemporaryDirectory() as directory:
        package = write_evidence_package(Path(directory), plan.cells[0], plan, ledger)
        original = governance._derive_authenticated_evidence

        def mutate_after_semantics(state, *args):
            result = original(state, *args)
            retained = package.with_name(package.name + ".retained")
            if mutation in {"rename_away", "byte_identical_root_replacement", "changed_root_replacement", "root_symlink_replacement"}:
                package.rename(retained)
            if mutation == "byte_identical_root_replacement":
                clone_sealed_package(retained, package)
            elif mutation == "changed_root_replacement":
                clone_sealed_package(retained, package)
                target = package / "tactile_trace.json"
                package.chmod(0o755)
                target.chmod(0o644)
                target.write_bytes(target.read_bytes() + b" ")
                target.chmod(0o444)
                package.chmod(0o555)
            elif mutation == "root_symlink_replacement":
                package.symlink_to(retained, target_is_directory=True)
            elif mutation == "root_mode_change":
                package.chmod(0o755)
            elif mutation == "file_added":
                package.chmod(0o755)
                extra = package / "late.json"
                extra.write_bytes(b"{}")
                extra.chmod(0o444)
                package.chmod(0o555)
            elif mutation == "file_removed":
                package.chmod(0o755)
                (package / "tactile_trace.json").unlink()
                package.chmod(0o555)
            elif mutation == "member_inode_replacement":
                package.chmod(0o755)
                target = package / "tactile_trace.json"
                raw = target.read_bytes()
                target.unlink()
                target.write_bytes(raw)
                target.chmod(0o444)
                package.chmod(0o555)
            return result

        before = len(os.listdir("/proc/self/fd"))
        monkeypatch.setattr(governance, "_derive_authenticated_evidence", mutate_after_semantics)
        assert_code("evidence_late_change", authenticate_slice_7g_cell_evidence_package, package, charter, ledger, plan)
        assert len(os.listdir("/proc/self/fd")) == before


def test_individual_package_final_barrier_rejects_parent_component_substitution(monkeypatch):
    charter, ledger, plan = committed_context()
    with tempfile.TemporaryDirectory() as directory:
        outer = Path(directory)
        live_parent = outer / "live"
        live_parent.mkdir()
        package = write_evidence_package(live_parent, plan.cells[0], plan, ledger)
        original = governance._derive_authenticated_evidence

        def replace_parent_after_semantics(state, *args):
            result = original(state, *args)
            moved = outer / "moved"
            live_parent.rename(moved)
            live_parent.mkdir()
            clone_sealed_package(moved / package.name, live_parent / package.name)
            return result

        monkeypatch.setattr(governance, "_derive_authenticated_evidence", replace_parent_after_semantics)
        assert_code("evidence_late_change", authenticate_slice_7g_cell_evidence_package, package, charter, ledger, plan)


def test_individual_package_final_barrier_unchanged_control_and_verified_path():
    charter, ledger, plan = committed_context()
    with tempfile.TemporaryDirectory() as directory:
        package = write_evidence_package(Path(directory), plan.cells[0], plan, ledger)
        observed = authenticate_slice_7g_cell_evidence_package(package, charter, ledger, plan)
        current = package.stat(follow_symlinks=False)
        assert (observed.root_device, observed.root_inode) == (current.st_dev, current.st_ino)


def test_evidence_root_parent_symlink_is_rejected_component_by_component():
    charter, ledger, plan = committed_context()
    with tempfile.TemporaryDirectory() as directory:
        outer = Path(directory)
        real_parent = outer / "real"
        real_parent.mkdir()
        package = write_evidence_package(real_parent, plan.cells[0], plan, ledger)
        link = outer / "link"
        link.symlink_to(real_parent, target_is_directory=True)
        assert_code(
            "evidence_root_open", authenticate_slice_7g_cell_evidence_package,
            link / package.name, charter, ledger, plan,
        )


@pytest.mark.parametrize(
    "mutation",
    ["early_content", "early_byte_identical_root", "early_add", "early_remove", "early_parent_substitution"],
)
def test_campaign_final_barrier_rejects_early_package_mutation_while_later_packages_authenticate(
    monkeypatch, mutation
):
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, _, packages, _):
        original = governance._open_campaign_package_authority
        calls = 0

        def mutate_before_fifteenth(campaign, package_record, *args):
            nonlocal calls
            calls += 1
            if calls == 15:
                early = packages[0]
                if mutation == "early_content":
                    early.chmod(0o755)
                    target = early / "tactile_trace.json"
                    target.chmod(0o644)
                    target.write_bytes(target.read_bytes() + b" ")
                    target.chmod(0o444)
                    early.chmod(0o555)
                elif mutation == "early_byte_identical_root":
                    early.parent.chmod(0o755)
                    retained = early.with_name(early.name + ".retained")
                    early.rename(retained)
                    clone_sealed_package(retained, early)
                    early.parent.chmod(0o555)
                elif mutation == "early_add":
                    early.chmod(0o755)
                    extra = early / "late.json"
                    extra.write_bytes(b"{}")
                    extra.chmod(0o444)
                    early.chmod(0o555)
                elif mutation == "early_remove":
                    early.chmod(0o755)
                    (early / "tactile_trace.json").unlink()
                    early.chmod(0o555)
                else:
                    packages_root = early.parent
                    packages_root.parent.chmod(0o755)
                    moved = packages_root.with_name("packages.retained")
                    packages_root.rename(moved)
                    packages_root.mkdir()
                    for retained in moved.iterdir():
                        clone_sealed_package(retained, packages_root / retained.name)
                    packages_root.chmod(0o555)
                    packages_root.parent.chmod(0o555)
            return original(campaign, package_record, *args)

        before = len(os.listdir("/proc/self/fd"))
        monkeypatch.setattr(governance, "_open_campaign_package_authority", mutate_before_fifteenth)
        assert_code(
            "evidence_late_change", reconcile_slice_7g_campaign_results,
            charter, plan, ledger, campaign_root,
        )
        assert calls == 15
        assert len(os.listdir("/proc/self/fd")) == before


def test_campaign_final_barrier_unchanged_fifteen_package_control():
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, _, _, _):
        before = len(os.listdir("/proc/self/fd"))
        result = reconcile_slice_7g_campaign_results(charter, plan, ledger, campaign_root)
        assert result.total_result_count == 15
        assert result.functional_promotion_pass is True
        assert len(os.listdir("/proc/self/fd")) == before


def test_all_public_path_boundaries_normalize_hostile_pathlikes_once_without_raw_exceptions():
    class CustomPathError(Exception):
        pass

    behaviors = [
        RuntimeError("boom"), TypeError("boom"), ValueError("boom"), OSError("boom"), CustomPathError("boom"),
        7, b"/tmp/bytes", "bad\x00path", "../escape", lambda call: "/first" if call == 1 else "/second",
    ]
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, _, _, _):
        for index, behavior in enumerate(behaviors):
            for kind in ("charter", "snapshot", "package", "campaign"):
                hostile = HostilePathLike(behavior)
                if kind == "charter":
                    call = lambda: load_slice_7g_charter(hostile)
                    expected = "charter_path_type" if index <= 6 else "charter_open"
                elif kind == "snapshot":
                    call = lambda: verify_authoring_source_snapshot(charter, hostile)
                    expected = "snapshot_root_type" if index <= 6 else (
                        "snapshot_root_missing" if index == 9 else "snapshot_root_open"
                    )
                elif kind == "package":
                    call = lambda: authenticate_slice_7g_cell_evidence_package(hostile, charter, ledger, plan)
                    expected = "evidence_root_type" if index <= 6 else "evidence_root_open"
                else:
                    call = lambda: reconcile_slice_7g_campaign_results(charter, plan, ledger, hostile)
                    expected = "campaign_root_type" if index <= 6 else (
                        "campaign_root_ledger_mismatch" if index == 9 else "campaign_root_open"
                    )
                assert_code(expected, call)
                assert hostile.calls == 1


def test_public_path_boundaries_accept_pathlib_and_call_custom_pathlike_once():
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, _, packages, _):
        root = HostilePathLike(str(packages[0]))
        observed = authenticate_slice_7g_cell_evidence_package(root, charter, ledger, plan)
        assert observed.cell_result.cell_id == plan.cells[0].cell_id
        assert root.calls == 1
        campaign_path = HostilePathLike(str(campaign_root))
        result = reconcile_slice_7g_campaign_results(charter, plan, ledger, campaign_path)
        assert result.total_result_count == 15
        assert campaign_path.calls == 1


def test_public_path_boundary_does_not_mask_base_exceptions():
    hostile = HostilePathLike(KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        load_slice_7g_charter(hostile)
    assert hostile.calls == 1


def test_campaign_performs_no_evidence_reads_after_global_final_barrier(monkeypatch):
    with sealed_campaign() as (charter, ledger, plan, campaign_root, _, _, _, _):
        original_capture = governance._capture_evidence_observations
        original_cross = governance._validate_cross_package_authority
        global_barrier_complete = False

        def observe_capture(root_fd):
            assert global_barrier_complete is False
            return original_capture(root_fd)

        def observe_cross(states, *, final):
            nonlocal global_barrier_complete
            result = original_cross(states, final=final)
            if final:
                global_barrier_complete = True
            return result

        monkeypatch.setattr(governance, "_capture_evidence_observations", observe_capture)
        monkeypatch.setattr(governance, "_validate_cross_package_authority", observe_cross)
        result = reconcile_slice_7g_campaign_results(charter, plan, ledger, campaign_root)
        assert result.total_result_count == 15
        assert global_barrier_complete is True


def test_evidence_authority_cleanup_emits_no_unraisable_exceptions(monkeypatch):
    charter, ledger, plan = committed_context()
    unraisable = []
    monkeypatch.setattr(sys, "unraisablehook", unraisable.append)
    with tempfile.TemporaryDirectory() as directory:
        package = write_evidence_package(Path(directory), plan.cells[0], plan, ledger)
        authenticate_slice_7g_cell_evidence_package(package, charter, ledger, plan)
    gc.collect()
    assert unraisable == []


def test_member_replacement_observation_is_rejected(monkeypatch):
    charter, ledger, plan = committed_context()
    with tempfile.TemporaryDirectory() as directory:
        package = write_evidence_package(Path(directory), plan.cells[0], plan, ledger)
        real_stat = os.stat

        def replaced_stat(path, *args, **kwargs):
            observed = real_stat(path, *args, **kwargs)
            if path == "tactile_trace.json" and kwargs.get("dir_fd") is not None:
                fields = list(observed)
                fields[1] += 1
                return os.stat_result(fields)
            return observed

        monkeypatch.setattr(governance.os, "stat", replaced_stat)
        assert_code("evidence_member_replaced", authenticate_slice_7g_cell_evidence_package, package, charter, ledger, plan)


@pytest.mark.parametrize(
    ("changes", "reason"),
    [
        ({"process_exit_status": 1}, "process_exit_status"),
        ({"readiness_success": False}, "readiness_success"),
        ({"stable_sample_count": 9}, "stable_sample_count"),
        ({"stable_interval_seconds": 0.49}, "stable_interval"),
        ({"q_variation": 5.1e-5}, "q_variation"),
        ({"tip_variation_m": 5.1e-5}, "tip_variation"),
        ({"valid_aligned_sample_count": 19}, "valid_aligned_sample_count"),
        ({"invalid_sample_count": 3, "invalid_sample_percentage": 100.0 * 3 / 23}, "invalid_sample_percentage"),
        ({"steady_state_error_m": 0.0031}, "steady_state_error"),
        ({"final_goal_error_m": 0.0031}, "final_goal_error"),
        ({"goal_hold_duration_seconds": 0.49}, "goal_hold_duration"),
        ({"minimum_physical_wall_clearance_m": -0.0001}, "minimum_physical_wall_clearance"),
        ({"minimum_safety_margin_wall_clearance_m": 0.0019}, "minimum_safety_margin_wall_clearance"),
        ({"collision_sample_count": 1}, "collision_sample_count"),
        ({"safety_fault_count": 1}, "safety_fault_count"),
        ({"nonfinite_value_count": 1}, "nonfinite_value_count"),
        ({"missing_required_topic_count": 1}, "missing_required_topic_count"),
        ({"missing_required_result_file_count": 1}, "missing_required_result_file_count"),
        ({"saturation_percentage": 1.1}, "saturation_percentage"),
    ],
)
def test_each_promotion_blocking_result_family_fails_campaign(changes, reason):
    with sealed_campaign(first_changes=changes) as (charter, ledger, plan, campaign_root, _, _, _, _):
        aggregate = reconcile_slice_7g_campaign_results(charter, plan, ledger, campaign_root)
    assert aggregate.functional_promotion_pass is False
    assert any(item.endswith(":" + reason) for item in aggregate.functional_failure_reasons)


def test_diagnostic_only_timing_failure_preserves_functional_promotion():
    with sealed_campaign(first_changes={
        "deadline_overrun_percentage": 5.1, "timing_pass": False, "non_real_time_label": True,
    }) as (charter, ledger, plan, campaign_root, _, _, _, _):
        aggregate = reconcile_slice_7g_campaign_results(charter, plan, ledger, campaign_root)
    assert aggregate.functional_promotion_pass is True
    assert aggregate.timing_all_pass is False
    assert aggregate.non_real_time_limitation_required is True
    assert aggregate.timing_failure_cell_count == 1
    assert aggregate.total_valid_aligned_samples == 300
    assert aggregate.total_collision_samples == 0


def test_timing_failure_without_non_real_time_label_rejected():
    _, ledger, plan = committed_context()
    assert_code(
        "missing_non_real_time_label",
        replace,
        passing_result(plan.cells[0], plan, ledger),
        deadline_overrun_percentage=5.1,
        timing_pass=False,
        non_real_time_label=False,
    )


@pytest.mark.parametrize("mutation", ["missing", "extra", "duplicate"])
def test_campaign_result_bijection_rejects_missing_extra_and_duplicate(mutation):
    with sealed_campaign() as (charter, ledger, plan, campaign_root, evidence_root, seal_path, packages, seal_data):
        evidence_root.chmod(0o755)
        seal_path.chmod(0o644)
        if mutation == "missing":
            seal_data["packages"].pop()
        elif mutation == "extra":
            seal_data["packages"].append(deepcopy(seal_data["packages"][-1]))
        else:
            seal_data["packages"][1] = deepcopy(seal_data["packages"][0])
        seal_path.write_bytes(canonical(seal_data))
        seal_path.chmod(0o444)
        evidence_root.chmod(0o555)
        expected = {
            "missing": "campaign_evidence_seal_count",
            "extra": "campaign_evidence_seal_count",
            "duplicate": "campaign_evidence_seal_duplicate_cell",
        }[mutation]
        assert_code(expected, reconcile_slice_7g_campaign_results, charter, plan, ledger, campaign_root)


def test_result_record_forgery_is_revalidated():
    _, ledger, plan = committed_context()
    valid = passing_result(plan.cells[0], plan, ledger)
    forged = object.__new__(Slice7GCellResult)
    for name in valid.__dataclass_fields__:
        object.__setattr__(forged, name, getattr(valid, name))
    object.__setattr__(forged, "seed", True)
    assert_code("result_seed", slice_7g_cell_result_identity, forged)


def test_documentation_and_json_revised_policy_agree():
    data = charter_data()
    document = DOC_PATH.read_text(encoding="utf-8")
    assert data["schema_version"] == "ctr-slice-7g-charter-7"
    assert "ctr7g-campaign" in document
    assert "global" in document.lower()
    assert "exactly 15" in document
    assert "atomic compare-and-swap" in document
    assert "single ledger-bound ROS domain" in document
    assert "campaign-wide result reconciler" in document
    assert "does not yet orchestrate" in document
    assert "diagnostic only" in document
    assert "physical evidence package" in document
    assert "authority booleans" in document
    assert "15 authenticated packages" in document
    assert "nonblocking shared" in document
    assert "exclusive lock" in document
    assert "do not claim a mathematically atomic filesystem instant" in document
    assert data["cell_evidence_contract"]["caller_evidence_identity_trusted"] is False
    assert data["cell_evidence_contract"]["caller_authority_booleans_trusted"] is False
    assert set(data["cell_evidence_contract"]["mandatory_roles"]) == set(MANDATORY_EVIDENCE_ROLE_PATHS)
    assert data["campaign_result_contract"]["aggregates_recomputed_from_authenticated_packages"] is True
    assert data["campaign_evidence_seal_contract"]["schema_version"] == CAMPAIGN_EVIDENCE_SEAL_SCHEMA_VERSION
    assert data["campaign_evidence_seal_contract"]["sequential_rehash_is_atomic_snapshot"] is False
    assert data["campaign_evidence_seal_contract"]["exclusive_writer_lock_required"] is True
    assert len(data["implementation_requirements"]) == 13


def test_production_module_has_standard_library_imports_and_no_write_or_runtime_symbols():
    tree = ast.parse(MODULE_PATH.read_text(encoding="utf-8"), filename=str(MODULE_PATH))
    imported_roots = set()
    called_attributes = set()
    called_names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_roots.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_roots.add(node.module.split(".")[0])
        elif isinstance(node, ast.Call):
            if isinstance(node.func, ast.Attribute):
                called_attributes.add(node.func.attr)
            elif isinstance(node.func, ast.Name):
                called_names.add(node.func.id)
    assert imported_roots <= {
        "__future__", "collections", "dataclasses", "datetime", "errno", "fcntl", "hashlib", "json", "math", "os", "pathlib", "re", "stat", "types", "typing", "unicodedata"
    }
    assert not ({"write", "write_text", "write_bytes", "mkdir", "chmod", "putenv", "system", "popen"} & called_attributes)
    assert not ({"exec", "eval", "compile"} & called_names)


def test_validation_has_no_environment_or_repository_side_effects():
    environment_before = dict(os.environ)
    charter_before = CHARTER_PATH.read_bytes()
    module_before = MODULE_PATH.read_bytes()
    record = load_slice_7g_charter(CHARTER_PATH)
    assert slice_7g_charter_identity(record)
    assert dict(os.environ) == environment_before
    assert CHARTER_PATH.read_bytes() == charter_before
    assert MODULE_PATH.read_bytes() == module_before


def test_all_public_malformed_input_boundaries_use_governance_errors():
    charter, ledger, plan = plan_and_ledger()
    calls = [
        lambda: load_slice_7g_charter(object()),
        lambda: validate_slice_7g_charter(object()),
        lambda: canonical_slice_7g_charter_bytes(object()),
        lambda: slice_7g_charter_identity(object()),
        lambda: validate_slice_7g_attempt_budget(object()),
        lambda: validate_slice_7g_domain_policy(object()),
        lambda: validate_slice_7g_scenario(object()),
        lambda: validate_slice_7g_metric(object()),
        lambda: canonical_slice_7g_attempt_ledger_bytes(object()),
        lambda: canonical_slice_7g_attempt_event_bytes(object()),
        lambda: create_slice_7g_initial_attempt_ledger(object(), "campaign-001"),
        lambda: propose_slice_7g_attempt_event(object(), "x", "x", "2026-08-19T00:00:00Z"),
        lambda: validate_slice_7g_attempt_transition(object(), object()),
        lambda: generate_slice_7g_campaign_plan(charter, object()),
        lambda: validate_slice_7g_campaign_plan(object(), charter, ledger),
        lambda: canonical_slice_7g_campaign_plan_bytes(object()),
        lambda: canonical_slice_7g_cell_result_bytes(object()),
        lambda: validate_slice_7g_campaign_evidence_seal(object()),
        lambda: canonical_slice_7g_campaign_evidence_seal_bytes(object()),
        lambda: slice_7g_campaign_evidence_snapshot_identity(object()),
        lambda: reconcile_slice_7g_campaign_results(charter, plan, ledger, object()),
        lambda: canonical_slice_7g_campaign_result_bytes(object()),
        lambda: verify_authoring_source_snapshot(charter, object()),
    ]
    leaks = []
    for call in calls:
        try:
            call()
        except Slice7GGovernanceError:
            continue
        except BaseException as exc:  # probe classification, not production masking
            leaks.append(type(exc).__name__)
        else:
            leaks.append("NO_ERROR")
    assert leaks == []


@pytest.mark.parametrize("attack", ["__iter__", "__getitem__", "items", "keys", "__len__"])
def test_every_public_mapping_boundary_rejects_hostile_mapping_without_calling_hooks(attack):
    hostile = HostileMapping(attack)
    calls = [
        lambda: validate_slice_7g_charter(hostile),
        lambda: canonical_slice_7g_charter_bytes(hostile),
        lambda: slice_7g_charter_identity(hostile),
        lambda: validate_slice_7g_attempt_budget(hostile),
        lambda: validate_slice_7g_domain_policy(hostile),
        lambda: validate_slice_7g_scenario(hostile),
        lambda: validate_slice_7g_metric(hostile),
        lambda: canonical_slice_7g_attempt_ledger_bytes(hostile),
        lambda: slice_7g_attempt_ledger_identity(hostile),
        lambda: canonical_slice_7g_attempt_event_bytes(hostile),
        lambda: slice_7g_attempt_event_identity(hostile),
        lambda: create_slice_7g_initial_attempt_ledger(hostile, "campaign-001"),
        lambda: propose_slice_7g_attempt_event(hostile, "x", "x", "2026-08-19T00:00:00Z"),
        lambda: validate_slice_7g_attempt_transition(hostile, hostile),
        lambda: slice_7g_metric_profile_identity(hostile),
        lambda: generate_slice_7g_campaign_plan(hostile, hostile),
        lambda: validate_slice_7g_campaign_plan(hostile, hostile, hostile),
        lambda: canonical_slice_7g_campaign_plan_bytes(hostile),
        lambda: slice_7g_campaign_plan_identity(hostile),
        lambda: canonical_slice_7g_cell_result_bytes(hostile),
        lambda: slice_7g_cell_result_identity(hostile),
        lambda: validate_slice_7g_campaign_evidence_seal(hostile),
        lambda: canonical_slice_7g_campaign_evidence_seal_bytes(hostile),
        lambda: slice_7g_campaign_evidence_snapshot_identity(hostile),
        lambda: authenticate_slice_7g_cell_evidence_package(".", hostile, hostile, hostile),
        lambda: reconcile_slice_7g_campaign_results(hostile, hostile, hostile, []),
        lambda: canonical_slice_7g_campaign_result_bytes(hostile),
        lambda: slice_7g_campaign_result_identity(hostile),
        lambda: verify_authoring_source_snapshot(hostile, "."),
    ]
    for call in calls:
        with pytest.raises(Slice7GGovernanceError):
            call()
    assert hostile.calls == []


def test_hostile_nested_mapping_is_rejected_without_invoking_hooks_or_creating_identity():
    hostile = HostileMapping("items")
    data = charter_data()
    data["authoring"] = hostile
    assert_code("scalar_type", validate_slice_7g_charter, data)
    assert hostile.calls == []


def test_dict_subclasses_are_not_accepted_at_public_primitive_boundaries():
    class DictSubclass(dict):
        def items(self):
            raise RuntimeError("must not run")

    assert_code("charter_type", validate_slice_7g_charter, DictSubclass(charter_data()))
    assert_code("attempt_budget_type", validate_slice_7g_attempt_budget, DictSubclass())


@pytest.mark.parametrize("mapping_type", [InconsistentMapping, MutatingMapping])
def test_inconsistent_and_mutating_mappings_are_rejected_without_access(mapping_type):
    hostile = mapping_type()
    assert_code("charter_type", validate_slice_7g_charter, hostile)
    assert hostile.calls == 0
