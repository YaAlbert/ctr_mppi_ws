"""Strict, non-executable governance contract for Slice 7G.

The module owns the ``ctr-slice-7g-charter-7`` schema.  It deliberately has no
ROS, subprocess, network, environment-mutation, allocation, or file-writing
capability.  The charter logical identity is::

    SHA256(b"ctr-slice-7g-charter-canonical-7\\0" + canonical_charter_bytes)

Canonical bytes are compact UTF-8 JSON with recursively sorted keys,
``ensure_ascii=False``, no non-finite values, and no trailing newline.  The
authoring snapshot is acyclic because it covers only pre-charter source facts;
the charter, this module, its tests, and downstream runtime evidence are not
snapshot members.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import errno
import fcntl
import hashlib
import json
import math
import os
import re
import stat
from types import MappingProxyType
from typing import Any
import unicodedata


SCHEMA_VERSION = "ctr-slice-7g-charter-7"
CHARTER_IDENTITY_ALGORITHM = "sha256:ctr-slice-7g-charter-canonical-7"
CHARTER_IDENTITY_DOMAIN = b"ctr-slice-7g-charter-canonical-7\0"
HISTORICAL_SCHEMA_VERSION = "ctr-slice-7g-charter-5"
AUTHORITY_OUTPUT_PARENT = "/home/ankid/ctr_mppi_evidence/slice_7g"
HISTORICAL_CHARTER_IDENTITY_DOMAIN = b"ctr-slice-7g-charter-canonical-5\0"
RUNTIME_AUTHORITY_CONTRACT_SCHEMA_VERSION = "ctr-slice-7g-runtime-authority-contract-4"
SNAPSHOT_SCHEMA_VERSION = "ctr-scoped-source-snapshot-1"
SNAPSHOT_IDENTITY_ALGORITHM = "sha256:ctr-slice-7g-authoring-source-snapshot-1"
SNAPSHOT_IDENTITY_DOMAIN = b"ctr-slice-7g-authoring-source-snapshot-1\0"
ATTEMPT_LEDGER_SCHEMA_VERSION = "ctr-slice-7g-attempt-ledger-1"
ATTEMPT_EVENT_SCHEMA_VERSION = "ctr-slice-7g-attempt-event-1"
CAMPAIGN_PLAN_SCHEMA_VERSION = "ctr-slice-7g-campaign-plan-1"
CAMPAIGN_CELL_SCHEMA_VERSION = "ctr-slice-7g-campaign-cell-1"
CELL_RESULT_SCHEMA_VERSION = "ctr-slice-7g-cell-result-2"
CAMPAIGN_RESULT_SCHEMA_VERSION = "ctr-slice-7g-campaign-result-3"
CAMPAIGN_EVIDENCE_SEAL_SCHEMA_VERSION = "ctr-slice-7g-campaign-evidence-seal-1"
CAMPAIGN_EVIDENCE_PACKAGE_RECORD_SCHEMA_VERSION = "ctr-slice-7g-campaign-evidence-package-record-1"
CAMPAIGN_EVIDENCE_SNAPSHOT_SCHEMA_VERSION = "ctr-slice-7g-campaign-evidence-snapshot-1"
CAMPAIGN_EVIDENCE_SNAPSHOT_IDENTITY_ALGORITHM = "sha256:ctr-slice-7g-campaign-evidence-snapshot-canonical-1"
CELL_EVIDENCE_PROJECTION_SCHEMA_VERSION = "ctr-slice-7g-cell-evidence-projection-1"
CELL_EVIDENCE_ENVELOPE_SCHEMA_VERSION = "ctr-slice-7g-cell-evidence-envelope-1"
CELL_EVIDENCE_MEMBER_SCHEMA_VERSION = "ctr-slice-7g-cell-evidence-member-1"
CELL_EVIDENCE_PROJECTION_IDENTITY_ALGORITHM = "sha256:ctr-slice-7g-cell-evidence-projection-canonical-1"
CELL_EVIDENCE_PACKAGE_IDENTITY_ALGORITHM = "sha256:ctr-slice-7g-cell-evidence-package-physical-1"
METRIC_PROFILE_SCHEMA_VERSION = "ctr-slice-7g-metric-profile-1"
LEDGER_IDENTITY_DOMAIN = b"ctr-slice-7g-attempt-ledger-canonical-1\0"
EVENT_IDENTITY_DOMAIN = b"ctr-slice-7g-attempt-event-canonical-1\0"
CAMPAIGN_IDENTITY_DOMAIN = b"ctr-slice-7g-campaign-1\0"
CAMPAIGN_PLAN_IDENTITY_DOMAIN = b"ctr-slice-7g-campaign-plan-canonical-1\0"
CELL_RESULT_IDENTITY_DOMAIN = b"ctr-slice-7g-cell-result-canonical-2\0"
CAMPAIGN_RESULT_IDENTITY_DOMAIN = b"ctr-slice-7g-campaign-result-canonical-3\0"
CAMPAIGN_EVIDENCE_SNAPSHOT_IDENTITY_DOMAIN = b"ctr-slice-7g-campaign-evidence-snapshot-canonical-1\0"
CELL_EVIDENCE_PROJECTION_IDENTITY_DOMAIN = b"ctr-slice-7g-cell-evidence-projection-canonical-1\0"
CELL_EVIDENCE_PACKAGE_IDENTITY_DOMAIN = b"ctr-slice-7g-cell-evidence-package-physical-1\0"
METRIC_PROFILE_IDENTITY_DOMAIN = b"ctr-slice-7g-metric-profile-canonical-1\0"
EXPECTED_BRANCH = "milestone/06b-curved-lumen-sim"
EXPECTED_HEAD = "8b8249dd62313faa63ba6380eb70145050331b39"
EXPECTED_SLICE_7F_CLOSURE = (
    "/home/ankid/ctr_mppi_evidence/slice_7f/"
    "ctr_m7f_final_candidate_v3.20260818T142150Z_evidence_closure.json"
)
EXPECTED_SLICE_7F_SHA256 = "b71e65e9570177bad86eb5e4ebb2306430668b17ea266ea457eb840afc8112fe"
EXPECTED_SOURCE_SCENARIOS = {
    "centerline": "centerline_target",
    "lateral_offset": "lateral_offset_target",
    "near_safety_boundary": "near_safety_boundary_target",
}
EXPECTED_SEEDS = (11, 22, 33, 44, 55)
EXPECTED_IMPLEMENTATION_GATES = frozenset(
    {
        "START_AND_VERIFY_SAFETY_SUPERVISOR",
        "ENABLE_SIMULATED_TACTILE_INPUT",
        "ENABLE_TACTILE_COST",
        "ENABLE_SAFETY_TACTILE_HANDLING",
        "ROUTE_CONTROLLER_THROUGH_SAFETY_SUPERVISOR",
        "AUTHENTICATE_TACTILE_READINESS",
        "AUTHENTICATE_SAFETY_READINESS_AND_FAULT_STATE",
        "IMPLEMENT_COLLISION_AWARE_DOMAIN_ALLOCATION",
        "PROVE_SELECTED_PROFILE_AVOIDS_UNFINISHED_COSTS",
        "CREATE_POST_IMPLEMENTATION_SOURCE_SNAPSHOT",
        "GENERATE_IMMUTABLE_CAMPAIGN_PLAN",
        "RECONCILE_CAMPAIGN_RESULTS",
        "PROPAGATE_SINGLE_LEDGER_BOUND_DOMAIN",
    }
)
EXPECTED_ENTRY_GATES = frozenset(
    {
        "SLICE_7F_CLOSED_UNCHANGED",
        "CHARTER_INDEPENDENTLY_REVIEWED",
        "IMPLEMENTATION_COMPLETE",
        "PARAMETERS_COMMITTED_AND_AUTHENTICATED",
        "NO_REACHABLE_NOT_IMPLEMENTED_ERROR",
        "POST_IMPLEMENTATION_SOURCE_SNAPSHOT_CREATED",
        "ISOLATED_BUILD_PASSED",
        "COMPLETE_TEST_MATRIX_PASSED",
        "RUNTIME_PLAN_AND_ARGV_VALIDATED",
        "OUTPUT_ROOT_NEW_AND_EMPTY",
        "FRESH_DOMAIN_RESERVED_BY_POLICY",
        "ATTEMPT_REMAINS_ZERO_OF_ONE",
        "SEPARATE_RUNTIME_AUTHORIZATION_ISSUED",
    }
)
EXPECTED_PROMOTION_GATES = frozenset(
    {
        "IMPLEMENTATION_REVIEW_PASSED",
        "BUILD_AND_STATIC_VERIFICATION_PASSED",
        "EXACTLY_ONE_AUTHORIZED_CAMPAIGN_COMPLETED",
        "CAMPAIGN_PLAN_AND_RESULTS_RECONCILED",
        "EVERY_REQUIRED_RUN_CELL_PASSED",
        "INDEPENDENT_READ_ONLY_AUDIT_APPROVED",
        "EXTERNAL_IMMUTABLE_PROMOTION_RECORD_CREATED",
        "FINAL_SIMULATION_PROJECT_CLOSURE_CREATED",
    }
)
EXPECTED_PROMOTION_LIMITATIONS = frozenset(
    {
        "SIMULATION_ONLY_COMPLETION",
        "NO_PHYSICAL_HARDWARE_READINESS_CLAIM",
        "NO_REAL_TIME_PERFORMANCE_CLAIM",
        "NO_CLINICAL_CLAIM",
        "PHYSICAL_DEPLOYMENT_DEFERRED",
    }
)
EXPECTED_SCOPE_INCLUDED = frozenset(
    {
        "Circular-arc curved-lumen geometry and target generation",
        "Curved-lumen MPPI integration",
        "Simulated tactile generation",
        "Tactile processing and tactile cost integration",
        "Safety-supervisor state, gating, and latching behavior",
        "Readiness monitoring",
        "Deterministic evaluation orchestration",
        "Clean isolated build and complete static tests",
        "One governed simulation-acceptance campaign",
        "Independent acceptance audit",
        "External immutable promotion decision",
        "Final simulation-project closure",
    }
)
EXPECTED_SCOPE_EXCLUDED = frozenset(
    {
        "Physical CTR drivers",
        "Calibrated physical tube parameters",
        "Physical tactile sensor calibration",
        "Hardware motor and control-board commissioning",
        "Physical safety certification",
        "Clinical use",
        "Autonomous clinical decision-making",
        "Shape, obstacle, and stability costs while disabled by the selected profile",
        "Nonideal actuator and mock-hardware milestones while excluded by the selected profile",
    }
)
EXPECTED_SCOPE_LIMITATIONS = frozenset(
    {
        "The endpoint is software simulation only.",
        "The charter is scope authority, not runtime authority.",
        "The authoring dirty worktree is not an acceptance subject.",
        "No real-time, physical-hardware, safety-certification, or clinical claim is permitted.",
    }
)
EXPECTED_READINESS_NODES = frozenset(
    {
        "parameter_validator",
        "ctr_simulator",
        "safety_supervisor",
        "mppi_controller",
        "reference_manager",
        "evaluation_node",
        "ctr_run_evaluation_monitor",
    }
)
EXPECTED_READINESS_TOPICS = frozenset(
    {
        "/ctr/state",
        "/ctr/tip",
        "/ctr/reference/tip",
        "/ctr/reference/path",
        "/ctr/mppi_command",
        "/ctr/safe_command",
        "/ctr/tactile/state",
        "/ctr/safety/status",
    }
)
EXPECTED_READINESS_SERVICES = frozenset({"/ctr/start_experiment", "/ctr/stop_experiment"})
EXPECTED_READINESS_CONDITIONS = frozenset(
    {
        "All expected processes and nodes are present.",
        "All required topics publish within the timeout.",
        "State and tip values are finite and correctly dimensioned.",
        "The minimum stable sample count and interval are satisfied.",
        "No state, command, tactile, or reference freshness condition is stale.",
        "No safety fault is active or latched at acceptance start.",
    }
)
EXPECTED_BUILD_COMMANDS = (
    "colcon --log-base <external_log> build --base-paths src --build-base <external_build> --install-base <external_install>",
    "colcon --log-base <external_test_log> test --build-base <external_build> --install-base <external_install>",
    "colcon test-result --test-result-base <external_build> --verbose",
    "/usr/bin/python3 -B -m pytest -p no:cacheprovider <authorized_test_paths>",
)
SAFE_UNITS = frozenset({"boolean", "m", "s", "count", "percent", "exit_code"})
EVIDENCE_ENVELOPE_PATH = "evidence_envelope.json"
EVIDENCE_PROJECTION_PATH = "evidence_projection.json"
CAMPAIGN_EVIDENCE_ROOT_RELATIVE_PATH = "evidence"
CAMPAIGN_EVIDENCE_PACKAGES_RELATIVE_PATH = "packages"
CAMPAIGN_EVIDENCE_SEAL_PATH = "campaign_evidence_seal.json"
SLICE_7G_EVIDENCE_PARENT = "/home/ankid/ctr_mppi_evidence/slice_7g"
MANDATORY_EVIDENCE_ROLE_PATHS = {
    "invocation_process_start_receipt": "invocation_process_start_receipt.json",
    "runtime_authorization_binding": "runtime_authorization_binding.json",
    "readiness_trace": "readiness_trace.json",
    "safety_trace": "safety_trace.json",
    "tactile_trace": "tactile_trace.json",
    "cell_result": "cell_result.json",
    "output_inventory_receipt": "output_inventory_receipt.json",
}


class Slice7GGovernanceError(ValueError):
    """Stable public error carrying a machine-readable contract code."""

    def __init__(self, code: str, message: str, *, path: str = "$") -> None:
        self.code = code
        self.path = path
        super().__init__(f"{code}:{path}:{message}")


@dataclass(frozen=True)
class HistoricalSlice7GCharterInspection:
    schema_version: str
    logical_identity: str
    canonical_bytes: bytes
    runtime_authoritative: bool = False


@dataclass(frozen=True)
class Slice7GAttemptBudget:
    maximum_campaigns: int
    consumed_campaigns: int
    retries_authorized: int

    def __post_init__(self) -> None:
        for name, value in (
            ("maximum_campaigns", self.maximum_campaigns),
            ("consumed_campaigns", self.consumed_campaigns),
            ("retries_authorized", self.retries_authorized),
        ):
            if type(value) is not int:
                _fail("attempt_type", f"{name} must be an exact integer", f"$.attempt_budget.{name}")
        if self.maximum_campaigns != 1:
            _fail("attempt_maximum", "maximum campaigns must equal one", "$.attempt_budget.maximum_campaigns")
        if self.consumed_campaigns not in (0, 1):
            _fail("attempt_count", "consumed campaigns must be zero or one", "$.attempt_budget.consumed_campaigns")
        if self.retries_authorized != 0:
            _fail("retry_count", "retries authorized must equal zero", "$.attempt_budget.retries_authorized")


@dataclass(frozen=True)
class Slice7GDomainPolicy:
    minimum_domain_id: int
    maximum_domain_id: int
    domain_allocated: bool
    selected_domain_id: int | None

    def __post_init__(self) -> None:
        if type(self.minimum_domain_id) is not int or type(self.maximum_domain_id) is not int:
            _fail("domain_type", "domain bounds must be exact integers", "$.domain_policy")
        if (self.minimum_domain_id, self.maximum_domain_id) != (100, 199):
            _fail("domain_range", "Slice 7G domain range must be 100 through 199", "$.domain_policy")
        if type(self.domain_allocated) is not bool:
            _fail("domain_type", "domain_allocated must be a bool", "$.domain_policy.domain_allocated")
        if self.domain_allocated or self.selected_domain_id is not None:
            _fail("domain_preallocated", "charter creation cannot allocate a domain", "$.domain_policy")


@dataclass(frozen=True)
class Slice7GScenario:
    scenario_id: str
    source_scenario_id: str
    geometry_profile: str

    def __post_init__(self) -> None:
        _safe_identifier(self.scenario_id, "scenario_id", "$.scenario.scenario_id")
        _safe_identifier(self.source_scenario_id, "source_scenario_id", "$.scenario.source_scenario_id")
        if EXPECTED_SOURCE_SCENARIOS.get(self.scenario_id) != self.source_scenario_id:
            _fail("scenario_source_mismatch", "scenario/source-scenario binding is not approved", "$.scenario")
        _exact_string(self.geometry_profile, "circular_arc", "scenario_geometry", "$.scenario.geometry_profile")


@dataclass(frozen=True)
class Slice7GMetric:
    name: str
    unit: str
    aggregation: str
    comparison: str
    threshold: bool | int | float | None
    promotion_blocking: bool
    rationale: str

    def __post_init__(self) -> None:
        name = _safe_identifier(self.name, "metric_name", "$.metric.name")
        unit = _nonempty_string(self.unit, "metric_unit", "$.metric.unit")
        if unit not in SAFE_UNITS:
            _fail("metric_unit", f"unsupported unit: {unit}", "$.metric.unit")
        _nonempty_string(self.aggregation, "metric_aggregation", "$.metric.aggregation")
        comparison = _nonempty_string(self.comparison, "metric_comparison", "$.metric.comparison")
        if comparison not in {"equal", "less_than_or_equal", "greater_than_or_equal", "report_only"}:
            _fail("metric_comparison", f"unsupported comparison: {comparison}", "$.metric.comparison")
        if type(self.promotion_blocking) is not bool:
            _fail("metric_promotion_type", "promotion_blocking must be an exact bool", "$.metric.promotion_blocking")
        _nonempty_string(self.rationale, "metric_rationale", "$.metric.rationale")
        if comparison == "report_only":
            if self.threshold is not None or self.promotion_blocking:
                _fail("metric_threshold", "report-only metrics must be nonblocking with a null threshold", "$.metric")
        elif unit == "boolean":
            if type(self.threshold) is not bool:
                _fail("metric_threshold", "boolean metrics require an exact bool threshold", "$.metric.threshold")
        elif type(self.threshold) not in (int, float) or type(self.threshold) is bool:
            _fail("metric_threshold", "numeric metrics require an exact numeric threshold", "$.metric.threshold")
        elif type(self.threshold) is float and not math.isfinite(self.threshold):
            _fail("metric_threshold", "numeric metric thresholds must be finite", "$.metric.threshold")
        expected = _expected_metrics().get(name)
        if expected is None:
            _fail("metric_name", "metric is not in the approved profile", "$.metric.name")
        for field, required in expected.items():
            observed = getattr(self, field)
            if type(observed) is not type(required) or observed != required:
                _fail("metric_contract", f"{name}.{field} must equal {required!r}", f"$.metric.{field}")


@dataclass(frozen=True, init=False)
class Slice7GCharter:
    """Deeply immutable validated charter value."""

    data: dict[str, Any]
    canonical_bytes: bytes
    attempt_budget: Slice7GAttemptBudget
    domain_policy: Slice7GDomainPolicy
    scenarios: tuple[Slice7GScenario, ...]
    metrics: tuple[Slice7GMetric, ...]

    def __init__(self, *_: Any, **__: Any) -> None:
        _fail("direct_construction", "use validate_slice_7g_charter or load_slice_7g_charter")

    @classmethod
    def _create(cls, data: dict[str, Any]) -> "Slice7GCharter":
        instance = object.__new__(cls)
        canonical = _canonical_json(data)
        object.__setattr__(instance, "data", _freeze(data))
        object.__setattr__(instance, "canonical_bytes", canonical)
        budget = data["attempt_budget"]
        object.__setattr__(
            instance,
            "attempt_budget",
            Slice7GAttemptBudget(
                budget["maximum_campaigns"], budget["consumed_campaigns"], budget["retries_authorized"]
            ),
        )
        policy = data["domain_policy"]
        object.__setattr__(
            instance,
            "domain_policy",
            Slice7GDomainPolicy(
                policy["minimum_domain_id"],
                policy["maximum_domain_id"],
                policy["domain_allocated"],
                policy["selected_domain_id"],
            ),
        )
        object.__setattr__(
            instance,
            "scenarios",
            tuple(Slice7GScenario(**item) for item in data["campaign"]["scenarios"]),
        )
        object.__setattr__(
            instance,
            "metrics",
            tuple(Slice7GMetric(**item) for item in data["acceptance_contract"]["metrics"]),
        )
        return instance


@dataclass(frozen=True)
class Slice7GAttemptLedger:
    schema_version: str
    charter_logical_identity: str
    campaign_id: str
    campaign_identity: str
    campaign_plan_identity: str | None
    runtime_authorization_identity: str | None
    revision: int
    predecessor_ledger_identity: str | None
    applied_event_identities: tuple[str, ...]
    applied_event_ids: tuple[str, ...]
    last_event_identity: str | None
    maximum_campaign_attempts: int
    consumed_campaign_attempts: int
    retry_count: int
    maximum_retries: int
    domain_allocated: bool
    domain_id: int | None
    output_root_allocated: bool
    output_root: str | None
    process_start_committed: bool

    def __post_init__(self) -> None:
        _exact_string(self.schema_version, ATTEMPT_LEDGER_SCHEMA_VERSION, "ledger_schema", "$.ledger.schema_version")
        _digest(self.charter_logical_identity, "$.ledger.charter_logical_identity")
        _opaque_identifier(self.campaign_id, "campaign_id", "$.ledger.campaign_id")
        _digest(self.campaign_identity, "$.ledger.campaign_identity")
        if self.campaign_plan_identity is not None:
            _digest(self.campaign_plan_identity, "$.ledger.campaign_plan_identity")
        if self.runtime_authorization_identity is not None:
            _digest(self.runtime_authorization_identity, "$.ledger.runtime_authorization_identity")
        _nonnegative_int(self.revision, "ledger_revision", "$.ledger.revision")
        if self.predecessor_ledger_identity is not None:
            _digest(self.predecessor_ledger_identity, "$.ledger.predecessor_ledger_identity")
        identities = _detached_string_tuple(self.applied_event_identities, "ledger_event_identities", "$.ledger.applied_event_identities")
        object.__setattr__(self, "applied_event_identities", identities)
        if len(identities) != len(set(identities)):
            _fail("duplicate_ledger_event", "applied event identities must be unique", "$.ledger.applied_event_identities")
        for index, identity in enumerate(identities):
            _digest(identity, f"$.ledger.applied_event_identities[{index}]")
        events = _detached_string_tuple(self.applied_event_ids, "ledger_event_ids", "$.ledger.applied_event_ids")
        object.__setattr__(self, "applied_event_ids", events)
        if len(events) != len(set(events)):
            _fail("duplicate_ledger_event_id", "applied event IDs must be unique", "$.ledger.applied_event_ids")
        if len(events) != len(identities):
            _fail("ledger_event_history", "event IDs and identities must have equal cardinality", "$.ledger")
        for index, event_id in enumerate(events):
            _opaque_identifier(event_id, "ledger_event_id", f"$.ledger.applied_event_ids[{index}]")
        if self.last_event_identity is not None:
            _digest(self.last_event_identity, "$.ledger.last_event_identity")
        _exact_int(self.maximum_campaign_attempts, 1, "attempt_maximum", "$.ledger.maximum_campaign_attempts")
        if type(self.consumed_campaign_attempts) is not int or self.consumed_campaign_attempts not in (0, 1):
            _fail("attempt_count", "consumed attempts must be zero or one", "$.ledger.consumed_campaign_attempts")
        _exact_int(self.maximum_retries, 0, "retry_count", "$.ledger.maximum_retries")
        _exact_int(self.retry_count, 0, "retry_count", "$.ledger.retry_count")
        for field in ("domain_allocated", "output_root_allocated", "process_start_committed"):
            if type(getattr(self, field)) is not bool:
                _fail("ledger_bool", f"{field} must be an exact bool", f"$.ledger.{field}")
        if self.revision == 0:
            if self.predecessor_ledger_identity is not None or events or identities or self.last_event_identity is not None:
                _fail("ledger_initial_state", "revision zero cannot have a predecessor or events", "$.ledger")
        elif self.predecessor_ledger_identity is None or not events or self.last_event_identity is None:
            _fail("ledger_history", "advanced ledgers require predecessor and event identities", "$.ledger")
        if len(events) != self.revision:
            _fail("ledger_revision_history", "ledger revision must equal the applied-event count", "$.ledger.revision")
        if identities and self.last_event_identity != identities[-1]:
            _fail("ledger_last_event", "last-event identity must equal the final applied identity", "$.ledger.last_event_identity")
        if self.domain_allocated:
            _domain_id(self.domain_id, "$.ledger.domain_id")
            if self.runtime_authorization_identity is None:
                _fail("runtime_authorization_required", "allocated ledger requires an external runtime authorization identity", "$.ledger.runtime_authorization_identity")
        elif self.domain_id is not None:
            _fail("ledger_domain_state", "unallocated domain must be null", "$.ledger.domain_id")
        elif self.runtime_authorization_identity is not None:
            _fail("ledger_authorization_state", "unallocated ledger cannot bind runtime authorization", "$.ledger.runtime_authorization_identity")
        if self.output_root_allocated:
            _external_output_root(self.output_root, "$.ledger.output_root")
        elif self.output_root is not None:
            _fail("ledger_output_state", "unallocated output root must be null", "$.ledger.output_root")
        if self.domain_allocated is not self.output_root_allocated:
            _fail("ledger_allocation_state", "domain and output root allocation must be committed together", "$.ledger")
        if self.process_start_committed:
            if self.consumed_campaign_attempts != 1 or not self.domain_allocated or self.campaign_plan_identity is None:
                _fail("ledger_process_state", "process-start commit requires consumed attempt, plan, and allocations", "$.ledger")
        elif self.consumed_campaign_attempts != 0:
            _fail("ledger_process_state", "uncommitted process state cannot consume the attempt", "$.ledger")
        elif self.campaign_plan_identity is not None:
            _fail("ledger_plan_state", "uncommitted ledger cannot bind a started campaign plan", "$.ledger.campaign_plan_identity")


@dataclass(frozen=True)
class Slice7GAttemptEvent:
    schema_version: str
    charter_logical_identity: str
    campaign_identity: str
    campaign_plan_identity: str | None
    runtime_authorization_identity: str | None
    event_id: str
    event_kind: str
    expected_revision: int
    expected_predecessor_ledger_identity: str
    previous_attempt_count: int
    resulting_attempt_count: int
    retry_count: int
    maximum_retries: int
    domain_allocated: bool
    domain_id: int | None
    output_root_allocated: bool
    output_root: str | None
    process_start_consumed: bool
    event_timestamp_utc: str

    def __post_init__(self) -> None:
        _exact_string(self.schema_version, ATTEMPT_EVENT_SCHEMA_VERSION, "event_schema", "$.event.schema_version")
        _digest(self.charter_logical_identity, "$.event.charter_logical_identity")
        _digest(self.campaign_identity, "$.event.campaign_identity")
        if self.campaign_plan_identity is not None:
            _digest(self.campaign_plan_identity, "$.event.campaign_plan_identity")
        if self.runtime_authorization_identity is not None:
            _digest(self.runtime_authorization_identity, "$.event.runtime_authorization_identity")
        _opaque_identifier(self.event_id, "event_id", "$.event.event_id")
        if type(self.event_kind) is not str or self.event_kind not in {
            "preflight_failed_before_process_creation", "domain_and_output_allocated", "process_start_commit"
        }:
            _fail("attempt_event", "unsupported attempt event kind", "$.event.event_kind")
        _nonnegative_int(self.expected_revision, "event_revision", "$.event.expected_revision")
        _digest(self.expected_predecessor_ledger_identity, "$.event.expected_predecessor_ledger_identity")
        for field in ("previous_attempt_count", "resulting_attempt_count"):
            value = getattr(self, field)
            if type(value) is not int or value not in (0, 1):
                _fail("attempt_count", f"{field} must be zero or one", f"$.event.{field}")
        _exact_int(self.retry_count, 0, "retry_count", "$.event.retry_count")
        _exact_int(self.maximum_retries, 0, "retry_count", "$.event.maximum_retries")
        for field in ("domain_allocated", "output_root_allocated", "process_start_consumed"):
            if type(getattr(self, field)) is not bool:
                _fail("event_bool", f"{field} must be an exact bool", f"$.event.{field}")
        if self.domain_allocated:
            _domain_id(self.domain_id, "$.event.domain_id")
            if self.runtime_authorization_identity is None:
                _fail("runtime_authorization_required", "allocated event requires external runtime authorization", "$.event.runtime_authorization_identity")
        elif self.domain_id is not None:
            _fail("event_domain_state", "unallocated event domain must be null", "$.event.domain_id")
        elif self.runtime_authorization_identity is not None:
            _fail("event_authorization_state", "unallocated event cannot bind runtime authorization", "$.event.runtime_authorization_identity")
        if self.output_root_allocated:
            _external_output_root(self.output_root, "$.event.output_root")
        elif self.output_root is not None:
            _fail("event_output_state", "unallocated event output root must be null", "$.event.output_root")
        if self.domain_allocated is not self.output_root_allocated:
            _fail("event_allocation_state", "domain and output root must be allocated together", "$.event")
        _validate_utc_timestamp(self.event_timestamp_utc, "$.event.event_timestamp_utc", "event_timestamp")
        if self.event_kind == "preflight_failed_before_process_creation":
            if self.previous_attempt_count != self.resulting_attempt_count or self.process_start_consumed or self.campaign_plan_identity is not None:
                _fail("event_semantics", "preflight failure cannot consume an attempt", "$.event")
        elif self.event_kind == "domain_and_output_allocated":
            if self.previous_attempt_count != self.resulting_attempt_count or self.process_start_consumed or not self.domain_allocated or self.campaign_plan_identity is not None:
                _fail("event_semantics", "allocation event must allocate without consuming", "$.event")
        elif self.event_kind == "process_start_commit":
            if (self.previous_attempt_count, self.resulting_attempt_count) != (0, 1) or not self.process_start_consumed or not self.domain_allocated or self.campaign_plan_identity is None:
                _fail("event_semantics", "process-start event must atomically consume 0/1 with allocations", "$.event")


@dataclass(frozen=True)
class Slice7GCampaignCell:
    schema_version: str
    cell_id: str
    charter_logical_identity: str
    campaign_id: str
    campaign_identity: str
    attempt_ledger_identity: str
    scenario_id: str
    source_scenario_id: str
    seed: int
    geometry_profile: str
    task: str
    duration_seconds: float
    runtime_mode: str
    ros_domain_id: int
    campaign_output_root: str
    cell_output_path: str
    argv: tuple[str, ...]
    metric_profile_identity: str
    domain_allocation_requested: bool

    def __post_init__(self) -> None:
        _exact_string(self.schema_version, CAMPAIGN_CELL_SCHEMA_VERSION, "campaign_cell_schema", "$.cell.schema_version")
        _opaque_identifier(self.cell_id, "cell_id", "$.cell.cell_id")
        _digest(self.charter_logical_identity, "$.cell.charter_logical_identity")
        _opaque_identifier(self.campaign_id, "campaign_id", "$.cell.campaign_id")
        _digest(self.campaign_identity, "$.cell.campaign_identity")
        _digest(self.attempt_ledger_identity, "$.cell.attempt_ledger_identity")
        Slice7GScenario(self.scenario_id, self.source_scenario_id, self.geometry_profile)
        if type(self.seed) is not int or self.seed not in EXPECTED_SEEDS:
            _fail("cell_seed", "cell seed is not approved", "$.cell.seed")
        _exact_string(self.task, "curved_lumen_navigation", "cell_task", "$.cell.task")
        _exact_number(self.duration_seconds, 25.0, "cell_duration", "$.cell.duration_seconds")
        _exact_string(self.runtime_mode, "simulation", "cell_runtime_mode", "$.cell.runtime_mode")
        _domain_id(self.ros_domain_id, "$.cell.ros_domain_id")
        root = _external_output_root(self.campaign_output_root, "$.cell.campaign_output_root")
        output = _external_output_root(self.cell_output_path, "$.cell.cell_output_path")
        if not _is_strict_descendant(output, root):
            _fail("cell_output_escape", "cell output path must be beneath the campaign output root", "$.cell.cell_output_path")
        argv = _detached_string_tuple(self.argv, "cell_argv", "$.cell.argv")
        object.__setattr__(self, "argv", argv)
        _digest(self.metric_profile_identity, "$.cell.metric_profile_identity")
        _exact_bool(self.domain_allocation_requested, False, "cell_domain_allocation", "$.cell.domain_allocation_requested")
        expected_campaign = _campaign_identity(self.charter_logical_identity, self.campaign_id)
        if self.campaign_identity != expected_campaign:
            _fail("cell_campaign_identity", "cell campaign identity does not derive from charter and campaign ID", "$.cell.campaign_identity")
        expected_id = f"{self.scenario_id}.seed_{self.seed:010d}"
        if self.cell_id != expected_id:
            _fail("cell_id_mismatch", "cell ID is not the deterministic scenario/seed identity", "$.cell.cell_id")
        expected_output = f"{root}/cells/{self.cell_id}"
        if output != expected_output:
            _fail("cell_output_path", "cell output path is not the deterministic campaign child", "$.cell.cell_output_path")
        expected_argv = (
            "ctr_run_evaluation", "--experiment-group", self.campaign_id,
            "--task", "curved_lumen_navigation", "--curved-lumen-type", "circular_arc",
            "--scenario", self.source_scenario_id, "--seed", str(self.seed), "--duration", "25.0",
            "--runtime-mode", "simulation", "--output-root", output,
        )
        if self.argv != expected_argv:
            _fail("cell_argv_mismatch", "cell argv differs or semantic ordering changed", "$.cell.argv")


@dataclass(frozen=True)
class Slice7GCampaignPlan:
    schema_version: str
    charter_logical_identity: str
    campaign_id: str
    campaign_identity: str
    attempt_ledger_identity: str
    ros_domain_id: int
    campaign_output_root: str
    metric_profile_identity: str
    cells: tuple[Slice7GCampaignCell, ...]

    def __post_init__(self) -> None:
        _exact_string(self.schema_version, CAMPAIGN_PLAN_SCHEMA_VERSION, "campaign_plan_schema", "$.plan.schema_version")
        _digest(self.charter_logical_identity, "$.plan.charter_logical_identity")
        _opaque_identifier(self.campaign_id, "campaign_id", "$.plan.campaign_id")
        _digest(self.campaign_identity, "$.plan.campaign_identity")
        _digest(self.attempt_ledger_identity, "$.plan.attempt_ledger_identity")
        _domain_id(self.ros_domain_id, "$.plan.ros_domain_id")
        _external_output_root(self.campaign_output_root, "$.plan.campaign_output_root")
        _digest(self.metric_profile_identity, "$.plan.metric_profile_identity")
        cells = _detached_exact_record_tuple(self.cells, Slice7GCampaignCell, "campaign_cell_type", "$.plan.cells")
        object.__setattr__(self, "cells", cells)
        if len(cells) != 15:
            _fail("campaign_cell_count", "campaign plan must contain exactly 15 cells", "$.plan.cells")
        if self.campaign_identity != _campaign_identity(self.charter_logical_identity, self.campaign_id):
            _fail("plan_campaign_identity", "plan campaign identity does not derive from charter and campaign ID", "$.plan.campaign_identity")
        pairs = [(cell.scenario_id, cell.seed) for cell in cells]
        expected_pairs = [(scenario, seed) for scenario in EXPECTED_SOURCE_SCENARIOS for seed in EXPECTED_SEEDS]
        if len(pairs) != len(set(pairs)):
            _fail("duplicate_campaign_cell", "campaign plan contains duplicate scenario/seed cells", "$.plan.cells")
        if pairs != expected_pairs:
            _fail("campaign_plan_bijection", "campaign plan is not the exact approved Cartesian product", "$.plan.cells")
        for cell in cells:
            if (
                cell.charter_logical_identity != self.charter_logical_identity or cell.campaign_id != self.campaign_id
                or cell.campaign_identity != self.campaign_identity or cell.attempt_ledger_identity != self.attempt_ledger_identity
                or cell.ros_domain_id != self.ros_domain_id or cell.campaign_output_root != self.campaign_output_root
                or cell.metric_profile_identity != self.metric_profile_identity
            ):
                _fail("cell_binding_mismatch", "cell bindings differ from plan authority", "$.plan.cells")


@dataclass(frozen=True)
class Slice7GCellResult:
    schema_version: str
    cell_id: str
    charter_logical_identity: str
    campaign_identity: str
    campaign_plan_identity: str
    attempt_ledger_identity: str
    attempt_ledger_revision: int
    process_start_event_identity: str
    runtime_authorization_identity: str
    metric_profile_identity: str
    scenario_id: str
    source_scenario_id: str
    seed: int
    duration_seconds: float
    runtime_mode: str
    ros_domain_id: int
    campaign_output_root: str
    cell_output_path: str
    argv: tuple[str, ...]
    process_exit_status: int
    readiness_success: bool
    stable_sample_count: int
    stable_interval_seconds: float
    q_variation: float
    tip_variation_m: float
    valid_aligned_sample_count: int
    invalid_sample_count: int
    invalid_sample_percentage: float
    steady_state_error_m: float
    final_goal_error_m: float
    goal_hold_duration_seconds: float
    minimum_physical_wall_clearance_m: float
    minimum_safety_margin_wall_clearance_m: float
    collision_sample_count: int
    safety_fault_count: int
    nonfinite_value_count: int
    missing_required_topic_count: int
    missing_required_result_file_count: int
    saturation_percentage: float
    deadline_overrun_percentage: float
    timing_pass: bool
    non_real_time_label: bool

    def __post_init__(self) -> None:
        _exact_string(self.schema_version, CELL_RESULT_SCHEMA_VERSION, "cell_result_schema", "$.cell_result.schema_version")
        _opaque_identifier(self.cell_id, "cell_id", "$.cell_result.cell_id")
        for field in (
            "charter_logical_identity", "campaign_identity", "campaign_plan_identity",
            "attempt_ledger_identity", "process_start_event_identity",
            "runtime_authorization_identity", "metric_profile_identity",
        ):
            _digest(getattr(self, field), f"$.cell_result.{field}")
        if type(self.attempt_ledger_revision) is not int or self.attempt_ledger_revision < 1:
            _fail("result_ledger_revision", "ledger revision must be a positive exact integer", "$.cell_result.attempt_ledger_revision")
        Slice7GScenario(self.scenario_id, self.source_scenario_id, "circular_arc")
        if type(self.seed) is not int or self.seed not in EXPECTED_SEEDS:
            _fail("result_seed", "result seed is not approved", "$.cell_result.seed")
        if self.cell_id != f"{self.scenario_id}.seed_{self.seed:010d}":
            _fail("result_cell_id", "result cell ID does not derive from scenario and seed", "$.cell_result.cell_id")
        _exact_number(self.duration_seconds, 25.0, "result_duration", "$.cell_result.duration_seconds")
        _exact_string(self.runtime_mode, "simulation", "result_runtime_mode", "$.cell_result.runtime_mode")
        _domain_id(self.ros_domain_id, "$.cell_result.ros_domain_id")
        campaign_root = _external_output_root(self.campaign_output_root, "$.cell_result.campaign_output_root")
        _external_output_root(self.cell_output_path, "$.cell_result.cell_output_path")
        if not _is_strict_descendant(self.cell_output_path, campaign_root):
            _fail("result_output_binding", "cell output must be beneath campaign output root", "$.cell_result.cell_output_path")
        object.__setattr__(self, "argv", _detached_string_tuple(self.argv, "result_argv", "$.cell_result.argv"))
        for field in ("readiness_success", "timing_pass", "non_real_time_label"):
            if type(getattr(self, field)) is not bool:
                _fail("result_bool", f"{field} must be an exact bool", f"$.cell_result.{field}")
        for field in (
            "process_exit_status", "stable_sample_count", "valid_aligned_sample_count", "invalid_sample_count",
            "collision_sample_count", "safety_fault_count", "nonfinite_value_count",
            "missing_required_topic_count", "missing_required_result_file_count",
        ):
            _nonnegative_int(getattr(self, field), "result_count", f"$.cell_result.{field}")
        for field in (
            "stable_interval_seconds", "q_variation", "tip_variation_m", "invalid_sample_percentage",
            "steady_state_error_m", "final_goal_error_m", "goal_hold_duration_seconds",
            "minimum_physical_wall_clearance_m", "minimum_safety_margin_wall_clearance_m",
            "saturation_percentage", "deadline_overrun_percentage",
        ):
            _finite_number(getattr(self, field), "result_number", f"$.cell_result.{field}")
        for field in (
            "stable_interval_seconds", "q_variation", "tip_variation_m", "invalid_sample_percentage",
            "steady_state_error_m", "final_goal_error_m", "goal_hold_duration_seconds",
            "saturation_percentage", "deadline_overrun_percentage",
        ):
            if getattr(self, field) < 0.0:
                _fail("result_number", f"{field} cannot be negative", f"$.cell_result.{field}")
        if self.invalid_sample_percentage > 100.0 or self.saturation_percentage > 100.0:
            _fail("result_percentage", "sample percentages cannot exceed 100", "$.cell_result")
        total_samples = self.valid_aligned_sample_count + self.invalid_sample_count
        expected_invalid_percentage = 0.0 if total_samples == 0 else 100.0 * self.invalid_sample_count / total_samples
        if not math.isclose(self.invalid_sample_percentage, expected_invalid_percentage, rel_tol=0.0, abs_tol=1.0e-12):
            _fail("result_sample_accounting", "invalid sample count and percentage disagree", "$.cell_result.invalid_sample_percentage")
        if self.timing_pass is not (self.deadline_overrun_percentage <= 5.0):
            _fail("timing_pass_mismatch", "timing_pass must reflect the 5 percent diagnostic target", "$.cell_result.timing_pass")
        if not self.timing_pass and not self.non_real_time_label:
            _fail("missing_non_real_time_label", "timing failure requires a non-real-time label", "$.cell_result.non_real_time_label")


@dataclass(frozen=True)
class Slice7GCampaignEvidencePackage:
    schema_version: str
    cell_id: str
    relative_path: str
    package_identity: str

    def __post_init__(self) -> None:
        _exact_string(
            self.schema_version,
            CAMPAIGN_EVIDENCE_PACKAGE_RECORD_SCHEMA_VERSION,
            "campaign_evidence_package_schema",
            "$.campaign_evidence_package.schema_version",
        )
        _opaque_identifier(self.cell_id, "campaign_evidence_cell_id", "$.campaign_evidence_package.cell_id")
        relative = _safe_relative_path(self.relative_path, "$.campaign_evidence_package.relative_path")
        expected = f"{CAMPAIGN_EVIDENCE_PACKAGES_RELATIVE_PATH}/{self.cell_id}"
        if relative != expected:
            _fail(
                "campaign_evidence_package_path",
                "package path must be the deterministic packages/<cell_id> path",
                "$.campaign_evidence_package.relative_path",
            )
        _digest(self.package_identity, "$.campaign_evidence_package.package_identity")


@dataclass(frozen=True)
class Slice7GCampaignEvidenceSeal:
    schema_version: str
    charter_logical_identity: str
    campaign_identity: str
    campaign_plan_identity: str
    runtime_authorization_identity: str
    attempt_ledger_identity: str
    attempt_ledger_revision: int
    process_start_event_identity: str
    ros_domain_id: int
    campaign_output_root: str
    evidence_root_relative_path: str
    packages: tuple[Slice7GCampaignEvidencePackage, ...]

    def __post_init__(self) -> None:
        _exact_string(
            self.schema_version,
            CAMPAIGN_EVIDENCE_SEAL_SCHEMA_VERSION,
            "campaign_evidence_seal_schema",
            "$.campaign_evidence_seal.schema_version",
        )
        for field in (
            "charter_logical_identity", "campaign_identity", "campaign_plan_identity",
            "runtime_authorization_identity", "attempt_ledger_identity", "process_start_event_identity",
        ):
            _digest(getattr(self, field), f"$.campaign_evidence_seal.{field}")
        if type(self.attempt_ledger_revision) is not int or self.attempt_ledger_revision < 1:
            _fail(
                "campaign_evidence_ledger_revision",
                "seal ledger revision must be a positive exact integer",
                "$.campaign_evidence_seal.attempt_ledger_revision",
            )
        _domain_id(self.ros_domain_id, "$.campaign_evidence_seal.ros_domain_id")
        _external_output_root(self.campaign_output_root, "$.campaign_evidence_seal.campaign_output_root")
        _exact_string(
            self.evidence_root_relative_path,
            CAMPAIGN_EVIDENCE_ROOT_RELATIVE_PATH,
            "campaign_evidence_root_path",
            "$.campaign_evidence_seal.evidence_root_relative_path",
        )
        if type(self.packages) not in (tuple, list):
            _fail("campaign_evidence_packages_type", "seal packages must be an exact list or tuple")
        packages = tuple(_validate_campaign_evidence_package_record(item) for item in self.packages)
        object.__setattr__(self, "packages", packages)
        if len(packages) != 15:
            _fail("campaign_evidence_seal_count", "seal must bind exactly 15 packages")
        cells = [item.cell_id for item in packages]
        paths = [item.relative_path for item in packages]
        identities = [item.package_identity for item in packages]
        if len(cells) != len(set(cells)):
            _fail("campaign_evidence_seal_duplicate_cell", "seal cell IDs must be unique")
        if len(paths) != len(set(paths)):
            _fail("campaign_evidence_seal_duplicate_path", "seal package paths must be unique")
        if len(identities) != len(set(identities)):
            _fail("campaign_evidence_seal_duplicate_identity", "seal package identities must be unique")


@dataclass(frozen=True)
class Slice7GCampaignResult:
    schema_version: str
    charter_logical_identity: str
    campaign_identity: str
    campaign_plan_identity: str
    campaign_evidence_snapshot_identity: str
    evidence_package_identities: tuple[str, ...]
    result_identities: tuple[str, ...]
    total_result_count: int
    functionally_passing_cell_count: int
    functionally_failing_cell_ids: tuple[str, ...]
    functional_failure_reasons: tuple[str, ...]
    functional_promotion_pass: bool
    timing_all_pass: bool
    non_real_time_limitation_required: bool
    total_valid_aligned_samples: int
    total_invalid_samples: int
    total_collision_samples: int
    total_safety_faults: int
    total_nonfinite_values: int
    total_missing_required_topics: int
    total_missing_required_results: int
    timing_failure_cell_count: int

    def __post_init__(self) -> None:
        _exact_string(self.schema_version, CAMPAIGN_RESULT_SCHEMA_VERSION, "campaign_result_schema", "$.campaign_result.schema_version")
        for field in (
            "charter_logical_identity", "campaign_identity", "campaign_plan_identity",
            "campaign_evidence_snapshot_identity",
        ):
            _digest(getattr(self, field), f"$.campaign_result.{field}")
        package_identities = _detached_string_tuple(
            self.evidence_package_identities, "evidence_package_identities", "$.campaign_result.evidence_package_identities"
        )
        object.__setattr__(self, "evidence_package_identities", package_identities)
        if len(package_identities) != 15 or len(package_identities) != len(set(package_identities)):
            _fail("campaign_evidence_count", "campaign must bind 15 unique physical evidence packages", "$.campaign_result.evidence_package_identities")
        for index, identity in enumerate(package_identities):
            _digest(identity, f"$.campaign_result.evidence_package_identities[{index}]")
        identities = _detached_string_tuple(self.result_identities, "result_identities", "$.campaign_result.result_identities")
        object.__setattr__(self, "result_identities", identities)
        if len(identities) != len(set(identities)):
            _fail("duplicate_result_identity", "result identities must be unique", "$.campaign_result.result_identities")
        for index, identity in enumerate(identities):
            _digest(identity, f"$.campaign_result.result_identities[{index}]")
        if len(identities) != 15:
            _fail("campaign_result_count", "campaign result must bind exactly 15 result identities", "$.campaign_result.result_identities")
        _exact_int(self.total_result_count, 15, "campaign_result_count", "$.campaign_result.total_result_count")
        if type(self.functionally_passing_cell_count) is not int or not 0 <= self.functionally_passing_cell_count <= 15:
            _fail("campaign_pass_count", "passing cell count must be in 0..15", "$.campaign_result.functionally_passing_cell_count")
        failing_cells = _detached_string_tuple(self.functionally_failing_cell_ids, "failing_cells", "$.campaign_result.functionally_failing_cell_ids")
        if len(failing_cells) != len(set(failing_cells)):
            _fail("duplicate_failing_cell", "failing cell IDs must be unique", "$.campaign_result.functionally_failing_cell_ids")
        for index, cell_id in enumerate(failing_cells):
            _opaque_identifier(cell_id, "failing_cell_id", f"$.campaign_result.functionally_failing_cell_ids[{index}]")
        object.__setattr__(self, "functionally_failing_cell_ids", failing_cells)
        failure_reasons = _detached_string_tuple(self.functional_failure_reasons, "failure_reasons", "$.campaign_result.functional_failure_reasons")
        object.__setattr__(self, "functional_failure_reasons", failure_reasons)
        for field in ("functional_promotion_pass", "timing_all_pass", "non_real_time_limitation_required"):
            if type(getattr(self, field)) is not bool:
                _fail("campaign_result_bool", f"{field} must be an exact bool", f"$.campaign_result.{field}")
        if self.functional_promotion_pass is not (self.functionally_passing_cell_count == 15):
            _fail("campaign_result_consistency", "functional promotion flag disagrees with passing cells", "$.campaign_result")
        if self.non_real_time_limitation_required is not (not self.timing_all_pass):
            _fail("campaign_timing_consistency", "non-real-time limitation must follow timing failures", "$.campaign_result")
        for field in (
            "total_valid_aligned_samples", "total_invalid_samples", "total_collision_samples",
            "total_safety_faults", "total_nonfinite_values", "total_missing_required_topics",
            "total_missing_required_results", "timing_failure_cell_count",
        ):
            _nonnegative_int(getattr(self, field), "campaign_aggregate_count", f"$.campaign_result.{field}")
        if self.timing_all_pass is not (self.timing_failure_cell_count == 0):
            _fail("campaign_timing_count", "timing pass flag disagrees with timing failure count", "$.campaign_result")
        if self.functionally_passing_cell_count + len(self.functionally_failing_cell_ids) != 15:
            _fail("campaign_pass_count", "passing and failing cell counts must total 15", "$.campaign_result")
        if self.functional_promotion_pass and (failure_reasons or failing_cells):
            _fail("campaign_result_consistency", "passing campaign cannot retain functional failures", "$.campaign_result")
        if not self.functional_promotion_pass and not failure_reasons:
            _fail("campaign_result_consistency", "failing campaign requires stable failure reasons", "$.campaign_result")


@dataclass(frozen=True)
class Slice7GEvidenceMember:
    role: str
    path: str
    size: int
    sha256: str
    mode: int
    link_count: int
    file_type: str

    def __post_init__(self) -> None:
        _safe_identifier(self.role, "evidence_role", "$.evidence_member.role")
        expected_path = MANDATORY_EVIDENCE_ROLE_PATHS.get(self.role)
        if expected_path is None:
            _fail("unknown_evidence_role", "evidence role is not mandatory", "$.evidence_member.role")
        _safe_relative_path(self.path, "$.evidence_member.path")
        if self.path != expected_path:
            _fail("evidence_role_path", "evidence role has the wrong fixed path", "$.evidence_member.path")
        _nonnegative_int(self.size, "evidence_member_size", "$.evidence_member.size")
        if self.size == 0:
            _fail("evidence_member_size", "evidence members must be nonempty", "$.evidence_member.size")
        _digest(self.sha256, "$.evidence_member.sha256")
        _exact_int(self.mode, 0o444, "evidence_member_mode", "$.evidence_member.mode")
        _exact_int(self.link_count, 1, "evidence_member_link_count", "$.evidence_member.link_count")
        _exact_string(self.file_type, "regular_file", "evidence_member_type", "$.evidence_member.file_type")


@dataclass(frozen=True)
class Slice7GCellEvidenceEnvelope:
    schema_version: str
    charter_logical_identity: str
    campaign_identity: str
    campaign_plan_identity: str
    cell_id: str
    scenario_id: str
    source_scenario_id: str
    seed: int
    metric_profile_identity: str
    attempt_ledger_identity: str
    attempt_ledger_revision: int
    process_start_event_identity: str
    runtime_authorization_identity: str
    ros_domain_id: int
    campaign_output_root: str
    cell_output_path: str
    argv: tuple[str, ...]
    process_exit_status: int
    projection_identity: str
    members: tuple[Slice7GEvidenceMember, ...]

    def __post_init__(self) -> None:
        _exact_string(
            self.schema_version, CELL_EVIDENCE_ENVELOPE_SCHEMA_VERSION,
            "evidence_envelope_schema", "$.evidence_envelope.schema_version",
        )
        for field in (
            "charter_logical_identity", "campaign_identity", "campaign_plan_identity",
            "metric_profile_identity", "attempt_ledger_identity", "process_start_event_identity",
            "runtime_authorization_identity", "projection_identity",
        ):
            _digest(getattr(self, field), f"$.evidence_envelope.{field}")
        _opaque_identifier(self.cell_id, "evidence_cell_id", "$.evidence_envelope.cell_id")
        Slice7GScenario(self.scenario_id, self.source_scenario_id, "circular_arc")
        if type(self.seed) is not int or self.seed not in EXPECTED_SEEDS:
            _fail("evidence_seed", "evidence seed is not approved", "$.evidence_envelope.seed")
        if self.cell_id != f"{self.scenario_id}.seed_{self.seed:010d}":
            _fail("evidence_cell_id", "cell identity does not derive from scenario and seed", "$.evidence_envelope.cell_id")
        if type(self.attempt_ledger_revision) is not int or self.attempt_ledger_revision < 1:
            _fail("evidence_ledger_revision", "ledger revision must be a positive exact integer", "$.evidence_envelope.attempt_ledger_revision")
        _domain_id(self.ros_domain_id, "$.evidence_envelope.ros_domain_id")
        root = _external_output_root(self.campaign_output_root, "$.evidence_envelope.campaign_output_root")
        output = _external_output_root(self.cell_output_path, "$.evidence_envelope.cell_output_path")
        if not _is_strict_descendant(output, root):
            _fail("evidence_output_binding", "cell output is not below campaign output root", "$.evidence_envelope.cell_output_path")
        object.__setattr__(self, "argv", _detached_string_tuple(self.argv, "evidence_argv", "$.evidence_envelope.argv"))
        _nonnegative_int(self.process_exit_status, "evidence_exit_status", "$.evidence_envelope.process_exit_status")
        if type(self.members) not in (tuple, list):
            _fail("evidence_members_type", "evidence members must be an exact list or tuple", "$.evidence_envelope.members")
        members = tuple(_validate_evidence_member_record(item) for item in self.members)
        object.__setattr__(self, "members", members)
        roles = [member.role for member in members]
        paths = [member.path for member in members]
        if len(roles) != len(set(roles)):
            _fail("duplicate_evidence_role", "evidence roles must be unique", "$.evidence_envelope.members")
        if len(paths) != len(set(paths)):
            _fail("duplicate_evidence_path", "evidence paths must be unique", "$.evidence_envelope.members")
        if set(roles) != set(MANDATORY_EVIDENCE_ROLE_PATHS):
            _fail("evidence_role_set", "evidence envelope does not contain the exact mandatory role set", "$.evidence_envelope.members")
        expected_order = list(MANDATORY_EVIDENCE_ROLE_PATHS)
        if roles != expected_order:
            _fail("evidence_role_order", "evidence roles are not in canonical mandatory order", "$.evidence_envelope.members")


@dataclass(frozen=True, init=False)
class Slice7GAuthenticatedCellEvidence:
    """A derived observation; construction itself is not an authority API."""

    package_root: str
    root_device: int
    root_inode: int
    projection_identity: str
    package_identity: str
    envelope: Slice7GCellEvidenceEnvelope
    cell_result: Slice7GCellResult

    @classmethod
    def _create(
        cls,
        package_root: str,
        root_device: int,
        root_inode: int,
        projection_identity: str,
        package_identity: str,
        envelope: Slice7GCellEvidenceEnvelope,
        cell_result: Slice7GCellResult,
    ) -> "Slice7GAuthenticatedCellEvidence":
        instance = object.__new__(cls)
        for name, value in (
            ("package_root", package_root), ("root_device", root_device), ("root_inode", root_inode),
            ("projection_identity", projection_identity), ("package_identity", package_identity),
            ("envelope", envelope), ("cell_result", cell_result),
        ):
            object.__setattr__(instance, name, value)
        return instance


@dataclass
class _EvidenceAuthorityState:
    """Private live authority for one physically authenticated package.

    The descriptor is intentionally retained until the authoritative caller has
    completed semantic reconciliation and its final physical barrier.  The
    public immutable observation is not itself a capability and this state is
    never returned across the public API.
    """

    package_root: str
    root_fd: int
    root_baseline: os.stat_result
    observations: dict[str, tuple[os.stat_result, bytes, str]]
    authenticated: Slice7GAuthenticatedCellEvidence | None = None
    final_observations: dict[str, tuple[os.stat_result, bytes, str]] | None = None
    barrier_complete: bool = False
    closed: bool = False

    def close(self) -> None:
        if not self.closed:
            os.close(self.root_fd)
            self.closed = True


@dataclass
class _CampaignEvidenceAuthorityState:
    """Private cooperative-lock authority for one finalized campaign tree."""

    campaign_root: str
    campaign_fd: int
    campaign_baseline: os.stat_result
    evidence_fd: int
    evidence_baseline: os.stat_result
    packages_fd: int
    packages_baseline: os.stat_result
    seal_fd: int
    seal_baseline: os.stat_result
    seal_raw: bytes
    seal: Slice7GCampaignEvidenceSeal
    lock_held: bool = True
    closed: bool = False

    def close(self) -> None:
        if self.closed:
            return
        if self.lock_held:
            try:
                fcntl.flock(self.seal_fd, fcntl.LOCK_UN)
            except OSError:
                # Closing the open file description is the final lock release.
                pass
            self.lock_held = False
        for descriptor in (self.seal_fd, self.packages_fd, self.evidence_fd, self.campaign_fd):
            try:
                os.close(descriptor)
            except OSError:
                pass
        self.closed = True


def load_slice_7g_charter(path: str | os.PathLike[str]) -> Slice7GCharter:
    """Descriptor-read, reject noncanonical bytes, and return an immutable charter."""

    file_path = _normalize_public_path(path, "charter_path_type", "charter_open")
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(file_path, flags)
    except (OSError, ValueError) as exc:
        raise Slice7GGovernanceError("charter_open", str(exc), path=file_path) from exc
    try:
        try:
            info = os.fstat(descriptor)
        except OSError as exc:
            raise Slice7GGovernanceError("charter_stat", str(exc), path=file_path) from exc
        if not stat.S_ISREG(info.st_mode):
            _fail("charter_file_type", "charter must be a regular file", file_path)
        chunks: list[bytes] = []
        try:
            while True:
                chunk = os.read(descriptor, 1024 * 1024)
                if not chunk:
                    break
                chunks.append(chunk)
        except OSError as exc:
            raise Slice7GGovernanceError("charter_read", str(exc), path=file_path) from exc
        try:
            final_info = os.fstat(descriptor)
        except OSError as exc:
            raise Slice7GGovernanceError("charter_stat", str(exc), path=file_path) from exc
        stable = lambda item: (
            item.st_dev, item.st_ino, item.st_mode, item.st_nlink, item.st_size,
            item.st_mtime_ns, item.st_ctime_ns,
        )
        if stable(info) != stable(final_info):
            _fail("charter_changed", "charter metadata changed while reading", file_path)
    finally:
        os.close(descriptor)
    raw = b"".join(chunks)
    data = _parse_json_bytes(raw)
    charter = validate_slice_7g_charter(data)
    if raw != charter.canonical_bytes:
        _fail("noncanonical_json", "charter bytes are not canonical", file_path)
    return charter


def inspect_historical_slice_7g_charter(raw: bytes) -> HistoricalSlice7GCharterInspection:
    """Inspect canonical v4 bytes without upgrading them into runtime authority."""

    if type(raw) is not bytes:
        _fail("historical_charter_type", "historical charter must be exact bytes")
    data = _parse_json_bytes(raw)
    if data.get("schema_version") != HISTORICAL_SCHEMA_VERSION:
        _fail("historical_charter_schema", "only charter-v4 is accepted for historical inspection")
    if data.get("charter_id") != "slice_7g_simulation_promotion_v5":
        _fail("historical_charter_id", "historical charter ID differs")
    if "runtime_authority_contract" in data:
        _fail("historical_charter_injection", "historical charter cannot gain runtime-authority fields")
    canonical = _canonical_json(data)
    if raw != canonical:
        _fail("historical_charter_noncanonical", "historical charter bytes are not canonical")
    return HistoricalSlice7GCharterInspection(
        HISTORICAL_SCHEMA_VERSION,
        hashlib.sha256(HISTORICAL_CHARTER_IDENTITY_DOMAIN + canonical).hexdigest(),
        canonical,
        False,
    )


def validate_slice_7g_charter(value: dict[str, Any] | Slice7GCharter) -> Slice7GCharter:
    """Validate the complete closed schema and return a detached immutable value."""

    retained_canonical: bytes | None = None
    if type(value) is Slice7GCharter:
        try:
            supplied_data = value.data
            retained_canonical = value.canonical_bytes
        except AttributeError as exc:
            raise Slice7GGovernanceError("invalid_charter_record", "charter record is partially initialized") from exc
        if type(retained_canonical) is not bytes:
            _fail("invalid_charter_record", "retained canonical representation must be exact bytes")
        if type(supplied_data) is dict:
            data = _plain_detached(supplied_data, "$.data")
        else:
            data = _thaw_owned_immutable(supplied_data, "$.data")
    elif type(value) is dict:
        data = _plain_detached(value)
    elif Slice7GCharter in type(value).__mro__[1:]:
        _fail("charter_exact_type", "Slice7GCharter subclasses are not accepted")
    else:
        _fail("charter_type", "charter must be an exact object or exact Slice7GCharter")
    _validate_top_level(data)
    rebuilt = Slice7GCharter._create(data)
    if retained_canonical is not None and retained_canonical != rebuilt.canonical_bytes:
        _fail("charter_canonical_mismatch", "retained canonical bytes disagree with reconstructed data")
    return rebuilt


def canonical_slice_7g_charter_bytes(value: dict[str, Any] | Slice7GCharter) -> bytes:
    return validate_slice_7g_charter(value).canonical_bytes


def slice_7g_charter_identity(value: dict[str, Any] | Slice7GCharter) -> str:
    canonical = canonical_slice_7g_charter_bytes(value)
    return hashlib.sha256(CHARTER_IDENTITY_DOMAIN + canonical).hexdigest()


def validate_slice_7g_attempt_budget(value: dict[str, Any] | Slice7GAttemptBudget) -> Slice7GAttemptBudget:
    fields = ("maximum_campaigns", "consumed_campaigns", "retries_authorized")
    return Slice7GAttemptBudget(**_record_data(value, Slice7GAttemptBudget, fields, "attempt_budget_type"))


def validate_slice_7g_domain_policy(value: dict[str, Any] | Slice7GDomainPolicy) -> Slice7GDomainPolicy:
    fields = ("minimum_domain_id", "maximum_domain_id", "domain_allocated", "selected_domain_id")
    return Slice7GDomainPolicy(**_record_data(value, Slice7GDomainPolicy, fields, "domain_policy_type"))


def validate_slice_7g_scenario(value: dict[str, Any] | Slice7GScenario) -> Slice7GScenario:
    fields = ("scenario_id", "source_scenario_id", "geometry_profile")
    return Slice7GScenario(**_record_data(value, Slice7GScenario, fields, "scenario_type"))


def validate_slice_7g_metric(value: dict[str, Any] | Slice7GMetric) -> Slice7GMetric:
    fields = ("name", "unit", "aggregation", "comparison", "threshold", "promotion_blocking", "rationale")
    return Slice7GMetric(**_record_data(value, Slice7GMetric, fields, "metric_type"))


def canonical_slice_7g_attempt_ledger_bytes(value: dict[str, Any] | Slice7GAttemptLedger) -> bytes:
    return _canonical_json(_attempt_ledger_data(_validate_attempt_ledger_record(value)))


def slice_7g_attempt_ledger_identity(value: dict[str, Any] | Slice7GAttemptLedger) -> str:
    return hashlib.sha256(LEDGER_IDENTITY_DOMAIN + canonical_slice_7g_attempt_ledger_bytes(value)).hexdigest()


def canonical_slice_7g_attempt_event_bytes(value: dict[str, Any] | Slice7GAttemptEvent) -> bytes:
    return _canonical_json(_attempt_event_data(_validate_attempt_event_record(value)))


def slice_7g_attempt_event_identity(value: dict[str, Any] | Slice7GAttemptEvent) -> str:
    return hashlib.sha256(EVENT_IDENTITY_DOMAIN + canonical_slice_7g_attempt_event_bytes(value)).hexdigest()


def create_slice_7g_initial_attempt_ledger(
    charter: dict[str, Any] | Slice7GCharter, campaign_id: str
) -> Slice7GAttemptLedger:
    record = validate_slice_7g_charter(charter)
    charter_identity = slice_7g_charter_identity(record)
    campaign = _opaque_identifier(campaign_id, "campaign_id", "$.campaign_id")
    campaign_identity = _campaign_identity(charter_identity, campaign)
    return Slice7GAttemptLedger(
        ATTEMPT_LEDGER_SCHEMA_VERSION,
        charter_identity,
        campaign,
        campaign_identity,
        None,
        None,
        0,
        None,
        (),
        (),
        None,
        1,
        0,
        0,
        0,
        False,
        None,
        False,
        None,
        False,
    )


def propose_slice_7g_attempt_event(
    ledger: dict[str, Any] | Slice7GAttemptLedger,
    event_kind: str,
    event_id: str,
    event_timestamp_utc: str,
    *,
    domain_id: int | None = None,
    output_root: str | None = None,
    runtime_authorization_identity: str | None = None,
    campaign_plan: dict[str, Any] | Slice7GCampaignPlan | None = None,
) -> Slice7GAttemptEvent:
    """Derive a side-effect-free proposal; it is not a committed ledger update."""

    current = _validate_attempt_ledger_record(ledger)
    if type(event_kind) is not str:
        _fail("attempt_event_type", "event kind must be an exact string")
    current_identity = slice_7g_attempt_ledger_identity(current)
    allocated = current.domain_allocated
    selected_domain = current.domain_id
    allocated_output = current.output_root
    resulting_attempts = current.consumed_campaign_attempts
    process_consumed = False
    plan_identity: str | None = None
    authorization_identity = current.runtime_authorization_identity
    if event_kind == "domain_and_output_allocated":
        if current.process_start_committed or current.domain_allocated:
            _fail("allocation_already_committed", "domain/output allocation is already committed")
        selected_domain = _domain_id(domain_id, "$.event.domain_id")
        allocated_output = _external_output_root(output_root, "$.event.output_root")
        if runtime_authorization_identity is None:
            _fail("runtime_authorization_required", "allocation proposal requires a separate runtime authorization identity")
        authorization_identity = _digest(runtime_authorization_identity, "$.event.runtime_authorization_identity")
        allocated = True
    elif event_kind == "process_start_commit":
        if current.consumed_campaign_attempts >= current.maximum_campaign_attempts:
            _fail("attempt_exhausted", "the one campaign attempt is already consumed")
        if not current.domain_allocated or domain_id not in (None, current.domain_id) or output_root not in (None, current.output_root):
            _fail("process_start_binding", "process-start commit requires the ledger-bound domain and output root")
        if runtime_authorization_identity not in (None, current.runtime_authorization_identity):
            _fail("process_start_authorization", "process-start authorization identity differs from the allocated ledger")
        if campaign_plan is None:
            _fail("process_start_plan", "process-start commit requires the final validated campaign plan")
        plan = _validate_campaign_plan_record(campaign_plan)
        current_identity_for_plan = slice_7g_attempt_ledger_identity(current)
        if (
            plan.charter_logical_identity != current.charter_logical_identity
            or plan.campaign_identity != current.campaign_identity
            or plan.attempt_ledger_identity != current_identity_for_plan
            or plan.ros_domain_id != current.domain_id
            or plan.campaign_output_root != current.output_root
        ):
            _fail("process_start_plan", "campaign plan does not bind the current allocated ledger")
        plan_identity = slice_7g_campaign_plan_identity(plan)
        resulting_attempts = 1
        process_consumed = True
    elif event_kind == "preflight_failed_before_process_creation":
        if current.process_start_committed:
            _fail("preflight_after_process_start", "preflight failure cannot follow process-start commit")
        if domain_id is not None or output_root is not None or runtime_authorization_identity is not None:
            _fail("preflight_allocation", "preflight failure cannot introduce allocations")
    elif event_kind == "retry_requested":
        _fail("retry_not_authorized", "no retry is authorized")
    else:
        _fail("attempt_event", f"unknown attempt event: {event_kind}")
    return Slice7GAttemptEvent(
        ATTEMPT_EVENT_SCHEMA_VERSION,
        current.charter_logical_identity,
        current.campaign_identity,
        plan_identity,
        authorization_identity,
        _opaque_identifier(event_id, "event_id", "$.event.event_id"),
        event_kind,
        current.revision,
        current_identity,
        current.consumed_campaign_attempts,
        resulting_attempts,
        0,
        0,
        allocated,
        selected_domain,
        allocated,
        allocated_output,
        process_consumed,
        _validate_utc_timestamp(event_timestamp_utc, "$.event.event_timestamp_utc", "event_timestamp"),
    )


def validate_slice_7g_attempt_transition(
    ledger: dict[str, Any] | Slice7GAttemptLedger,
    event: dict[str, Any] | Slice7GAttemptEvent,
    *,
    campaign_plan: dict[str, Any] | Slice7GCampaignPlan | None = None,
) -> Slice7GAttemptLedger:
    """Apply one proposal to its exact predecessor in memory.

    Cross-process exactly-once behavior additionally requires the charter's
    external writer to atomically commit this result against the expected
    predecessor before any process is created.
    """

    current = _validate_attempt_ledger_record(ledger)
    proposal = _validate_attempt_event_record(event)
    current_identity = slice_7g_attempt_ledger_identity(current)
    if proposal.charter_logical_identity != current.charter_logical_identity or proposal.campaign_identity != current.campaign_identity:
        _fail("ledger_subject_mismatch", "event and ledger subjects differ")
    if proposal.expected_revision != current.revision or proposal.expected_predecessor_ledger_identity != current_identity:
        _fail("stale_ledger_predecessor", "event does not target the current ledger revision and identity")
    event_identity = slice_7g_attempt_event_identity(proposal)
    if proposal.event_id in current.applied_event_ids or event_identity in current.applied_event_identities:
        _fail("duplicate_ledger_event", "event ID or identity was already applied")
    if proposal.previous_attempt_count != current.consumed_campaign_attempts:
        _fail("stale_ledger_predecessor", "event previous count differs from current ledger")
    expected = propose_slice_7g_attempt_event(
        current,
        proposal.event_kind,
        proposal.event_id,
        proposal.event_timestamp_utc,
        domain_id=proposal.domain_id if proposal.event_kind == "domain_and_output_allocated" else None,
        output_root=proposal.output_root if proposal.event_kind == "domain_and_output_allocated" else None,
        runtime_authorization_identity=(
            proposal.runtime_authorization_identity if proposal.event_kind == "domain_and_output_allocated" else None
        ),
        campaign_plan=campaign_plan,
    )
    if canonical_slice_7g_attempt_event_bytes(expected) != canonical_slice_7g_attempt_event_bytes(proposal):
        _fail("attempt_event_mismatch", "event fields do not match the deterministic transition")
    return Slice7GAttemptLedger(
        ATTEMPT_LEDGER_SCHEMA_VERSION,
        current.charter_logical_identity,
        current.campaign_id,
        current.campaign_identity,
        proposal.campaign_plan_identity,
        proposal.runtime_authorization_identity,
        current.revision + 1,
        current_identity,
        current.applied_event_identities + (event_identity,),
        current.applied_event_ids + (proposal.event_id,),
        event_identity,
        1,
        proposal.resulting_attempt_count,
        0,
        0,
        proposal.domain_allocated,
        proposal.domain_id,
        proposal.output_root_allocated,
        proposal.output_root,
        proposal.process_start_consumed,
    )


def slice_7g_metric_profile_identity(charter: dict[str, Any] | Slice7GCharter) -> str:
    record = validate_slice_7g_charter(charter)
    projection = {
        "schema_version": METRIC_PROFILE_SCHEMA_VERSION,
        "readiness": _thaw_owned_immutable(record.data["readiness"]),
        "acceptance_contract": _thaw_owned_immutable(record.data["acceptance_contract"]),
    }
    return hashlib.sha256(METRIC_PROFILE_IDENTITY_DOMAIN + _canonical_json(projection)).hexdigest()


def generate_slice_7g_campaign_plan(
    charter: dict[str, Any] | Slice7GCharter,
    ledger: dict[str, Any] | Slice7GAttemptLedger,
) -> Slice7GCampaignPlan:
    record = validate_slice_7g_charter(charter)
    current = _validate_attempt_ledger_record(ledger)
    charter_identity = slice_7g_charter_identity(record)
    if current.charter_logical_identity != charter_identity:
        _fail("plan_charter_mismatch", "ledger does not bind the supplied charter")
    if not current.domain_allocated or current.domain_id is None or current.output_root is None:
        _fail("plan_ledger_unallocated", "campaign plan requires committed domain/output allocation")
    if current.process_start_committed:
        _fail("plan_after_process_start", "campaign plan must be final before process-start commit")
    ledger_identity = slice_7g_attempt_ledger_identity(current)
    metric_identity = slice_7g_metric_profile_identity(record)
    template = tuple(record.data["runtime_template"]["argv_template"])
    cells: list[Slice7GCampaignCell] = []
    for scenario in record.scenarios:
        for seed in EXPECTED_SEEDS:
            cell_id = f"{scenario.scenario_id}.seed_{seed:010d}"
            output_path = f"{current.output_root}/cells/{cell_id}"
            argv = tuple(
                current.campaign_id if token == "<campaign_id>" else
                scenario.source_scenario_id if token == "<source_scenario_id>" else
                str(seed) if token == "<seed>" else
                output_path if token == "<new_external_output_root>" else token
                for token in template
            )
            cells.append(
                Slice7GCampaignCell(
                    CAMPAIGN_CELL_SCHEMA_VERSION, cell_id, charter_identity, current.campaign_id,
                    current.campaign_identity, ledger_identity, scenario.scenario_id,
                    scenario.source_scenario_id, seed, "circular_arc", "curved_lumen_navigation",
                    25.0, "simulation", current.domain_id, current.output_root, output_path, argv,
                    metric_identity, False,
                )
            )
    return validate_slice_7g_campaign_plan(
        Slice7GCampaignPlan(
            CAMPAIGN_PLAN_SCHEMA_VERSION, charter_identity, current.campaign_id,
            current.campaign_identity, ledger_identity, current.domain_id, current.output_root,
            metric_identity, tuple(cells),
        ),
        record,
        current,
    )


def validate_slice_7g_campaign_plan(
    plan: dict[str, Any] | Slice7GCampaignPlan,
    charter: dict[str, Any] | Slice7GCharter,
    ledger: dict[str, Any] | Slice7GAttemptLedger,
) -> Slice7GCampaignPlan:
    supplied = _validate_campaign_plan_record(plan)
    record = validate_slice_7g_charter(charter)
    current = _validate_attempt_ledger_record(ledger)
    expected_charter = slice_7g_charter_identity(record)
    current_identity = slice_7g_attempt_ledger_identity(current)
    expected_ledger = current_identity
    if current.process_start_committed:
        supplied_identity = slice_7g_campaign_plan_identity(supplied)
        if current.campaign_plan_identity != supplied_identity or current.predecessor_ledger_identity != supplied.attempt_ledger_identity:
            _fail("plan_process_commit_mismatch", "committed ledger does not bind this campaign plan and predecessor")
        expected_ledger = supplied.attempt_ledger_identity
    if supplied.charter_logical_identity != expected_charter or supplied.campaign_identity != current.campaign_identity:
        _fail("plan_subject_mismatch", "plan subject bindings differ")
    if supplied.campaign_id != current.campaign_id or supplied.attempt_ledger_identity != expected_ledger:
        _fail("plan_ledger_mismatch", "plan ledger binding differs")
    if supplied.ros_domain_id != current.domain_id:
        _fail("plan_domain_mismatch", "plan must use the single ledger-bound domain")
    if supplied.campaign_output_root != current.output_root:
        _fail("plan_output_root_mismatch", "plan must use the ledger-bound output root")
    expected_metric = slice_7g_metric_profile_identity(record)
    if supplied.metric_profile_identity != expected_metric:
        _fail("plan_metric_profile_mismatch", "plan metric profile differs")
    expected_pairs = [(scenario.scenario_id, seed) for scenario in record.scenarios for seed in EXPECTED_SEEDS]
    observed_pairs = [(cell.scenario_id, cell.seed) for cell in supplied.cells]
    if len(observed_pairs) != len(set(observed_pairs)):
        _fail("duplicate_campaign_cell", "campaign plan contains duplicate scenario/seed cells")
    if observed_pairs != expected_pairs:
        missing = sorted(set(expected_pairs) - set(observed_pairs))
        extra = sorted(set(observed_pairs) - set(expected_pairs))
        _fail("campaign_plan_bijection", f"campaign cells differ: missing={missing} extra={extra}")
    for cell in supplied.cells:
        expected_id = f"{cell.scenario_id}.seed_{cell.seed:010d}"
        if cell.cell_id != expected_id:
            _fail("cell_id_mismatch", "cell ID is not the deterministic scenario/seed identity", cell.cell_id)
        if (
            cell.charter_logical_identity != expected_charter or cell.campaign_id != supplied.campaign_id
            or cell.campaign_identity != supplied.campaign_identity or cell.attempt_ledger_identity != expected_ledger
            or cell.ros_domain_id != supplied.ros_domain_id or cell.campaign_output_root != supplied.campaign_output_root
            or cell.metric_profile_identity != expected_metric
        ):
            _fail("cell_binding_mismatch", "cell bindings differ from the campaign plan", cell.cell_id)
        expected_source = EXPECTED_SOURCE_SCENARIOS[cell.scenario_id]
        if cell.source_scenario_id != expected_source:
            _fail("scenario_source_mismatch", "cell source scenario differs", cell.cell_id)
        expected_output = f"{supplied.campaign_output_root}/cells/{cell.cell_id}"
        if cell.cell_output_path != expected_output:
            _fail("cell_output_path", "cell output path differs from deterministic allocation", cell.cell_id)
        expected_argv = (
            "ctr_run_evaluation", "--experiment-group", supplied.campaign_id,
            "--task", "curved_lumen_navigation", "--curved-lumen-type", "circular_arc",
            "--scenario", expected_source, "--seed", str(cell.seed), "--duration", "25.0",
            "--runtime-mode", "simulation", "--output-root", expected_output,
        )
        if cell.argv != expected_argv:
            _fail("cell_argv_mismatch", "cell argv differs or semantic ordering changed", cell.cell_id)
    return supplied


def canonical_slice_7g_campaign_plan_bytes(value: dict[str, Any] | Slice7GCampaignPlan) -> bytes:
    return _canonical_json(_campaign_plan_data(_validate_campaign_plan_record(value)))


def slice_7g_campaign_plan_identity(value: dict[str, Any] | Slice7GCampaignPlan) -> str:
    return hashlib.sha256(CAMPAIGN_PLAN_IDENTITY_DOMAIN + canonical_slice_7g_campaign_plan_bytes(value)).hexdigest()


def canonical_slice_7g_cell_result_bytes(value: dict[str, Any] | Slice7GCellResult) -> bytes:
    """Serialize a structurally valid cell result; this is not authentication."""

    return _canonical_json(_cell_result_data(_validate_cell_result_record(value)))


def slice_7g_cell_result_identity(value: dict[str, Any] | Slice7GCellResult) -> str:
    return hashlib.sha256(CELL_RESULT_IDENTITY_DOMAIN + canonical_slice_7g_cell_result_bytes(value)).hexdigest()


def validate_slice_7g_campaign_evidence_seal(
    value: dict[str, Any] | Slice7GCampaignEvidenceSeal,
) -> Slice7GCampaignEvidenceSeal:
    """Validate a seal structurally; durable authority requires locked reconciliation."""

    return _validate_campaign_evidence_seal_record(value)


def canonical_slice_7g_campaign_evidence_seal_bytes(
    value: dict[str, Any] | Slice7GCampaignEvidenceSeal,
) -> bytes:
    """Serialize a structural seal without claiming filesystem or lock authority."""

    return _canonical_json(_campaign_evidence_seal_data(_validate_campaign_evidence_seal_record(value)))


def slice_7g_campaign_evidence_snapshot_identity(
    value: dict[str, Any] | Slice7GCampaignEvidenceSeal,
) -> str:
    """Derive the canonical seal/package snapshot identity structurally."""

    seal = _validate_campaign_evidence_seal_record(value)
    raw = _canonical_json(_campaign_evidence_seal_data(seal))
    return _campaign_evidence_snapshot_identity(raw, seal)


def authenticate_slice_7g_cell_evidence_package(
    package_root: str | os.PathLike[str],
    charter: dict[str, Any] | Slice7GCharter,
    ledger: dict[str, Any] | Slice7GAttemptLedger,
    plan: dict[str, Any] | Slice7GCampaignPlan,
) -> Slice7GAuthenticatedCellEvidence:
    """Authenticate one package while retaining root authority through return."""

    record = validate_slice_7g_charter(charter)
    committed = _validate_committed_evidence_ledger(ledger, record)
    validated_plan = validate_slice_7g_campaign_plan(plan, record, committed)
    root_path = _normalize_public_path(
        package_root, "evidence_root_type", "evidence_root_open", require_absolute=True,
    )
    state = _open_evidence_authority(root_path, record, committed, validated_plan)
    try:
        if state.authenticated is None:
            _fail("evidence_internal_state", "authenticated observation was not constructed")
        result = state.authenticated
        _final_evidence_barrier(state)
        return result
    finally:
        state.close()


def reconcile_slice_7g_campaign_results(
    charter: dict[str, Any] | Slice7GCharter,
    plan: dict[str, Any] | Slice7GCampaignPlan,
    ledger: dict[str, Any] | Slice7GAttemptLedger,
    campaign_output_root: str | os.PathLike[str],
    supplied_campaign_result: dict[str, Any] | Slice7GCampaignResult | None = None,
) -> Slice7GCampaignResult:
    """Recompute authority under one locked, ledger-confined campaign seal.

    The shared nonblocking seal lock is a cooperative writer-protocol boundary,
    not a claim of a mathematically atomic filesystem snapshot.  Every
    authorized writer must acquire the corresponding exclusive lock before any
    mutation.  Deliberate same-owner or privileged lock bypass is a governance
    violation outside that accepted writer threat model.
    """

    record = validate_slice_7g_charter(charter)
    current = _validate_committed_evidence_ledger(ledger, record)
    validated_plan = validate_slice_7g_campaign_plan(plan, record, current)
    normalized_root = _normalize_public_path(
        campaign_output_root,
        "campaign_root_type",
        "campaign_root_open",
        require_absolute=True,
    )
    if normalized_root != current.output_root:
        _fail(
            "campaign_root_ledger_mismatch",
            "campaign root must exactly equal the committed ledger output root",
            normalized_root,
        )
    campaign = _open_campaign_evidence_authority(
        normalized_root, record, current, validated_plan,
    )
    states: list[_EvidenceAuthorityState] = []
    try:
        for package in campaign.seal.packages:
            state = _open_campaign_package_authority(
                campaign, package, record, current, validated_plan,
            )
            states.append(state)
            if _required_authenticated(state).package_identity != package.package_identity:
                _fail(
                    "campaign_seal_package_identity_mismatch",
                    "authenticated package identity differs from the finalized seal",
                    package.relative_path,
                )
        _validate_cross_package_authority(states, final=False)
        authenticated = tuple(_required_authenticated(state) for state in states)
        snapshot_identity = _campaign_evidence_snapshot_identity(campaign.seal_raw, campaign.seal)
        recomputed = _recompute_campaign_result(validated_plan, authenticated, snapshot_identity)
        if supplied_campaign_result is not None:
            supplied = _validate_campaign_result_record(supplied_campaign_result)
            if _campaign_result_data(supplied) != _campaign_result_data(recomputed):
                _fail("campaign_result_mismatch", "supplied campaign record differs from authenticated recomputation")
        for state in states:
            _final_evidence_barrier(state)
        _final_campaign_evidence_barrier(campaign, states, snapshot_identity)
        return recomputed
    finally:
        for state in reversed(states):
            state.close()
        campaign.close()


def _recompute_campaign_result(
    validated_plan: Slice7GCampaignPlan,
    authenticated: tuple[Slice7GAuthenticatedCellEvidence, ...],
    campaign_evidence_snapshot_identity: str,
) -> Slice7GCampaignResult:
    """Derive the immutable aggregate before the campaign final barrier."""

    rebuilt = tuple(item.cell_result for item in authenticated)
    plan_by_id = {cell.cell_id: cell for cell in validated_plan.cells}
    result_ids = [item.cell_id for item in rebuilt]
    if len(rebuilt) != 15:
        _fail("campaign_result_count", "campaign requires exactly 15 result records")
    if len(result_ids) != len(set(result_ids)):
        _fail("duplicate_campaign_result", "campaign results contain duplicate cell IDs")
    if set(result_ids) != set(plan_by_id):
        missing = sorted(set(plan_by_id) - set(result_ids))
        extra = sorted(set(result_ids) - set(plan_by_id))
        _fail("campaign_result_bijection", f"campaign results differ: missing={missing} extra={extra}")
    plan_identity = slice_7g_campaign_plan_identity(validated_plan)
    passing = 0
    failed_cells: list[str] = []
    reasons: list[str] = []
    timing_all_pass = True
    ordered_identities: list[str] = []
    ordered_package_identities: list[str] = []
    aggregate_fields = {
        "total_valid_aligned_samples": 0,
        "total_invalid_samples": 0,
        "total_collision_samples": 0,
        "total_safety_faults": 0,
        "total_nonfinite_values": 0,
        "total_missing_required_topics": 0,
        "total_missing_required_results": 0,
        "timing_failure_cell_count": 0,
    }
    by_id = {item.cell_id: item for item in rebuilt}
    package_by_id = {item.cell_result.cell_id: item for item in authenticated}
    for cell in validated_plan.cells:
        result = by_id[cell.cell_id]
        if (
            result.charter_logical_identity != validated_plan.charter_logical_identity
            or result.campaign_identity != validated_plan.campaign_identity
            or result.campaign_plan_identity != plan_identity
            or result.scenario_id != cell.scenario_id or result.source_scenario_id != cell.source_scenario_id
            or result.seed != cell.seed or result.duration_seconds != cell.duration_seconds
            or result.runtime_mode != cell.runtime_mode or result.ros_domain_id != cell.ros_domain_id
            or result.cell_output_path != cell.cell_output_path
        ):
            _fail("campaign_result_binding", "result does not bind its planned cell", cell.cell_id)
        ordered_identities.append(slice_7g_cell_result_identity(result))
        ordered_package_identities.append(package_by_id[cell.cell_id].package_identity)
        cell_reasons = _functional_result_failures(result)
        if cell_reasons:
            failed_cells.append(cell.cell_id)
            reasons.extend(f"{cell.cell_id}:{reason}" for reason in cell_reasons)
        else:
            passing += 1
        timing_all_pass = timing_all_pass and result.timing_pass
        aggregate_fields["total_valid_aligned_samples"] += result.valid_aligned_sample_count
        aggregate_fields["total_invalid_samples"] += result.invalid_sample_count
        aggregate_fields["total_collision_samples"] += result.collision_sample_count
        aggregate_fields["total_safety_faults"] += result.safety_fault_count
        aggregate_fields["total_nonfinite_values"] += result.nonfinite_value_count
        aggregate_fields["total_missing_required_topics"] += result.missing_required_topic_count
        aggregate_fields["total_missing_required_results"] += result.missing_required_result_file_count
        aggregate_fields["timing_failure_cell_count"] += int(not result.timing_pass)
    recomputed = Slice7GCampaignResult(
        CAMPAIGN_RESULT_SCHEMA_VERSION,
        validated_plan.charter_logical_identity,
        validated_plan.campaign_identity,
        plan_identity,
        campaign_evidence_snapshot_identity,
        tuple(ordered_package_identities),
        tuple(ordered_identities),
        15,
        passing,
        tuple(failed_cells),
        tuple(reasons),
        passing == 15,
        timing_all_pass,
        not timing_all_pass,
        aggregate_fields["total_valid_aligned_samples"],
        aggregate_fields["total_invalid_samples"],
        aggregate_fields["total_collision_samples"],
        aggregate_fields["total_safety_faults"],
        aggregate_fields["total_nonfinite_values"],
        aggregate_fields["total_missing_required_topics"],
        aggregate_fields["total_missing_required_results"],
        aggregate_fields["timing_failure_cell_count"],
    )
    return recomputed


def canonical_slice_7g_campaign_result_bytes(value: dict[str, Any] | Slice7GCampaignResult) -> bytes:
    """Structural serialization only; this function never proves authority."""

    return _canonical_json(_campaign_result_data(_validate_campaign_result_record(value)))


def slice_7g_campaign_result_identity(value: dict[str, Any] | Slice7GCampaignResult) -> str:
    return hashlib.sha256(CAMPAIGN_RESULT_IDENTITY_DOMAIN + canonical_slice_7g_campaign_result_bytes(value)).hexdigest()


def verify_authoring_source_snapshot(
    charter: dict[str, Any] | Slice7GCharter, repository_root: str | os.PathLike[str]
) -> bool:
    """Verify snapshot members with component-by-component descriptor confinement."""

    record = validate_slice_7g_charter(charter)
    root_path = _normalize_public_path(repository_root, "snapshot_root_type", "snapshot_root_open")
    try:
        root_fd = _open_directory_path_nofollow(root_path)
    except FileNotFoundError as exc:
        raise Slice7GGovernanceError("snapshot_root_missing", "snapshot root does not exist") from exc
    except NotADirectoryError as exc:
        raise Slice7GGovernanceError("snapshot_root_not_directory", "snapshot root is not a directory") from exc
    except (OSError, ValueError) as exc:
        raise Slice7GGovernanceError("snapshot_root_open", str(exc)) from exc
    seen_inodes: set[tuple[int, int]] = set()
    try:
        try:
            root_info = os.fstat(root_fd)
        except OSError as exc:
            raise Slice7GGovernanceError("snapshot_root_stat", str(exc)) from exc
        if not stat.S_ISDIR(root_info.st_mode):
            _fail("snapshot_root_not_directory", "snapshot root is not a real directory")
        snapshot = record.data["authoring"]["scoped_source_snapshot"]
        for member in snapshot["members"]:
            relative = _safe_relative_path(
                member["path"], f"$.authoring.scoped_source_snapshot.members[{member['path']}].path"
            )
            try:
                info, size, digest = _hash_confined_member(root_fd, relative)
            except OSError as exc:
                raise Slice7GGovernanceError("snapshot_member_io", str(exc), path=relative) from exc
            inode = (info.st_dev, info.st_ino)
            if inode in seen_inodes:
                _fail("snapshot_hardlink_alias", "snapshot members must have unique physical inodes", relative)
            seen_inodes.add(inode)
            if size != member["size"] or digest != member["sha256"]:
                _fail("snapshot_member_mismatch", "source member size or digest differs", relative)
        final_root_info = os.fstat(root_fd)
        stable = lambda item: (
            item.st_dev, item.st_ino, item.st_mode, item.st_nlink, item.st_size,
            item.st_mtime_ns, item.st_ctime_ns,
        )
        if stable(root_info) != stable(final_root_info):
            _fail("snapshot_root_changed", "snapshot root metadata changed during verification")
    finally:
        os.close(root_fd)
    return True


def _validate_top_level(data: dict[str, Any]) -> None:
    expected = {
        "schema_version", "charter_id", "endpoint", "objective", "authoring",
        "slice_7f_closure", "charter_status", "supersedes", "creation_timestamp",
        "governance", "completion_scope", "implementation_requirements", "entry_criteria",
        "campaign", "runtime_template", "readiness", "acceptance_contract", "domain_policy",
        "attempt_budget", "attempt_ledger_contract", "campaign_plan_contract",
        "campaign_evidence_seal_contract",
        "cell_evidence_contract", "campaign_result_contract", "trust_model", "build_test_gate", "evidence_outputs",
        "promotion_contract", "runtime_authority_contract",
    }
    _closed(data, expected, "$")
    _exact_string(data["schema_version"], SCHEMA_VERSION, "schema_version", "$.schema_version")
    _exact_string(data["charter_id"], "slice_7g_simulation_promotion_v7", "charter_id", "$.charter_id")
    _exact_string(data["endpoint"], "simulation_only_promoted_completion", "endpoint", "$.endpoint")
    _nonempty_string(data["objective"], "objective", "$.objective")
    _exact_string(
        data["charter_status"], "APPROVED_SCOPE_NOT_RUNTIME_AUTHORIZATION", "charter_status", "$.charter_status"
    )
    _validate_timestamp(data["creation_timestamp"])
    if _string_list(data["supersedes"], "$.supersedes", allow_empty=False) != [
        "slice_7g_simulation_promotion_v3", "slice_7g_simulation_promotion_v4",
        "slice_7g_simulation_promotion_v5", "slice_7g_simulation_promotion_v6",
    ]:
        _fail("charter_supersedes", "charter predecessor list differs", "$.supersedes")
    _validate_authoring(data["authoring"])
    _validate_slice_7f(data["slice_7f_closure"])
    _validate_governance(data["governance"])
    _validate_scope(data["completion_scope"])
    _validate_requirements(data["implementation_requirements"], EXPECTED_IMPLEMENTATION_GATES, "implementation_gate")
    _validate_requirements(data["entry_criteria"], EXPECTED_ENTRY_GATES, "entry_gate")
    _validate_campaign(data["campaign"])
    _validate_runtime_template(data["runtime_template"])
    _validate_readiness(data["readiness"])
    _validate_acceptance(data["acceptance_contract"])
    _validate_domain(data["domain_policy"])
    _validate_attempt_budget(data["attempt_budget"])
    _validate_attempt_ledger_contract(data["attempt_ledger_contract"])
    _validate_campaign_plan_contract(data["campaign_plan_contract"])
    _validate_campaign_evidence_seal_contract(data["campaign_evidence_seal_contract"])
    _validate_cell_evidence_contract(data["cell_evidence_contract"])
    _validate_campaign_result_contract(data["campaign_result_contract"])
    _validate_trust_model(data["trust_model"])
    _validate_build_gate(data["build_test_gate"])
    _validate_evidence_outputs(data["evidence_outputs"])
    _validate_promotion(data["promotion_contract"])
    _validate_runtime_authority_contract(data["runtime_authority_contract"])


def _validate_runtime_authority_contract(value: Any) -> None:
    obj = _object(value, "runtime_authority_contract", "$.runtime_authority_contract")
    _closed(
        obj,
        {"schema_version", "fixed_paths", "principals", "schemas", "systemd_units",
         "global_budget", "rollback", "output_authority", "process_authority",
         "node_authority", "observation_policy", "overrides", "cleanup_authority",
         "observer_containment", "privileged_protocol", "root_helper_bootstrap"},
        "$.runtime_authority_contract",
    )
    _exact_string(
        obj["schema_version"], RUNTIME_AUTHORITY_CONTRACT_SCHEMA_VERSION,
        "runtime_authority_contract_schema", "$.runtime_authority_contract.schema_version",
    )
    fixed = _object(obj["fixed_paths"], "authority_fixed_paths", "$.runtime_authority_contract.fixed_paths")
    expected_paths = {
        "bootstrap": "/etc/ctr-mppi/slice-7g-authority/bootstrap.json",
        "service_executable": "/usr/libexec/ctr-mppi/ctr-slice7g-authorityd",
        "state_root": "/var/lib/ctr-mppi/slice-7g-authority",
        "socket": "/run/ctr-mppi/slice-7g-authority/authority.sock",
        "authority_runtime_directory": "/run/ctr-mppi/slice-7g-authority",
        "installed_runtime_parent": "/opt/ctr-mppi/slice-7g",
        "output_parent": AUTHORITY_OUTPUT_PARENT,
        "global_lease_registry": AUTHORITY_OUTPUT_PARENT + "/.ctr_slice_7g_domain_leases",
        "cleanup_authority_executable": "/usr/libexec/ctr-mppi/ctr-slice7g-cleanupd",
        "cleanup_authority_state_root": "/var/lib/ctr-mppi/slice-7g-cleanup-authority",
        "cleanup_authority_runtime_directory": "/run/ctr-mppi/slice-7g-cleanup-authority",
        "cleanup_authority_socket": "/run/ctr-mppi/slice-7g-cleanup-authority/cleanup-authority.sock",
        "cleanup_recovery_socket": "/run/ctr-mppi/slice-7g-cleanup-authority/cleanup-recovery.sock",
        "observer_supervisor_executable": "/usr/libexec/ctr-mppi/ctr-slice7g-observerd",
        "observer_supervisor_runtime_directory": "/run/ctr-mppi/slice-7g-observer-supervisor",
        "observer_supervisor_socket": "/run/ctr-mppi/slice-7g-observer-supervisor/observer-supervisor.sock",
    }
    _closed(fixed, set(expected_paths), "$.runtime_authority_contract.fixed_paths")
    for key, expected in expected_paths.items():
        _exact_string(fixed[key], expected, "authority_fixed_path", f"$.runtime_authority_contract.fixed_paths.{key}")

    principals = _object(obj["principals"], "authority_principals", "$.runtime_authority_contract.principals")
    _closed(
        principals,
        {"authority_account", "authority_group", "campaign_account", "runtime_group",
         "authority_uid", "authority_gid", "campaign_uid", "runtime_gid",
         "numeric_ids_future_provisioning_bound", "campaign_supplementary_groups",
         "observer_account", "observer_uid", "observer_gid", "observer_supplementary_groups",
         "recovery_account", "recovery_uid", "recovery_gid"},
        "$.runtime_authority_contract.principals",
    )
    for field, expected in {
        "authority_account": "ctr7g-authority", "authority_group": "ctr7g-authority",
        "campaign_account": "ctr7g-campaign", "runtime_group": "ctr7g-runtime",
        "observer_account": "ctr7g-observer", "recovery_account": "ctr7g-recovery",
    }.items():
        _exact_string(principals[field], expected, "authority_principal", f"$.runtime_authority_contract.principals.{field}")
    for field in (
        "authority_uid", "authority_gid", "campaign_uid", "runtime_gid",
        "observer_uid", "observer_gid", "recovery_uid", "recovery_gid",
    ):
        if principals[field] is not None:
            _fail("authority_numeric_identity", "numeric identities are future provisioning-bound", f"$.runtime_authority_contract.principals.{field}")
    _exact_bool(principals["numeric_ids_future_provisioning_bound"], True, "authority_numeric_identity", "$.runtime_authority_contract.principals.numeric_ids_future_provisioning_bound")
    if _list(principals["campaign_supplementary_groups"], "campaign_supplementary_groups", "$.runtime_authority_contract.principals.campaign_supplementary_groups"):
        _fail("campaign_supplementary_groups", "campaign supplementary-group set must be empty", "$.runtime_authority_contract.principals.campaign_supplementary_groups")
    if _list(principals["observer_supplementary_groups"], "observer_supplementary_groups", "$.runtime_authority_contract.principals.observer_supplementary_groups"):
        _fail("observer_supplementary_groups", "observer supplementary-group set must be empty", "$.runtime_authority_contract.principals.observer_supplementary_groups")

    schemas = _object(obj["schemas"], "authority_schemas", "$.runtime_authority_contract.schemas")
    expected_schemas = {
        "authority_bootstrap": "ctr-slice-7g-authority-bootstrap-3",
        "installed_runtime_manifest": "ctr-slice-7g-installed-runtime-manifest-3",
        "isolated_build_test_approval": "ctr-slice-7g-isolated-build-test-approval-1",
        "runtime_authorization": "ctr-slice-7g-runtime-authorization-3",
        "process_manifest": "ctr-slice-7g-process-manifest-2",
        "environment_manifest": "ctr-slice-7g-environment-manifest-1",
        "global_attempt_budget": "ctr-slice-7g-global-attempt-budget-4",
        "runtime_authority_request": "ctr-slice-7g-runtime-authority-request-4",
        "runtime_authority_receipt": "ctr-slice-7g-runtime-authority-receipt-4",
        "runtime_authority_revocation": "ctr-slice-7g-runtime-authority-revocation-1",
        "observation_session": "ctr-slice-7g-observation-session-3",
        "ros_graph_observation_receipt": "ctr-slice-7g-ros-graph-observation-receipt-3",
        "four_source_domain_observation": "ctr-slice-7g-four-source-domain-observation-4",
        "global_lease_observation": "ctr-slice-7g-global-lease-observation-2",
        "cleanup_authority_revision": "ctr-slice-7g-cleanup-authority-revision-1",
        "cleanup_authority_anchor": "ctr-slice-7g-cleanup-authority-anchor-1",
        "cleanup_authority_head": "ctr-slice-7g-cleanup-authority-head-1",
        "privileged_helper_request": "ctr-slice-7g-privileged-helper-request-1",
        "privileged_helper_receipt": "ctr-slice-7g-privileged-helper-receipt-1",
        "observer_containment_receipt": "ctr-slice-7g-observer-containment-receipt-2",
        "privileged_service_manifest": "ctr-slice-7g-privileged-service-manifest-1",
        "cleanup_recovery_authorization": "ctr-slice-7g-cleanup-recovery-authorization-2",
        "cleanup_recovery_provider_receipt": "ctr-slice-7g-cleanup-recovery-provider-receipt-1",
        "cleanup_recovery_observation": "ctr-slice-7g-cleanup-recovery-observation-1",
    }
    if schemas != expected_schemas:
        _fail("authority_schemas", "authority schema inventory differs", "$.runtime_authority_contract.schemas")

    units = _object(obj["systemd_units"], "systemd_units", "$.runtime_authority_contract.systemd_units")
    expected_units = {
        "authority": "ctr-slice7g-authority.service",
        "campaign": "ctr-slice7g-campaign.service",
        "revocation_path": "ctr-slice7g-revocation.path",
        "revocation_service": "ctr-slice7g-revocation.service",
        "campaign_kill_mode": "control-group",
        "campaign_delegate": False,
        "cleanup_authority": "ctr-slice7g-cleanup-authority.service",
        "cleanup_authority_delegate": False,
        "observer_supervisor": "ctr-slice7g-observer-supervisor.service",
        "observer_supervisor_delegate": True,
        "observer_child_delegate": False,
        "postcommit_revocation": "mandatory",
    }
    if units != expected_units:
        _fail("systemd_units", "systemd authority unit contract differs", "$.runtime_authority_contract.systemd_units")

    budget = _object(obj["global_budget"], "global_budget", "$.runtime_authority_contract.global_budget")
    expected_budget = {
        "scope": "all_slice_7g_campaign_ids_output_roots_domains_authorizations_and_process_restarts",
        "initial_state": "UNCONSUMED", "maximum_attempts": 1, "retries": 0,
        "revision_zero_provisioned_externally": True, "prepare_consumes_attempt": False,
        "commit_consumes_attempt_permanently": True,
        "campaign_project_children_before_commit": 0,
        "other_precommit_project_ros_children": 0,
        "allowed_transitions": ["UNCONSUMED_TO_COMMITTED", "COMMITTED_TO_COMPLETED", "COMMITTED_TO_FAILED_AFTER_COMMIT"],
    }
    if budget != expected_budget:
        _fail("global_budget", "global exactly-once budget contract differs", "$.runtime_authority_contract.global_budget")

    observer = _object(
        obj["observation_policy"], "observation_policy",
        "$.runtime_authority_contract.observation_policy",
    )
    expected_observer = {
        "authority_owner": "root_observer_supervisor",
        "cleanup_anchor_owner": "root_cleanup_authority",
        "observer_class": "PRECOMMIT_ROS_GRAPH_OBSERVER",
        "executable": "/opt/ros/humble/bin/ros2",
        "argv": ["node", "list", "--no-daemon"],
        "shell": False,
        "timeout_seconds": 10.0,
        "maximum_stdout_bytes": 1_048_576,
        "maximum_stderr_bytes": 1_048_576,
        "maximum_precommit_observers": 100,
        "maximum_postcommit_observers": 1,
        "maximum_transaction_observers": 101,
        "concurrency": 1,
        "retries": 0,
        "unexpected_descendants": 0,
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
        "global_lease_registry": AUTHORITY_OUTPUT_PARENT + "/.ctr_slice_7g_domain_leases",
        "global_lease_lock": "registry.lock",
        "global_lease_clear_required": True,
        "cleanup_guard_durable_nonconsuming": True,
        "cleanup_guard_created_before_process": True,
        "cleanup_quarantine_survives_restart": True,
        "cleanup_recovery_production_available": False,
        "dedicated_process_session_required": True,
        "exclusive_cgroup_before_exec": True,
        "leader_reaped_after_provenance_and_cleanup": True,
        "leaf_cgroup_grammar": "/system.slice/ctr-slice7g-observer-supervisor.service/observer-[0-9]{20}-[0-9a-f]{32}",
        "sealed_output_memfd_count": 2,
        "setsid_and_double_fork_escape_prevented_by_cgroup": True,
        "postexec_identity_reconciliation_required": True,
    }
    if observer != expected_observer:
        _fail(
            "observation_policy", "narrow ROS graph observer policy differs",
            "$.runtime_authority_contract.observation_policy",
        )

    cleanup = _object(
        obj["cleanup_authority"], "cleanup_authority",
        "$.runtime_authority_contract.cleanup_authority",
    )
    if cleanup != {
        "anchor_mode": 0o400,
        "attempt_consumption": False,
        "directories": ["revisions", "anchors", "heads"],
        "directory_mode": 0o700,
        "filename_grammars": {
            "anchor": "anchor-[0-9]{20}.json",
            "head": "head-[0-9]{20}.json",
            "revision": "revision-[0-9]{20}.json",
        },
        "head_mode": 0o400,
        "lock_mode": 0o600,
        "lock_name": "ledger.lock",
        "lock_protocol": "exclusive_nonblocking_flock",
        "owner": "root:root",
        "recovery_appends_successor": True,
        "revision_anchor_head_triple_per_transition": True,
        "revision_mode": 0o400,
        "state_root_mode": 0o700,
        "states": [
            "ACTIVE_UNBOUND", "ACTIVE_BOUND", "CLEARED", "QUARANTINED", "RECOVERED",
        ],
        "writer": "ctr-slice7g-cleanup-authority.service",
        "write_sequence": "revision_fsync_anchor_fsync_head_fsync_directories_root_reauthenticate",
        "writes_by_ctr7g_authority": False,
    }:
        _fail("cleanup_authority", "root cleanup authority contract differs", "$.runtime_authority_contract.cleanup_authority")

    containment = _object(
        obj["observer_containment"], "observer_containment",
        "$.runtime_authority_contract.observer_containment",
    )
    if containment != {
        "child_cgroup_delegated": False,
        "cleanup_final_action": "cgroup.kill_or_authenticated_SIGKILL",
        "cleanup_membership_source": "complete_leaf_cgroup_procs",
        "exclusive_leaf_per_invocation": True,
        "leaf_removed_after_stable_empty_and_dds_barrier": True,
        "pre_exec_start_barrier": True,
        "principal": "ctr7g-observer:ctr7g-observer",
        "supplementary_groups": [],
        "supervisor_cgroup": "/system.slice/ctr-slice7g-observer-supervisor.service",
        "supervisor_delegated": True,
        "supervisor_principal": "root",
        "untrusted_exec_after_active_bound": True,
        "dedicated_process_session_before_release": True,
    }:
        _fail("observer_containment", "observer containment contract differs", "$.runtime_authority_contract.observer_containment")

    privileged = _object(
        obj["privileged_protocol"], "privileged_protocol",
        "$.runtime_authority_contract.privileged_protocol",
    )
    if privileged != {
        "address_family": "AF_UNIX",
        "arbitrary_argv_allowed": False,
        "arbitrary_environment_allowed": False,
        "arbitrary_executable_allowed": False,
        "arbitrary_path_pid_signal_or_cgroup_allowed": False,
        "connection_concurrency": 8,
        "frame_header": "four_byte_big_endian_length_in_one_packet",
        "frames_per_connection": 128,
        "maximum_frame_bytes": 262_144,
        "maximum_transferred_fds": 2,
        "operations": [
            "CLEANUP_STATE_QUERY", "CLEANUP_REVISION_APPEND", "OBSERVE_START",
            "OBSERVE_STATUS", "OBSERVE_CANCEL_AND_CLEANUP", "RECOVERY_OBSERVE",
            "RECOVERY_COMMIT",
        ],
        "peer_cgroup_required": True,
        "peer_pid_start_time_required": True,
        "sequence_starts_at_zero_and_contiguous": True,
        "so_peercred_required": True,
        "socket_type": "SOCK_SEQPACKET",
        "response_request_echo_binding_required": True,
        "completed_operation_replay_rejected": True,
        "cross_connection_replay_rejected_within_generation": True,
        "peer_credential_errors_normalized": True,
    }:
        _fail("privileged_protocol", "privileged helper protocol differs", "$.runtime_authority_contract.privileged_protocol")

    bootstrap = _object(
        obj["root_helper_bootstrap"], "root_helper_bootstrap",
        "$.runtime_authority_contract.root_helper_bootstrap",
    )
    if bootstrap != {
        "schema_version": "ctr-slice-7g-authority-bootstrap-3",
        "authority_evidence_read_capability": "CAP_DAC_READ_SEARCH",
        "socket_group_assignment_capability": "CAP_CHOWN",
        "authority_evidence_root_mode": 0o700,
        "record_owner": "root:root", "record_mode": 0o444,
        "record_single_link": True, "authority_writable": False,
        "broad_dac_override_allowed": False,
        "installed_root_selected_by_root_record": True,
        "authority_manifest_selects_root_code": False,
        "complete_privileged_module_inventory": True,
        "complete_service_executable_inventory": True,
        "observer_executable_and_interpreter_bound": True,
        "descriptor_relative_nofollow": True,
        "final_path_inode_barrier": True,
        "same_byte_inode_replacement_rejected": True,
    }:
        _fail("root_helper_bootstrap", "root-helper bootstrap trust differs", "$.runtime_authority_contract.root_helper_bootstrap")

    rollback = _object(obj["rollback"], "rollback", "$.runtime_authority_contract.rollback")
    if rollback != {
        "precommit_owned_residue": 0, "ordinary_exception": True, "base_exception": True,
        "all_cleanup_steps_attempted": True, "primary_failure_preserved": True,
        "path_inode_reconciliation": True, "postcommit_attempt_restoration": False,
    }:
        _fail("precommit_rollback", "pre-commit rollback contract differs", "$.runtime_authority_contract.rollback")

    output = _object(obj["output_authority"], "output_authority", "$.runtime_authority_contract.output_authority")
    if output != {
        "acl_policy_schema": "ctr-slice-7g-output-parent-acl-policy-1",
        "acl_policy_identity": "e66b7103b47263c91f94a79db381fdeafbb96439f008c4ab8d7f0b8845ca12fb",
        "authority_creates_campaign_root": True, "campaign_parent_create_remove_list": False,
        "campaign_root_authority_owned": True, "cell_outputs_narrowly_delegated": True,
        "descriptors_retained_through_final_barrier": True, "member_reads_after_final_barrier": False,
    }:
        _fail("output_authority", "output authority contract differs", "$.runtime_authority_contract.output_authority")

    process = _object(obj["process_authority"], "process_authority", "$.runtime_authority_contract.process_authority")
    if process != {
        "shell": False, "caller_path_resolution": False, "caller_environment_inheritance": False,
        "caller_working_directory": False, "absolute_authenticated_executables": True,
        "installed_runtime_only": True, "campaign_cgroup": "/system.slice/ctr-slice7g-campaign.service",
        "all_descendants_authenticated": True, "timeout_values_authorization_bound": True,
    }:
        _fail("process_authority", "closed process authority differs", "$.runtime_authority_contract.process_authority")

    nodes = _object(obj["node_authority"], "node_authority", "$.runtime_authority_contract.node_authority")
    if nodes != {
        "exact_node_count": 7,
        "exact_nodes": ["/parameter_validator", "/ctr_simulator", "/safety_supervisor", "/mppi_controller", "/reference_manager", "/evaluation_node", "/ctr_run_evaluation_monitor"],
        "safety_supervisor_owned_child_required": True,
        "safety_supervisor_ready_fault_free_required": True,
        "command_route": ["/ctr/mppi_command", "/safety_supervisor", "/ctr/safe_command", "/ctr_simulator"],
    }:
        _fail("node_authority", "exact ROS node authority differs", "$.runtime_authority_contract.node_authority")

    overrides = _object(obj["overrides"], "authority_overrides", "$.runtime_authority_contract.overrides")
    expected_overrides = {
        "caller_authority_path", "caller_installed_runtime_path", "caller_executable",
        "caller_environment", "caller_working_directory", "caller_process_provider", "caller_uid_gid",
        "caller_observation_receipts",
    }
    _closed(overrides, expected_overrides, "$.runtime_authority_contract.overrides")
    for field in expected_overrides:
        _exact_bool(overrides[field], False, "authority_override", f"$.runtime_authority_contract.overrides.{field}")


def _validate_authoring(value: Any) -> None:
    obj = _object(value, "authoring", "$.authoring")
    _closed(
        obj,
        {"branch", "head", "dirty_worktree_is_authoring_baseline", "post_implementation_snapshot_required", "scoped_source_snapshot"},
        "$.authoring",
    )
    _exact_string(obj["branch"], EXPECTED_BRANCH, "authoring_branch", "$.authoring.branch")
    _exact_string(obj["head"], EXPECTED_HEAD, "authoring_head", "$.authoring.head")
    _exact_bool(obj["dirty_worktree_is_authoring_baseline"], True, "authoring_baseline", "$.authoring.dirty_worktree_is_authoring_baseline")
    _exact_bool(obj["post_implementation_snapshot_required"], True, "source_snapshot_gate", "$.authoring.post_implementation_snapshot_required")
    snapshot = _object(obj["scoped_source_snapshot"], "snapshot", "$.authoring.scoped_source_snapshot")
    _closed(snapshot, {"algorithm", "projection_schema", "identity", "members"}, "$.authoring.scoped_source_snapshot")
    _exact_string(snapshot["algorithm"], SNAPSHOT_IDENTITY_ALGORITHM, "snapshot_algorithm", "$.authoring.scoped_source_snapshot.algorithm")
    _exact_string(snapshot["projection_schema"], SNAPSHOT_SCHEMA_VERSION, "snapshot_schema", "$.authoring.scoped_source_snapshot.projection_schema")
    _digest(snapshot["identity"], "$.authoring.scoped_source_snapshot.identity")
    members = _list(snapshot["members"], "snapshot_members", "$.authoring.scoped_source_snapshot.members")
    if not members:
        _fail("snapshot_members", "snapshot must retain at least one member", "$.authoring.scoped_source_snapshot.members")
    normalized: list[dict[str, Any]] = []
    seen: set[str] = set()
    for index, item in enumerate(members):
        path = f"$.authoring.scoped_source_snapshot.members[{index}]"
        member = _object(item, "snapshot_member", path)
        _closed(member, {"path", "size", "sha256"}, path)
        relative = _safe_relative_path(member["path"], f"{path}.path")
        if relative in seen:
            _fail("snapshot_duplicate_path", "snapshot paths must be unique", f"{path}.path")
        seen.add(relative)
        _nonnegative_int(member["size"], "snapshot_size", f"{path}.size")
        _digest(member["sha256"], f"{path}.sha256")
        normalized.append({"path": relative, "size": member["size"], "sha256": member["sha256"]})
    if [item["path"] for item in normalized] != sorted(seen):
        _fail("snapshot_order", "snapshot members must be sorted by path", "$.authoring.scoped_source_snapshot.members")
    projection = {"schema_version": SNAPSHOT_SCHEMA_VERSION, "members": normalized}
    expected_identity = hashlib.sha256(SNAPSHOT_IDENTITY_DOMAIN + _canonical_json(projection)).hexdigest()
    if snapshot["identity"] != expected_identity:
        _fail("snapshot_identity", "snapshot logical identity does not match members", "$.authoring.scoped_source_snapshot.identity")


def _validate_slice_7f(value: Any) -> None:
    obj = _object(value, "slice_7f_closure", "$.slice_7f_closure")
    _closed(obj, {"path", "size", "sha256", "decision", "state", "scope"}, "$.slice_7f_closure")
    _exact_string(obj["path"], EXPECTED_SLICE_7F_CLOSURE, "slice_7f_path", "$.slice_7f_closure.path")
    _exact_int(obj["size"], 5580, "slice_7f_size", "$.slice_7f_closure.size")
    _exact_string(obj["sha256"], EXPECTED_SLICE_7F_SHA256, "slice_7f_digest", "$.slice_7f_closure.sha256")
    _exact_string(obj["decision"], "APPROVED", "slice_7f_decision", "$.slice_7f_closure.decision")
    _exact_string(obj["state"], "CLOSED", "slice_7f_state", "$.slice_7f_closure.state")
    _exact_string(obj["scope"], "static/offline evidence only", "slice_7f_scope", "$.slice_7f_closure.scope")


def _validate_governance(value: Any) -> None:
    obj = _object(value, "governance", "$.governance")
    fields = {
        "execution_authorized", "launchable", "runtime_attempt_allocated", "domain_allocated",
        "output_root_allocated", "physical_hardware_claim", "real_time_performance_claim",
    }
    _closed(obj, fields, "$.governance")
    if obj["execution_authorized"]:
        _fail("runtime_authorized", "charter cannot authorize runtime", "$.governance.execution_authorized")
    if obj["launchable"]:
        _fail("launchable", "charter cannot be launchable", "$.governance.launchable")
    if obj["physical_hardware_claim"]:
        _fail("physical_hardware_claim", "simulation charter cannot claim physical readiness", "$.governance.physical_hardware_claim")
    if obj["real_time_performance_claim"]:
        _fail("real_time_claim", "simulation charter cannot claim real-time performance", "$.governance.real_time_performance_claim")
    for field in fields:
        _exact_bool(obj[field], False, "governance_flag", f"$.governance.{field}")


def _validate_scope(value: Any) -> None:
    obj = _object(value, "completion_scope", "$.completion_scope")
    _closed(obj, {"included", "excluded", "limitations"}, "$.completion_scope")
    expected = {
        "included": EXPECTED_SCOPE_INCLUDED,
        "excluded": EXPECTED_SCOPE_EXCLUDED,
        "limitations": EXPECTED_SCOPE_LIMITATIONS,
    }
    for field, required in expected.items():
        observed = frozenset(_string_list(obj[field], f"$.completion_scope.{field}", allow_empty=False))
        if observed != required:
            _fail("completion_scope", f"{field} scope differs", f"$.completion_scope.{field}")


def _validate_requirements(value: Any, required_ids: frozenset[str], code: str) -> None:
    items = _list(value, code, f"$.{code}")
    seen: set[str] = set()
    for index, item in enumerate(items):
        path = f"$.{code}[{index}]"
        obj = _object(item, code, path)
        _closed(obj, {"id", "requirement", "required_before_runtime_authorization"}, path)
        identifier = _nonempty_string(obj["id"], code, f"{path}.id")
        if identifier in seen:
            _fail(f"duplicate_{code}", "requirement identifiers must be unique", f"{path}.id")
        seen.add(identifier)
        _nonempty_string(obj["requirement"], code, f"{path}.requirement")
        _exact_bool(obj["required_before_runtime_authorization"], True, code, f"{path}.required_before_runtime_authorization")
    if seen != set(required_ids):
        _fail(f"missing_{code}", f"required identifiers differ: missing={sorted(required_ids-seen)} extra={sorted(seen-required_ids)}")


def _validate_campaign(value: Any) -> None:
    obj = _object(value, "campaign", "$.campaign")
    _closed(
        obj,
        {"task", "geometry_profile", "scenarios", "seeds", "duration_seconds", "run_cell_count", "one_governed_campaign"},
        "$.campaign",
    )
    _exact_string(obj["task"], "curved_lumen_navigation", "campaign_task", "$.campaign.task")
    _exact_string(obj["geometry_profile"], "circular_arc", "geometry_profile", "$.campaign.geometry_profile")
    scenarios = _list(obj["scenarios"], "scenarios", "$.campaign.scenarios")
    seen: dict[str, str] = {}
    for index, item in enumerate(scenarios):
        path = f"$.campaign.scenarios[{index}]"
        scenario = _object(item, "scenario", path)
        _closed(scenario, {"scenario_id", "source_scenario_id", "geometry_profile"}, path)
        identifier = _nonempty_string(scenario["scenario_id"], "scenario", f"{path}.scenario_id")
        if identifier in seen:
            _fail("duplicate_scenario", "scenario identifiers must be unique", f"{path}.scenario_id")
        seen[identifier] = _nonempty_string(scenario["source_scenario_id"], "scenario", f"{path}.source_scenario_id")
        _exact_string(scenario["geometry_profile"], "circular_arc", "scenario_geometry", f"{path}.geometry_profile")
    if seen != EXPECTED_SOURCE_SCENARIOS:
        _fail("scenario_matrix", "required circular-arc scenarios differ", "$.campaign.scenarios")
    seeds = _list(obj["seeds"], "seeds", "$.campaign.seeds")
    if any(type(item) is not int for item in seeds):
        _fail("seed_type", "seeds must be exact integers", "$.campaign.seeds")
    if len(seeds) != len(set(seeds)):
        _fail("duplicate_seed", "seeds must be unique", "$.campaign.seeds")
    if tuple(seeds) != EXPECTED_SEEDS:
        _fail("seed_matrix", "seed matrix must be 11,22,33,44,55", "$.campaign.seeds")
    _exact_number(obj["duration_seconds"], 25.0, "campaign_duration", "$.campaign.duration_seconds")
    _exact_int(obj["run_cell_count"], 15, "run_cell_count", "$.campaign.run_cell_count")
    _exact_bool(obj["one_governed_campaign"], True, "campaign_semantics", "$.campaign.one_governed_campaign")


def _validate_runtime_template(value: Any) -> None:
    obj = _object(value, "runtime_template", "$.runtime_template")
    expected = {
        "platform", "entrypoint", "argv_template", "environment_template", "supported_cli_arguments",
        "required_child_launch_arguments", "required_effective_configuration", "placeholder_contract",
        "launchable", "execution_authorized", "notes",
    }
    _closed(obj, expected, "$.runtime_template")
    _exact_string(obj["platform"], "ROS 2 Humble on Ubuntu 22.04", "runtime_platform", "$.runtime_template.platform")
    _exact_string(obj["entrypoint"], "ctr_run_evaluation", "runtime_entrypoint", "$.runtime_template.entrypoint")
    argv = _string_list(obj["argv_template"], "$.runtime_template.argv_template", allow_empty=False)
    required_argv = [
        "ctr_run_evaluation", "--experiment-group", "<campaign_id>",
        "--task", "curved_lumen_navigation", "--curved-lumen-type", "circular_arc",
        "--scenario", "<source_scenario_id>", "--seed", "<seed>",
        "--duration", "25.0", "--runtime-mode", "simulation",
        "--output-root", "<new_external_output_root>",
    ]
    if argv != required_argv:
        _fail("runtime_argv", "prospective argv template differs from the supported runner contract", "$.runtime_template.argv_template")
    supported = _string_list(obj["supported_cli_arguments"], "$.runtime_template.supported_cli_arguments", allow_empty=False)
    if set(supported) != {token for token in required_argv if token.startswith("--")}:
        _fail("runtime_cli_arguments", "supported CLI argument set differs", "$.runtime_template.supported_cli_arguments")
    environment = _object(obj["environment_template"], "runtime_environment", "$.runtime_template.environment_template")
    _closed(environment, {"ROS_DISTRO", "ROS_DOMAIN_ID"}, "$.runtime_template.environment_template")
    _exact_string(environment["ROS_DISTRO"], "humble", "runtime_environment", "$.runtime_template.environment_template.ROS_DISTRO")
    _exact_string(environment["ROS_DOMAIN_ID"], "<allocated_domain>", "runtime_environment", "$.runtime_template.environment_template.ROS_DOMAIN_ID")
    child = set(_string_list(obj["required_child_launch_arguments"], "$.runtime_template.required_child_launch_arguments", allow_empty=False))
    required_child = {
        "runtime_mode:=simulation", "start_evaluation:=true", "enable_cylindrical_lumen:=false",
        "enable_curved_lumen:=true", "curved_lumen_type:=circular_arc", "tactile_enabled:=true",
        "start_safety_supervisor:=true", "mppi_publish_safe_for_simulation:=false",
        "publish_safe_command_for_simulation:=false",
    }
    if child != required_child:
        _fail("runtime_child_bindings", "required child launch bindings differ", "$.runtime_template.required_child_launch_arguments")
    config_requirements = set(_string_list(obj["required_effective_configuration"], "$.runtime_template.required_effective_configuration", allow_empty=False))
    if config_requirements != {
        "tactile.enabled=true", "mppi.tactile.enabled=true", "mppi.weights.force>0.0",
        "safety.tactile_enabled=true", "mppi.weights.shape=0.0", "mppi.weights.obstacle=0.0",
        "mppi.weights.stability=0.0",
    }:
        _fail("runtime_config_bindings", "effective configuration requirements differ", "$.runtime_template.required_effective_configuration")
    placeholders = _object(obj["placeholder_contract"], "placeholder_contract", "$.runtime_template.placeholder_contract")
    _closed(placeholders, {"<campaign_id>", "<source_scenario_id>", "<seed>", "<allocated_domain>", "<new_external_output_root>"}, "$.runtime_template.placeholder_contract")
    for key, explanation in placeholders.items():
        _nonempty_string(explanation, "placeholder_contract", f"$.runtime_template.placeholder_contract.{key}")
    _exact_bool(obj["launchable"], False, "launchable", "$.runtime_template.launchable")
    _exact_bool(obj["execution_authorized"], False, "runtime_authorized", "$.runtime_template.execution_authorized")
    _string_list(obj["notes"], "$.runtime_template.notes", allow_empty=False)


def _validate_readiness(value: Any) -> None:
    obj = _object(value, "readiness", "$.readiness")
    expected = {
        "timeout_seconds", "minimum_stable_samples", "minimum_stable_interval_seconds",
        "q_variation_tolerance", "tip_variation_tolerance_m", "expected_nodes",
        "required_topics", "required_services", "conditions",
    }
    _closed(obj, expected, "$.readiness")
    _exact_number(obj["timeout_seconds"], 10.0, "readiness_timeout", "$.readiness.timeout_seconds")
    _exact_int(obj["minimum_stable_samples"], 10, "readiness_samples", "$.readiness.minimum_stable_samples")
    _exact_number(obj["minimum_stable_interval_seconds"], 0.5, "readiness_interval", "$.readiness.minimum_stable_interval_seconds")
    _exact_number(obj["q_variation_tolerance"], 5.0e-5, "readiness_q_tolerance", "$.readiness.q_variation_tolerance")
    _exact_number(obj["tip_variation_tolerance_m"], 5.0e-5, "readiness_tip_tolerance", "$.readiness.tip_variation_tolerance_m")
    expected_sets = {
        "expected_nodes": EXPECTED_READINESS_NODES,
        "required_topics": EXPECTED_READINESS_TOPICS,
        "required_services": EXPECTED_READINESS_SERVICES,
        "conditions": EXPECTED_READINESS_CONDITIONS,
    }
    for field, required in expected_sets.items():
        observed = frozenset(_string_list(obj[field], f"$.readiness.{field}", allow_empty=False))
        if observed != required:
            code = "readiness_tactile_topic" if field == "required_topics" and "/ctr/tactile/state" not in observed else (
                "readiness_safety_topic" if field == "required_topics" and "/ctr/safety/status" not in observed else "readiness_contract"
            )
            _fail(code, f"{field} set differs", f"$.readiness.{field}")


def _validate_acceptance(value: Any) -> None:
    obj = _object(value, "acceptance_contract", "$.acceptance_contract")
    _closed(
        obj,
        {"descriptive_tracking_tolerance_m", "all_run_cells_must_pass", "timing_failure_alone_blocks_promotion", "timing_policy", "metrics"},
        "$.acceptance_contract",
    )
    _exact_number(obj["descriptive_tracking_tolerance_m"], 0.001, "tracking_tolerance", "$.acceptance_contract.descriptive_tracking_tolerance_m")
    _exact_bool(obj["all_run_cells_must_pass"], True, "acceptance_matrix", "$.acceptance_contract.all_run_cells_must_pass")
    _exact_bool(obj["timing_failure_alone_blocks_promotion"], False, "timing_policy", "$.acceptance_contract.timing_failure_alone_blocks_promotion")
    timing = _object(obj["timing_policy"], "timing_policy", "$.acceptance_contract.timing_policy")
    _closed(
        timing,
        {"configured_deadline_target_percentage", "promotion_blocking", "failure_in_functional_reasons", "non_real_time_label_required_on_failure", "physical_deployment_requires_separate_contract"},
        "$.acceptance_contract.timing_policy",
    )
    _exact_number(timing["configured_deadline_target_percentage"], 5.0, "deadline_threshold", "$.acceptance_contract.timing_policy.configured_deadline_target_percentage")
    _exact_bool(timing["promotion_blocking"], False, "timing_promotion_blocking", "$.acceptance_contract.timing_policy.promotion_blocking")
    _exact_bool(timing["failure_in_functional_reasons"], False, "timing_functional_reason", "$.acceptance_contract.timing_policy.failure_in_functional_reasons")
    _exact_bool(timing["non_real_time_label_required_on_failure"], True, "timing_limitation", "$.acceptance_contract.timing_policy.non_real_time_label_required_on_failure")
    _exact_bool(timing["physical_deployment_requires_separate_contract"], True, "timing_physical_contract", "$.acceptance_contract.timing_policy.physical_deployment_requires_separate_contract")
    metrics = _list(obj["metrics"], "metrics", "$.acceptance_contract.metrics")
    parsed: dict[str, dict[str, Any]] = {}
    for index, item in enumerate(metrics):
        path = f"$.acceptance_contract.metrics[{index}]"
        metric = _object(item, "metric", path)
        _closed(metric, {"name", "unit", "aggregation", "comparison", "threshold", "promotion_blocking", "rationale"}, path)
        name = _nonempty_string(metric["name"], "metric_name", f"{path}.name")
        if name in parsed:
            _fail("duplicate_metric", "metric names must be unique", f"{path}.name")
        unit = _nonempty_string(metric["unit"], "metric_unit", f"{path}.unit")
        if unit not in SAFE_UNITS:
            _fail("metric_unit", f"unsupported unit: {unit}", f"{path}.unit")
        _nonempty_string(metric["aggregation"], "metric_aggregation", f"{path}.aggregation")
        comparison = _nonempty_string(metric["comparison"], "metric_comparison", f"{path}.comparison")
        if comparison not in {"equal", "less_than_or_equal", "greater_than_or_equal", "report_only"}:
            _fail("metric_comparison", f"unsupported comparison: {comparison}", f"{path}.comparison")
        threshold = metric["threshold"]
        if comparison == "report_only":
            if threshold is not None:
                _fail("metric_threshold", "report-only metric threshold must be null", f"{path}.threshold")
        elif type(threshold) not in (bool, int, float) or (type(threshold) is float and not math.isfinite(threshold)):
            _fail("metric_threshold", "threshold must be a finite exact scalar", f"{path}.threshold")
        if type(metric["promotion_blocking"]) is not bool:
            _fail("metric_promotion_type", "promotion_blocking must be a bool", f"{path}.promotion_blocking")
        _nonempty_string(metric["rationale"], "metric_rationale", f"{path}.rationale")
        parsed[name] = metric
    expected = _expected_metrics()
    if set(parsed) != set(expected):
        _fail("metric_set", f"metric set differs: missing={sorted(set(expected)-set(parsed))} extra={sorted(set(parsed)-set(expected))}")
    for name, expected_fields in expected.items():
        for field, expected_value in expected_fields.items():
            if type(parsed[name][field]) is not type(expected_value) or parsed[name][field] != expected_value:
                code = "tracking_threshold" if name in {"steady_state_error", "final_goal_error", "goal_hold_duration"} else (
                    "timing_promotion_blocking" if name == "deadline_overrun_percentage" and field == "promotion_blocking" else "metric_contract"
                )
                _fail(code, f"{name}.{field} must equal {expected_value!r}", f"$.acceptance_contract.metrics.{name}.{field}")


def _expected_metrics() -> dict[str, dict[str, Any]]:
    return {
        "readiness_success": {"unit": "boolean", "aggregation": "all", "comparison": "equal", "threshold": True, "promotion_blocking": True},
        "steady_state_error": {"unit": "m", "aggregation": "maximum_per_cell", "comparison": "less_than_or_equal", "threshold": 0.003, "promotion_blocking": True},
        "final_goal_error": {"unit": "m", "aggregation": "final_per_cell", "comparison": "less_than_or_equal", "threshold": 0.003, "promotion_blocking": True},
        "goal_hold_duration": {"unit": "s", "aggregation": "contiguous_per_cell", "comparison": "greater_than_or_equal", "threshold": 0.5, "promotion_blocking": True},
        "minimum_physical_wall_clearance": {"unit": "m", "aggregation": "minimum_per_cell", "comparison": "greater_than_or_equal", "threshold": 0.0, "promotion_blocking": True},
        "minimum_safety_margin_wall_clearance": {"unit": "m", "aggregation": "minimum_per_cell", "comparison": "greater_than_or_equal", "threshold": 0.002, "promotion_blocking": True},
        "collision_sample_count": {"unit": "count", "aggregation": "sum_per_cell", "comparison": "equal", "threshold": 0, "promotion_blocking": True},
        "safety_fault_count": {"unit": "count", "aggregation": "sum_per_cell", "comparison": "equal", "threshold": 0, "promotion_blocking": True},
        "nonfinite_value_count": {"unit": "count", "aggregation": "sum_per_cell", "comparison": "equal", "threshold": 0, "promotion_blocking": True},
        "missing_required_topic_count": {"unit": "count", "aggregation": "sum_per_cell", "comparison": "equal", "threshold": 0, "promotion_blocking": True},
        "missing_required_result_file_count": {"unit": "count", "aggregation": "sum_per_cell", "comparison": "equal", "threshold": 0, "promotion_blocking": True},
        "valid_aligned_sample_count": {"unit": "count", "aggregation": "minimum_per_cell", "comparison": "greater_than_or_equal", "threshold": 20, "promotion_blocking": True},
        "invalid_sample_percentage": {"unit": "percent", "aggregation": "maximum_per_cell", "comparison": "less_than_or_equal", "threshold": 10.0, "promotion_blocking": True},
        "saturation_percentage": {"unit": "percent", "aggregation": "maximum_per_cell", "comparison": "less_than_or_equal", "threshold": 1.0, "promotion_blocking": True},
        "process_exit_status": {"unit": "exit_code", "aggregation": "all_processes", "comparison": "equal", "threshold": 0, "promotion_blocking": True},
        "deadline_overrun_percentage": {"unit": "percent", "aggregation": "maximum_per_cell", "comparison": "less_than_or_equal", "threshold": 5.0, "promotion_blocking": False},
        "rmse": {"unit": "m", "aggregation": "report_per_cell", "comparison": "report_only", "threshold": None, "promotion_blocking": False},
        "inside_tracking_tolerance_percentage": {"unit": "percent", "aggregation": "report_per_cell", "comparison": "report_only", "threshold": None, "promotion_blocking": False},
    }


def _validate_domain(value: Any) -> None:
    obj = _object(value, "domain_policy", "$.domain_policy")
    _closed(
        obj,
        {"minimum_domain_id", "maximum_domain_id", "domain_allocated", "selected_domain_id", "allocation_stage", "occupancy_checks", "external_attempt_ledger_required", "exactly_one_domain", "one_domain_per_campaign", "ledger_binding_required", "lower_level_override_prohibited", "fail_before_process_on_binding_change", "no_second_domain_after_process_start", "allocation_evidence_required", "release_accounting_required"},
        "$.domain_policy",
    )
    Slice7GDomainPolicy(obj["minimum_domain_id"], obj["maximum_domain_id"], obj["domain_allocated"], obj["selected_domain_id"])
    _exact_string(obj["allocation_stage"], "future_authorized_runtime_preflight", "domain_allocation_stage", "$.domain_policy.allocation_stage")
    checks = set(_string_list(obj["occupancy_checks"], "$.domain_policy.occupancy_checks", allow_empty=False))
    if checks != {"active_processes", "ros_graph", "dds_participants", "external_domain_ledger"}:
        _fail("domain_occupancy_checks", "domain occupancy checks differ", "$.domain_policy.occupancy_checks")
    _exact_bool(obj["external_attempt_ledger_required"], True, "domain_ledger", "$.domain_policy.external_attempt_ledger_required")
    _exact_bool(obj["exactly_one_domain"], True, "domain_count", "$.domain_policy.exactly_one_domain")
    _exact_bool(obj["one_domain_per_campaign"], True, "domain_count", "$.domain_policy.one_domain_per_campaign")
    _exact_bool(obj["ledger_binding_required"], True, "domain_ledger", "$.domain_policy.ledger_binding_required")
    _exact_bool(obj["lower_level_override_prohibited"], True, "domain_override", "$.domain_policy.lower_level_override_prohibited")
    _exact_bool(obj["fail_before_process_on_binding_change"], True, "domain_binding", "$.domain_policy.fail_before_process_on_binding_change")
    _exact_bool(obj["no_second_domain_after_process_start"], True, "domain_count", "$.domain_policy.no_second_domain_after_process_start")
    _exact_bool(obj["allocation_evidence_required"], True, "domain_evidence", "$.domain_policy.allocation_evidence_required")
    _exact_bool(obj["release_accounting_required"], True, "domain_evidence", "$.domain_policy.release_accounting_required")


def _validate_attempt_budget(value: Any) -> None:
    obj = _object(value, "attempt_budget", "$.attempt_budget")
    _closed(
        obj,
        {"maximum_campaigns", "consumed_campaigns", "retries_authorized", "preflight_failure_consumes_attempt", "process_start_consumes_attempt", "process_start_commit_before_process_creation", "atomic_writer_required", "commit_strategy", "pure_transition_is_not_cross_process_enforcement", "additional_attempt_requires_user_authorization"},
        "$.attempt_budget",
    )
    Slice7GAttemptBudget(obj["maximum_campaigns"], obj["consumed_campaigns"], obj["retries_authorized"])
    _exact_bool(obj["preflight_failure_consumes_attempt"], False, "attempt_preflight", "$.attempt_budget.preflight_failure_consumes_attempt")
    _exact_bool(obj["process_start_consumes_attempt"], True, "attempt_process_start", "$.attempt_budget.process_start_consumes_attempt")
    _exact_bool(obj["process_start_commit_before_process_creation"], True, "attempt_commit_order", "$.attempt_budget.process_start_commit_before_process_creation")
    _exact_bool(obj["atomic_writer_required"], True, "attempt_atomic_writer", "$.attempt_budget.atomic_writer_required")
    strategies = set(_string_list(obj["commit_strategy"], "$.attempt_budget.commit_strategy", allow_empty=False))
    if strategies != {"atomic_compare_and_swap", "exclusive_no_replace_creation"}:
        _fail("attempt_commit_strategy", "atomic commit strategies differ", "$.attempt_budget.commit_strategy")
    _exact_bool(obj["pure_transition_is_not_cross_process_enforcement"], True, "attempt_pure_limit", "$.attempt_budget.pure_transition_is_not_cross_process_enforcement")
    _exact_bool(obj["additional_attempt_requires_user_authorization"], True, "attempt_authorization", "$.attempt_budget.additional_attempt_requires_user_authorization")


def _validate_attempt_ledger_contract(value: Any) -> None:
    obj = _object(value, "attempt_ledger_contract", "$.attempt_ledger_contract")
    _closed(obj, {"ledger_schema", "event_schema", "canonical_identity_algorithms", "event_kinds", "required_fields", "commit_requirements"}, "$.attempt_ledger_contract")
    _exact_string(obj["ledger_schema"], ATTEMPT_LEDGER_SCHEMA_VERSION, "ledger_schema", "$.attempt_ledger_contract.ledger_schema")
    _exact_string(obj["event_schema"], ATTEMPT_EVENT_SCHEMA_VERSION, "event_schema", "$.attempt_ledger_contract.event_schema")
    algorithms = _object(obj["canonical_identity_algorithms"], "ledger_algorithms", "$.attempt_ledger_contract.canonical_identity_algorithms")
    _closed(algorithms, {"ledger", "event"}, "$.attempt_ledger_contract.canonical_identity_algorithms")
    _exact_string(algorithms["ledger"], "sha256:ctr-slice-7g-attempt-ledger-canonical-1", "ledger_algorithm", "$.attempt_ledger_contract.canonical_identity_algorithms.ledger")
    _exact_string(algorithms["event"], "sha256:ctr-slice-7g-attempt-event-canonical-1", "event_algorithm", "$.attempt_ledger_contract.canonical_identity_algorithms.event")
    kinds = set(_string_list(obj["event_kinds"], "$.attempt_ledger_contract.event_kinds", allow_empty=False))
    if kinds != {"preflight_failed_before_process_creation", "domain_and_output_allocated", "process_start_commit"}:
        _fail("ledger_event_kinds", "attempt event kinds differ", "$.attempt_ledger_contract.event_kinds")
    fields = set(_string_list(obj["required_fields"], "$.attempt_ledger_contract.required_fields", allow_empty=False))
    required = {
        "schema_version", "charter_logical_identity", "campaign_identity", "ledger_revision",
        "canonical_predecessor_ledger_identity", "campaign_plan_identity", "runtime_authorization_identity", "unique_event_identity", "event_kind",
        "previous_and_resulting_attempt_counts", "maximum_campaign_attempts", "retry_counts",
        "domain_and_output_allocation_state", "process_start_consumption_state", "utc_event_timestamp",
    }
    if fields != required:
        _fail("ledger_required_fields", "attempt ledger/event field set differs", "$.attempt_ledger_contract.required_fields")
    requirements = set(_string_list(obj["commit_requirements"], "$.attempt_ledger_contract.commit_requirements", allow_empty=False))
    expected = {
        "STALE_REVISION_OR_PREDECESSOR_REJECTED", "DUPLICATE_EVENT_IDENTITY_REJECTED",
        "PROCESS_START_COMMITTED_BEFORE_PROCESS_CREATION", "ATOMIC_CAS_OR_EXCLUSIVE_NO_REPLACE_REQUIRED",
        "COMMIT_FAILURE_PROHIBITS_PROCESS_START", "NO_RETRY_AFTER_PROCESS_START",
        "PURE_FUNCTION_DOES_NOT_CLAIM_CROSS_PROCESS_ENFORCEMENT",
    }
    if requirements != expected:
        _fail("ledger_commit_requirements", "ledger commit requirements differ", "$.attempt_ledger_contract.commit_requirements")


def _validate_campaign_plan_contract(value: Any) -> None:
    obj = _object(value, "campaign_plan_contract", "$.campaign_plan_contract")
    _closed(obj, {"schema_version", "cell_schema_version", "generator", "validator", "exact_cell_count", "cartesian_product_required", "single_ledger_domain_required", "single_campaign_output_root_required", "lower_level_domain_allocation_allowed", "runner_limitation"}, "$.campaign_plan_contract")
    _exact_string(obj["schema_version"], CAMPAIGN_PLAN_SCHEMA_VERSION, "plan_schema", "$.campaign_plan_contract.schema_version")
    _exact_string(obj["cell_schema_version"], CAMPAIGN_CELL_SCHEMA_VERSION, "cell_schema", "$.campaign_plan_contract.cell_schema_version")
    _exact_string(obj["generator"], "generate_slice_7g_campaign_plan", "plan_generator", "$.campaign_plan_contract.generator")
    _exact_string(obj["validator"], "validate_slice_7g_campaign_plan", "plan_validator", "$.campaign_plan_contract.validator")
    _exact_int(obj["exact_cell_count"], 15, "campaign_cell_count", "$.campaign_plan_contract.exact_cell_count")
    _exact_bool(obj["cartesian_product_required"], True, "campaign_bijection", "$.campaign_plan_contract.cartesian_product_required")
    _exact_bool(obj["single_ledger_domain_required"], True, "campaign_domain", "$.campaign_plan_contract.single_ledger_domain_required")
    _exact_bool(obj["single_campaign_output_root_required"], True, "campaign_output", "$.campaign_plan_contract.single_campaign_output_root_required")
    _exact_bool(obj["lower_level_domain_allocation_allowed"], False, "campaign_domain", "$.campaign_plan_contract.lower_level_domain_allocation_allowed")
    _exact_string(obj["runner_limitation"], "ctr_run_evaluation supports per-cell options but does not yet orchestrate or authenticate the complete 15-cell campaign", "runner_limitation", "$.campaign_plan_contract.runner_limitation")


def _validate_campaign_evidence_seal_contract(value: Any) -> None:
    obj = _object(value, "campaign_evidence_seal_contract", "$.campaign_evidence_seal_contract")
    fields = {
        "schema_version", "package_record_schema_version", "snapshot_schema_version",
        "snapshot_identity_algorithm", "seal_path", "evidence_root_relative_path",
        "packages_relative_path", "package_relative_path_template", "exact_package_count",
        "final_root_mode", "final_seal_mode", "shared_nonblocking_reader_lock_required",
        "exclusive_writer_lock_required", "lock_unavailable_fails_closed",
        "lock_held_through_reconciliation", "ledger_root_confinement_required",
        "sequential_rehash_is_atomic_snapshot", "lock_bypass_is_governance_violation",
    }
    _closed(obj, fields, "$.campaign_evidence_seal_contract")
    exact_strings = {
        "schema_version": CAMPAIGN_EVIDENCE_SEAL_SCHEMA_VERSION,
        "package_record_schema_version": CAMPAIGN_EVIDENCE_PACKAGE_RECORD_SCHEMA_VERSION,
        "snapshot_schema_version": CAMPAIGN_EVIDENCE_SNAPSHOT_SCHEMA_VERSION,
        "snapshot_identity_algorithm": CAMPAIGN_EVIDENCE_SNAPSHOT_IDENTITY_ALGORITHM,
        "seal_path": CAMPAIGN_EVIDENCE_SEAL_PATH,
        "evidence_root_relative_path": CAMPAIGN_EVIDENCE_ROOT_RELATIVE_PATH,
        "packages_relative_path": CAMPAIGN_EVIDENCE_PACKAGES_RELATIVE_PATH,
        "package_relative_path_template": "packages/<cell_id>",
    }
    for field, expected in exact_strings.items():
        _exact_string(obj[field], expected, "campaign_evidence_seal_contract", f"$.campaign_evidence_seal_contract.{field}")
    _exact_int(obj["exact_package_count"], 15, "campaign_evidence_seal_count", "$.campaign_evidence_seal_contract.exact_package_count")
    _exact_int(obj["final_root_mode"], 0o555, "campaign_evidence_root_mode", "$.campaign_evidence_seal_contract.final_root_mode")
    _exact_int(obj["final_seal_mode"], 0o444, "campaign_evidence_seal_mode", "$.campaign_evidence_seal_contract.final_seal_mode")
    for field in (
        "shared_nonblocking_reader_lock_required", "exclusive_writer_lock_required",
        "lock_unavailable_fails_closed", "lock_held_through_reconciliation",
        "ledger_root_confinement_required", "lock_bypass_is_governance_violation",
    ):
        _exact_bool(obj[field], True, "campaign_evidence_seal_contract", f"$.campaign_evidence_seal_contract.{field}")
    _exact_bool(
        obj["sequential_rehash_is_atomic_snapshot"], False,
        "campaign_evidence_atomicity", "$.campaign_evidence_seal_contract.sequential_rehash_is_atomic_snapshot",
    )


def _validate_campaign_result_contract(value: Any) -> None:
    obj = _object(value, "campaign_result_contract", "$.campaign_result_contract")
    _closed(obj, {"cell_result_schema", "campaign_result_schema", "reconciler", "exact_result_count", "physical_package_authentication_required", "standalone_serialization_is_authority", "caller_authority_booleans_allowed", "all_promotion_blocking_metrics_enforced", "timing_diagnostic_only", "non_real_time_label_required_on_timing_failure", "aggregates_recomputed_from_authenticated_packages", "supplied_record_requires_exact_recomputed_equality", "campaign_evidence_seal_required", "campaign_snapshot_identity_required", "cooperative_lock_protocol_required"}, "$.campaign_result_contract")
    _exact_string(obj["cell_result_schema"], CELL_RESULT_SCHEMA_VERSION, "cell_result_schema", "$.campaign_result_contract.cell_result_schema")
    _exact_string(obj["campaign_result_schema"], CAMPAIGN_RESULT_SCHEMA_VERSION, "campaign_result_schema", "$.campaign_result_contract.campaign_result_schema")
    _exact_string(obj["reconciler"], "reconcile_slice_7g_campaign_results", "campaign_reconciler", "$.campaign_result_contract.reconciler")
    _exact_int(obj["exact_result_count"], 15, "campaign_result_count", "$.campaign_result_contract.exact_result_count")
    for field in ("physical_package_authentication_required", "all_promotion_blocking_metrics_enforced", "timing_diagnostic_only", "non_real_time_label_required_on_timing_failure", "aggregates_recomputed_from_authenticated_packages", "supplied_record_requires_exact_recomputed_equality", "campaign_evidence_seal_required", "campaign_snapshot_identity_required", "cooperative_lock_protocol_required"):
        _exact_bool(obj[field], True, "campaign_result_contract", f"$.campaign_result_contract.{field}")
    _exact_bool(obj["standalone_serialization_is_authority"], False, "campaign_result_authority", "$.campaign_result_contract.standalone_serialization_is_authority")
    _exact_bool(obj["caller_authority_booleans_allowed"], False, "campaign_result_authority", "$.campaign_result_contract.caller_authority_booleans_allowed")


def _validate_cell_evidence_contract(value: Any) -> None:
    obj = _object(value, "cell_evidence_contract", "$.cell_evidence_contract")
    _closed(
        obj,
        {
            "projection_schema", "envelope_schema", "member_schema", "projection_path", "envelope_path",
            "projection_identity_algorithm", "package_identity_algorithm",
            "mandatory_roles", "final_root_mode", "final_file_mode", "unique_inode_required",
            "complete_inventory_required", "descriptor_authentication_required", "identity_recomputed_from_bytes",
            "committed_ledger_required", "runtime_authorization_required", "campaign_plan_required",
            "process_start_event_required", "caller_evidence_identity_trusted", "caller_authority_booleans_trusted",
        },
        "$.cell_evidence_contract",
    )
    _exact_string(obj["projection_schema"], CELL_EVIDENCE_PROJECTION_SCHEMA_VERSION, "evidence_projection_schema", "$.cell_evidence_contract.projection_schema")
    _exact_string(obj["envelope_schema"], CELL_EVIDENCE_ENVELOPE_SCHEMA_VERSION, "evidence_envelope_schema", "$.cell_evidence_contract.envelope_schema")
    _exact_string(obj["member_schema"], CELL_EVIDENCE_MEMBER_SCHEMA_VERSION, "evidence_member_schema", "$.cell_evidence_contract.member_schema")
    _exact_string(obj["projection_path"], EVIDENCE_PROJECTION_PATH, "evidence_projection_path", "$.cell_evidence_contract.projection_path")
    _exact_string(obj["envelope_path"], EVIDENCE_ENVELOPE_PATH, "evidence_envelope_path", "$.cell_evidence_contract.envelope_path")
    _exact_string(obj["projection_identity_algorithm"], CELL_EVIDENCE_PROJECTION_IDENTITY_ALGORITHM, "evidence_projection_algorithm", "$.cell_evidence_contract.projection_identity_algorithm")
    _exact_string(obj["package_identity_algorithm"], CELL_EVIDENCE_PACKAGE_IDENTITY_ALGORITHM, "evidence_package_algorithm", "$.cell_evidence_contract.package_identity_algorithm")
    roles = _object(obj["mandatory_roles"], "evidence_roles", "$.cell_evidence_contract.mandatory_roles")
    _closed(roles, set(MANDATORY_EVIDENCE_ROLE_PATHS), "$.cell_evidence_contract.mandatory_roles")
    if roles != MANDATORY_EVIDENCE_ROLE_PATHS:
        _fail("evidence_role_contract", "mandatory role paths differ", "$.cell_evidence_contract.mandatory_roles")
    _exact_int(obj["final_root_mode"], 0o555, "evidence_root_mode", "$.cell_evidence_contract.final_root_mode")
    _exact_int(obj["final_file_mode"], 0o444, "evidence_file_mode", "$.cell_evidence_contract.final_file_mode")
    for field in (
        "unique_inode_required", "complete_inventory_required", "descriptor_authentication_required",
        "identity_recomputed_from_bytes", "committed_ledger_required", "runtime_authorization_required",
        "campaign_plan_required", "process_start_event_required",
    ):
        _exact_bool(obj[field], True, "evidence_contract", f"$.cell_evidence_contract.{field}")
    _exact_bool(obj["caller_evidence_identity_trusted"], False, "evidence_authority", "$.cell_evidence_contract.caller_evidence_identity_trusted")
    _exact_bool(obj["caller_authority_booleans_trusted"], False, "evidence_authority", "$.cell_evidence_contract.caller_authority_booleans_trusted")


def _validate_trust_model(value: Any) -> None:
    obj = _object(value, "trust_model", "$.trust_model")
    _closed(obj, {"untrusted_inputs", "validated_records", "canonical_charter_authority", "snapshot_authority", "campaign_plan_authority", "attempt_commit_authority", "domain_output_authority", "cell_result_authority", "promotion_authority", "primitive_mapping_boundary", "arbitrary_mapping_hooks_invoked", "legacy_parallel_pipeline_allowed"}, "$.trust_model")
    expected_untrusted = {
        "external_json_and_paths", "caller_supplied_record_instances", "ledger_proposals",
        "runtime_authorization_records", "domain_and_output_allocations", "physical_cell_evidence_packages",
        "physical_campaign_evidence_seal",
    }
    if set(_string_list(obj["untrusted_inputs"], "$.trust_model.untrusted_inputs", allow_empty=False)) != expected_untrusted:
        _fail("trust_model_inputs", "untrusted input set differs", "$.trust_model.untrusted_inputs")
    expected_records = {
        "charter", "scenario", "metric", "domain_policy", "attempt_budget", "attempt_ledger",
        "attempt_event", "campaign_cell", "campaign_plan", "evidence_member",
        "cell_evidence_envelope", "authenticated_cell_evidence", "campaign_evidence_package",
        "campaign_evidence_seal", "cell_result", "campaign_result",
    }
    if set(_string_list(obj["validated_records"], "$.trust_model.validated_records", allow_empty=False)) != expected_records:
        _fail("trust_model_records", "validated record set differs", "$.trust_model.validated_records")
    exact = {
        "canonical_charter_authority": "recomputed canonical bytes and domain-separated logical identity",
        "snapshot_authority": "component-by-component descriptor-relative no-follow traversal beneath the authenticated root",
        "campaign_plan_authority": "repository-owned deterministic 15-cell generator and bijection validator",
        "attempt_commit_authority": "external atomic compare-and-swap or exclusive no-replace ledger commit before process creation",
        "domain_output_authority": "separate runtime-authorization identity plus one committed ledger binding shared by every cell and subprocess",
        "cell_result_authority": "descriptor-authenticated sealed package bytes bound to the committed ledger, runtime authorization, plan, exact cell, and locked campaign seal",
        "promotion_authority": "repository-owned recomputation from one locked ledger-confined seal and exactly 15 authenticated packages plus later independent audit and external promotion decision",
    }
    for field, text in exact.items():
        _exact_string(obj[field], text, "trust_model", f"$.trust_model.{field}")
    _exact_string(obj["primitive_mapping_boundary"], "exact built-in dictionaries detached recursively", "mapping_boundary", "$.trust_model.primitive_mapping_boundary")
    _exact_bool(obj["arbitrary_mapping_hooks_invoked"], False, "mapping_boundary", "$.trust_model.arbitrary_mapping_hooks_invoked")
    _exact_bool(obj["legacy_parallel_pipeline_allowed"], False, "legacy_pipeline", "$.trust_model.legacy_parallel_pipeline_allowed")


def _validate_build_gate(value: Any) -> None:
    obj = _object(value, "build_test_gate", "$.build_test_gate")
    _closed(obj, {"packages", "requirements", "command_templates", "fresh_external_directories_required", "source_tree_cache_allowed"}, "$.build_test_gate")
    packages = set(_string_list(obj["packages"], "$.build_test_gate.packages", allow_empty=False))
    expected = {"ctr_interfaces", "ctr_bringup", "ctr_model", "ctr_mppi_controller", "ctr_tactile", "ctr_sim", "ctr_safety", "ctr_evaluation"}
    if packages != expected:
        _fail("build_packages", "build package set differs", "$.build_test_gate.packages")
    requirements = set(_string_list(obj["requirements"], "$.build_test_gate.requirements", allow_empty=False))
    required = {
        "COLCON_BUILD_SUCCESS", "COLCON_TEST_SUCCESS", "ZERO_TEST_FAILURES", "CACHE_DISABLED_PYTHON_TESTS",
        "INTERFACE_AND_PACKAGE_RESOLUTION", "LAUNCH_DESCRIPTION_STATIC_VALIDATION", "RUNTIME_PLAN_AND_CHARTER_VALIDATION",
        "NO_SOURCE_TREE_CACHE_OR_BYTECODE", "NO_TEMPORARY_RPATH_OR_RUNPATH", "NO_UNRESOLVED_PROJECT_DEPENDENCY",
    }
    if requirements != required:
        _fail("build_requirements", "build/test requirement set differs", "$.build_test_gate.requirements")
    commands = _string_list(obj["command_templates"], "$.build_test_gate.command_templates", allow_empty=False)
    if tuple(commands) != EXPECTED_BUILD_COMMANDS:
        _fail("build_commands", "isolated build/test command templates differ", "$.build_test_gate.command_templates")
    _exact_bool(obj["fresh_external_directories_required"], True, "build_isolation", "$.build_test_gate.fresh_external_directories_required")
    _exact_bool(obj["source_tree_cache_allowed"], False, "build_cache", "$.build_test_gate.source_tree_cache_allowed")


def _validate_evidence_outputs(value: Any) -> None:
    obj = _object(value, "evidence_outputs", "$.evidence_outputs")
    _closed(obj, {"external_parent", "output_root_allocated", "required_artifacts"}, "$.evidence_outputs")
    _exact_string(obj["external_parent"], "/home/ankid/ctr_mppi_evidence/slice_7g", "evidence_parent", "$.evidence_outputs.external_parent")
    _exact_bool(obj["output_root_allocated"], False, "output_root_allocated", "$.evidence_outputs.output_root_allocated")
    artifacts = set(_string_list(obj["required_artifacts"], "$.evidence_outputs.required_artifacts", allow_empty=False))
    required = {
        "immutable_attempt_ledger", "exact_source_snapshot", "build_test_receipts", "runtime_argv_and_environment",
        "domain_allocation_record", "immutable_campaign_plan", "sealed_per_cell_evidence_packages",
        "campaign_evidence_seal", "campaign_evidence_snapshot_identity",
        "physical_package_authentication", "contextual_campaign_reconciliation", "per_seed_raw_outputs", "metrics", "readiness_trace", "safety_trace",
        "tactile_trace", "controller_trace", "process_stdout_stderr_receipts", "report_source", "final_report",
        "physical_tree_inventory", "post_run_preservation", "independent_audit_target",
    }
    if artifacts != required:
        _fail("evidence_artifacts", "required evidence artifact set differs", "$.evidence_outputs.required_artifacts")


def _validate_promotion(value: Any) -> None:
    obj = _object(value, "promotion_contract", "$.promotion_contract")
    _closed(obj, {"required_gates", "limitations", "independent_audit_required", "external_promotion_record_required", "final_closure_record_required"}, "$.promotion_contract")
    gates = set(_string_list(obj["required_gates"], "$.promotion_contract.required_gates", allow_empty=False))
    if gates != set(EXPECTED_PROMOTION_GATES):
        _fail("missing_promotion_gate", "promotion gate set differs", "$.promotion_contract.required_gates")
    limitations = set(_string_list(obj["limitations"], "$.promotion_contract.limitations", allow_empty=False))
    if limitations != set(EXPECTED_PROMOTION_LIMITATIONS):
        code = "missing_non_real_time_limitation" if "NO_REAL_TIME_PERFORMANCE_CLAIM" not in limitations else "promotion_limitations"
        _fail(code, "promotion limitation set differs", "$.promotion_contract.limitations")
    _exact_bool(obj["independent_audit_required"], True, "independent_audit_gate", "$.promotion_contract.independent_audit_required")
    _exact_bool(obj["external_promotion_record_required"], True, "promotion_record_gate", "$.promotion_contract.external_promotion_record_required")
    _exact_bool(obj["final_closure_record_required"], True, "final_closure_gate", "$.promotion_contract.final_closure_record_required")


def _validate_attempt_ledger_record(value: dict[str, Any] | Slice7GAttemptLedger) -> Slice7GAttemptLedger:
    fields = (
        "schema_version", "charter_logical_identity", "campaign_id", "campaign_identity", "campaign_plan_identity", "runtime_authorization_identity", "revision",
        "predecessor_ledger_identity", "applied_event_identities", "applied_event_ids", "last_event_identity",
        "maximum_campaign_attempts", "consumed_campaign_attempts", "retry_count", "maximum_retries",
        "domain_allocated", "domain_id", "output_root_allocated", "output_root", "process_start_committed",
    )
    data = _record_data(value, Slice7GAttemptLedger, fields, "attempt_ledger_type")
    return Slice7GAttemptLedger(**data)


def _attempt_ledger_data(value: Slice7GAttemptLedger) -> dict[str, Any]:
    return {
        "schema_version": value.schema_version,
        "charter_logical_identity": value.charter_logical_identity,
        "campaign_id": value.campaign_id,
        "campaign_identity": value.campaign_identity,
        "campaign_plan_identity": value.campaign_plan_identity,
        "runtime_authorization_identity": value.runtime_authorization_identity,
        "revision": value.revision,
        "predecessor_ledger_identity": value.predecessor_ledger_identity,
        "applied_event_identities": list(value.applied_event_identities),
        "applied_event_ids": list(value.applied_event_ids),
        "last_event_identity": value.last_event_identity,
        "maximum_campaign_attempts": value.maximum_campaign_attempts,
        "consumed_campaign_attempts": value.consumed_campaign_attempts,
        "retry_count": value.retry_count,
        "maximum_retries": value.maximum_retries,
        "domain_allocated": value.domain_allocated,
        "domain_id": value.domain_id,
        "output_root_allocated": value.output_root_allocated,
        "output_root": value.output_root,
        "process_start_committed": value.process_start_committed,
    }


def _validate_attempt_event_record(value: dict[str, Any] | Slice7GAttemptEvent) -> Slice7GAttemptEvent:
    fields = (
        "schema_version", "charter_logical_identity", "campaign_identity", "campaign_plan_identity", "runtime_authorization_identity", "event_id", "event_kind",
        "expected_revision", "expected_predecessor_ledger_identity", "previous_attempt_count",
        "resulting_attempt_count", "retry_count", "maximum_retries", "domain_allocated", "domain_id",
        "output_root_allocated", "output_root", "process_start_consumed", "event_timestamp_utc",
    )
    return Slice7GAttemptEvent(**_record_data(value, Slice7GAttemptEvent, fields, "attempt_event_type"))


def _attempt_event_data(value: Slice7GAttemptEvent) -> dict[str, Any]:
    return {field: getattr(value, field) for field in (
        "schema_version", "charter_logical_identity", "campaign_identity", "campaign_plan_identity", "runtime_authorization_identity", "event_id", "event_kind",
        "expected_revision", "expected_predecessor_ledger_identity", "previous_attempt_count",
        "resulting_attempt_count", "retry_count", "maximum_retries", "domain_allocated", "domain_id",
        "output_root_allocated", "output_root", "process_start_consumed", "event_timestamp_utc",
    )}


def _validate_campaign_cell_record(value: dict[str, Any] | Slice7GCampaignCell) -> Slice7GCampaignCell:
    fields = (
        "schema_version", "cell_id", "charter_logical_identity", "campaign_id", "campaign_identity",
        "attempt_ledger_identity", "scenario_id", "source_scenario_id", "seed", "geometry_profile",
        "task", "duration_seconds", "runtime_mode", "ros_domain_id", "campaign_output_root",
        "cell_output_path", "argv", "metric_profile_identity", "domain_allocation_requested",
    )
    return Slice7GCampaignCell(**_record_data(value, Slice7GCampaignCell, fields, "campaign_cell_type"))


def _campaign_cell_data(value: Slice7GCampaignCell) -> dict[str, Any]:
    result = {field: getattr(value, field) for field in (
        "schema_version", "cell_id", "charter_logical_identity", "campaign_id", "campaign_identity",
        "attempt_ledger_identity", "scenario_id", "source_scenario_id", "seed", "geometry_profile",
        "task", "duration_seconds", "runtime_mode", "ros_domain_id", "campaign_output_root",
        "cell_output_path", "metric_profile_identity", "domain_allocation_requested",
    )}
    result["argv"] = list(value.argv)
    return result


def _validate_campaign_plan_record(value: dict[str, Any] | Slice7GCampaignPlan) -> Slice7GCampaignPlan:
    fields = (
        "schema_version", "charter_logical_identity", "campaign_id", "campaign_identity",
        "attempt_ledger_identity", "ros_domain_id", "campaign_output_root", "metric_profile_identity", "cells",
    )
    data = _record_data(value, Slice7GCampaignPlan, fields, "campaign_plan_type")
    cells = data["cells"]
    if type(cells) not in (tuple, list):
        _fail("campaign_cells_type", "campaign cells must be an exact list or tuple", "$.plan.cells")
    data["cells"] = tuple(_validate_campaign_cell_record(cell) for cell in cells)
    return Slice7GCampaignPlan(**data)


def _campaign_plan_data(value: Slice7GCampaignPlan) -> dict[str, Any]:
    return {
        "schema_version": value.schema_version,
        "charter_logical_identity": value.charter_logical_identity,
        "campaign_id": value.campaign_id,
        "campaign_identity": value.campaign_identity,
        "attempt_ledger_identity": value.attempt_ledger_identity,
        "ros_domain_id": value.ros_domain_id,
        "campaign_output_root": value.campaign_output_root,
        "metric_profile_identity": value.metric_profile_identity,
        "cells": [_campaign_cell_data(cell) for cell in value.cells],
    }


def _validate_cell_result_record(value: dict[str, Any] | Slice7GCellResult) -> Slice7GCellResult:
    fields = (
        "schema_version", "cell_id", "charter_logical_identity", "campaign_identity",
        "campaign_plan_identity", "attempt_ledger_identity", "attempt_ledger_revision",
        "process_start_event_identity", "runtime_authorization_identity", "metric_profile_identity",
        "scenario_id", "source_scenario_id", "seed", "duration_seconds",
        "runtime_mode", "ros_domain_id", "campaign_output_root", "cell_output_path", "argv",
        "process_exit_status", "readiness_success", "stable_sample_count",
        "stable_interval_seconds", "q_variation", "tip_variation_m", "valid_aligned_sample_count",
        "invalid_sample_count", "invalid_sample_percentage", "steady_state_error_m", "final_goal_error_m",
        "goal_hold_duration_seconds", "minimum_physical_wall_clearance_m",
        "minimum_safety_margin_wall_clearance_m", "collision_sample_count", "safety_fault_count",
        "nonfinite_value_count", "missing_required_topic_count", "missing_required_result_file_count",
        "saturation_percentage", "deadline_overrun_percentage", "timing_pass", "non_real_time_label",
    )
    return Slice7GCellResult(**_record_data(value, Slice7GCellResult, fields, "cell_result_type"))


def _cell_result_data(value: Slice7GCellResult) -> dict[str, Any]:
    return {field: getattr(value, field) for field in (
        "schema_version", "cell_id", "charter_logical_identity", "campaign_identity",
        "campaign_plan_identity", "attempt_ledger_identity", "attempt_ledger_revision",
        "process_start_event_identity", "runtime_authorization_identity", "metric_profile_identity",
        "scenario_id", "source_scenario_id", "seed", "duration_seconds",
        "runtime_mode", "ros_domain_id", "campaign_output_root", "cell_output_path",
        "process_exit_status", "readiness_success", "stable_sample_count",
        "stable_interval_seconds", "q_variation", "tip_variation_m", "valid_aligned_sample_count",
        "invalid_sample_count", "invalid_sample_percentage", "steady_state_error_m", "final_goal_error_m",
        "goal_hold_duration_seconds", "minimum_physical_wall_clearance_m",
        "minimum_safety_margin_wall_clearance_m", "collision_sample_count", "safety_fault_count",
        "nonfinite_value_count", "missing_required_topic_count", "missing_required_result_file_count",
        "saturation_percentage", "deadline_overrun_percentage", "timing_pass", "non_real_time_label",
    )} | {"argv": list(value.argv)}


def _validate_campaign_result_record(value: dict[str, Any] | Slice7GCampaignResult) -> Slice7GCampaignResult:
    fields = (
        "schema_version", "charter_logical_identity", "campaign_identity", "campaign_plan_identity",
        "campaign_evidence_snapshot_identity",
        "evidence_package_identities", "result_identities", "total_result_count", "functionally_passing_cell_count",
        "functionally_failing_cell_ids", "functional_failure_reasons", "functional_promotion_pass",
        "timing_all_pass", "non_real_time_limitation_required", "total_valid_aligned_samples",
        "total_invalid_samples", "total_collision_samples", "total_safety_faults", "total_nonfinite_values",
        "total_missing_required_topics", "total_missing_required_results", "timing_failure_cell_count",
    )
    return Slice7GCampaignResult(**_record_data(value, Slice7GCampaignResult, fields, "campaign_result_type"))


def _campaign_result_data(value: Slice7GCampaignResult) -> dict[str, Any]:
    result = {field: getattr(value, field) for field in (
        "schema_version", "charter_logical_identity", "campaign_identity", "campaign_plan_identity",
        "campaign_evidence_snapshot_identity",
        "total_result_count", "functionally_passing_cell_count", "functional_promotion_pass",
        "timing_all_pass", "non_real_time_limitation_required", "total_valid_aligned_samples",
        "total_invalid_samples", "total_collision_samples", "total_safety_faults", "total_nonfinite_values",
        "total_missing_required_topics", "total_missing_required_results", "timing_failure_cell_count",
    )}
    result["evidence_package_identities"] = list(value.evidence_package_identities)
    result["result_identities"] = list(value.result_identities)
    result["functionally_failing_cell_ids"] = list(value.functionally_failing_cell_ids)
    result["functional_failure_reasons"] = list(value.functional_failure_reasons)
    return result


def _validate_campaign_evidence_package_record(value: Any) -> Slice7GCampaignEvidencePackage:
    fields = ("schema_version", "cell_id", "relative_path", "package_identity")
    return Slice7GCampaignEvidencePackage(
        **_record_data(value, Slice7GCampaignEvidencePackage, fields, "campaign_evidence_package_type")
    )


def _campaign_evidence_package_data(value: Slice7GCampaignEvidencePackage) -> dict[str, Any]:
    record = _validate_campaign_evidence_package_record(value)
    return {
        "schema_version": record.schema_version,
        "cell_id": record.cell_id,
        "relative_path": record.relative_path,
        "package_identity": record.package_identity,
    }


def _validate_campaign_evidence_seal_record(value: Any) -> Slice7GCampaignEvidenceSeal:
    fields = (
        "schema_version", "charter_logical_identity", "campaign_identity", "campaign_plan_identity",
        "runtime_authorization_identity", "attempt_ledger_identity", "attempt_ledger_revision",
        "process_start_event_identity", "ros_domain_id", "campaign_output_root",
        "evidence_root_relative_path", "packages",
    )
    data = _record_data(value, Slice7GCampaignEvidenceSeal, fields, "campaign_evidence_seal_type")
    packages = data["packages"]
    if type(packages) not in (tuple, list):
        _fail("campaign_evidence_packages_type", "seal packages must be an exact list or tuple")
    data["packages"] = tuple(_validate_campaign_evidence_package_record(item) for item in packages)
    return Slice7GCampaignEvidenceSeal(**data)


def _campaign_evidence_seal_data(value: Slice7GCampaignEvidenceSeal) -> dict[str, Any]:
    seal = _validate_campaign_evidence_seal_record(value)
    return {
        "schema_version": seal.schema_version,
        "charter_logical_identity": seal.charter_logical_identity,
        "campaign_identity": seal.campaign_identity,
        "campaign_plan_identity": seal.campaign_plan_identity,
        "runtime_authorization_identity": seal.runtime_authorization_identity,
        "attempt_ledger_identity": seal.attempt_ledger_identity,
        "attempt_ledger_revision": seal.attempt_ledger_revision,
        "process_start_event_identity": seal.process_start_event_identity,
        "ros_domain_id": seal.ros_domain_id,
        "campaign_output_root": seal.campaign_output_root,
        "evidence_root_relative_path": seal.evidence_root_relative_path,
        "packages": [_campaign_evidence_package_data(item) for item in seal.packages],
    }


def _campaign_evidence_snapshot_identity(
    seal_raw: bytes, seal: Slice7GCampaignEvidenceSeal,
) -> str:
    payload = {
        "schema_version": CAMPAIGN_EVIDENCE_SNAPSHOT_SCHEMA_VERSION,
        "seal_sha256": hashlib.sha256(seal_raw).hexdigest(),
        "packages": [
            {
                "cell_id": item.cell_id,
                "relative_path": item.relative_path,
                "package_identity": item.package_identity,
            }
            for item in seal.packages
        ],
    }
    return hashlib.sha256(
        CAMPAIGN_EVIDENCE_SNAPSHOT_IDENTITY_DOMAIN + _canonical_json(payload)
    ).hexdigest()


def _validate_evidence_member_record(value: Any) -> Slice7GEvidenceMember:
    fields = ("role", "path", "size", "sha256", "mode", "link_count", "file_type")
    return Slice7GEvidenceMember(**_record_data(value, Slice7GEvidenceMember, fields, "evidence_member_type"))


def _evidence_member_data(value: Slice7GEvidenceMember) -> dict[str, Any]:
    member = _validate_evidence_member_record(value)
    return {field: getattr(member, field) for field in ("role", "path", "size", "sha256", "mode", "link_count", "file_type")}


def _validate_evidence_envelope_record(value: Any) -> Slice7GCellEvidenceEnvelope:
    fields = (
        "schema_version", "charter_logical_identity", "campaign_identity", "campaign_plan_identity",
        "cell_id", "scenario_id", "source_scenario_id", "seed", "metric_profile_identity",
        "attempt_ledger_identity", "attempt_ledger_revision", "process_start_event_identity",
        "runtime_authorization_identity", "ros_domain_id", "campaign_output_root", "cell_output_path",
        "argv", "process_exit_status", "projection_identity", "members",
    )
    data = _record_data(value, Slice7GCellEvidenceEnvelope, fields, "evidence_envelope_type")
    members = data["members"]
    if type(members) not in (tuple, list):
        _fail("evidence_members_type", "evidence members must be an exact list or tuple")
    data["members"] = tuple(_validate_evidence_member_record(member) for member in members)
    return Slice7GCellEvidenceEnvelope(**data)


def _evidence_envelope_bindings(value: Slice7GCellEvidenceEnvelope) -> dict[str, Any]:
    return {field: getattr(value, field) for field in (
        "charter_logical_identity", "campaign_identity", "campaign_plan_identity", "cell_id",
        "attempt_ledger_identity", "attempt_ledger_revision", "process_start_event_identity",
        "runtime_authorization_identity", "ros_domain_id", "campaign_output_root", "cell_output_path",
    )}


def _validate_evidence_role_document(
    value: dict[str, Any], role: str, envelope: Slice7GCellEvidenceEnvelope
) -> dict[str, Any]:
    _closed(value, {"schema_version", "role", "bindings", "payload"}, f"$.evidence.{role}")
    _exact_string(
        value["schema_version"], CELL_EVIDENCE_MEMBER_SCHEMA_VERSION,
        "evidence_member_schema", f"$.evidence.{role}.schema_version",
    )
    _exact_string(value["role"], role, "evidence_member_role", f"$.evidence.{role}.role")
    bindings = _object(value["bindings"], "evidence_bindings", f"$.evidence.{role}.bindings")
    expected_bindings = _evidence_envelope_bindings(envelope)
    _closed(bindings, set(expected_bindings), f"$.evidence.{role}.bindings")
    if bindings != expected_bindings:
        _fail("evidence_member_binding", "member bindings differ from the authenticated envelope", role)
    payload = _object(value["payload"], "evidence_payload", f"$.evidence.{role}.payload")
    return payload


def _validate_committed_evidence_ledger(
    value: dict[str, Any] | Slice7GAttemptLedger, charter: Slice7GCharter
) -> Slice7GAttemptLedger:
    ledger = _validate_attempt_ledger_record(value)
    if ledger.charter_logical_identity != slice_7g_charter_identity(charter):
        _fail("evidence_ledger_charter", "committed ledger does not bind the supplied charter")
    if (
        not ledger.process_start_committed or ledger.consumed_campaign_attempts != 1
        or ledger.maximum_campaign_attempts != 1
    ):
        _fail("evidence_process_start_uncommitted", "evidence requires a committed 1/1 process-start ledger")
    if ledger.retry_count != 0 or ledger.maximum_retries != 0:
        _fail("evidence_ledger_retry", "evidence ledger cannot authorize or record a retry")
    if (
        not ledger.domain_allocated or ledger.domain_id is None or not ledger.output_root_allocated
        or ledger.output_root is None or ledger.runtime_authorization_identity is None
        or ledger.campaign_plan_identity is None or ledger.last_event_identity is None
    ):
        _fail("evidence_ledger_binding", "committed ledger lacks runtime, plan, domain, output, or process-event authority")
    return ledger


def _reconcile_evidence_context(
    envelope: Slice7GCellEvidenceEnvelope,
    charter: Slice7GCharter,
    ledger: Slice7GAttemptLedger,
    ledger_identity: str,
    plan: Slice7GCampaignPlan,
    plan_identity: str,
    cell: Slice7GCampaignCell,
) -> None:
    expected = {
        "charter_logical_identity": slice_7g_charter_identity(charter),
        "campaign_identity": plan.campaign_identity,
        "campaign_plan_identity": plan_identity,
        "cell_id": cell.cell_id,
        "scenario_id": cell.scenario_id,
        "source_scenario_id": cell.source_scenario_id,
        "seed": cell.seed,
        "metric_profile_identity": plan.metric_profile_identity,
        "attempt_ledger_identity": ledger_identity,
        "attempt_ledger_revision": ledger.revision,
        "process_start_event_identity": ledger.last_event_identity,
        "runtime_authorization_identity": ledger.runtime_authorization_identity,
        "ros_domain_id": ledger.domain_id,
        "campaign_output_root": ledger.output_root,
        "cell_output_path": cell.cell_output_path,
        "argv": cell.argv,
    }
    for field, required in expected.items():
        if getattr(envelope, field) != required:
            _fail(f"evidence_{field}_mismatch", "evidence envelope differs from committed authority", f"$.evidence_envelope.{field}")


def _reconcile_cell_result_context(
    result: Slice7GCellResult, envelope: Slice7GCellEvidenceEnvelope, cell: Slice7GCampaignCell
) -> None:
    expected = {
        "charter_logical_identity": envelope.charter_logical_identity,
        "campaign_identity": envelope.campaign_identity,
        "campaign_plan_identity": envelope.campaign_plan_identity,
        "attempt_ledger_identity": envelope.attempt_ledger_identity,
        "attempt_ledger_revision": envelope.attempt_ledger_revision,
        "process_start_event_identity": envelope.process_start_event_identity,
        "runtime_authorization_identity": envelope.runtime_authorization_identity,
        "metric_profile_identity": envelope.metric_profile_identity,
        "cell_id": cell.cell_id,
        "scenario_id": cell.scenario_id,
        "source_scenario_id": cell.source_scenario_id,
        "seed": cell.seed,
        "duration_seconds": cell.duration_seconds,
        "runtime_mode": cell.runtime_mode,
        "ros_domain_id": cell.ros_domain_id,
        "campaign_output_root": cell.campaign_output_root,
        "cell_output_path": cell.cell_output_path,
        "argv": cell.argv,
        "process_exit_status": envelope.process_exit_status,
    }
    for field, required in expected.items():
        if getattr(result, field) != required:
            _fail("cell_result_context_mismatch", f"cell result field {field} differs from authenticated authority", f"$.cell_result.{field}")


def _reconcile_role_payloads(
    documents: dict[str, dict[str, Any]], result: Slice7GCellResult, envelope: Slice7GCellEvidenceEnvelope
) -> None:
    expected_payloads: dict[str, dict[str, Any]] = {
        "invocation_process_start_receipt": {
            "argv": list(envelope.argv), "process_exit_status": result.process_exit_status,
        },
        "runtime_authorization_binding": {
            "runtime_authorization_identity": envelope.runtime_authorization_identity,
        },
        "readiness_trace": {
            "readiness_success": result.readiness_success,
            "stable_sample_count": result.stable_sample_count,
            "stable_interval_seconds": result.stable_interval_seconds,
            "q_variation": result.q_variation,
            "tip_variation_m": result.tip_variation_m,
        },
        "safety_trace": {
            "minimum_physical_wall_clearance_m": result.minimum_physical_wall_clearance_m,
            "minimum_safety_margin_wall_clearance_m": result.minimum_safety_margin_wall_clearance_m,
            "collision_sample_count": result.collision_sample_count,
            "safety_fault_count": result.safety_fault_count,
            "nonfinite_value_count": result.nonfinite_value_count,
        },
        "tactile_trace": {
            "valid_aligned_sample_count": result.valid_aligned_sample_count,
            "invalid_sample_count": result.invalid_sample_count,
            "invalid_sample_percentage": result.invalid_sample_percentage,
            "saturation_percentage": result.saturation_percentage,
            "missing_required_topic_count": result.missing_required_topic_count,
        },
    }
    for role, expected in expected_payloads.items():
        payload = documents[role]
        _closed(payload, set(expected), f"$.evidence.{role}.payload")
        if payload != expected:
            _fail("evidence_payload_mismatch", "retained role payload differs from the cell result", role)
    output = documents["output_inventory_receipt"]
    _closed(
        output,
        {"missing_required_result_file_count", "output_tree_identity", "regular_file_count", "regular_file_bytes"},
        "$.evidence.output_inventory_receipt.payload",
    )
    if output["missing_required_result_file_count"] != result.missing_required_result_file_count:
        _fail("evidence_payload_mismatch", "output receipt disagrees with cell result", "output_inventory_receipt")
    _digest(output["output_tree_identity"], "$.evidence.output_inventory_receipt.payload.output_tree_identity")
    _nonnegative_int(output["regular_file_count"], "output_inventory_count", "$.evidence.output_inventory_receipt.payload.regular_file_count")
    _nonnegative_int(output["regular_file_bytes"], "output_inventory_bytes", "$.evidence.output_inventory_receipt.payload.regular_file_bytes")


def _normalize_public_path(
    value: Any, type_code: str, unsafe_code: str, *, require_absolute: bool = False,
) -> str:
    """Detach one caller path without invoking conversion hooks more than once."""

    try:
        raw = os.fspath(value)
    except Exception as exc:
        raise Slice7GGovernanceError(type_code, "path conversion failed") from exc
    if type(raw) is not str:
        _fail(type_code, "path conversion must return an exact string")
    try:
        detached = raw.encode("utf-8", errors="strict").decode("utf-8", errors="strict")
    except UnicodeError as exc:
        raise Slice7GGovernanceError(unsafe_code, "path is not valid UTF-8 text") from exc
    if not detached or "\x00" in detached or "\\" in detached:
        _fail(unsafe_code, "path is empty or contains a prohibited character", detached or "$")
    if any(ord(character) < 0x20 or ord(character) == 0x7f for character in detached):
        _fail(unsafe_code, "path contains a control character", detached)
    if unicodedata.normalize("NFC", detached) != detached:
        _fail(unsafe_code, "path must use NFC normalization", detached)
    body = detached[1:] if detached.startswith("/") else detached
    if detached != "/" and (body.endswith("/") or "//" in body):
        _fail(unsafe_code, "path contains an empty component", detached)
    components = [component for component in body.split("/") if component]
    if any(component in (".", "..") for component in components) and detached != ".":
        _fail(unsafe_code, "path contains an alias or traversal component", detached)
    if require_absolute and not detached.startswith("/"):
        _fail(unsafe_code, "evidence package path must be absolute", detached)
    return detached


def _open_directory_path_nofollow(path: str) -> int:
    """Open every directory component relative to a retained ancestor."""

    flags = (
        os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
        | getattr(os, "O_CLOEXEC", 0)
    )
    anchor = "/" if path.startswith("/") else "."
    current = os.open(anchor, flags)
    components = [component for component in (path[1:] if path.startswith("/") else path).split("/") if component]
    if path == ".":
        components = []
    try:
        for component in components:
            child = os.open(component, flags, dir_fd=current)
            os.close(current)
            current = child
        return current
    except BaseException:
        os.close(current)
        raise


def _open_relative_directory_nofollow(parent_fd: int, relative: str) -> int:
    """Open a safe relative directory path without consulting ambient pathnames."""

    _safe_relative_path(relative, "$.relative_directory")
    flags = (
        os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
        | getattr(os, "O_CLOEXEC", 0)
    )
    current = os.dup(parent_fd)
    try:
        for component in relative.split("/"):
            child = os.open(component, flags, dir_fd=current)
            os.close(current)
            current = child
        return current
    except BaseException:
        os.close(current)
        raise


def _read_exact_sealed_fd(
    descriptor: int, baseline: os.stat_result, *, code: str, path: str,
) -> bytes:
    try:
        os.lseek(descriptor, 0, os.SEEK_SET)
        chunks: list[bytes] = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(descriptor)
    except OSError as exc:
        raise Slice7GGovernanceError(code, "sealed file could not be read", path=path) from exc
    if _stable_stat(after) != _stable_stat(baseline):
        _fail(code, "sealed file metadata changed while reading", path)
    raw = b"".join(chunks)
    if not raw or len(raw) != baseline.st_size:
        _fail(code, "sealed file is empty, truncated, or changed", path)
    return raw


def _expected_evidence_names() -> set[str]:
    return {EVIDENCE_ENVELOPE_PATH, EVIDENCE_PROJECTION_PATH, *MANDATORY_EVIDENCE_ROLE_PATHS.values()}


def _capture_evidence_observations(
    root_fd: int,
) -> dict[str, tuple[os.stat_result, bytes, str]]:
    try:
        names = os.listdir(root_fd)
    except OSError as exc:
        raise Slice7GGovernanceError("evidence_inventory_read", str(exc)) from exc
    if any(type(name) is not str for name in names):
        _fail("evidence_inventory_name", "evidence paths must be text names")
    normalized = [unicodedata.normalize("NFC", name) for name in names]
    if len(normalized) != len(set(normalized)):
        _fail("evidence_unicode_collision", "evidence inventory contains Unicode-colliding names")
    expected = _expected_evidence_names()
    observed = set(names)
    if observed != expected:
        missing = sorted(expected - observed)
        extra = sorted(observed - expected)
        _fail("evidence_inventory_mismatch", f"evidence inventory differs: missing={missing} extra={extra}")
    result: dict[str, tuple[os.stat_result, bytes, str]] = {}
    inodes: set[tuple[int, int]] = set()
    for name in sorted(expected):
        _safe_relative_path(name, f"$.evidence_inventory.{name}")
        info, raw, digest = _read_sealed_evidence_member(root_fd, name)
        inode = (info.st_dev, info.st_ino)
        if inode in inodes:
            _fail("evidence_hardlink_alias", "evidence members must have unique physical inodes", name)
        inodes.add(inode)
        result[name] = (info, raw, digest)
    try:
        final_names = set(os.listdir(root_fd))
    except OSError as exc:
        raise Slice7GGovernanceError("evidence_inventory_read", str(exc)) from exc
    if final_names != expected:
        _fail("evidence_inventory_changed", "evidence inventory changed during authentication")
    return result


def _validate_campaign_seal_context(
    seal: Slice7GCampaignEvidenceSeal,
    record: Slice7GCharter,
    committed: Slice7GAttemptLedger,
    validated_plan: Slice7GCampaignPlan,
) -> None:
    plan_identity = slice_7g_campaign_plan_identity(validated_plan)
    ledger_identity = slice_7g_attempt_ledger_identity(committed)
    expected = {
        "charter_logical_identity": slice_7g_charter_identity(record),
        "campaign_identity": validated_plan.campaign_identity,
        "campaign_plan_identity": plan_identity,
        "runtime_authorization_identity": committed.runtime_authorization_identity,
        "attempt_ledger_identity": ledger_identity,
        "attempt_ledger_revision": committed.revision,
        "process_start_event_identity": committed.last_event_identity,
        "ros_domain_id": committed.domain_id,
        "campaign_output_root": committed.output_root,
    }
    for field, required in expected.items():
        if getattr(seal, field) != required:
            _fail(
                f"campaign_seal_{field}_mismatch",
                "campaign seal differs from committed authority",
                f"$.campaign_evidence_seal.{field}",
            )
    expected_cells = [cell.cell_id for cell in validated_plan.cells]
    if [item.cell_id for item in seal.packages] != expected_cells:
        _fail("campaign_seal_bijection", "seal packages are not in exact campaign-plan order")


def _open_campaign_evidence_authority(
    campaign_root: str,
    record: Slice7GCharter,
    committed: Slice7GAttemptLedger,
    validated_plan: Slice7GCampaignPlan,
) -> _CampaignEvidenceAuthorityState:
    descriptors: list[int] = []
    lock_held = False
    locked_seal_fd: int | None = None
    try:
        try:
            campaign_fd = _open_directory_path_nofollow(campaign_root)
        except (OSError, ValueError) as exc:
            raise Slice7GGovernanceError("campaign_root_open", str(exc), path=campaign_root) from exc
        descriptors.append(campaign_fd)
        campaign_baseline = os.fstat(campaign_fd)
        if not stat.S_ISDIR(campaign_baseline.st_mode) or stat.S_IMODE(campaign_baseline.st_mode) != 0o555:
            _fail("campaign_root_mode", "finalized campaign root must be a real 0555 directory", campaign_root)
        try:
            evidence_fd = _open_relative_directory_nofollow(campaign_fd, CAMPAIGN_EVIDENCE_ROOT_RELATIVE_PATH)
        except OSError as exc:
            raise Slice7GGovernanceError("campaign_evidence_root_open", str(exc)) from exc
        descriptors.append(evidence_fd)
        evidence_baseline = os.fstat(evidence_fd)
        if stat.S_IMODE(evidence_baseline.st_mode) != 0o555:
            _fail("campaign_evidence_root_mode", "finalized evidence root must have mode 0555")
        flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
        try:
            seal_fd = os.open(CAMPAIGN_EVIDENCE_SEAL_PATH, flags, dir_fd=evidence_fd)
        except OSError as exc:
            raise Slice7GGovernanceError("campaign_seal_open", str(exc)) from exc
        descriptors.append(seal_fd)
        seal_baseline = os.fstat(seal_fd)
        if not stat.S_ISREG(seal_baseline.st_mode):
            _fail("campaign_seal_type", "campaign seal must be a regular file")
        if stat.S_IMODE(seal_baseline.st_mode) != 0o444:
            _fail("campaign_seal_mode", "finalized campaign seal must have mode 0444")
        if seal_baseline.st_nlink != 1:
            _fail("campaign_seal_hardlink", "campaign seal link count must equal one")
        if seal_baseline.st_size == 0:
            _fail("campaign_seal_empty", "campaign seal must be nonempty")
        flock = getattr(fcntl, "flock", None)
        if flock is None:
            _fail("campaign_seal_lock_unavailable", "flock is unavailable")
        try:
            flock(seal_fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
            lock_held = True
            locked_seal_fd = seal_fd
        except OSError as exc:
            unavailable = {errno.ENOSYS, errno.ENOTSUP}
            if hasattr(errno, "EOPNOTSUPP"):
                unavailable.add(errno.EOPNOTSUPP)
            if exc.errno in (errno.EACCES, errno.EAGAIN):
                raise Slice7GGovernanceError("campaign_seal_lock_busy", "campaign seal is exclusively locked") from exc
            if exc.errno in unavailable:
                raise Slice7GGovernanceError("campaign_seal_lock_unavailable", "flock is unsupported") from exc
            raise Slice7GGovernanceError("campaign_seal_lock_failed", "campaign seal lock failed") from exc
        seal_raw = _read_exact_sealed_fd(
            seal_fd, seal_baseline, code="campaign_seal_changed", path=CAMPAIGN_EVIDENCE_SEAL_PATH,
        )
        seal_data = _parse_json_bytes(seal_raw)
        if seal_raw != _canonical_json(seal_data):
            _fail("campaign_seal_noncanonical", "campaign seal bytes must be canonical")
        seal = _validate_campaign_evidence_seal_record(seal_data)
        _validate_campaign_seal_context(seal, record, committed, validated_plan)
        by_name = os.stat(CAMPAIGN_EVIDENCE_SEAL_PATH, dir_fd=evidence_fd, follow_symlinks=False)
        if _stable_stat(by_name) != _stable_stat(seal_baseline):
            _fail("campaign_seal_replaced", "seal directory entry changed during opening")
        evidence_names = set(os.listdir(evidence_fd))
        if evidence_names != {CAMPAIGN_EVIDENCE_SEAL_PATH, CAMPAIGN_EVIDENCE_PACKAGES_RELATIVE_PATH}:
            _fail("campaign_evidence_inventory", "evidence root must contain only the seal and packages directory")
        try:
            packages_fd = _open_relative_directory_nofollow(
                evidence_fd, CAMPAIGN_EVIDENCE_PACKAGES_RELATIVE_PATH,
            )
        except OSError as exc:
            raise Slice7GGovernanceError("campaign_packages_root_open", str(exc)) from exc
        descriptors.append(packages_fd)
        packages_baseline = os.fstat(packages_fd)
        if stat.S_IMODE(packages_baseline.st_mode) != 0o555:
            _fail("campaign_packages_root_mode", "finalized packages root must have mode 0555")
        expected_cells = {cell.cell_id for cell in validated_plan.cells}
        if set(os.listdir(packages_fd)) != expected_cells:
            _fail("campaign_package_inventory", "packages directory must contain the exact 15 plan cells")
        state = _CampaignEvidenceAuthorityState(
            campaign_root, campaign_fd, campaign_baseline, evidence_fd, evidence_baseline,
            packages_fd, packages_baseline, seal_fd, seal_baseline, seal_raw, seal,
        )
        descriptors.clear()
        lock_held = False
        return state
    except BaseException:
        if lock_held and locked_seal_fd is not None:
            try:
                fcntl.flock(locked_seal_fd, fcntl.LOCK_UN)
            except OSError:
                pass
        for descriptor in reversed(descriptors):
            try:
                os.close(descriptor)
            except OSError:
                pass
        raise


def _open_campaign_package_authority(
    campaign: _CampaignEvidenceAuthorityState,
    package: Slice7GCampaignEvidencePackage,
    record: Slice7GCharter,
    committed: Slice7GAttemptLedger,
    validated_plan: Slice7GCampaignPlan,
) -> _EvidenceAuthorityState:
    relative_to_packages = package.cell_id
    try:
        descriptor = _open_relative_directory_nofollow(campaign.packages_fd, relative_to_packages)
    except OSError as exc:
        raise Slice7GGovernanceError("campaign_package_open", str(exc), path=package.relative_path) from exc
    package_root = (
        f"{campaign.campaign_root}/{CAMPAIGN_EVIDENCE_ROOT_RELATIVE_PATH}/"
        f"{CAMPAIGN_EVIDENCE_PACKAGES_RELATIVE_PATH}/{package.cell_id}"
    )
    return _build_evidence_authority(descriptor, package_root, record, committed, validated_plan)


def _final_campaign_evidence_barrier(
    campaign: _CampaignEvidenceAuthorityState,
    states: list[_EvidenceAuthorityState],
    expected_snapshot_identity: str,
) -> None:
    """Final cooperative-lock barrier; no evidence reads may follow this call."""

    try:
        if campaign.closed or not campaign.lock_held:
            _fail("campaign_evidence_late_change", "campaign seal authority is not live")
        for current, baseline, label in (
            (os.fstat(campaign.campaign_fd), campaign.campaign_baseline, "campaign root"),
            (os.fstat(campaign.evidence_fd), campaign.evidence_baseline, "evidence root"),
            (os.fstat(campaign.packages_fd), campaign.packages_baseline, "packages root"),
            (os.fstat(campaign.seal_fd), campaign.seal_baseline, "campaign seal"),
        ):
            if _stable_stat(current) != _stable_stat(baseline):
                _fail("campaign_evidence_late_change", f"{label} metadata changed")
        if set(os.listdir(campaign.evidence_fd)) != {
            CAMPAIGN_EVIDENCE_SEAL_PATH, CAMPAIGN_EVIDENCE_PACKAGES_RELATIVE_PATH,
        }:
            _fail("campaign_evidence_late_change", "evidence-root inventory changed")
        if set(os.listdir(campaign.packages_fd)) != {item.cell_id for item in campaign.seal.packages}:
            _fail("campaign_evidence_late_change", "package-root inventory changed")
        final_seal_raw = _read_exact_sealed_fd(
            campaign.seal_fd,
            campaign.seal_baseline,
            code="campaign_evidence_late_change",
            path=CAMPAIGN_EVIDENCE_SEAL_PATH,
        )
        if final_seal_raw != campaign.seal_raw:
            _fail("campaign_evidence_late_change", "campaign seal bytes changed")
        named_seal = os.stat(
            CAMPAIGN_EVIDENCE_SEAL_PATH, dir_fd=campaign.evidence_fd, follow_symlinks=False,
        )
        if _stable_stat(named_seal) != _stable_stat(campaign.seal_baseline):
            _fail("campaign_evidence_late_change", "seal pathname no longer names the locked seal")
        reopened_campaign = _open_directory_path_nofollow(campaign.campaign_root)
        try:
            if _stable_stat(os.fstat(reopened_campaign)) != _stable_stat(campaign.campaign_baseline):
                _fail("campaign_evidence_late_change", "campaign pathname no longer names the authenticated root")
            reopened_evidence = _open_relative_directory_nofollow(
                reopened_campaign, CAMPAIGN_EVIDENCE_ROOT_RELATIVE_PATH,
            )
            try:
                if _stable_stat(os.fstat(reopened_evidence)) != _stable_stat(campaign.evidence_baseline):
                    _fail("campaign_evidence_late_change", "evidence pathname was replaced")
                reopened_packages = _open_relative_directory_nofollow(
                    reopened_evidence, CAMPAIGN_EVIDENCE_PACKAGES_RELATIVE_PATH,
                )
                try:
                    if _stable_stat(os.fstat(reopened_packages)) != _stable_stat(campaign.packages_baseline):
                        _fail("campaign_evidence_late_change", "packages pathname was replaced")
                finally:
                    os.close(reopened_packages)
            finally:
                os.close(reopened_evidence)
        finally:
            os.close(reopened_campaign)
        observed_snapshot_identity = _campaign_evidence_snapshot_identity(final_seal_raw, campaign.seal)
        if observed_snapshot_identity != expected_snapshot_identity:
            _fail("campaign_evidence_late_change", "campaign snapshot identity changed")
        _validate_cross_package_authority(states, final=True)
    except Slice7GGovernanceError as exc:
        if exc.code == "campaign_evidence_late_change":
            raise
        raise Slice7GGovernanceError(
            "campaign_evidence_late_change", "campaign evidence changed at its final barrier",
            path=campaign.campaign_root,
        ) from exc
    except (OSError, ValueError) as exc:
        raise Slice7GGovernanceError(
            "campaign_evidence_late_change", "campaign evidence changed at its final barrier",
            path=campaign.campaign_root,
        ) from exc


def _open_evidence_authority(
    root_path: str,
    record: Slice7GCharter,
    committed: Slice7GAttemptLedger,
    validated_plan: Slice7GCampaignPlan,
) -> _EvidenceAuthorityState:
    root_fd: int | None = None
    try:
        try:
            root_fd = _open_directory_path_nofollow(root_path)
        except (OSError, ValueError) as exc:
            raise Slice7GGovernanceError("evidence_root_open", str(exc), path=root_path) from exc
        owned_fd = root_fd
        root_fd = None
        state = _build_evidence_authority(owned_fd, root_path, record, committed, validated_plan)
        return state
    except BaseException:
        if root_fd is not None:
            os.close(root_fd)
        raise


def _build_evidence_authority(
    root_fd: int,
    root_path: str,
    record: Slice7GCharter,
    committed: Slice7GAttemptLedger,
    validated_plan: Slice7GCampaignPlan,
) -> _EvidenceAuthorityState:
    """Consume a confined package descriptor and retain it as live authority."""

    state: _EvidenceAuthorityState | None = None
    try:
        try:
            baseline = os.fstat(root_fd)
        except OSError as exc:
            raise Slice7GGovernanceError("evidence_root_stat", str(exc), path=root_path) from exc
        if not stat.S_ISDIR(baseline.st_mode):
            _fail("evidence_root_type", "evidence root must be a real directory", root_path)
        if stat.S_IMODE(baseline.st_mode) != 0o555:
            _fail("evidence_root_mode", "finalized evidence root must have mode 0555", root_path)
        observations = _capture_evidence_observations(root_fd)
        try:
            after_capture = os.fstat(root_fd)
        except OSError as exc:
            raise Slice7GGovernanceError("evidence_root_stat", str(exc), path=root_path) from exc
        if _stable_stat(baseline) != _stable_stat(after_capture):
            _fail("evidence_root_changed", "evidence root metadata changed during authentication")
        state = _EvidenceAuthorityState(root_path, root_fd, baseline, observations)
        state.authenticated = _derive_authenticated_evidence(state, record, committed, validated_plan)
        return state
    except BaseException:
        if state is not None:
            state.close()
        else:
            os.close(root_fd)
        raise


def _derive_authenticated_evidence(
    state: _EvidenceAuthorityState,
    record: Slice7GCharter,
    committed: Slice7GAttemptLedger,
    validated_plan: Slice7GCampaignPlan,
) -> Slice7GAuthenticatedCellEvidence:
    observations = state.observations
    envelope_raw = observations[EVIDENCE_ENVELOPE_PATH][1]
    envelope_data = _parse_json_bytes(envelope_raw)
    if envelope_raw != _canonical_json(envelope_data):
        _fail("evidence_envelope_noncanonical", "evidence envelope bytes are not canonical")
    envelope = _validate_evidence_envelope_record(envelope_data)
    descriptors = {member.path: member for member in envelope.members}
    for role, path in MANDATORY_EVIDENCE_ROLE_PATHS.items():
        info, raw, digest = observations[path]
        descriptor = descriptors[path]
        if (
            descriptor.role != role or descriptor.size != len(raw) or descriptor.sha256 != digest
            or descriptor.mode != stat.S_IMODE(info.st_mode) or descriptor.link_count != info.st_nlink
        ):
            _fail("evidence_member_descriptor_mismatch", "authenticated member disagrees with envelope descriptor", path)
    projection = {
        "schema_version": CELL_EVIDENCE_PROJECTION_SCHEMA_VERSION,
        "members": [_evidence_member_data(member) for member in envelope.members],
    }
    projection_raw = observations[EVIDENCE_PROJECTION_PATH][1]
    projection_data = _parse_json_bytes(projection_raw)
    if projection_raw != _canonical_json(projection_data) or projection_data != projection:
        _fail("evidence_projection_mismatch", "retained projection bytes differ from authenticated member descriptors")
    projection_identity = hashlib.sha256(
        CELL_EVIDENCE_PROJECTION_IDENTITY_DOMAIN + _canonical_json(projection)
    ).hexdigest()
    if envelope.projection_identity != projection_identity:
        _fail("evidence_projection_identity", "envelope projection identity was not derived from authenticated descriptors")

    plan_identity = slice_7g_campaign_plan_identity(validated_plan)
    cell = {item.cell_id: item for item in validated_plan.cells}.get(envelope.cell_id)
    if cell is None:
        _fail("evidence_cell_binding", "evidence cell is not present in the campaign plan")
    committed_identity = slice_7g_attempt_ledger_identity(committed)
    _reconcile_evidence_context(envelope, record, committed, committed_identity, validated_plan, plan_identity, cell)
    parsed_members: dict[str, dict[str, Any]] = {}
    for role, path in MANDATORY_EVIDENCE_ROLE_PATHS.items():
        raw = observations[path][1]
        data = _parse_json_bytes(raw)
        if raw != _canonical_json(data):
            _fail("evidence_member_noncanonical", "evidence member bytes are not canonical", path)
        if role != "cell_result":
            parsed_members[role] = _validate_evidence_role_document(data, role, envelope)
    result = _validate_cell_result_record(
        _parse_json_bytes(observations[MANDATORY_EVIDENCE_ROLE_PATHS["cell_result"]][1])
    )
    _reconcile_cell_result_context(result, envelope, cell)
    _reconcile_role_payloads(parsed_members, result, envelope)

    envelope_info, _, envelope_digest = observations[EVIDENCE_ENVELOPE_PATH]
    projection_info, _, projection_digest = observations[EVIDENCE_PROJECTION_PATH]
    physical_projection = {
        "root": {"path": ".", "type": "directory", "mode": stat.S_IMODE(state.root_baseline.st_mode)},
        "envelope": {
            "path": EVIDENCE_ENVELOPE_PATH, "type": "regular_file",
            "mode": stat.S_IMODE(envelope_info.st_mode), "link_count": envelope_info.st_nlink,
            "size": envelope_info.st_size, "sha256": envelope_digest,
        },
        "projection": {
            "path": EVIDENCE_PROJECTION_PATH, "type": "regular_file",
            "mode": stat.S_IMODE(projection_info.st_mode), "link_count": projection_info.st_nlink,
            "size": projection_info.st_size, "sha256": projection_digest,
        },
        "projection_identity": projection_identity,
        "schema_version": "ctr-slice-7g-cell-evidence-package-physical-1",
    }
    package_identity = hashlib.sha256(
        CELL_EVIDENCE_PACKAGE_IDENTITY_DOMAIN + _canonical_json(physical_projection)
    ).hexdigest()
    return Slice7GAuthenticatedCellEvidence._create(
        state.package_root, state.root_baseline.st_dev, state.root_baseline.st_ino,
        projection_identity, package_identity, envelope, result,
    )


def _final_evidence_barrier(state: _EvidenceAuthorityState) -> None:
    """Reauthenticate retained bytes and pathname immediately before authority returns."""

    try:
        if state.closed or state.barrier_complete:
            _fail("evidence_late_change", "evidence authority is not live at its final barrier")
        if _stable_stat(os.fstat(state.root_fd)) != _stable_stat(state.root_baseline):
            _fail("evidence_late_change", "evidence root metadata changed before final barrier")
        final = _capture_evidence_observations(state.root_fd)
        for name, retained in state.observations.items():
            observed = final[name]
            if _stable_stat(retained[0]) != _stable_stat(observed[0]) or retained[1:] != observed[1:]:
                _fail("evidence_late_change", "evidence member changed before final barrier", name)
        if _stable_stat(os.fstat(state.root_fd)) != _stable_stat(state.root_baseline):
            _fail("evidence_late_change", "evidence root changed during final barrier")
        reopened = _open_directory_path_nofollow(state.package_root)
        try:
            reopened_info = os.fstat(reopened)
            if _stable_stat(reopened_info) != _stable_stat(state.root_baseline):
                _fail("evidence_late_change", "public pathname no longer names the authenticated root")
            if set(os.listdir(reopened)) != _expected_evidence_names():
                _fail("evidence_late_change", "reopened package inventory differs")
        finally:
            os.close(reopened)
        # Re-establish the acyclic projection/envelope relation from the final bytes.
        envelope_data = _parse_json_bytes(final[EVIDENCE_ENVELOPE_PATH][1])
        envelope = _validate_evidence_envelope_record(envelope_data)
        projection = {
            "schema_version": CELL_EVIDENCE_PROJECTION_SCHEMA_VERSION,
            "members": [_evidence_member_data(member) for member in envelope.members],
        }
        if final[EVIDENCE_PROJECTION_PATH][1] != _canonical_json(projection):
            _fail("evidence_late_change", "projection relationship changed at final barrier")
        projection_identity = hashlib.sha256(
            CELL_EVIDENCE_PROJECTION_IDENTITY_DOMAIN + _canonical_json(projection)
        ).hexdigest()
        if state.authenticated is None or projection_identity != state.authenticated.projection_identity:
            _fail("evidence_late_change", "projection identity changed at final barrier")
        envelope_info, _, envelope_digest = final[EVIDENCE_ENVELOPE_PATH]
        projection_info, _, projection_digest = final[EVIDENCE_PROJECTION_PATH]
        physical_projection = {
            "root": {"path": ".", "type": "directory", "mode": stat.S_IMODE(state.root_baseline.st_mode)},
            "envelope": {
                "path": EVIDENCE_ENVELOPE_PATH, "type": "regular_file",
                "mode": stat.S_IMODE(envelope_info.st_mode), "link_count": envelope_info.st_nlink,
                "size": envelope_info.st_size, "sha256": envelope_digest,
            },
            "projection": {
                "path": EVIDENCE_PROJECTION_PATH, "type": "regular_file",
                "mode": stat.S_IMODE(projection_info.st_mode), "link_count": projection_info.st_nlink,
                "size": projection_info.st_size, "sha256": projection_digest,
            },
            "projection_identity": projection_identity,
            "schema_version": "ctr-slice-7g-cell-evidence-package-physical-1",
        }
        package_identity = hashlib.sha256(
            CELL_EVIDENCE_PACKAGE_IDENTITY_DOMAIN + _canonical_json(physical_projection)
        ).hexdigest()
        if package_identity != state.authenticated.package_identity:
            _fail("evidence_late_change", "package identity changed at final barrier")
        state.final_observations = final
        state.barrier_complete = True
    except Slice7GGovernanceError as exc:
        if exc.code == "evidence_late_change":
            raise
        raise Slice7GGovernanceError(
            "evidence_late_change", "evidence changed or became inaccessible at final barrier",
            path=state.package_root,
        ) from exc
    except (OSError, ValueError) as exc:
        raise Slice7GGovernanceError(
            "evidence_late_change", "evidence changed or became inaccessible at final barrier",
            path=state.package_root,
        ) from exc


def _required_authenticated(state: _EvidenceAuthorityState) -> Slice7GAuthenticatedCellEvidence:
    if state.authenticated is None:
        _fail("evidence_internal_state", "evidence authority has no immutable observation")
    return state.authenticated


def _validate_cross_package_authority(
    states: list[_EvidenceAuthorityState], *, final: bool,
) -> None:
    observations = [state.final_observations if final else state.observations for state in states]
    if final and any(not state.barrier_complete for state in states):
        _fail("evidence_late_change", "not every package completed its final barrier")
    roots = [(state.root_baseline.st_dev, state.root_baseline.st_ino) for state in states]
    if len(roots) != len(set(roots)):
        _fail("reused_evidence_package", "package root inodes must be unique")
    identities = [_required_authenticated(state).package_identity for state in states]
    if len(identities) != len(set(identities)):
        _fail("reused_evidence_package", "package identities must be unique")
    absolute_paths = [os.path.abspath(state.package_root).rstrip("/") for state in states]
    for index, left in enumerate(absolute_paths):
        for right in absolute_paths[index + 1:]:
            if left.startswith(right + "/") or right.startswith(left + "/"):
                _fail("nested_evidence_package", "one evidence package cannot contain another")
    member_inodes: set[tuple[int, int]] = set()
    for package_observations in observations:
        if package_observations is None:
            _fail("evidence_late_change", "final package observations are unavailable")
        for info, _, _ in package_observations.values():
            inode = (info.st_dev, info.st_ino)
            if inode in member_inodes:
                _fail("cross_package_hardlink_alias", "evidence members cannot be hardlinked across packages")
            member_inodes.add(inode)


def _stable_stat(info: os.stat_result) -> tuple[int, int, int, int, int, int, int]:
    return (
        info.st_dev, info.st_ino, info.st_mode, info.st_nlink, info.st_size,
        info.st_mtime_ns, info.st_ctime_ns,
    )


def _read_sealed_evidence_member(root_fd: int, name: str) -> tuple[os.stat_result, bytes, str]:
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
    descriptor: int | None = None
    try:
        try:
            descriptor = os.open(name, flags, dir_fd=root_fd)
        except OSError as exc:
            raise Slice7GGovernanceError("evidence_member_open", str(exc), path=name) from exc
        try:
            before = os.fstat(descriptor)
        except OSError as exc:
            raise Slice7GGovernanceError("evidence_member_stat", str(exc), path=name) from exc
        if not stat.S_ISREG(before.st_mode):
            _fail("evidence_member_type", "evidence member must be a regular file", name)
        if stat.S_IMODE(before.st_mode) != 0o444:
            _fail("evidence_member_mode", "finalized evidence member must have mode 0444", name)
        if before.st_nlink != 1:
            _fail("evidence_hardlink_alias", "evidence member link count must equal one", name)
        chunks: list[bytes] = []
        while True:
            try:
                chunk = os.read(descriptor, 1024 * 1024)
            except OSError as exc:
                raise Slice7GGovernanceError("evidence_member_read", str(exc), path=name) from exc
            if not chunk:
                break
            chunks.append(chunk)
        try:
            after = os.fstat(descriptor)
        except OSError as exc:
            raise Slice7GGovernanceError("evidence_member_stat", str(exc), path=name) from exc
        if _stable_stat(before) != _stable_stat(after):
            _fail("evidence_member_changed", "member metadata changed while hashing", name)
        try:
            by_name = os.stat(name, dir_fd=root_fd, follow_symlinks=False)
        except OSError as exc:
            raise Slice7GGovernanceError("evidence_member_replaced", str(exc), path=name) from exc
        if _stable_stat(after) != _stable_stat(by_name):
            _fail("evidence_member_replaced", "directory entry no longer identifies the opened member", name)
        raw = b"".join(chunks)
        return after, raw, hashlib.sha256(raw).hexdigest()
    finally:
        if descriptor is not None:
            os.close(descriptor)


def _record_data(value: Any, exact_class: type[Any], fields: tuple[str, ...], code: str) -> dict[str, Any]:
    if type(value) is exact_class:
        try:
            raw = {field: getattr(value, field) for field in fields}
        except AttributeError as exc:
            raise Slice7GGovernanceError(code, "record is partially initialized") from exc
    elif type(value) is dict:
        raw = _plain_detached(value)
        _closed(raw, set(fields), "$")
    else:
        _fail(code, f"value must be an exact {exact_class.__name__} or object")
    return {field: raw[field] for field in fields}


def _functional_result_failures(result: Slice7GCellResult) -> tuple[str, ...]:
    failures: list[str] = []
    gates: tuple[tuple[str, bool], ...] = (
        ("process_exit_status", result.process_exit_status == 0),
        ("readiness_success", result.readiness_success),
        ("stable_sample_count", result.stable_sample_count >= 10),
        ("stable_interval", result.stable_interval_seconds >= 0.5),
        ("q_variation", result.q_variation <= 5.0e-5),
        ("tip_variation", result.tip_variation_m <= 5.0e-5),
        ("valid_aligned_sample_count", result.valid_aligned_sample_count >= 20),
        ("invalid_sample_percentage", result.invalid_sample_percentage <= 10.0),
        ("steady_state_error", result.steady_state_error_m <= 0.003),
        ("final_goal_error", result.final_goal_error_m <= 0.003),
        ("goal_hold_duration", result.goal_hold_duration_seconds >= 0.5),
        ("minimum_physical_wall_clearance", result.minimum_physical_wall_clearance_m >= 0.0),
        ("minimum_safety_margin_wall_clearance", result.minimum_safety_margin_wall_clearance_m >= 0.002),
        ("collision_sample_count", result.collision_sample_count == 0),
        ("safety_fault_count", result.safety_fault_count == 0),
        ("nonfinite_value_count", result.nonfinite_value_count == 0),
        ("missing_required_topic_count", result.missing_required_topic_count == 0),
        ("missing_required_result_file_count", result.missing_required_result_file_count == 0),
        ("saturation_percentage", result.saturation_percentage <= 1.0),
    )
    for name, passed in gates:
        if not passed:
            failures.append(name)
    return tuple(failures)


def _hash_confined_member(root_fd: int, relative: str) -> tuple[os.stat_result, int, str]:
    components = relative.split("/")
    directory_fd = os.dup(root_fd)
    member_fd: int | None = None
    try:
        for component in components[:-1]:
            flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
            try:
                child_fd = os.open(component, flags, dir_fd=directory_fd)
            except OSError as exc:
                raise Slice7GGovernanceError("snapshot_component_not_directory", str(exc), path=relative) from exc
            os.close(directory_fd)
            directory_fd = child_fd
            if not stat.S_ISDIR(os.fstat(directory_fd).st_mode):
                _fail("snapshot_component_not_directory", "intermediate component is not a real directory", relative)
        flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
        try:
            member_fd = os.open(components[-1], flags, dir_fd=directory_fd)
        except OSError as exc:
            raise Slice7GGovernanceError("snapshot_member_open", str(exc), path=relative) from exc
        before = os.fstat(member_fd)
        if not stat.S_ISREG(before.st_mode):
            _fail("snapshot_member_type", "snapshot member is not a regular file", relative)
        digest = hashlib.sha256()
        size = 0
        while True:
            chunk = os.read(member_fd, 1024 * 1024)
            if not chunk:
                break
            size += len(chunk)
            digest.update(chunk)
        after = os.fstat(member_fd)
        stable = lambda item: (
            item.st_dev, item.st_ino, item.st_mode, item.st_nlink, item.st_size,
            item.st_mtime_ns, item.st_ctime_ns,
        )
        if stable(before) != stable(after):
            _fail("snapshot_member_changed", "snapshot member metadata changed while hashing", relative)
        return after, size, digest.hexdigest()
    finally:
        if member_fd is not None:
            os.close(member_fd)
        os.close(directory_fd)


def _parse_json_bytes(raw: bytes) -> dict[str, Any]:
    if not isinstance(raw, bytes):
        _fail("json_bytes_type", "raw charter must be bytes")
    try:
        text = raw.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise Slice7GGovernanceError("invalid_utf8", str(exc)) from exc

    def pairs_hook(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                _fail("duplicate_json_key", f"duplicate JSON key: {key}")
            result[key] = value
        return result

    def reject_constant(value: str) -> Any:
        _fail("nonfinite_json", f"non-finite JSON value: {value}")

    try:
        parsed = json.loads(text, object_pairs_hook=pairs_hook, parse_constant=reject_constant)
    except Slice7GGovernanceError:
        raise
    except json.JSONDecodeError as exc:
        raise Slice7GGovernanceError("invalid_json", str(exc)) from exc
    if type(parsed) is not dict:
        _fail("charter_type", "top-level charter must be an object")
    return parsed


def _canonical_json(value: Any) -> bytes:
    try:
        return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise Slice7GGovernanceError("canonical_json", str(exc)) from exc


def _plain_detached(value: Any, path: str = "$") -> Any:
    if type(value) is dict:
        result: dict[str, Any] = {}
        try:
            items = tuple(dict.items(value))
        except RuntimeError as exc:
            raise Slice7GGovernanceError("mapping_mutated", "object changed during detachment", path=path) from exc
        for key, item in items:
            if type(key) is not str:
                _fail("field_name_type", "object field names must be exact strings", path)
            result[key] = _plain_detached(item, f"{path}.{key}")
        return result
    if type(value) is list or type(value) is tuple:
        return [_plain_detached(item, f"{path}[]") for item in value]
    if value is None or type(value) in (str, bool, int):
        return value
    if type(value) is float:
        if not math.isfinite(value):
            _fail("nonfinite_value", "float values must be finite", path)
        return value
    _fail("scalar_type", f"unsupported scalar type: {type(value).__name__}", path)


def _thaw_owned_immutable(value: Any, path: str = "$") -> Any:
    """Detach only immutable containers retained by module-owned records.

    This is intentionally distinct from ``_plain_detached``: arbitrary
    ``Mapping`` objects and externally-created mapping proxies are never
    accepted at primitive public boundaries.
    """

    if type(value) is MappingProxyType:
        result: dict[str, Any] = {}
        try:
            items = tuple(value.items())
        except RuntimeError as exc:
            raise Slice7GGovernanceError("invalid_owned_mapping", "retained mapping changed", path=path) from exc
        for key, item in items:
            if type(key) is not str:
                _fail("field_name_type", "retained field names must be exact strings", path)
            result[key] = _thaw_owned_immutable(item, f"{path}.{key}")
        return result
    if type(value) is tuple:
        return [_thaw_owned_immutable(item, f"{path}[]") for item in value]
    if value is None or type(value) in (str, bool, int):
        return value
    if type(value) is float:
        if not math.isfinite(value):
            _fail("nonfinite_value", "float values must be finite", path)
        return value
    _fail("invalid_owned_mapping", "retained charter data has an unsupported type", path)


def _freeze(value: Any) -> Any:
    if type(value) is dict:
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if type(value) is list:
        return tuple(_freeze(item) for item in value)
    return value


def _closed(obj: dict[str, Any], expected: set[str], path: str) -> None:
    actual = set(obj)
    missing = expected - actual
    unknown = actual - expected
    if missing:
        _fail("missing_field", f"missing fields: {sorted(missing)}", path)
    if unknown:
        _fail("unknown_field", f"unknown fields: {sorted(unknown)}", path)


def _object(value: Any, code: str, path: str) -> dict[str, Any]:
    if type(value) is not dict:
        _fail(f"{code}_type", "value must be an object", path)
    return value


def _list(value: Any, code: str, path: str) -> list[Any]:
    if type(value) is not list:
        _fail(f"{code}_type", "value must be an array", path)
    return value


def _string_list(value: Any, path: str, *, allow_empty: bool) -> list[str]:
    items = _list(value, "string_list", path)
    if not allow_empty and not items:
        _fail("string_list_empty", "array cannot be empty", path)
    seen: set[str] = set()
    for index, item in enumerate(items):
        text = _nonempty_string(item, "string_list", f"{path}[{index}]")
        if text in seen:
            _fail("duplicate_string", "array values must be unique", f"{path}[{index}]")
        seen.add(text)
    return items


def _nonempty_string(value: Any, code: str, path: str) -> str:
    if type(value) is not str or not value or any(ord(char) < 0x20 for char in value):
        _fail(code, "value must be a non-empty control-free exact string", path)
    return value


def _safe_relative_path(value: Any, path: str) -> str:
    text = _nonempty_string(value, "unsafe_path", path)
    if unicodedata.normalize("NFC", text) != text:
        _fail("unsafe_path", "path must use canonical NFC Unicode", path)
    if "\\" in text or "://" in text or text.startswith("/"):
        _fail("unsafe_path", "path must be repository-relative", path)
    segments = text.split("/")
    if any(segment in ("", ".", "..") for segment in segments):
        _fail("unsafe_path", "path contains an empty, dot, or traversal segment", path)
    return text


def _safe_identifier(value: Any, code: str, path: str) -> str:
    text = _nonempty_string(value, code, path)
    if re.fullmatch(r"[a-z][a-z0-9_]*", text) is None:
        _fail(code, "identifier must use lowercase letters, digits, and underscores", path)
    return text


def _opaque_identifier(value: Any, code: str, path: str) -> str:
    text = _nonempty_string(value, code, path)
    if len(text) > 128 or re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._:-]*", text) is None:
        _fail(code, "identifier contains unsupported characters", path)
    return text


def _detached_string_tuple(value: Any, code: str, path: str) -> tuple[str, ...]:
    if type(value) not in (tuple, list):
        _fail(code, "value must be an exact list or tuple", path)
    result: list[str] = []
    for index, item in enumerate(value):
        result.append(_nonempty_string(item, code, f"{path}[{index}]"))
    return tuple(result)


def _detached_exact_record_tuple(value: Any, record_type: type[Any], code: str, path: str) -> tuple[Any, ...]:
    if type(value) not in (tuple, list):
        _fail(code, "records must be an exact list or tuple", path)
    result: list[Any] = []
    for index, item in enumerate(value):
        if record_type is Slice7GCampaignCell:
            result.append(_validate_campaign_cell_record(item))
        else:
            _fail(code, "unsupported record type", f"{path}[{index}]")
    return tuple(result)


def _domain_id(value: Any, path: str) -> int:
    if type(value) is not int or not 100 <= value <= 199:
        _fail("domain_id", "domain ID must be an exact integer in 100..199", path)
    return value


def _external_output_root(value: Any, path: str) -> str:
    text = _nonempty_string(value, "output_root", path)
    if unicodedata.normalize("NFC", text) != text or "\\" in text or "://" in text:
        _fail("output_root", "output root uses an unsafe representation", path)
    components = text.split("/")
    if not text.startswith("/") or any(component in (".", "..") for component in components) or any(not component for component in components[1:]):
        _fail("output_root", "output root must be a normalized absolute path", path)
    parent = SLICE_7G_EVIDENCE_PARENT
    if text == parent or not text.startswith(parent + "/"):
        _fail("output_root", "output root must be a strict descendant of the Slice 7G evidence parent", path)
    return text


def _is_strict_descendant(path: str, parent: str) -> bool:
    return path != parent and path.startswith(parent.rstrip("/") + "/")


def _finite_number(value: Any, code: str, path: str) -> float:
    if type(value) not in (int, float) or type(value) is bool:
        _fail(code, "value must be an exact finite number", path)
    number = float(value)
    if not math.isfinite(number):
        _fail(code, "value must be finite", path)
    return number


def _campaign_identity(charter_identity: str, campaign_id: str) -> str:
    payload = _canonical_json({"campaign_id": campaign_id, "charter_logical_identity": charter_identity})
    return hashlib.sha256(CAMPAIGN_IDENTITY_DOMAIN + payload).hexdigest()


def _digest(value: Any, path: str) -> str:
    text = _nonempty_string(value, "digest", path)
    if re.fullmatch(r"[0-9a-f]{64}", text) is None:
        _fail("digest", "SHA-256 must be 64 lowercase hexadecimal characters", path)
    return text


def _exact_string(value: Any, expected: str, code: str, path: str) -> None:
    if type(value) is not str or value != expected:
        _fail(code, f"value must equal {expected!r}", path)


def _exact_bool(value: Any, expected: bool, code: str, path: str) -> None:
    if type(value) is not bool or value is not expected:
        _fail(code, f"value must be exact bool {expected}", path)


def _exact_int(value: Any, expected: int, code: str, path: str) -> None:
    if type(value) is not int or value != expected:
        _fail(code, f"value must be exact integer {expected}", path)


def _nonnegative_int(value: Any, code: str, path: str) -> int:
    if type(value) is not int or value < 0:
        _fail(code, "value must be a non-negative exact integer", path)
    return value


def _exact_number(value: Any, expected: float, code: str, path: str) -> None:
    if type(value) is not float or not math.isfinite(value) or value != expected:
        _fail(code, f"value must equal exact finite float {expected}", path)


def _validate_utc_timestamp(value: Any, path: str, code: str) -> str:
    text = _nonempty_string(value, code, path)
    if re.fullmatch(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z", text) is None:
        _fail(code, "timestamp must be exact second-resolution UTC with Z", path)
    try:
        parsed = datetime.strptime(text, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
    except ValueError as exc:
        raise Slice7GGovernanceError(code, "timestamp is not a real UTC date/time", path=path) from exc
    if parsed.strftime("%Y-%m-%dT%H:%M:%SZ") != text:
        _fail(code, "timestamp does not round-trip canonically", path)
    return text


def _validate_timestamp(value: Any) -> None:
    _validate_utc_timestamp(value, "$.creation_timestamp", "creation_timestamp")


def _fail(code: str, message: str, path: str = "$") -> None:
    raise Slice7GGovernanceError(code, message, path=path)


__all__ = [
    "CHARTER_IDENTITY_ALGORITHM",
    "ATTEMPT_LEDGER_SCHEMA_VERSION",
    "ATTEMPT_EVENT_SCHEMA_VERSION",
    "CAMPAIGN_PLAN_SCHEMA_VERSION",
    "CAMPAIGN_RESULT_SCHEMA_VERSION",
    "CAMPAIGN_EVIDENCE_SEAL_SCHEMA_VERSION",
    "CAMPAIGN_EVIDENCE_PACKAGE_RECORD_SCHEMA_VERSION",
    "CAMPAIGN_EVIDENCE_SNAPSHOT_SCHEMA_VERSION",
    "CAMPAIGN_EVIDENCE_SNAPSHOT_IDENTITY_ALGORITHM",
    "CELL_RESULT_SCHEMA_VERSION",
    "CELL_EVIDENCE_PROJECTION_SCHEMA_VERSION",
    "CELL_EVIDENCE_ENVELOPE_SCHEMA_VERSION",
    "CELL_EVIDENCE_MEMBER_SCHEMA_VERSION",
    "CELL_EVIDENCE_PROJECTION_IDENTITY_ALGORITHM",
    "CELL_EVIDENCE_PACKAGE_IDENTITY_ALGORITHM",
    "MANDATORY_EVIDENCE_ROLE_PATHS",
    "EVIDENCE_PROJECTION_PATH",
    "CAMPAIGN_EVIDENCE_ROOT_RELATIVE_PATH",
    "CAMPAIGN_EVIDENCE_PACKAGES_RELATIVE_PATH",
    "CAMPAIGN_EVIDENCE_SEAL_PATH",
    "Slice7GGovernanceError",
    "HistoricalSlice7GCharterInspection",
    "Slice7GAttemptBudget",
    "Slice7GDomainPolicy",
    "Slice7GScenario",
    "Slice7GMetric",
    "Slice7GCharter",
    "Slice7GAttemptLedger",
    "Slice7GAttemptEvent",
    "Slice7GCampaignCell",
    "Slice7GCampaignPlan",
    "Slice7GCampaignEvidencePackage",
    "Slice7GCampaignEvidenceSeal",
    "Slice7GCellResult",
    "Slice7GCampaignResult",
    "Slice7GEvidenceMember",
    "Slice7GCellEvidenceEnvelope",
    "Slice7GAuthenticatedCellEvidence",
    "load_slice_7g_charter",
    "inspect_historical_slice_7g_charter",
    "canonical_slice_7g_charter_bytes",
    "slice_7g_charter_identity",
    "validate_slice_7g_charter",
    "validate_slice_7g_attempt_budget",
    "validate_slice_7g_domain_policy",
    "validate_slice_7g_scenario",
    "validate_slice_7g_metric",
    "canonical_slice_7g_attempt_ledger_bytes",
    "slice_7g_attempt_ledger_identity",
    "canonical_slice_7g_attempt_event_bytes",
    "slice_7g_attempt_event_identity",
    "create_slice_7g_initial_attempt_ledger",
    "propose_slice_7g_attempt_event",
    "validate_slice_7g_attempt_transition",
    "slice_7g_metric_profile_identity",
    "generate_slice_7g_campaign_plan",
    "validate_slice_7g_campaign_plan",
    "canonical_slice_7g_campaign_plan_bytes",
    "slice_7g_campaign_plan_identity",
    "canonical_slice_7g_cell_result_bytes",
    "slice_7g_cell_result_identity",
    "validate_slice_7g_campaign_evidence_seal",
    "canonical_slice_7g_campaign_evidence_seal_bytes",
    "slice_7g_campaign_evidence_snapshot_identity",
    "authenticate_slice_7g_cell_evidence_package",
    "reconcile_slice_7g_campaign_results",
    "canonical_slice_7g_campaign_result_bytes",
    "slice_7g_campaign_result_identity",
    "verify_authoring_source_snapshot",
]
