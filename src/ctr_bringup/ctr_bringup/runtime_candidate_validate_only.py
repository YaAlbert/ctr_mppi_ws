"""ROS-independent, read-only validation of a frozen runtime candidate."""
from __future__ import annotations

import argparse
import json
import os
import re
import stat
import sys
import unicodedata
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path, PurePosixPath
from types import MappingProxyType

from .exercised_subject_identity import (
    DEFAULT_RECORD_PATH as EXERCISED_SUBJECT_RECORD_PATH,
    IDENTITY_ALGORITHM_ID as EXERCISED_SUBJECT_IDENTITY_ALGORITHM_ID,
    STALE_V2_DIAGNOSTIC_IDENTITY,
    AuthenticatedFile as SubjectAuthenticatedFile,
    ExercisedSubject,
    ExercisedSubjectError,
    exercised_subject_identity,
    parse_exercised_subject,
    validate_exercised_subject,
)
from .materialization_identity import (
    LOGICAL_ALGORITHM_ID,
    MATERIALIZATION_PROJECTION_SCHEMA,
    PHYSICAL_REHASH_ALGORITHM_ID,
    PROJECTION_FRAMING_ALGORITHM_ID,
    SUPERSEDED_HISTORICAL_IDENTITIES,
    MaterializationIdentityError,
    MaterializationPhysicalVerificationResult,
    MaterializationProjection,
    materialization_projection_framing_digest,
    materialization_projection_from_bytes,
    materialization_root_identity,
    projection_identity_result,
    runtime_member_bindings,
    verify_materialization_root_at,
)
from .runtime_plan_validation import (
    RuntimeDependency,
    RuntimeDependencyClosure,
    RuntimePlan,
    RuntimeProjection,
    RuntimeProjectionReconciliation,
    RuntimeValidationError,
    load_runtime_plan,
    load_runtime_projection,
    open_authenticated_runtime_snapshot,
    reconcile_runtime_projection,
    runtime_projection_identity,
    validate_runtime_dependency_closure,
    validate_runtime_plan,
    validate_six_plan_set,
)

BUNDLE_SCHEMA = "ctr-frozen-candidate-bundle-2"
INVENTORY_SCHEMA = "ctr-frozen-candidate-inventory-2"
AUTHORITY_SCHEMA = "ctr-root-authority-2"
CLOSURE_SOURCE_PROJECTION_SCHEMA = "ctr-closure-source-projection-1"
STATIC_CLOSURE_SCHEMA = "ctr-static-closure-2"
REPORT_SOURCE_SCHEMA = "ctr-report-source-2"
SOURCE_IDENTITY_SCHEMA = "ctr-source-identity-2"
RESULT_SCHEMA = "ctr-runtime-candidate-validate-only-result-1"
TRACE_NAMES = (
    "candidate_inventory", "root_authority", "runtime_projection",
    "runtime_physical", "runtime_dependency_closure", "six_plan_set",
    "focused_evidence", "attachments", "report_and_static_closure",
    "capsule_policy",
)
_FAILURE_PASS_PREFIX_COUNTS = {
    "candidate_inventory": 0,
    "root_authority": 1,
    "runtime_projection": 2,
    "runtime_physical": 3,
    "runtime_dependency_closure": 4,
    "six_plan_set": 5,
    "focused_evidence": 6,
    "attachments": 7,
    "report_and_static_closure": 8,
    "capsule_policy": 8,
}
_ROOT_AUTHORITY_PATH = "manifests/root_authority.json"
_FORMAL_META_FIELDS = (
    "inventory_path", "closure_source_projection_path",
    "static_closure_path", "report_source_path",
)
_HEX = re.compile(r"^[0-9a-f]{64}$")
_TRANSIENT = {
    ".cache", ".coverage", ".hypothesis", ".mypy_cache", ".nox",
    ".pytest_cache", ".ruff_cache", ".tox", "__pycache__", "build",
    "htmlcov", "install", "log",
}
_PLAN_ROLES = (
    "production_root", "production_duplicate", "offline_root",
    "offline_duplicate", "test_only_root", "test_only_duplicate",
)
_BUNDLE_PATH_FIELDS = (
    "inventory_path", "projection_path", "runtime_root_path", "dependency_graph_path",
    "plans_manifest_path", "focused_results_path", "attachment_manifest_path",
    "report_source_path", "static_closure_path", "capsule_path",
    "correction_report_path", "source_identity_path",
    "predecessor_preservation_path", "closure_source_projection_path",
    "materialization_projection_path", "materialization_root_path",
    "exercised_subject_path",
)
_BUNDLE_FILE_PATH_FIELDS = tuple(
    field for field in _BUNDLE_PATH_FIELDS
    if field not in {"runtime_root_path", "materialization_root_path"}
)
_BUNDLE_FIELDS = {"schema", "profile", *_BUNDLE_PATH_FIELDS}
_AUTHORITY_ROLE_PATH_FIELDS = (
    ("candidate_bundle", None),
    ("candidate_inventory", "inventory_path"),
    ("runtime_projection", "projection_path"),
    ("runtime_dependency_graph", "dependency_graph_path"),
    ("six_plan_manifest", "plans_manifest_path"),
    ("focused_results", "focused_results_path"),
    ("attachment_manifest", "attachment_manifest_path"),
    ("closure_source_projection", "closure_source_projection_path"),
    ("materialization_projection", "materialization_projection_path"),
    ("exercised_subject", "exercised_subject_path"),
    ("source_identity", "source_identity_path"),
    ("report_source", "report_source_path"),
    ("static_closure", "static_closure_path"),
    ("capsule", "capsule_path"),
)
_FOCUSED_CATEGORIES = {
    "authority": 43, "output_isolation": 31, "digest_allocation": 18,
    "capsule": 11, "regression": 1, "relocation": 16,
    "schema_negative": 16, "additional_schema_negative": 8,
    "report_consistency": 2, "validate_only_capsule_policy": 4,
}
_PROVENANCE = {
    "authority": "repository_production",
    "digest_allocation": "repository_production",
    "regression": "repository_production",
    "schema_negative": "repository_production",
    "additional_schema_negative": "repository_production",
    "output_isolation": "candidate_evidence_integrity",
    "capsule": "candidate_evidence_integrity",
    "relocation": "candidate_evidence_integrity",
    "report_consistency": "candidate_evidence_integrity",
    "validate_only_capsule_policy": "candidate_evidence_integrity",
}
_CLOSURE_CATEGORIES = (
    "candidate_inventory", "closure_source_projection", "runtime_projection",
    "runtime_physical", "runtime_dependencies", "plan_production_root",
    "plan_production_duplicate", "plan_offline_root",
    "plan_offline_duplicate", "plan_test_only_root",
    "plan_test_only_duplicate", "focused_authority",
    "focused_output_isolation", "focused_digest_allocation",
    "focused_capsule", "focused_regression", "focused_relocation",
    "focused_schema_negative", "focused_additional_schema_negative",
    "focused_report_consistency", "focused_validate_only_capsule_policy",
    "raw_packages", "authorization_attachments", "allocation_attachments",
    "durable_receipts", "child_boundary", "materialization_projection",
    "correction_report", "capsule_policy", "candidate_path",
    "source_identity", "predecessor_preservation", "side_effect_boundary",
    "materialization_runtime_binding", "exercised_subject",
)
_ATTACHMENTS = tuple(
    [(f"AUTH-{i:02d}", "authorization", f"case-{i:03d}", "attachment_authorization") for i in range(1, 9)]
    + [(f"ALLOC-{i-74:02d}", "allocation", f"case-{i:03d}", "attachment_allocation") for i in range(75, 83)]
    + [(f"RECEIPT-{i-82:02d}", "durable_receipt", f"case-{i:03d}", "attachment_durable_receipt") for i in range(83, 91)]
    + [(f"CHILD-{i-92:02d}", "child_boundary", f"case-{i:03d}", "attachment_child_boundary") for i in range(93, 97)]
)
_CAPSULE_POLICY = {
    "validate_only": True,
    "allow_full_launch": False,
    "launchable": False,
    "execution_authorized": False,
    "argv_role": "validated_prospective_argv",
    "domain_role": "validated_prospective_environment",
    "output_allocation_allowed": False,
    "child_creation_allowed": False,
    "output_allocation_performed": False,
}
_SIDE_EFFECTS = {
    "output_allocation_performed": False, "process_factory_calls": 0,
    "real_popen_calls": 0, "target_children": 0, "rclpy_activity": 0,
    "ros_commands": 0, "dds_participants": 0,
}
_RESULT_COUNTS_PASS = {
    "runtime_members": 172, "plans": 6, "focused_cases": 150,
    "raw_packages": 150, "attachments": 28,
}
_RESULT_COUNTS_FAIL = {key: 0 for key in _RESULT_COUNTS_PASS}
_OBSERVATION_KEY_CATEGORIES = {
    "candidate.inventory": "candidate_inventory",
    "closure_source_projection.coverage": "closure_source_projection",
    "runtime.projection": "runtime_projection",
    "runtime.physical": "runtime_physical",
    "runtime.dependencies": "runtime_dependencies",
    **{f"plan.{role}": f"plan_{role}" for role in _PLAN_ROLES},
    **{f"focused.{category}": f"focused_{category}" for category in _FOCUSED_CATEGORIES},
    "raw_packages.summary": "raw_packages",
    "attachments.authorization": "authorization_attachments",
    "attachments.allocation": "allocation_attachments",
    "attachments.durable_receipts": "durable_receipts",
    "attachments.child_boundary": "child_boundary",
    "materialization.projection": "materialization_projection",
    "correction_report.bytes": "correction_report",
    "capsule.policy": "capsule_policy",
    "candidate.basename": "candidate_path",
    "source.identity": "source_identity",
    "predecessor.preservation": "predecessor_preservation",
    "side_effect.boundary": "side_effect_boundary",
    "materialization.runtime_binding": "materialization_runtime_binding",
    "exercised_subject.identity": "exercised_subject",
}

_SOURCE_IDENTITY_FILES = (
    ("production_candidate_validator", "src/ctr_bringup/ctr_bringup/runtime_candidate_validate_only.py"),
    ("candidate_validator_tests", "src/ctr_bringup/test/test_runtime_candidate_validate_only.py"),
    ("materialization_identity", "src/ctr_bringup/ctr_bringup/materialization_identity.py"),
    ("materialization_identity_tests", "src/ctr_bringup/test/test_materialization_identity.py"),
    ("runtime_validator", "src/ctr_bringup/ctr_bringup/runtime_plan_validation.py"),
    ("runtime_validator_tests", "src/ctr_bringup/test/test_runtime_plan_validation.py"),
    ("exercised_subject_contract", "src/ctr_bringup/ctr_bringup/exercised_subject_identity.py"),
    ("exercised_subject_contract_tests", "src/ctr_bringup/test/test_exercised_subject_identity.py"),
    ("setup", "src/ctr_bringup/setup.py"),
)
_HISTORICAL_LINEAGE = (
    ("unknown-historical-complete-root", "078727ff4cdb535d71f98ef4f2ae1487f4609b38ba5401df6ee424fa89a572e1"),
    ("unknown-historical-physical-rehash", "f981c80c9e366e12a1406f52f452e09243800baa178412efb6900afc015bed94"),
)


class CandidateValidateOnlyError(Exception):
    def __init__(self, code: str, detail: str = "", *, stage: str | None = None):
        self.code, self.detail, self.stage = str(code), str(detail), stage
        super().__init__(self.code + (": " + self.detail if self.detail else ""))


def _fact_error(code: str, detail: str = ""):
    raise CandidateValidateOnlyError(code, detail)


def _tuple_value(value, field: str):
    if type(value) in (set, frozenset):
        return tuple(sorted(value, key=repr))
    if type(value) not in (tuple, list):
        _fact_error("FACT_COLLECTION_TYPE", field)
    return tuple(value)


def _string(value, field: str, *, nonempty: bool = True):
    if type(value) is not str or (nonempty and not value) or any(
        unicodedata.category(char) in {"Cc", "Cs"} for char in value
    ):
        _fact_error("FACT_STRING_INVALID", field)
    return value


def _count(value, field: str, *, positive: bool = False):
    if type(value) is not int or value < (1 if positive else 0):
        _fact_error("FACT_INTEGER_INVALID", field)
    return value


def _fact_type(value, expected, field: str):
    if type(value) is not expected:
        _fact_error("FACT_TYPE_INVALID", field)
    return value


def _normalized_string_pairs(value, field: str, value_kind: str, *, allow_mapping=False):
    if allow_mapping and type(value) is dict:
        pairs = tuple(value.items())
    else:
        pairs = _tuple_value(value, field)
    normalized, keys = [], set()
    for index, pair in enumerate(pairs):
        if type(pair) not in (tuple, list) or len(pair) != 2:
            _fact_error("FACT_PAIR_INVALID", f"{field}[{index}]")
        key = _string(pair[0], f"{field}[{index}].key")
        if key in keys:
            _fact_error("FACT_PAIR_DUPLICATE", key)
        if value_kind == "integer":
            item = _count(pair[1], f"{field}[{index}].value")
        elif value_kind == "sha256":
            item = _sha_field(pair[1], f"{field}[{index}].value")
        else:
            _fact_error("FACT_PAIR_VALUE_KIND", value_kind)
        keys.add(key)
        normalized.append((key, item))
    return tuple(sorted(normalized))


def _normalized_side_effects(value):
    if type(value) is dict:
        pairs = tuple(value.items())
    else:
        pairs = _tuple_value(value, "result.side_effects")
    normalized, keys = [], set()
    for index, pair in enumerate(pairs):
        if type(pair) not in (tuple, list) or len(pair) != 2:
            _fact_error("RESULT_SIDE_EFFECT_PAIR", str(index))
        key = _string(pair[0], f"result.side_effects[{index}].key")
        if key in keys or key not in _SIDE_EFFECTS:
            _fact_error("RESULT_SIDE_EFFECT_KEY", key)
        item = pair[1]
        expected = _SIDE_EFFECTS[key]
        if type(item) is not type(expected) or item != expected:
            _fact_error("RESULT_SIDE_EFFECT_VALUE", key)
        keys.add(key)
        normalized.append((key, item))
    if keys != set(_SIDE_EFFECTS):
        _fact_error("RESULT_SIDE_EFFECT_SET")
    return tuple(sorted(normalized))


def _normalized_frozen_pairs(value, field: str):
    pairs = _tuple_value(value, field)
    normalized, keys = [], set()
    for index, pair in enumerate(pairs):
        if type(pair) not in (tuple, list) or len(pair) != 2:
            _fact_error("FACT_PAIR_INVALID", f"{field}[{index}]")
        key = _string(pair[0], f"{field}[{index}].key")
        if key in keys:
            _fact_error("FACT_PAIR_DUPLICATE", key)
        keys.add(key)
        normalized.append((key, _freeze(pair[1])))
    return tuple(sorted(normalized))


@dataclass(frozen=True, slots=True)
class ValidationTrace:
    name: str
    status: str
    code: str
    detail: str = ""

    def __post_init__(self):
        _string(self.name, "result.trace.name")
        if self.name not in TRACE_NAMES:
            _fact_error("RESULT_TRACE_NAME")
        _string(self.status, "result.trace.status")
        if self.status not in {"PASS", "FAIL"}:
            _fact_error("RESULT_TRACE_STATUS", self.name)
        _string(self.code, "result.trace.code")
        _string(self.detail, "result.trace.detail", nonempty=False)
        if self.status == "PASS" and self.code != "OK":
            _fact_error("RESULT_TRACE_PASS_CODE", self.name)


@dataclass(frozen=True, slots=True)
class CandidateValidationResult:
    schema: str
    overall: str
    candidate_root: str
    root_authority: str
    runtime_identity: str
    traces: tuple[ValidationTrace, ...]
    counts: tuple[tuple[str, int], ...]
    side_effects: tuple[tuple[str, object], ...]

    def __post_init__(self):
        _string(self.schema, "result.schema")
        if self.schema != RESULT_SCHEMA:
            _fact_error("RESULT_SCHEMA")
        _string(self.overall, "result.overall")
        if self.overall not in {"PASS", "FAIL"}:
            _fact_error("RESULT_OVERALL")
        for field in ("candidate_root", "root_authority", "runtime_identity"):
            _string(getattr(self, field), f"result.{field}")
        traces = _tuple_value(self.traces, "result.traces")
        if not traces or any(type(trace) is not ValidationTrace for trace in traces):
            _fact_error("RESULT_TRACES")
        if len({trace.name for trace in traces}) != len(traces):
            _fact_error("RESULT_TRACE_DUPLICATE")
        failures = tuple(trace for trace in traces if trace.status == "FAIL")
        if self.overall == "PASS":
            if (len(traces) != len(TRACE_NAMES) or failures
                    or tuple(trace.name for trace in traces) != TRACE_NAMES):
                _fact_error("RESULT_PASS_TRACES")
        else:
            if len(failures) != 1 or traces[-1] is not failures[0]:
                _fact_error("RESULT_FAIL_TRACES")
            prefix = traces[:-1]
            if (any(trace.status != "PASS" for trace in prefix)
                    or tuple(trace.name for trace in prefix) != TRACE_NAMES[:len(prefix)]
                    or len(prefix) != _FAILURE_PASS_PREFIX_COUNTS[failures[0].name]):
                _fact_error("RESULT_FAIL_PREFIX")
        counts = _normalized_string_pairs(
            self.counts, "result.counts", "integer", allow_mapping=True,
        )
        expected_counts = _RESULT_COUNTS_PASS if self.overall == "PASS" else _RESULT_COUNTS_FAIL
        if dict(counts) != expected_counts:
            _fact_error("RESULT_COUNTS")
        side_effects = _normalized_side_effects(self.side_effects)
        object.__setattr__(self, "traces", tuple(traces))
        object.__setattr__(self, "counts", counts)
        object.__setattr__(self, "side_effects", side_effects)


@dataclass(frozen=True, slots=True)
class _FileFact:
    path: str
    size: int
    sha256: str
    mode: str

    def __post_init__(self):
        object.__setattr__(self, "path", _safe_rel(self.path, "file.path"))
        _count(self.size, "file.size")
        _sha_field(self.sha256, "file.sha256")
        if type(self.mode) is not str or not re.fullmatch(r"0[0-7]{3}", self.mode):
            _fact_error("FACT_MODE_INVALID", "file.mode")
        if int(self.mode, 8) & 0o222:
            _fact_error("FACT_MODE_WRITABLE", self.path)


@dataclass(frozen=True, slots=True)
class _BundleFacts:
    file: _FileFact
    paths: tuple[tuple[str, str], ...]

    def __post_init__(self):
        _fact_type(self.file, _FileFact, "bundle.file")
        pairs = _tuple_value(self.paths, "bundle.paths")
        normalized = []
        for pair in pairs:
            if type(pair) not in (tuple, list) or len(pair) != 2:
                _fact_error("BUNDLE_FACT_PATH", "bundle.paths")
            field, path = pair
            _string(field, "bundle.field")
            normalized.append((field, _safe_rel(path, field)))
        normalized = tuple(sorted(normalized))
        if (len({field for field, _ in normalized}) != len(normalized)
                or {field for field, _ in normalized} != set(_BUNDLE_PATH_FIELDS)
                or len({path for _, path in normalized}) != len(normalized)):
            _fact_error("BUNDLE_FACT_PATHS")
        object.__setattr__(self, "paths", normalized)

    def path(self, field: str) -> str:
        return dict(self.paths)[field]


@dataclass(frozen=True, slots=True)
class _InventoryFacts:
    file: _FileFact
    members: tuple[_FileFact, ...]

    def __post_init__(self):
        _fact_type(self.file, _FileFact, "inventory.file")
        members = _tuple_value(self.members, "inventory.members")
        if not members or any(type(member) is not _FileFact for member in members):
            _fact_error("INVENTORY_FACT_MEMBERS")
        members = tuple(sorted(members, key=lambda member: member.path))
        if len({member.path for member in members}) != len(members):
            _fact_error("INVENTORY_FACT_DUPLICATE")
        object.__setattr__(self, "members", members)


@dataclass(frozen=True, slots=True)
class _AuthorityChildFact:
    role: str
    file: _FileFact

    def __post_init__(self):
        _string(self.role, "authority_child.role")
        _fact_type(self.file, _FileFact, "authority_child.file")


@dataclass(frozen=True, slots=True)
class _AuthorityFacts:
    file: _FileFact
    expected_sha256: str
    children: tuple[_AuthorityChildFact, ...]

    def __post_init__(self):
        _fact_type(self.file, _FileFact, "authority.file")
        _sha_field(self.expected_sha256, "authority.expected_sha256")
        if self.file.sha256 != self.expected_sha256:
            _fact_error("AUTHORITY_FACT_DIGEST")
        children = _tuple_value(self.children, "authority.children")
        if (len(children) != len(_AUTHORITY_ROLE_PATH_FIELDS)
                or any(type(child) is not _AuthorityChildFact for child in children)):
            _fact_error("AUTHORITY_FACT_CHILDREN")
        if (len({child.role for child in children}) != len(_AUTHORITY_ROLE_PATH_FIELDS)
                or len({child.file.path for child in children}) != len(_AUTHORITY_ROLE_PATH_FIELDS)):
            _fact_error("AUTHORITY_FACT_DUPLICATE")
        if {child.role for child in children} != {role for role, _ in _AUTHORITY_ROLE_PATH_FIELDS}:
            _fact_error("AUTHORITY_FACT_ROLES")
        object.__setattr__(self, "children", tuple(children))


@dataclass(frozen=True, slots=True)
class _RuntimeProjectionFacts:
    projection_file: _FileFact
    projection: RuntimeProjection
    identity: str
    runtime_root: str

    def __post_init__(self):
        _fact_type(self.projection_file, _FileFact, "runtime.projection_file")
        _fact_type(self.projection, RuntimeProjection, "runtime.projection")
        _sha_field(self.identity, "runtime.identity")
        if runtime_projection_identity(self.projection) != self.identity:
            _fact_error("RUNTIME_FACT_IDENTITY")
        object.__setattr__(
            self, "runtime_root", _safe_rel(self.runtime_root, "runtime.root"),
        )


@dataclass(frozen=True, slots=True)
class _RuntimeFacts:
    projection_file: _FileFact
    projection: RuntimeProjection
    identity: str
    runtime_root: str
    members: tuple[_FileFact, ...]
    dependency_file: _FileFact
    dependencies: tuple[RuntimeDependency, ...]
    graph: object
    projection_reconciliation: RuntimeProjectionReconciliation
    dependency_closure: RuntimeDependencyClosure

    def __post_init__(self):
        _fact_type(self.projection_file, _FileFact, "runtime.projection_file")
        _fact_type(self.projection, RuntimeProjection, "runtime.projection")
        _sha_field(self.identity, "runtime.identity")
        if runtime_projection_identity(self.projection) != self.identity:
            _fact_error("RUNTIME_FACT_IDENTITY")
        object.__setattr__(self, "runtime_root", _safe_rel(self.runtime_root, "runtime.root"))
        members = _tuple_value(self.members, "runtime.members")
        if len(members) != 172 or any(type(member) is not _FileFact for member in members):
            _fact_error("RUNTIME_FACT_MEMBER_COUNT")
        if len({member.path for member in members}) != 172:
            _fact_error("RUNTIME_FACT_MEMBER_DUPLICATE")
        runtime_prefix = self.runtime_root.rstrip("/") + "/"
        if {member.path for member in members} != {
            runtime_prefix + member.path for member in self.projection.members
        }:
            _fact_error("RUNTIME_FACT_MEMBER_SET")
        object.__setattr__(self, "members", tuple(sorted(members, key=lambda member: member.path)))
        _fact_type(self.dependency_file, _FileFact, "runtime.dependency_file")
        dependencies = _tuple_value(self.dependencies, "runtime.dependencies")
        if len(dependencies) != 174 or any(type(edge) is not RuntimeDependency for edge in dependencies):
            _fact_error("RUNTIME_FACT_DEPENDENCY_COUNT")
        if len(set(dependencies)) != 174:
            _fact_error("RUNTIME_FACT_DEPENDENCY_DUPLICATE")
        object.__setattr__(self, "dependencies", tuple(dependencies))
        object.__setattr__(self, "graph", _freeze(self.graph))
        _fact_type(self.projection_reconciliation, RuntimeProjectionReconciliation,
                   "runtime.projection_reconciliation")
        _fact_type(self.dependency_closure, RuntimeDependencyClosure,
                   "runtime.dependency_closure")
        reconciliation = self.projection_reconciliation
        if (reconciliation.issues or reconciliation.declared_count != 172
                or reconciliation.physical_regular_file_count != 172
                or reconciliation.matched_count != 172):
            _fact_error("RUNTIME_FACT_RECONCILIATION")
        if self.dependency_closure.issues or len(self.dependency_closure.reachable_members) != 172:
            _fact_error("RUNTIME_FACT_DEPENDENCY_CLOSURE")


@dataclass(frozen=True, slots=True)
class _PlanFact:
    role: str
    file: _FileFact
    plan: RuntimePlan
    embedded_runtime_identity: str
    canonical_runtime_identity: str
    expected_runtime_identity: str

    def __post_init__(self):
        if self.role not in _PLAN_ROLES:
            _fact_error("PLAN_FACT_ROLE")
        _fact_type(self.file, _FileFact, "plan.file")
        _fact_type(self.plan, RuntimePlan, "plan.plan")
        for field, value in (
            ("embedded", self.embedded_runtime_identity),
            ("canonical", self.canonical_runtime_identity),
            ("expected", self.expected_runtime_identity),
        ):
            _sha_field(value, f"plan.{field}_runtime_identity")
        if not (self.embedded_runtime_identity == self.canonical_runtime_identity
                == self.expected_runtime_identity == self.plan.production_runtime_identity):
            _fact_error("PLAN_FACT_RUNTIME_IDENTITY", self.role)


@dataclass(frozen=True, slots=True)
class _RawPackageFact:
    case_id: str
    manifest: _FileFact
    package_identity: str
    members: tuple[tuple[str, str, int, str], ...]

    def __post_init__(self):
        if type(self.case_id) is not str or not re.fullmatch(r"case-(?:0(?:0[1-9]|[1-9][0-9])|1(?:[0-4][0-9]|50))", self.case_id):
            _fact_error("RAW_PACKAGE_FACT_CASE")
        _fact_type(self.manifest, _FileFact, "raw_package.manifest")
        _sha_field(self.package_identity, "raw_package.identity")
        members = _tuple_value(self.members, "raw_package.members")
        normalized = []
        for member in members:
            if type(member) not in (tuple, list) or len(member) != 4:
                _fact_error("RAW_PACKAGE_FACT_MEMBER")
            path, role, size, digest = member
            normalized.append((_safe_rel(path), _string(role, "raw_package.role"),
                               _count(size, "raw_package.size"),
                               _sha_field(digest, "raw_package.sha256")))
        if len(normalized) < 5 or len({item[0] for item in normalized}) != len(normalized):
            _fact_error("RAW_PACKAGE_FACT_MEMBERS")
        prefix = f"focused_raw/{self.case_id}/"
        if (not self.manifest.path.startswith(prefix)
                or any(not path.startswith(prefix) for path, _, _, _ in normalized)):
            _fact_error("RAW_PACKAGE_FACT_PATHS")
        object.__setattr__(self, "members", tuple(sorted(normalized)))
        projection = {
            "schema_version": "ctr-focused-raw-package-projection-1",
            "case_id": self.case_id,
            "members": [
                {"path": path, "role": role, "size": size, "sha256": digest}
                for path, role, size, digest in sorted(normalized)
            ],
        }
        if _canonical_digest(projection) != self.package_identity:
            _fact_error("RAW_PACKAGE_FACT_IDENTITY")


@dataclass(frozen=True, slots=True)
class _FocusedCaseFact:
    case_id: str
    category: str
    validator_origin: str
    passed: bool
    attachment_ids: tuple[str, ...]
    package: _RawPackageFact

    def __post_init__(self):
        if type(self.case_id) is not str or not re.fullmatch(r"case-(?:0(?:0[1-9]|[1-9][0-9])|1(?:[0-4][0-9]|50))", self.case_id):
            _fact_error("FOCUSED_FACT_CASE")
        if self.category not in _FOCUSED_CATEGORIES or self.validator_origin != _PROVENANCE[self.category]:
            _fact_error("FOCUSED_FACT_PROVENANCE")
        if type(self.passed) is not bool or self.passed is not True:
            _fact_error("FOCUSED_FACT_RESULT")
        attachments = _tuple_value(self.attachment_ids, "focused.attachments")
        if any(type(item) is not str or not item for item in attachments) or len(set(attachments)) != len(attachments):
            _fact_error("FOCUSED_FACT_ATTACHMENTS")
        object.__setattr__(self, "attachment_ids", tuple(attachments))
        _fact_type(self.package, _RawPackageFact, "focused.package")
        if self.package.case_id != self.case_id:
            _fact_error("FOCUSED_FACT_PACKAGE_CASE")


@dataclass(frozen=True, slots=True)
class _FocusedFacts:
    file: _FileFact
    cases: tuple[_FocusedCaseFact, ...]
    category_totals: tuple[tuple[str, int], ...]
    provenance_totals: tuple[tuple[str, int], ...]
    package_aggregate_sha256: str

    def __post_init__(self):
        _fact_type(self.file, _FileFact, "focused.file")
        cases = _tuple_value(self.cases, "focused.cases")
        if len(cases) != 150 or any(type(case) is not _FocusedCaseFact for case in cases):
            _fact_error("FOCUSED_FACT_COUNT")
        if len({case.case_id for case in cases}) != 150 or len({case.package.manifest.path for case in cases}) != 150:
            _fact_error("FOCUSED_FACT_DUPLICATE")
        object.__setattr__(self, "cases", tuple(sorted(cases, key=lambda case: case.case_id)))
        category_totals = _normalized_string_pairs(
            self.category_totals, "focused.category_totals", "integer",
        )
        provenance_totals = _normalized_string_pairs(
            self.provenance_totals, "focused.provenance_totals", "integer",
        )
        if dict(category_totals) != _FOCUSED_CATEGORIES:
            _fact_error("FOCUSED_FACT_CATEGORY_TOTALS")
        if dict(provenance_totals) != {"repository_production": 86, "candidate_evidence_integrity": 64}:
            _fact_error("FOCUSED_FACT_PROVENANCE_TOTALS")
        if dict((key, sum(case.category == key for case in cases)) for key in _FOCUSED_CATEGORIES) != _FOCUSED_CATEGORIES:
            _fact_error("FOCUSED_FACT_RECOMPUTED_TOTALS")
        if {key: sum(case.validator_origin == key for case in cases)
                for key in ("repository_production", "candidate_evidence_integrity")} != {
                    "repository_production": 86, "candidate_evidence_integrity": 64,
                }:
            _fact_error("FOCUSED_FACT_RECOMPUTED_PROVENANCE")
        object.__setattr__(self, "category_totals", category_totals)
        object.__setattr__(self, "provenance_totals", provenance_totals)
        _sha_field(self.package_aggregate_sha256, "focused.package_aggregate")
        aggregate = {
            "schema_version": "ctr-raw-package-aggregate-1",
            "packages": [
                {"case_id": case.case_id, "package_identity": case.package.package_identity}
                for case in sorted(cases, key=lambda item: item.case_id)
            ],
        }
        if _canonical_digest(aggregate) != self.package_aggregate_sha256:
            _fact_error("FOCUSED_FACT_PACKAGE_AGGREGATE")


@dataclass(frozen=True, slots=True)
class _AttachmentFact:
    attachment_id: str
    role: str
    case_id: str
    file: _FileFact

    def __post_init__(self):
        _string(self.attachment_id, "attachment.id")
        _string(self.role, "attachment.role")
        _string(self.case_id, "attachment.case_id")
        _fact_type(self.file, _FileFact, "attachment.file")
        expected = {attachment_id: (role, case_id) for attachment_id, role, case_id, _ in _ATTACHMENTS}
        if expected.get(self.attachment_id) != (self.role, self.case_id):
            _fact_error("ATTACHMENT_FACT_BINDING", self.attachment_id)


@dataclass(frozen=True, slots=True)
class _AttachmentFacts:
    manifest: _FileFact
    records: tuple[_AttachmentFact, ...]
    role_totals: tuple[tuple[str, int], ...]
    role_aggregates: tuple[tuple[str, str], ...]

    def __post_init__(self):
        _fact_type(self.manifest, _FileFact, "attachments.manifest")
        records = _tuple_value(self.records, "attachments.records")
        if len(records) != 28 or any(type(record) is not _AttachmentFact for record in records):
            _fact_error("ATTACHMENT_FACT_COUNT")
        if len({record.attachment_id for record in records}) != 28 or len({record.file.path for record in records}) != 28:
            _fact_error("ATTACHMENT_FACT_DUPLICATE")
        object.__setattr__(self, "records", tuple(sorted(records, key=lambda record: record.attachment_id)))
        totals = _normalized_string_pairs(
            self.role_totals, "attachments.role_totals", "integer",
        )
        if dict(totals) != {"authorization": 8, "allocation": 8, "durable_receipt": 8, "child_boundary": 4}:
            _fact_error("ATTACHMENT_FACT_TOTALS")
        if {role: sum(record.role == role for record in records) for role in dict(totals)} != dict(totals):
            _fact_error("ATTACHMENT_FACT_RECOMPUTED_TOTALS")
        aggregates = _normalized_string_pairs(
            self.role_aggregates, "attachments.role_aggregates", "sha256",
        )
        if {role for role, _ in aggregates} != {"authorization", "allocation", "durable_receipt", "child_boundary"}:
            _fact_error("ATTACHMENT_FACT_AGGREGATES")
        for role, digest in aggregates:
            selected = sorted(
                (record for record in records if record.role == role),
                key=lambda record: record.attachment_id,
            )
            projection = {
                "schema_version": "ctr-attachment-role-aggregate-1",
                "role": role,
                "attachments": [
                    {
                        "attachment_id": record.attachment_id,
                        "case_id": record.case_id,
                        "path": record.file.path,
                        "size": record.file.size,
                        "sha256": record.file.sha256,
                    }
                    for record in selected
                ],
            }
            if _canonical_digest(projection) != digest:
                _fact_error("ATTACHMENT_FACT_AGGREGATE", role)
        object.__setattr__(self, "role_totals", totals)
        object.__setattr__(self, "role_aggregates", aggregates)


@dataclass(frozen=True, slots=True)
class _CapsuleFact:
    file: _FileFact
    runtime_identity: str
    exercised_subject_identity: str
    policy: tuple[tuple[str, object], ...]

    def __post_init__(self):
        _fact_type(self.file, _FileFact, "capsule.file")
        _sha_field(self.runtime_identity, "capsule.runtime_identity")
        _sha_field(
            self.exercised_subject_identity,
            "capsule.exercised_subject_identity",
        )
        policy = _normalized_frozen_pairs(self.policy, "capsule.policy")
        if not _exact_equal(_thaw(policy), _CAPSULE_POLICY):
            _fact_error("CAPSULE_FACT_POLICY")
        object.__setattr__(self, "policy", policy)


@dataclass(frozen=True, slots=True)
class _ReportInputFacts:
    report_source: _FileFact
    static_closure: _FileFact
    correction_report: _FileFact
    source_identity: _FileFact
    predecessor_preservation: _FileFact

    def __post_init__(self):
        files = (self.report_source, self.static_closure, self.correction_report,
                 self.source_identity, self.predecessor_preservation)
        if any(type(file) is not _FileFact or file.size <= 0 for file in files):
            _fact_error("REPORT_INPUT_FACT")
        if len({file.path for file in files}) != 5:
            _fact_error("REPORT_INPUT_FACT_ALIAS")


@dataclass(frozen=True, slots=True)
class _ClosureSourceFact:
    source_fact_id: str
    semantic_category: str
    source_path: str
    role: str
    size: int
    sha256: str
    mode: str
    assertion_kind: str = "physical_file_identity"
    derivation_rule: str = "inventory-declaration-vs-descriptor-authenticated-physical-file"

    def __post_init__(self):
        _string(self.source_fact_id, "closure_source.source_fact_id")
        _string(self.semantic_category, "closure_source.semantic_category")
        object.__setattr__(self, "source_path", _safe_rel(self.source_path, "closure_source.source_path"))
        _string(self.role, "closure_source.role")
        _count(self.size, "closure_source.size")
        _sha_field(self.sha256, "closure_source.sha256")
        if type(self.mode) is not str or not re.fullmatch(r"0[0-7]{3}", self.mode):
            _fact_error("CLOSURE_SOURCE_MODE", self.source_path)
        if (self.assertion_kind != "physical_file_identity"
                or self.derivation_rule != "inventory-declaration-vs-descriptor-authenticated-physical-file"):
            _fact_error("CLOSURE_SOURCE_DERIVATION", self.source_path)

    @property
    def semantic_key(self):
        return (
            self.assertion_kind, self.semantic_category, self.source_path,
            self.role, self.size, self.sha256, self.mode, self.derivation_rule,
        )

    def projection_record(self):
        identity = {"mode": self.mode, "sha256": self.sha256, "size": self.size}
        return {
            "assertion_kind": self.assertion_kind,
            "derivation_rule": self.derivation_rule,
            "expected": dict(identity),
            "observed": dict(identity),
            "role": self.role,
            "semantic_category": self.semantic_category,
            "sha256": self.sha256,
            "size": self.size,
            "source_fact_id": self.source_fact_id,
            "source_path": self.source_path,
        }


@dataclass(frozen=True, slots=True)
class _ClosureProjectionFacts:
    file: _FileFact
    records: tuple[_ClosureSourceFact, ...]
    exclusions: tuple[str, ...]
    category_totals: tuple[tuple[str, int], ...]

    def __post_init__(self):
        _fact_type(self.file, _FileFact, "closure_source.file")
        records = _tuple_value(self.records, "closure_source.records")
        if not records or any(type(record) is not _ClosureSourceFact for record in records):
            _fact_error("CLOSURE_SOURCE_RECORDS")
        if (len({record.source_fact_id for record in records}) != len(records)
                or len({record.source_path for record in records}) != len(records)
                or len({record.semantic_key for record in records}) != len(records)):
            _fact_error("CLOSURE_DUPLICATE_SEMANTIC_SOURCE_ASSERTION")
        object.__setattr__(self, "records", tuple(sorted(records, key=lambda record: record.source_fact_id)))
        exclusions = tuple(sorted(_safe_rel(path) for path in _tuple_value(self.exclusions, "closure_source.exclusions")))
        if len(set(exclusions)) != len(exclusions):
            _fact_error("CLOSURE_DEPENDENCY_CYCLE")
        object.__setattr__(self, "exclusions", exclusions)
        totals = _normalized_string_pairs(self.category_totals, "closure_source.category_totals", "integer")
        actual = {category: sum(record.semantic_category == category for record in records)
                  for category in {record.semantic_category for record in records}}
        if dict(totals) != actual:
            _fact_error("CLOSURE_SOURCE_PROJECTION_MISMATCH")
        object.__setattr__(self, "category_totals", totals)


@dataclass(frozen=True, slots=True)
class _MaterializationFacts:
    projection_file: _FileFact
    source_identity_file: _FileFact
    projection: MaterializationProjection
    materialization_root: str
    projection_sha256: str
    logical_identity: str
    projection_framing_digest: str
    physical_rehash: str
    runtime_binding_count: int
    verification: MaterializationPhysicalVerificationResult

    def __post_init__(self):
        _fact_type(self.projection_file, _FileFact, "materialization.projection_file")
        _fact_type(self.source_identity_file, _FileFact, "materialization.source_identity_file")
        _fact_type(self.projection, MaterializationProjection, "materialization.projection")
        object.__setattr__(
            self, "materialization_root",
            _safe_rel(self.materialization_root, "materialization.root"),
        )
        for field in (
            "projection_sha256", "logical_identity",
            "projection_framing_digest", "physical_rehash",
        ):
            _sha_field(getattr(self, field), "materialization." + field)
        if self.projection_file.sha256 != self.projection_sha256:
            _fact_error("MATERIALIZATION_PROJECTION_MISMATCH")
        if (materialization_root_identity(self.projection) != self.logical_identity
                or materialization_projection_framing_digest(self.projection)
                != self.projection_framing_digest):
            _fact_error("MATERIALIZATION_IDENTITY_MISMATCH")
        if type(self.runtime_binding_count) is not int or self.runtime_binding_count != 172:
            _fact_error("MATERIALIZATION_RUNTIME_BINDING")
        _fact_type(
            self.verification, MaterializationPhysicalVerificationResult,
            "materialization.verification",
        )
        retained = self.verification.projection_identity
        if (retained.projection != self.projection
                or retained.projection_sha256 != self.projection_sha256
                or retained.logical_identity != self.logical_identity
                or retained.projection_framing_digest != self.projection_framing_digest
                or self.verification.physical_rehash != self.physical_rehash
                or len(self.verification.observed_members) != len(self.projection.members)):
            _fact_error("MATERIALIZATION_PHYSICAL_VERIFICATION_FACT")


@dataclass(frozen=True, slots=True)
class _ExercisedSubjectFacts:
    file: _FileFact
    subject: ExercisedSubject
    identity: str

    def __post_init__(self):
        _fact_type(self.file, _FileFact, "exercised_subject.file")
        _fact_type(self.subject, ExercisedSubject, "exercised_subject.subject")
        _sha_field(self.identity, "exercised_subject.identity")
        if exercised_subject_identity(self.subject) != self.identity:
            _fact_error("SUBJECT_LOGICAL_IDENTITY_MISMATCH")


@dataclass(frozen=True, slots=True)
class _InvocationFacts:
    candidate_basename: str
    expected_root_authority: str
    expected_runtime_identity: str
    side_effects: tuple[tuple[str, object], ...]

    def __post_init__(self):
        _string(self.candidate_basename, "invocation.candidate_basename")
        if "/" in self.candidate_basename or "\\" in self.candidate_basename:
            _fact_error("INVOCATION_BASENAME")
        _sha_field(self.expected_root_authority, "invocation.root_authority")
        _sha_field(self.expected_runtime_identity, "invocation.runtime_identity")
        side_effects = _normalized_side_effects(self.side_effects)
        if not _exact_equal(dict(side_effects), _SIDE_EFFECTS):
            _fact_error("INVOCATION_SIDE_EFFECTS")
        object.__setattr__(self, "side_effects", side_effects)


@dataclass(frozen=True, slots=True)
class _CandidateFacts:
    bundle: _BundleFacts
    inventory: _InventoryFacts
    authority: _AuthorityFacts
    runtime: _RuntimeFacts
    plans: tuple[_PlanFact, ...]
    focused: _FocusedFacts
    attachments: _AttachmentFacts
    capsule: _CapsuleFact
    report_inputs: _ReportInputFacts
    closure_source: _ClosureProjectionFacts
    materialization: _MaterializationFacts
    exercised_subject: _ExercisedSubjectFacts
    invocation: _InvocationFacts

    def __post_init__(self):
        expected_types = (
            ("bundle", _BundleFacts), ("inventory", _InventoryFacts),
            ("authority", _AuthorityFacts), ("runtime", _RuntimeFacts),
            ("focused", _FocusedFacts), ("attachments", _AttachmentFacts),
            ("capsule", _CapsuleFact), ("report_inputs", _ReportInputFacts),
            ("closure_source", _ClosureProjectionFacts),
            ("materialization", _MaterializationFacts),
            ("exercised_subject", _ExercisedSubjectFacts),
            ("invocation", _InvocationFacts),
        )
        for field, expected in expected_types:
            _fact_type(getattr(self, field), expected, f"candidate.{field}")
        plans = _tuple_value(self.plans, "candidate.plans")
        if len(plans) != 6 or any(type(plan) is not _PlanFact for plan in plans):
            _fact_error("CANDIDATE_FACT_PLANS")
        if {plan.role for plan in plans} != set(_PLAN_ROLES):
            _fact_error("CANDIDATE_FACT_PLAN_ROLES")
        object.__setattr__(self, "plans", tuple(sorted(plans, key=lambda plan: _PLAN_ROLES.index(plan.role))))
        if not (self.runtime.identity == self.capsule.runtime_identity
                == self.invocation.expected_runtime_identity):
            _fact_error("CANDIDATE_FACT_RUNTIME_IDENTITY")
        if self.exercised_subject.identity != self.capsule.exercised_subject_identity:
            _fact_error("CANDIDATE_FACT_SUBJECT_IDENTITY")
        if self.authority.expected_sha256 != self.invocation.expected_root_authority:
            _fact_error("CANDIDATE_FACT_AUTHORITY")
        authority_paths = {
            role: self.bundle.file.path if field is None else self.bundle.path(field)
            for role, field in _AUTHORITY_ROLE_PATH_FIELDS
        }
        if {child.role: child.file.path for child in self.authority.children} != authority_paths:
            _fact_error("CANDIDATE_FACT_AUTHORITY_CHILDREN")
        inventory_paths = {member.path for member in self.inventory.members}
        required_inventory_paths = {
            self.bundle.file.path, self.runtime.projection_file.path,
            self.runtime.dependency_file.path, self.focused.file.path,
            self.attachments.manifest.path, self.capsule.file.path,
            self.exercised_subject.file.path,
            *(plan.file.path for plan in self.plans),
        }
        if not required_inventory_paths.issubset(inventory_paths):
            _fact_error("CANDIDATE_FACT_INVENTORY_BINDING")
        if (self.closure_source.file.path != self.bundle.path("closure_source_projection_path")
                or self.materialization.projection_file.path != self.bundle.path("materialization_projection_path")
                or self.exercised_subject.file.path != self.bundle.path("exercised_subject_path")
                or self.materialization.source_identity_file != self.report_inputs.source_identity):
            _fact_error("CANDIDATE_FACT_CONTRACT_BINDING")
        report_paths = {
            self.report_inputs.report_source.path,
            self.report_inputs.static_closure.path,
            self.report_inputs.correction_report.path,
            self.report_inputs.source_identity.path,
            self.report_inputs.predecessor_preservation.path,
        }
        if report_paths != {
            self.bundle.path(field) for field in (
                "report_source_path", "static_closure_path", "correction_report_path",
                "source_identity_path", "predecessor_preservation_path",
            )
        }:
            _fact_error("CANDIDATE_FACT_REPORT_INPUTS")


@dataclass(frozen=True, slots=True)
class _Observation:
    key: str
    category: str
    value: object
    source_paths: tuple[str, ...]

    def __post_init__(self):
        _string(self.key, "observation.key")
        if _OBSERVATION_KEY_CATEGORIES.get(self.key) != self.category:
            _fact_error("OBSERVATION_KEY_CATEGORY", self.key)
        object.__setattr__(self, "value", _freeze(self.value))
        paths = _tuple_value(self.source_paths, "observation.source_paths")
        normalized = tuple(sorted({_safe_rel(path, "observation.source_path") for path in paths}))
        object.__setattr__(self, "source_paths", normalized)


@dataclass(frozen=True, slots=True)
class _ReportFacts:
    file: _FileFact

    def __post_init__(self):
        _fact_type(self.file, _FileFact, "report.file")


@dataclass(frozen=True, slots=True)
class _ClosureFacts:
    file: _FileFact
    critical_records: int
    source_records: int
    category_count: int

    def __post_init__(self):
        _fact_type(self.file, _FileFact, "closure.file")
        if type(self.critical_records) is not int or self.critical_records != 35:
            _fact_error("CLOSURE_FACT_CRITICAL")
        if type(self.source_records) is not int or self.source_records <= 0:
            _fact_error("CLOSURE_FACT_SOURCE")
        if type(self.category_count) is not int or self.category_count <= 0:
            _fact_error("CLOSURE_FACT_CATEGORIES")


def _safe_rel(value, field="path") -> str:
    if (type(value) is not str or not value or "\\" in value or "\x00" in value
            or any(unicodedata.category(c) in {"Cc", "Cs"} for c in value)):
        raise CandidateValidateOnlyError("PATH_INVALID", field)
    if unicodedata.normalize("NFC", value) != value:
        raise CandidateValidateOnlyError("PATH_INVALID", field)
    p = PurePosixPath(value)
    if (p.is_absolute() or str(p) != value or any(x in {"", ".", ".."} for x in p.parts)
            or re.match(r"^[A-Za-z][A-Za-z0-9+.-]*:", value)):
        raise CandidateValidateOnlyError("PATH_INVALID", field)
    return value


def _digest(data: bytes) -> str:
    return sha256(data).hexdigest()


def _sha_field(value, field):
    if type(value) is not str or not _HEX.fullmatch(value):
        raise CandidateValidateOnlyError("DIGEST_INVALID", field)
    return value


def _json(raw: bytes, label: str):
    try:
        def hook(pairs):
            out = {}
            for key, value in pairs:
                if key in out:
                    raise CandidateValidateOnlyError("JSON_DUPLICATE_KEY", label)
                out[key] = value
            return out
        value = json.loads(raw.decode("utf-8"), object_pairs_hook=hook,
                           parse_constant=lambda _: (_ for _ in ()).throw(CandidateValidateOnlyError("JSON_CONSTANT", label)))
        if type(value) is not dict:
            raise CandidateValidateOnlyError("JSON_TOP_LEVEL_TYPE", label)
        return value
    except CandidateValidateOnlyError:
        raise
    except (UnicodeError, ValueError, TypeError) as exc:
        raise CandidateValidateOnlyError("JSON_INVALID", label) from exc


def _freeze(value):
    if type(value) is dict:
        if any(type(key) is not str for key in value):
            raise CandidateValidateOnlyError("IMMUTABLE_MAPPING_KEY")
        return tuple((key, _freeze(item)) for key, item in sorted(value.items()))
    if type(value) in (list, tuple):
        return tuple(_freeze(v) for v in value)
    if type(value) is float and (value != value or value in (float("inf"), float("-inf"))):
        raise CandidateValidateOnlyError("IMMUTABLE_VALUE_NONFINITE")
    if value is None or type(value) in (str, int, float, bool):
        return value
    raise CandidateValidateOnlyError("IMMUTABLE_VALUE_TYPE")


def _thaw(value):
    if type(value) is tuple:
        if all(type(x) is tuple and len(x) == 2 and type(x[0]) is str for x in value):
            return {k: _thaw(v) for k, v in value}
        return [_thaw(v) for v in value]
    return value


def _canonical_digest(value) -> str:
    return _digest(json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8"))


def _exact_equal(left, right) -> bool:
    if type(left) is not type(right):
        return False
    if type(left) is dict:
        return set(left) == set(right) and all(_exact_equal(left[key], right[key]) for key in left)
    if type(left) in (list, tuple):
        return len(left) == len(right) and all(_exact_equal(a, b) for a, b in zip(left, right))
    return left == right


def _repository_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _repository_branch_head(root=None):
    root = _repository_root() if root is None else Path(root)
    git = root / ".git"
    try:
        if git.is_file():
            content = git.read_text(encoding="utf-8").strip()
            if not content.startswith("gitdir: "):
                raise ValueError
            git = (root / content[8:]).resolve()
        head_text = (git / "HEAD").read_text(encoding="ascii").strip()
        if head_text.startswith("ref: refs/heads/"):
            reference = head_text[5:]
            branch = reference[len("refs/heads/"):]
            ref_path = git / reference
            if ref_path.is_file():
                head = ref_path.read_text(encoding="ascii").strip()
            else:
                head = ""
                for line in (git / "packed-refs").read_text(encoding="ascii").splitlines():
                    if line and not line.startswith(("#", "^")):
                        digest, name = line.split(" ", 1)
                        if name == reference:
                            head = digest
                            break
        else:
            branch, head = "DETACHED", head_text
    except (OSError, UnicodeError, ValueError) as exc:
        raise CandidateValidateOnlyError("REPOSITORY_IDENTITY_UNAVAILABLE") from exc
    if (type(branch) is not str or not branch or any(ord(c) < 32 for c in branch)
            or type(head) is not str or not re.fullmatch(r"[0-9a-f]{40}", head)):
        raise CandidateValidateOnlyError("REPOSITORY_IDENTITY_INVALID")
    return branch, head


def _repository_identity_snapshot():
    root = _repository_root()
    branch, head = _repository_branch_head(root)
    files = []
    for role, relative in _SOURCE_IDENTITY_FILES:
        path = root / relative
        try:
            raw = path.read_bytes()
        except OSError as exc:
            raise CandidateValidateOnlyError("REPOSITORY_SOURCE_MISSING", relative) from exc
        files.append({
            "path": relative, "role": role, "sha256": _digest(raw), "size": len(raw),
        })
    return {"branch": branch, "files": files, "head": head}


def _closure_source_role(
    path,
    *,
    materialization_root="materialization",
    runtime_root="materialization/runtime_root",
):
    path = _safe_rel(path, "closure_source.path")
    materialization_root = _safe_rel(
        materialization_root, "closure_source.materialization_root",
    )
    runtime_root = _safe_rel(runtime_root, "closure_source.runtime_root")
    if path.startswith(runtime_root.rstrip("/") + "/"):
        return "runtime", "runtime_member"
    if path.startswith(materialization_root.rstrip("/") + "/"):
        return "materialization", "materialization_member"
    if path.startswith("focused_raw/"):
        name = PurePosixPath(path).name
        if name == "manifest.json":
            return "focused_evidence", "raw_package_manifest"
        if name in {"attachment.json", "attachment.bin"}:
            return "attachment_evidence", "attachment"
        return "focused_evidence", "raw_package_member"
    if path.startswith("plans/"):
        return "plan_evidence", "runtime_plan"
    if path.startswith("reports/"):
        return "report_input", "correction_report"
    known = {
        "manifests/validate_only_bundle.json": ("candidate_contract", "candidate_bundle"),
        "manifests/payload_identity_projection.json": ("runtime_contract", "runtime_projection"),
        "manifests/runtime_dependency_graph.json": ("runtime_contract", "runtime_dependency_graph"),
        "manifests/plans.json": ("plan_evidence", "six_plan_manifest"),
        "manifests/focused_results.json": ("focused_evidence", "focused_results"),
        "manifests/attachments.json": ("attachment_evidence", "attachment_manifest"),
        "manifests/capsule.json": ("policy", "validate_only_capsule"),
        "manifests/source_identity.json": ("repository_contract", "source_identity"),
        "manifests/predecessor_preservation.json": ("preservation", "predecessor_preservation"),
        "manifests/materialization_projection.json": ("materialization", "materialization_projection"),
        EXERCISED_SUBJECT_RECORD_PATH: (
            "exercised_subject", "exercised_subject_record",
        ),
    }
    return known.get(path, ("candidate_contract", "authenticated_candidate_file"))


def _closure_source_fact(
    file,
    *,
    materialization_root="materialization",
    runtime_root="materialization/runtime_root",
):
    category, role = _closure_source_role(
        file.path,
        materialization_root=materialization_root,
        runtime_root=runtime_root,
    )
    return _ClosureSourceFact(
        "physical-file:" + file.path, category, file.path, role,
        file.size, file.sha256, file.mode,
    )


def _closure_projection_value(records, exclusions):
    records = tuple(sorted(records, key=lambda record: record.source_fact_id))
    category_totals = {
        category: sum(record.semantic_category == category for record in records)
        for category in sorted({record.semantic_category for record in records})
    }
    return {
        "category_counts": category_totals,
        "exclusions": sorted(exclusions),
        "fact_count": len(records),
        "facts": [record.projection_record() for record in records],
        "inclusion_rule": "all-physically-authenticated-inventory-regular-files",
        "schema_version": CLOSURE_SOURCE_PROJECTION_SCHEMA,
    }


class _CandidateReadSession:
    def __init__(self, candidate_root):
        if type(candidate_root) not in (str, type(Path("."))):
            raise CandidateValidateOnlyError("CANDIDATE_ROOT_TYPE")
        self.root = Path(candidate_root)
        if not self.root.is_absolute():
            raise CandidateValidateOnlyError("CANDIDATE_ROOT_ABSOLUTE")
        if not hasattr(os, "O_NOFOLLOW") or not hasattr(os, "O_DIRECTORY"):
            raise CandidateValidateOnlyError("CANDIDATE_NOFOLLOW_UNAVAILABLE")
        try:
            self.fd = os.open(self.root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | getattr(os, "O_CLOEXEC", 0))
        except (OSError, TypeError) as exc:
            raise CandidateValidateOnlyError("CANDIDATE_ROOT_INVALID") from exc
        self.data, self.meta, self.paths = {}, {}, set()
        self.file_meta = {}
        self.directories, self.directory_meta, self._inodes = set(), {}, set()
        try:
            st = os.fstat(self.fd)
            if not stat.S_ISDIR(st.st_mode) or st.st_mode & 0o222:
                raise CandidateValidateOnlyError("CANDIDATE_ROOT_POLICY")
            self.root_id = self._directory_metadata(st)
            self._walk(self.fd, "")
            self.check_root()
        except CandidateValidateOnlyError:
            self.close()
            raise
        except OSError as exc:
            self.close()
            raise CandidateValidateOnlyError("CANDIDATE_TRAVERSAL") from exc

    @staticmethod
    def _directory_metadata(value):
        return (
            value.st_dev, value.st_ino, stat.S_IFMT(value.st_mode), value.st_mode,
            value.st_nlink, value.st_size, value.st_mtime_ns, value.st_ctime_ns,
        )

    @staticmethod
    def _file_metadata(value, digest):
        return (
            value.st_dev, value.st_ino, stat.S_IFMT(value.st_mode), value.st_mode,
            value.st_nlink, value.st_size, value.st_mtime_ns, value.st_ctime_ns,
            digest,
        )

    @staticmethod
    def _changed(detail=""):
        return CandidateValidateOnlyError(
            "CANDIDATE_CHANGED_AFTER_AUTHENTICATION", detail,
            stage="candidate_inventory",
        )

    def _walk(self, directory_fd, prefix):
        for name in os.listdir(directory_fd):
            rel = f"{prefix}/{name}" if prefix else name
            _safe_rel(rel, "candidate.path")
            if (name in _TRANSIENT or name.endswith((".pyc", ".pyo"))
                    or name.startswith(".coverage.")):
                raise CandidateValidateOnlyError("INVENTORY_TRANSIENT", rel)
            try:
                fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | getattr(os, "O_CLOEXEC", 0), dir_fd=directory_fd)
            except OSError as exc:
                raise CandidateValidateOnlyError("CANDIDATE_OPEN", rel) from exc
            try:
                st = os.fstat(fd)
                if stat.S_ISDIR(st.st_mode):
                    if st.st_mode & 0o222:
                        raise CandidateValidateOnlyError("INVENTORY_WRITABLE_DIR", rel)
                    self.directories.add(rel)
                    baseline = self._directory_metadata(st)
                    self.directory_meta[rel] = baseline
                    self._walk(fd, rel)
                    if self._directory_metadata(os.fstat(fd)) != baseline:
                        raise CandidateValidateOnlyError("CANDIDATE_MUTATED", rel)
                elif stat.S_ISREG(st.st_mode):
                    if st.st_mode & 0o222 or st.st_nlink != 1:
                        raise CandidateValidateOnlyError("INVENTORY_FILE_POLICY", rel)
                    inode = (st.st_dev, st.st_ino)
                    if inode in self._inodes:
                        raise CandidateValidateOnlyError("INVENTORY_HARDLINK", rel)
                    self._inodes.add(inode)
                    before = self._file_metadata(st, "")[:-1]
                    parts, h = [], sha256()
                    while True:
                        chunk = os.read(fd, 1024 * 1024)
                        if not chunk:
                            break
                        parts.append(chunk)
                        h.update(chunk)
                    after = os.fstat(fd)
                    now = self._file_metadata(after, "")[:-1]
                    if before != now:
                        raise CandidateValidateOnlyError("CANDIDATE_MUTATED", rel)
                    self.data[rel] = b"".join(parts)
                    self.meta[rel] = (after.st_size, h.hexdigest(), f"{stat.S_IMODE(after.st_mode):04o}")
                    self.file_meta[rel] = self._file_metadata(after, h.hexdigest())
                    self.paths.add(rel)
                else:
                    raise CandidateValidateOnlyError("INVENTORY_NONREGULAR", rel)
            finally:
                os.close(fd)

    def check_root(self):
        descriptor = self._directory_metadata(os.fstat(self.fd))
        pathname = self._directory_metadata(os.stat(self.root, follow_symlinks=False))
        if descriptor != self.root_id or pathname != self.root_id:
            raise CandidateValidateOnlyError("CANDIDATE_ROOT_CHANGED")

    def _verify_walk(self, directory_fd, prefix, directories, files, inodes):
        try:
            names = os.listdir(directory_fd)
        except OSError as exc:
            raise self._changed(prefix) from exc
        for name in names:
            rel = f"{prefix}/{name}" if prefix else name
            try:
                _safe_rel(rel, "candidate.path")
                if (name in _TRANSIENT or name.endswith((".pyc", ".pyo"))
                        or name.startswith(".coverage.")):
                    raise self._changed(rel)
                fd = os.open(
                    name,
                    os.O_RDONLY | os.O_NOFOLLOW | getattr(os, "O_CLOEXEC", 0),
                    dir_fd=directory_fd,
                )
            except CandidateValidateOnlyError as exc:
                if exc.code == "CANDIDATE_CHANGED_AFTER_AUTHENTICATION":
                    raise
                raise self._changed(rel) from exc
            except OSError as exc:
                raise self._changed(rel) from exc
            try:
                before = os.fstat(fd)
                if stat.S_ISDIR(before.st_mode):
                    if before.st_mode & 0o222 or rel not in self.directory_meta:
                        raise self._changed(rel)
                    metadata = self._directory_metadata(before)
                    if metadata != self.directory_meta[rel]:
                        raise self._changed(rel)
                    directories.add(rel)
                    self._verify_walk(fd, rel, directories, files, inodes)
                    if self._directory_metadata(os.fstat(fd)) != metadata:
                        raise self._changed(rel)
                elif stat.S_ISREG(before.st_mode):
                    if before.st_mode & 0o222 or before.st_nlink != 1 or rel not in self.file_meta:
                        raise self._changed(rel)
                    inode = (before.st_dev, before.st_ino)
                    if inode in inodes:
                        raise self._changed(rel)
                    inodes.add(inode)
                    baseline = self.file_meta[rel]
                    if self._file_metadata(before, baseline[-1]) != baseline:
                        raise self._changed(rel)
                    digest = sha256()
                    while True:
                        chunk = os.read(fd, 1024 * 1024)
                        if not chunk:
                            break
                        digest.update(chunk)
                    after = os.fstat(fd)
                    if self._file_metadata(after, digest.hexdigest()) != baseline:
                        raise self._changed(rel)
                    files.add(rel)
                else:
                    raise self._changed(rel)
            except OSError as exc:
                raise self._changed(rel) from exc
            finally:
                os.close(fd)

    def verify_current_tree(self, runtime_root):
        """Reauthenticate the complete frozen tree at the final success barrier."""
        directories, files, inodes = set(), set(), set()
        try:
            root_before = os.fstat(self.fd)
            if (not stat.S_ISDIR(root_before.st_mode) or root_before.st_mode & 0o222
                    or self._directory_metadata(root_before) != self.root_id):
                raise self._changed(".")
            self._verify_walk(self.fd, "", directories, files, inodes)
            if directories != self.directories or files != self.paths:
                raise self._changed("inventory")
            root_after = os.fstat(self.fd)
            if self._directory_metadata(root_after) != self.root_id:
                raise self._changed(".")
            self.check_directory(runtime_root)
            pathname = os.stat(self.root, follow_symlinks=False)
            if self._directory_metadata(pathname) != self.root_id:
                raise self._changed(".")
        except CandidateValidateOnlyError as exc:
            if exc.code == "CANDIDATE_CHANGED_AFTER_AUTHENTICATION":
                raise
            raise self._changed(exc.detail) from exc
        except OSError as exc:
            raise self._changed("tree") from exc

    def bytes(self, rel):
        rel = _safe_rel(rel)
        if rel not in self.data:
            raise CandidateValidateOnlyError("FILE_MISSING", rel)
        return self.data[rel]

    def obj(self, rel):
        return _json(self.bytes(rel), rel)

    def fact(self, rel):
        rel = _safe_rel(rel)
        data = self.bytes(rel)
        size, digest, mode = self.meta[rel]
        if size != len(data) or digest != _digest(data):
            raise CandidateValidateOnlyError("CACHE_DIVERGENCE", rel)
        return _FileFact(rel, size, digest, mode)

    def check_directory(self, rel):
        rel = _safe_rel(rel, "directory")
        if rel not in self.directory_meta:
            raise CandidateValidateOnlyError("DIRECTORY_MISSING", rel)
        current_fd = os.dup(self.fd)
        try:
            for segment in PurePosixPath(rel).parts:
                next_fd = os.open(
                    segment,
                    os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | getattr(os, "O_CLOEXEC", 0),
                    dir_fd=current_fd,
                )
                os.close(current_fd)
                current_fd = next_fd
            st = os.fstat(current_fd)
            current = self._directory_metadata(st)
            if current != self.directory_meta[rel]:
                raise CandidateValidateOnlyError("DIRECTORY_CHANGED", rel)
        except CandidateValidateOnlyError:
            raise
        except OSError as exc:
            raise CandidateValidateOnlyError("DIRECTORY_CHANGED", rel) from exc
        finally:
            os.close(current_fd)

    def close(self):
        if getattr(self, "fd", -1) >= 0:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = -1

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def _load_bundle(session, bundle_path) -> _BundleFacts:
    file = session.fact(bundle_path)
    value = session.obj(bundle_path)
    if value.get("schema") == "ctr-frozen-candidate-bundle-1":
        raise CandidateValidateOnlyError("LEGACY_DUPLICATED_CLOSURE")
    if set(value) != _BUNDLE_FIELDS or value["schema"] != BUNDLE_SCHEMA or value["profile"] != "slice-7f-final":
        raise CandidateValidateOnlyError("BUNDLE_SCHEMA")
    if value["inventory_path"] != "manifests/candidate_inventory.json":
        raise CandidateValidateOnlyError("BUNDLE_FIXED_PATH")
    if value["exercised_subject_path"] != EXERCISED_SUBJECT_RECORD_PATH:
        raise CandidateValidateOnlyError("SUBJECT_FIXED_PATH")
    paths = tuple((field, _safe_rel(value[field], field)) for field in _BUNDLE_PATH_FIELDS)
    selected = [path for _, path in paths]
    if len(set(selected)) != len(selected) or "manifests/root_authority.json" in selected:
        raise CandidateValidateOnlyError("BUNDLE_PATH_ALIAS")
    materialization_root = value["materialization_root_path"].rstrip("/") + "/"
    for field in _BUNDLE_FILE_PATH_FIELDS:
        selected_path = value[field]
        if selected_path.startswith(materialization_root):
            raise CandidateValidateOnlyError("BUNDLE_PATH_ALIAS", field)
    for field in _BUNDLE_FILE_PATH_FIELDS:
        try:
            session.fact(value[field])
        except CandidateValidateOnlyError as error:
            if field == "materialization_projection_path" and error.code == "FILE_MISSING":
                raise CandidateValidateOnlyError("MATERIALIZATION_PROJECTION_MISSING") from error
            raise
    return _BundleFacts(file, paths)


def _inventory_exclusions(bundle):
    exclusions = {_ROOT_AUTHORITY_PATH}
    exclusions.update(bundle.path(field) for field in _FORMAL_META_FIELDS)
    if len(exclusions) != 5 or bundle.file.path in exclusions:
        raise CandidateValidateOnlyError("CLOSURE_DEPENDENCY_CYCLE")
    return frozenset(exclusions)


def _validate_inventory(session, bundle) -> _InventoryFacts:
    file = session.fact(bundle.path("inventory_path"))
    value = session.obj(file.path)
    if set(value) != {"schema", "files"} or value["schema"] != INVENTORY_SCHEMA or type(value["files"]) is not list:
        raise CandidateValidateOnlyError("INVENTORY_SCHEMA")
    exclusions = _inventory_exclusions(bundle)
    members, seen = [], set()
    for record in value["files"]:
        if set(record) != {"path", "size", "sha256", "mode"}:
            raise CandidateValidateOnlyError("INVENTORY_RECORD")
        path = _safe_rel(record["path"])
        if path in exclusions:
            raise CandidateValidateOnlyError("CLOSURE_DEPENDENCY_CYCLE", path)
        if path in seen or type(record["size"]) is not int or record["size"] < 0 or type(record["mode"]) is not str:
            raise CandidateValidateOnlyError("INVENTORY_RECORD", path)
        _sha_field(record["sha256"], path)
        fact = session.fact(path)
        if (record["size"], record["sha256"], record["mode"]) != (fact.size, fact.sha256, fact.mode):
            raise CandidateValidateOnlyError("INVENTORY_DIGEST_MISMATCH", path)
        seen.add(path)
        members.append(fact)
    physical = session.paths - exclusions
    subject_path = bundle.path("exercised_subject_path")
    if subject_path not in seen:
        raise CandidateValidateOnlyError("SUBJECT_RECORD_NOT_IN_INVENTORY")
    if seen != physical:
        raise CandidateValidateOnlyError("INVENTORY_SET_MISMATCH")
    return _InventoryFacts(file, tuple(sorted(members, key=lambda x: x.path)))


def _validate_authority(session, bundle, inventory, expected_root_authority) -> _AuthorityFacts:
    file = session.fact(_ROOT_AUTHORITY_PATH)
    if file.sha256 != expected_root_authority:
        raise CandidateValidateOnlyError("ROOT_AUTHORITY_MISMATCH")
    value = session.obj(file.path)
    if (set(value) != {"schema", "children"} or value["schema"] != AUTHORITY_SCHEMA
            or type(value["children"]) is not list):
        raise CandidateValidateOnlyError("AUTHORITY_SCHEMA")
    if len(value["children"]) == len(_AUTHORITY_ROLE_PATH_FIELDS) - 1:
        raise CandidateValidateOnlyError("LEGACY_13_AUTHORITY_ROLES")
    if len(value["children"]) != len(_AUTHORITY_ROLE_PATH_FIELDS):
        raise CandidateValidateOnlyError("AUTHORITY_SCHEMA")
    children, paths, roles = [], set(), set()
    for child in value["children"]:
        if set(child) != {"path", "size", "sha256", "role"}:
            raise CandidateValidateOnlyError("AUTHORITY_CHILD_SCHEMA")
        if type(child["path"]) is not str or type(child["sha256"]) is not str:
            raise CandidateValidateOnlyError("AUTHORITY_CHILD_SCHEMA")
        _sha_field(child["sha256"], "authority.child.sha256")
        fact = session.fact(child["path"])
        role = child["role"]
        if type(role) is not str or not role:
            raise CandidateValidateOnlyError("AUTHORITY_CHILD_SCHEMA")
        if role == "exercised_subject" and (
            type(child["size"]) is not int
            or child["size"] != fact.size
            or child["sha256"] != fact.sha256
        ):
            raise CandidateValidateOnlyError(
                "SUBJECT_PHYSICAL_DIGEST_MISMATCH", fact.path,
            )
        if role == "exercised_subject" and role in roles:
            raise CandidateValidateOnlyError("SUBJECT_AUTHORITY_ROLE_DUPLICATE")
        if (fact.path in paths or role in roles or type(child["size"]) is not int
                or child["size"] != fact.size or child["sha256"] != fact.sha256):
            raise CandidateValidateOnlyError("AUTHORITY_CHILD_MISMATCH", fact.path)
        paths.add(fact.path); roles.add(role)
        children.append(_AuthorityChildFact(role, fact))
    expected = {
        role: bundle.file.path if field is None else bundle.path(field)
        for role, field in _AUTHORITY_ROLE_PATH_FIELDS
    }
    actual = {child.role: child.file.path for child in children}
    if "exercised_subject" not in actual:
        raise CandidateValidateOnlyError("SUBJECT_AUTHORITY_ROLE_MISSING")
    if actual != expected:
        raise CandidateValidateOnlyError("AUTHORITY_INVENTORY_BINDING")
    return _AuthorityFacts(file, expected_root_authority, tuple(children))


def _validate_runtime_projection(
    session, bundle, inventory, authority, expected_runtime_identity,
) -> _RuntimeProjectionFacts:
    projection_file = session.fact(bundle.path("projection_path"))
    projection = load_runtime_projection(session.bytes(projection_file.path))
    identity = runtime_projection_identity(projection)
    if identity != expected_runtime_identity:
        raise CandidateValidateOnlyError("RUNTIME_IDENTITY_MISMATCH")
    runtime_root = bundle.path("runtime_root_path")
    return _RuntimeProjectionFacts(
        projection_file, projection, identity, runtime_root,
    )


def _validate_runtime_physical(session, inventory, runtime_projection):
    projection = runtime_projection.projection
    runtime_root = runtime_projection.runtime_root
    runtime_prefix = runtime_root + "/"
    cached = {m.path[len(runtime_prefix):]: m for m in inventory.members if m.path.startswith(runtime_prefix)}
    expected = {m.path: m for m in projection.members}
    if len(expected) != 172 or set(cached) != set(expected):
        raise CandidateValidateOnlyError("RUNTIME_MEMBER_COUNT", stage="runtime_physical")
    runtime_members = []
    for path, member in expected.items():
        fact = cached[path]
        if (fact.size, fact.sha256, fact.mode) != (member.size_bytes, member.sha256, member.mode):
            raise CandidateValidateOnlyError("RUNTIME_CACHED_MISMATCH", path, stage="runtime_physical")
        runtime_members.append(fact)
    reconciliation = reconcile_runtime_projection(projection, session.root / runtime_root, complete_inventory=True)
    if reconciliation.issues:
        raise CandidateValidateOnlyError(reconciliation.issues[0].code, stage="runtime_physical")
    with open_authenticated_runtime_snapshot(
        projection, session.root / runtime_root, complete_inventory=True,
    ) as snapshot:
        for member in projection.members:
            if snapshot.read_member_bytes(member.path) != session.bytes(runtime_prefix + member.path):
                raise CandidateValidateOnlyError(
                    "RUNTIME_APPROVED_VALIDATOR_MISMATCH", member.path,
                    stage="runtime_physical",
                )
    session.check_directory(runtime_root)
    return tuple(sorted(runtime_members, key=lambda x: x.path)), reconciliation


def _validate_runtime_dependencies(session, bundle, projection):
    dependency_file = session.fact(bundle.path("dependency_graph_path"))
    graph = session.obj(dependency_file.path)
    if set(graph) != {"entrypoints", "project_nodes", "dependencies", "declared_external_dependencies"}:
        raise CandidateValidateOnlyError("DEPENDENCY_GRAPH_SCHEMA", stage="runtime_dependency_closure")
    dependencies = tuple(RuntimeDependency(**item) for item in graph["dependencies"])
    if len(dependencies) != 174:
        raise CandidateValidateOnlyError("DEPENDENCY_EDGE_COUNT", stage="runtime_dependency_closure")
    closure = validate_runtime_dependency_closure(
        projection, entrypoints=graph["entrypoints"], project_nodes=graph["project_nodes"],
        dependencies=dependencies,
        declared_external_dependencies=graph["declared_external_dependencies"],
    )
    if closure.issues:
        raise CandidateValidateOnlyError(closure.issues[0].code, stage="runtime_dependency_closure")
    return dependency_file, graph, dependencies, closure


def _assemble_runtime_facts(
    runtime_projection, runtime_members, reconciliation,
    dependency_file, graph, dependencies, closure,
) -> _RuntimeFacts:
    return _RuntimeFacts(
        runtime_projection.projection_file, runtime_projection.projection,
        runtime_projection.identity, runtime_projection.runtime_root,
        runtime_members,
        dependency_file, dependencies, _freeze(graph), reconciliation, closure,
    )


def _validate_runtime(session, bundle, inventory, authority, expected_runtime_identity) -> _RuntimeFacts:
    """Internal aggregate helper retained for focused fact-construction tests."""

    projection = _validate_runtime_projection(
        session, bundle, inventory, authority, expected_runtime_identity,
    )
    members, reconciliation = _validate_runtime_physical(
        session, inventory, projection,
    )
    dependency_file, graph, dependencies, closure = _validate_runtime_dependencies(
        session, bundle, projection.projection,
    )
    return _assemble_runtime_facts(
        projection, members, reconciliation,
        dependency_file, graph, dependencies, closure,
    )


def _validate_plans(session, bundle, runtime, expected_runtime_identity) -> tuple[_PlanFact, ...]:
    manifest = session.obj(bundle.path("plans_manifest_path"))
    if (set(manifest) != {"plans", "allowed_external_dependencies"}
            or type(manifest["plans"]) is not dict or set(manifest["plans"]) != set(_PLAN_ROLES)
            or type(manifest["allowed_external_dependencies"]) is not list
            or any(type(item) is not str for item in manifest["allowed_external_dependencies"])
            or len(manifest["allowed_external_dependencies"]) != len(set(manifest["allowed_external_dependencies"]))):
        raise CandidateValidateOnlyError("PLAN_MANIFEST_SCHEMA")
    plan_paths = {role: _safe_rel(manifest["plans"][role], role) for role in _PLAN_ROLES}
    if (len(set(plan_paths.values())) != 6
            or set(plan_paths.values()) & {path for _, path in bundle.paths}):
        raise CandidateValidateOnlyError("PLAN_PATH_ALIAS")
    raw = {role: session.bytes(plan_paths[role]) for role in _PLAN_ROLES}
    parsed = {role: load_runtime_plan(raw[role]) for role in _PLAN_ROLES}
    with open_authenticated_runtime_snapshot(runtime.projection, session.root / runtime.runtime_root, complete_inventory=True) as snapshot:
        for role in _PLAN_ROLES:
            parsed[role] = validate_runtime_plan(
                parsed[role], runtime.projection, snapshot,
                expected_runtime_identity=expected_runtime_identity,
                allowed_external_dependencies=set(manifest["allowed_external_dependencies"]),
            )
        validate_six_plan_set(
            raw, runtime.projection, snapshot,
            expected_runtime_identity=expected_runtime_identity,
            allowed_external_dependencies=set(manifest["allowed_external_dependencies"]),
        )
    facts = []
    for role in _PLAN_ROLES:
        plan = parsed[role]
        if plan.production_runtime_identity != runtime.identity or plan.production_runtime_identity != expected_runtime_identity:
            raise CandidateValidateOnlyError("PLAN_RUNTIME_IDENTITY", role)
        facts.append(_PlanFact(role, session.fact(plan_paths[role]), plan,
                               plan.production_runtime_identity, runtime.identity,
                               expected_runtime_identity))
    return tuple(facts)


def _validate_focused(session, bundle, inventory) -> _FocusedFacts:
    file = session.fact(bundle.path("focused_results_path"))
    value = session.obj(file.path)
    if (set(value) != {"cases", "category_totals"} or not _exact_equal(value["category_totals"], _FOCUSED_CATEGORIES)
            or type(value["cases"]) is not list):
        raise CandidateValidateOnlyError("FOCUSED_SCHEMA")
    cases, package_paths, package_members = [], set(), set()
    required = {"case_id", "category", "validator_origin", "validator_symbol", "expected_code_or_result", "observed_code_or_result", "passed", "raw_package", "attachment_ids"}
    for record in value["cases"]:
        if (type(record) is not dict or set(record) != required or type(record["case_id"]) is not str
                or not re.fullmatch(r"case-(?:0(?:0[1-9]|[1-9][0-9])|1(?:[0-4][0-9]|50))", record["case_id"])
                or type(record["validator_symbol"]) is not str or not record["validator_symbol"]
                or any(ord(char) < 32 for char in record["validator_symbol"])
                or type(record["expected_code_or_result"]) not in (str, int, bool)
                or (type(record["expected_code_or_result"]) is str
                    and (not record["expected_code_or_result"]
                         or any(unicodedata.category(char) == "Cc" for char in record["expected_code_or_result"])))
                or type(record["passed"]) is not bool or record["passed"] is not True
                or not _exact_equal(record["expected_code_or_result"], record["observed_code_or_result"])):
            raise CandidateValidateOnlyError("FOCUSED_RECORD")
        category = record["category"]
        if (type(category) is not str or category not in _FOCUSED_CATEGORIES
                or type(record["validator_origin"]) is not str
                or record["validator_origin"] != _PROVENANCE[category]
                or type(record["attachment_ids"]) is not list
                or any(type(item) is not str or not item for item in record["attachment_ids"])
                or len(record["attachment_ids"]) != len(set(record["attachment_ids"]))):
            raise CandidateValidateOnlyError("FOCUSED_RECORD")
        binding = record["raw_package"]
        if (type(binding) is not dict or set(binding) != {"manifest_path", "manifest_size", "manifest_sha256", "package_identity"}
                or type(binding["manifest_size"]) is not int or binding["manifest_size"] < 0):
            raise CandidateValidateOnlyError("RAW_PACKAGE_BINDING")
        _sha_field(binding["manifest_sha256"], "raw_package.manifest_sha256")
        _sha_field(binding["package_identity"], "raw_package.package_identity")
        manifest = session.fact(binding["manifest_path"])
        required_prefix = f"focused_raw/{record['case_id']}/"
        if not manifest.path.startswith(required_prefix) or PurePosixPath(manifest.path).parent.as_posix() != required_prefix[:-1]:
            raise CandidateValidateOnlyError("RAW_PACKAGE_DIRECTORY", manifest.path)
        if manifest.path in package_paths or (manifest.size, manifest.sha256) != (binding["manifest_size"], binding["manifest_sha256"]):
            raise CandidateValidateOnlyError("RAW_PACKAGE_BINDING", manifest.path)
        package_paths.add(manifest.path)
        raw = session.obj(manifest.path)
        if (set(raw) != {"schema_version", "case_id", "members", "package_identity"}
                or raw["schema_version"] != "ctr-focused-raw-package-1"
                or raw["case_id"] != record["case_id"] or type(raw["members"]) is not list):
            raise CandidateValidateOnlyError("RAW_PACKAGE_SCHEMA", manifest.path)
        _sha_field(raw["package_identity"], "raw_package.package_identity")
        members, roles, seen = [], set(), set()
        prefix = str(PurePosixPath(manifest.path).parent) + "/"
        for member in raw["members"]:
            if (type(member) is not dict or set(member) != {"path", "role", "size", "sha256"}
                    or type(member["role"]) is not str or member["role"] in roles
                    or type(member["size"]) is not int or member["size"] < 0):
                raise CandidateValidateOnlyError("RAW_PACKAGE_MEMBER")
            _sha_field(member["sha256"], "raw_package.member.sha256")
            fact = session.fact(member["path"])
            if (fact.path == manifest.path or not fact.path.startswith(prefix) or fact.path in seen
                    or fact.path in package_members
                    or (fact.size, fact.sha256) != (member["size"], member["sha256"])):
                raise CandidateValidateOnlyError("RAW_PACKAGE_MEMBER", fact.path)
            seen.add(fact.path); package_members.add(fact.path); roles.add(member["role"])
            members.append((fact.path, member["role"], fact.size, fact.sha256))
        mandatory = {"invocation_metadata", "events", "event_manifest", "raw_capture_manifest", "raw_result"}
        allowed = mandatory | {"attachment_authorization", "attachment_allocation", "attachment_durable_receipt", "attachment_child_boundary"}
        if not mandatory.issubset(roles) or not roles.issubset(allowed) or {p for p in session.paths if p.startswith(prefix)} != seen | {manifest.path}:
            raise CandidateValidateOnlyError("RAW_PACKAGE_ROLES", manifest.path)
        projection = {"schema_version": "ctr-focused-raw-package-projection-1", "case_id": record["case_id"], "members": [dict(path=p, role=r, size=s, sha256=d) for p, r, s, d in sorted(members)]}
        identity = _canonical_digest(projection)
        if identity != raw["package_identity"] or identity != binding["package_identity"]:
            raise CandidateValidateOnlyError("RAW_PACKAGE_IDENTITY", manifest.path)
        package = _RawPackageFact(record["case_id"], manifest, identity, tuple(sorted(members)))
        cases.append(_FocusedCaseFact(record["case_id"], category, record["validator_origin"], True, tuple(record["attachment_ids"]), package))
    if len(cases) != 150 or {c.case_id for c in cases} != {f"case-{i:03d}" for i in range(1, 151)}:
        raise CandidateValidateOnlyError("FOCUSED_COUNT")
    actual_categories = {key: sum(c.category == key for c in cases) for key in _FOCUSED_CATEGORIES}
    if actual_categories != _FOCUSED_CATEGORIES:
        raise CandidateValidateOnlyError("FOCUSED_CATEGORY_TOTALS")
    origins = {key: sum(c.validator_origin == key for c in cases) for key in ("repository_production", "candidate_evidence_integrity")}
    if origins != {"repository_production": 86, "candidate_evidence_integrity": 64}:
        raise CandidateValidateOnlyError("FOCUSED_PROVENANCE_TOTALS")
    aggregate = {"schema_version": "ctr-raw-package-aggregate-1", "packages": [{"case_id": c.case_id, "package_identity": c.package.package_identity} for c in sorted(cases, key=lambda x: x.case_id)]}
    return _FocusedFacts(file, tuple(sorted(cases, key=lambda x: x.case_id)), tuple(sorted(actual_categories.items())), tuple(sorted(origins.items())), _canonical_digest(aggregate))


def _validate_attachments(session, bundle, inventory, focused) -> _AttachmentFacts:
    manifest = session.fact(bundle.path("attachment_manifest_path"))
    value = session.obj(manifest.path)
    if (set(value) != {"schema_version", "attachments"} or value["schema_version"] != "ctr-focused-attachments-1"
            or type(value["attachments"]) is not list or len(value["attachments"]) != 28):
        raise CandidateValidateOnlyError("ATTACHMENT_SCHEMA")
    expected = {x[0]: x for x in _ATTACHMENTS}; records, seen, paths = [], set(), set()
    cases = {c.case_id: c for c in focused.cases}
    attachment_member_roles = {
        "attachment_authorization", "attachment_allocation",
        "attachment_durable_receipt", "attachment_child_boundary",
    }
    for record in value["attachments"]:
        if (type(record) is not dict or set(record) != {"attachment_id", "role", "case_id", "path", "size", "sha256"}
                or any(type(record[key]) is not str for key in ("attachment_id", "role", "case_id", "path", "sha256"))
                or type(record["size"]) is not int or record["size"] <= 0):
            raise CandidateValidateOnlyError("ATTACHMENT_RECORD")
        _sha_field(record["sha256"], "attachment.sha256")
        aid = record["attachment_id"]
        if aid not in expected or aid in seen or (record["role"], record["case_id"]) != expected[aid][1:3]:
            raise CandidateValidateOnlyError("ATTACHMENT_BINDING")
        file = session.fact(record["path"])
        if file.path in paths or file.size <= 0 or (file.size, file.sha256) != (record["size"], record["sha256"]):
            raise CandidateValidateOnlyError("ATTACHMENT_MISMATCH", file.path)
        case = cases[record["case_id"]]
        if case.attachment_ids != (aid,):
            raise CandidateValidateOnlyError("ATTACHMENT_FORWARD_REFERENCE", aid)
        role = expected[aid][3]
        package_members = [m for m in case.package.members if m[1] == role]
        if package_members != [(file.path, role, file.size, file.sha256)]:
            raise CandidateValidateOnlyError("ATTACHMENT_RAW_PACKAGE", aid)
        seen.add(aid); paths.add(file.path)
        records.append(_AttachmentFact(aid, record["role"], record["case_id"], file))
    expected_by_case = {case_id: raw_role for _, _, case_id, raw_role in _ATTACHMENTS}
    for case in focused.cases:
        actual_roles = [role for _, role, _, _ in case.package.members if role in attachment_member_roles]
        required_roles = [] if case.case_id not in expected_by_case else [expected_by_case[case.case_id]]
        if actual_roles != required_roles:
            raise CandidateValidateOnlyError("ATTACHMENT_RAW_PACKAGE", case.case_id)
    if seen != set(expected) or any(c.attachment_ids for c in focused.cases if c.case_id not in expected_by_case):
        raise CandidateValidateOnlyError("ATTACHMENT_COUNT")
    totals, aggregates = {}, {}
    for role in ("authorization", "allocation", "durable_receipt", "child_boundary"):
        selected = sorted((r for r in records if r.role == role), key=lambda x: x.attachment_id)
        totals[role] = len(selected)
        projection = {"schema_version": "ctr-attachment-role-aggregate-1", "role": role, "attachments": [{"attachment_id": r.attachment_id, "case_id": r.case_id, "path": r.file.path, "size": r.file.size, "sha256": r.file.sha256} for r in selected]}
        aggregates[role] = _canonical_digest(projection)
    if totals != {"authorization": 8, "allocation": 8, "durable_receipt": 8, "child_boundary": 4}:
        raise CandidateValidateOnlyError("ATTACHMENT_ROLE_TOTALS")
    return _AttachmentFacts(manifest, tuple(sorted(records, key=lambda x: x.attachment_id)), tuple(sorted(totals.items())), tuple(sorted(aggregates.items())))


def _validate_capsule(
    session, bundle, inventory, runtime, exercised_subject,
) -> _CapsuleFact:
    file = session.fact(bundle.path("capsule_path")); value = session.obj(file.path)
    required = {
        **_CAPSULE_POLICY,
        "runtime_identity": runtime.identity,
        "exercised_subject_identity": exercised_subject.identity,
    }
    if value.get("exercised_subject_identity") == STALE_V2_DIAGNOSTIC_IDENTITY:
        raise CandidateValidateOnlyError("SUBJECT_STALE_DIAGNOSTIC_IDENTITY")
    if set(value) != set(required) or any(not _exact_equal(value[k], v) for k, v in required.items()):
        if value.get("exercised_subject_identity") != exercised_subject.identity:
            raise CandidateValidateOnlyError("CAPSULE_SUBJECT_MISMATCH")
        raise CandidateValidateOnlyError("CAPSULE_POLICY")
    return _CapsuleFact(
        file,
        value["runtime_identity"],
        value["exercised_subject_identity"],
        tuple(sorted((k, _freeze(value[k])) for k in _CAPSULE_POLICY)),
    )


def _validate_materialization_contract(session, bundle, inventory, runtime):
    projection_file = session.fact(bundle.path("materialization_projection_path"))
    inventory_paths = {member.path for member in inventory.members}
    if projection_file.path not in inventory_paths:
        raise CandidateValidateOnlyError(
            "MATERIALIZATION_PROJECTION_MISMATCH", stage="runtime_physical",
        )
    materialization_root = bundle.path("materialization_root_path")
    try:
        projection = materialization_projection_from_bytes(session.bytes(projection_file.path))
        projection_result = projection_identity_result(projection)
        verification = verify_materialization_root_at(
            session.fd, materialization_root, projection,
        )
        materialization_parts = PurePosixPath(materialization_root).parts
        runtime_parts = PurePosixPath(runtime.runtime_root).parts
        if (len(runtime_parts) <= len(materialization_parts)
                or runtime_parts[:len(materialization_parts)] != materialization_parts):
            raise CandidateValidateOnlyError(
                "MATERIALIZATION_RUNTIME_MEMBERSHIP_MISMATCH",
                runtime.runtime_root,
                stage="runtime_physical",
            )
        relative_runtime_root = PurePosixPath(
            *runtime_parts[len(materialization_parts):],
        ).as_posix()
        bindings = runtime_member_bindings(
            verification.projection_identity.projection,
            runtime.projection.members,
            materialization_runtime_root=relative_runtime_root,
        )
    except MaterializationIdentityError as error:
        code = error.code
        if code in {
            "MATERIALIZATION_INVENTORY_MISMATCH",
            "MATERIALIZATION_PROJECTION_MISMATCH",
        }:
            code = "MATERIALIZATION_PHYSICAL_PROJECTION_MISMATCH"
        raise CandidateValidateOnlyError(
            code, error.path or error.detail, stage="runtime_physical",
        ) from error
    if len(bindings) != 172:
        raise CandidateValidateOnlyError(
            "MATERIALIZATION_RUNTIME_MEMBERSHIP_MISMATCH",
            stage="runtime_physical",
        )

    projected_files = {
        member.path: member for member in projection.members
        if member.kind == "regular_file"
    }
    prefix = materialization_root.rstrip("/") + "/"
    physical_inventory = {
        member.path[len(prefix):]: member for member in inventory.members
        if member.path.startswith(prefix)
    }
    if set(projected_files) != set(physical_inventory):
        raise CandidateValidateOnlyError(
            "MATERIALIZATION_PHYSICAL_PROJECTION_MISMATCH",
            materialization_root,
            stage="runtime_physical",
        )
    for path, projected in projected_files.items():
        physical = physical_inventory[path]
        if (physical.size, physical.sha256, physical.mode) != (
            projected.size, projected.sha256, projected.mode,
        ):
            raise CandidateValidateOnlyError(
                "MATERIALIZATION_PHYSICAL_PROJECTION_MISMATCH",
                path,
                stage="runtime_physical",
            )

    source_file = session.fact(bundle.path("source_identity_path"))
    value = session.obj(source_file.path)
    fields = {
        "schema_version", "repository", "materialization",
        "historical_lineage", "exercised_subject",
    }
    if set(value) != fields or value["schema_version"] != SOURCE_IDENTITY_SCHEMA:
        raise CandidateValidateOnlyError(
            "SOURCE_IDENTITY_SCHEMA", stage="runtime_physical",
        )
    if not _exact_equal(value["repository"], _repository_identity_snapshot()):
        raise CandidateValidateOnlyError(
            "SOURCE_IDENTITY_REPOSITORY_MISMATCH", stage="runtime_physical",
        )
    lineage = value["historical_lineage"]
    expected_lineage = {
        "operative": False,
        "superseded_identities": [
            {"algorithm": algorithm, "status": "diagnostic_only", "value": digest}
            for algorithm, digest in _HISTORICAL_LINEAGE
        ],
    }
    if not _exact_equal(lineage, expected_lineage):
        raise CandidateValidateOnlyError(
            "SUPERSEDED_OPERATIVE_IDENTITY", stage="runtime_physical",
        )
    materialization = value["materialization"]
    expected_materialization = {
        "logical_algorithm_id": LOGICAL_ALGORITHM_ID,
        "logical_identity": projection_result.logical_identity,
        "materialization_authority": "candidate-contained-descriptor-authenticated-root",
        "materialization_root_path": materialization_root,
        "physical_rehash": verification.physical_rehash,
        "physical_rehash_algorithm_id": PHYSICAL_REHASH_ALGORITHM_ID,
        "projection_framing_algorithm_id": PROJECTION_FRAMING_ALGORITHM_ID,
        "projection_framing_digest": projection_result.projection_framing_digest,
        "projection_path": projection_file.path,
        "projection_schema": MATERIALIZATION_PROJECTION_SCHEMA,
        "projection_sha256": projection_result.projection_sha256,
        "projection_size": projection_result.projection_size,
        "runtime_binding_count": len(bindings),
        "runtime_projection_identity": runtime.identity,
    }
    if type(materialization) is not dict:
        raise CandidateValidateOnlyError(
            "MATERIALIZATION_SOURCE_IDENTITY_SCHEMA", stage="runtime_physical",
        )
    operative_values = {
        item for item in materialization.values() if type(item) is str
    }
    if operative_values & SUPERSEDED_HISTORICAL_IDENTITIES:
        raise CandidateValidateOnlyError(
            "SUPERSEDED_OPERATIVE_IDENTITY", stage="runtime_physical",
        )
    if set(materialization) != set(expected_materialization):
        raise CandidateValidateOnlyError(
            "MATERIALIZATION_SOURCE_IDENTITY_SCHEMA", stage="runtime_physical",
        )
    if not _exact_equal(materialization, expected_materialization):
        for field, code in (
            ("projection_sha256", "MATERIALIZATION_PROJECTION_MISMATCH"),
            ("logical_identity", "MATERIALIZATION_LOGICAL_IDENTITY_MISMATCH"),
            ("projection_framing_digest", "MATERIALIZATION_PROJECTION_MISMATCH"),
            ("physical_rehash", "MATERIALIZATION_PHYSICAL_REHASH_MISMATCH"),
            ("materialization_root_path", "MATERIALIZATION_PHYSICAL_ROOT_MISSING"),
            ("runtime_binding_count", "MATERIALIZATION_RUNTIME_MEMBERSHIP_MISMATCH"),
            ("runtime_projection_identity", "MATERIALIZATION_RUNTIME_MEMBERSHIP_MISMATCH"),
        ):
            if materialization.get(field) != expected_materialization[field]:
                raise CandidateValidateOnlyError(code, field, stage="runtime_physical")
        raise CandidateValidateOnlyError(
            "MISSING_MATERIALIZATION_ALGORITHM", stage="runtime_physical",
        )
    return _MaterializationFacts(
        projection_file, source_file, projection, materialization_root,
        projection_result.projection_sha256, projection_result.logical_identity,
        projection_result.projection_framing_digest, verification.physical_rehash,
        len(bindings), verification,
    )


def _subject_authenticated_file(file: _FileFact) -> SubjectAuthenticatedFile:
    return SubjectAuthenticatedFile(file.path, file.size, file.sha256)


def _subject_binding_value(subject: _ExercisedSubjectFacts) -> dict:
    return {
        "identity_algorithm_id": EXERCISED_SUBJECT_IDENTITY_ALGORITHM_ID,
        "logical_identity": subject.identity,
        "path": subject.file.path,
        "sha256": subject.file.sha256,
        "size": subject.file.size,
    }


def _validate_exercised_subject_contract(
    session, bundle, inventory, authority, runtime, materialization,
) -> _ExercisedSubjectFacts:
    path = bundle.path("exercised_subject_path")
    file = session.fact(path)
    inventory_by_path = {member.path: member for member in inventory.members}
    if path not in inventory_by_path:
        raise CandidateValidateOnlyError(
            "SUBJECT_RECORD_NOT_IN_INVENTORY", stage="runtime_physical",
        )
    authority_by_role = {child.role: child.file for child in authority.children}
    if authority_by_role.get("exercised_subject") != file:
        raise CandidateValidateOnlyError(
            "SUBJECT_AUTHORITY_BINDING_MISMATCH", stage="runtime_physical",
        )
    try:
        subject = parse_exercised_subject(session.bytes(path))
        identity = validate_exercised_subject(
            subject,
            candidate_bundle=_subject_authenticated_file(bundle.file),
            runtime_projection=_subject_authenticated_file(runtime.projection_file),
            runtime_identity=runtime.identity,
            materialization_projection=_subject_authenticated_file(
                materialization.projection_file,
            ),
            materialization_logical_identity=materialization.logical_identity,
        )
    except ExercisedSubjectError as error:
        detail = ":".join(item for item in (error.field, error.detail) if item)
        raise CandidateValidateOnlyError(
            error.code, detail, stage="runtime_physical",
        ) from error
    facts = _ExercisedSubjectFacts(file, subject, identity)
    source_identity = session.obj(materialization.source_identity_file.path)
    source_subject = source_identity.get("exercised_subject")
    if (type(source_subject) is dict
            and source_subject.get("logical_identity") == STALE_V2_DIAGNOSTIC_IDENTITY):
        raise CandidateValidateOnlyError(
            "SUBJECT_STALE_DIAGNOSTIC_IDENTITY", stage="runtime_physical",
        )
    if not _exact_equal(
        source_subject, _subject_binding_value(facts),
    ):
        raise CandidateValidateOnlyError(
            "SOURCE_IDENTITY_SUBJECT_MISMATCH", stage="runtime_physical",
        )
    return facts


def _validate_closure_source_projection(session, bundle, inventory):
    file = session.fact(bundle.path("closure_source_projection_path"))
    exclusions = tuple(sorted(_inventory_exclusions(bundle)))
    records = tuple(
        _closure_source_fact(
            member,
            materialization_root=bundle.path("materialization_root_path"),
            runtime_root=bundle.path("runtime_root_path"),
        )
        for member in inventory.members
    )
    expected_value = _closure_projection_value(records, exclusions)
    value = session.obj(file.path)
    required = {
        "schema_version", "inclusion_rule", "exclusions", "fact_count",
        "category_counts", "facts",
    }
    if (set(value) != required
            or value.get("schema_version") != CLOSURE_SOURCE_PROJECTION_SCHEMA
            or value.get("inclusion_rule") != "all-physically-authenticated-inventory-regular-files"
            or type(value.get("facts")) is not list
            or type(value.get("fact_count")) is not int
            or type(value.get("category_counts")) is not dict
            or type(value.get("exclusions")) is not list):
        raise CandidateValidateOnlyError("CLOSURE_SOURCE_PROJECTION_MISMATCH")
    if value["exclusions"] != list(exclusions):
        raise CandidateValidateOnlyError("CLOSURE_DEPENDENCY_CYCLE")

    actual_by_id, actual_paths, actual_keys = {}, set(), set()
    fact_fields = {
        "source_fact_id", "semantic_category", "source_path", "role", "size",
        "sha256", "expected", "observed", "derivation_rule", "assertion_kind",
    }
    for raw in value["facts"]:
        if type(raw) is not dict or set(raw) != fact_fields:
            raise CandidateValidateOnlyError("CLOSURE_SOURCE_PROJECTION_MISMATCH")
        try:
            fact = _ClosureSourceFact(
                raw["source_fact_id"], raw["semantic_category"], raw["source_path"],
                raw["role"], raw["size"], raw["sha256"],
                raw.get("expected", {}).get("mode") if type(raw.get("expected")) is dict else "",
                raw["assertion_kind"], raw["derivation_rule"],
            )
        except CandidateValidateOnlyError as error:
            if error.code == "CLOSURE_SOURCE_DERIVATION":
                raise CandidateValidateOnlyError(
                    "CLOSURE_MANIFEST_CLAIM_TAUTOLOGY", str(raw.get("source_path", "")),
                ) from error
            raise CandidateValidateOnlyError("CLOSURE_SOURCE_PROJECTION_MISMATCH", error.detail) from error
        identity = {"mode": fact.mode, "sha256": fact.sha256, "size": fact.size}
        if (not _exact_equal(raw["expected"], identity)
                or not _exact_equal(raw["observed"], identity)):
            raise CandidateValidateOnlyError("CLOSURE_PHYSICAL_SOURCE_MISMATCH", fact.source_path)
        if (fact.source_path in exclusions or fact.source_path == file.path
                or fact.source_path in actual_paths):
            code = ("CLOSURE_DEPENDENCY_CYCLE" if fact.source_path in exclusions or fact.source_path == file.path
                    else "CLOSURE_DUPLICATE_SEMANTIC_SOURCE_ASSERTION")
            raise CandidateValidateOnlyError(code, fact.source_path)
        if fact.source_fact_id in actual_by_id or fact.semantic_key in actual_keys:
            raise CandidateValidateOnlyError("CLOSURE_DUPLICATE_SEMANTIC_SOURCE_ASSERTION", fact.source_path)
        actual_by_id[fact.source_fact_id] = (fact, raw)
        actual_paths.add(fact.source_path); actual_keys.add(fact.semantic_key)

    expected_by_id = {record.source_fact_id: record for record in records}
    missing = set(expected_by_id) - set(actual_by_id)
    extra = set(actual_by_id) - set(expected_by_id)
    if missing:
        raise CandidateValidateOnlyError("CLOSURE_MISSING_SOURCE_FACT", sorted(missing)[0])
    if extra:
        raise CandidateValidateOnlyError("CLOSURE_EXTRA_SOURCE_FACT", sorted(extra)[0])
    for source_id, expected in expected_by_id.items():
        actual, raw = actual_by_id[source_id]
        if actual.source_path != expected.source_path:
            raise CandidateValidateOnlyError("CLOSURE_UNKNOWN_SOURCE_FACT", actual.source_path)
        if not _exact_equal(raw, expected.projection_record()):
            physical = (raw.get("sha256"), raw.get("size"), raw.get("expected"), raw.get("observed"))
            expected_physical = (
                expected.sha256, expected.size,
                expected.projection_record()["expected"], expected.projection_record()["observed"],
            )
            code = ("CLOSURE_PHYSICAL_SOURCE_MISMATCH" if physical != expected_physical else
                    "CLOSURE_SOURCE_PROJECTION_MISMATCH")
            raise CandidateValidateOnlyError(code, expected.source_path)
    if (value["fact_count"] != len(records)
            or not _exact_equal(value["category_counts"], expected_value["category_counts"])):
        raise CandidateValidateOnlyError("CLOSURE_SOURCE_PROJECTION_MISMATCH")
    if session.bytes(file.path) != json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
    ).encode("utf-8"):
        raise CandidateValidateOnlyError("CLOSURE_SOURCE_PROJECTION_NONCANONICAL")
    if not _exact_equal(value, expected_value):
        raise CandidateValidateOnlyError("CLOSURE_SOURCE_PROJECTION_MISMATCH")
    return _ClosureProjectionFacts(
        file, records, exclusions,
        tuple(sorted(expected_value["category_counts"].items())),
    )


def _resolve_report_inputs(session, bundle, inventory) -> _ReportInputFacts:
    paths = [bundle.path(k) for k in ("report_source_path", "static_closure_path", "correction_report_path", "source_identity_path", "predecessor_preservation_path")]
    if len(set(paths)) != 5:
        raise CandidateValidateOnlyError("REPORT_INPUT_ALIAS")
    facts = [session.fact(path) for path in paths]
    if any(f.size <= 0 for f in facts):
        raise CandidateValidateOnlyError("REPORT_INPUT_EMPTY")
    return _ReportInputFacts(*facts)


def _make_observations(items):
    result = {}
    for key, category, value, paths in items:
        if key in result or category not in _CLOSURE_CATEGORIES:
            raise CandidateValidateOnlyError("OBSERVATION_DUPLICATE", key)
        source_paths = tuple(sorted(set(_safe_rel(p) for p in paths)))
        result[key] = _Observation(key, category, _freeze(value), source_paths)
    if len(result) != 35 or {o.category for o in result.values()} != set(_CLOSURE_CATEGORIES):
        raise CandidateValidateOnlyError("OBSERVATION_SET")
    return MappingProxyType(result)


def _build_required_observations(facts):
    items = []
    def add(key, category, value, paths=()): items.append((key, category, value, paths))
    add("candidate.inventory", "candidate_inventory", {"path": facts.inventory.file.path, "member_count": len(facts.inventory.members)}, (facts.inventory.file.path,))
    add("closure_source_projection.coverage", "closure_source_projection", {
        "path": facts.closure_source.file.path,
        "size": facts.closure_source.file.size,
        "sha256": facts.closure_source.file.sha256,
        "fact_count": len(facts.closure_source.records),
        "category_count": len(facts.closure_source.category_totals),
    }, (facts.closure_source.file.path,))
    add("runtime.projection", "runtime_projection", {"path": facts.runtime.projection_file.path, "identity": facts.runtime.identity}, (facts.runtime.projection_file.path,))
    reconciliation = facts.runtime.projection_reconciliation
    add("runtime.physical", "runtime_physical", {
        "root": facts.runtime.runtime_root,
        "member_count": len(facts.runtime.members),
        "declared_count": reconciliation.declared_count,
        "physical_regular_file_count": reconciliation.physical_regular_file_count,
        "matched_count": reconciliation.matched_count,
        "issue_count": len(reconciliation.issues),
        "reconciled": not reconciliation.issues,
    }, tuple(m.path for m in facts.runtime.members))
    dependency_closure = facts.runtime.dependency_closure
    add("runtime.dependencies", "runtime_dependencies", {
        "path": facts.runtime.dependency_file.path,
        "edge_count": len(facts.runtime.dependencies),
        "reachable_member_count": len(dependency_closure.reachable_members),
        "external_dependency_count": len(dependency_closure.external_dependencies),
        "unresolved": len(dependency_closure.issues),
    }, (facts.runtime.dependency_file.path,))
    for plan in facts.plans:
        add("plan." + plan.role, "plan_" + plan.role, {"role": plan.role, "path": plan.file.path, "size": plan.file.size, "sha256": plan.file.sha256, "embedded_runtime_identity": plan.embedded_runtime_identity, "canonical_runtime_identity": plan.canonical_runtime_identity}, (plan.file.path,))
    for category, count in facts.focused.category_totals:
        add("focused." + category, "focused_" + category, count, (facts.focused.file.path,))
    add("raw_packages.summary", "raw_packages", {"count": len(facts.focused.cases), "aggregate_sha256": facts.focused.package_aggregate_sha256}, tuple(c.package.manifest.path for c in facts.focused.cases))
    category_for_role = {"authorization": "authorization_attachments", "allocation": "allocation_attachments", "durable_receipt": "durable_receipts", "child_boundary": "child_boundary"}
    for role, count in facts.attachments.role_totals:
        add("attachments." + ("durable_receipts" if role == "durable_receipt" else role), category_for_role[role], {"count": count, "aggregate_sha256": dict(facts.attachments.role_aggregates)[role]}, tuple(r.file.path for r in facts.attachments.records if r.role == role))
    add("materialization.projection", "materialization_projection", {
        "path": facts.materialization.projection_file.path,
        "size": facts.materialization.projection_file.size,
        "sha256": facts.materialization.projection_sha256,
        "logical_identity": facts.materialization.logical_identity,
        "materialization_root_path": facts.materialization.materialization_root,
        "projection_framing_digest": facts.materialization.projection_framing_digest,
        "physical_rehash": facts.materialization.physical_rehash,
        "physically_observed_member_count": len(
            facts.materialization.verification.observed_members
        ),
    }, (
        facts.materialization.projection_file.path,
        *(
            facts.materialization.materialization_root.rstrip("/") + "/" + member.path
            for member in facts.materialization.projection.members
            if member.kind == "regular_file"
        ),
    ))
    add("exercised_subject.identity", "exercised_subject", {
        "identity_algorithm_id": EXERCISED_SUBJECT_IDENTITY_ALGORITHM_ID,
        "logical_identity": facts.exercised_subject.identity,
        "path": facts.exercised_subject.file.path,
        "size": facts.exercised_subject.file.size,
        "sha256": facts.exercised_subject.file.sha256,
        "candidate_bundle": facts.exercised_subject.subject.candidate_bundle.as_dict(),
        "runtime_projection": facts.exercised_subject.subject.runtime_projection.as_dict(),
        "runtime_identity": facts.exercised_subject.subject.runtime_identity,
        "materialization_projection": (
            facts.exercised_subject.subject.materialization_projection.as_dict()
        ),
        "materialization_logical_identity": (
            facts.exercised_subject.subject.materialization_logical_identity
        ),
    }, (facts.exercised_subject.file.path,))
    add("correction_report.bytes", "correction_report", {"path": facts.report_inputs.correction_report.path, "size": facts.report_inputs.correction_report.size, "sha256": facts.report_inputs.correction_report.sha256}, (facts.report_inputs.correction_report.path,))
    add("capsule.policy", "capsule_policy", {"path": facts.capsule.file.path, "size": facts.capsule.file.size, "sha256": facts.capsule.file.sha256, "policy": dict(facts.capsule.policy), "exercised_subject_identity": facts.capsule.exercised_subject_identity}, (facts.capsule.file.path,))
    add("candidate.basename", "candidate_path", facts.invocation.candidate_basename)
    add("source.identity", "source_identity", {"path": facts.report_inputs.source_identity.path, "size": facts.report_inputs.source_identity.size, "sha256": facts.report_inputs.source_identity.sha256, "authentication_scope": "root_authority_inventory"}, (facts.report_inputs.source_identity.path,))
    add("predecessor.preservation", "predecessor_preservation", {"path": facts.report_inputs.predecessor_preservation.path, "size": facts.report_inputs.predecessor_preservation.size, "sha256": facts.report_inputs.predecessor_preservation.sha256, "authentication_scope": "root_authority_inventory", "external_predecessors_inspected": False}, (facts.report_inputs.predecessor_preservation.path,))
    add("side_effect.boundary", "side_effect_boundary", dict(facts.invocation.side_effects))
    add("materialization.runtime_binding", "materialization_runtime_binding", {
        "runtime_identity": facts.runtime.identity,
        "bound_members": facts.materialization.runtime_binding_count,
    }, (facts.materialization.projection_file.path, facts.runtime.projection_file.path))
    return _make_observations(items)


def _validate_report_source(session, facts, observations, closure) -> _ReportFacts:
    file = facts.report_inputs.report_source; value = session.obj(file.path)
    fields = {
        "schema_version", "runtime", "plans", "focused", "attachments",
        "closure_source_projection", "static_closure", "materialization",
        "exercised_subject", "correction_report", "capsule",
        "candidate_basename",
    }
    if set(value) != fields or value["schema_version"] != REPORT_SOURCE_SCHEMA:
        raise CandidateValidateOnlyError("REPORT_SOURCE_SCHEMA")
    runtime = value["runtime"]
    if (type(runtime) is not dict or set(runtime) != {"identity", "member_count", "dependency_edge_count"}
            or not _exact_equal(runtime, {"identity": facts.runtime.identity, "member_count": 172, "dependency_edge_count": 174})):
        raise CandidateValidateOnlyError("REPORT_RUNTIME")
    if set(value["plans"]) != set(_PLAN_ROLES):
        raise CandidateValidateOnlyError("REPORT_PLANS")
    for plan in facts.plans:
        expected = {"path": plan.file.path, "size": plan.file.size, "sha256": plan.file.sha256}
        if not _exact_equal(value["plans"][plan.role], expected):
            raise CandidateValidateOnlyError("REPORT_PLAN", plan.role)
    focused = value["focused"]
    if (type(focused) is not dict or set(focused) != {"case_count", "raw_package_count", "categories", "validator_origins"}
            or not _exact_equal(focused, {"case_count": 150, "raw_package_count": 150,
                                         "categories": dict(facts.focused.category_totals),
                                         "validator_origins": dict(facts.focused.provenance_totals)})):
        raise CandidateValidateOnlyError("REPORT_FOCUSED")
    attachments = value["attachments"]
    if not _exact_equal(attachments, {"attachment_count": 28, "roles": dict(facts.attachments.role_totals)}):
        raise CandidateValidateOnlyError("REPORT_ATTACHMENTS")
    source_projection = facts.closure_source
    expected_projection = {
        "category_count": len(source_projection.category_totals),
        "fact_count": len(source_projection.records),
        "path": source_projection.file.path,
        "sha256": source_projection.file.sha256,
        "size": source_projection.file.size,
    }
    if not _exact_equal(value["closure_source_projection"], expected_projection):
        raise CandidateValidateOnlyError("CLOSURE_SOURCE_PROJECTION_MISMATCH")
    expected_closure = {
        "authenticated_source_count": closure.source_records,
        "category_count": closure.category_count,
        "check_count": closure.critical_records + closure.source_records,
        "critical_count": closure.critical_records,
        "failed_checks": 0,
    }
    if not _exact_equal(value["static_closure"], expected_closure):
        raise CandidateValidateOnlyError("CLOSURE_REPORT_COUNT_MISMATCH")
    materialization = facts.materialization
    expected_materialization = {
        "logical_identity": materialization.logical_identity,
        "materialization_root_path": materialization.materialization_root,
        "physical_rehash": materialization.physical_rehash,
        "projection_framing_digest": materialization.projection_framing_digest,
        "projection_path": materialization.projection_file.path,
        "projection_schema": MATERIALIZATION_PROJECTION_SCHEMA,
        "projection_sha256": materialization.projection_sha256,
        "projection_size": materialization.projection_file.size,
        "runtime_binding_count": materialization.runtime_binding_count,
    }
    if not _exact_equal(value["materialization"], expected_materialization):
        raise CandidateValidateOnlyError("MATERIALIZATION_PROJECTION_MISMATCH")
    report_subject = value["exercised_subject"]
    if (type(report_subject) is dict
            and report_subject.get("logical_identity") == STALE_V2_DIAGNOSTIC_IDENTITY):
        raise CandidateValidateOnlyError("SUBJECT_STALE_DIAGNOSTIC_IDENTITY")
    if not _exact_equal(
        report_subject,
        _subject_binding_value(facts.exercised_subject),
    ):
        raise CandidateValidateOnlyError("REPORT_SOURCE_SUBJECT_MISMATCH")
    correction = facts.report_inputs.correction_report
    if not _exact_equal(value["correction_report"], {"path": correction.path, "size": correction.size, "sha256": correction.sha256}):
        raise CandidateValidateOnlyError("REPORT_CORRECTION")
    capsule = facts.capsule.file
    if not _exact_equal(value["capsule"], {"path": capsule.path, "size": capsule.size, "sha256": capsule.sha256, "policy": dict(facts.capsule.policy), "exercised_subject_identity": facts.exercised_subject.identity}):
        raise CandidateValidateOnlyError("REPORT_CAPSULE")
    if value["candidate_basename"] != facts.invocation.candidate_basename:
        raise CandidateValidateOnlyError("REPORT_CANDIDATE")
    return _ReportFacts(file)


def _expected_static_closure_value(observations, closure_source):
    critical = []
    for index, key in enumerate(sorted(observations), 1):
        observation = observations[key]
        value = _thaw(observation.value)
        critical.append({
            "category": observation.category,
            "check_id": f"critical-{index:03d}",
            "expected": value,
            "kind": "critical_observation",
            "observation_key": key,
            "observed": value,
            "passed": True,
            "source_paths": list(observation.source_paths),
        })
    source = []
    for fact in sorted(closure_source.records, key=lambda item: item.source_fact_id):
        source.append({
            **fact.projection_record(),
            "check_id": "source:" + fact.source_fact_id,
            "kind": "authenticated_source",
            "passed": True,
        })
    categories = {
        check["category"] if check["kind"] == "critical_observation"
        else check["semantic_category"]
        for check in critical + source
    }
    return {
        "authenticated_source_count": len(source),
        "category_count": len(categories),
        "check_count": len(critical) + len(source),
        "checks": critical + source,
        "critical_count": len(critical),
        "failed_checks": 0,
        "schema_version": STATIC_CLOSURE_SCHEMA,
    }


def _validate_static_closure(session, facts, observations) -> _ClosureFacts:
    file = facts.report_inputs.static_closure; value = session.obj(file.path)
    required = {
        "schema_version", "critical_count", "authenticated_source_count",
        "check_count", "category_count", "failed_checks", "checks",
    }
    if value.get("schema_version") == "ctr-static-closure-1":
        raise CandidateValidateOnlyError("LEGACY_DUPLICATED_CLOSURE")
    if (set(value) != required or value.get("schema_version") != STATIC_CLOSURE_SCHEMA
            or type(value.get("critical_count")) is not int
            or type(value.get("authenticated_source_count")) is not int
            or type(value.get("check_count")) is not int
            or type(value.get("category_count")) is not int
            or type(value.get("failed_checks")) is not int
            or type(value.get("checks")) is not list):
        raise CandidateValidateOnlyError("STATIC_CLOSURE_SCHEMA")
    expected_closure = _expected_static_closure_value(observations, facts.closure_source)
    ids, critical, source_ids, semantic_keys, source_paths, categories = set(), set(), set(), set(), set(), set()
    projected = {record.source_fact_id: record for record in facts.closure_source.records}
    for check in value["checks"]:
        if (type(check) is not dict or type(check.get("passed")) is not bool or check["passed"] is not True
                or type(check.get("check_id")) is not str or not check["check_id"]
                or any(ord(char) < 32 for char in check["check_id"]) or check["check_id"] in ids):
            raise CandidateValidateOnlyError("STATIC_CLOSURE_RECORD")
        ids.add(check["check_id"])
        if check.get("kind") == "critical_observation":
            fields = {"check_id", "kind", "category", "observation_key", "expected", "observed", "source_paths", "passed"}
            if (set(check) != fields or check["observation_key"] not in observations
                    or check["observation_key"] in critical
                    or check["category"] not in _CLOSURE_CATEGORIES):
                raise CandidateValidateOnlyError("STATIC_CLOSURE_CRITICAL")
            obs = observations[check["observation_key"]]; expected_observation = _thaw(obs.value)
            if (check["observation_key"] == "exercised_subject.identity"
                    and type(check.get("expected")) is dict
                    and check["expected"].get("logical_identity")
                    == STALE_V2_DIAGNOSTIC_IDENTITY):
                raise CandidateValidateOnlyError("SUBJECT_STALE_DIAGNOSTIC_IDENTITY")
            if (check["category"] != obs.category or not _exact_equal(check["expected"], expected_observation)
                    or not _exact_equal(check["observed"], expected_observation)
                    or not _exact_equal(check["source_paths"], list(obs.source_paths))):
                if check["observation_key"] == "exercised_subject.identity":
                    raise CandidateValidateOnlyError("CLOSURE_SUBJECT_MISMATCH")
                raise CandidateValidateOnlyError("STATIC_CLOSURE_CRITICAL")
            critical.add(check["observation_key"]); categories.add(check["category"])
        elif check.get("kind") == "authenticated_source":
            fields = {
                "check_id", "kind", "source_fact_id", "semantic_category",
                "source_path", "role", "size", "sha256", "expected", "observed",
                "derivation_rule", "assertion_kind", "passed",
            }
            if set(check) != fields:
                raise CandidateValidateOnlyError("CLOSURE_SOURCE_PROJECTION_MISMATCH")
            if type(check.get("semantic_category")) is not str:
                raise CandidateValidateOnlyError("CLOSURE_SOURCE_PROJECTION_MISMATCH")
            categories.add(check["semantic_category"])
            source_id = check.get("source_fact_id")
            path = check.get("source_path")
            if path in facts.closure_source.exclusions or path == file.path:
                raise CandidateValidateOnlyError("CLOSURE_DEPENDENCY_CYCLE", str(path))
            if source_id in source_ids or path in source_paths:
                raise CandidateValidateOnlyError("CLOSURE_DUPLICATE_SEMANTIC_SOURCE_ASSERTION", str(path))
            source_ids.add(source_id); source_paths.add(path)
            fact = projected.get(source_id)
            if fact is None:
                raise CandidateValidateOnlyError("CLOSURE_EXTRA_SOURCE_FACT", str(source_id))
            if check.get("source_path") != fact.source_path:
                raise CandidateValidateOnlyError("CLOSURE_UNKNOWN_SOURCE_FACT", str(check.get("source_path")))
            key = fact.semantic_key
            if key in semantic_keys:
                raise CandidateValidateOnlyError("CLOSURE_DUPLICATE_SEMANTIC_SOURCE_ASSERTION", fact.source_path)
            semantic_keys.add(key)
            expected_check = {
                **fact.projection_record(),
                "check_id": "source:" + fact.source_fact_id,
                "kind": "authenticated_source",
                "passed": True,
            }
            if check.get("derivation_rule") != fact.derivation_rule:
                raise CandidateValidateOnlyError("CLOSURE_MANIFEST_CLAIM_TAUTOLOGY", fact.source_path)
            if not _exact_equal(check, expected_check):
                physical_fields = ("size", "sha256", "expected", "observed")
                if any(not _exact_equal(check.get(field), expected_check[field]) for field in physical_fields):
                    raise CandidateValidateOnlyError("CLOSURE_PHYSICAL_SOURCE_MISMATCH", fact.source_path)
                raise CandidateValidateOnlyError("CLOSURE_SOURCE_PROJECTION_MISMATCH", fact.source_path)
        else:
            raise CandidateValidateOnlyError("STATIC_CLOSURE_KIND")
    missing = set(projected) - source_ids
    extra = source_ids - set(projected)
    if missing:
        raise CandidateValidateOnlyError("CLOSURE_MISSING_SOURCE_FACT", sorted(missing)[0])
    if extra:
        raise CandidateValidateOnlyError("CLOSURE_EXTRA_SOURCE_FACT", sorted(extra)[0])
    if critical != set(observations):
        raise CandidateValidateOnlyError("STATIC_CLOSURE_CRITICAL")
    expected_categories = {
        observation.category for observation in observations.values()
    } | {record.semantic_category for record in facts.closure_source.records}
    if categories != expected_categories:
        raise CandidateValidateOnlyError("CLOSURE_SOURCE_PROJECTION_MISMATCH")
    for field in ("critical_count", "authenticated_source_count", "check_count", "category_count", "failed_checks"):
        if value[field] != expected_closure[field]:
            raise CandidateValidateOnlyError("CLOSURE_REPORT_COUNT_MISMATCH", field)
    if not _exact_equal(value["checks"], expected_closure["checks"]):
        raise CandidateValidateOnlyError("CLOSURE_SOURCE_PROJECTION_MISMATCH")
    return _ClosureFacts(file, len(critical), len(source_ids), len(categories))


def _failure_result(candidate_root, root_authority, runtime_identity, traces, stage, error):
    if stage not in TRACE_NAMES:
        stage = TRACE_NAMES[0]
    prefix_count = _FAILURE_PASS_PREFIX_COUNTS[stage]
    prefix = []
    for expected_name, trace in zip(TRACE_NAMES[:prefix_count], tuple(traces)):
        if (type(trace) is not ValidationTrace or trace.status != "PASS"
                or trace.name != expected_name):
            break
        prefix.append(trace)
    complete = tuple(prefix) + (ValidationTrace(stage, "FAIL", error.code, error.detail),)
    counts = tuple(_RESULT_COUNTS_FAIL.items())
    def display(value):
        if type(value) in (str, type(Path("."))):
            return str(value)
        kind = type(value)
        return f"<invalid:{kind.__module__}.{kind.__qualname__}>"
    return CandidateValidationResult(
        RESULT_SCHEMA, "FAIL", display(candidate_root), display(root_authority),
        display(runtime_identity), complete, counts, tuple(_SIDE_EFFECTS.items()),
    )


def validate_frozen_candidate(candidate_root, *, expected_root_authority, expected_runtime_identity, bundle_path="manifests/validate_only_bundle.json"):
    try:
        _sha_field(expected_root_authority, "expected_root_authority")
        _sha_field(expected_runtime_identity, "expected_runtime_identity")
        bundle_path = _safe_rel(bundle_path, "bundle_path")
    except CandidateValidateOnlyError as error:
        return _failure_result(candidate_root, expected_root_authority, expected_runtime_identity, (), TRACE_NAMES[0], error)
    traces = []
    try:
        session_cm = _CandidateReadSession(candidate_root)
    except CandidateValidateOnlyError as error:
        return _failure_result(candidate_root, expected_root_authority, expected_runtime_identity, traces, TRACE_NAMES[0], error)
    with session_cm as session:
        active_stage = "candidate_inventory"
        try:
            bundle = _load_bundle(session, bundle_path)
            inventory = _validate_inventory(session, bundle)
            traces.append(ValidationTrace("candidate_inventory", "PASS", "OK"))
            active_stage = "root_authority"
            authority = _validate_authority(session, bundle, inventory, expected_root_authority)
            traces.append(ValidationTrace("root_authority", "PASS", "OK"))
            active_stage = "runtime_projection"
            runtime_projection = _validate_runtime_projection(
                session, bundle, inventory, authority, expected_runtime_identity,
            )
            traces.append(ValidationTrace("runtime_projection", "PASS", "OK"))
            active_stage = "runtime_physical"
            runtime_members, reconciliation = _validate_runtime_physical(
                session, inventory, runtime_projection,
            )
            materialization = _validate_materialization_contract(
                session, bundle, inventory, runtime_projection,
            )
            exercised_subject = _validate_exercised_subject_contract(
                session, bundle, inventory, authority, runtime_projection,
                materialization,
            )
            traces.append(ValidationTrace("runtime_physical", "PASS", "OK"))
            active_stage = "runtime_dependency_closure"
            dependency_file, graph, dependencies, dependency_closure = (
                _validate_runtime_dependencies(
                    session, bundle, runtime_projection.projection,
                )
            )
            runtime = _assemble_runtime_facts(
                runtime_projection, runtime_members, reconciliation,
                dependency_file, graph, dependencies, dependency_closure,
            )
            traces.append(ValidationTrace("runtime_dependency_closure", "PASS", "OK"))
            active_stage = "six_plan_set"
            plans = _validate_plans(session, bundle, runtime, expected_runtime_identity)
            traces.append(ValidationTrace("six_plan_set", "PASS", "OK"))
            active_stage = "focused_evidence"
            focused = _validate_focused(session, bundle, inventory)
            traces.append(ValidationTrace("focused_evidence", "PASS", "OK"))
            active_stage = "attachments"
            attachments = _validate_attachments(session, bundle, inventory, focused)
            traces.append(ValidationTrace("attachments", "PASS", "OK"))
            active_stage = "capsule_policy"
            capsule = _validate_capsule(
                session, bundle, inventory, runtime, exercised_subject,
            )
            report_inputs = _resolve_report_inputs(session, bundle, inventory)
            active_stage = "report_and_static_closure"
            closure_source = _validate_closure_source_projection(
                session, bundle, inventory,
            )
            invocation = _InvocationFacts(session.root.name, expected_root_authority, expected_runtime_identity, tuple(_SIDE_EFFECTS.items()))
            facts = _CandidateFacts(
                bundle, inventory, authority, runtime, plans, focused,
                attachments, capsule, report_inputs, closure_source,
                materialization, exercised_subject, invocation,
            )
            observations = _build_required_observations(facts)
            closure = _validate_static_closure(session, facts, observations)
            _validate_report_source(session, facts, observations, closure)
            traces.append(ValidationTrace("report_and_static_closure", "PASS", "OK"))
            traces.append(ValidationTrace("capsule_policy", "PASS", "OK"))
            active_stage = "candidate_inventory"
            session.verify_current_tree(runtime.runtime_root)
        except CandidateValidateOnlyError as error:
            stage = error.stage or active_stage
            return _failure_result(candidate_root, expected_root_authority, expected_runtime_identity, traces, stage, error)
        except RuntimeValidationError as error:
            wrapped = CandidateValidateOnlyError(error.code, f"{error.field or ''}:{error.path or ''}", stage=active_stage)
            stage = active_stage
            return _failure_result(candidate_root, expected_root_authority, expected_runtime_identity, traces, stage, wrapped)
        except (OSError, TypeError, ValueError, KeyError, AttributeError, UnicodeError) as error:
            wrapped = CandidateValidateOnlyError("CANDIDATE_INPUT_INVALID", active_stage, stage=active_stage)
            return _failure_result(candidate_root, expected_root_authority, expected_runtime_identity, traces, active_stage, wrapped)
        counts = tuple(_RESULT_COUNTS_PASS.items())
        return CandidateValidationResult(RESULT_SCHEMA, "PASS", str(Path(candidate_root)), expected_root_authority, expected_runtime_identity, tuple(traces), counts, tuple(_SIDE_EFFECTS.items()))


def _result_dict(result):
    return {"schema": result.schema, "overall": result.overall, "candidate_root": result.candidate_root, "root_authority": result.root_authority, "runtime_identity": result.runtime_identity, "traces": [dict(name=t.name, status=t.status, code=t.code, detail=t.detail) for t in result.traces], "counts": dict(result.counts), "side_effects": dict(result.side_effects)}


def main(argv=None):
    parser = argparse.ArgumentParser(prog="ctr-runtime-candidate-validate-only")
    parser.add_argument("--candidate-root", required=True)
    parser.add_argument("--expected-root-authority", required=True)
    parser.add_argument("--expected-runtime-identity", required=True)
    parser.add_argument("--bundle-path", default="manifests/validate_only_bundle.json")
    try:
        args = parser.parse_args(argv)
        result = validate_frozen_candidate(args.candidate_root, expected_root_authority=args.expected_root_authority, expected_runtime_identity=args.expected_runtime_identity, bundle_path=args.bundle_path)
        print(json.dumps(_result_dict(result), sort_keys=True, separators=(",", ":"), ensure_ascii=False))
        return 0 if result.overall == "PASS" else 1
    except SystemExit:
        raise
    except CandidateValidateOnlyError as error:
        print(json.dumps({"schema": RESULT_SCHEMA, "overall": "FAIL", "code": error.code, "detail": error.detail}, sort_keys=True, separators=(",", ":")))
        return 1


__all__ = ["CandidateValidateOnlyError", "ValidationTrace", "CandidateValidationResult", "validate_frozen_candidate", "main"]
