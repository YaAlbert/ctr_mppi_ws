"""Disconnected immutable publication inventory and state records."""
# flake8: noqa: E501

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path, PurePosixPath
import math
import re
from typing import Any, Iterable


class _StringEnum(str, Enum):
    def __str__(self) -> str:
        return self.value


class ArtifactRepresentation(_StringEnum):
    SELF_DESCRIBING = "self-describing"
    OPAQUE = "opaque"


class Applicability(_StringEnum):
    APPLICABLE = "APPLICABLE"
    NOT_APPLICABLE = "NOT_APPLICABLE"


class RenderStatus(_StringEnum):
    NOT_STARTED = "NOT_STARTED"
    RENDERED = "RENDERED"
    RENDER_FAILED = "RENDER_FAILED"
    DEPENDENCY_FAILED = "DEPENDENCY_FAILED"
    NOT_APPLICABLE = "NOT_APPLICABLE"


class StagingStatus(_StringEnum):
    NOT_STARTED = "NOT_STARTED"
    STAGED = "STAGED"
    STAGE_FAILED = "STAGE_FAILED"
    DEPENDENCY_FAILED = "DEPENDENCY_FAILED"
    NOT_APPLICABLE = "NOT_APPLICABLE"


class VisibilityStatus(_StringEnum):
    NOT_OBSERVED = "NOT_OBSERVED"
    VISIBLE = "VISIBLE"
    MISSING = "MISSING"
    INVALID_FILE_TYPE = "INVALID_FILE_TYPE"
    UNREADABLE = "UNREADABLE"
    NOT_APPLICABLE = "NOT_APPLICABLE"


class ContentVerificationStatus(_StringEnum):
    NOT_ATTEMPTED = "NOT_ATTEMPTED"
    VERIFIED = "VERIFIED"
    FAILED = "FAILED"
    NOT_APPLICABLE = "NOT_APPLICABLE"


class PublicationStatus(_StringEnum):
    PUBLISHED = "PUBLISHED"
    RENDER_FAILED = "RENDER_FAILED"
    STAGE_FAILED = "STAGE_FAILED"
    DEPENDENCY_FAILED = "DEPENDENCY_FAILED"
    VERIFICATION_FAILED = "VERIFICATION_FAILED"
    NOT_APPLICABLE = "NOT_APPLICABLE"


class FailureStage(_StringEnum):
    RENDER = "RENDER"
    DEPENDENCY = "DEPENDENCY"
    STAGING_WRITE = "STAGING_WRITE"
    FINAL_PATH_OBSERVATION = "FINAL_PATH_OBSERVATION"
    DIGEST_VERIFICATION = "DIGEST_VERIFICATION"
    SNAPSHOT_VERIFICATION = "SNAPSHOT_VERIFICATION"


class SnapshotBindingStatus(_StringEnum):
    NOT_ATTEMPTED = "NOT_ATTEMPTED"
    VERIFIED = "VERIFIED"
    FAILED = "FAILED"
    NOT_APPLICABLE = "NOT_APPLICABLE"


class RecordPhase(_StringEnum):
    PRE_PUBLICATION = "PRE_PUBLICATION"
    PRE_PROMOTION = "PRE_PROMOTION"
    FINAL = "FINAL"


class ResultAuthority(_StringEnum):
    LEGACY_UNVERIFIED = "LEGACY_UNVERIFIED"
    RECEIPT_PRESENT_UNVALIDATED = "RECEIPT_PRESENT_UNVALIDATED"
    UNCOMMITTED = "UNCOMMITTED"


RECEIPT_LOGICAL_NAME = "publication_receipt"
RECEIPT_RELATIVE_PATH = "publication_receipt.json"
_DIGEST_RE = re.compile(r"^[0-9a-f]{64}$")
_TERMINAL_FAILURES = frozenset({PublicationStatus.RENDER_FAILED, PublicationStatus.STAGE_FAILED, PublicationStatus.DEPENDENCY_FAILED, PublicationStatus.VERIFICATION_FAILED})
_STATE_FIELDS = frozenset({"record_phase", "render_status", "staging_status", "visibility_status", "content_verification_status", "publication_status", "failure_stage", "failure_reason", "staged_sha256", "final_sha256", "snapshot_id_verified", "snapshot_binding_status"})


def _freeze_value(value: Any, path: str = "value") -> Any:
    if isinstance(value, Enum):
        return _freeze_value(value.value, path)
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"{path} must contain finite floats")
        return value
    if isinstance(value, dict):
        pairs = [(_freeze_value(k, f"{path}.key"), _freeze_value(v, f"{path}[{k!r}]")) for k, v in value.items()]
        return tuple(sorted(pairs, key=repr))
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_value(item, f"{path}[{i}]") for i, item in enumerate(value))
    if isinstance(value, (set, frozenset)):
        return tuple(sorted((_freeze_value(item, path) for item in value), key=repr))
    raise TypeError(f"{path} contains a mutable or unsupported object")


@dataclass(frozen=True)
class LayerASnapshot:
    snapshot_id: str
    operational_reason: str | None
    workflow_classification: str
    workflow_exit_code: int
    comparison_valid: bool | None
    timing_descriptive: bool = True
    timeout_status: str | None = None
    cancellation_evidence: Any = ()
    delivery_classification: str | None = None
    compatibility_valid: bool | None = None
    timing_data: Any = ()

    def __post_init__(self) -> None:
        if not isinstance(self.snapshot_id, str) or not self.snapshot_id.strip():
            raise ValueError("snapshot_id must be non-empty")
        if not isinstance(self.workflow_classification, str) or not self.workflow_classification:
            raise ValueError("workflow_classification must be non-empty")
        if not isinstance(self.workflow_exit_code, int) or isinstance(self.workflow_exit_code, bool):
            raise ValueError("workflow_exit_code must be an integer")
        if not self.timing_descriptive:
            raise ValueError("timing fields must remain descriptive")
        object.__setattr__(self, "cancellation_evidence", _freeze_value(self.cancellation_evidence, "cancellation_evidence"))
        object.__setattr__(self, "timing_data", _freeze_value(self.timing_data, "timing_data"))


@dataclass(frozen=True)
class ArtifactSpec:
    logical_name: str
    relative_path: str
    required: bool
    applicability: Applicability
    representation: ArtifactRepresentation
    expected_file_type: str
    dependencies: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        _validate_relative_path(self.relative_path)
        if not isinstance(self.logical_name, str) or not self.logical_name:
            raise ValueError("logical_name must be non-empty")
        if type(self.required) is not bool:
            raise TypeError("required must be a bool")
        if not isinstance(self.applicability, Applicability):
            raise TypeError("applicability must be an Applicability")
        if not isinstance(self.representation, ArtifactRepresentation):
            raise TypeError("representation must be an ArtifactRepresentation")
        if not isinstance(self.expected_file_type, str) or not self.expected_file_type:
            raise ValueError("expected_file_type must be non-empty")
        if self.logical_name == RECEIPT_LOGICAL_NAME or self.relative_path == RECEIPT_RELATIVE_PATH:
            raise ValueError("receipt identity is reserved for CommitArtifactSpec")
        if self.applicability is Applicability.NOT_APPLICABLE and self.required:
            raise ValueError(f"required artifact cannot be NOT_APPLICABLE: {self.logical_name}")
        if isinstance(self.dependencies, (str, set, frozenset, dict)):
            raise TypeError("dependencies must be an ordered iterable of logical names")
        dependencies = tuple(self.dependencies)
        if any(not isinstance(item, str) or not item.strip() for item in dependencies):
            raise ValueError(f"dependencies must contain nonempty names: {self.logical_name}")
        if len(dependencies) != len(set(dependencies)):
            raise ValueError(f"duplicate artifact dependencies: {self.logical_name}")
        if self.logical_name in dependencies:
            raise ValueError(f"artifact cannot depend on itself: {self.logical_name}")
        object.__setattr__(self, "dependencies", dependencies)


@dataclass(frozen=True)
class CommitArtifactSpec:
    logical_name: str = RECEIPT_LOGICAL_NAME
    relative_path: str = RECEIPT_RELATIVE_PATH
    expected_file_type: str = "regular_file"

    def __post_init__(self) -> None:
        if self.logical_name != RECEIPT_LOGICAL_NAME or self.relative_path != RECEIPT_RELATIVE_PATH:
            raise ValueError("CommitArtifactSpec has a fixed reserved identity")
        _validate_relative_path(self.relative_path, allow_receipt=True)


@dataclass(frozen=True, init=False)
class ArtifactRecord:
    _spec: ArtifactSpec
    _layer_a: LayerASnapshot
    record_phase: RecordPhase
    render_status: RenderStatus
    staging_status: StagingStatus
    visibility_status: VisibilityStatus
    content_verification_status: ContentVerificationStatus
    publication_status: PublicationStatus | None
    failure_stage: FailureStage | None
    failure_reason: str | None
    staged_sha256: str | None
    final_sha256: str | None
    snapshot_id_verified: bool | None
    snapshot_binding_status: SnapshotBindingStatus
    run_applicability: Applicability

    def __init__(self, spec: ArtifactSpec, layer_a: LayerASnapshot, record_phase: RecordPhase, render_status: RenderStatus, staging_status: StagingStatus, visibility_status: VisibilityStatus, content_verification_status: ContentVerificationStatus, publication_status: PublicationStatus | None, failure_stage: FailureStage | None, failure_reason: str | None, staged_sha256: str | None, final_sha256: str | None, snapshot_id_verified: bool | None, snapshot_binding_status: SnapshotBindingStatus, run_applicability: Applicability | None = None) -> None:
        if not isinstance(spec, ArtifactSpec):
            raise TypeError("spec must be an ArtifactSpec")
        if not isinstance(layer_a, LayerASnapshot):
            raise TypeError("layer_a must be a LayerASnapshot")
        object.__setattr__(self, "_spec", spec)
        object.__setattr__(self, "_layer_a", layer_a)
        if run_applicability is None:
            run_applicability = spec.applicability
        object.__setattr__(self, "run_applicability", run_applicability)
        for name, value in locals().items():
            if name not in {"self", "spec", "layer_a"}:
                object.__setattr__(self, name, value)
        self._validate()

    @property
    def spec(self) -> ArtifactSpec:
        return self._spec

    @property
    def layer_a(self) -> LayerASnapshot:
        return self._layer_a

    @property
    def logical_name(self) -> str:
        return self._spec.logical_name

    @property
    def relative_path(self) -> str:
        return self._spec.relative_path

    @property
    def required(self) -> bool:
        return self._spec.required

    @property
    def applicability(self) -> Applicability:
        return self._spec.applicability

    @property
    def execution_applicability(self) -> Applicability:
        return self.run_applicability

    @property
    def representation(self) -> ArtifactRepresentation:
        return self._spec.representation

    @property
    def expected_file_type(self) -> str:
        return self._spec.expected_file_type

    @property
    def dependencies(self) -> tuple[str, ...]:
        return self._spec.dependencies

    @property
    def snapshot_id(self) -> str:
        return self._layer_a.snapshot_id

    def _validate(self) -> None:
        if self.logical_name == RECEIPT_LOGICAL_NAME or self.relative_path == RECEIPT_RELATIVE_PATH:
            raise ValueError("receipt identity is reserved for CommitArtifactSpec")
        _validate_relative_path(self.relative_path)
        _validate_digest(self.staged_sha256, "staged_sha256")
        _validate_digest(self.final_sha256, "final_sha256")
        if not isinstance(self.run_applicability, Applicability):
            raise TypeError("run_applicability must be an Applicability")
        if self.run_applicability is Applicability.NOT_APPLICABLE:
            self._validate_not_applicable()
        elif self.record_phase is RecordPhase.PRE_PUBLICATION:
            self._validate_prepublication()
        elif self.record_phase is RecordPhase.PRE_PROMOTION:
            self._validate_prepromotion()
        else:
            if self.record_phase is not RecordPhase.FINAL:
                raise ValueError("record_phase must be PRE_PUBLICATION, PRE_PROMOTION, or FINAL")
            if self.publication_status is None:
                raise ValueError("final record requires terminal publication status")
            if self.publication_status is PublicationStatus.PUBLISHED:
                self._validate_published()
            elif self.publication_status in _TERMINAL_FAILURES:
                self._validate_failure()
            else:
                raise ValueError("invalid final publication status")

    def _validate_not_applicable(self) -> None:
        values = ((self.render_status, RenderStatus.NOT_APPLICABLE), (self.staging_status, StagingStatus.NOT_APPLICABLE), (self.visibility_status, VisibilityStatus.NOT_APPLICABLE), (self.content_verification_status, ContentVerificationStatus.NOT_APPLICABLE), (self.publication_status, PublicationStatus.NOT_APPLICABLE), (self.snapshot_binding_status, SnapshotBindingStatus.NOT_APPLICABLE))
        if self.record_phase not in {RecordPhase.PRE_PROMOTION, RecordPhase.FINAL} or any(actual is not expected for actual, expected in values):
            raise ValueError("NOT_APPLICABLE artifact must be a fully inapplicable terminal record")
        if any(value is not None for value in (self.snapshot_id_verified, self.staged_sha256, self.final_sha256, self.failure_stage, self.failure_reason)):
            raise ValueError("NOT_APPLICABLE artifact cannot contain identity, digest, or failure data")

    def _validate_prepublication(self) -> None:
        values = ((self.render_status, RenderStatus.NOT_STARTED), (self.staging_status, StagingStatus.NOT_STARTED), (self.visibility_status, VisibilityStatus.NOT_OBSERVED), (self.content_verification_status, ContentVerificationStatus.NOT_ATTEMPTED), (self.snapshot_binding_status, SnapshotBindingStatus.NOT_ATTEMPTED))
        if any(actual is not expected for actual, expected in values):
            raise ValueError("pre-publication record must use the initial unreached state")
        if self.publication_status is not None or self.failure_stage is not None or self.failure_reason is not None or self.staged_sha256 is not None or self.final_sha256 is not None or self.snapshot_id_verified is not None:
            raise ValueError("pre-publication record cannot contain terminal publication data")

    def _validate_prepromotion(self) -> None:
        if self.visibility_status is not VisibilityStatus.NOT_OBSERVED:
            raise ValueError("pre-promotion record cannot claim final visibility")
        if self.content_verification_status is not ContentVerificationStatus.NOT_ATTEMPTED:
            raise ValueError("pre-promotion record cannot claim content verification")
        if self.snapshot_binding_status is not SnapshotBindingStatus.NOT_ATTEMPTED:
            raise ValueError("pre-promotion record cannot claim snapshot binding")
        if self.staged_sha256 is not None or self.final_sha256 is not None:
            raise ValueError("pre-promotion record cannot contain authoritative digests")
        if self.snapshot_id_verified is not None:
            raise ValueError("pre-promotion record cannot claim snapshot verification")
        if self.publication_status is None:
            if (self.render_status is not RenderStatus.RENDERED or
                    self.staging_status is not StagingStatus.STAGED or
                    self.failure_stage is not None or
                    self.failure_reason is not None):
                raise ValueError("staged pre-promotion record has inconsistent state")
            return
        if self.publication_status is PublicationStatus.RENDER_FAILED:
            if not isinstance(self.failure_reason, str) or not self.failure_reason.strip():
                raise ValueError("pre-promotion terminal failure requires a non-empty failure_reason")
            if (self.failure_stage is not FailureStage.RENDER or
                    self.render_status is not RenderStatus.RENDER_FAILED or
                    self.staging_status is not StagingStatus.NOT_STARTED):
                raise ValueError("pre-promotion RENDER_FAILED has inconsistent state")
            return
        if self.publication_status is PublicationStatus.STAGE_FAILED:
            if not isinstance(self.failure_reason, str) or not self.failure_reason.strip():
                raise ValueError("pre-promotion terminal failure requires a non-empty failure_reason")
            if (self.failure_stage is not FailureStage.STAGING_WRITE or
                    self.render_status is not RenderStatus.RENDERED or
                    self.staging_status is not StagingStatus.STAGE_FAILED):
                raise ValueError("pre-promotion STAGE_FAILED has inconsistent state")
            return
        if self.publication_status is PublicationStatus.DEPENDENCY_FAILED:
            if not isinstance(self.failure_reason, str) or not self.failure_reason.strip():
                raise ValueError("pre-promotion terminal failure requires a non-empty failure_reason")
            if (self.failure_stage is not FailureStage.DEPENDENCY or
                    self.render_status not in {RenderStatus.RENDERED, RenderStatus.DEPENDENCY_FAILED} or
                    self.staging_status is not StagingStatus.DEPENDENCY_FAILED):
                raise ValueError("pre-promotion DEPENDENCY_FAILED has inconsistent state")
            return
        raise ValueError("pre-promotion record cannot claim final publication")

    def _validate_published(self) -> None:
        values = ((self.render_status, RenderStatus.RENDERED, "RENDERED"), (self.staging_status, StagingStatus.STAGED, "STAGED"), (self.visibility_status, VisibilityStatus.VISIBLE, "VISIBLE"), (self.content_verification_status, ContentVerificationStatus.VERIFIED, "VERIFIED"), (self.snapshot_binding_status, SnapshotBindingStatus.VERIFIED, "verified binding"))
        for actual, expected, label in values:
            if actual is not expected:
                raise ValueError(f"PUBLISHED requires {label}")
        if self.staged_sha256 is None or self.final_sha256 is None or self.staged_sha256 != self.final_sha256:
            raise ValueError("PUBLISHED requires equal staged and final SHA-256 digests")
        if self.failure_stage is not None or self.failure_reason is not None:
            raise ValueError("PUBLISHED cannot contain failure metadata")
        if self.representation is ArtifactRepresentation.SELF_DESCRIBING and self.snapshot_id_verified is not True:
            raise ValueError("self-describing PUBLISHED artifact requires verified embedded identity")
        if self.representation is ArtifactRepresentation.OPAQUE and self.snapshot_id_verified is not None:
            raise ValueError("opaque PUBLISHED artifact requires snapshot_id_verified=None")

    def _validate_failure(self) -> None:
        if not isinstance(self.failure_reason, str) or not self.failure_reason.strip():
            raise ValueError("terminal failure requires a non-empty failure_reason")
        if self.final_sha256 is not None:
            raise ValueError("terminal failure cannot expose an authoritative final digest")
        if self.publication_status is PublicationStatus.RENDER_FAILED:
            if (self.failure_stage is not FailureStage.RENDER or self.render_status is not RenderStatus.RENDER_FAILED or self.staging_status is not StagingStatus.NOT_STARTED or self.visibility_status is not VisibilityStatus.NOT_OBSERVED or self.content_verification_status is not ContentVerificationStatus.NOT_ATTEMPTED or self.snapshot_binding_status is not SnapshotBindingStatus.NOT_ATTEMPTED or self.staged_sha256 is not None or self.snapshot_id_verified is not None):
                raise ValueError("RENDER_FAILED cannot claim later-stage success")
            return
        if self.publication_status is PublicationStatus.STAGE_FAILED:
            if (self.failure_stage is not FailureStage.STAGING_WRITE or self.render_status is not RenderStatus.RENDERED or self.staging_status is not StagingStatus.STAGE_FAILED or self.visibility_status is not VisibilityStatus.NOT_OBSERVED or self.content_verification_status is not ContentVerificationStatus.NOT_ATTEMPTED or self.snapshot_binding_status is not SnapshotBindingStatus.NOT_ATTEMPTED or self.staged_sha256 is not None or self.snapshot_id_verified is not None):
                raise ValueError("STAGE_FAILED cannot claim later-stage success")
            return
        if self.publication_status is PublicationStatus.DEPENDENCY_FAILED:
            if (self.failure_stage is not FailureStage.DEPENDENCY or self.render_status not in {RenderStatus.RENDERED, RenderStatus.DEPENDENCY_FAILED} or self.staging_status is not StagingStatus.DEPENDENCY_FAILED or self.visibility_status is not VisibilityStatus.NOT_OBSERVED or self.content_verification_status is not ContentVerificationStatus.NOT_ATTEMPTED or self.snapshot_binding_status is not SnapshotBindingStatus.NOT_ATTEMPTED or self.staged_sha256 is not None or self.snapshot_id_verified is not None):
                raise ValueError("DEPENDENCY_FAILED cannot claim later-stage success")
            return
        if self.render_status is not RenderStatus.RENDERED or self.staging_status is not StagingStatus.STAGED or self.staged_sha256 is None:
            raise ValueError("VERIFICATION_FAILED requires completed rendering and staging")
        if self.failure_stage is FailureStage.FINAL_PATH_OBSERVATION:
            if (self.visibility_status not in {VisibilityStatus.MISSING, VisibilityStatus.INVALID_FILE_TYPE, VisibilityStatus.UNREADABLE} or self.content_verification_status is not ContentVerificationStatus.NOT_ATTEMPTED or self.snapshot_binding_status is not SnapshotBindingStatus.NOT_ATTEMPTED or self.snapshot_id_verified is not None):
                raise ValueError("FINAL_PATH_OBSERVATION failure has inconsistent subordinate states")
        elif self.failure_stage is FailureStage.DIGEST_VERIFICATION:
            if (self.visibility_status is not VisibilityStatus.VISIBLE or self.content_verification_status is not ContentVerificationStatus.FAILED or self.snapshot_binding_status is not SnapshotBindingStatus.NOT_ATTEMPTED or self.snapshot_id_verified is not None):
                raise ValueError("DIGEST_VERIFICATION failure has inconsistent subordinate states")
        elif self.failure_stage is FailureStage.SNAPSHOT_VERIFICATION:
            if (self.representation is not ArtifactRepresentation.SELF_DESCRIBING or self.visibility_status is not VisibilityStatus.VISIBLE or self.content_verification_status is not ContentVerificationStatus.VERIFIED or self.snapshot_binding_status is not SnapshotBindingStatus.FAILED or self.snapshot_id_verified is not False):
                raise ValueError("SNAPSHOT_VERIFICATION failure has inconsistent subordinate states")
        else:
            raise ValueError("VERIFICATION_FAILED requires a final verification failure stage")

    def with_updates(self, **changes: Any) -> "ArtifactRecord":
        prohibited = set(changes) - _STATE_FIELDS
        if prohibited:
            raise ValueError(f"immutable record identity cannot be replaced: {', '.join(sorted(prohibited))}")
        values = {name: getattr(self, name) for name in _STATE_FIELDS}
        values.update(changes)
        return ArtifactRecord(self._spec, self._layer_a, **values, run_applicability=self.run_applicability)


def build_artifact_inventory(*, include_lumen: bool, include_plots: bool, include_comparison: bool, include_finalization_error: bool = False, include_cylinder: bool | None = None, include_diagnostics: bool = False) -> tuple[ArtifactSpec, ...]:
    """Return current producer-backed payloads; late sidecar is deferred.

    Optional dependencies are satisfied by a NOT_APPLICABLE dependency when
    the corresponding producer is disabled. A failed applicable dependency
    remains a dependency failure for later publication orchestration.
    """
    if include_cylinder is None:
        include_cylinder = include_lumen
    plots = (("tracking_error_plot", "tracking_error.png"), ("trajectory_xy_plot", "trajectory_xy.png"), ("trajectory_3d_plot", "trajectory_3d.png"), ("tip_trajectory_plot", "tip_trajectory.png"), ("command_history_plot", "command_history.png"), ("solve_time_plot", "solve_time.png"), ("cumulative_control_effort_plot", "cumulative_control_effort.png"))
    curved_plots = (
        ("curved_wall_clearance_plot", "curved_wall_clearance.png"),
        ("centerline_tracking_error_plot", "centerline_tracking_error.png"),
        ("curved_lumen_trajectory_plot", "curved_lumen_trajectory_3d.png"),
    )
    diagnostic_plots = (
        ("tactile_safety_response_plot", "tactile_safety_response.png"),
        ("cost_term_breakdown_plot", "cost_term_breakdown.png"),
        ("mppi_computation_breakdown_plot", "mppi_computation_breakdown.png"),
        ("deadline_analysis_plot", "deadline_analysis.png"),
    )
    plot_names = tuple(name for name, _ in plots)
    specs = [
        ArtifactSpec("raw_state", "state.csv", True, Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("raw_tip", "tip.csv", True, Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("raw_reference", "reference.csv", True, Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("raw_command", "command.csv", True, Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("solve_timing", "solve_timing.csv", True, Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("horizon", "horizon.csv", True, Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("reference_path", "reference_path.csv", True, Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("backbone", "backbone.csv", True, Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("tactile_safety_evidence", "tactile_safety.csv", False, _app(include_diagnostics), ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("mppi_cost_terms", "mppi_cost_terms.csv", False, _app(include_diagnostics), ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("mppi_computation", "mppi_computation.csv", False, _app(include_diagnostics), ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("metadata", "metadata.yaml", True, Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("summary", "summary.json", True, Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("aligned_samples", "aligned_samples.csv", True, Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("lumen_evaluation", "lumen_evaluation.csv", False, _app(include_lumen), ArtifactRepresentation.OPAQUE, "regular_file", ("aligned_samples",)),
        ArtifactSpec("cylinder_navigation", "cylinder_navigation.csv", False, _app(include_cylinder), ArtifactRepresentation.OPAQUE, "regular_file"),
        *[ArtifactSpec(name, path, False, _app(include_plots), ArtifactRepresentation.OPAQUE, "regular_file", ("aligned_samples",)) for name, path in plots],
        *[
            ArtifactSpec(
                name,
                path,
                False,
                _app(include_lumen and include_plots),
                ArtifactRepresentation.OPAQUE,
                "regular_file",
                ("lumen_evaluation", "aligned_samples", "summary"),
            )
            for name, path in curved_plots
        ],
        *[
            ArtifactSpec(
                name,
                path,
                False,
                _app(include_diagnostics and include_plots),
                ArtifactRepresentation.OPAQUE,
                "regular_file",
                (
                    "tactile_safety_evidence"
                    if name == "tactile_safety_response_plot"
                    else "mppi_cost_terms"
                    if name == "cost_term_breakdown_plot"
                    else "mppi_computation"
                ,),
            )
            for name, path in diagnostic_plots
        ],
        ArtifactSpec("wall_clearance_plot", "wall_clearance.png", False, _app(include_cylinder and include_plots), ArtifactRepresentation.OPAQUE, "regular_file", ("cylinder_navigation",)),
        ArtifactSpec("cylinder_backbone_target_plot", "cylinder_backbone_target_3d.png", False, _app(include_cylinder and include_plots), ArtifactRepresentation.OPAQUE, "regular_file", ("cylinder_navigation", "metadata", "backbone")),
        ArtifactSpec("comparison", "comparison.json", False, _app(include_comparison), ArtifactRepresentation.OPAQUE, "regular_file", ("metadata", "summary")),
        ArtifactSpec("comparison_report", "comparison.md", False, _app(include_comparison), ArtifactRepresentation.OPAQUE, "regular_file", ("comparison",)),
        # The report remains publishable when optional lumen diagnostics fail;
        # each curved plot has its own explicit dependency record and is listed
        # by the report only when it was actually produced.
        ArtifactSpec("report", "report.md", True, Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE, "regular_file", ("metadata", "summary", "aligned_samples", "comparison", "comparison_report", *plot_names, "wall_clearance_plot", "cylinder_backbone_target_plot", *(name for name, _ in diagnostic_plots))),
        ArtifactSpec("orchestration", "orchestration.json", True, Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("finalization_trace", "finalization_trace.json", False, Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE, "regular_file"),
        ArtifactSpec("finalization_error", "finalization_error.json", False, _app(include_finalization_error), ArtifactRepresentation.OPAQUE, "regular_file"),
    ]
    validate_artifact_specs(specs)
    return tuple(specs)


def prepublication_record(spec: ArtifactSpec, layer_a: LayerASnapshot) -> ArtifactRecord:
    if not isinstance(spec, ArtifactSpec):
        raise TypeError("prepublication_record requires an ArtifactSpec")
    if not isinstance(layer_a, LayerASnapshot):
        raise TypeError("prepublication_record requires a LayerASnapshot")
    if spec.applicability is Applicability.NOT_APPLICABLE:
        return ArtifactRecord(spec, layer_a, RecordPhase.FINAL, RenderStatus.NOT_APPLICABLE, StagingStatus.NOT_APPLICABLE, VisibilityStatus.NOT_APPLICABLE, ContentVerificationStatus.NOT_APPLICABLE, PublicationStatus.NOT_APPLICABLE, None, None, None, None, None, SnapshotBindingStatus.NOT_APPLICABLE, run_applicability=Applicability.NOT_APPLICABLE)
    return ArtifactRecord(spec, layer_a, RecordPhase.PRE_PUBLICATION, RenderStatus.NOT_STARTED, StagingStatus.NOT_STARTED, VisibilityStatus.NOT_OBSERVED, ContentVerificationStatus.NOT_ATTEMPTED, None, None, None, None, None, None, SnapshotBindingStatus.NOT_ATTEMPTED)


def prepromotion_staged_record(spec: ArtifactSpec, layer_a: LayerASnapshot, *, run_applicability: Applicability = Applicability.APPLICABLE) -> ArtifactRecord:
    if not isinstance(spec, ArtifactSpec) or not isinstance(layer_a, LayerASnapshot):
        raise TypeError("prepromotion_staged_record requires valid authority objects")
    return ArtifactRecord(
        spec, layer_a, RecordPhase.PRE_PROMOTION,
        RenderStatus.RENDERED, StagingStatus.STAGED,
        VisibilityStatus.NOT_OBSERVED, ContentVerificationStatus.NOT_ATTEMPTED,
        None, None, None, None, None, None,
        SnapshotBindingStatus.NOT_ATTEMPTED,
        run_applicability=run_applicability,
    )


def prepromotion_not_applicable_record(spec: ArtifactSpec, layer_a: LayerASnapshot, *, run_applicability: Applicability = Applicability.NOT_APPLICABLE) -> ArtifactRecord:
    if not isinstance(spec, ArtifactSpec) or not isinstance(layer_a, LayerASnapshot):
        raise TypeError("prepromotion_not_applicable_record requires valid authority objects")
    if run_applicability is not Applicability.NOT_APPLICABLE:
        raise ValueError("prepromotion_not_applicable_record requires NOT_APPLICABLE execution applicability")
    return ArtifactRecord(
        spec, layer_a, RecordPhase.PRE_PROMOTION,
        RenderStatus.NOT_APPLICABLE, StagingStatus.NOT_APPLICABLE,
        VisibilityStatus.NOT_APPLICABLE, ContentVerificationStatus.NOT_APPLICABLE,
        PublicationStatus.NOT_APPLICABLE, None, None, None, None, None,
        SnapshotBindingStatus.NOT_APPLICABLE,
        run_applicability=run_applicability,
    )


def prepromotion_failure_record(
    spec: ArtifactSpec,
    layer_a: LayerASnapshot,
    publication_status: PublicationStatus,
    failure_reason: str,
    *,
    render_status: RenderStatus | None = None,
    run_applicability: Applicability = Applicability.APPLICABLE,
) -> ArtifactRecord:
    if publication_status is PublicationStatus.RENDER_FAILED:
        stage = FailureStage.RENDER
        render = RenderStatus.RENDER_FAILED
        staging = StagingStatus.NOT_STARTED
    elif publication_status is PublicationStatus.STAGE_FAILED:
        stage = FailureStage.STAGING_WRITE
        render = RenderStatus.RENDERED
        staging = StagingStatus.STAGE_FAILED
    elif publication_status is PublicationStatus.DEPENDENCY_FAILED:
        stage = FailureStage.DEPENDENCY
        render = render_status or RenderStatus.DEPENDENCY_FAILED
        staging = StagingStatus.DEPENDENCY_FAILED
    else:
        raise ValueError("prepromotion_failure_record requires a pre-promotion failure status")
    return ArtifactRecord(
        spec, layer_a, RecordPhase.PRE_PROMOTION,
        render, staging, VisibilityStatus.NOT_OBSERVED,
        ContentVerificationStatus.NOT_ATTEMPTED, publication_status,
        stage, failure_reason, None, None, None,
        SnapshotBindingStatus.NOT_ATTEMPTED,
        run_applicability=run_applicability,
    )


@dataclass(frozen=True)
class PrePromotionLedger:
    records: tuple[ArtifactRecord, ...]
    inventory: tuple[ArtifactSpec, ...]
    staging_root: Path

    def __post_init__(self) -> None:
        records = tuple(self.records)
        inventory = tuple(self.inventory)
        object.__setattr__(self, "records", records)
        object.__setattr__(self, "inventory", inventory)
        validate_prepromotion_ledger(records, inventory)
        if not isinstance(self.staging_root, Path):
            raise TypeError("staging_root must be a Path")

    @property
    def by_name(self) -> dict[str, ArtifactRecord]:
        return {record.logical_name: record for record in self.records}


def validate_prepromotion_ledger(
    records: Iterable[ArtifactRecord],
    inventory: Iterable[ArtifactSpec],
) -> None:
    items = tuple(records)
    if any(not isinstance(record, ArtifactRecord) for record in items):
        raise TypeError("pre-promotion ledger records must be ArtifactRecord instances")
    names = tuple(record.logical_name for record in items)
    if len(names) != len(set(names)):
        raise ValueError("pre-promotion ledger contains duplicate logical names")
    specs = tuple(inventory)
    validate_artifact_specs(specs)
    expected = tuple(spec.logical_name for spec in specs)
    if set(names) != set(expected) or len(names) != len(expected):
        raise ValueError("pre-promotion ledger must contain exactly the complete inventory")
    if any(record.spec is not spec for record, spec in zip(items, specs)):
        raise ValueError("pre-promotion ledger record order must match the supplied inventory")
    expected_by_name = {spec.logical_name: spec for spec in specs}
    if any(record.spec is not expected_by_name[record.logical_name] for record in items):
        raise ValueError("pre-promotion ledger records must retain the supplied inventory specs")
    for record in items:
        if record.run_applicability is Applicability.NOT_APPLICABLE:
            continue
        if record.record_phase is not RecordPhase.PRE_PROMOTION:
            raise ValueError(f"ledger record is not pre-promotion: {record.logical_name}")
        if record.publication_status is PublicationStatus.PUBLISHED:
            raise ValueError("pre-promotion ledger cannot contain PUBLISHED")
        if record.visibility_status is not VisibilityStatus.NOT_OBSERVED:
            raise ValueError("pre-promotion ledger cannot contain final visibility")
        if record.content_verification_status is not ContentVerificationStatus.NOT_ATTEMPTED:
            raise ValueError("pre-promotion ledger cannot contain content verification")
        if record.snapshot_binding_status is not SnapshotBindingStatus.NOT_ATTEMPTED:
            raise ValueError("pre-promotion ledger cannot contain snapshot binding")
        if record.final_sha256 is not None or record.snapshot_id_verified is not None:
            raise ValueError("pre-promotion ledger cannot contain final publication data")


def classify_result_directory(path: str | Path) -> ResultAuthority:
    return ResultAuthority.RECEIPT_PRESENT_UNVALIDATED if (Path(path) / RECEIPT_RELATIVE_PATH).is_file() else ResultAuthority.LEGACY_UNVERIFIED


def validate_artifact_specs(specs: Iterable[ArtifactSpec]) -> None:
    items = tuple(specs)
    names = [item.logical_name for item in items]
    paths = [_canonical_relative_path(item.relative_path) for item in items]
    if len(names) != len(set(names)):
        raise ValueError("duplicate logical artifact name")
    if len(paths) != len(set(paths)):
        raise ValueError("duplicate artifact relative path")
    name_set = set(names)
    graph = {item.logical_name: item.dependencies for item in items}
    for item in items:
        for dependency in item.dependencies:
            if dependency == RECEIPT_LOGICAL_NAME:
                raise ValueError("receipt commit artifact cannot be a payload dependency")
            if dependency not in name_set:
                raise ValueError(f"unknown artifact dependency: {item.logical_name}->{dependency}")
    visiting: set[str] = set()
    visited: set[str] = set()

    def visit(name: str) -> None:
        if name in visiting:
            raise ValueError(f"artifact dependency cycle involving: {name}")
        if name in visited:
            return
        visiting.add(name)
        for dependency in graph[name]:
            visit(dependency)
        visiting.remove(name)
        visited.add(name)

    for name in sorted(name_set):
        visit(name)


def _app(enabled: bool) -> Applicability:
    return Applicability.APPLICABLE if enabled else Applicability.NOT_APPLICABLE


def _canonical_relative_path(value: str, *, allow_receipt: bool = False) -> str:
    if not isinstance(value, str) or not value or "\x00" in value:
        raise ValueError(f"unsafe relative artifact path: {value!r}")
    if value.startswith("/") or value.endswith("/") or "\\" in value or "//" in value:
        raise ValueError(f"unsafe relative artifact path: {value!r}")
    path = PurePosixPath(value)
    if path == PurePosixPath(".") or any(part in {"", ".", ".."} for part in path.parts) or str(path) != value:
        raise ValueError(f"artifact path is not canonical POSIX form: {value!r}")
    if not allow_receipt and value == RECEIPT_RELATIVE_PATH:
        raise ValueError("receipt path is reserved for CommitArtifactSpec")
    return value


def _validate_relative_path(value: str, *, allow_receipt: bool = False) -> None:
    _canonical_relative_path(value, allow_receipt=allow_receipt)


def _validate_digest(value: str | None, label: str) -> None:
    if value is not None and not _DIGEST_RE.fullmatch(value):
        raise ValueError(f"{label} must be lowercase hexadecimal SHA-256")
