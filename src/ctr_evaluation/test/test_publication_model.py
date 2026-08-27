import sys
# flake8: noqa: E501
import tempfile
import math
import types
import unittest
from dataclasses import replace
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ctr_evaluation.publication_model import (  # noqa: E402
    Applicability, ArtifactRecord, ArtifactRepresentation, ArtifactSpec,
    CommitArtifactSpec, ContentVerificationStatus, FailureStage, LayerASnapshot,
    PublicationStatus, RecordPhase, RenderStatus, ResultAuthority,
    SnapshotBindingStatus, StagingStatus, VisibilityStatus,
    build_artifact_inventory, classify_result_directory, prepublication_record,
    prepromotion_failure_record, prepromotion_staged_record,
    prepromotion_not_applicable_record, PrePromotionLedger,
    validate_artifact_specs, validate_prepromotion_ledger,
)

DIGEST = "a" * 64


def layer(**changes):
    values = dict(snapshot_id="snapshot-1", operational_reason="none",
                  workflow_classification="COMPLETED", workflow_exit_code=0,
                  comparison_valid=True, timing_descriptive=True,
                  timeout_status="NONE", cancellation_evidence=(),
                  delivery_classification="both_confirmed", compatibility_valid=True,
                  timing_data=(("solve_latency_s", 0.1),))
    values.update(changes)
    return LayerASnapshot(**values)


def spec(name="summary", path="summary.json", *, required=True,
         applicability=Applicability.APPLICABLE,
         representation=ArtifactRepresentation.SELF_DESCRIBING,
         dependencies=()):
    return ArtifactSpec(name, path, required, applicability, representation,
                        "regular_file", dependencies)


def published(*, representation=ArtifactRepresentation.SELF_DESCRIBING):
    return ArtifactRecord(
        spec("summary", "summary.json", representation=representation), layer(),
        RecordPhase.FINAL, RenderStatus.RENDERED, StagingStatus.STAGED,
        VisibilityStatus.VISIBLE, ContentVerificationStatus.VERIFIED,
        PublicationStatus.PUBLISHED, None, None, DIGEST, DIGEST,
        True if representation is ArtifactRepresentation.SELF_DESCRIBING else None,
        SnapshotBindingStatus.VERIFIED,
    )


def failure(status, stage, **changes):
    values = dict(
        spec=spec("summary", "summary.json"), layer_a=layer(),
        record_phase=RecordPhase.FINAL, render_status=RenderStatus.RENDERED,
        staging_status=StagingStatus.STAGED, visibility_status=VisibilityStatus.VISIBLE,
        content_verification_status=ContentVerificationStatus.FAILED,
        publication_status=status, failure_stage=stage,
        failure_reason="deterministic failure", staged_sha256=DIGEST,
        final_sha256=None, snapshot_id_verified=None,
        snapshot_binding_status=SnapshotBindingStatus.NOT_ATTEMPTED,
    )
    if status is PublicationStatus.RENDER_FAILED:
        values.update(render_status=RenderStatus.RENDER_FAILED,
                      staging_status=StagingStatus.NOT_STARTED,
                      visibility_status=VisibilityStatus.NOT_OBSERVED,
                      content_verification_status=ContentVerificationStatus.NOT_ATTEMPTED,
                      staged_sha256=None)
    elif status is PublicationStatus.STAGE_FAILED:
        values.update(staging_status=StagingStatus.STAGE_FAILED,
                      visibility_status=VisibilityStatus.NOT_OBSERVED,
                      content_verification_status=ContentVerificationStatus.NOT_ATTEMPTED,
                      staged_sha256=None)
    elif status is PublicationStatus.DEPENDENCY_FAILED:
        values.update(staging_status=StagingStatus.DEPENDENCY_FAILED,
                      visibility_status=VisibilityStatus.NOT_OBSERVED,
                      content_verification_status=ContentVerificationStatus.NOT_ATTEMPTED,
                      staged_sha256=None)
    values.update(changes)
    values["layer_a"] = values.pop("layer_a")
    return ArtifactRecord(**values)


class PublicationModelTest(unittest.TestCase):
    def test_inventory_is_deterministic_and_current_producer_backed(self):
        kwargs = dict(include_lumen=True, include_plots=True,
                      include_comparison=True, include_finalization_error=True)
        first = build_artifact_inventory(**kwargs)
        self.assertEqual(first, build_artifact_inventory(**kwargs))
        names = {item.logical_name for item in first}
        expected = {"raw_state", "raw_tip", "raw_reference", "raw_command",
                    "solve_timing", "horizon", "reference_path", "backbone",
                    "metadata", "summary", "aligned_samples", "lumen_evaluation",
                    "cylinder_navigation", "tracking_error_plot", "trajectory_xy_plot",
                    "trajectory_3d_plot", "tip_trajectory_plot", "command_history_plot",
                    "solve_time_plot", "cumulative_control_effort_plot",
                    "curved_wall_clearance_plot", "centerline_tracking_error_plot",
                    "curved_lumen_trajectory_plot",
                    "tactile_safety_evidence", "mppi_cost_terms", "mppi_computation",
                    "tactile_safety_response_plot", "cost_term_breakdown_plot",
                    "mppi_computation_breakdown_plot", "deadline_analysis_plot",
                    "wall_clearance_plot", "cylinder_backbone_target_plot", "comparison",
                    "comparison_report", "report", "orchestration", "finalization_trace",
                    "finalization_error"}
        self.assertEqual(expected, names)
        self.assertNotIn("publication_receipt", names)
        self.assertNotIn("late_response_sidecar", names)
        self.assertTrue(all(item.representation is ArtifactRepresentation.OPAQUE for item in first))

    def test_conditional_inventory_and_producer_classifications(self):
        specs = build_artifact_inventory(include_lumen=False, include_plots=False,
                                         include_comparison=False)
        by_name = {item.logical_name: item for item in specs}
        for name in ("lumen_evaluation", "tracking_error_plot", "comparison", "finalization_error"):
            self.assertIs(by_name[name].applicability, Applicability.NOT_APPLICABLE)
        self.assertFalse(by_name["finalization_trace"].required)
        self.assertIs(by_name["finalization_trace"].representation, ArtifactRepresentation.OPAQUE)
        self.assertEqual((), by_name["cylinder_navigation"].dependencies)
        self.assertIn("wall_clearance_plot", by_name["report"].dependencies)
        self.assertIn("cylinder_backbone_target_plot", by_name["report"].dependencies)
        self.assertIs(by_name["wall_clearance_plot"].applicability, Applicability.NOT_APPLICABLE)
        self.assertNotIn("aggregate_summary", by_name)
        self.assertNotIn("aggregate_report", by_name)

    def test_cylinder_applicability_is_independent_of_lumen_evaluation(self):
        specs = build_artifact_inventory(
            include_lumen=False,
            include_cylinder=True,
            include_plots=True,
            include_comparison=False,
        )
        by_name = {item.logical_name: item for item in specs}
        self.assertIs(
            Applicability.NOT_APPLICABLE,
            by_name["lumen_evaluation"].applicability,
        )
        self.assertIs(
            Applicability.APPLICABLE,
            by_name["cylinder_navigation"].applicability,
        )
        self.assertIs(
            Applicability.APPLICABLE,
            by_name["wall_clearance_plot"].applicability,
        )
        self.assertNotIn(
            "lumen_evaluation",
            by_name["wall_clearance_plot"].dependencies,
        )

    def test_inventory_dependencies_are_real_payload_edges(self):
        by_name = {item.logical_name: item for item in build_artifact_inventory(include_lumen=True, include_plots=True, include_comparison=True)}
        self.assertEqual(("aligned_samples",), by_name["lumen_evaluation"].dependencies)
        self.assertEqual(("cylinder_navigation",), by_name["wall_clearance_plot"].dependencies)
        self.assertEqual(("cylinder_navigation", "metadata", "backbone"), by_name["cylinder_backbone_target_plot"].dependencies)
        self.assertNotIn("lumen_evaluation", by_name["wall_clearance_plot"].dependencies)
        self.assertNotIn("lumen_evaluation", by_name["cylinder_backbone_target_plot"].dependencies)
        self.assertIn("summary", by_name["comparison"].dependencies)
        self.assertIn("comparison", by_name["report"].dependencies)
        self.assertNotIn("baseline", by_name["comparison"].dependencies)

    def test_multiple_dependency_graph_and_order_independence(self):
        specs = [spec("root", "root.json"), spec("left", "left.json", dependencies=("root",)), spec("right", "right.json", dependencies=("root",)), spec("out", "out.json", dependencies=("left", "right"))]
        validate_artifact_specs(specs)
        validate_artifact_specs(list(reversed(specs)))
        with self.assertRaisesRegex(ValueError, "duplicate"):
            spec("bad", "bad.json", dependencies=("root", "root"))
        with self.assertRaisesRegex(ValueError, "cycle"):
            validate_artifact_specs([spec("a", "a.json", dependencies=("b", "c")), spec("b", "b.json", dependencies=("c",)), spec("c", "c.json", dependencies=("a",))])

    def test_dependency_rejections(self):
        with self.assertRaisesRegex(TypeError, "ordered iterable"):
            spec("set_deps", "set_deps.json", dependencies={"a", "b"})
        with self.assertRaisesRegex(ValueError, "nonempty"):
            spec("blank_deps", "blank_deps.json", dependencies=("   ",))
        with self.assertRaisesRegex(ValueError, "unknown"):
            validate_artifact_specs([spec("a", "a.json", dependencies=("missing",))])
        with self.assertRaisesRegex(ValueError, "cannot depend"):
            spec("a", "a.json", dependencies=("a",))
        with self.assertRaisesRegex(ValueError, "receipt"):
            validate_artifact_specs([spec("a", "a.json", dependencies=("publication_receipt",))])

    def test_identity_is_structural_and_replace_cannot_rebind(self):
        current = published()
        self.assertIsInstance(current.spec, ArtifactSpec)
        self.assertIsInstance(current.layer_a, LayerASnapshot)
        self.assertEqual(current.layer_a.snapshot_id, current.snapshot_id)
        with self.assertRaises((TypeError, ValueError)):
            replace(current, snapshot_id="other")
        with self.assertRaises((TypeError, ValueError)):
            replace(current, _layer_a=layer(snapshot_id="other"))
        with self.assertRaises((TypeError, ValueError)):
            replace(current, _spec=spec("other", "other.json"))

    def test_constructor_rejects_fake_authority_objects(self):
        state = dict(
            record_phase=RecordPhase.PRE_PUBLICATION,
            render_status=RenderStatus.NOT_STARTED,
            staging_status=StagingStatus.NOT_STARTED,
            visibility_status=VisibilityStatus.NOT_OBSERVED,
            content_verification_status=ContentVerificationStatus.NOT_ATTEMPTED,
            publication_status=None,
            failure_stage=None,
            failure_reason=None,
            staged_sha256=None,
            final_sha256=None,
            snapshot_id_verified=None,
            snapshot_binding_status=SnapshotBindingStatus.NOT_ATTEMPTED,
        )
        fake_spec = types.SimpleNamespace(**spec().__dict__)
        fake_layer = types.SimpleNamespace(**layer().__dict__)
        for fake_spec_value, fake_layer_value in ((fake_spec, layer()), (spec(), fake_layer), ({}, layer()), (spec(), {})):
            with self.subTest(spec_type=type(fake_spec_value).__name__, layer_type=type(fake_layer_value).__name__):
                with self.assertRaises(TypeError):
                    ArtifactRecord(spec=fake_spec_value, layer_a=fake_layer_value, **state)
        fake_spec.logical_name = "mutated"
        fake_layer.snapshot_id = "mutated"
        valid = published()
        self.assertEqual("summary", valid.logical_name)
        self.assertEqual("snapshot-1", valid.snapshot_id)

    def test_spec_types_and_separate_snapshot_identity(self):
        with self.assertRaises(TypeError):
            spec(applicability="APPLICABLE")
        with self.assertRaises(TypeError):
            spec(representation="opaque")
        for value in (0, 1, None, "true", object()):
            with self.subTest(required=value):
                with self.assertRaises(TypeError):
                    spec(required=value)
        first = prepublication_record(spec(), layer())
        second_layer = layer(snapshot_id="snapshot-2")
        second = prepublication_record(spec("second", "second.json"), second_layer)
        self.assertIsNot(first.layer_a, second.layer_a)
        self.assertEqual("snapshot-1", first.snapshot_id)
        self.assertEqual("snapshot-2", second.snapshot_id)

    def test_with_updates_rejects_all_identity_fields(self):
        current = published()
        values = {"snapshot_id": "x", "logical_name": "x", "relative_path": "x.json", "required": False, "applicability": Applicability.NOT_APPLICABLE, "representation": ArtifactRepresentation.OPAQUE, "expected_file_type": "directory", "dependencies": ("x",), "spec": spec("x", "x.json"), "layer_a": layer(snapshot_id="other")}
        for name, value in values.items():
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    current.with_updates(**{name: value})

    def test_with_updates_returns_new_validated_record_and_preserves_authority(self):
        current = published()
        changed = current.with_updates(record_phase=RecordPhase.FINAL)
        self.assertIsNot(current, changed)
        self.assertIs(current.spec, changed.spec)
        self.assertIs(current.layer_a, changed.layer_a)
        self.assertIs(current.publication_status, PublicationStatus.PUBLISHED)
        self.assertIsNone(current.failure_stage)
        self.assertIsNone(current.failure_reason)
        with self.assertRaises(ValueError):
            current.with_updates(failure_reason="bad")

    def test_layer_a_freezes_nested_mutable_values(self):
        original = {"controller": ["confirmed"]}
        timing = [{"timing_pass": False}]
        snapshot = layer(cancellation_evidence=original, timing_data=timing)
        original["controller"].append("changed")
        timing[0]["timing_pass"] = True
        self.assertNotIn("changed", repr(snapshot.cancellation_evidence))
        self.assertIn("False", repr(snapshot.timing_data))
        enum_snapshot = layer(cancellation_evidence=(Applicability.APPLICABLE,))
        self.assertEqual(("APPLICABLE",), enum_snapshot.cancellation_evidence)
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(non_finite=value):
                with self.assertRaises(ValueError):
                    layer(cancellation_evidence=value)
        for value in ({"set": {1, 2}}, {"tuple": ([1],)}):
            with self.subTest(value=value):
                frozen = layer(cancellation_evidence=value)
                value.clear()
                self.assertTrue(frozen.cancellation_evidence)
        for value in ((object(),),):
            with self.subTest(value=value):
                with self.assertRaises((TypeError, ValueError)):
                    layer(cancellation_evidence=value)

    def test_layer_a_operational_fields_are_preserved(self):
        snapshot = layer(operational_reason="timeout", workflow_classification="BLOCKED", workflow_exit_code=2, comparison_valid=False, compatibility_valid=False, timeout_status="TIMED_OUT", delivery_classification="none_confirmed", timing_data=(("timing_pass", False),))
        changed = prepublication_record(spec(), snapshot).with_updates(record_phase=RecordPhase.PRE_PUBLICATION)
        self.assertIs(changed.layer_a, snapshot)
        self.assertEqual((snapshot.snapshot_id, snapshot.workflow_classification, snapshot.workflow_exit_code, snapshot.operational_reason, snapshot.timeout_status, snapshot.cancellation_evidence, snapshot.delivery_classification, snapshot.comparison_valid, snapshot.compatibility_valid, snapshot.timing_data), (changed.snapshot_id, changed.layer_a.workflow_classification, changed.layer_a.workflow_exit_code, changed.layer_a.operational_reason, changed.layer_a.timeout_status, changed.layer_a.cancellation_evidence, changed.layer_a.delivery_classification, changed.layer_a.comparison_valid, changed.layer_a.compatibility_valid, changed.layer_a.timing_data))
        self.assertTrue(changed.layer_a.timing_descriptive)

    def test_published_success_and_representation_rules(self):
        published()
        published(representation=ArtifactRepresentation.OPAQUE)
        with self.assertRaises(ValueError):
            ArtifactRecord(spec("summary", "summary.json"), layer(), RecordPhase.FINAL, RenderStatus.RENDERED, StagingStatus.STAGED, VisibilityStatus.VISIBLE, ContentVerificationStatus.VERIFIED, PublicationStatus.PUBLISHED, None, None, None, DIGEST, True, SnapshotBindingStatus.VERIFIED)
        with self.assertRaises(ValueError):
            ArtifactRecord(spec("plot", "plot.png", representation=ArtifactRepresentation.OPAQUE), layer(), RecordPhase.FINAL, RenderStatus.RENDERED, StagingStatus.STAGED, VisibilityStatus.VISIBLE, ContentVerificationStatus.VERIFIED, PublicationStatus.PUBLISHED, None, None, DIGEST, DIGEST, False, SnapshotBindingStatus.VERIFIED)

    def test_published_digest_and_failure_rules(self):
        for changes in ({"staged_sha256": None}, {"final_sha256": None}, {"final_sha256": "b" * 64}, {"failure_stage": FailureStage.RENDER}, {"failure_reason": "bad"}):
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    values = dict(spec=spec(), layer_a=layer(), record_phase=RecordPhase.FINAL, render_status=RenderStatus.RENDERED, staging_status=StagingStatus.STAGED, visibility_status=VisibilityStatus.VISIBLE, content_verification_status=ContentVerificationStatus.VERIFIED, publication_status=PublicationStatus.PUBLISHED, failure_stage=None, failure_reason=None, staged_sha256=DIGEST, final_sha256=DIGEST, snapshot_id_verified=True, snapshot_binding_status=SnapshotBindingStatus.VERIFIED)
                    values.update(changes)
                    ArtifactRecord(**values)

    def test_prepublication_and_not_applicable(self):
        pre = prepublication_record(spec(), layer())
        self.assertIsNone(pre.publication_status)
        with self.assertRaises(ValueError):
            pre.with_updates(publication_status=PublicationStatus.PUBLISHED)
        optional = prepublication_record(spec("optional", "optional.json", required=False, applicability=Applicability.NOT_APPLICABLE), layer())
        self.assertIs(optional.publication_status, PublicationStatus.NOT_APPLICABLE)
        with self.assertRaises(ValueError):
            ArtifactRecord(spec("optional", "optional.json", required=False, applicability=Applicability.APPLICABLE), layer(), RecordPhase.FINAL, RenderStatus.NOT_APPLICABLE, StagingStatus.NOT_APPLICABLE, VisibilityStatus.NOT_APPLICABLE, ContentVerificationStatus.NOT_APPLICABLE, PublicationStatus.NOT_APPLICABLE, None, None, None, None, None, SnapshotBindingStatus.NOT_APPLICABLE)

    def test_failure_matrix(self):
        failure(PublicationStatus.RENDER_FAILED, FailureStage.RENDER)
        failure(PublicationStatus.STAGE_FAILED, FailureStage.STAGING_WRITE)
        failure(PublicationStatus.DEPENDENCY_FAILED, FailureStage.DEPENDENCY)
        failure(PublicationStatus.VERIFICATION_FAILED, FailureStage.FINAL_PATH_OBSERVATION, visibility_status=VisibilityStatus.MISSING, content_verification_status=ContentVerificationStatus.NOT_ATTEMPTED)
        failure(PublicationStatus.VERIFICATION_FAILED, FailureStage.DIGEST_VERIFICATION, visibility_status=VisibilityStatus.VISIBLE, content_verification_status=ContentVerificationStatus.FAILED)
        failure(PublicationStatus.VERIFICATION_FAILED, FailureStage.SNAPSHOT_VERIFICATION, visibility_status=VisibilityStatus.VISIBLE, content_verification_status=ContentVerificationStatus.VERIFIED, snapshot_binding_status=SnapshotBindingStatus.FAILED, snapshot_id_verified=False, spec=spec("summary", "summary.json", representation=ArtifactRepresentation.SELF_DESCRIBING))

    def test_failure_matrix_rejects_impossible_sequences(self):
        cases = [(PublicationStatus.RENDER_FAILED, FailureStage.RENDER, {"staged_sha256": DIGEST}), (PublicationStatus.RENDER_FAILED, FailureStage.RENDER, {"staging_status": StagingStatus.STAGED}), (PublicationStatus.RENDER_FAILED, FailureStage.RENDER, {"visibility_status": VisibilityStatus.VISIBLE}), (PublicationStatus.RENDER_FAILED, FailureStage.RENDER, {"content_verification_status": ContentVerificationStatus.VERIFIED}), (PublicationStatus.RENDER_FAILED, FailureStage.RENDER, {"snapshot_binding_status": SnapshotBindingStatus.VERIFIED}), (PublicationStatus.STAGE_FAILED, FailureStage.STAGING_WRITE, {"final_sha256": DIGEST}), (PublicationStatus.DEPENDENCY_FAILED, FailureStage.DEPENDENCY, {"staged_sha256": DIGEST}), (PublicationStatus.VERIFICATION_FAILED, FailureStage.DIGEST_VERIFICATION, {"staging_status": StagingStatus.NOT_STARTED}), (PublicationStatus.VERIFICATION_FAILED, FailureStage.DIGEST_VERIFICATION, {"snapshot_binding_status": SnapshotBindingStatus.NOT_APPLICABLE}), (PublicationStatus.VERIFICATION_FAILED, FailureStage.DIGEST_VERIFICATION, {"snapshot_binding_status": SnapshotBindingStatus.VERIFIED})]
        for status, stage, changes in cases:
            with self.subTest(status=status, changes=changes):
                with self.assertRaises(ValueError):
                    failure(status, stage, **changes)

    def test_each_verification_failure_stage_rejects_stage_specific_contradictions(self):
        invalid = (
            (FailureStage.FINAL_PATH_OBSERVATION, {"visibility_status": VisibilityStatus.VISIBLE}),
            (FailureStage.DIGEST_VERIFICATION, {"content_verification_status": ContentVerificationStatus.NOT_ATTEMPTED}),
            (FailureStage.SNAPSHOT_VERIFICATION, {"snapshot_id_verified": None}),
        )
        for stage, changes in invalid:
            with self.subTest(stage=stage):
                with self.assertRaises(ValueError):
                    failure(PublicationStatus.VERIFICATION_FAILED, stage, **changes)

    def test_cross_stage_sequencing_and_failure_reason_rules(self):
        cases = (
            {"visibility_status": VisibilityStatus.VISIBLE},
            {"content_verification_status": ContentVerificationStatus.VERIFIED},
            {"snapshot_binding_status": SnapshotBindingStatus.VERIFIED},
            {"final_sha256": DIGEST},
            {"failure_reason": "   "},
        )
        for changes in cases:
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    failure(PublicationStatus.RENDER_FAILED, FailureStage.RENDER, **changes)

    def test_prepromotion_failure_reason_is_required_for_direct_and_factory_paths(self):
        cases = (
            (PublicationStatus.RENDER_FAILED, FailureStage.RENDER),
            (PublicationStatus.STAGE_FAILED, FailureStage.STAGING_WRITE),
            (PublicationStatus.DEPENDENCY_FAILED, FailureStage.DEPENDENCY),
        )
        for status, stage in cases:
            with self.subTest(status=status, reason="valid"):
                record = ArtifactRecord(
                    spec("direct", "direct.json"), layer(),
                    RecordPhase.PRE_PROMOTION,
                    RenderStatus.RENDER_FAILED if status is PublicationStatus.RENDER_FAILED else RenderStatus.RENDERED,
                    StagingStatus.NOT_STARTED if status is PublicationStatus.RENDER_FAILED else StagingStatus.STAGE_FAILED if status is PublicationStatus.STAGE_FAILED else StagingStatus.DEPENDENCY_FAILED,
                    VisibilityStatus.NOT_OBSERVED,
                    ContentVerificationStatus.NOT_ATTEMPTED,
                    status, stage, "valid reason", None, None, None,
                    SnapshotBindingStatus.NOT_ATTEMPTED,
                )
                self.assertEqual("valid reason", record.failure_reason)
            for reason in (None, "", "   ", "\t", "\n"):
                with self.subTest(status=status, reason=reason):
                    with self.assertRaises(ValueError):
                        ArtifactRecord(
                            spec("direct", "direct.json"), layer(),
                            RecordPhase.PRE_PROMOTION,
                            RenderStatus.RENDER_FAILED if status is PublicationStatus.RENDER_FAILED else RenderStatus.RENDERED,
                            StagingStatus.NOT_STARTED if status is PublicationStatus.RENDER_FAILED else StagingStatus.STAGE_FAILED if status is PublicationStatus.STAGE_FAILED else StagingStatus.DEPENDENCY_FAILED,
                            VisibilityStatus.NOT_OBSERVED,
                            ContentVerificationStatus.NOT_ATTEMPTED,
                            status, stage, reason, None, None, None,
                            SnapshotBindingStatus.NOT_ATTEMPTED,
                        )
                    with self.assertRaises(ValueError):
                        prepromotion_failure_record(
                            spec("factory", "factory.json"), layer(), status, reason
                        )

    def test_paths_and_receipt_boundaries(self):
        for value in ("", "./a", "a/./b", "a//b", "a/b/", "a/../b", "../a", "/a", "\\a", "a\\b", "a\x00b", "publication_receipt.json"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    spec(path=value)
        spec("nested_receipt", "x/publication_receipt.json")
        with self.assertRaises(ValueError):
            ArtifactRecord(spec("publication_receipt", "other.json"), layer(), RecordPhase.FINAL, RenderStatus.RENDERED, StagingStatus.STAGED, VisibilityStatus.VISIBLE, ContentVerificationStatus.VERIFIED, PublicationStatus.PUBLISHED, None, None, DIGEST, DIGEST, True, SnapshotBindingStatus.VERIFIED)
        self.assertEqual(CommitArtifactSpec().relative_path, "publication_receipt.json")

    def test_duplicate_paths_and_names(self):
        with self.assertRaises(ValueError):
            validate_artifact_specs([spec("x", "x.json"), spec("x", "y.json")])
        with self.assertRaises(ValueError):
            validate_artifact_specs([spec("x", "x.json"), spec("y", "x.json")])

    def test_receipt_presence_classification(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            self.assertIs(classify_result_directory(path), ResultAuthority.LEGACY_UNVERIFIED)
            (path / "publication_receipt.json").write_text("malformed", encoding="utf-8")
            self.assertIs(classify_result_directory(path), ResultAuthority.RECEIPT_PRESENT_UNVALIDATED)

    def test_import_and_timing_are_disconnected(self):
        self.assertTrue(layer().timing_descriptive)
        self.assertEqual(layer().workflow_exit_code, 0)
        self.assertFalse(layer(comparison_valid=False, compatibility_valid=False).comparison_valid)

    def test_prepromotion_staged_record_has_no_final_claims(self):
        record = prepromotion_staged_record(spec(), layer())
        self.assertIs(record.record_phase, RecordPhase.PRE_PROMOTION)
        self.assertIs(record.render_status, RenderStatus.RENDERED)
        self.assertIs(record.staging_status, StagingStatus.STAGED)
        self.assertIs(record.visibility_status, VisibilityStatus.NOT_OBSERVED)
        self.assertIs(record.content_verification_status, ContentVerificationStatus.NOT_ATTEMPTED)
        self.assertIs(record.snapshot_binding_status, SnapshotBindingStatus.NOT_ATTEMPTED)
        self.assertIsNone(record.publication_status)
        self.assertIsNone(record.staged_sha256)
        self.assertIsNone(record.final_sha256)

    def test_prepromotion_not_applicable_is_terminal(self):
        record = prepromotion_not_applicable_record(
            spec("optional", "optional.json", required=False,
                 applicability=Applicability.NOT_APPLICABLE), layer())
        self.assertIs(record.record_phase, RecordPhase.PRE_PROMOTION)
        self.assertIs(record.publication_status, PublicationStatus.NOT_APPLICABLE)

    def test_prepromotion_failure_records_and_ledger_completeness(self):
        records = (
            prepromotion_failure_record(spec("render", "render.json"), layer(), PublicationStatus.RENDER_FAILED, "render failed"),
            prepromotion_failure_record(spec("stage", "stage.json"), layer(), PublicationStatus.STAGE_FAILED, "stage failed"),
            prepromotion_failure_record(spec("dependency", "dependency.json"), layer(), PublicationStatus.DEPENDENCY_FAILED, "dependency failed"),
        )
        for record, stage in zip(records, (FailureStage.RENDER, FailureStage.STAGING_WRITE, FailureStage.DEPENDENCY)):
            self.assertIs(record.record_phase, RecordPhase.PRE_PROMOTION)
            self.assertIs(record.failure_stage, stage)
            self.assertIs(record.visibility_status, VisibilityStatus.NOT_OBSERVED)
            self.assertIsNone(record.final_sha256)
        inventory = tuple(record.spec for record in records)
        validate_prepromotion_ledger(records, inventory)
        with self.assertRaisesRegex(ValueError, "complete inventory"):
            validate_prepromotion_ledger(records[:-1], inventory)
        with self.assertRaises(ValueError):
            validate_prepromotion_ledger((prepublication_record(inventory[0], layer()),), inventory[:1])

    def test_prepromotion_rejects_final_publication_claims(self):
        staged = prepromotion_staged_record(spec(), layer())
        for changes in (
            {"visibility_status": VisibilityStatus.VISIBLE},
            {"content_verification_status": ContentVerificationStatus.VERIFIED},
            {"snapshot_binding_status": SnapshotBindingStatus.VERIFIED},
            {"publication_status": PublicationStatus.PUBLISHED},
            {"final_sha256": DIGEST},
        ):
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    staged.with_updates(**changes)
        ledger = PrePromotionLedger((staged,), (staged.spec,), Path("run.partial"))
        self.assertEqual((staged,), ledger.records)
        self.assertEqual((staged.spec,), ledger.inventory)

    def test_prepromotion_ledger_requires_complete_immutable_inventory(self):
        staged = prepromotion_staged_record(spec(), layer())
        inventory = (staged.spec,)
        ledger = PrePromotionLedger((staged,), inventory, Path("run.partial"))
        self.assertEqual(inventory, ledger.inventory)
        with self.assertRaisesRegex(ValueError, "complete inventory"):
            PrePromotionLedger((), inventory, Path("empty.partial"))
        self.assertEqual((), PrePromotionLedger((), (), Path("empty.partial")).records)
        other = spec("other", "other.json")
        with self.assertRaisesRegex(ValueError, "complete inventory"):
            PrePromotionLedger((staged,), (other,), Path("extra.partial"))
        same_name_different_spec = spec("summary", "different.json")
        mismatched = prepromotion_staged_record(same_name_different_spec, layer())
        with self.assertRaisesRegex(ValueError, "record order|supplied inventory specs"):
            PrePromotionLedger((mismatched,), inventory, Path("mismatch.partial"))
        mutable_inventory = [staged.spec]
        immutable_ledger = PrePromotionLedger((staged,), mutable_inventory, Path("copy.partial"))
        mutable_inventory.append(other)
        self.assertEqual((staged.spec,), immutable_ledger.inventory)

    def test_prepromotion_ledger_binds_record_order_to_inventory(self):
        specs = (
            spec("a", "a.json"),
            spec("b", "b.json"),
            spec("c", "c.json"),
        )
        records = tuple(prepromotion_staged_record(item, layer()) for item in specs)
        ledger = PrePromotionLedger(records, specs, Path("ordered.partial"))
        self.assertEqual(("a", "b", "c"), tuple(item.logical_name for item in ledger.records))
        for permutation in ((records[1], records[0], records[2]), (records[2], records[0], records[1])):
            with self.subTest(permutation=permutation):
                with self.assertRaisesRegex(ValueError, "record order"):
                    PrePromotionLedger(permutation, specs, Path("permutation.partial"))
        mutable_records = list(records)
        copied = PrePromotionLedger(mutable_records, specs, Path("copy-order.partial"))
        mutable_records.reverse()
        self.assertEqual(("a", "b", "c"), tuple(item.logical_name for item in copied.records))

    def test_execution_applicability_does_not_rewrite_requiredness(self):
        required = spec("required", "required.json")
        record = prepromotion_not_applicable_record(required, layer())
        self.assertTrue(record.required)
        self.assertIs(record.applicability, Applicability.APPLICABLE)
        with self.assertRaisesRegex(ValueError, "immutable record identity"):
            record.with_updates(run_applicability=Applicability.APPLICABLE)
        self.assertIs(record.execution_applicability, Applicability.NOT_APPLICABLE)


if __name__ == "__main__":
    unittest.main()
