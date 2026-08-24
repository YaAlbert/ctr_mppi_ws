import json
import ast
import sys
from dataclasses import FrozenInstanceError
from hashlib import sha256
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ctr_bringup.exercised_subject_identity import (
    IDENTITY_DOMAIN,
    SCHEMA_VERSION,
    STALE_V2_DIAGNOSTIC_IDENTITY,
    AuthenticatedFile,
    ExercisedSubjectError,
    canonical_exercised_subject_bytes,
    exercised_subject_from_mapping,
    exercised_subject_identity,
    make_exercised_subject,
    parse_exercised_subject,
    validate_exercised_subject,
)


def _mapping():
    return {
        "schema_version": SCHEMA_VERSION,
        "candidate_bundle": {
            "path": "manifests/validate_only_bundle.json",
            "size": 501,
            "sha256": "1" * 64,
        },
        "runtime_projection": {
            "path": "manifests/payload_identity_projection.json",
            "size": 502,
            "sha256": "2" * 64,
        },
        "runtime_identity": "3" * 64,
        "materialization_projection": {
            "path": "manifests/materialization_projection.json",
            "size": 503,
            "sha256": "4" * 64,
        },
        "materialization_logical_identity": "5" * 64,
    }


def _subject():
    return exercised_subject_from_mapping(_mapping())


def _assert_code(code, operation):
    with pytest.raises(ExercisedSubjectError) as raised:
        operation()
    assert raised.value.code == code


def test_canonical_contract_and_two_independent_identity_methods():
    subject = _subject()
    expected_bytes = json.dumps(
        _mapping(), sort_keys=True, separators=(",", ":"), ensure_ascii=False,
    ).encode("utf-8")
    assert canonical_exercised_subject_bytes(subject) == expected_bytes
    assert not expected_bytes.endswith(b"\n")
    independently_hashed = sha256(IDENTITY_DOMAIN + expected_bytes).hexdigest()
    assert exercised_subject_identity(subject) == independently_hashed
    assert parse_exercised_subject(expected_bytes) == subject


def test_pure_producer_api_builds_the_same_value():
    value = _mapping()
    produced = make_exercised_subject(
        candidate_bundle=AuthenticatedFile(**value["candidate_bundle"]),
        runtime_projection=AuthenticatedFile(**value["runtime_projection"]),
        runtime_identity=value["runtime_identity"],
        materialization_projection=AuthenticatedFile(
            **value["materialization_projection"],
        ),
        materialization_logical_identity=value["materialization_logical_identity"],
    )
    assert produced == _subject()


@pytest.mark.parametrize(
    ("mutation", "code"),
    [
        (lambda value: value.__setitem__("unknown", 1), "SUBJECT_FIELDS"),
        (lambda value: value.pop("runtime_identity"), "SUBJECT_FIELDS"),
        (
            lambda value: value.__setitem__("schema_version", "ctr-exercised-subject-1"),
            "SUBJECT_SCHEMA",
        ),
        (
            lambda value: value.__setitem__("runtime_identity", 3),
            "SUBJECT_TYPE",
        ),
        (
            lambda value: value.__setitem__("runtime_identity", "x" * 64),
            "SUBJECT_DIGEST",
        ),
        (
            lambda value: value["candidate_bundle"].__setitem__("size", True),
            "SUBJECT_SIZE",
        ),
        (
            lambda value: value["candidate_bundle"].__setitem__("path", "../bundle.json"),
            "SUBJECT_PATH",
        ),
        (
            lambda value: value["candidate_bundle"].__setitem__("extra", 1),
            "SUBJECT_FIELDS",
        ),
    ],
)
def test_strict_schema_failures_have_stable_codes(mutation, code):
    value = _mapping()
    mutation(value)
    _assert_code(code, lambda: exercised_subject_from_mapping(value))


@pytest.mark.parametrize(
    ("field", "code"),
    [
        ("root_authority", "SUBJECT_ROOT_AUTHORITY_DEPENDENCY_CYCLE"),
        ("frozen_root_identity", "SUBJECT_FROZEN_ROOT_DEPENDENCY_CYCLE"),
        ("subject_identity", "SUBJECT_SELF_DEPENDENCY_CYCLE"),
        ("static_closure", "SUBJECT_DOWNSTREAM_DEPENDENCY_CYCLE"),
    ],
)
def test_dependency_cycle_fields_are_explicitly_rejected(field, code):
    value = _mapping()
    value[field] = "6" * 64
    _assert_code(code, lambda: exercised_subject_from_mapping(value))


@pytest.mark.parametrize(
    ("path", "code"),
    [
        ("manifests/root_authority.json", "SUBJECT_ROOT_AUTHORITY_DEPENDENCY_CYCLE"),
        ("manifests/exercised_subject.json", "SUBJECT_SELF_DEPENDENCY_CYCLE"),
        ("manifests/static_closure.json", "SUBJECT_DOWNSTREAM_DEPENDENCY_CYCLE"),
    ],
)
def test_dependency_cycle_paths_are_explicitly_rejected(path, code):
    value = _mapping()
    value["candidate_bundle"]["path"] = path
    _assert_code(code, lambda: exercised_subject_from_mapping(value))


def test_noncanonical_bytes_are_rejected():
    raw = json.dumps(_mapping(), indent=2, ensure_ascii=False).encode("utf-8") + b"\n"
    _assert_code("SUBJECT_NONCANONICAL", lambda: parse_exercised_subject(raw))


def test_duplicate_json_key_is_rejected():
    raw = canonical_exercised_subject_bytes(_subject())
    raw = raw.replace(b'{"candidate_bundle":', b'{"candidate_bundle":{},"candidate_bundle":', 1)
    _assert_code("SUBJECT_JSON_DUPLICATE_KEY", lambda: parse_exercised_subject(raw))


def test_stale_diagnostic_identity_is_never_operative():
    value = _mapping()
    value["runtime_identity"] = STALE_V2_DIAGNOSTIC_IDENTITY
    raw = json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    _assert_code("SUBJECT_STALE_DIAGNOSTIC_IDENTITY", lambda: parse_exercised_subject(raw))


def test_caller_alias_mutation_cannot_change_immutable_subject():
    value = _mapping()
    subject = exercised_subject_from_mapping(value)
    before = canonical_exercised_subject_bytes(subject)
    value["candidate_bundle"]["path"] = "mutated.json"
    value["runtime_identity"] = "f" * 64
    assert canonical_exercised_subject_bytes(subject) == before
    with pytest.raises(FrozenInstanceError):
        subject.runtime_identity = "f" * 64


@pytest.mark.parametrize(
    ("field", "code"),
    [
        ("candidate_bundle", "SUBJECT_BUNDLE_BINDING_MISMATCH"),
        (
            "runtime_projection",
            "SUBJECT_RUNTIME_PROJECTION_BINDING_MISMATCH",
        ),
        ("runtime_identity", "SUBJECT_RUNTIME_IDENTITY_MISMATCH"),
        (
            "materialization_projection",
            "SUBJECT_MATERIALIZATION_PROJECTION_BINDING_MISMATCH",
        ),
        (
            "materialization_logical_identity",
            "SUBJECT_MATERIALIZATION_LOGICAL_IDENTITY_MISMATCH",
        ),
    ],
)
def test_upstream_binding_mismatches_are_distinct(field, code):
    value = _mapping()
    expected = {
        "candidate_bundle": AuthenticatedFile(**value["candidate_bundle"]),
        "runtime_projection": AuthenticatedFile(**value["runtime_projection"]),
        "runtime_identity": value["runtime_identity"],
        "materialization_projection": AuthenticatedFile(
            **value["materialization_projection"],
        ),
        "materialization_logical_identity": value["materialization_logical_identity"],
    }
    if field in {"candidate_bundle", "runtime_projection", "materialization_projection"}:
        current = expected[field]
        expected[field] = AuthenticatedFile(current.path, current.size + 1, current.sha256)
    else:
        expected[field] = "f" * 64
    _assert_code(
        code,
        lambda: validate_exercised_subject(_subject(), **expected),
    )


def test_production_contract_has_no_forbidden_side_effect_surface():
    production = ROOT / "ctr_bringup/exercised_subject_identity.py"
    validator = ROOT / "ctr_bringup/runtime_candidate_validate_only.py"
    forbidden_calls = {
        "write", "write_bytes", "write_text", "chmod", "unlink", "remove",
        "rmdir", "mkdir", "makedirs", "rename", "replace", "Popen", "run",
        "system", "fork", "execve", "socket", "create_connection",
    }
    forbidden_imports = {
        "subprocess", "socket", "rclpy", "launch", "launch_ros",
    }
    for path in (production, validator):
        tree = ast.parse(path.read_bytes(), filename=str(path))
        imports = {
            alias.name.split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, (ast.Import, ast.ImportFrom))
            for alias in node.names
        }
        calls = {
            node.func.attr if isinstance(node.func, ast.Attribute) else node.func.id
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, (ast.Attribute, ast.Name))
        }
        assert not imports & forbidden_imports
        assert not calls & forbidden_calls
