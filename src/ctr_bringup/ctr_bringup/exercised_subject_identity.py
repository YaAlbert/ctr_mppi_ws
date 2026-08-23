"""Strict, acyclic identity contract for a Slice 7F exercised subject.

Dependency order (arrows mean "is an authenticated input to")::

    candidate bundle -----------+
    runtime projection ---------+--> exercised subject
    materialization projection -+          |
    runtime/materialization identities ----+
                                             v
                         inventory/report/closure/capsule/source identity
                                             |
                                             v
                                      root authority
                                             |
                                             v
                                      frozen tree identity

The exercised-subject record deliberately cannot bind itself, root authority,
downstream reports/closure, the frozen tree, a timestamp, or a candidate name.

Canonical record bytes are JSON encoded as UTF-8 with object keys sorted by
Unicode code point, compact ``,``/``:`` separators, ``ensure_ascii=False``, no
non-finite numbers, and no trailing newline.  The logical identity is SHA-256
over ``IDENTITY_DOMAIN + canonical_record_bytes``; it is intentionally distinct
from the physical file SHA-256 even when the retained file is canonical.
"""
from __future__ import annotations

import json
import re
import unicodedata
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path, PurePosixPath


SCHEMA_VERSION = "ctr-exercised-subject-2"
IDENTITY_ALGORITHM_ID = "sha256:ctr-exercised-subject-canonical-2"
IDENTITY_DOMAIN = b"ctr-exercised-subject-canonical-2\0"
DEFAULT_RECORD_PATH = "manifests/exercised_subject.json"
STALE_V2_DIAGNOSTIC_IDENTITY = (
    "b598a83add11e2c071c158229bdf4ee61edad502fa99a639802839de7547f0f2"
)

_HEX = re.compile(r"^[0-9a-f]{64}$")
_TOP_FIELDS = {
    "schema_version",
    "candidate_bundle",
    "runtime_projection",
    "runtime_identity",
    "materialization_projection",
    "materialization_logical_identity",
}
_BINDING_FIELDS = {"path", "size", "sha256"}
_ROOT_AUTHORITY_NAMES = {
    "root_authority", "root_authority_digest", "root_authority_identity",
}
_FROZEN_ROOT_NAMES = {
    "frozen_root", "frozen_root_digest", "frozen_root_identity",
    "physical_tree_identity",
}
_SELF_NAMES = {
    "identity", "subject_identity", "exercised_subject_identity",
    "self_digest", "physical_digest",
}
_DOWNSTREAM_NAMES = {
    "static_closure", "static_closure_digest", "report_source",
    "report_source_digest", "correction_report", "correction_report_digest",
    "root_authority_path", "frozen_tree_identity",
}
_FORBIDDEN_PATHS = {
    DEFAULT_RECORD_PATH,
    "manifests/root_authority.json",
    "manifests/static_closure.json",
    "manifests/report_source.json",
}


class ExercisedSubjectError(ValueError):
    """Stable public exercised-subject contract failure."""

    def __init__(self, code: str, field: str = "", detail: str = ""):
        self.code = str(code)
        self.field = str(field)
        self.detail = str(detail)
        suffix = ":".join(item for item in (self.field, self.detail) if item)
        super().__init__(self.code + (":" + suffix if suffix else ""))


def _fail(code: str, field: str = "", detail: str = ""):
    raise ExercisedSubjectError(code, field, detail)


def _safe_relative_path(value, field: str) -> str:
    if (type(value) is not str or not value or "\\" in value or "\x00" in value
            or any(unicodedata.category(char) in {"Cc", "Cs"} for char in value)
            or unicodedata.normalize("NFC", value) != value):
        _fail("SUBJECT_PATH", field)
    parsed = PurePosixPath(value)
    if (parsed.is_absolute() or parsed.as_posix() != value
            or any(part in {"", ".", ".."} for part in parsed.parts)
            or re.match(r"^[A-Za-z][A-Za-z0-9+.-]*:", value)):
        _fail("SUBJECT_PATH", field)
    if value == "manifests/root_authority.json":
        _fail("SUBJECT_ROOT_AUTHORITY_DEPENDENCY_CYCLE", field)
    if value in {"manifests/static_closure.json", "manifests/report_source.json"}:
        _fail("SUBJECT_DOWNSTREAM_DEPENDENCY_CYCLE", field)
    if value == DEFAULT_RECORD_PATH:
        _fail("SUBJECT_SELF_DEPENDENCY_CYCLE", field)
    return value


def _digest(value, field: str) -> str:
    if type(value) is not str or not _HEX.fullmatch(value):
        _fail("SUBJECT_DIGEST", field)
    if value == STALE_V2_DIAGNOSTIC_IDENTITY:
        _fail("SUBJECT_STALE_DIAGNOSTIC_IDENTITY", field)
    return value


def _size(value, field: str) -> int:
    if type(value) is not int or value <= 0:
        _fail("SUBJECT_SIZE", field)
    return value


@dataclass(frozen=True, slots=True)
class AuthenticatedFile:
    """Immutable physical-file binding used by the subject contract."""

    path: str
    size: int
    sha256: str

    def __post_init__(self):
        object.__setattr__(self, "path", _safe_relative_path(self.path, "path"))
        _size(self.size, "size")
        _digest(self.sha256, "sha256")

    def as_dict(self) -> dict:
        return {"path": self.path, "size": self.size, "sha256": self.sha256}


@dataclass(frozen=True, slots=True)
class ExercisedSubject:
    """Deeply immutable logical exercised-subject value."""

    schema_version: str
    candidate_bundle: AuthenticatedFile
    runtime_projection: AuthenticatedFile
    runtime_identity: str
    materialization_projection: AuthenticatedFile
    materialization_logical_identity: str

    def __post_init__(self):
        if type(self.schema_version) is not str or self.schema_version != SCHEMA_VERSION:
            _fail("SUBJECT_SCHEMA", "schema_version")
        for field in (
            "candidate_bundle", "runtime_projection", "materialization_projection",
        ):
            if type(getattr(self, field)) is not AuthenticatedFile:
                _fail("SUBJECT_TYPE", field)
        _digest(self.runtime_identity, "runtime_identity")
        _digest(
            self.materialization_logical_identity,
            "materialization_logical_identity",
        )
        paths = {
            self.candidate_bundle.path,
            self.runtime_projection.path,
            self.materialization_projection.path,
        }
        if len(paths) != 3:
            _fail("SUBJECT_PATH_ALIAS")

    def as_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "candidate_bundle": self.candidate_bundle.as_dict(),
            "runtime_projection": self.runtime_projection.as_dict(),
            "runtime_identity": self.runtime_identity,
            "materialization_projection": self.materialization_projection.as_dict(),
            "materialization_logical_identity": self.materialization_logical_identity,
        }


def _reject_forbidden_dependencies(value: dict) -> None:
    keys = set(value)
    if keys & _ROOT_AUTHORITY_NAMES:
        _fail("SUBJECT_ROOT_AUTHORITY_DEPENDENCY_CYCLE", sorted(keys & _ROOT_AUTHORITY_NAMES)[0])
    if keys & _FROZEN_ROOT_NAMES:
        _fail("SUBJECT_FROZEN_ROOT_DEPENDENCY_CYCLE", sorted(keys & _FROZEN_ROOT_NAMES)[0])
    if keys & _SELF_NAMES:
        _fail("SUBJECT_SELF_DEPENDENCY_CYCLE", sorted(keys & _SELF_NAMES)[0])
    if keys & _DOWNSTREAM_NAMES:
        _fail("SUBJECT_DOWNSTREAM_DEPENDENCY_CYCLE", sorted(keys & _DOWNSTREAM_NAMES)[0])


def _binding(value, field: str) -> AuthenticatedFile:
    if type(value) is not dict:
        _fail("SUBJECT_TYPE", field)
    if set(value) != _BINDING_FIELDS:
        _fail("SUBJECT_FIELDS", field)
    return AuthenticatedFile(
        _safe_relative_path(value["path"], field + ".path"),
        _size(value["size"], field + ".size"),
        _digest(value["sha256"], field + ".sha256"),
    )


def exercised_subject_from_mapping(value) -> ExercisedSubject:
    """Strictly detach and parse a mapping into an immutable subject value."""

    if type(value) is not dict:
        _fail("SUBJECT_TYPE", "root")
    _reject_forbidden_dependencies(value)
    if "schema_version" in value and value["schema_version"] != SCHEMA_VERSION:
        _fail("SUBJECT_SCHEMA", "schema_version")
    if set(value) != _TOP_FIELDS:
        _fail("SUBJECT_FIELDS", "root")
    if type(value["runtime_identity"]) is not str:
        _fail("SUBJECT_TYPE", "runtime_identity")
    if type(value["materialization_logical_identity"]) is not str:
        _fail("SUBJECT_TYPE", "materialization_logical_identity")
    return ExercisedSubject(
        SCHEMA_VERSION,
        _binding(value["candidate_bundle"], "candidate_bundle"),
        _binding(value["runtime_projection"], "runtime_projection"),
        _digest(value["runtime_identity"], "runtime_identity"),
        _binding(value["materialization_projection"], "materialization_projection"),
        _digest(
            value["materialization_logical_identity"],
            "materialization_logical_identity",
        ),
    )


def _parse_json(raw: bytes) -> dict:
    if type(raw) is not bytes:
        _fail("SUBJECT_TYPE", "raw")
    if STALE_V2_DIAGNOSTIC_IDENTITY.encode("ascii") in raw:
        _fail("SUBJECT_STALE_DIAGNOSTIC_IDENTITY")
    try:
        def pairs_hook(pairs):
            result = {}
            for key, value in pairs:
                if key in result:
                    _fail("SUBJECT_JSON_DUPLICATE_KEY", str(key))
                result[key] = value
            return result

        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=pairs_hook,
            parse_constant=lambda token: _fail("SUBJECT_JSON_INVALID", token),
        )
    except ExercisedSubjectError:
        raise
    except (UnicodeError, ValueError, TypeError) as error:
        raise ExercisedSubjectError("SUBJECT_JSON_INVALID") from error
    return value


def canonical_exercised_subject_bytes(subject: ExercisedSubject) -> bytes:
    """Return UTF-8, sorted-key, compact JSON with no trailing newline."""

    if type(subject) is not ExercisedSubject:
        _fail("SUBJECT_TYPE", "subject")
    return json.dumps(
        subject.as_dict(),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def parse_exercised_subject(raw: bytes) -> ExercisedSubject:
    """Parse strict canonical bytes and reject all noncanonical encodings."""

    subject = exercised_subject_from_mapping(_parse_json(raw))
    if canonical_exercised_subject_bytes(subject) != raw:
        _fail("SUBJECT_NONCANONICAL")
    return subject


def load_exercised_subject(path) -> ExercisedSubject:
    """Load a subject file; candidate validation should use descriptor-owned bytes."""

    return parse_exercised_subject(Path(path).read_bytes())


def exercised_subject_identity(subject: ExercisedSubject) -> str:
    """Compute the domain-separated logical identity of a canonical subject."""

    return sha256(IDENTITY_DOMAIN + canonical_exercised_subject_bytes(subject)).hexdigest()


def validate_exercised_subject(
    subject: ExercisedSubject,
    *,
    candidate_bundle: AuthenticatedFile,
    runtime_projection: AuthenticatedFile,
    runtime_identity: str,
    materialization_projection: AuthenticatedFile,
    materialization_logical_identity: str,
) -> str:
    """Validate every subject binding and return its canonical logical identity."""

    if type(subject) is not ExercisedSubject:
        _fail("SUBJECT_TYPE", "subject")
    expected_files = (
        ("candidate_bundle", candidate_bundle, "SUBJECT_BUNDLE_BINDING_MISMATCH"),
        (
            "runtime_projection", runtime_projection,
            "SUBJECT_RUNTIME_PROJECTION_BINDING_MISMATCH",
        ),
        (
            "materialization_projection", materialization_projection,
            "SUBJECT_MATERIALIZATION_PROJECTION_BINDING_MISMATCH",
        ),
    )
    for field, expected, code in expected_files:
        if type(expected) is not AuthenticatedFile:
            _fail("SUBJECT_TYPE", "expected." + field)
        if getattr(subject, field) != expected:
            _fail(code, field)
    expected_runtime = _digest(runtime_identity, "expected.runtime_identity")
    if subject.runtime_identity != expected_runtime:
        _fail("SUBJECT_RUNTIME_IDENTITY_MISMATCH", "runtime_identity")
    expected_materialization = _digest(
        materialization_logical_identity,
        "expected.materialization_logical_identity",
    )
    if subject.materialization_logical_identity != expected_materialization:
        _fail(
            "SUBJECT_MATERIALIZATION_LOGICAL_IDENTITY_MISMATCH",
            "materialization_logical_identity",
        )
    return exercised_subject_identity(subject)


def make_exercised_subject(
    *,
    candidate_bundle: AuthenticatedFile,
    runtime_projection: AuthenticatedFile,
    runtime_identity: str,
    materialization_projection: AuthenticatedFile,
    materialization_logical_identity: str,
) -> ExercisedSubject:
    """Repository-owned pure producer API for future candidate staging."""

    return ExercisedSubject(
        SCHEMA_VERSION,
        candidate_bundle,
        runtime_projection,
        _digest(runtime_identity, "runtime_identity"),
        materialization_projection,
        _digest(
            materialization_logical_identity,
            "materialization_logical_identity",
        ),
    )


__all__ = [
    "AuthenticatedFile",
    "DEFAULT_RECORD_PATH",
    "ExercisedSubject",
    "ExercisedSubjectError",
    "IDENTITY_ALGORITHM_ID",
    "IDENTITY_DOMAIN",
    "SCHEMA_VERSION",
    "STALE_V2_DIAGNOSTIC_IDENTITY",
    "canonical_exercised_subject_bytes",
    "exercised_subject_from_mapping",
    "exercised_subject_identity",
    "load_exercised_subject",
    "make_exercised_subject",
    "parse_exercised_subject",
    "validate_exercised_subject",
]
