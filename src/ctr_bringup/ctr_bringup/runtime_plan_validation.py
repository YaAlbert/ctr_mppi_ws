"""ROS-independent validation for authenticated production runtime plans.

The schemas in this module deliberately describe only prospective runtime
content.  They do not authorize execution and they do not discover project
dependencies.  A caller supplies a dependency graph; this module validates
that graph against an authenticated runtime projection.

Runtime projection schema ``ctr-runtime-projection-1``::

    {
      "schema_version": "ctr-runtime-projection-1",
      "members": [
        {
          "path": "relative/posix/path",
          "size_bytes": 123,
          "sha256": "<lowercase sha256>",
          "mode": "0644",
          "role": "python_module"
        }
      ]
    }

Members are ordered by their NFC-normalized path.  Canonical projection bytes
are UTF-8 JSON with sorted object keys, compact separators, Unicode preserved,
and no trailing newline.  The logical identity is the SHA-256 of exactly those
canonical bytes.  The projection has no timestamp, host path, or self-digest.

Runtime plan schema ``ctr-runtime-plan-2`` contains the following top-level
fields: ``schema_version``, ``mode``, ``production_runtime_identity``,
``runtime_root_role``, ``prospective_argv``,
``project_owned_argv_indices``, ``argv_bindings``, ``external_commands``,
``argv_classifications``, ``prospective_environment``,
``external_dependencies``, and ``policy``.
An optional ``diagnostic_lineage`` object may contain non-operative identities.
Every argv position has exactly one explicit semantic classification.  Literal
classifications use deliberately narrow grammars and cannot carry paths.
Declared external dependencies must be observed in the supplied dependency
graph, or used by a classified external command when no graph allowlist is
supplied.  This validates declaration and usage only; it does not establish
installed-system availability or authenticate external dependency bytes.

Physical runtime authentication requires an immutable tree: the root,
traversed directories, and members must have no write bits.  Runtime members
are opened relative to trusted directory descriptors with no-follow semantics,
hashed through their open descriptors, and checked for identity or metadata
changes before and after reading.

The plan shape is::

    {
      "schema_version": "ctr-runtime-plan-2",
      "mode": "production | offline | test_only",
      "production_runtime_identity": "<lowercase sha256>",
      "runtime_root_role": "AUTHENTICATED_RUNTIME_ROOT",
      "prospective_argv": ["external-command", "member/path"],
      "project_owned_argv_indices": [1],
      "argv_bindings": [{"argv_index": 1, "member_path": "member/path"}],
      "external_commands": [
        {"argv_index": 0, "command": "external-command",
         "dependency": "external-package"}
      ],
      "argv_classifications": [
        {"argv_index": 0, "kind": "external_command",
         "value": "external-command", "dependency": "external-package"},
        {"argv_index": 1, "kind": "project_member",
         "value": "member/path", "member_path": "member/path"}
      ],
      "prospective_environment": {"NAME": "prospective metadata"},
      "external_dependencies": ["external-package"],
      "policy": {
        "validate_only": true,
        "allow_full_launch": false,
        "launchable": false,
        "execution_authorized": false
      },
      "diagnostic_lineage": {"identities": ["<lowercase sha256>"]}
    }

This module uses only the Python standard library.  All filesystem operations
are read-only, and no function reads process environment implicitly.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import wraps
from hashlib import sha256
from pathlib import Path, PurePosixPath
from types import MappingProxyType
from typing import Any, Iterable, Mapping, Sequence

import json
import os
import re
import stat
import unicodedata
import errno
import threading
import weakref
from contextlib import contextmanager


PROJECTION_SCHEMA_VERSION = "ctr-runtime-projection-1"
PLAN_SCHEMA_VERSION = "ctr-runtime-plan-2"
AUTHENTICATED_RUNTIME_ROOT = "AUTHENTICATED_RUNTIME_ROOT"

RUNTIME_MODES = frozenset({"production", "offline", "test_only"})
RUNTIME_MEMBER_ROLES = frozenset(
    {
        "configuration",
        "executable_entrypoint",
        "interface_definition",
        "launch_file",
        "package_data",
        "package_manifest",
        "package_setup",
        "python_module",
        "resource_index",
        "runtime_resource",
    }
)
FORBIDDEN_RUNTIME_PATH_SEGMENTS = frozenset(
    {
        "candidate_tooling",
        "correction_tooling",
        "evidence_tooling",
        "focused_evidence",
        "focused_raw",
        "tooling",
    }
)

_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
_MODE_PATTERN = re.compile(r"^0[0-7]{3}$")
_DRIVE_PATTERN = re.compile(r"^[A-Za-z]:")
_URI_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9+.-]*:")
_EXTERNAL_DEPENDENCY_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.+:-]*$")
_ENVIRONMENT_NAME_PATTERN = re.compile(r"^[A-Z_][A-Z0-9_]*$")
_COMMAND_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.+-]*$")
_FLAG_PATTERN = re.compile(r"^(?:--[a-z][a-z0-9-]*|-[A-Za-z])$")
_IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_.-]*$")
_INTEGER_PATTERN = re.compile(r"^-?(?:0|[1-9][0-9]*)$")
_NUMERIC_PATTERN = re.compile(
    r"^-?(?:0|[1-9][0-9]*)(?:\.[0-9]+)(?:[eE][+-]?[0-9]+)?$"
)
_ROS_NAME_PATTERN = re.compile(r"^/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*$")
_ASSIGNMENT_NAME_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_.-]*$")
_FILE_LIKE_SUFFIX_PATTERN = re.compile(
    r"\.(?:bag|cfg|dae|db3|ini|json|launch|mesh|obj|py|rviz|stl|txt|urdf|xacro|ya?ml|xml)$",
    re.IGNORECASE,
)

_PROJECTION_FIELDS = frozenset({"schema_version", "members"})
_PROJECTION_MEMBER_FIELDS = frozenset({"path", "size_bytes", "sha256", "mode", "role"})
_FORBIDDEN_PROJECTION_METADATA = frozenset(
    {
        "absolute_path",
        "created_at",
        "created_at_utc",
        "digest",
        "host",
        "host_path",
        "identity",
        "projection_digest",
        "projection_sha256",
        "self_digest",
        "timestamp",
    }
)

_PLAN_FIELDS = frozenset(
    {
        "schema_version",
        "mode",
        "production_runtime_identity",
        "runtime_root_role",
        "prospective_argv",
        "project_owned_argv_indices",
        "argv_bindings",
        "external_commands",
        "argv_classifications",
        "prospective_environment",
        "external_dependencies",
        "policy",
        "diagnostic_lineage",
    }
)
_PLAN_REQUIRED_FIELDS = _PLAN_FIELDS - {"diagnostic_lineage"}
_PLAN_BINDING_FIELDS = frozenset({"argv_index", "member_path"})
_PLAN_EXTERNAL_COMMAND_FIELDS = frozenset({"argv_index", "command", "dependency"})
_ARGV_CLASSIFICATION_COMMON_FIELDS = frozenset({"argv_index", "kind", "value"})
_ARGV_CLASSIFICATION_FIELDS = {
    "project_member": _ARGV_CLASSIFICATION_COMMON_FIELDS | {"member_path"},
    "external_command": _ARGV_CLASSIFICATION_COMMON_FIELDS | {"dependency"},
    "flag": _ARGV_CLASSIFICATION_COMMON_FIELDS,
    "identifier": _ARGV_CLASSIFICATION_COMMON_FIELDS,
    "integer": _ARGV_CLASSIFICATION_COMMON_FIELDS,
    "numeric": _ARGV_CLASSIFICATION_COMMON_FIELDS,
    "ros_name": _ARGV_CLASSIFICATION_COMMON_FIELDS,
    "assignment": _ARGV_CLASSIFICATION_COMMON_FIELDS,
}
_PLAN_POLICY_FIELDS = frozenset(
    {"validate_only", "allow_full_launch", "launchable", "execution_authorized"}
)
_DIAGNOSTIC_LINEAGE_FIELDS = frozenset({"identities"})
_SIX_PLAN_ROLES = (
    "production_root",
    "production_duplicate",
    "offline_root",
    "offline_duplicate",
    "test_only_root",
    "test_only_duplicate",
)


class RuntimeValidationError(ValueError):
    """Validation failure with a stable code and structured location."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        field: str | None = None,
        path: str | None = None,
    ) -> None:
        self.code = code
        self.field = field
        self.path = path
        location = field or path
        suffix = f" [{location}]" if location else ""
        super().__init__(f"{code}{suffix}: {message}")


def _exact_string(value: Any, *, code: str, field: str, nonempty: bool = True) -> str:
    if type(value) is not str or (nonempty and not value):
        raise _error(code, "value must be an exact built-in string", field=field)
    try:
        value.encode("utf-8", errors="strict")
    except UnicodeEncodeError as exc:
        raise _error("JSON_UNICODE_ENCODING", "text contains a non-UTF-8 Unicode scalar", field=field) from exc
    return value


@dataclass(frozen=True, slots=True)
class RuntimeMember:
    """One canonical member of a production runtime projection."""

    path: str
    size_bytes: int
    sha256: str
    mode: str
    role: str

    def __post_init__(self) -> None:
        _post_runtime_member(self)


@dataclass(frozen=True, slots=True)
class RuntimeProjection:
    """Immutable, canonical logical production-runtime projection."""

    schema_version: str
    members: tuple[RuntimeMember, ...]

    def __post_init__(self) -> None:
        _post_runtime_projection(self)

    @property
    def member_paths(self) -> tuple[str, ...]:
        return tuple(member.path for member in self.members)


@dataclass(frozen=True, slots=True)
class RuntimeIssue:
    """One deterministic physical or graph reconciliation issue."""

    code: str
    field: str | None = None
    path: str | None = None
    expected: str | None = None
    observed: str | None = None

    def __post_init__(self) -> None:
        _post_runtime_issue(self)


@dataclass(frozen=True, slots=True)
class RuntimeProjectionReconciliation:
    """Read-only physical reconciliation; validity is ``not issues``."""

    declared_count: int
    physical_regular_file_count: int
    matched_count: int
    issues: tuple[RuntimeIssue, ...]
    authoritative: bool = False

    def __post_init__(self) -> None:
        _post_projection_reconciliation(self)


@dataclass(frozen=True, slots=True)
class RuntimeDependency:
    """A supplied dependency edge from a project runtime member."""

    source: str
    target: str
    dependency_type: str = "project"
    resolved: bool = True

    def __post_init__(self) -> None:
        _post_runtime_dependency(self)


@dataclass(frozen=True, slots=True)
class RuntimeDependencyClosure:
    """Immutable result of validating a caller-supplied dependency graph."""

    entrypoints: tuple[str, ...]
    project_nodes: tuple[str, ...]
    reachable_members: tuple[str, ...]
    external_dependencies: tuple[str, ...]
    issues: tuple[RuntimeIssue, ...]

    def __post_init__(self) -> None:
        _post_dependency_closure(self)


@dataclass(frozen=True, slots=True)
class RuntimeArgvBinding:
    argv_index: int
    member_path: str

    def __post_init__(self) -> None:
        _post_argv_binding(self)


@dataclass(frozen=True, slots=True)
class RuntimeExternalCommand:
    argv_index: int
    command: str
    dependency: str

    def __post_init__(self) -> None:
        _post_external_command(self)


@dataclass(frozen=True, slots=True)
class RuntimeArgvClassification:
    """One complete, value-bound semantic classification of an argv token."""

    argv_index: int
    kind: str
    value: str
    member_path: str | None = None
    dependency: str | None = None

    def __post_init__(self) -> None:
        _post_argv_classification(self)


@dataclass(frozen=True, slots=True)
class RuntimePlanPolicy:
    validate_only: bool
    allow_full_launch: bool
    launchable: bool
    execution_authorized: bool

    def __post_init__(self) -> None:
        _post_plan_policy(self)


@dataclass(frozen=True, slots=True)
class RuntimePlan:
    """Immutable prospective runtime plan; it conveys no execution authority."""

    schema_version: str
    mode: str
    production_runtime_identity: str
    runtime_root_role: str
    prospective_argv: tuple[str, ...]
    project_owned_argv_indices: tuple[int, ...]
    argv_bindings: tuple[RuntimeArgvBinding, ...]
    external_commands: tuple[RuntimeExternalCommand, ...]
    argv_classifications: tuple[RuntimeArgvClassification, ...]
    prospective_environment: tuple[tuple[str, str], ...]
    external_dependencies: tuple[str, ...]
    policy: RuntimePlanPolicy
    diagnostic_lineage: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        _post_runtime_plan(self)

    @property
    def environment(self) -> Mapping[str, str]:
        return MappingProxyType(dict(self.prospective_environment))


@dataclass(frozen=True, slots=True)
class RuntimeReconciliation:
    """Successful deterministic reconciliation of the required six plans."""

    runtime_identity: str
    roles: tuple[str, ...]
    plan_sha256: tuple[tuple[str, str], ...]
    modes: tuple[tuple[str, str], ...]

    def __post_init__(self) -> None:
        _post_runtime_reconciliation(self)


@dataclass(frozen=True, slots=True)
class AuthenticatedMember:
    """Immutable metadata for one descriptor-authenticated member."""

    path: str
    size_bytes: int
    sha256: str
    mode: str
    device: int
    inode: int

    def __post_init__(self) -> None:
        _exact_string(self.path, code="PATH_TYPE", field="path")
        _nonnegative_integer(self.size_bytes, field="size_bytes")
        _sha256_value(self.sha256, field="sha256")
        _exact_string(self.mode, code="MODE_TYPE", field="mode")
        if not _MODE_PATTERN.fullmatch(self.mode):
            raise _error("MODE_FORMAT", "invalid mode", field="mode")
        _nonnegative_integer(self.device, field="device")
        _nonnegative_integer(self.inode, field="inode")


class _SnapshotAuthorityState:
    __slots__ = ("lock", "root_fd", "member_fds", "member_parents", "member_stats", "closed", "cleaned")
    def __init__(self, root_fd, member_fds, member_parents, member_stats):
        self.lock = threading.RLock(); self.root_fd = root_fd
        self.member_fds = dict(member_fds); self.member_parents = dict(member_parents)
        self.member_stats = dict(member_stats); self.closed = False; self.cleaned = False


class AuthenticatedRuntimeSnapshot:
    """Descriptor-backed runtime authority valid only while open."""

    __slots__ = (
        "_root_path", "_projection", "_state", "_directory_stats", "_issues", "__weakref__",
    )

    _CONSTRUCTION_TOKEN = object()
    _REGISTRY_LOCK = threading.RLock()
    _REGISTRY: dict[int, tuple[weakref.ReferenceType, object]] = {}

    def __init_subclass__(cls, **kwargs):
        raise TypeError("AuthenticatedRuntimeSnapshot cannot be subclassed")

    def __setattr__(self, name, value):
        if name in {"_projection", "_root_path", "_state", "_directory_stats", "_issues"} and hasattr(self, name):
            raise AttributeError("snapshot metadata is immutable")
        object.__setattr__(self, name, value)

    def __init__(self, root_path, projection, state, directory_stats, issues, *, _token=None):
        if _token is not AuthenticatedRuntimeSnapshot._CONSTRUCTION_TOKEN:
            raise _error("SNAPSHOT_CONSTRUCTION_FORBIDDEN", "use open_authenticated_runtime_snapshot")
        self._root_path = str(root_path)
        self._projection = projection
        self._state = state
        self._directory_stats = dict(directory_stats)
        self._issues = tuple(issues)

    def _provenance(self):
        with AuthenticatedRuntimeSnapshot._REGISTRY_LOCK:
            entry = AuthenticatedRuntimeSnapshot._REGISTRY.get(id(self))
            if entry is None or entry[0]() is not self:
                raise _error("SNAPSHOT_PROVENANCE_INVALID", "snapshot was not created by the factory")
            return entry[2]

    def __del__(self):
        try:
            self._provenance()
            self._discard_unregistered()
        except Exception:
            pass

    @property
    def projection(self):
        self._provenance()
        return self._projection

    @property
    def root_path(self):
        self._provenance()
        return self._root_path

    @property
    def issues(self):
        self._provenance()
        return self._issues

    @property
    def authoritative(self):
        self._provenance()
        state = self._provenance()
        return not state.closed and not self._issues

    @property
    def closed(self):
        return self._provenance().closed

    @property
    def member_metadata(self):
        state = self._provenance()
        return tuple(
            AuthenticatedMember(member.path, info.st_size, member.sha256, member.mode,
                                info.st_dev, info.st_ino)
            for member, info in ((m, state.member_stats[m.path]) for m in self._projection.members)
            if member.path in state.member_stats
        )

    def _ensure_open(self):
        state = self._provenance()
        with state.lock:
            if state.closed:
                raise _error("SNAPSHOT_CLOSED", "authenticated runtime snapshot is closed")
        if self._issues:
            raise _error("SNAPSHOT_NOT_AUTHENTICATED", "snapshot does not hold authenticated authority")

    @contextmanager
    def _borrow(self):
        state = self._provenance()
        state.lock.acquire()
        try:
            if state.closed:
                raise _error("SNAPSHOT_CLOSED", "authenticated runtime snapshot is closed")
            if self._issues:
                raise _error("SNAPSHOT_NOT_AUTHENTICATED", "snapshot does not hold authenticated authority")
            yield state
        finally:
            state.lock.release()

    def _fd_for(self, path: str) -> int:
        state = self._provenance()
        state.lock.acquire()
        if state.closed:
            state.lock.release(); raise _error("SNAPSHOT_CLOSED", "authenticated runtime snapshot is closed")
        try:
            return state.member_fds[path]
        except KeyError as exc:
            state.lock.release()
            raise _error("SNAPSHOT_MEMBER_ABSENT", "member is not in the authenticated snapshot", path=path) from exc

    def read_member_bytes(self, path: str) -> bytes:
        state = self._provenance(); state.lock.acquire()
        if state.closed: state.lock.release(); raise _error("SNAPSHOT_CLOSED", "authenticated runtime snapshot is closed")
        fd = state.member_fds.get(path)
        if fd is None: state.lock.release(); raise _error("SNAPSHOT_MEMBER_ABSENT", "member absent", path=path)
        duplicate = None
        try:
            duplicate = os.dup(fd)
            os.set_inheritable(duplicate, False)
            before = os.fstat(fd)
            digest = sha256()
            chunks = []
            os.lseek(duplicate, 0, os.SEEK_SET)
            while True:
                block = os.read(duplicate, 1024 * 1024)
                if not block:
                    break
                digest.update(block)
                chunks.append(block)
            after = os.fstat(fd)
            member = next(item for item in self._projection.members if item.path == path)
            if _stat_stability(before) != _stat_stability(after) or digest.hexdigest() != member.sha256:
                raise _error("PHYSICAL_CHANGED_DURING_READ", "descriptor-backed member changed", path=path)
            return b"".join(chunks)
        except OSError as exc:
            raise _error("PHYSICAL_READ_ERROR", str(exc), path=path) from exc
        finally:
            if duplicate is not None: os.close(duplicate)
            state.lock.release()

    def duplicate_member_fd(self, path: str) -> int:
        state = self._provenance(); state.lock.acquire()
        if state.closed: state.lock.release(); raise _error("SNAPSHOT_CLOSED", "authenticated runtime snapshot is closed")
        fd = state.member_fds.get(path)
        if fd is None: state.lock.release(); raise _error("SNAPSHOT_MEMBER_ABSENT", "member absent", path=path)
        try:
            duplicate = os.dup(fd)
            os.set_inheritable(duplicate, False)
            return duplicate
        except OSError as exc:
            code = "DESCRIPTOR_EXHAUSTED" if exc.errno in {errno.EMFILE, errno.ENFILE} else "SNAPSHOT_DUPLICATE_ERROR"
            raise _error(code, str(exc), path=path) from exc
        finally:
            state.lock.release()

    def verify_current_paths(self) -> tuple[RuntimeIssue, ...]:
        with self._borrow() as state:
            issues = []
            for member in self._projection.members:
                fd = state.member_fds.get(member.path)
                parent, name = state.member_parents.get(member.path, (None, None))
                if fd is None or parent is None:
                    issues.append(RuntimeIssue("SNAPSHOT_MEMBER_ABSENT", path=member.path)); continue
                try:
                    current = _entry_stat(parent, name); observed = os.fstat(fd)
                except OSError as exc:
                    issues.append(RuntimeIssue("PHYSICAL_STAT_ERROR", path=member.path, observed=str(exc))); continue
                expected = state.member_stats[member.path]
                if current is None or _stat_identity(current) != _stat_identity(expected):
                    issues.append(RuntimeIssue("PHYSICAL_INODE_CHANGED", path=member.path))
                elif _stat_stability(current) != _stat_stability(expected):
                    issues.append(RuntimeIssue("PHYSICAL_CHANGED_DURING_READ", path=member.path))
                if _stat_stability(observed) != _stat_stability(expected):
                    issues.append(RuntimeIssue("PHYSICAL_CHANGED_DURING_READ", path=member.path))
            return tuple(sorted(set(issues), key=_issue_sort_key))

    def close(self):
        state = self._provenance()
        with state.lock:
            if state.closed: return
            state.closed = True; state.cleaned = True
            descriptors = list(state.member_fds.values()) + [p[0] for p in state.member_parents.values()]
            if state.root_fd is not None: descriptors.append(state.root_fd)
            state.member_fds.clear(); state.member_parents.clear(); state.root_fd = None
            for descriptor in set(descriptors):
                try: os.close(descriptor)
                except OSError: pass

    def _discard_unregistered(self):
        return

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        return False

    def __copy__(self):
        raise _error("SNAPSHOT_COPY_FORBIDDEN", "authenticated snapshots cannot be copied")

    def __deepcopy__(self, memo):
        raise _error("SNAPSHOT_COPY_FORBIDDEN", "authenticated snapshots cannot be copied")

    def __reduce__(self):
        raise _error("SNAPSHOT_PICKLE_FORBIDDEN", "authenticated snapshots cannot be serialized")

    def __reduce_ex__(self, protocol):
        raise _error("SNAPSHOT_PICKLE_FORBIDDEN", "authenticated snapshots cannot be serialized")

def open_authenticated_runtime_snapshot(
    projection: JsonSource | Mapping[str, Any] | RuntimeProjection,
    runtime_root: str | Path,
    *,
    complete_inventory: bool = True,
    require_unique_physical_files: bool = True,
) -> AuthenticatedRuntimeSnapshot:
    """Open and retain a descriptor-backed authenticated runtime snapshot."""
    if type(complete_inventory) is not bool:
        raise _error("RECONCILE_COMPLETE_INVENTORY_TYPE", "must be bool", field="complete_inventory")
    if type(require_unique_physical_files) is not bool:
        raise _error("RECONCILE_UNIQUE_FILES_TYPE", "must be bool", field="require_unique_physical_files")
    parsed = load_runtime_projection(projection)
    root = Path(runtime_root)
    issues: list[RuntimeIssue] = []
    member_fds: dict[str, int] = {}
    member_parents: dict[str, tuple[int, str]] = {}
    member_stats: dict[str, os.stat_result] = {}
    directory_stats: dict[str, os.stat_result] = {}
    root_fd = None
    if not _descriptor_primitives_available():
        issues.append(RuntimeIssue("PHYSICAL_NOFOLLOW_UNAVAILABLE", path=str(root)))
    try:
        if not issues:
            root_fd = os.open(root, _descriptor_flags(directory=True))
            os.set_inheritable(root_fd, False)
            root_stat = os.fstat(root_fd)
            if not stat.S_ISDIR(root_stat.st_mode):
                issues.append(RuntimeIssue("RUNTIME_ROOT_NOT_DIRECTORY", path=str(root)))
            if _writable(root_stat):
                issues.append(RuntimeIssue("RUNTIME_ROOT_WRITABLE", path=str(root)))
            directory_stats[""] = root_stat
        seen: dict[tuple[int, int], str] = {}
        for member in parsed.members:
            if issues and root_fd is None:
                break
            current_fd = os.dup(root_fd)
            os.set_inheritable(current_fd, False)
            parts = member.path.split("/")
            name = parts[-1]
            try:
                for part in parts[:-1]:
                    child = os.open(part, _descriptor_flags(directory=True), dir_fd=current_fd)
                    os.set_inheritable(child, False)
                    os.close(current_fd)
                    current_fd = child
                    ds = os.fstat(current_fd)
                    if _writable(ds):
                        issues.append(RuntimeIssue("PHYSICAL_DIRECTORY_WRITABLE", path=member.path))
                fd = os.open(name, _descriptor_flags(directory=False), dir_fd=current_fd)
                os.set_inheritable(fd, False)
                before = os.fstat(fd)
                if not stat.S_ISREG(before.st_mode):
                    issues.append(RuntimeIssue("PHYSICAL_MEMBER_NOT_REGULAR", path=member.path))
                    os.close(fd); continue
                if _writable(before):
                    issues.append(RuntimeIssue("PHYSICAL_FILE_WRITABLE", path=member.path))
                if require_unique_physical_files:
                    key = (before.st_dev, before.st_ino)
                    if key in seen:
                        issues.append(RuntimeIssue("PHYSICAL_HARDLINK_ALIAS", path=member.path, observed=seen[key]))
                    seen[key] = member.path
                digest = sha256()
                chunks = []
                while True:
                    block = os.read(fd, 1024 * 1024)
                    if not block: break
                    digest.update(block); chunks.append(block)
                after = os.fstat(fd)
                if _stat_stability(before) != _stat_stability(after) or digest.hexdigest() != member.sha256 or before.st_size != member.size_bytes:
                    issues.append(RuntimeIssue("PHYSICAL_CHANGED_DURING_READ", path=member.path))
                else:
                    member_fds[member.path] = fd
                    member_parents[member.path] = (current_fd, name)
                    member_stats[member.path] = after
                    current_fd = None
                    continue
                os.close(fd)
            except OSError as exc:
                code = "DESCRIPTOR_EXHAUSTED" if exc.errno in {errno.EMFILE, errno.ENFILE} else "PHYSICAL_OPEN_ERROR"
                issues.append(RuntimeIssue(code, path=member.path, observed=str(exc)))
            finally:
                if current_fd is not None:
                    os.close(current_fd)
        if complete_inventory and root_fd is not None:
            declared = {m.path for m in parsed.members}; physical: set[str] = set()
            _inventory_descriptor(root_fd, PurePosixPath(), declared, physical, issues)
        if root_fd is not None:
            final_root = os.fstat(root_fd)
            if _stat_stability(final_root) != _stat_stability(directory_stats[""]):
                issues.append(RuntimeIssue("PHYSICAL_CHANGED_DURING_READ", path=str(root)))
        for path, fd in member_fds.items():
            current = os.fstat(fd)
            if _stat_stability(current) != _stat_stability(member_stats[path]):
                issues.append(RuntimeIssue("PHYSICAL_CHANGED_DURING_READ", path=path))
    except OSError as exc:
        issues.append(RuntimeIssue("PHYSICAL_STAT_ERROR", path=str(root), observed=str(exc)))
    authoritative = not issues and len(member_fds) == len(parsed.members)
    if issues or not authoritative:
        descriptors = list(member_fds.values()) + [p[0] for p in member_parents.values()]
        if root_fd is not None: descriptors.append(root_fd)
        for fd in set(descriptors):
            try: os.close(fd)
            except OSError: pass
        first = sorted(set(issues), key=_issue_sort_key)[0] if issues else RuntimeIssue("SNAPSHOT_AUTHENTICATION_FAILED", path=str(root))
        raise _error(first.code, "runtime snapshot authentication failed", field=first.field, path=first.path)
    state = _SnapshotAuthorityState(root_fd, member_fds, member_parents, member_stats) if authoritative else None
    snapshot = AuthenticatedRuntimeSnapshot(root, parsed, state, directory_stats, tuple(sorted(set(issues), key=_issue_sort_key)), _token=AuthenticatedRuntimeSnapshot._CONSTRUCTION_TOKEN)
    if authoritative:
        token = object()
        with AuthenticatedRuntimeSnapshot._REGISTRY_LOCK:
            ident = id(snapshot)
            def cleanup(ref, ident=ident):
                with AuthenticatedRuntimeSnapshot._REGISTRY_LOCK:
                    current = AuthenticatedRuntimeSnapshot._REGISTRY.get(ident)
                    if current is not None and current[0] is ref:
                        AuthenticatedRuntimeSnapshot._REGISTRY.pop(ident, None)
                        state = current[2]
                        with state.lock:
                            if not state.cleaned:
                                state.closed = True; state.cleaned = True
                                descriptors = list(state.member_fds.values()) + [p[0] for p in state.member_parents.values()]
                                if state.root_fd is not None: descriptors.append(state.root_fd)
                                state.member_fds.clear(); state.member_parents.clear(); state.root_fd = None
                                for fd in set(descriptors):
                                    try: os.close(fd)
                                    except OSError: pass
            ref = weakref.ref(snapshot, cleanup)
            AuthenticatedRuntimeSnapshot._REGISTRY[ident] = (ref, token, state)
    return snapshot

JsonSource = bytes | bytearray | memoryview | str | Path


def _error(
    code: str,
    message: str,
    *,
    field: str | None = None,
    path: str | None = None,
) -> RuntimeValidationError:
    return RuntimeValidationError(code, message, field=field, path=path)


def _stable_public_errors(function):
    @wraps(function)
    def guarded(*args, **kwargs):
        try:
            return function(*args, **kwargs)
        except RuntimeValidationError:
            raise
        except (TypeError, UnicodeError, KeyError, AttributeError, ValueError, OverflowError, OSError) as exc:
            raise _error(
                "PUBLIC_INPUT_INVALID",
                f"malformed public API input: {type(exc).__name__}",
                field=function.__name__,
            ) from exc

    return guarded


def _tuple_value(value: Any, *, code: str, field: str) -> tuple[Any, ...]:
    if isinstance(value, (str, bytes, bytearray, memoryview, Mapping)):
        raise _error(code, "value must be a sequence", field=field)
    try:
        return tuple(value)
    except (TypeError, ValueError) as exc:
        raise _error(code, "value must be a sequence", field=field) from exc


def _string_value(value: Any, *, code: str, field: str, nonempty: bool = True) -> str:
    if not isinstance(value, str) or (nonempty and not value):
        raise _error(code, "value must be a string", field=field)
    try:
        value.encode("utf-8", errors="strict")
    except UnicodeEncodeError as exc:
        raise _error("JSON_UNICODE_ENCODING", "text contains a non-UTF-8 Unicode scalar", field=field) from exc
    return value


def _optional_string(value: Any, *, field: str) -> str | None:
    if value is None:
        return None
    return _string_value(value, code="RECORD_FIELD_TYPE", field=field, nonempty=False)


def _mapping_value(value: Any, *, code: str, key_code: str, field: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise _error(code, "value must be an object", field=field)
    try:
        items = tuple(value.items())
    except (TypeError, ValueError, KeyError, AttributeError) as exc:
        raise _error(code, "object items could not be read", field=field) from exc
    result: dict[str, Any] = {}
    for key, item in items:
        if not isinstance(key, str):
            raise _error(key_code, "object keys must be strings", field=field)
        _string_value(key, code=key_code, field=field, nonempty=False)
        result[key] = item
    return result


def _post_runtime_member(record: RuntimeMember) -> None:
    path = _safe_member_path(record.path, field="member.path")
    size_bytes = _nonnegative_integer(record.size_bytes, field="member.size_bytes")
    digest = _sha256_value(record.sha256, field="member.sha256")
    mode = _string_value(record.mode, code="MODE_FORMAT", field="member.mode")
    if not _MODE_PATTERN.fullmatch(mode):
        raise _error("MODE_FORMAT", "mode must be a four-digit POSIX octal string", field="member.mode")
    role = _string_value(record.role, code="MEMBER_ROLE_TYPE", field="member.role")
    if role not in RUNTIME_MEMBER_ROLES:
        raise _error("MEMBER_ROLE", "unsupported runtime member role", field="member.role")
    object.__setattr__(record, "path", path)
    object.__setattr__(record, "size_bytes", size_bytes)
    object.__setattr__(record, "sha256", digest)
    object.__setattr__(record, "mode", mode)
    object.__setattr__(record, "role", role)


def _coerce_member(value: Any, *, field: str) -> RuntimeMember:
    if isinstance(value, RuntimeMember):
        return value
    mapping = _mapping_value(
        value,
        code="PROJECTION_MEMBER_TYPE",
        key_code="PROJECTION_MEMBER_KEY_TYPE",
        field=field,
    )
    _require_fields(
        mapping,
        required=_PROJECTION_MEMBER_FIELDS,
        allowed=_PROJECTION_MEMBER_FIELDS,
        prefix=field,
        missing_code="PROJECTION_MEMBER_MISSING_FIELD",
        unknown_code="PROJECTION_MEMBER_UNKNOWN_FIELD",
    )
    return RuntimeMember(
        mapping["path"], mapping["size_bytes"], mapping["sha256"], mapping["mode"], mapping["role"]
    )


def _post_runtime_projection(record: RuntimeProjection) -> None:
    version = _string_value(
        record.schema_version,
        code="PROJECTION_SCHEMA_VERSION_TYPE",
        field="projection.schema_version",
    )
    if version != PROJECTION_SCHEMA_VERSION:
        raise _error("PROJECTION_UNSUPPORTED_VERSION", "unsupported runtime projection version")
    raw_members = _tuple_value(record.members, code="PROJECTION_MEMBERS_TYPE", field="projection.members")
    if not raw_members:
        raise _error("PROJECTION_EMPTY_MEMBERS", "members must not be empty", field="projection.members")
    members = tuple(_coerce_member(item, field=f"projection.members[{index}]") for index, item in enumerate(raw_members))
    paths = tuple(member.path for member in members)
    if len(paths) != len(set(paths)):
        raise _error("PROJECTION_DUPLICATE_PATH", "duplicate member path", field="projection.members")
    if paths != tuple(sorted(paths)):
        raise _error("PROJECTION_MEMBER_ORDER", "members must be ordered by canonical path", field="projection.members")
    object.__setattr__(record, "schema_version", version)
    object.__setattr__(record, "members", members)


def _post_runtime_issue(record: RuntimeIssue) -> None:
    object.__setattr__(record, "code", _string_value(record.code, code="ISSUE_CODE_TYPE", field="issue.code"))
    for name in ("field", "path", "expected", "observed"):
        object.__setattr__(record, name, _optional_string(getattr(record, name), field=f"issue.{name}"))


def _post_projection_reconciliation(record: RuntimeProjectionReconciliation) -> None:
    for name in ("declared_count", "physical_regular_file_count", "matched_count"):
        object.__setattr__(record, name, _nonnegative_integer(getattr(record, name), field=name, code="RECONCILIATION_COUNT_TYPE"))
    issues = _tuple_value(record.issues, code="RECONCILIATION_ISSUES_TYPE", field="issues")
    if not all(isinstance(issue, RuntimeIssue) for issue in issues):
        raise _error("RECONCILIATION_ISSUE_TYPE", "issues must contain RuntimeIssue records", field="issues")
    object.__setattr__(record, "issues", tuple(sorted(set(issues), key=_issue_sort_key)))
    if record.matched_count > record.declared_count or record.physical_regular_file_count < record.matched_count:
        raise _error("RECONCILIATION_COUNT_INCONSISTENT", "reconciliation counts are inconsistent")
    if not record.issues and record.matched_count != record.declared_count:
        raise _error("RECONCILIATION_CLEAN_INCONSISTENT", "clean reconciliation must match every declaration")
    if type(record.authoritative) is not bool:
        raise _error("RECONCILIATION_AUTHORITATIVE_TYPE", "authoritative must be bool")


def _post_runtime_dependency(record: RuntimeDependency) -> None:
    dependency_type = _string_value(
        record.dependency_type,
        code="DEPENDENCY_TYPE_TYPE",
        field="dependency.dependency_type",
    )
    if dependency_type not in {"project", "external"}:
        raise _error("DEPENDENCY_TYPE", "unsupported dependency type", field="dependency.dependency_type")
    source = _safe_member_path(record.source, field="dependency.source")
    target = (
        _safe_member_path(record.target, field="dependency.target")
        if dependency_type == "project"
        else _external_dependency(record.target, field="dependency.target")
    )
    if not isinstance(record.resolved, bool):
        raise _error("DEPENDENCY_RESOLVED_TYPE", "resolved must be a boolean", field="dependency.resolved")
    object.__setattr__(record, "source", source)
    object.__setattr__(record, "target", target)
    object.__setattr__(record, "dependency_type", dependency_type)


def _path_tuple(value: Any, *, field: str) -> tuple[str, ...]:
    raw = _tuple_value(value, code="RECORD_SEQUENCE_TYPE", field=field)
    return tuple(_safe_member_path(item, field=f"{field}[{index}]") for index, item in enumerate(raw))


def _post_dependency_closure(record: RuntimeDependencyClosure) -> None:
    object.__setattr__(record, "entrypoints", tuple(sorted(_path_tuple(record.entrypoints, field="closure.entrypoints"))))
    object.__setattr__(record, "project_nodes", tuple(sorted(_path_tuple(record.project_nodes, field="closure.project_nodes"))))
    object.__setattr__(record, "reachable_members", tuple(sorted(_path_tuple(record.reachable_members, field="closure.reachable_members"))))
    raw_external = _tuple_value(record.external_dependencies, code="RECORD_SEQUENCE_TYPE", field="closure.external_dependencies")
    external = tuple(_external_dependency(item, field="closure.external_dependencies") for item in raw_external)
    issues = _tuple_value(record.issues, code="RECONCILIATION_ISSUES_TYPE", field="closure.issues")
    if not all(isinstance(issue, RuntimeIssue) for issue in issues):
        raise _error("RECONCILIATION_ISSUE_TYPE", "issues must contain RuntimeIssue records", field="closure.issues")
    object.__setattr__(record, "external_dependencies", tuple(sorted(external)))
    object.__setattr__(record, "issues", tuple(sorted(set(issues), key=_issue_sort_key)))
    if len(record.project_nodes) != len(set(record.project_nodes)) or len(record.reachable_members) != len(set(record.reachable_members)):
        raise _error("DEPENDENCY_DUPLICATE_NODE", "dependency nodes must be unique")
    if not record.issues and not set(record.entrypoints).issubset(record.project_nodes):
        raise _error("DEPENDENCY_ENTRYPOINT_UNDECLARED", "entrypoints must be project nodes")
    if not record.issues and not set(record.reachable_members).issubset(record.project_nodes):
        raise _error("DEPENDENCY_REACHABILITY_INCONSISTENT", "reachable members must be project nodes")
    if not record.issues and set(record.reachable_members) != set(record.project_nodes):
        raise _error("DEPENDENCY_CLEAN_INCONSISTENT", "clean closure must reach every project node")


def _post_argv_binding(record: RuntimeArgvBinding) -> None:
    index = _nonnegative_integer(record.argv_index, field="binding.argv_index", code="PLAN_ARGV_INDEX")
    try:
        path = _safe_member_path(record.member_path, field="binding.member_path")
    except RuntimeValidationError as exc:
        if exc.code == "PATH_ABSOLUTE":
            raise _error("PLAN_ABSOLUTE_ARGV", "absolute project argv path is forbidden") from exc
        raise
    object.__setattr__(record, "argv_index", index)
    object.__setattr__(record, "member_path", path)


def _post_external_command(record: RuntimeExternalCommand) -> None:
    index = _nonnegative_integer(record.argv_index, field="external_command.argv_index", code="PLAN_ARGV_INDEX")
    command = _string_value(record.command, code="PLAN_EXTERNAL_COMMAND_VALUE", field="external_command.command")
    if _contains_control(command):
        raise _error("PLAN_ARGV_CONTROL_CHARACTER", "external command contains a control character")
    if _token_looks_path_like(command):
        raise _error("PLAN_EXTERNAL_COMMAND_VALUE", "external command must be a bare command name")
    if not _COMMAND_PATTERN.fullmatch(command):
        raise _error("PLAN_EXTERNAL_COMMAND_VALUE", "external command name is not canonical")
    dependency = _external_dependency(record.dependency, field="external_command.dependency")
    object.__setattr__(record, "argv_index", index)
    object.__setattr__(record, "command", command)
    object.__setattr__(record, "dependency", dependency)


def _token_looks_path_like(value: str) -> bool:
    return bool(
        "/" in value
        or "\\" in value
        or value.startswith((".", "~"))
        or ".." in value
        or _DRIVE_PATTERN.match(value)
        or _URI_PATTERN.match(value)
        or _FILE_LIKE_SUFFIX_PATTERN.search(value)
    )


def _validate_literal(kind: str, value: str, *, field: str) -> None:
    if kind != "ros_name" and _token_looks_path_like(value):
        code = "PLAN_ARGV_ASSIGNMENT_PATH" if kind == "assignment" else "PLAN_ARGV_LITERAL_PATH"
        raise _error(code, "literal argv classification cannot contain a path", field=field)
    if kind == "flag" and not _FLAG_PATTERN.fullmatch(value):
        raise _error("PLAN_ARGV_FLAG", "flag token is not canonical", field=field)
    if kind == "identifier" and not _IDENTIFIER_PATTERN.fullmatch(value):
        raise _error("PLAN_ARGV_IDENTIFIER", "identifier token is not canonical", field=field)
    if kind == "integer" and not _INTEGER_PATTERN.fullmatch(value):
        raise _error("PLAN_ARGV_INTEGER", "integer token is not canonical", field=field)
    if kind == "numeric" and not _NUMERIC_PATTERN.fullmatch(value):
        raise _error("PLAN_ARGV_NUMERIC", "numeric token is not canonical", field=field)
    if kind == "ros_name" and not _ROS_NAME_PATTERN.fullmatch(value):
        raise _error("PLAN_ARGV_ROS_NAME", "ROS name token is not canonical", field=field)
    if kind == "assignment":
        separator = ":=" if ":=" in value else "="
        if value.count(separator) != 1:
            raise _error("PLAN_ARGV_ASSIGNMENT", "assignment token must contain one assignment separator", field=field)
        name, assigned = value.split(separator, 1)
        if not _ASSIGNMENT_NAME_PATTERN.fullmatch(name) or not assigned:
            raise _error("PLAN_ARGV_ASSIGNMENT", "assignment token is not canonical", field=field)
        if _token_looks_path_like(assigned):
            raise _error("PLAN_ARGV_ASSIGNMENT_PATH", "assignment value cannot contain a path", field=field)
        scalar_ok = (
            assigned in {"true", "false"}
            or bool(_IDENTIFIER_PATTERN.fullmatch(assigned))
            or bool(_INTEGER_PATTERN.fullmatch(assigned))
            or bool(_NUMERIC_PATTERN.fullmatch(assigned))
        )
        if not scalar_ok:
            raise _error("PLAN_ARGV_ASSIGNMENT", "assignment value is not a validated scalar", field=field)


def _post_argv_classification(record: RuntimeArgvClassification) -> None:
    index = _nonnegative_integer(
        record.argv_index,
        field="argv_classification.argv_index",
        code="PLAN_ARGV_CLASSIFICATION_INDEX",
    )
    kind = _string_value(record.kind, code="ARGV_CLASSIFICATION_TYPE", field="argv_classification.kind")
    if kind not in _ARGV_CLASSIFICATION_FIELDS:
        raise _error("PLAN_ARGV_CLASSIFICATION_KIND", "unknown argv classification kind")
    value = _string_value(record.value, code="ARGV_CLASSIFICATION_TYPE", field="argv_classification.value")
    if _contains_control(value):
        raise _error("PLAN_ARGV_CONTROL_CHARACTER", "argv classification value contains a control character")
    member_path = record.member_path
    dependency = record.dependency
    if kind == "project_member":
        member_path = _safe_member_path(member_path, field="argv_classification.member_path")
        if dependency is not None:
            raise _error("PLAN_ARGV_CLASSIFICATION_FIELDS", "project member cannot declare a dependency")
        if value != member_path:
            raise _error("PLAN_ARGV_CLASSIFICATION_VALUE_MISMATCH", "project value must equal member path")
    elif kind == "external_command":
        dependency = _external_dependency(dependency, field="argv_classification.dependency")
        if member_path is not None:
            raise _error("PLAN_ARGV_CLASSIFICATION_FIELDS", "external command cannot declare a member path")
        if _token_looks_path_like(value) or not _COMMAND_PATTERN.fullmatch(value):
            raise _error("PLAN_EXTERNAL_COMMAND_VALUE", "external command value must be a bare command name")
    else:
        if member_path is not None or dependency is not None:
            raise _error("PLAN_ARGV_CLASSIFICATION_FIELDS", "literal classification has unexpected binding fields")
        _validate_literal(kind, value, field="argv_classification.value")
    object.__setattr__(record, "argv_index", index)
    object.__setattr__(record, "kind", kind)
    object.__setattr__(record, "value", value)
    object.__setattr__(record, "member_path", member_path)
    object.__setattr__(record, "dependency", dependency)


def _post_plan_policy(record: RuntimePlanPolicy) -> None:
    for name in ("validate_only", "allow_full_launch", "launchable", "execution_authorized"):
        if not isinstance(getattr(record, name), bool):
            raise _error("PLAN_POLICY_BOOLEAN", "policy values must be booleans", field=f"policy.{name}")


def _coerce_binding(value: Any, *, field: str) -> RuntimeArgvBinding:
    if isinstance(value, RuntimeArgvBinding):
        return value
    mapping = _mapping_value(value, code="PLAN_BINDING_TYPE", key_code="PLAN_BINDING_KEY_TYPE", field=field)
    _require_fields(mapping, required=_PLAN_BINDING_FIELDS, allowed=_PLAN_BINDING_FIELDS, prefix=field,
                    missing_code="PLAN_BINDING_MISSING_FIELD", unknown_code="PLAN_BINDING_UNKNOWN_FIELD")
    return RuntimeArgvBinding(mapping["argv_index"], mapping["member_path"])


def _coerce_command(value: Any, *, field: str) -> RuntimeExternalCommand:
    if isinstance(value, RuntimeExternalCommand):
        return value
    mapping = _mapping_value(value, code="PLAN_EXTERNAL_COMMAND_TYPE", key_code="PLAN_EXTERNAL_COMMAND_KEY_TYPE", field=field)
    _require_fields(mapping, required=_PLAN_EXTERNAL_COMMAND_FIELDS, allowed=_PLAN_EXTERNAL_COMMAND_FIELDS, prefix=field,
                    missing_code="PLAN_EXTERNAL_COMMAND_MISSING_FIELD", unknown_code="PLAN_EXTERNAL_COMMAND_UNKNOWN_FIELD")
    return RuntimeExternalCommand(mapping["argv_index"], mapping["command"], mapping["dependency"])


def _coerce_classification(value: Any, *, field: str) -> RuntimeArgvClassification:
    if isinstance(value, RuntimeArgvClassification):
        return value
    mapping = _mapping_value(value, code="ARGV_CLASSIFICATION_TYPE", key_code="ARGV_CLASSIFICATION_KEY_TYPE", field=field)
    kind = mapping.get("kind")
    if not isinstance(kind, str):
        raise _error("ARGV_CLASSIFICATION_TYPE", "classification kind must be a string", field=f"{field}.kind")
    allowed = _ARGV_CLASSIFICATION_FIELDS.get(kind)
    if allowed is None:
        raise _error("PLAN_ARGV_CLASSIFICATION_KIND", "unknown argv classification kind", field=f"{field}.kind")
    _require_fields(mapping, required=allowed, allowed=allowed, prefix=field,
                    missing_code="PLAN_ARGV_CLASSIFICATION_MISSING_FIELD",
                    unknown_code="PLAN_ARGV_CLASSIFICATION_UNKNOWN_FIELD")
    return RuntimeArgvClassification(
        mapping["argv_index"], kind, mapping["value"], mapping.get("member_path"), mapping.get("dependency")
    )


def _post_runtime_plan(record: RuntimePlan) -> None:
    version = _string_value(record.schema_version, code="PLAN_SCHEMA_VERSION_TYPE", field="plan.schema_version")
    if version != PLAN_SCHEMA_VERSION:
        raise _error("PLAN_UNSUPPORTED_VERSION", "unsupported runtime plan version", field="plan.schema_version")
    mode = _string_value(record.mode, code="PLAN_MODE_TYPE", field="plan.mode")
    if mode not in RUNTIME_MODES:
        raise _error("PLAN_MODE", "unsupported runtime mode", field="plan.mode")
    identity = _sha256_value(record.production_runtime_identity, field="plan.production_runtime_identity")
    root_role = _string_value(record.runtime_root_role, code="PLAN_RUNTIME_ROOT_ROLE_TYPE", field="plan.runtime_root_role")
    raw_argv = _tuple_value(record.prospective_argv, code="PLAN_ARGV", field="plan.prospective_argv")
    if not raw_argv:
        raise _error("PLAN_ARGV", "prospective_argv must be nonempty", field="plan.prospective_argv")
    argv: list[str] = []
    for index, value in enumerate(raw_argv):
        token = _string_value(value, code="PLAN_ARGV_TOKEN", field=f"plan.prospective_argv[{index}]")
        if _contains_control(token):
            raise _error("PLAN_ARGV_CONTROL_CHARACTER", "argv token contains a control character")
        argv.append(token)
    raw_indices = _tuple_value(record.project_owned_argv_indices, code="PLAN_PROJECT_INDICES_TYPE", field="plan.project_owned_argv_indices")
    indices = tuple(_nonnegative_integer(item, field="plan.project_owned_argv_indices", code="PLAN_ARGV_INDEX") for item in raw_indices)
    if len(indices) != len(set(indices)):
        raise _error("PLAN_DUPLICATE_PROJECT_INDEX", "duplicate project argv index")
    bindings = tuple(_coerce_binding(item, field=f"plan.argv_bindings[{index}]") for index, item in enumerate(
        _tuple_value(record.argv_bindings, code="PLAN_BINDINGS_TYPE", field="plan.argv_bindings")))
    commands = tuple(_coerce_command(item, field=f"plan.external_commands[{index}]") for index, item in enumerate(
        _tuple_value(record.external_commands, code="PLAN_EXTERNAL_COMMANDS_TYPE", field="plan.external_commands")))
    classifications = tuple(_coerce_classification(item, field=f"plan.argv_classifications[{index}]") for index, item in enumerate(
        _tuple_value(record.argv_classifications, code="ARGV_CLASSIFICATION_TYPE", field="plan.argv_classifications")))
    if isinstance(record.prospective_environment, Mapping):
        environment_mapping = _mapping_value(record.prospective_environment, code="PLAN_ENVIRONMENT_TYPE",
                                             key_code="PLAN_ENVIRONMENT_KEY_TYPE", field="plan.prospective_environment")
        environment_items = tuple(environment_mapping.items())
    else:
        environment_items = _tuple_value(record.prospective_environment, code="PLAN_ENVIRONMENT_TYPE",
                                         field="plan.prospective_environment")
    environment: list[tuple[str, str]] = []
    for index, item in enumerate(environment_items):
        try:
            key, value = item
        except (TypeError, ValueError) as exc:
            raise _error("PLAN_ENVIRONMENT_ENTRY_TYPE", "environment entries must be pairs", field=str(index)) from exc
        key = _string_value(key, code="PLAN_ENVIRONMENT_KEY_TYPE", field=f"plan.prospective_environment[{index}].key")
        if not _ENVIRONMENT_NAME_PATTERN.fullmatch(key):
            raise _error("PLAN_ENVIRONMENT_KEY", "environment key is not canonical", field=key)
        value = _string_value(value, code="PLAN_ENVIRONMENT_VALUE", field=f"plan.prospective_environment.{key}", nonempty=False)
        if _contains_control(value):
            raise _error("PLAN_ENVIRONMENT_VALUE", "environment value contains a control character", field=key)
        environment.append((key, value))
    environment.sort(key=lambda item: item[0])
    if len(environment) != len({item[0] for item in environment}):
        raise _error("PLAN_DUPLICATE_ENVIRONMENT_KEY", "duplicate environment key")
    raw_external = _tuple_value(record.external_dependencies, code="PLAN_EXTERNAL_DEPENDENCIES_TYPE",
                                field="plan.external_dependencies")
    external = tuple(_external_dependency(item, field="plan.external_dependencies") for item in raw_external)
    if len(external) != len(set(external)):
        raise _error("PLAN_DUPLICATE_EXTERNAL_DEPENDENCY", "duplicate external dependency")
    policy = record.policy
    if isinstance(policy, Mapping):
        mapping = _mapping_value(policy, code="PLAN_POLICY_TYPE", key_code="PLAN_POLICY_KEY_TYPE", field="plan.policy")
        _require_fields(mapping, required=_PLAN_POLICY_FIELDS, allowed=_PLAN_POLICY_FIELDS, prefix="plan.policy",
                        missing_code="PLAN_POLICY_MISSING_FIELD", unknown_code="PLAN_POLICY_UNKNOWN_FIELD")
        policy = RuntimePlanPolicy(**mapping)
    if not isinstance(policy, RuntimePlanPolicy):
        raise _error("PLAN_POLICY_TYPE", "policy must be RuntimePlanPolicy or a mapping", field="plan.policy")
    raw_lineage = _tuple_value(record.diagnostic_lineage, code="PLAN_DIAGNOSTIC_IDENTITIES_TYPE",
                               field="plan.diagnostic_lineage")
    lineage = tuple(_sha256_value(item, field="plan.diagnostic_lineage") for item in raw_lineage)
    if len(lineage) != len(set(lineage)):
        raise _error("PLAN_DUPLICATE_DIAGNOSTIC_IDENTITY", "duplicate diagnostic identity")
    length = len(argv)
    for index in (*indices, *(item.argv_index for item in bindings), *(item.argv_index for item in commands),
                  *(item.argv_index for item in classifications)):
        if index >= length:
            raise _error("PLAN_ARGV_INDEX", "argv index is out of range")
    class_indices = tuple(item.argv_index for item in classifications)
    if len(class_indices) != len(set(class_indices)):
        raise _error("PLAN_DUPLICATE_ARGV_CLASSIFICATION", "duplicate argv classification index")
    if set(class_indices) != set(range(length)):
        raise _error("PLAN_ARGV_CLASSIFICATION_COVERAGE", "argv classifications must cover every index exactly once")
    for item in classifications:
        if argv[item.argv_index] != item.value:
            raise _error("PLAN_ARGV_CLASSIFICATION_VALUE_MISMATCH", "classified value differs from argv token")
    project_class = {item.argv_index: item for item in classifications if item.kind == "project_member"}
    external_class = {item.argv_index: item for item in classifications if item.kind == "external_command"}
    binding_by_index = {item.argv_index: item for item in bindings}
    command_by_index = {item.argv_index: item for item in commands}
    if len(binding_by_index) != len(bindings):
        raise _error("PLAN_DUPLICATE_ARGV_BINDING", "duplicate argv binding")
    if len(command_by_index) != len(commands):
        raise _error("PLAN_DUPLICATE_EXTERNAL_COMMAND", "duplicate external command")
    if set(indices) != set(binding_by_index) or set(indices) != set(project_class):
        missing = set(indices) - set(binding_by_index)
        code = "PLAN_MISSING_ARGV_BINDING" if missing else "PLAN_UNDECLARED_ARGV_BINDING"
        raise _error(code, "project classification and binding indices differ")
    if set(command_by_index) != set(external_class):
        raise _error("PLAN_COMMAND_UNCLASSIFIED", "external command classifications and records differ")
    for index, item in project_class.items():
        binding = binding_by_index[index]
        if item.member_path != binding.member_path or item.value != binding.member_path:
            raise _error("PLAN_BINDING_ARGV_MISMATCH", "project classification and binding differ")
    for index, item in external_class.items():
        command = command_by_index[index]
        if item.value != command.command or item.dependency != command.dependency:
            raise _error("PLAN_EXTERNAL_COMMAND_ARGV_MISMATCH", "external classification and command differ")
    object.__setattr__(record, "schema_version", version)
    object.__setattr__(record, "mode", mode)
    object.__setattr__(record, "production_runtime_identity", identity)
    object.__setattr__(record, "runtime_root_role", root_role)
    object.__setattr__(record, "prospective_argv", tuple(argv))
    object.__setattr__(record, "project_owned_argv_indices", tuple(sorted(indices)))
    object.__setattr__(record, "argv_bindings", tuple(sorted(bindings, key=lambda item: item.argv_index)))
    object.__setattr__(record, "external_commands", tuple(sorted(commands, key=lambda item: item.argv_index)))
    object.__setattr__(record, "argv_classifications", tuple(sorted(classifications, key=lambda item: item.argv_index)))
    object.__setattr__(record, "prospective_environment", tuple(environment))
    object.__setattr__(record, "external_dependencies", tuple(sorted(external)))
    object.__setattr__(record, "policy", policy)
    object.__setattr__(record, "diagnostic_lineage", tuple(sorted(lineage)))


def _post_runtime_reconciliation(record: RuntimeReconciliation) -> None:
    identity = _sha256_value(record.runtime_identity, field="reconciliation.runtime_identity")
    roles = _tuple_value(record.roles, code="RECONCILIATION_ROLES_TYPE", field="reconciliation.roles")
    if not all(isinstance(role, str) for role in roles):
        raise _error("RECONCILIATION_ROLE_TYPE", "roles must be strings")
    def pairs(value: Any, field: str) -> tuple[tuple[str, str], ...]:
        raw = _tuple_value(value, code="RECONCILIATION_PAIRS_TYPE", field=field)
        result: list[tuple[str, str]] = []
        for index, item in enumerate(raw):
            try:
                left, right = item
            except (TypeError, ValueError) as exc:
                raise _error("RECONCILIATION_PAIR_TYPE", "entries must be pairs", field=f"{field}[{index}]") from exc
            result.append((_string_value(left, code="RECONCILIATION_PAIR_TYPE", field=field),
                           _string_value(right, code="RECONCILIATION_PAIR_TYPE", field=field)))
        return tuple(result)
    object.__setattr__(record, "runtime_identity", identity)
    object.__setattr__(record, "roles", tuple(roles))
    object.__setattr__(record, "plan_sha256", pairs(record.plan_sha256, "reconciliation.plan_sha256"))
    object.__setattr__(record, "modes", pairs(record.modes, "reconciliation.modes"))
    if tuple(record.roles) != _SIX_PLAN_ROLES:
        raise _error("SIX_PLAN_ROLES_INCONSISTENT", "required six roles must be exact")
    if tuple(role for role, _ in record.plan_sha256) != _SIX_PLAN_ROLES or tuple(role for role, _ in record.modes) != _SIX_PLAN_ROLES:
        raise _error("SIX_PLAN_ROLES_INCONSISTENT", "role collections must be exact")
    for _, digest in record.plan_sha256:
        _sha256_value(digest, field="reconciliation.plan_sha256")
    for role, mode in record.modes:
        if mode != role.rsplit("_", 1)[0]:
            raise _error("SIX_PLAN_MODE_INCONSISTENT", "role and mode differ")


def _reject_constant(value: str) -> None:
    raise _error("JSON_NONFINITE_NUMBER", f"non-finite JSON number {value!r} is forbidden")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise _error("JSON_DUPLICATE_KEY", f"duplicate JSON key {key!r}", field=key)
        result[key] = value
    return result


def _source_bytes(source: JsonSource) -> bytes:
    if isinstance(source, Path):
        try:
            with source.open("rb") as stream:
                return stream.read()
        except OSError as exc:
            raise _error("SOURCE_READ_ERROR", str(exc), path=str(source)) from exc
    if isinstance(source, str):
        try:
            return source.encode("utf-8", errors="strict")
        except UnicodeEncodeError as exc:
            raise _error("JSON_UNICODE_ENCODING", "JSON text contains a non-UTF-8 Unicode scalar") from exc
    if isinstance(source, (bytes, bytearray, memoryview)):
        return bytes(source)
    raise _error("SOURCE_TYPE", "JSON source must be bytes, text, or pathlib.Path")


def _load_json(source: JsonSource) -> tuple[dict[str, Any], bytes]:
    raw = _source_bytes(source)
    try:
        text = raw.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise _error("JSON_INVALID_UTF8", "JSON source is not strict UTF-8") from exc
    try:
        value = json.loads(
            text,
            object_pairs_hook=_unique_object,
            parse_constant=_reject_constant,
        )
    except RuntimeValidationError:
        raise
    except json.JSONDecodeError as exc:
        raise _error("JSON_MALFORMED", exc.msg, field=f"line {exc.lineno} column {exc.colno}") from exc
    if not isinstance(value, dict):
        raise _error("JSON_TOP_LEVEL_TYPE", "top-level JSON value must be an object")
    return value, raw


def _require_fields(
    value: Mapping[str, Any],
    *,
    required: frozenset[str],
    allowed: frozenset[str],
    prefix: str,
    missing_code: str,
    unknown_code: str,
) -> None:
    try:
        keys = tuple(value.keys())
    except (TypeError, ValueError, KeyError, AttributeError) as exc:
        raise _error("OBJECT_KEYS_INVALID", "object keys could not be read", field=prefix) from exc
    for key in keys:
        if not isinstance(key, str):
            raise _error("OBJECT_KEY_TYPE", "object keys must be strings", field=prefix)
        _string_value(key, code="OBJECT_KEY_TYPE", field=prefix, nonempty=False)
    key_set = set(keys)
    missing = sorted(required - key_set)
    if missing:
        raise _error(missing_code, f"missing required field {missing[0]!r}", field=f"{prefix}.{missing[0]}")
    unknown = sorted(key_set - allowed)
    if unknown:
        raise _error(unknown_code, f"unknown field {unknown[0]!r}", field=f"{prefix}.{unknown[0]}")


def _contains_control(value: str) -> bool:
    return any(ord(character) <= 0x1F or ord(character) == 0x7F for character in value)


def _safe_member_path(value: Any, *, field: str) -> str:
    if not isinstance(value, str):
        raise _error("PATH_TYPE", "member path must be a string", field=field)
    try:
        value.encode("utf-8", errors="strict")
    except UnicodeEncodeError as exc:
        raise _error("PATH_UNICODE_ENCODING", "member path contains a non-UTF-8 Unicode scalar", field=field) from exc
    if not value:
        raise _error("PATH_EMPTY", "member path must not be empty", field=field)
    if _contains_control(value):
        code = "PATH_NUL" if "\x00" in value else "PATH_CONTROL_CHARACTER"
        raise _error(code, "member path contains a control character", field=field)
    if "\\" in value:
        raise _error("PATH_BACKSLASH", "member path must use POSIX separators", field=field)
    if value.startswith("/"):
        raise _error("PATH_ABSOLUTE", "absolute member paths are forbidden", field=field)
    if _DRIVE_PATTERN.match(value):
        raise _error("PATH_DRIVE_LETTER", "drive-letter paths are forbidden", field=field)
    if _URI_PATTERN.match(value):
        raise _error("PATH_URI", "URI-like member paths are forbidden", field=field)
    if "//" in value or value.endswith("/"):
        raise _error("PATH_NONCANONICAL_SEPARATOR", "member path has a repeated or trailing separator", field=field)
    normalized = unicodedata.normalize("NFC", value)
    if normalized != value:
        raise _error("PATH_UNICODE_NONCANONICAL", "member path must use NFC Unicode", field=field)
    parts = value.split("/")
    if any(part in {"", ".", ".."} for part in parts):
        raise _error("PATH_TRAVERSAL", "member path contains an empty, dot, or parent component", field=field)
    if str(PurePosixPath(value)) != value:
        raise _error("PATH_NONCANONICAL", "member path is not canonical POSIX form", field=field)
    return value


def _sha256_value(value: Any, *, field: str) -> str:
    if not isinstance(value, str) or not _SHA256_PATTERN.fullmatch(value):
        raise _error("SHA256_FORMAT", "value must be a lowercase 64-character SHA-256", field=field)
    return value


def _nonnegative_integer(value: Any, *, field: str, code: str = "SIZE_INVALID") -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise _error(code, "value must be a nonnegative integer", field=field)
    return value


def _projection_from_mapping(value: Mapping[str, Any]) -> RuntimeProjection:
    value = _mapping_value(
        value,
        code="PROJECTION_TYPE",
        key_code="PROJECTION_KEY_TYPE",
        field="projection",
    )
    forbidden = sorted(set(value) & _FORBIDDEN_PROJECTION_METADATA)
    if forbidden:
        raise _error(
            "PROJECTION_FORBIDDEN_METADATA",
            f"projection field {forbidden[0]!r} would make the logical identity envelope-dependent",
            field=forbidden[0],
        )
    _require_fields(
        value,
        required=_PROJECTION_FIELDS,
        allowed=_PROJECTION_FIELDS,
        prefix="projection",
        missing_code="PROJECTION_MISSING_FIELD",
        unknown_code="PROJECTION_UNKNOWN_FIELD",
    )
    if not isinstance(value["schema_version"], str):
        raise _error(
            "PROJECTION_SCHEMA_VERSION_TYPE",
            "projection schema version must be a string",
            field="projection.schema_version",
        )
    if value["schema_version"] != PROJECTION_SCHEMA_VERSION:
        raise _error(
            "PROJECTION_UNSUPPORTED_VERSION",
            f"supported version is {PROJECTION_SCHEMA_VERSION!r}",
            field="projection.schema_version",
        )
    raw_members = value["members"]
    if not isinstance(raw_members, list):
        raise _error("PROJECTION_MEMBERS_TYPE", "members must be an array", field="projection.members")
    if not raw_members:
        raise _error("PROJECTION_EMPTY_MEMBERS", "members must not be empty", field="projection.members")

    raw_paths: list[tuple[str, str, int]] = []
    for index, item in enumerate(raw_members):
        if isinstance(item, dict) and isinstance(item.get("path"), str):
            raw_path = item["path"]
            raw_paths.append((unicodedata.normalize("NFC", raw_path), raw_path, index))
    by_normalized: dict[str, tuple[str, int]] = {}
    for normalized, raw_path, index in raw_paths:
        previous = by_normalized.get(normalized)
        if previous is not None and previous[0] != raw_path:
            raise _error(
                "PATH_UNICODE_COLLISION",
                "member paths collide after NFC normalization",
                field=f"projection.members[{index}].path",
            )
        by_normalized[normalized] = (raw_path, index)

    members: list[RuntimeMember] = []
    seen_paths: set[str] = set()
    for index, item in enumerate(raw_members):
        prefix = f"projection.members[{index}]"
        if not isinstance(item, Mapping):
            raise _error("PROJECTION_MEMBER_TYPE", "member must be an object", field=prefix)
        item = _mapping_value(
            item,
            code="PROJECTION_MEMBER_TYPE",
            key_code="PROJECTION_MEMBER_KEY_TYPE",
            field=prefix,
        )
        forbidden_member = sorted(set(item) & _FORBIDDEN_PROJECTION_METADATA)
        if forbidden_member:
            raise _error(
                "PROJECTION_FORBIDDEN_METADATA",
                f"member field {forbidden_member[0]!r} is forbidden",
                field=f"{prefix}.{forbidden_member[0]}",
            )
        _require_fields(
            item,
            required=_PROJECTION_MEMBER_FIELDS,
            allowed=_PROJECTION_MEMBER_FIELDS,
            prefix=prefix,
            missing_code="PROJECTION_MEMBER_MISSING_FIELD",
            unknown_code="PROJECTION_MEMBER_UNKNOWN_FIELD",
        )
        path = _safe_member_path(item["path"], field=f"{prefix}.path")
        if path in seen_paths:
            raise _error("PROJECTION_DUPLICATE_PATH", "duplicate member path", field=f"{prefix}.path")
        seen_paths.add(path)
        size_bytes = _nonnegative_integer(item["size_bytes"], field=f"{prefix}.size_bytes")
        digest = _sha256_value(item["sha256"], field=f"{prefix}.sha256")
        mode = item["mode"]
        if not isinstance(mode, str) or not _MODE_PATTERN.fullmatch(mode):
            raise _error("MODE_FORMAT", "mode must be a four-digit POSIX octal string", field=f"{prefix}.mode")
        role = item["role"]
        if not isinstance(role, str):
            raise _error("MEMBER_ROLE_TYPE", "runtime member role must be a string", field=f"{prefix}.role")
        if role not in RUNTIME_MEMBER_ROLES:
            raise _error("MEMBER_ROLE", "unsupported runtime member role", field=f"{prefix}.role")
        members.append(RuntimeMember(path, size_bytes, digest, mode, role))

    ordered = sorted(members, key=lambda member: member.path)
    if members != ordered:
        raise _error(
            "PROJECTION_MEMBER_ORDER",
            "members must be ordered by canonical path",
            field="projection.members",
        )
    return RuntimeProjection(PROJECTION_SCHEMA_VERSION, tuple(members))


def load_runtime_projection(source: JsonSource | Mapping[str, Any] | RuntimeProjection) -> RuntimeProjection:
    """Strictly parse and validate one runtime projection."""

    if isinstance(source, RuntimeProjection):
        return _projection_from_mapping(_projection_mapping(source))
    if isinstance(source, Mapping):
        return _projection_from_mapping(source)
    value, _ = _load_json(source)
    return _projection_from_mapping(value)


def _projection_mapping(projection: RuntimeProjection) -> dict[str, Any]:
    return {
        "schema_version": projection.schema_version,
        "members": [
            {
                "path": member.path,
                "size_bytes": member.size_bytes,
                "sha256": member.sha256,
                "mode": member.mode,
                "role": member.role,
            }
            for member in projection.members
        ],
    }


def canonical_runtime_projection_bytes(
    projection: JsonSource | Mapping[str, Any] | RuntimeProjection,
) -> bytes:
    """Return canonical UTF-8 projection bytes with no trailing newline."""

    parsed = load_runtime_projection(projection)
    return json.dumps(
        _projection_mapping(parsed),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def runtime_projection_identity(
    projection: JsonSource | Mapping[str, Any] | RuntimeProjection,
) -> str:
    """Hash only the canonical logical projection bytes."""

    return sha256(canonical_runtime_projection_bytes(projection)).hexdigest()


def _issue_sort_key(issue: RuntimeIssue) -> tuple[str, str, str, str, str]:
    return (
        issue.path or "",
        issue.field or "",
        issue.code,
        issue.expected or "",
        issue.observed or "",
    )


def _beneath(candidate: Path, root: Path) -> bool:
    try:
        candidate.relative_to(root)
    except ValueError:
        return False
    return True


def _descriptor_primitives_available() -> bool:
    return bool(
        hasattr(os, "O_NOFOLLOW")
        and hasattr(os, "O_DIRECTORY")
        and os.open in os.supports_dir_fd
        and os.stat in os.supports_dir_fd
        and os.stat in os.supports_follow_symlinks
        and os.listdir in os.supports_fd
    )


def _descriptor_flags(*, directory: bool) -> int:
    flags = os.O_RDONLY | os.O_NOFOLLOW
    if directory:
        flags |= os.O_DIRECTORY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    return flags


def _stat_identity(info: os.stat_result) -> tuple[int, int]:
    return info.st_dev, info.st_ino


def _stat_stability(info: os.stat_result) -> tuple[int, int, int, int, int, int, int]:
    return (
        info.st_dev,
        info.st_ino,
        info.st_mode,
        info.st_nlink,
        info.st_size,
        info.st_mtime_ns,
        info.st_ctime_ns,
    )


def _writable(info: os.stat_result) -> bool:
    return bool(stat.S_IMODE(info.st_mode) & 0o222)


def _entry_stat(parent_fd: int, name: str) -> os.stat_result | None:
    try:
        return os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
    except OSError:
        return None


def _append_stability_issues(
    issues: list[RuntimeIssue],
    *,
    path: str,
    before: os.stat_result,
    after: os.stat_result,
) -> None:
    if _stat_identity(before) != _stat_identity(after):
        issues.append(RuntimeIssue("PHYSICAL_INODE_CHANGED", path=path))
    elif _stat_stability(before) != _stat_stability(after):
        issues.append(RuntimeIssue("PHYSICAL_CHANGED_DURING_READ", path=path))


def _hash_descriptor(file_fd: int) -> tuple[str, int]:
    digest = sha256()
    total = 0
    os.lseek(file_fd, 0, os.SEEK_SET)
    while True:
        block = os.read(file_fd, 1024 * 1024)
        if not block:
            break
        digest.update(block)
        total += len(block)
    return digest.hexdigest(), total


def _authenticate_member_descriptor(
    root_fd: int,
    member: RuntimeMember,
) -> tuple[tuple[RuntimeIssue, ...], tuple[int, int] | None, bool]:
    issues: list[RuntimeIssue] = []
    directory_fds: list[int] = []
    directory_records: list[tuple[int, str, int, os.stat_result, str]] = []
    file_fd: int | None = None
    opened_regular = False
    inode: tuple[int, int] | None = None
    try:
        parent_fd = os.dup(root_fd)
        directory_fds.append(parent_fd)
        parts = PurePosixPath(member.path).parts
        for index, part in enumerate(parts[:-1]):
            relative = PurePosixPath(*parts[: index + 1]).as_posix()
            try:
                child_fd = os.open(part, _descriptor_flags(directory=True), dir_fd=parent_fd)
            except OSError as exc:
                entry = _entry_stat(parent_fd, part)
                code = "PHYSICAL_SYMLINK" if entry is not None and stat.S_ISLNK(entry.st_mode) else "PHYSICAL_OPEN_ERROR"
                issues.append(RuntimeIssue(code, path=relative, observed=str(exc)))
                if code == "PHYSICAL_SYMLINK":
                    issues.append(RuntimeIssue("RUNTIME_ROOT_ESCAPE", path=member.path))
                return tuple(issues), None, False
            child_info = os.fstat(child_fd)
            entry_info = _entry_stat(parent_fd, part)
            if entry_info is None or _stat_identity(entry_info) != _stat_identity(child_info):
                issues.append(RuntimeIssue("PHYSICAL_INODE_CHANGED", path=relative))
            elif _stat_stability(entry_info) != _stat_stability(child_info):
                issues.append(RuntimeIssue("PHYSICAL_CHANGED_DURING_READ", path=relative))
            if not stat.S_ISDIR(child_info.st_mode):
                issues.append(RuntimeIssue("PHYSICAL_MEMBER_NOT_REGULAR", path=relative))
            if _writable(child_info):
                issues.append(RuntimeIssue("PHYSICAL_DIRECTORY_WRITABLE", path=relative))
            directory_fds.append(child_fd)
            directory_records.append((parent_fd, part, child_fd, child_info, relative))
            parent_fd = child_fd

        final_name = parts[-1]
        try:
            file_fd = os.open(final_name, _descriptor_flags(directory=False), dir_fd=parent_fd)
        except FileNotFoundError:
            issues.append(RuntimeIssue("PHYSICAL_MEMBER_MISSING", path=member.path))
            return tuple(issues), None, False
        except OSError as exc:
            entry = _entry_stat(parent_fd, final_name)
            code = "PHYSICAL_SYMLINK" if entry is not None and stat.S_ISLNK(entry.st_mode) else "PHYSICAL_OPEN_ERROR"
            issues.append(RuntimeIssue(code, path=member.path, observed=str(exc)))
            return tuple(issues), None, False

        before = os.fstat(file_fd)
        opened_regular = stat.S_ISREG(before.st_mode)
        if not opened_regular:
            issues.append(RuntimeIssue("PHYSICAL_MEMBER_NOT_REGULAR", path=member.path))
            return tuple(issues), None, False
        inode = _stat_identity(before)
        entry_before = _entry_stat(parent_fd, final_name)
        if entry_before is None or _stat_identity(entry_before) != inode:
            issues.append(RuntimeIssue("PHYSICAL_INODE_CHANGED", path=member.path))
        elif _stat_stability(entry_before) != _stat_stability(before):
            issues.append(RuntimeIssue("PHYSICAL_CHANGED_DURING_READ", path=member.path))
        if before.st_nlink != 1:
            issues.append(
                RuntimeIssue(
                    "PHYSICAL_HARDLINK_ALIAS",
                    path=member.path,
                    expected="link count 1",
                    observed=str(before.st_nlink),
                )
            )
        if _writable(before):
            issues.append(RuntimeIssue("PHYSICAL_FILE_WRITABLE", path=member.path))
        actual_mode = f"0{stat.S_IMODE(before.st_mode):03o}"
        if actual_mode != member.mode:
            issues.append(RuntimeIssue("PHYSICAL_MODE_MISMATCH", path=member.path, expected=member.mode, observed=actual_mode))
        if before.st_size != member.size_bytes:
            issues.append(
                RuntimeIssue(
                    "PHYSICAL_SIZE_MISMATCH",
                    path=member.path,
                    expected=str(member.size_bytes),
                    observed=str(before.st_size),
                )
            )
        try:
            actual_digest, bytes_read = _hash_descriptor(file_fd)
        except OSError as exc:
            issues.append(RuntimeIssue("PHYSICAL_READ_ERROR", path=member.path, observed=str(exc)))
            return tuple(issues), inode, True
        after = os.fstat(file_fd)
        _append_stability_issues(issues, path=member.path, before=before, after=after)
        if bytes_read != before.st_size:
            issues.append(
                RuntimeIssue(
                    "PHYSICAL_CHANGED_DURING_READ",
                    path=member.path,
                    expected=str(before.st_size),
                    observed=str(bytes_read),
                )
            )
        entry_after = _entry_stat(parent_fd, final_name)
        if entry_after is None or _stat_identity(entry_after) != inode:
            issues.append(RuntimeIssue("PHYSICAL_INODE_CHANGED", path=member.path))
        elif _stat_stability(entry_after) != _stat_stability(after):
            issues.append(RuntimeIssue("PHYSICAL_CHANGED_DURING_READ", path=member.path))
        if actual_digest != member.sha256:
            issues.append(
                RuntimeIssue(
                    "PHYSICAL_DIGEST_MISMATCH",
                    path=member.path,
                    expected=member.sha256,
                    observed=actual_digest,
                )
            )
        for ancestor_parent, name, child_fd, before_directory, relative in reversed(directory_records):
            after_directory = os.fstat(child_fd)
            _append_stability_issues(
                issues,
                path=relative,
                before=before_directory,
                after=after_directory,
            )
            current_entry = _entry_stat(ancestor_parent, name)
            if current_entry is None or _stat_identity(current_entry) != _stat_identity(before_directory):
                issues.append(RuntimeIssue("PHYSICAL_INODE_CHANGED", path=relative))
            elif _stat_stability(current_entry) != _stat_stability(after_directory):
                issues.append(RuntimeIssue("PHYSICAL_CHANGED_DURING_READ", path=relative))
        return tuple(sorted(set(issues), key=_issue_sort_key)), inode, True
    finally:
        if file_fd is not None:
            os.close(file_fd)
        for descriptor in reversed(directory_fds):
            os.close(descriptor)


def _inventory_descriptor(
    directory_fd: int,
    relative_directory: PurePosixPath,
    declared_paths: set[str],
    physical_regular_paths: set[str],
    issues: list[RuntimeIssue],
) -> None:
    try:
        names = sorted(os.listdir(directory_fd))
    except OSError as exc:
        issues.append(RuntimeIssue("PHYSICAL_INVENTORY_ERROR", path=relative_directory.as_posix(), observed=str(exc)))
        return
    for name in names:
        if not isinstance(name, str):
            issues.append(RuntimeIssue("PHYSICAL_INVENTORY_NAME_TYPE", path=relative_directory.as_posix()))
            continue
        relative = (relative_directory / name).as_posix() if relative_directory.parts else name
        entry = _entry_stat(directory_fd, name)
        if entry is None:
            issues.append(RuntimeIssue("PHYSICAL_CHANGED_DURING_READ", path=relative))
            continue
        if stat.S_ISLNK(entry.st_mode):
            issues.append(RuntimeIssue("PHYSICAL_SYMLINK", path=relative))
            continue
        if stat.S_ISDIR(entry.st_mode):
            if _writable(entry):
                issues.append(RuntimeIssue("PHYSICAL_DIRECTORY_WRITABLE", path=relative))
            child_fd: int | None = None
            try:
                child_fd = os.open(name, _descriptor_flags(directory=True), dir_fd=directory_fd)
                opened = os.fstat(child_fd)
                if _stat_identity(opened) != _stat_identity(entry):
                    issues.append(RuntimeIssue("PHYSICAL_INODE_CHANGED", path=relative))
                elif _stat_stability(opened) != _stat_stability(entry):
                    issues.append(RuntimeIssue("PHYSICAL_CHANGED_DURING_READ", path=relative))
                _inventory_descriptor(
                    child_fd,
                    relative_directory / name,
                    declared_paths,
                    physical_regular_paths,
                    issues,
                )
                after = os.fstat(child_fd)
                _append_stability_issues(issues, path=relative, before=opened, after=after)
                current = _entry_stat(directory_fd, name)
                if current is None or _stat_identity(current) != _stat_identity(opened):
                    issues.append(RuntimeIssue("PHYSICAL_INODE_CHANGED", path=relative))
                elif _stat_stability(current) != _stat_stability(after):
                    issues.append(RuntimeIssue("PHYSICAL_CHANGED_DURING_READ", path=relative))
            except OSError as exc:
                issues.append(RuntimeIssue("PHYSICAL_OPEN_ERROR", path=relative, observed=str(exc)))
            finally:
                if child_fd is not None:
                    os.close(child_fd)
        elif stat.S_ISREG(entry.st_mode):
            physical_regular_paths.add(relative)
            if relative not in declared_paths:
                issues.append(RuntimeIssue("PHYSICAL_UNDECLARED_FILE", path=relative))
        else:
            issues.append(RuntimeIssue("PHYSICAL_UNSUPPORTED_TYPE", path=relative))


def reconcile_runtime_projection(
    projection: JsonSource | Mapping[str, Any] | RuntimeProjection,
    runtime_root: str | Path,
    *,
    complete_inventory: bool = True,
    require_unique_physical_files: bool = True,
) -> RuntimeProjectionReconciliation:
    """Authenticate an immutable runtime tree through no-follow descriptors."""

    if type(complete_inventory) is not bool:
        raise _error("RECONCILE_COMPLETE_INVENTORY_TYPE", "must be bool", field="complete_inventory")
    if type(require_unique_physical_files) is not bool:
        raise _error("RECONCILE_UNIQUE_FILES_TYPE", "must be bool", field="require_unique_physical_files")
    parsed = load_runtime_projection(projection)
    root = Path(runtime_root)
    issues: list[RuntimeIssue] = []
    if not root.exists():
        issues.append(RuntimeIssue("RUNTIME_ROOT_MISSING", path=str(root)))
        return RuntimeProjectionReconciliation(len(parsed.members), 0, 0, tuple(issues))
    if not _descriptor_primitives_available():
        issues.append(RuntimeIssue("PHYSICAL_NOFOLLOW_UNAVAILABLE", path=str(root)))
        return RuntimeProjectionReconciliation(len(parsed.members), 0, 0, tuple(issues))
    try:
        root_lstat = os.lstat(root)
    except OSError as exc:
        issues.append(RuntimeIssue("RUNTIME_ROOT_STAT_ERROR", path=str(root), observed=str(exc)))
        return RuntimeProjectionReconciliation(len(parsed.members), 0, 0, tuple(issues))
    if stat.S_ISLNK(root_lstat.st_mode):
        issues.append(RuntimeIssue("RUNTIME_ROOT_SYMLINK", path=str(root)))
        return RuntimeProjectionReconciliation(len(parsed.members), 0, 0, tuple(issues))
    if not stat.S_ISDIR(root_lstat.st_mode):
        issues.append(RuntimeIssue("RUNTIME_ROOT_NOT_DIRECTORY", path=str(root)))
        return RuntimeProjectionReconciliation(len(parsed.members), 0, 0, tuple(issues))
    if _writable(root_lstat):
        issues.append(RuntimeIssue("RUNTIME_ROOT_WRITABLE", path=str(root)))
    declared_paths = {member.path for member in parsed.members}
    matched = 0
    physical_regular_paths: set[str] = set()
    inodes: dict[tuple[int, int], str] = {}
    root_fd: int | None = None
    try:
        try:
            root_fd = os.open(root, _descriptor_flags(directory=True))
        except OSError as exc:
            issues.append(RuntimeIssue("RUNTIME_ROOT_OPEN_ERROR", path=str(root), observed=str(exc)))
            return RuntimeProjectionReconciliation(len(parsed.members), 0, 0, tuple(sorted(set(issues), key=_issue_sort_key)))
        root_before = os.fstat(root_fd)
        if _stat_identity(root_before) != _stat_identity(root_lstat):
            issues.append(RuntimeIssue("PHYSICAL_INODE_CHANGED", path=str(root)))
        elif _stat_stability(root_before) != _stat_stability(root_lstat):
            issues.append(RuntimeIssue("PHYSICAL_CHANGED_DURING_READ", path=str(root)))
        if _writable(root_before):
            issues.append(RuntimeIssue("RUNTIME_ROOT_WRITABLE", path=str(root)))
        for member in parsed.members:
            member_issues, inode, opened_regular = _authenticate_member_descriptor(root_fd, member)
            issues.extend(member_issues)
            if opened_regular:
                physical_regular_paths.add(member.path)
            if inode is not None and require_unique_physical_files:
                previous = inodes.get(inode)
                if previous is not None and previous != member.path:
                    issues.append(
                        RuntimeIssue(
                            "PHYSICAL_HARDLINK_ALIAS",
                            path=member.path,
                            expected="unique inode",
                            observed=previous,
                        )
                    )
                inodes[inode] = member.path
            if not member_issues:
                matched += 1
        if complete_inventory:
            _inventory_descriptor(root_fd, PurePosixPath(), declared_paths, physical_regular_paths, issues)
        root_after = os.fstat(root_fd)
        _append_stability_issues(issues, path=str(root), before=root_before, after=root_after)
        try:
            root_current = os.lstat(root)
        except OSError:
            issues.append(RuntimeIssue("PHYSICAL_INODE_CHANGED", path=str(root)))
        else:
            if _stat_identity(root_current) != _stat_identity(root_before):
                issues.append(RuntimeIssue("PHYSICAL_INODE_CHANGED", path=str(root)))
            elif _stat_stability(root_current) != _stat_stability(root_before):
                issues.append(RuntimeIssue("PHYSICAL_CHANGED_DURING_READ", path=str(root)))
    finally:
        if root_fd is not None:
            os.close(root_fd)

    unique_issues = tuple(sorted(set(issues), key=_issue_sort_key))
    return RuntimeProjectionReconciliation(
        declared_count=len(parsed.members),
        physical_regular_file_count=len(physical_regular_paths),
        matched_count=matched,
        issues=unique_issues,
        authoritative=False,
    )


def _external_dependency(value: Any, *, field: str) -> str:
    if not isinstance(value, str) or not _EXTERNAL_DEPENDENCY_PATTERN.fullmatch(value):
        raise _error(
            "EXTERNAL_DEPENDENCY_FORMAT",
            "external dependency must use a portable package/command identifier",
            field=field,
        )
    return value


def validate_runtime_dependency_closure(
    projection: JsonSource | Mapping[str, Any] | RuntimeProjection,
    *,
    entrypoints: Sequence[str],
    project_nodes: Sequence[str],
    dependencies: Sequence[RuntimeDependency],
    declared_external_dependencies: Sequence[str],
    required_members: Sequence[str] | None = None,
) -> RuntimeDependencyClosure:
    """Validate, but do not discover, a supplied project dependency graph."""

    parsed = load_runtime_projection(projection)
    issues: list[RuntimeIssue] = []
    projection_paths = set(parsed.member_paths)

    def safe_sequence(values: Sequence[str], label: str) -> list[str]:
        result: list[str] = []
        for index, value in enumerate(tuple(values)):
            try:
                result.append(_safe_member_path(value, field=f"{label}[{index}]"))
            except RuntimeValidationError as exc:
                issues.append(RuntimeIssue(exc.code, field=exc.field, path=value if isinstance(value, str) else None))
        return result

    entrypoint_values = safe_sequence(entrypoints, "entrypoints")
    node_values = safe_sequence(project_nodes, "project_nodes")
    required_source = required_members if required_members is not None else parsed.member_paths
    required_values = safe_sequence(required_source, "required_members")

    if not entrypoint_values:
        issues.append(RuntimeIssue("DEPENDENCY_EMPTY_ENTRYPOINTS", field="entrypoints"))
    if not node_values:
        issues.append(RuntimeIssue("DEPENDENCY_EMPTY_PROJECT_NODES", field="project_nodes"))

    for label, values, code in (
        ("entrypoints", entrypoint_values, "DEPENDENCY_DUPLICATE_ENTRYPOINT"),
        ("project_nodes", node_values, "DEPENDENCY_DUPLICATE_NODE"),
        ("required_members", required_values, "DEPENDENCY_DUPLICATE_REQUIRED_MEMBER"),
    ):
        seen: set[str] = set()
        for value in values:
            if value in seen:
                issues.append(RuntimeIssue(code, field=label, path=value))
            seen.add(value)

    node_set = set(node_values)
    for entrypoint in entrypoint_values:
        if entrypoint not in projection_paths or entrypoint not in node_set:
            issues.append(RuntimeIssue("DEPENDENCY_ENTRYPOINT_MISSING", path=entrypoint))
        elif next(member.role for member in parsed.members if member.path == entrypoint) not in {
            "executable_entrypoint",
            "launch_file",
            "python_module",
        }:
            issues.append(RuntimeIssue("DEPENDENCY_ENTRYPOINT_ROLE", path=entrypoint))
    for node in node_values:
        if node not in projection_paths:
            issues.append(RuntimeIssue("DEPENDENCY_NODE_NOT_RUNTIME_MEMBER", path=node))
    for required in required_values:
        if required not in projection_paths:
            issues.append(RuntimeIssue("DEPENDENCY_REQUIRED_NOT_RUNTIME_MEMBER", path=required))

    for member in parsed.members:
        segments = set(PurePosixPath(member.path).parts)
        if segments & FORBIDDEN_RUNTIME_PATH_SEGMENTS:
            issues.append(RuntimeIssue("DEPENDENCY_EVIDENCE_TOOLING_MEMBER", path=member.path))

    declared_external: list[str] = []
    for index, value in enumerate(tuple(declared_external_dependencies)):
        try:
            declared_external.append(_external_dependency(value, field=f"declared_external_dependencies[{index}]"))
        except RuntimeValidationError as exc:
            issues.append(RuntimeIssue(exc.code, field=exc.field, path=str(value)))
    if len(declared_external) != len(set(declared_external)):
        issues.append(
            RuntimeIssue(
                "DEPENDENCY_DUPLICATE_EXTERNAL_DECLARATION",
                field="declared_external_dependencies",
            )
        )
    declared_external_set = set(declared_external)

    edge_values = tuple(dependencies)
    seen_edges: set[tuple[str, str, str]] = set()
    adjacency: dict[str, set[str]] = {node: set() for node in node_set}
    observed_external: set[str] = set()
    for index, edge in enumerate(edge_values):
        field = f"dependencies[{index}]"
        if not isinstance(edge, RuntimeDependency):
            issues.append(RuntimeIssue("DEPENDENCY_EDGE_TYPE", field=field))
            continue
        try:
            source = _safe_member_path(edge.source, field=f"{field}.source")
        except RuntimeValidationError as exc:
            issues.append(RuntimeIssue(exc.code, field=exc.field, path=edge.source))
            continue
        if edge.dependency_type not in {"project", "external"}:
            issues.append(RuntimeIssue("DEPENDENCY_TYPE", field=f"{field}.dependency_type", path=edge.target))
            continue
        if not isinstance(edge.resolved, bool):
            issues.append(RuntimeIssue("DEPENDENCY_RESOLVED_TYPE", field=f"{field}.resolved", path=edge.target))
            continue
        target = edge.target
        if edge.dependency_type == "project":
            try:
                target = _safe_member_path(edge.target, field=f"{field}.target")
            except RuntimeValidationError as exc:
                issues.append(RuntimeIssue(exc.code, field=exc.field, path=edge.target))
                continue
        else:
            try:
                target = _external_dependency(edge.target, field=f"{field}.target")
            except RuntimeValidationError as exc:
                issues.append(RuntimeIssue(exc.code, field=exc.field, path=edge.target))
                continue
        edge_key = (source, target, edge.dependency_type)
        if edge_key in seen_edges:
            issues.append(RuntimeIssue("DEPENDENCY_DUPLICATE_EDGE", field=field, path=f"{source}->{target}"))
        seen_edges.add(edge_key)
        if source not in node_set:
            issues.append(RuntimeIssue("DEPENDENCY_EDGE_SOURCE_UNDECLARED", field=field, path=source))
        if edge.dependency_type == "project":
            if not edge.resolved:
                issues.append(RuntimeIssue("DEPENDENCY_UNRESOLVED_PROJECT", field=field, path=target))
            if target not in node_set or target not in projection_paths:
                issues.append(RuntimeIssue("DEPENDENCY_EDGE_UNDECLARED_MEMBER", field=field, path=target))
            if source in adjacency and target in node_set:
                adjacency[source].add(target)
        else:
            observed_external.add(target)
            if not edge.resolved:
                issues.append(RuntimeIssue("DEPENDENCY_UNRESOLVED_EXTERNAL", field=field, path=target))
            if target not in declared_external_set:
                issues.append(RuntimeIssue("DEPENDENCY_UNDECLARED_EXTERNAL", field=field, path=target))

    reachable: set[str] = set()
    pending = sorted(set(entrypoint_values), reverse=True)
    while pending:
        current = pending.pop()
        if current in reachable or current not in node_set:
            continue
        reachable.add(current)
        for target in sorted(adjacency.get(current, ()), reverse=True):
            if target not in reachable:
                pending.append(target)
    for required in sorted(set(required_values)):
        if required in projection_paths and required not in reachable:
            issues.append(RuntimeIssue("DEPENDENCY_UNREACHABLE_REQUIRED_MEMBER", path=required))
    for unused in sorted(declared_external_set - observed_external):
        issues.append(
            RuntimeIssue(
                "DEPENDENCY_UNUSED_EXTERNAL_DECLARATION",
                field="declared_external_dependencies",
                path=unused,
            )
        )

    return RuntimeDependencyClosure(
        entrypoints=tuple(sorted(set(entrypoint_values))),
        project_nodes=tuple(sorted(set(node_values))),
        reachable_members=tuple(sorted(reachable)),
        external_dependencies=tuple(sorted(observed_external)),
        issues=tuple(sorted(set(issues), key=_issue_sort_key)),
    )


def _plan_from_mapping(value: Mapping[str, Any]) -> RuntimePlan:
    value = _mapping_value(value, code="PLAN_TYPE", key_code="PLAN_KEY_TYPE", field="plan")
    _require_fields(
        value,
        required=_PLAN_REQUIRED_FIELDS,
        allowed=_PLAN_FIELDS,
        prefix="plan",
        missing_code="PLAN_MISSING_FIELD",
        unknown_code="PLAN_UNKNOWN_FIELD",
    )
    if not isinstance(value["schema_version"], str):
        raise _error("PLAN_SCHEMA_VERSION_TYPE", "runtime plan version must be a string", field="plan.schema_version")
    if value["schema_version"] != PLAN_SCHEMA_VERSION:
        raise _error("PLAN_UNSUPPORTED_VERSION", "unsupported runtime plan version", field="plan.schema_version")
    mode = value["mode"]
    if not isinstance(mode, str):
        raise _error("PLAN_MODE_TYPE", "runtime plan mode must be a string", field="plan.mode")
    if mode not in RUNTIME_MODES:
        raise _error("PLAN_MODE", "unsupported runtime mode", field="plan.mode")
    identity = _sha256_value(value["production_runtime_identity"], field="plan.production_runtime_identity")
    runtime_root_role = value["runtime_root_role"]
    if not isinstance(runtime_root_role, str):
        raise _error(
            "PLAN_RUNTIME_ROOT_ROLE_TYPE",
            "runtime_root_role must be a string",
            field="plan.runtime_root_role",
        )

    raw_argv = value["prospective_argv"]
    if not isinstance(raw_argv, list) or not raw_argv:
        raise _error("PLAN_ARGV", "prospective_argv must be a nonempty string array", field="plan.prospective_argv")
    argv: list[str] = []
    for index, token in enumerate(raw_argv):
        if not isinstance(token, str) or not token:
            raise _error(
                "PLAN_ARGV_TOKEN",
                "argv tokens must be nonempty strings",
                field=f"plan.prospective_argv[{index}]",
            )
        try:
            token.encode("utf-8", errors="strict")
        except UnicodeEncodeError as exc:
            raise _error("JSON_UNICODE_ENCODING", "argv token contains a non-UTF-8 Unicode scalar") from exc
        if _contains_control(token):
            code = "PLAN_ARGV_NUL" if "\x00" in token else "PLAN_ARGV_CONTROL_CHARACTER"
            raise _error(code, "argv token contains a control character", field=f"plan.prospective_argv[{index}]")
        argv.append(token)

    raw_project_indices = value["project_owned_argv_indices"]
    if not isinstance(raw_project_indices, list):
        raise _error("PLAN_PROJECT_INDICES_TYPE", "project_owned_argv_indices must be an array")
    project_indices: list[int] = []
    for index, item in enumerate(raw_project_indices):
        if isinstance(item, bool) or not isinstance(item, int) or not 0 <= item < len(argv):
            raise _error(
                "PLAN_ARGV_INDEX",
                "project argv index is out of range",
                field=f"plan.project_owned_argv_indices[{index}]",
            )
        if item in project_indices:
            raise _error(
                "PLAN_DUPLICATE_PROJECT_INDEX",
                "duplicate project argv index",
                field=f"plan.project_owned_argv_indices[{index}]",
            )
        project_indices.append(item)

    raw_bindings = value["argv_bindings"]
    if not isinstance(raw_bindings, list):
        raise _error("PLAN_BINDINGS_TYPE", "argv_bindings must be an array", field="plan.argv_bindings")
    bindings: list[RuntimeArgvBinding] = []
    binding_indices: set[int] = set()
    for index, item in enumerate(raw_bindings):
        prefix = f"plan.argv_bindings[{index}]"
        if not isinstance(item, Mapping):
            raise _error("PLAN_BINDING_TYPE", "argv binding must be an object", field=prefix)
        item = _mapping_value(item, code="PLAN_BINDING_TYPE", key_code="PLAN_BINDING_KEY_TYPE", field=prefix)
        _require_fields(
            item,
            required=_PLAN_BINDING_FIELDS,
            allowed=_PLAN_BINDING_FIELDS,
            prefix=prefix,
            missing_code="PLAN_BINDING_MISSING_FIELD",
            unknown_code="PLAN_BINDING_UNKNOWN_FIELD",
        )
        argv_index = item["argv_index"]
        if isinstance(argv_index, bool) or not isinstance(argv_index, int) or not 0 <= argv_index < len(argv):
            raise _error("PLAN_ARGV_INDEX", "binding argv index is out of range", field=f"{prefix}.argv_index")
        if argv_index in binding_indices:
            raise _error("PLAN_DUPLICATE_ARGV_BINDING", "duplicate argv binding", field=f"{prefix}.argv_index")
        binding_indices.add(argv_index)
        bindings.append(RuntimeArgvBinding(argv_index, item["member_path"]))

    raw_commands = value["external_commands"]
    if not isinstance(raw_commands, list):
        raise _error("PLAN_EXTERNAL_COMMANDS_TYPE", "external_commands must be an array")
    commands: list[RuntimeExternalCommand] = []
    command_indices: set[int] = set()
    for index, item in enumerate(raw_commands):
        prefix = f"plan.external_commands[{index}]"
        if not isinstance(item, Mapping):
            raise _error("PLAN_EXTERNAL_COMMAND_TYPE", "external command must be an object", field=prefix)
        item = _mapping_value(item, code="PLAN_EXTERNAL_COMMAND_TYPE", key_code="PLAN_EXTERNAL_COMMAND_KEY_TYPE", field=prefix)
        _require_fields(
            item,
            required=_PLAN_EXTERNAL_COMMAND_FIELDS,
            allowed=_PLAN_EXTERNAL_COMMAND_FIELDS,
            prefix=prefix,
            missing_code="PLAN_EXTERNAL_COMMAND_MISSING_FIELD",
            unknown_code="PLAN_EXTERNAL_COMMAND_UNKNOWN_FIELD",
        )
        argv_index = item["argv_index"]
        if isinstance(argv_index, bool) or not isinstance(argv_index, int) or not 0 <= argv_index < len(argv):
            raise _error(
                "PLAN_ARGV_INDEX",
                "external command argv index is out of range",
                field=f"{prefix}.argv_index",
            )
        if argv_index in command_indices:
            raise _error(
                "PLAN_DUPLICATE_EXTERNAL_COMMAND",
                "duplicate external command index",
                field=f"{prefix}.argv_index",
            )
        command_indices.add(argv_index)
        dependency = _external_dependency(item["dependency"], field=f"{prefix}.dependency")
        command = item["command"]
        if not isinstance(command, str) or not command or "\x00" in command:
            raise _error(
                "PLAN_EXTERNAL_COMMAND_VALUE",
                "external command must be a nonempty NUL-free string",
                field=f"{prefix}.command",
            )
        commands.append(RuntimeExternalCommand(argv_index, command, dependency))

    raw_classifications = value["argv_classifications"]
    if not isinstance(raw_classifications, list):
        raise _error("ARGV_CLASSIFICATION_TYPE", "argv_classifications must be an array")
    classifications = [
        _coerce_classification(item, field=f"plan.argv_classifications[{index}]")
        for index, item in enumerate(raw_classifications)
    ]

    raw_environment = value["prospective_environment"]
    if not isinstance(raw_environment, Mapping):
        raise _error("PLAN_ENVIRONMENT_TYPE", "prospective_environment must be an object")
    raw_environment = _mapping_value(
        raw_environment,
        code="PLAN_ENVIRONMENT_TYPE",
        key_code="PLAN_ENVIRONMENT_KEY_TYPE",
        field="plan.prospective_environment",
    )
    environment: list[tuple[str, str]] = []
    for key in sorted(raw_environment):
        item = raw_environment[key]
        if not _ENVIRONMENT_NAME_PATTERN.fullmatch(key):
            raise _error(
                "PLAN_ENVIRONMENT_KEY",
                "environment key is not canonical",
                field=f"plan.prospective_environment.{key}",
            )
        if not isinstance(item, str) or "\x00" in item:
            raise _error(
                "PLAN_ENVIRONMENT_VALUE",
                "environment value must be a NUL-free string",
                field=f"plan.prospective_environment.{key}",
            )
        environment.append((key, item))

    raw_external = value["external_dependencies"]
    if not isinstance(raw_external, list):
        raise _error("PLAN_EXTERNAL_DEPENDENCIES_TYPE", "external_dependencies must be an array")
    external: list[str] = []
    for index, item in enumerate(raw_external):
        dependency = _external_dependency(item, field=f"plan.external_dependencies[{index}]")
        if dependency in external:
            raise _error("PLAN_DUPLICATE_EXTERNAL_DEPENDENCY", "duplicate external dependency")
        external.append(dependency)

    raw_policy = value["policy"]
    if not isinstance(raw_policy, Mapping):
        raise _error("PLAN_POLICY_TYPE", "policy must be an object", field="plan.policy")
    raw_policy = _mapping_value(raw_policy, code="PLAN_POLICY_TYPE", key_code="PLAN_POLICY_KEY_TYPE", field="plan.policy")
    _require_fields(
        raw_policy,
        required=_PLAN_POLICY_FIELDS,
        allowed=_PLAN_POLICY_FIELDS,
        prefix="plan.policy",
        missing_code="PLAN_POLICY_MISSING_FIELD",
        unknown_code="PLAN_POLICY_UNKNOWN_FIELD",
    )
    for field in sorted(_PLAN_POLICY_FIELDS):
        if not isinstance(raw_policy[field], bool):
            raise _error("PLAN_POLICY_BOOLEAN", "policy values must be booleans", field=f"plan.policy.{field}")
    policy = RuntimePlanPolicy(
        validate_only=raw_policy["validate_only"],
        allow_full_launch=raw_policy["allow_full_launch"],
        launchable=raw_policy["launchable"],
        execution_authorized=raw_policy["execution_authorized"],
    )

    diagnostic_lineage: list[str] = []
    if "diagnostic_lineage" in value:
        raw_lineage = value["diagnostic_lineage"]
        if not isinstance(raw_lineage, Mapping):
            raise _error("PLAN_DIAGNOSTIC_LINEAGE_TYPE", "diagnostic_lineage must be an object")
        raw_lineage = _mapping_value(
            raw_lineage,
            code="PLAN_DIAGNOSTIC_LINEAGE_TYPE",
            key_code="PLAN_DIAGNOSTIC_LINEAGE_KEY_TYPE",
            field="plan.diagnostic_lineage",
        )
        _require_fields(
            raw_lineage,
            required=_DIAGNOSTIC_LINEAGE_FIELDS,
            allowed=_DIAGNOSTIC_LINEAGE_FIELDS,
            prefix="plan.diagnostic_lineage",
            missing_code="PLAN_DIAGNOSTIC_LINEAGE_MISSING_FIELD",
            unknown_code="PLAN_DIAGNOSTIC_LINEAGE_UNKNOWN_FIELD",
        )
        raw_identities = raw_lineage["identities"]
        if not isinstance(raw_identities, list):
            raise _error("PLAN_DIAGNOSTIC_IDENTITIES_TYPE", "diagnostic identities must be an array")
        for index, item in enumerate(raw_identities):
            diagnostic_lineage.append(
                _sha256_value(item, field=f"plan.diagnostic_lineage.identities[{index}]")
            )
        if len(diagnostic_lineage) != len(set(diagnostic_lineage)):
            raise _error("PLAN_DUPLICATE_DIAGNOSTIC_IDENTITY", "duplicate diagnostic identity")

    return RuntimePlan(
        schema_version=PLAN_SCHEMA_VERSION,
        mode=mode,
        production_runtime_identity=identity,
        runtime_root_role=runtime_root_role,
        prospective_argv=tuple(argv),
        project_owned_argv_indices=tuple(sorted(project_indices)),
        argv_bindings=tuple(sorted(bindings, key=lambda binding: binding.argv_index)),
        external_commands=tuple(sorted(commands, key=lambda command: command.argv_index)),
        argv_classifications=tuple(sorted(classifications, key=lambda item: item.argv_index)),
        prospective_environment=tuple(environment),
        external_dependencies=tuple(sorted(external)),
        policy=policy,
        diagnostic_lineage=tuple(sorted(diagnostic_lineage)),
    )


def load_runtime_plan(source: JsonSource | Mapping[str, Any] | RuntimePlan) -> RuntimePlan:
    """Strictly parse one versioned prospective runtime plan."""

    if isinstance(source, RuntimePlan):
        return _plan_from_mapping(_plan_mapping(source))
    if isinstance(source, Mapping):
        return _plan_from_mapping(source)
    value, _ = _load_json(source)
    return _plan_from_mapping(value)


def _plan_mapping(plan: RuntimePlan) -> dict[str, Any]:
    value: dict[str, Any] = {
        "schema_version": plan.schema_version,
        "mode": plan.mode,
        "production_runtime_identity": plan.production_runtime_identity,
        "runtime_root_role": plan.runtime_root_role,
        "prospective_argv": list(plan.prospective_argv),
        "project_owned_argv_indices": list(plan.project_owned_argv_indices),
        "argv_bindings": [
            {"argv_index": binding.argv_index, "member_path": binding.member_path}
            for binding in plan.argv_bindings
        ],
        "external_commands": [
            {
                "argv_index": command.argv_index,
                "command": command.command,
                "dependency": command.dependency,
            }
            for command in plan.external_commands
        ],
        "argv_classifications": [
            {
                "argv_index": classification.argv_index,
                "kind": classification.kind,
                "value": classification.value,
                **(
                    {"member_path": classification.member_path}
                    if classification.kind == "project_member"
                    else {}
                ),
                **(
                    {"dependency": classification.dependency}
                    if classification.kind == "external_command"
                    else {}
                ),
            }
            for classification in plan.argv_classifications
        ],
        "prospective_environment": dict(plan.prospective_environment),
        "external_dependencies": list(plan.external_dependencies),
        "policy": {
            "validate_only": plan.policy.validate_only,
            "allow_full_launch": plan.policy.allow_full_launch,
            "launchable": plan.policy.launchable,
            "execution_authorized": plan.policy.execution_authorized,
        },
    }
    if plan.diagnostic_lineage:
        value["diagnostic_lineage"] = {"identities": list(plan.diagnostic_lineage)}
    return value


def validate_runtime_plan(
    plan, projection, runtime_authority, **kwargs
):
    if type(runtime_authority) is not AuthenticatedRuntimeSnapshot:
        raise _error("PLAN_SNAPSHOT_REQUIRED", "an open authenticated snapshot is required", field="runtime_authority")
    with runtime_authority._borrow():
        return _validate_runtime_plan_unlocked(plan, projection, runtime_authority, **kwargs)


def _validate_runtime_plan_unlocked(
    plan: JsonSource | Mapping[str, Any] | RuntimePlan,
    projection: JsonSource | Mapping[str, Any] | RuntimeProjection,
    runtime_authority: AuthenticatedRuntimeSnapshot,
    *,
    expected_runtime_identity: str | None = None,
    forbidden_identities: Iterable[str] = (),
    allowed_external_dependencies: Iterable[str] | None = None,
    ros_name_authority: Mapping[int, str] | None = None,
) -> RuntimePlan:
    """Validate one plan against an authenticated production runtime root."""

    if type(runtime_authority) is not AuthenticatedRuntimeSnapshot:
        raise _error("PLAN_SNAPSHOT_REQUIRED", "an open authenticated snapshot is required", field="runtime_authority")
    if runtime_authority.closed:
        raise _error("SNAPSHOT_CLOSED", "authenticated runtime snapshot is closed")
    if runtime_authority.issues:
        first = runtime_authority.issues[0]
        if first.code == "PHYSICAL_SYMLINK":
            raise _error("PLAN_BINDING_ROOT_ESCAPE", "bound member escapes runtime root", path=first.path)
        raise _error(first.code, "snapshot authentication failed", path=first.path)
    parsed_plan = load_runtime_plan(plan)
    parsed_projection = load_runtime_projection(projection)
    computed_identity = runtime_projection_identity(parsed_projection)
    expected = computed_identity if expected_runtime_identity is None else expected_runtime_identity
    _sha256_value(expected, field="expected_runtime_identity")
    if expected != computed_identity:
        raise _error(
            "EXPECTED_RUNTIME_IDENTITY_MISMATCH",
            "explicit expected identity does not match the supplied projection",
            field="expected_runtime_identity",
        )
    forbidden = frozenset(
        _sha256_value(value, field="forbidden_identities") for value in tuple(forbidden_identities)
    )
    if parsed_plan.production_runtime_identity in forbidden:
        raise _error(
            "PLAN_STALE_OPERATIVE_IDENTITY",
            "operative runtime identity is explicitly forbidden",
            field="plan.production_runtime_identity",
        )
    if parsed_plan.production_runtime_identity != expected:
        raise _error(
            "PLAN_RUNTIME_IDENTITY_MISMATCH",
            "plan does not bind the expected production runtime identity",
            field="plan.production_runtime_identity",
        )
    if parsed_plan.runtime_root_role != AUTHENTICATED_RUNTIME_ROOT:
        raise _error(
            "PLAN_RUNTIME_ROOT_ROLE",
            f"runtime_root_role must be {AUTHENTICATED_RUNTIME_ROOT}",
            field="plan.runtime_root_role",
        )
    if parsed_plan.mode == "production" and not parsed_plan.project_owned_argv_indices:
        raise _error(
            "PLAN_PRODUCTION_BINDING_REQUIRED",
            "production mode requires at least one explicit project-owned argv binding",
            field="plan.project_owned_argv_indices",
        )

    project_indices = set(parsed_plan.project_owned_argv_indices)
    binding_indices = {binding.argv_index for binding in parsed_plan.argv_bindings}
    if project_indices != binding_indices:
        missing = sorted(project_indices - binding_indices)
        extra = sorted(binding_indices - project_indices)
        code = "PLAN_MISSING_ARGV_BINDING" if missing else "PLAN_UNDECLARED_ARGV_BINDING"
        observed = missing[0] if missing else extra[0]
        raise _error(code, f"argv binding index mismatch at {observed}", field="plan.argv_bindings")

    members_by_path = {member.path: member for member in parsed_projection.members}
    member_paths = set(members_by_path)
    for binding in parsed_plan.argv_bindings:
        if binding.member_path not in member_paths:
            raise _error(
                "PLAN_BINDING_MEMBER_ABSENT",
                "argv binding references a non-member",
                field=f"plan.argv_bindings[{binding.argv_index}]",
                path=binding.member_path,
            )
        if parsed_plan.prospective_argv[binding.argv_index] != binding.member_path:
            raise _error(
                "PLAN_BINDING_ARGV_MISMATCH",
                "bound argv token must exactly equal its canonical member path",
                field=f"plan.prospective_argv[{binding.argv_index}]",
            )

    if runtime_authority.projection != parsed_projection:
        raise _error("PLAN_SNAPSHOT_PROJECTION_MISMATCH", "snapshot projection differs from plan projection")
    for binding in parsed_plan.argv_bindings:
        if binding.member_path not in runtime_authority._provenance().member_fds:
            raise _error("PLAN_BINDING_PHYSICAL_MISSING", "bound member is absent from snapshot", path=binding.member_path)

    external_command_indices = {command.argv_index for command in parsed_plan.external_commands}
    if 0 not in project_indices and 0 not in external_command_indices:
        raise _error("PLAN_COMMAND_UNCLASSIFIED", "argv[0] must be a project member or declared external command")
    if project_indices & external_command_indices:
        raise _error("PLAN_ARGV_CLASSIFICATION_OVERLAP", "argv index has both project and external roles")
    declared_external = set(parsed_plan.external_dependencies)
    for command in parsed_plan.external_commands:
        if parsed_plan.prospective_argv[command.argv_index] != command.command:
            raise _error(
                "PLAN_EXTERNAL_COMMAND_ARGV_MISMATCH",
                "external command must exactly match its argv token",
                field=f"plan.external_commands[{command.argv_index}]",
            )
        if command.dependency not in declared_external:
            raise _error(
                "PLAN_UNDECLARED_EXTERNAL_COMMAND",
                "external command dependency is not declared",
                field=f"plan.external_commands[{command.argv_index}]",
            )
    if allowed_external_dependencies is None:
        used_by_commands = {command.dependency for command in parsed_plan.external_commands}
        unused = sorted(declared_external - used_by_commands)
        if unused:
            raise _error(
                "DEPENDENCY_UNUSED_EXTERNAL_DECLARATION",
                "plan external dependency is neither graph-validated nor used by a classified command",
                path=unused[0],
            )
    if allowed_external_dependencies is not None:
        if isinstance(allowed_external_dependencies, (str, bytes, Mapping)):
            raise _error("EXTERNAL_DEPENDENCIES_CONTAINER_TYPE", "must be a collection of strings", field="allowed_external_dependencies")
        allowed = {
            _external_dependency(item, field="allowed_external_dependencies")
            for item in tuple(allowed_external_dependencies)
        }
        unexpected = sorted(declared_external - allowed)
        if unexpected:
            raise _error(
                "PLAN_EXTERNAL_DEPENDENCY_NOT_ALLOWED",
                "plan declares an external dependency absent from the validated dependency closure",
                path=unexpected[0],
            )
        missing = sorted(allowed - declared_external)
        if missing:
            raise _error(
                "PLAN_EXTERNAL_DEPENDENCY_MISSING",
                "plan omits a dependency from the validated dependency closure",
                path=missing[0],
            )

    ros_indices = {c.argv_index: c.value for c in parsed_plan.argv_classifications if c.kind == "ros_name"}
    if ros_indices:
        if ros_name_authority is None:
            raise _error("ROS_NAME_AUTHORITY_REQUIRED", "ros-name authority is required", field="ros_name_authority")
        if not isinstance(ros_name_authority, Mapping):
            raise _error("ROS_NAME_AUTHORITY_TYPE", "ros-name authority must be a mapping", field="ros_name_authority")
        normalized = {}
        for index, value in ros_name_authority.items():
            if type(index) is not int or type(value) is not str:
                raise _error("ROS_NAME_AUTHORITY_ENTRY_TYPE", "authority entries require int/string", field="ros_name_authority")
            normalized[index] = value
        if set(normalized) != set(ros_indices):
            raise _error("ROS_NAME_AUTHORITY_INDEX_MISMATCH", "authority indices do not match ros-name tokens")
        for index, value in ros_indices.items():
            if normalized[index] != value:
                raise _error("ROS_NAME_AUTHORITY_VALUE_MISMATCH", "authority value differs", field=f"argv[{index}]")
    elif ros_name_authority:
        raise _error("ROS_NAME_AUTHORITY_UNUSED", "ros-name authority was not consumed", field="ros_name_authority")

    policy = parsed_plan.policy
    if policy.validate_only and (
        policy.allow_full_launch or policy.launchable or policy.execution_authorized
    ):
        raise _error("PLAN_POLICY_INCONSISTENT", "validate-only policy cannot authorize or launch")
    if policy.launchable and not (policy.allow_full_launch and policy.execution_authorized):
        raise _error("PLAN_POLICY_INCONSISTENT", "launchable requires full-launch and execution authorization")
    if policy.allow_full_launch and not policy.execution_authorized:
        raise _error("PLAN_POLICY_INCONSISTENT", "full launch requires execution authorization")
    return parsed_plan


def validate_six_plan_set(
    plans: Mapping[str, JsonSource],
    projection: JsonSource | Mapping[str, Any] | RuntimeProjection,
    runtime_authority: AuthenticatedRuntimeSnapshot,
    *,
    expected_runtime_identity: str | None = None,
    forbidden_identities: Iterable[str] = (),
    allowed_external_dependencies: Iterable[str] | None = None,
    ros_name_authority: Mapping[int, str] | None = None,
) -> RuntimeReconciliation:
    """Validate the exact production/offline/TEST_ONLY root/duplicate set."""

    supplied_roles = set(plans.keys())
    required_roles = set(_SIX_PLAN_ROLES)
    if supplied_roles != required_roles:
        missing = sorted(required_roles - supplied_roles)
        extra = sorted(supplied_roles - required_roles)
        if missing:
            raise _error("SIX_PLAN_MISSING_ROLE", f"missing plan role {missing[0]!r}", field=missing[0])
        raise _error("SIX_PLAN_EXTRA_ROLE", f"unexpected plan role {extra[0]!r}", field=extra[0])

    raw_by_role = {role: _source_bytes(plans[role]) for role in _SIX_PLAN_ROLES}
    forbidden_values = tuple(forbidden_identities)
    allowed_values = None if allowed_external_dependencies is None else tuple(allowed_external_dependencies)
    for mode in ("production", "offline", "test_only"):
        root_role = f"{mode}_root"
        duplicate_role = f"{mode}_duplicate"
        if raw_by_role[root_role] != raw_by_role[duplicate_role]:
            raise _error(
                "SIX_PLAN_BYTE_MISMATCH",
                f"{mode} root and duplicate bytes differ",
                field=duplicate_role,
            )

    parsed_projection = load_runtime_projection(projection)
    computed_identity = runtime_projection_identity(parsed_projection)
    expected = computed_identity if expected_runtime_identity is None else expected_runtime_identity
    parsed_by_role: dict[str, RuntimePlan] = {}
    modes: list[tuple[str, str]] = []
    hashes: list[tuple[str, str]] = []
    for role in _SIX_PLAN_ROLES:
        parsed = load_runtime_plan(raw_by_role[role])
        assigned_mode = role.rsplit("_", 1)[0]
        if parsed.mode != assigned_mode:
            raise _error(
                "SIX_PLAN_ASSIGNED_MODE_MISMATCH",
                f"role {role!r} requires mode {assigned_mode!r}",
                field=role,
            )
        validated = validate_runtime_plan(
            parsed,
            parsed_projection,
            runtime_authority,
            expected_runtime_identity=expected,
            forbidden_identities=forbidden_values,
            allowed_external_dependencies=allowed_values,
            ros_name_authority=ros_name_authority,
        )
        parsed_by_role[role] = validated
        modes.append((role, validated.mode))
        hashes.append((role, sha256(raw_by_role[role]).hexdigest()))

    identities = {plan.production_runtime_identity for plan in parsed_by_role.values()}
    if identities != {expected}:
        raise _error("SIX_PLAN_RUNTIME_IDENTITY_MISMATCH", "not all plans bind the same expected identity")
    return RuntimeReconciliation(
        runtime_identity=expected,
        roles=_SIX_PLAN_ROLES,
        plan_sha256=tuple(hashes),
        modes=tuple(modes),
    )


__all__ = [
    "AUTHENTICATED_RUNTIME_ROOT",
    "PLAN_SCHEMA_VERSION",
    "PROJECTION_SCHEMA_VERSION",
    "RuntimeDependency",
    "RuntimeDependencyClosure",
    "RuntimeArgvClassification",
    "RuntimeIssue",
    "RuntimeMember",
    "RuntimePlan",
    "RuntimeProjection",
    "RuntimeProjectionReconciliation",
    "RuntimeReconciliation",
    "RuntimeValidationError",
    "canonical_runtime_projection_bytes",
    "load_runtime_plan",
    "load_runtime_projection",
    "reconcile_runtime_projection",
    "runtime_projection_identity",
    "validate_runtime_dependency_closure",
    "validate_runtime_plan",
    "validate_six_plan_set",
]
