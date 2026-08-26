"""Canonical, ROS-independent identities for sealed runtime materializations.

A projection is an authenticated, portable description, but it is not proof
that a corresponding tree exists.  Projection-only helpers therefore expose
only canonical bytes, a logical identity, and a clearly non-authoritative
framing digest.  A value named ``physical_rehash`` is returned only after a
fresh descriptor-relative, no-follow traversal of a real root.
"""
from __future__ import annotations

import json
import os
import re
import stat
import unicodedata
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path, PurePosixPath
from typing import Iterable


MATERIALIZATION_PROJECTION_SCHEMA = "ctr-materialization-projection-1"
LOGICAL_ALGORITHM_ID = "sha256:ctr-materialization-logical-1"
PHYSICAL_REHASH_ALGORITHM_ID = "sha256-framed:ctr-materialization-physical-1"
PROJECTION_FRAMING_ALGORITHM_ID = (
    "sha256-framed:ctr-materialization-physical-1:projection-only-nonauthoritative"
)
_LOGICAL_DOMAIN = b"CTR-MATERIALIZATION-LOGICAL-1\x00"
_PHYSICAL_DOMAIN = b"CTR-MATERIALIZATION-PHYSICAL-1\x00"
_HEX = re.compile(r"^[0-9a-f]{64}$")
_MODE = re.compile(r"^0[0-7]{3}$")
_TRANSIENT_COMPONENTS = frozenset({
    ".cache", ".coverage", ".hypothesis", ".mypy_cache", ".nox",
    ".pytest_cache", ".ruff_cache", ".tox", "__pycache__", "build",
    "htmlcov", "install", "log",
})
SUPERSEDED_HISTORICAL_IDENTITIES = frozenset({
    "078727ff4cdb535d71f98ef4f2ae1487f4609b38ba5401df6ee424fa89a572e1",
    "f981c80c9e366e12a1406f52f452e09243800baa178412efb6900afc015bed94",
})

_CANONICALIZATION = {
    "encoding": "UTF-8",
    "json": "RFC8259-sort-keys-compact-ensure-ascii-false",
    "path_order": "utf8-byte-ascending",
    "path_unicode_normalization": "NFC",
    "root_path": ".",
    "trailing_newline": False,
}
_POLICIES = {
    "empty_directories": "include",
    "executable_role": "derived-from-any-execute-mode-bit",
    "hardlinks": "reject",
    "symlinks": "reject",
    "transient_or_cache_paths": "reject-no-exclusions",
    "unsupported_file_types": "reject",
    "writable_paths": "reject",
}
_ALGORITHMS = {
    "logical_identity": LOGICAL_ALGORITHM_ID,
    "physical_rehash": PHYSICAL_REHASH_ALGORITHM_ID,
}


class MaterializationIdentityError(Exception):
    """Stable failure carrying a machine-readable materialization code."""

    def __init__(self, code: str, path: str = "", detail: str = ""):
        self.code = str(code)
        self.path = str(path)
        self.detail = str(detail)
        message = self.code
        if self.path:
            message += f": {self.path}"
        if self.detail:
            message += f": {self.detail}"
        super().__init__(message)


def _fail(code: str, path: str = "", detail: str = "") -> None:
    raise MaterializationIdentityError(code, path, detail)


def _safe_path(value: object, *, allow_root: bool = False) -> str:
    if type(value) is not str or not value or "\\" in value or "\x00" in value:
        _fail("MATERIALIZATION_PATH_INVALID", str(value))
    if value == ".":
        if allow_root:
            return value
        _fail("MATERIALIZATION_PATH_INVALID", value)
    normalized = unicodedata.normalize("NFC", value)
    if normalized != value:
        _fail("MATERIALIZATION_PATH_NOT_NFC", value)
    if any(unicodedata.category(char) in {"Cc", "Cs"} for char in value):
        _fail("MATERIALIZATION_PATH_INVALID", value)
    path = PurePosixPath(value)
    if (path.is_absolute() or str(path) != value
            or any(part in {"", ".", ".."} for part in path.parts)
            or re.match(r"^[A-Za-z][A-Za-z0-9+.-]*:", value)):
        _fail("MATERIALIZATION_PATH_INVALID", value)
    return value


def _mode(value: object, path: str) -> str:
    if type(value) is not str or not _MODE.fullmatch(value):
        _fail("MATERIALIZATION_MODE_INVALID", path)
    if int(value, 8) & 0o222:
        _fail("MATERIALIZATION_WRITABLE_MEMBER", path)
    return value


def _digest(value: object, path: str) -> str:
    if type(value) is not str or not _HEX.fullmatch(value):
        _fail("MATERIALIZATION_DIGEST_INVALID", path)
    return value


def _canonical(value: object) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def _check_components(path: str) -> None:
    if path == ".":
        return
    for part in PurePosixPath(path).parts:
        if (part in _TRANSIENT_COMPONENTS
                or part.endswith((".pyc", ".pyo"))
                or part.startswith(".coverage.")):
            _fail("MATERIALIZATION_TRANSIENT_PATH", path)


@dataclass(frozen=True, slots=True)
class MaterializationMember:
    """One immutable root, directory, or regular-file projection record."""

    path: str
    kind: str
    role: str
    mode: str
    size: int | None = None
    sha256: str | None = None

    def __post_init__(self) -> None:
        path = _safe_path(self.path, allow_root=True)
        _check_components(path)
        mode = _mode(self.mode, path)
        if type(self.kind) is not str or self.kind not in {"directory", "regular_file"}:
            _fail("MATERIALIZATION_MEMBER_KIND", path)
        if type(self.role) is not str:
            _fail("MATERIALIZATION_MEMBER_ROLE", path)
        if self.kind == "directory":
            expected_role = "root_directory" if path == "." else "directory"
            if self.role != expected_role or self.size is not None or self.sha256 is not None:
                _fail("MATERIALIZATION_DIRECTORY_RECORD", path)
        else:
            if path == ".":
                _fail("MATERIALIZATION_ROOT_RECORD", path)
            executable = bool(int(mode, 8) & 0o111)
            expected_role = "executable_regular_file" if executable else "regular_file"
            if self.role != expected_role:
                _fail("MATERIALIZATION_EXECUTABLE_ROLE", path)
            if type(self.size) is not int or self.size < 0:
                _fail("MATERIALIZATION_SIZE_INVALID", path)
            _digest(self.sha256, path)
        object.__setattr__(self, "path", path)
        object.__setattr__(self, "mode", mode)

    def to_dict(self) -> dict[str, object]:
        value: dict[str, object] = {
            "kind": self.kind,
            "mode": self.mode,
            "path": self.path,
            "role": self.role,
        }
        if self.kind == "regular_file":
            value["sha256"] = self.sha256
            value["size"] = self.size
        return value


@dataclass(frozen=True, slots=True)
class MaterializationProjection:
    """Complete, canonical materialization tree projection."""

    members: tuple[MaterializationMember, ...]
    schema_version: str = MATERIALIZATION_PROJECTION_SCHEMA

    def __post_init__(self) -> None:
        if (type(self.schema_version) is not str
                or self.schema_version != MATERIALIZATION_PROJECTION_SCHEMA):
            _fail("MATERIALIZATION_PROJECTION_SCHEMA")
        if type(self.members) not in (tuple, list) or not self.members:
            _fail("MATERIALIZATION_PROJECTION_MEMBERS")
        members = tuple(self.members)
        if any(type(member) is not MaterializationMember for member in members):
            _fail("MATERIALIZATION_PROJECTION_MEMBERS")
        paths = [member.path for member in members]
        if len(set(paths)) != len(paths):
            _fail("MATERIALIZATION_DUPLICATE_PATH")
        normalized = [unicodedata.normalize("NFC", path) for path in paths]
        if len(set(normalized)) != len(normalized):
            _fail("MATERIALIZATION_UNICODE_COLLISION")
        roots = [member for member in members if member.path == "."]
        if len(roots) != 1 or roots[0].kind != "directory":
            _fail("MATERIALIZATION_ROOT_RECORD")
        by_path = {member.path: member for member in members}
        for member in members:
            if member.path == ".":
                continue
            parent = PurePosixPath(member.path).parent.as_posix()
            parent = "." if parent == "." else parent
            parent_member = by_path.get(parent)
            if parent_member is None:
                _fail("MATERIALIZATION_PARENT_DIRECTORY_MISSING", member.path)
            if parent_member.kind != "directory":
                _fail("MATERIALIZATION_FILE_AS_PARENT", member.path)
        ordered = tuple(sorted(members, key=lambda member: member.path.encode("utf-8")))
        object.__setattr__(self, "members", ordered)

    def to_dict(self) -> dict[str, object]:
        return {
            "algorithms": dict(_ALGORITHMS),
            "canonicalization": dict(_CANONICALIZATION),
            "members": [member.to_dict() for member in self.members],
            "policies": dict(_POLICIES),
            "schema_version": self.schema_version,
        }


@dataclass(frozen=True, slots=True)
class MaterializationProjectionIdentityResult:
    """Projection-only identities; none of these fields assert physical proof."""

    projection: MaterializationProjection
    projection_size: int
    projection_sha256: str
    logical_identity: str
    projection_framing_digest: str
    regular_file_count: int
    directory_count: int
    regular_file_bytes: int

    def __post_init__(self) -> None:
        if type(self.projection) is not MaterializationProjection:
            _fail("MATERIALIZATION_RESULT_PROJECTION")
        for name in (
            "projection_sha256", "logical_identity", "projection_framing_digest",
        ):
            _digest(getattr(self, name), name)
        for name in ("projection_size", "regular_file_count", "directory_count", "regular_file_bytes"):
            value = getattr(self, name)
            if type(value) is not int or value < 0:
                _fail("MATERIALIZATION_RESULT_COUNT", name)


@dataclass(frozen=True, slots=True)
class MaterializationPhysicalVerificationResult:
    """Facts retained from a fresh physical traversal.

    Direct construction is data construction only.  Authority is granted by
    callers invoking :func:`verify_materialization_root` or
    :func:`verify_materialization_root_at` themselves.
    """

    projection_identity: MaterializationProjectionIdentityResult
    physical_rehash: str
    observed_root: tuple[int, ...]
    observed_members: tuple[tuple[str, tuple[int, ...]], ...]

    def __post_init__(self) -> None:
        if type(self.projection_identity) is not MaterializationProjectionIdentityResult:
            _fail("MATERIALIZATION_VERIFICATION_PROJECTION")
        _digest(self.physical_rehash, "physical_rehash")
        if type(self.observed_root) not in (tuple, list):
            _fail("MATERIALIZATION_VERIFICATION_OBSERVATIONS")
        root = tuple(self.observed_root)
        if not root or any(type(value) is not int for value in root):
            _fail("MATERIALIZATION_VERIFICATION_OBSERVATIONS")
        if type(self.observed_members) not in (tuple, list):
            _fail("MATERIALIZATION_VERIFICATION_OBSERVATIONS")
        members = []
        for item in self.observed_members:
            if type(item) not in (tuple, list) or len(item) != 2:
                _fail("MATERIALIZATION_VERIFICATION_OBSERVATIONS")
            path, metadata = item
            _safe_path(path, allow_root=True)
            _check_components(path)
            if type(metadata) not in (tuple, list) or any(
                type(value) is not int for value in metadata
            ):
                _fail("MATERIALIZATION_VERIFICATION_OBSERVATIONS", path)
            members.append((path, tuple(metadata)))
        members = tuple(sorted(members, key=lambda item: item[0].encode("utf-8")))
        if not members or members[0][0] != "." or len({p for p, _ in members}) != len(members):
            _fail("MATERIALIZATION_VERIFICATION_OBSERVATIONS")
        object.__setattr__(self, "observed_root", root)
        object.__setattr__(self, "observed_members", members)


def canonical_materialization_projection_bytes(
    projection: MaterializationProjection,
) -> bytes:
    if type(projection) is not MaterializationProjection:
        _fail("MATERIALIZATION_PROJECTION_TYPE")
    return _canonical(projection.to_dict())


def _json_object(raw: bytes) -> dict[str, object]:
    if type(raw) is not bytes:
        _fail("MATERIALIZATION_PROJECTION_BYTES")
    try:
        def hook(pairs):
            result = {}
            for key, value in pairs:
                if key in result:
                    _fail("MATERIALIZATION_JSON_DUPLICATE_KEY", str(key))
                result[key] = value
            return result
        value = json.loads(
            raw.decode("utf-8"), object_pairs_hook=hook,
            parse_constant=lambda value: _fail("MATERIALIZATION_JSON_CONSTANT", value),
        )
    except MaterializationIdentityError:
        raise
    except (UnicodeError, ValueError, TypeError) as exc:
        raise MaterializationIdentityError("MATERIALIZATION_JSON_INVALID") from exc
    if type(value) is not dict:
        _fail("MATERIALIZATION_JSON_TOP_LEVEL")
    return value


def materialization_projection_from_bytes(raw: bytes) -> MaterializationProjection:
    value = _json_object(raw)
    required = {"schema_version", "canonicalization", "algorithms", "policies", "members"}
    if set(value) != required or value.get("schema_version") != MATERIALIZATION_PROJECTION_SCHEMA:
        _fail("MATERIALIZATION_PROJECTION_SCHEMA")
    if value.get("canonicalization") != _CANONICALIZATION:
        _fail("MATERIALIZATION_CANONICALIZATION_CONTRACT")
    if value.get("algorithms") != _ALGORITHMS:
        _fail("MISSING_MATERIALIZATION_ALGORITHM")
    if value.get("policies") != _POLICIES or type(value.get("members")) is not list:
        _fail("MATERIALIZATION_PROJECTION_SCHEMA")
    raw_paths = []
    for record in value["members"]:
        if type(record) is dict and type(record.get("path")) is str:
            raw_paths.append(record["path"])
    if len({unicodedata.normalize("NFC", path) for path in raw_paths}) != len(raw_paths):
        _fail("MATERIALIZATION_UNICODE_COLLISION")
    members = []
    for record in value["members"]:
        if type(record) is not dict:
            _fail("MATERIALIZATION_MEMBER_SCHEMA")
        kind = record.get("kind")
        fields = {"kind", "mode", "path", "role"}
        if kind == "regular_file":
            fields |= {"size", "sha256"}
        if set(record) != fields:
            _fail("MATERIALIZATION_MEMBER_SCHEMA", str(record.get("path", "")))
        members.append(MaterializationMember(
            path=record["path"], kind=kind, role=record["role"],
            mode=record["mode"], size=record.get("size"),
            sha256=record.get("sha256"),
        ))
    projection = MaterializationProjection(tuple(members))
    if canonical_materialization_projection_bytes(projection) != raw:
        _fail("MATERIALIZATION_PROJECTION_NONCANONICAL")
    return projection


def materialization_root_identity(projection: MaterializationProjection) -> str:
    return sha256(_LOGICAL_DOMAIN + canonical_materialization_projection_bytes(projection)).hexdigest()


def materialization_projection_framing_digest(
    projection: MaterializationProjection,
) -> str:
    """Return the non-authoritative physical-algorithm framing of a projection."""

    if type(projection) is not MaterializationProjection:
        _fail("MATERIALIZATION_PROJECTION_TYPE")
    digest = sha256()
    digest.update(_PHYSICAL_DOMAIN)
    for member in projection.members:
        encoded = _canonical(member.to_dict())
        digest.update(len(encoded).to_bytes(8, "big"))
        digest.update(encoded)
    return digest.hexdigest()


def _stable_metadata(value: os.stat_result) -> tuple[int, ...]:
    return (
        value.st_dev, value.st_ino, stat.S_IFMT(value.st_mode), value.st_mode,
        value.st_nlink, value.st_size, value.st_mtime_ns, value.st_ctime_ns,
    )


def _visible_metadata(value: os.stat_result) -> tuple[int, int, int]:
    return stat.S_IFMT(value.st_mode), stat.S_IMODE(value.st_mode), value.st_size


@dataclass(frozen=True, slots=True)
class _Scan:
    projection: MaterializationProjection
    ephemeral: tuple[tuple[str, tuple[int, ...]], ...]


def _open_absolute_root_nofollow(root: Path) -> tuple[int, os.stat_result]:
    """Open every absolute root component with O_NOFOLLOW."""

    try:
        current_fd = os.open(
            "/", os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
            | getattr(os, "O_CLOEXEC", 0),
        )
    except OSError as exc:
        raise MaterializationIdentityError("MATERIALIZATION_ROOT_INVALID") from exc
    try:
        final_lstat = os.fstat(current_fd)
        for index, segment in enumerate(root.parts[1:], 1):
            display = "/".join(root.parts[1:index + 1])
            try:
                candidate = os.stat(
                    segment, dir_fd=current_fd, follow_symlinks=False,
                )
            except OSError as exc:
                raise MaterializationIdentityError(
                    "MATERIALIZATION_ROOT_INVALID", display,
                ) from exc
            if stat.S_ISLNK(candidate.st_mode):
                _fail("MATERIALIZATION_ROOT_SYMLINK", display)
            if not stat.S_ISDIR(candidate.st_mode):
                _fail("MATERIALIZATION_ROOT_INVALID", display)
            try:
                next_fd = os.open(
                    segment,
                    os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
                    | getattr(os, "O_CLOEXEC", 0),
                    dir_fd=current_fd,
                )
            except OSError as exc:
                raise MaterializationIdentityError(
                    "MATERIALIZATION_ROOT_INVALID", display,
                ) from exc
            opened = os.fstat(next_fd)
            if _stable_metadata(candidate) != _stable_metadata(opened):
                os.close(next_fd)
                _fail("MATERIALIZATION_INODE_SUBSTITUTION", display)
            os.close(current_fd)
            current_fd = next_fd
            final_lstat = candidate
        return current_fd, final_lstat
    except BaseException:
        os.close(current_fd)
        raise


def _open_relative_root_nofollow(
    parent_fd: int,
    relative_root: str,
) -> tuple[int, os.stat_result]:
    """Open a root beneath an already authenticated directory descriptor."""

    if type(parent_fd) is not int:
        _fail("MATERIALIZATION_PARENT_DESCRIPTOR")
    relative_root = _safe_path(relative_root)
    try:
        parent = os.fstat(parent_fd)
        if not stat.S_ISDIR(parent.st_mode):
            _fail("MATERIALIZATION_PARENT_DESCRIPTOR")
        current_fd = os.dup(parent_fd)
    except MaterializationIdentityError:
        raise
    except OSError as exc:
        raise MaterializationIdentityError("MATERIALIZATION_PARENT_DESCRIPTOR") from exc
    try:
        final_lstat = None
        traversed = []
        for segment in PurePosixPath(relative_root).parts:
            traversed.append(segment)
            display = "/".join(traversed)
            try:
                candidate = os.stat(
                    segment, dir_fd=current_fd, follow_symlinks=False,
                )
            except OSError as exc:
                raise MaterializationIdentityError(
                    "MATERIALIZATION_PHYSICAL_ROOT_MISSING", display,
                ) from exc
            if stat.S_ISLNK(candidate.st_mode):
                _fail("MATERIALIZATION_ROOT_SYMLINK", display)
            if not stat.S_ISDIR(candidate.st_mode):
                _fail("MATERIALIZATION_ROOT_INVALID", display)
            try:
                next_fd = os.open(
                    segment,
                    os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
                    | getattr(os, "O_CLOEXEC", 0),
                    dir_fd=current_fd,
                )
            except OSError as exc:
                raise MaterializationIdentityError(
                    "MATERIALIZATION_PHYSICAL_ROOT_MISSING", display,
                ) from exc
            opened = os.fstat(next_fd)
            if _stable_metadata(candidate) != _stable_metadata(opened):
                os.close(next_fd)
                _fail("MATERIALIZATION_INODE_SUBSTITUTION", display)
            os.close(current_fd)
            current_fd = next_fd
            final_lstat = candidate
        if final_lstat is None:
            _fail("MATERIALIZATION_PATH_INVALID", relative_root)
        return current_fd, final_lstat
    except BaseException:
        os.close(current_fd)
        raise


def _scan_from_opener(opener) -> _Scan:
    if not hasattr(os, "O_NOFOLLOW") or not hasattr(os, "O_DIRECTORY"):
        _fail("MATERIALIZATION_NOFOLLOW_UNAVAILABLE")
    root_fd, root_lstat = opener()
    members: list[MaterializationMember] = []
    ephemeral: dict[str, tuple[int, ...]] = {}
    inodes: set[tuple[int, int]] = set()
    try:
        root_stat = os.fstat(root_fd)
        if not stat.S_ISDIR(root_stat.st_mode):
            _fail("MATERIALIZATION_ROOT_INVALID", ".")
        if root_stat.st_mode & 0o222:
            _fail("MATERIALIZATION_WRITABLE_ROOT", ".")
        if _stable_metadata(root_lstat) != _stable_metadata(root_stat):
            _fail("MATERIALIZATION_INODE_SUBSTITUTION", ".")
        root_baseline = _stable_metadata(root_stat)
        inodes.add((root_stat.st_dev, root_stat.st_ino))
        ephemeral["."] = root_baseline
        members.append(MaterializationMember(
            ".", "directory", "root_directory",
            f"{stat.S_IMODE(root_stat.st_mode):04o}",
        ))

        def walk(directory_fd: int, prefix: str) -> None:
            try:
                names = os.listdir(directory_fd)
            except OSError as exc:
                raise MaterializationIdentityError(
                    "MATERIALIZATION_DIRECTORY_READ", prefix or ".",
                ) from exc
            normalized_names: dict[str, str] = {}
            for name in names:
                normalized = unicodedata.normalize("NFC", name)
                if normalized in normalized_names and normalized_names[normalized] != name:
                    _fail("MATERIALIZATION_UNICODE_COLLISION", prefix or ".")
                normalized_names[normalized] = name
            for name in sorted(names, key=lambda item: item.encode("utf-8")):
                rel = f"{prefix}/{name}" if prefix else name
                _safe_path(rel)
                _check_components(rel)
                try:
                    lst = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
                except OSError as exc:
                    raise MaterializationIdentityError(
                        "MATERIALIZATION_MEMBER_STAT", rel,
                    ) from exc
                if stat.S_ISLNK(lst.st_mode):
                    _fail("MATERIALIZATION_SYMLINK", rel)
                if not (stat.S_ISDIR(lst.st_mode) or stat.S_ISREG(lst.st_mode)):
                    _fail("MATERIALIZATION_UNSUPPORTED_FILE", rel)
                flags = os.O_RDONLY | os.O_NOFOLLOW | getattr(os, "O_CLOEXEC", 0)
                if stat.S_ISDIR(lst.st_mode):
                    flags |= os.O_DIRECTORY
                try:
                    fd = os.open(name, flags, dir_fd=directory_fd)
                except OSError as exc:
                    raise MaterializationIdentityError(
                        "MATERIALIZATION_MEMBER_OPEN", rel,
                    ) from exc
                try:
                    before = os.fstat(fd)
                    if _stable_metadata(lst) != _stable_metadata(before):
                        _fail("MATERIALIZATION_INODE_SUBSTITUTION", rel)
                    inode = (before.st_dev, before.st_ino)
                    if inode in inodes:
                        _fail("MATERIALIZATION_HARDLINK_ALIAS", rel)
                    inodes.add(inode)
                    baseline = _stable_metadata(before)
                    ephemeral[rel] = baseline
                    mode = f"{stat.S_IMODE(before.st_mode):04o}"
                    if stat.S_ISDIR(before.st_mode):
                        if before.st_mode & 0o222:
                            _fail("MATERIALIZATION_WRITABLE_DIRECTORY", rel)
                        members.append(MaterializationMember(
                            rel, "directory", "directory", mode,
                        ))
                        walk(fd, rel)
                        if _stable_metadata(os.fstat(fd)) != baseline:
                            _fail("MATERIALIZATION_CONCURRENT_MUTATION", rel)
                    elif stat.S_ISREG(before.st_mode):
                        if before.st_mode & 0o222:
                            _fail("MATERIALIZATION_WRITABLE_FILE", rel)
                        if before.st_nlink != 1:
                            _fail("MATERIALIZATION_HARDLINK_ALIAS", rel)
                        digest = sha256()
                        size = 0
                        while True:
                            chunk = os.read(fd, 1024 * 1024)
                            if not chunk:
                                break
                            digest.update(chunk)
                            size += len(chunk)
                        after = os.fstat(fd)
                        if _stable_metadata(after) != baseline or size != before.st_size:
                            _fail("MATERIALIZATION_CONCURRENT_MUTATION", rel)
                        role = (
                            "executable_regular_file" if before.st_mode & 0o111
                            else "regular_file"
                        )
                        members.append(MaterializationMember(
                            rel, "regular_file", role, mode,
                            before.st_size, digest.hexdigest(),
                        ))
                    else:
                        _fail("MATERIALIZATION_UNSUPPORTED_FILE", rel)
                finally:
                    os.close(fd)

        walk(root_fd, "")
        if _stable_metadata(os.fstat(root_fd)) != root_baseline:
            _fail("MATERIALIZATION_CONCURRENT_MUTATION", ".")
        verification_fd, pathname = opener()
        try:
            if (_stable_metadata(pathname) != root_baseline
                    or _stable_metadata(os.fstat(verification_fd)) != root_baseline):
                _fail("MATERIALIZATION_INODE_SUBSTITUTION", ".")
        finally:
            os.close(verification_fd)
        return _Scan(
            MaterializationProjection(tuple(members)),
            tuple(sorted(ephemeral.items(), key=lambda item: item[0].encode("utf-8"))),
        )
    finally:
        os.close(root_fd)


def _scan_materialization(root: Path) -> _Scan:
    return _scan_from_opener(lambda: _open_absolute_root_nofollow(root))


def _scan_materialization_at(parent_fd: int, relative_root: str) -> _Scan:
    return _scan_from_opener(
        lambda: _open_relative_root_nofollow(parent_fd, relative_root),
    )


def _authenticate_two_passes(scan_once) -> _Scan:
    first = scan_once()
    second = scan_once()
    if first.projection != second.projection:
        _fail("MATERIALIZATION_CONCURRENT_MUTATION", ".")
    if first.ephemeral != second.ephemeral:
        first_map, second_map = dict(first.ephemeral), dict(second.ephemeral)
        changed = next(
            (path for path in sorted(set(first_map) | set(second_map))
             if first_map.get(path) != second_map.get(path)),
            ".",
        )
        _fail("MATERIALIZATION_INODE_SUBSTITUTION", changed)
    return first


def build_materialization_projection(
    materialization_root: str | Path,
) -> MaterializationProjection:
    """Authenticate the complete tree twice and return its stable projection."""

    if type(materialization_root) not in (str, type(Path("."))):
        _fail("MATERIALIZATION_ROOT_TYPE")
    root = Path(materialization_root)
    if not root.is_absolute():
        _fail("MATERIALIZATION_ROOT_ABSOLUTE")
    return _authenticate_two_passes(lambda: _scan_materialization(root)).projection


def projection_identity_result(
    projection: MaterializationProjection,
) -> MaterializationProjectionIdentityResult:
    raw = canonical_materialization_projection_bytes(projection)
    files = [member for member in projection.members if member.kind == "regular_file"]
    directories = [member for member in projection.members if member.kind == "directory"]
    return MaterializationProjectionIdentityResult(
        projection=projection,
        projection_size=len(raw),
        projection_sha256=sha256(raw).hexdigest(),
        logical_identity=materialization_root_identity(projection),
        projection_framing_digest=materialization_projection_framing_digest(projection),
        regular_file_count=len(files),
        directory_count=len(directories),
        regular_file_bytes=sum(member.size for member in files),
    )


def _compare_projection(
    expected: MaterializationProjection,
    actual: MaterializationProjection,
) -> None:
    expected_by_path = {member.path: member for member in expected.members}
    actual_by_path = {member.path: member for member in actual.members}
    if set(expected_by_path) != set(actual_by_path):
        missing = sorted(set(expected_by_path) - set(actual_by_path))
        extra = sorted(set(actual_by_path) - set(expected_by_path))
        _fail(
            "MATERIALIZATION_INVENTORY_MISMATCH", ".",
            f"missing={missing!r};extra={extra!r}",
        )
    for path in sorted(expected_by_path, key=lambda item: item.encode("utf-8")):
        if expected_by_path[path] != actual_by_path[path]:
            _fail("MATERIALIZATION_PROJECTION_MISMATCH", path)


def _verification_result(scan: _Scan) -> MaterializationPhysicalVerificationResult:
    projection_result = projection_identity_result(scan.projection)
    return MaterializationPhysicalVerificationResult(
        projection_identity=projection_result,
        physical_rehash=materialization_projection_framing_digest(scan.projection),
        observed_root=dict(scan.ephemeral)["."],
        observed_members=scan.ephemeral,
    )


def verify_materialization_root(
    materialization_root: str | Path,
    projection: MaterializationProjection | bytes,
) -> MaterializationPhysicalVerificationResult:
    """Rebuild a physical tree and require complete equality with a projection."""

    expected = (
        materialization_projection_from_bytes(projection)
        if type(projection) is bytes else projection
    )
    if type(expected) is not MaterializationProjection:
        _fail("MATERIALIZATION_PROJECTION_TYPE")
    if type(materialization_root) not in (str, type(Path("."))):
        _fail("MATERIALIZATION_ROOT_TYPE")
    root = Path(materialization_root)
    if not root.is_absolute():
        _fail("MATERIALIZATION_ROOT_ABSOLUTE")
    scan = _authenticate_two_passes(lambda: _scan_materialization(root))
    _compare_projection(expected, scan.projection)
    return _verification_result(scan)


def verify_materialization_root_at(
    parent_fd: int,
    relative_root: str,
    projection: MaterializationProjection | bytes,
) -> MaterializationPhysicalVerificationResult:
    """Physically verify a root beneath an authenticated parent descriptor."""

    expected = (
        materialization_projection_from_bytes(projection)
        if type(projection) is bytes else projection
    )
    if type(expected) is not MaterializationProjection:
        _fail("MATERIALIZATION_PROJECTION_TYPE")
    relative_root = _safe_path(relative_root)
    scan = _authenticate_two_passes(
        lambda: _scan_materialization_at(parent_fd, relative_root),
    )
    _compare_projection(expected, scan.projection)
    return _verification_result(scan)


def complete_physical_rehash(
    materialization_root: str | Path,
    projection: MaterializationProjection | bytes | None = None,
) -> str:
    """Return a physical rehash only after authenticating ``materialization_root``."""

    if projection is None:
        _fail("MATERIALIZATION_PHYSICAL_PROVENANCE_REQUIRED")
    return verify_materialization_root(materialization_root, projection).physical_rehash


def verify_materialization_projection(
    materialization_root: str | Path,
    projection: MaterializationProjection | bytes,
) -> MaterializationPhysicalVerificationResult:
    """Compatibility spelling for the root-authoritative verifier."""

    return verify_materialization_root(materialization_root, projection)


def runtime_member_bindings(
    projection: MaterializationProjection,
    runtime_members: Iterable[object],
    *,
    materialization_runtime_root: str = "runtime_root",
) -> tuple[tuple[str, str, int, str, str], ...]:
    """Bind runtime member objects to materialization file facts.

    Runtime objects must expose ``path``, ``size_bytes``, ``sha256`` and
    ``mode``.  The returned tuples are stable and contain the runtime path,
    materialization path, size, digest and mode.
    """

    root = _safe_path(materialization_runtime_root)
    files = {
        member.path: member for member in projection.members
        if member.kind == "regular_file"
    }
    result = []
    seen = set()
    for runtime in runtime_members:
        try:
            runtime_path = _safe_path(runtime.path)
            size = runtime.size_bytes
            digest = runtime.sha256
            mode = runtime.mode
        except (AttributeError, TypeError) as exc:
            raise MaterializationIdentityError(
                "MATERIALIZATION_RUNTIME_MEMBER_INVALID",
            ) from exc
        projected_path = f"{root}/{runtime_path}"
        if runtime_path in seen:
            _fail("MATERIALIZATION_RUNTIME_DUPLICATE", runtime_path)
        seen.add(runtime_path)
        member = files.get(projected_path)
        if member is None:
            _fail("MATERIALIZATION_RUNTIME_MEMBER_MISSING", projected_path)
        if (member.size, member.sha256, member.mode) != (size, digest, mode):
            _fail("MATERIALIZATION_RUNTIME_MEMBER_MISMATCH", projected_path)
        result.append((runtime_path, projected_path, size, digest, mode))
    return tuple(sorted(result))


__all__ = [
    "LOGICAL_ALGORITHM_ID",
    "MATERIALIZATION_PROJECTION_SCHEMA",
    "PHYSICAL_REHASH_ALGORITHM_ID",
    "PROJECTION_FRAMING_ALGORITHM_ID",
    "SUPERSEDED_HISTORICAL_IDENTITIES",
    "MaterializationIdentityError",
    "MaterializationPhysicalVerificationResult",
    "MaterializationMember",
    "MaterializationProjection",
    "MaterializationProjectionIdentityResult",
    "build_materialization_projection",
    "canonical_materialization_projection_bytes",
    "complete_physical_rehash",
    "materialization_projection_framing_digest",
    "materialization_projection_from_bytes",
    "materialization_root_identity",
    "projection_identity_result",
    "runtime_member_bindings",
    "verify_materialization_projection",
    "verify_materialization_root",
    "verify_materialization_root_at",
]
