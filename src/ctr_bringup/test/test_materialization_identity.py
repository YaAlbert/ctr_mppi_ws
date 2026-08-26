import gc
import json
import os
import stat
import sys
from dataclasses import replace
from hashlib import sha256
from pathlib import Path, PurePath

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import ctr_bringup.materialization_identity as identity_module
from ctr_bringup.materialization_identity import (
    LOGICAL_ALGORITHM_ID,
    MATERIALIZATION_PROJECTION_SCHEMA,
    PHYSICAL_REHASH_ALGORITHM_ID,
    MaterializationIdentityError,
    MaterializationMember,
    MaterializationPhysicalVerificationResult,
    MaterializationProjection,
    build_materialization_projection,
    canonical_materialization_projection_bytes,
    complete_physical_rehash,
    materialization_projection_framing_digest,
    materialization_projection_from_bytes,
    materialization_root_identity,
    projection_identity_result,
    verify_materialization_projection,
    verify_materialization_root,
    verify_materialization_root_at,
)


def _canonical(value):
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
    ).encode("utf-8")


def _tree(tmp_path):
    root = tmp_path / "materialization"
    (root / "empty").mkdir(parents=True)
    (root / "bin").mkdir()
    (root / "data").mkdir()
    (root / "bin" / "entry").write_bytes(b"#!/bin/sh\nexit 0\n")
    (root / "data" / "value.txt").write_bytes(b"value\n")
    for path in sorted(root.rglob("*"), key=lambda item: len(item.parts), reverse=True):
        path.chmod(0o555 if path.is_dir() or path.name == "entry" else 0o444)
    root.chmod(0o555)
    return root


def _error(code, call):
    with pytest.raises(MaterializationIdentityError) as caught:
        call()
    assert caught.value.code == code
    return caught.value


def _make_writable(root):
    root.chmod(0o755)
    for path in root.rglob("*"):
        path.chmod(0o755 if path.is_dir() else 0o644)


def _refreeze(root):
    for path in sorted(root.rglob("*"), key=lambda item: len(item.parts), reverse=True):
        executable = path.is_file() and path.name == "entry"
        path.chmod(0o555 if path.is_dir() or executable else 0o444)
    root.chmod(0o555)


def test_stable_two_pass_projection_and_counts(tmp_path):
    root = _tree(tmp_path)
    first = build_materialization_projection(root)
    second = build_materialization_projection(root)
    assert first == second
    result = projection_identity_result(first)
    assert result.regular_file_count == 2
    assert result.directory_count == 4
    assert result.regular_file_bytes == 23
    assert first.members[0].path == "."
    assert any(member.path == "empty" for member in first.members)


def test_logical_and_physical_algorithms_are_distinct_and_domain_separated(tmp_path):
    root = _tree(tmp_path)
    projection = build_materialization_projection(root)
    logical = materialization_root_identity(projection)
    framing = materialization_projection_framing_digest(projection)
    physical = complete_physical_rehash(root, projection)
    assert framing == physical
    assert logical != physical
    assert LOGICAL_ALGORITHM_ID != PHYSICAL_REHASH_ALGORITHM_ID
    assert len(logical) == len(physical) == 64


def test_known_answer_canonicalization():
    projection = MaterializationProjection((
        MaterializationMember(".", "directory", "root_directory", "0555"),
        MaterializationMember(
            "a.txt", "regular_file", "regular_file", "0444", 1,
            sha256(b"a").hexdigest(),
        ),
    ))
    raw = canonical_materialization_projection_bytes(projection)
    assert raw == _canonical(projection.to_dict())
    assert not raw.endswith(b"\n")
    assert sha256(raw).hexdigest() == "4430c9c324c4cacd2b091714ebd2d07fc15c5675a86fbdf1d3504d0faf5f0460"
    assert materialization_root_identity(projection) == "9b1e53f3c395f07e1b999b707d6a8f8d62d68113df6c9d86ff739158ab5dcc5f"
    assert materialization_projection_framing_digest(projection) == "ddefab55a05c961450661d8ebd0a7e2bb2cd1f3eefa87844e1db7ea610422e9a"


def test_projection_round_trip_is_exact(tmp_path):
    projection = build_materialization_projection(_tree(tmp_path))
    raw = canonical_materialization_projection_bytes(projection)
    loaded = materialization_projection_from_bytes(raw)
    assert loaded == projection
    assert canonical_materialization_projection_bytes(loaded) == raw


@pytest.mark.parametrize(
    "mutation,code",
    [
        (lambda root: (root / "data" / "value.txt").write_bytes(b"other\n"), "MATERIALIZATION_PROJECTION_MISMATCH"),
        (lambda root: ((root / "data" / "value.txt").chmod(0o555), "mode")[1], "MATERIALIZATION_PROJECTION_MISMATCH"),
        (lambda root: (root / "data" / "value.txt").rename(root / "data" / "renamed.txt"), "MATERIALIZATION_INVENTORY_MISMATCH"),
        (lambda root: (root / "added").mkdir(), "MATERIALIZATION_INVENTORY_MISMATCH"),
        (lambda root: (root / "empty").rmdir(), "MATERIALIZATION_INVENTORY_MISMATCH"),
    ],
)
def test_physical_mutations_are_detected(tmp_path, mutation, code):
    root = _tree(tmp_path)
    expected = build_materialization_projection(root)
    _make_writable(root)
    mutation_kind = mutation(root)
    _refreeze(root)
    if mutation_kind == "mode":
        (root / "data" / "value.txt").chmod(0o555)
    _error(code, lambda: verify_materialization_projection(root, expected))


def test_duplicate_path_is_rejected():
    member = MaterializationMember(".", "directory", "root_directory", "0555")
    _error(
        "MATERIALIZATION_DUPLICATE_PATH",
        lambda: MaterializationProjection((member, member)),
    )


def test_unicode_collision_is_rejected_before_member_acceptance():
    base = MaterializationProjection((
        MaterializationMember(".", "directory", "root_directory", "0555"),
        MaterializationMember(
            "é", "regular_file", "regular_file", "0444", 0,
            sha256(b"").hexdigest(),
        ),
    )).to_dict()
    duplicate = dict(base["members"][1])
    duplicate["path"] = "e\u0301"
    base["members"].append(duplicate)
    _error(
        "MATERIALIZATION_UNICODE_COLLISION",
        lambda: materialization_projection_from_bytes(_canonical(base)),
    )


def test_root_symlink_is_rejected(tmp_path):
    root = _tree(tmp_path)
    link = tmp_path / "root-link"
    link.symlink_to(root, target_is_directory=True)
    _error("MATERIALIZATION_ROOT_SYMLINK", lambda: build_materialization_projection(link))


def test_root_intermediate_symlink_is_rejected(tmp_path):
    real_parent = tmp_path / "real-parent"; real_parent.mkdir()
    root = _tree(real_parent)
    linked_parent = tmp_path / "linked-parent"
    linked_parent.symlink_to(real_parent, target_is_directory=True)
    _error(
        "MATERIALIZATION_ROOT_SYMLINK",
        lambda: build_materialization_projection(linked_parent / root.name),
    )


@pytest.mark.parametrize("kind", ["intermediate", "member"])
def test_member_symlinks_are_rejected(tmp_path, kind):
    root = _tree(tmp_path)
    _make_writable(root)
    if kind == "intermediate":
        (root / "linked-dir").symlink_to(root / "data", target_is_directory=True)
    else:
        (root / "linked-file").symlink_to(root / "data" / "value.txt")
    _refreeze(root)
    _error("MATERIALIZATION_SYMLINK", lambda: build_materialization_projection(root))


def test_hardlink_alias_is_rejected(tmp_path):
    root = _tree(tmp_path)
    _make_writable(root)
    os.link(root / "data" / "value.txt", root / "data" / "alias.txt")
    _refreeze(root)
    _error("MATERIALIZATION_HARDLINK_ALIAS", lambda: build_materialization_projection(root))


@pytest.mark.parametrize("kind", ["root", "directory", "file"])
def test_writable_paths_are_rejected(tmp_path, kind):
    root = _tree(tmp_path)
    target = {"root": root, "directory": root / "data", "file": root / "data" / "value.txt"}[kind]
    target.chmod(stat.S_IMODE(target.stat().st_mode) | 0o200)
    code = {
        "root": "MATERIALIZATION_WRITABLE_ROOT",
        "directory": "MATERIALIZATION_WRITABLE_DIRECTORY",
        "file": "MATERIALIZATION_WRITABLE_FILE",
    }[kind]
    _error(code, lambda: build_materialization_projection(root))


def test_unsupported_fifo_is_rejected(tmp_path):
    root = _tree(tmp_path)
    _make_writable(root)
    os.mkfifo(root / "pipe", 0o444)
    _refreeze(root)
    _error("MATERIALIZATION_UNSUPPORTED_FILE", lambda: build_materialization_projection(root))


def test_transient_and_cache_paths_are_rejected(tmp_path):
    root = _tree(tmp_path)
    _make_writable(root)
    (root / "__pycache__").mkdir()
    _refreeze(root)
    _error("MATERIALIZATION_TRANSIENT_PATH", lambda: build_materialization_projection(root))


def test_concurrent_content_mutation_between_passes_is_detected(tmp_path, monkeypatch):
    root = _tree(tmp_path)
    original = identity_module._scan_materialization
    calls = 0

    def mutate_after_first(path):
        nonlocal calls
        result = original(path)
        calls += 1
        if calls == 1:
            target = root / "data" / "value.txt"
            root.chmod(0o755); (root / "data").chmod(0o755); target.chmod(0o644)
            target.write_bytes(b"mutated\n")
            _refreeze(root)
        return result

    monkeypatch.setattr(identity_module, "_scan_materialization", mutate_after_first)
    _error("MATERIALIZATION_CONCURRENT_MUTATION", lambda: build_materialization_projection(root))


def test_inode_substitution_between_passes_is_detected(tmp_path, monkeypatch):
    root = _tree(tmp_path)
    original = identity_module._scan_materialization
    calls = 0

    def replace_after_first(path):
        nonlocal calls
        result = original(path)
        calls += 1
        if calls == 1:
            target = root / "data" / "value.txt"
            raw = target.read_bytes()
            root.chmod(0o755); (root / "data").chmod(0o755)
            replacement = root / "data" / "replacement"
            replacement.write_bytes(raw); replacement.chmod(0o444)
            os.replace(replacement, target)
            _refreeze(root)
        return result

    monkeypatch.setattr(identity_module, "_scan_materialization", replace_after_first)
    _error("MATERIALIZATION_INODE_SUBSTITUTION", lambda: build_materialization_projection(root))


def test_missing_and_extra_projection_members_are_distinguished(tmp_path):
    root = _tree(tmp_path)
    projection = build_materialization_projection(root)
    missing = replace(projection, members=projection.members[:-1])
    _error("MATERIALIZATION_INVENTORY_MISMATCH", lambda: verify_materialization_projection(root, missing))
    extra_member = MaterializationMember(
        "never-present", "regular_file", "regular_file", "0444", 0,
        sha256(b"").hexdigest(),
    )
    extra = replace(projection, members=projection.members + (extra_member,))
    _error("MATERIALIZATION_INVENTORY_MISMATCH", lambda: verify_materialization_projection(root, extra))


def test_all_descriptors_close_on_success_and_failure(tmp_path):
    root = _tree(tmp_path)
    before = len(os.listdir("/proc/self/fd"))
    build_materialization_projection(root)
    gc.collect()
    assert len(os.listdir("/proc/self/fd")) == before
    _make_writable(root)
    (root / "bad-link").symlink_to(root / "data" / "value.txt")
    _refreeze(root)
    _error("MATERIALIZATION_SYMLINK", lambda: build_materialization_projection(root))
    gc.collect()
    assert len(os.listdir("/proc/self/fd")) == before


def test_projection_contract_ids_are_authenticated(tmp_path):
    projection = build_materialization_projection(_tree(tmp_path))
    value = projection.to_dict()
    assert value["schema_version"] == MATERIALIZATION_PROJECTION_SCHEMA
    assert value["algorithms"] == {
        "logical_identity": LOGICAL_ALGORITHM_ID,
        "physical_rehash": PHYSICAL_REHASH_ALGORITHM_ID,
    }
    value["algorithms"]["logical_identity"] = "unknown"
    _error(
        "MISSING_MATERIALIZATION_ALGORITHM",
        lambda: materialization_projection_from_bytes(_canonical(value)),
    )


def test_projection_only_api_cannot_claim_physical_authority(tmp_path):
    projection = build_materialization_projection(_tree(tmp_path))
    result = projection_identity_result(projection)
    assert not hasattr(result, "physical_rehash")
    assert result.projection_framing_digest == materialization_projection_framing_digest(projection)
    _error(
        "MATERIALIZATION_PHYSICAL_PROVENANCE_REQUIRED",
        lambda: complete_physical_rehash(projection),
    )


def test_caller_constructed_verification_result_is_not_accepted_as_authority(tmp_path):
    root = _tree(tmp_path)
    projection = build_materialization_projection(root)
    projection_result = projection_identity_result(projection)
    fake = MaterializationPhysicalVerificationResult(
        projection_result,
        "0" * 64,
        (1,),
        ((".", (1,)),),
    )
    _error(
        "MATERIALIZATION_PROJECTION_TYPE",
        lambda: verify_materialization_root(root, fake),
    )


@pytest.mark.parametrize(
    "path",
    [
        "ghost.pyc", "ghost.pyo", "__pycache__", "__pycache__/ghost",
        ".pytest_cache/state", ".mypy_cache/state", ".ruff_cache/state",
        ".coverage", ".coverage.worker", "nested/.pytest_cache/state",
    ],
)
def test_transient_paths_rejected_during_direct_record_construction(path):
    kind = "directory" if PurePath(path).name in {"__pycache__", ".coverage"} else "regular_file"
    role = "directory" if kind == "directory" else "regular_file"
    args = () if kind == "directory" else (0, sha256(b"").hexdigest())
    _error(
        "MATERIALIZATION_TRANSIENT_PATH",
        lambda: MaterializationMember(path, kind, role, "0555" if kind == "directory" else "0444", *args),
    )


def test_transient_path_rejected_during_projection_byte_parsing():
    value = MaterializationProjection((
        MaterializationMember(".", "directory", "root_directory", "0555"),
    )).to_dict()
    value["members"].append({
        "kind": "regular_file", "mode": "0444", "path": "__pycache__/ghost.pyc",
        "role": "regular_file", "sha256": sha256(b"").hexdigest(), "size": 0,
    })
    _error(
        "MATERIALIZATION_TRANSIENT_PATH",
        lambda: materialization_projection_from_bytes(_canonical(value)),
    )


def test_projection_topology_requires_directory_parents():
    root = MaterializationMember(".", "directory", "root_directory", "0555")
    orphan = MaterializationMember(
        "missing/child", "regular_file", "regular_file", "0444", 0,
        sha256(b"").hexdigest(),
    )
    _error(
        "MATERIALIZATION_PARENT_DIRECTORY_MISSING",
        lambda: MaterializationProjection((root, orphan)),
    )
    file_parent = MaterializationMember(
        "parent", "regular_file", "regular_file", "0444", 0,
        sha256(b"").hexdigest(),
    )
    child = MaterializationMember(
        "parent/child", "regular_file", "regular_file", "0444", 0,
        sha256(b"").hexdigest(),
    )
    _error(
        "MATERIALIZATION_FILE_AS_PARENT",
        lambda: MaterializationProjection((root, file_parent, child)),
    )


def test_sorted_caller_list_and_nested_outputs_are_detached():
    members = [
        MaterializationMember(".", "directory", "root_directory", "0555"),
        MaterializationMember("empty", "directory", "directory", "0555"),
    ]
    projection = MaterializationProjection(members)
    raw = canonical_materialization_projection_bytes(projection)
    logical = materialization_root_identity(projection)
    framing = materialization_projection_framing_digest(projection)
    members.append(MaterializationMember("other", "directory", "directory", "0555"))
    members.reverse()
    members.pop()
    exported = projection.to_dict()
    exported["members"].append({"path": "attacker"})
    exported["algorithms"]["logical_identity"] = "attacker"
    assert type(projection.members) is tuple
    assert canonical_materialization_projection_bytes(projection) == raw
    assert materialization_root_identity(projection) == logical
    assert materialization_projection_framing_digest(projection) == framing


def test_descriptor_relative_verification_and_missing_root(tmp_path):
    root = _tree(tmp_path)
    projection = build_materialization_projection(root)
    parent_fd = os.open(tmp_path, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        result = verify_materialization_root_at(parent_fd, root.name, projection)
        assert result.physical_rehash == materialization_projection_framing_digest(projection)
        _error(
            "MATERIALIZATION_PHYSICAL_ROOT_MISSING",
            lambda: verify_materialization_root_at(parent_fd, "missing", projection),
        )
    finally:
        os.close(parent_fd)


def test_descriptor_relative_materialization_root_symlink_is_rejected(tmp_path):
    root = _tree(tmp_path)
    projection = build_materialization_projection(root)
    link = tmp_path / "material-link"
    link.symlink_to(root, target_is_directory=True)
    parent_fd = os.open(tmp_path, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        _error(
            "MATERIALIZATION_ROOT_SYMLINK",
            lambda: verify_materialization_root_at(parent_fd, link.name, projection),
        )
    finally:
        os.close(parent_fd)
