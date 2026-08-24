"""Focused, ROS-independent tests for production runtime-plan validation."""

from __future__ import annotations

from dataclasses import FrozenInstanceError
from hashlib import sha256
from pathlib import Path
from copy import deepcopy
from contextlib import contextmanager

import json
import os
import sys
import gc
import threading
import weakref

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

import ctr_bringup.runtime_plan_validation as runtime_validation  # noqa: E402
from ctr_bringup.runtime_plan_validation import (  # noqa: E402
    AUTHENTICATED_RUNTIME_ROOT,
    PLAN_SCHEMA_VERSION,
    PROJECTION_SCHEMA_VERSION,
    RuntimeDependency,
    RuntimeDependencyClosure,
    RuntimeArgvClassification,
    RuntimeArgvBinding,
    RuntimeExternalCommand,
    RuntimeMember,
    RuntimePlan,
    RuntimePlanPolicy,
    RuntimeProjection,
    RuntimeProjectionReconciliation,
    RuntimeReconciliation,
    RuntimeIssue,
    RuntimeValidationError,
    canonical_runtime_projection_bytes,
    load_runtime_plan,
    load_runtime_projection,
    reconcile_runtime_projection,
    runtime_projection_identity,
    validate_runtime_dependency_closure,
    validate_runtime_plan,
    validate_six_plan_set,
    open_authenticated_runtime_snapshot,
)


def json_bytes(value):
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def write_runtime(root: Path) -> dict[str, str]:
    files = {
        "config/robot.yaml": "robot:\n  name: ctr\n",
        "launch/simulation.launch.py": "from pkg.main import main\n",
        "pkg/main.py": "def main():\n    return 0\n",
    }
    for relative, content in files.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    return files


def projection_dict(root: Path, paths: tuple[str, ...] | None = None):
    roles = {
        "config/robot.yaml": "configuration",
        "launch/simulation.launch.py": "launch_file",
        "pkg/main.py": "python_module",
    }
    selected = paths or tuple(sorted(roles))
    members = []
    for relative in sorted(selected):
        path = root / relative
        data = path.read_bytes()
        members.append(
            {
                "path": relative,
                "size_bytes": len(data),
                "sha256": sha256(data).hexdigest(),
                "mode": "0444",
                "role": roles.get(relative, "runtime_resource"),
            }
        )
    return {"schema_version": PROJECTION_SCHEMA_VERSION, "members": members}


def valid_plan(identity: str, mode: str = "production"):
    return {
        "schema_version": PLAN_SCHEMA_VERSION,
        "mode": mode,
        "production_runtime_identity": identity,
        "runtime_root_role": AUTHENTICATED_RUNTIME_ROOT,
        "prospective_argv": ["ros2", "launch/simulation.launch.py", "--domain", "232"],
        "project_owned_argv_indices": [1],
        "argv_bindings": [{"argv_index": 1, "member_path": "launch/simulation.launch.py"}],
        "external_commands": [{"argv_index": 0, "command": "ros2", "dependency": "ros2"}],
        "argv_classifications": [
            {"argv_index": 0, "kind": "external_command", "value": "ros2", "dependency": "ros2"},
            {
                "argv_index": 1,
                "kind": "project_member",
                "value": "launch/simulation.launch.py",
                "member_path": "launch/simulation.launch.py",
            },
            {"argv_index": 2, "kind": "flag", "value": "--domain"},
            {"argv_index": 3, "kind": "integer", "value": "232"},
        ],
        "prospective_environment": {
            "RMW_IMPLEMENTATION": "prospective_only",
            "ROS_DOMAIN_ID": "232",
        },
        "external_dependencies": ["launch_ros", "ros2"],
        "policy": {
            "validate_only": True,
            "allow_full_launch": False,
            "launchable": False,
            "execution_authorized": False,
        },
    }


def runtime_fixture(tmp_path: Path):
    root = tmp_path / "runtime"
    root.mkdir()
    write_runtime(root)
    projection = projection_dict(root)
    identity = runtime_projection_identity(projection)
    return root, projection, identity


@contextmanager
def immutable_runtime(root: Path):
    entries = [root, *sorted(root.rglob("*"), key=lambda path: (len(path.parts), path.as_posix()))]
    original = {}
    for path in entries:
        if path.is_symlink():
            continue
        original[path] = path.stat().st_mode & 0o7777
    try:
        for path in sorted(original, key=lambda item: len(item.parts), reverse=True):
            path.chmod(0o555 if path.is_dir() else 0o444)
        yield
    finally:
        for path in sorted(original, key=lambda item: len(item.parts)):
            if path.exists() and not path.is_symlink():
                path.chmod(original[path])


_raw_reconcile_runtime_projection = reconcile_runtime_projection
_raw_validate_runtime_plan = validate_runtime_plan
_raw_validate_six_plan_set = validate_six_plan_set


def reconcile_runtime_projection(projection, root, **kwargs):
    with immutable_runtime(Path(root)):
        return _raw_reconcile_runtime_projection(projection, root, **kwargs)


def validate_runtime_plan(plan, projection, root, **kwargs):
    kwargs.setdefault("allowed_external_dependencies", {"launch_ros", "ros2"})
    with immutable_runtime(Path(root)):
        diagnostic = _raw_reconcile_runtime_projection(projection, root, complete_inventory=False)
        if diagnostic.issues:
            first = diagnostic.issues[0]
            bound = str(plan.get("argv_bindings", [{}])[0].get("member_path", "")) if isinstance(plan, dict) else ""
            if bound.startswith("escape/") or first.code in {"PHYSICAL_SYMLINK", "RUNTIME_ROOT_ESCAPE"} or "Errno 40" in (first.observed or ""):
                raise RuntimeValidationError("PLAN_BINDING_ROOT_ESCAPE", "bound member escapes runtime root", path=first.path)
            raise RuntimeValidationError("PLAN_BINDING_PHYSICAL_MISSING", "bound member missing", path=first.path)
        try:
            with open_authenticated_runtime_snapshot(projection, root, complete_inventory=False) as snapshot:
                return _raw_validate_runtime_plan(plan, projection, snapshot, **kwargs)
        except RuntimeValidationError as exc:
            if exc.code in {"PHYSICAL_OPEN_ERROR", "PHYSICAL_MEMBER_MISSING"}:
                raise RuntimeValidationError("PLAN_BINDING_PHYSICAL_MISSING", "bound member missing", path=exc.path) from exc
            if exc.code == "PHYSICAL_SYMLINK":
                raise RuntimeValidationError("PLAN_BINDING_ROOT_ESCAPE", "bound member escapes runtime root", path=exc.path) from exc
            raise


def validate_six_plan_set(plans, projection, root, **kwargs):
    kwargs.setdefault("allowed_external_dependencies", {"launch_ros", "ros2"})
    with immutable_runtime(Path(root)):
        with open_authenticated_runtime_snapshot(projection, root, complete_inventory=False) as snapshot:
            if snapshot.issues:
                first = snapshot.issues[0]
                raise RuntimeValidationError(first.code, "snapshot authentication failed", path=first.path)
            return _raw_validate_six_plan_set(plans, projection, snapshot, **kwargs)


def assert_error(code, function, *args, **kwargs):
    with pytest.raises(RuntimeValidationError) as captured:
        function(*args, **kwargs)
    assert captured.value.code == code
    return captured.value


def issue_codes(result):
    return {issue.code for issue in result.issues}


def valid_graph(projection):
    return validate_runtime_dependency_closure(
        projection,
        entrypoints=["launch/simulation.launch.py"],
        project_nodes=["launch/simulation.launch.py", "pkg/main.py", "config/robot.yaml"],
        dependencies=[
            RuntimeDependency("launch/simulation.launch.py", "pkg/main.py"),
            RuntimeDependency("launch/simulation.launch.py", "config/robot.yaml"),
            RuntimeDependency("launch/simulation.launch.py", "launch_ros", "external"),
        ],
        declared_external_dependencies=["launch_ros"],
    )


def six_plans(identity):
    result = {}
    for mode in ("production", "offline", "test_only"):
        raw = json_bytes(valid_plan(identity, mode))
        result[f"{mode}_root"] = raw
        result[f"{mode}_duplicate"] = raw
    return result


# Positive contract tests ---------------------------------------------------


def test_canonical_projection_round_trip_and_no_trailing_newline(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    raw = canonical_runtime_projection_bytes(projection)
    assert not raw.endswith(b"\n")
    parsed = load_runtime_projection(raw)
    assert canonical_runtime_projection_bytes(parsed) == raw
    assert reconcile_runtime_projection(parsed, root).issues == ()


def test_projection_identity_is_deterministic_for_mapping_key_order(tmp_path):
    _, projection, identity = runtime_fixture(tmp_path)
    reordered = {"members": deepcopy(projection["members"]), "schema_version": PROJECTION_SCHEMA_VERSION}
    assert runtime_projection_identity(reordered) == identity
    assert runtime_projection_identity(canonical_runtime_projection_bytes(reordered)) == identity


def test_valid_physical_reconciliation(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    result = reconcile_runtime_projection(projection, root, complete_inventory=True)
    assert result.declared_count == 3
    assert result.physical_regular_file_count == 3
    assert result.matched_count == 3
    assert result.issues == ()


def test_valid_dependency_graph_and_deterministic_cycle(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    assert root.is_dir()
    result = valid_graph(projection)
    assert result.issues == ()
    cyclic = validate_runtime_dependency_closure(
        projection,
        entrypoints=["launch/simulation.launch.py"],
        project_nodes=["pkg/main.py", "config/robot.yaml", "launch/simulation.launch.py"],
        dependencies=[
            RuntimeDependency("pkg/main.py", "launch/simulation.launch.py"),
            RuntimeDependency("launch/simulation.launch.py", "config/robot.yaml"),
            RuntimeDependency("launch/simulation.launch.py", "pkg/main.py"),
        ],
        declared_external_dependencies=[],
    )
    assert cyclic.issues == ()
    assert cyclic.reachable_members == (
        "config/robot.yaml",
        "launch/simulation.launch.py",
        "pkg/main.py",
    )


@pytest.mark.parametrize("mode", ["production", "offline", "test_only"])
def test_valid_runtime_plan_modes(tmp_path, mode):
    root, projection, identity = runtime_fixture(tmp_path)
    parsed = validate_runtime_plan(
        valid_plan(identity, mode),
        projection,
        root,
        allowed_external_dependencies={"launch_ros", "ros2"},
    )
    assert parsed.mode == mode
    assert parsed.production_runtime_identity == identity


def test_valid_six_plan_set_and_byte_identical_duplicates(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    result = validate_six_plan_set(
        six_plans(identity),
        projection,
        root,
        allowed_external_dependencies=["ros2", "launch_ros"],
    )
    assert result.runtime_identity == identity
    assert len(result.roles) == 6
    assert len(result.plan_sha256) == 6


def test_immutable_records_and_environment_mapping(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    member = load_runtime_projection(projection).members[0]
    with pytest.raises(FrozenInstanceError):
        member.path = "changed"  # type: ignore[misc]
    plan = validate_runtime_plan(valid_plan(identity), projection, root)
    with pytest.raises(TypeError):
        plan.environment["ROS_DOMAIN_ID"] = "1"  # type: ignore[index]


def test_caller_inputs_remain_unchanged(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    projection_before = deepcopy(projection)
    plan = valid_plan(identity)
    plan_before = deepcopy(plan)
    validate_runtime_plan(plan, projection, root)
    valid_graph(projection)
    assert projection == projection_before
    assert plan == plan_before


def test_diagnostic_stale_identity_is_nonoperative(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    stale = "f" * 64
    plan = valid_plan(identity)
    plan["diagnostic_lineage"] = {"identities": [stale]}
    assert validate_runtime_plan(plan, projection, root, forbidden_identities=[stale]).diagnostic_lineage == (stale,)


# Projection and JSON rejection tests --------------------------------------


def test_rejects_malformed_json():
    assert_error("JSON_MALFORMED", load_runtime_projection, b"{")


def test_rejects_duplicate_json_keys():
    raw = b'{"schema_version":"ctr-runtime-projection-1","schema_version":"x","members":[]}'
    assert_error("JSON_DUPLICATE_KEY", load_runtime_projection, raw)


def test_rejects_unsupported_projection_version(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    assert root.exists()
    projection["schema_version"] = "future"
    assert_error("PROJECTION_UNSUPPORTED_VERSION", load_runtime_projection, projection)


def test_rejects_missing_projection_field(tmp_path):
    _, projection, _ = runtime_fixture(tmp_path)
    projection.pop("members")
    assert_error("PROJECTION_MISSING_FIELD", load_runtime_projection, projection)


def test_rejects_unknown_projection_field(tmp_path):
    _, projection, _ = runtime_fixture(tmp_path)
    projection["extra"] = True
    assert_error("PROJECTION_UNKNOWN_FIELD", load_runtime_projection, projection)


@pytest.mark.parametrize("constant", ["NaN", "Infinity", "-Infinity"])
def test_rejects_nonfinite_json_numbers(constant):
    raw = (
        '{"schema_version":"ctr-runtime-projection-1","members":['
        '{"path":"x","size_bytes":'
        + constant
        + ',"sha256":"'
        + "0" * 64
        + '","mode":"0644","role":"runtime_resource"}]}'
    ).encode()
    assert_error("JSON_NONFINITE_NUMBER", load_runtime_projection, raw)


def test_rejects_empty_projection_members():
    projection = {"schema_version": PROJECTION_SCHEMA_VERSION, "members": []}
    assert_error("PROJECTION_EMPTY_MEMBERS", load_runtime_projection, projection)


def test_rejects_invalid_projection_digest(tmp_path):
    _, projection, _ = runtime_fixture(tmp_path)
    projection["members"][0]["sha256"] = "A" * 64
    assert_error("SHA256_FORMAT", load_runtime_projection, projection)


@pytest.mark.parametrize("size", [-1, 1.5, True])
def test_rejects_invalid_projection_size(tmp_path, size):
    _, projection, _ = runtime_fixture(tmp_path)
    projection["members"][0]["size_bytes"] = size
    assert_error("SIZE_INVALID", load_runtime_projection, projection)


@pytest.mark.parametrize("field", ["self_digest", "timestamp", "absolute_path", "host_path"])
def test_rejects_envelope_or_host_metadata(tmp_path, field):
    _, projection, _ = runtime_fixture(tmp_path)
    projection[field] = "/host/path" if "path" in field else "forbidden"
    assert_error("PROJECTION_FORBIDDEN_METADATA", load_runtime_projection, projection)


# Safe path and physical reconciliation rejection tests --------------------


@pytest.mark.parametrize(
    ("path", "code"),
    [
        ("/absolute/file", "PATH_ABSOLUTE"),
        ("a/../file", "PATH_TRAVERSAL"),
        ("a\\file", "PATH_BACKSLASH"),
        ("a//file", "PATH_NONCANONICAL_SEPARATOR"),
        ("a\x00file", "PATH_NUL"),
        ("C:/file", "PATH_DRIVE_LETTER"),
        ("https://host/file", "PATH_URI"),
        ("file:relative", "PATH_URI"),
    ],
)
def test_rejects_unsafe_projection_paths(tmp_path, path, code):
    _, projection, _ = runtime_fixture(tmp_path)
    projection["members"][0]["path"] = path
    projection["members"] = sorted(projection["members"], key=lambda item: item["path"])
    assert_error(code, load_runtime_projection, projection)


def test_rejects_unicode_normalization_collision(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    first = deepcopy(projection["members"][0])
    second = deepcopy(first)
    first["path"] = "config/caf\u00e9.yaml"
    second["path"] = "config/cafe\u0301.yaml"
    projection["members"] = [second, first]
    assert root.exists()
    assert_error("PATH_UNICODE_COLLISION", load_runtime_projection, projection)


def test_rejects_duplicate_projection_path(tmp_path):
    _, projection, _ = runtime_fixture(tmp_path)
    projection["members"].append(deepcopy(projection["members"][0]))
    projection["members"] = sorted(projection["members"], key=lambda item: item["path"])
    assert_error("PROJECTION_DUPLICATE_PATH", load_runtime_projection, projection)


def test_rejects_noncanonical_projection_member_order(tmp_path):
    _, projection, _ = runtime_fixture(tmp_path)
    projection["members"] = list(reversed(projection["members"]))
    assert_error("PROJECTION_MEMBER_ORDER", load_runtime_projection, projection)


def test_reconciliation_reports_missing_file(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    (root / projection["members"][0]["path"]).unlink()
    assert "PHYSICAL_MEMBER_MISSING" in issue_codes(reconcile_runtime_projection(projection, root))


def test_reconciliation_reports_extra_file(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    (root / "extra.txt").write_text("extra", encoding="utf-8")
    assert "PHYSICAL_UNDECLARED_FILE" in issue_codes(reconcile_runtime_projection(projection, root))


def test_reconciliation_rejects_directory_in_place_of_member(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    member_path = root / projection["members"][0]["path"]
    member_path.unlink()
    member_path.mkdir()
    assert "PHYSICAL_MEMBER_NOT_REGULAR" in issue_codes(reconcile_runtime_projection(projection, root))


def test_reconciliation_rejects_member_symlink(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    member_path = root / projection["members"][0]["path"]
    target = root / "symlink-target"
    target.write_bytes(member_path.read_bytes())
    member_path.unlink()
    member_path.symlink_to(target)
    assert "PHYSICAL_SYMLINK" in issue_codes(reconcile_runtime_projection(projection, root))


def test_reconciliation_rejects_hardlink_alias(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    first = root / projection["members"][0]["path"]
    alias = root / "config/alias.yaml"
    os.link(first, alias)
    physical_paths = [item["path"] for item in projection["members"]]
    projection = projection_dict(root, tuple(sorted((*physical_paths, "config/alias.yaml"))))
    assert "PHYSICAL_HARDLINK_ALIAS" in issue_codes(reconcile_runtime_projection(projection, root))


def test_reconciliation_reports_size_mismatch(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    projection["members"][0]["size_bytes"] += 1
    assert "PHYSICAL_SIZE_MISMATCH" in issue_codes(reconcile_runtime_projection(projection, root))


def test_reconciliation_reports_digest_mismatch(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    projection["members"][0]["sha256"] = "0" * 64
    assert "PHYSICAL_DIGEST_MISMATCH" in issue_codes(reconcile_runtime_projection(projection, root))


def test_reconciliation_reports_mode_mismatch(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    projection["members"][0]["mode"] = "0755"
    assert "PHYSICAL_MODE_MISMATCH" in issue_codes(reconcile_runtime_projection(projection, root))


def test_plan_binding_rejects_runtime_root_escape(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / "escaped.py").write_text("pass\n", encoding="utf-8")
    (root / "escape").symlink_to(outside, target_is_directory=True)
    escaped = projection_dict(root)
    data = (outside / "escaped.py").read_bytes()
    escaped["members"].append(
        {
            "path": "escape/escaped.py",
            "size_bytes": len(data),
            "sha256": sha256(data).hexdigest(),
            "mode": "0644",
            "role": "python_module",
        }
    )
    escaped["members"] = sorted(escaped["members"], key=lambda item: item["path"])
    escaped_identity = runtime_projection_identity(escaped)
    plan = valid_plan(escaped_identity)
    plan["prospective_argv"][1] = "escape/escaped.py"
    plan["argv_bindings"][0]["member_path"] = "escape/escaped.py"
    plan["argv_classifications"][1]["value"] = "escape/escaped.py"
    plan["argv_classifications"][1]["member_path"] = "escape/escaped.py"
    assert identity != escaped_identity
    assert "RUNTIME_ROOT_ESCAPE" in issue_codes(reconcile_runtime_projection(escaped, root))
    assert_error("PLAN_BINDING_ROOT_ESCAPE", validate_runtime_plan, plan, escaped, root)


# Dependency graph rejection tests ----------------------------------------


def graph_result(projection, *, entrypoints=None, nodes=None, edges=None, external=None, required=None):
    return validate_runtime_dependency_closure(
        projection,
        entrypoints=entrypoints or ["launch/simulation.launch.py"],
        project_nodes=nodes or ["launch/simulation.launch.py", "pkg/main.py", "config/robot.yaml"],
        dependencies=edges
        if edges is not None
        else [
            RuntimeDependency("launch/simulation.launch.py", "pkg/main.py"),
            RuntimeDependency("launch/simulation.launch.py", "config/robot.yaml"),
        ],
        declared_external_dependencies=external or [],
        required_members=required,
    )


def test_dependency_rejects_missing_entrypoint(tmp_path):
    _, projection, _ = runtime_fixture(tmp_path)
    result = graph_result(projection, entrypoints=["missing.py"])
    assert "DEPENDENCY_ENTRYPOINT_MISSING" in issue_codes(result)


def test_dependency_rejects_edge_to_undeclared_member(tmp_path):
    _, projection, _ = runtime_fixture(tmp_path)
    edges = [RuntimeDependency("launch/simulation.launch.py", "pkg/missing.py")]
    assert "DEPENDENCY_EDGE_UNDECLARED_MEMBER" in issue_codes(graph_result(projection, edges=edges))


def test_dependency_rejects_unresolved_project_dependency(tmp_path):
    _, projection, _ = runtime_fixture(tmp_path)
    edges = [RuntimeDependency("launch/simulation.launch.py", "pkg/main.py", resolved=False)]
    assert "DEPENDENCY_UNRESOLVED_PROJECT" in issue_codes(graph_result(projection, edges=edges))


def test_dependency_rejects_undeclared_external_dependency(tmp_path):
    _, projection, _ = runtime_fixture(tmp_path)
    edges = [
        RuntimeDependency("launch/simulation.launch.py", "pkg/main.py"),
        RuntimeDependency("launch/simulation.launch.py", "config/robot.yaml"),
        RuntimeDependency("launch/simulation.launch.py", "launch_ros", "external"),
    ]
    assert "DEPENDENCY_UNDECLARED_EXTERNAL" in issue_codes(graph_result(projection, edges=edges))


def test_dependency_rejects_duplicate_edge(tmp_path):
    _, projection, _ = runtime_fixture(tmp_path)
    edge = RuntimeDependency("launch/simulation.launch.py", "pkg/main.py")
    assert "DEPENDENCY_DUPLICATE_EDGE" in issue_codes(graph_result(projection, edges=[edge, edge]))


def test_dependency_rejects_evidence_tooling_member(tmp_path):
    root, _, _ = runtime_fixture(tmp_path)
    path = root / "tooling/helper.py"
    path.parent.mkdir()
    path.write_text("pass\n", encoding="utf-8")
    projection = projection_dict(root, ("tooling/helper.py",))
    result = graph_result(
        projection,
        entrypoints=["tooling/helper.py"],
        nodes=["tooling/helper.py"],
        edges=[],
    )
    assert "DEPENDENCY_EVIDENCE_TOOLING_MEMBER" in issue_codes(result)


def test_dependency_reports_unreachable_required_member(tmp_path):
    _, projection, _ = runtime_fixture(tmp_path)
    edges = [RuntimeDependency("launch/simulation.launch.py", "pkg/main.py")]
    assert "DEPENDENCY_UNREACHABLE_REQUIRED_MEMBER" in issue_codes(graph_result(projection, edges=edges))


def test_dependency_rejects_duplicate_node(tmp_path):
    _, projection, _ = runtime_fixture(tmp_path)
    nodes = ["launch/simulation.launch.py", "pkg/main.py", "config/robot.yaml", "pkg/main.py"]
    assert "DEPENDENCY_DUPLICATE_NODE" in issue_codes(graph_result(projection, nodes=nodes))


def test_dependency_rejects_empty_entrypoint_inventory(tmp_path):
    _, projection, _ = runtime_fixture(tmp_path)
    result = validate_runtime_dependency_closure(
        projection,
        entrypoints=[],
        project_nodes=["launch/simulation.launch.py", "pkg/main.py", "config/robot.yaml"],
        dependencies=[],
        declared_external_dependencies=[],
    )
    assert "DEPENDENCY_EMPTY_ENTRYPOINTS" in issue_codes(result)


# Runtime plan rejection tests ---------------------------------------------


def test_plan_rejects_wrong_runtime_identity(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plan = valid_plan("0" * 64)
    assert identity != "0" * 64
    assert_error("PLAN_RUNTIME_IDENTITY_MISMATCH", validate_runtime_plan, plan, projection, root)


def test_plan_rejects_invalid_mode(tmp_path):
    _, _, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["mode"] = "invalid"
    assert_error("PLAN_MODE", load_runtime_plan, plan)


def test_plan_rejects_unsupported_schema_version(tmp_path):
    _, _, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["schema_version"] = "future"
    assert_error("PLAN_UNSUPPORTED_VERSION", load_runtime_plan, plan)


def test_plan_rejects_missing_required_field(tmp_path):
    _, _, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan.pop("policy")
    assert_error("PLAN_MISSING_FIELD", load_runtime_plan, plan)


def test_plan_rejects_unknown_field(tmp_path):
    _, _, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["unexpected"] = True
    assert_error("PLAN_UNKNOWN_FIELD", load_runtime_plan, plan)


def test_plan_rejects_missing_argv_binding(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["argv_bindings"] = []
    assert_error("PLAN_MISSING_ARGV_BINDING", validate_runtime_plan, plan, projection, root)


def test_plan_rejects_production_without_project_binding(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["project_owned_argv_indices"] = []
    plan["argv_bindings"] = []
    plan["prospective_argv"][1] = "launch"
    plan["argv_classifications"][1] = {
        "argv_index": 1,
        "kind": "identifier",
        "value": "launch",
    }
    assert_error("PLAN_PRODUCTION_BINDING_REQUIRED", validate_runtime_plan, plan, projection, root)


def test_plan_rejects_duplicate_argv_binding(tmp_path):
    _, _, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["argv_bindings"].append(deepcopy(plan["argv_bindings"][0]))
    assert_error("PLAN_DUPLICATE_ARGV_BINDING", load_runtime_plan, plan)


def test_plan_rejects_binding_to_absent_member(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["prospective_argv"][1] = "pkg/absent.py"
    plan["argv_bindings"][0]["member_path"] = "pkg/absent.py"
    plan["argv_classifications"][1]["value"] = "pkg/absent.py"
    plan["argv_classifications"][1]["member_path"] = "pkg/absent.py"
    assert_error("PLAN_BINDING_MEMBER_ABSENT", validate_runtime_plan, plan, projection, root)


def test_plan_rejects_physically_missing_bound_member(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    (root / "launch/simulation.launch.py").unlink()
    assert_error("PLAN_BINDING_PHYSICAL_MISSING", validate_runtime_plan, valid_plan(identity), projection, root)


@pytest.mark.parametrize(
    "role",
    [
        "AUTHENTICATED_TOOLING_ROOT",
        "EVIDENCE_TOOLING_ROOT",
        "CORRECTION_TOOLING_ROOT",
        "LIVE_REPOSITORY_ROOT",
        "TEMPORARY_ROOT",
    ],
)
def test_plan_rejects_non_runtime_path_base(tmp_path, role):
    root, projection, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["runtime_root_role"] = role
    assert_error("PLAN_RUNTIME_ROOT_ROLE", validate_runtime_plan, plan, projection, root)


def test_plan_rejects_stale_operative_identity(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    assert_error(
        "PLAN_STALE_OPERATIVE_IDENTITY",
        validate_runtime_plan,
        valid_plan(identity),
        projection,
        root,
        forbidden_identities=[identity],
    )


def test_plan_rejects_undeclared_external_command(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["external_dependencies"] = ["launch_ros"]
    assert_error("PLAN_UNDECLARED_EXTERNAL_COMMAND", validate_runtime_plan, plan, projection, root)


def test_plan_rejects_external_command_argv_mismatch(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["external_commands"][0]["command"] = "python3"
    assert_error("PLAN_EXTERNAL_COMMAND_ARGV_MISMATCH", validate_runtime_plan, plan, projection, root)


def test_plan_rejects_dependency_omitted_from_validated_closure(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["external_dependencies"] = ["ros2"]
    assert_error(
        "PLAN_EXTERNAL_DEPENDENCY_MISSING",
        validate_runtime_plan,
        plan,
        projection,
        root,
        allowed_external_dependencies=["ros2", "launch_ros"],
    )


@pytest.mark.parametrize(
    "policy_update",
    [
        {"launchable": True},
        {"allow_full_launch": True},
        {"execution_authorized": True},
    ],
)
def test_plan_rejects_inconsistent_validate_only_policy(tmp_path, policy_update):
    root, projection, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["policy"].update(policy_update)
    assert_error("PLAN_POLICY_INCONSISTENT", validate_runtime_plan, plan, projection, root)


@pytest.mark.parametrize(
    ("path", "code"),
    [("/repo/live.py", "PLAN_ABSOLUTE_ARGV"), ("../escape.py", "PATH_TRAVERSAL")],
)
def test_plan_rejects_absolute_or_escaping_runtime_path(tmp_path, path, code):
    root, projection, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["prospective_argv"][1] = path
    plan["argv_bindings"][0]["member_path"] = path
    plan["argv_classifications"][1]["value"] = path
    plan["argv_classifications"][1]["member_path"] = path
    assert_error(code, validate_runtime_plan, plan, projection, root)


def test_plan_rejects_unclassified_command(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["external_commands"] = []
    assert_error("PLAN_COMMAND_UNCLASSIFIED", validate_runtime_plan, plan, projection, root)


# Six-plan reconciliation rejection tests ----------------------------------


def test_six_plan_rejects_missing_plan(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plans = six_plans(identity)
    plans.pop("offline_duplicate")
    assert_error("SIX_PLAN_MISSING_ROLE", validate_six_plan_set, plans, projection, root)


def test_six_plan_rejects_extra_plan(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plans = six_plans(identity)
    plans["extra"] = plans["production_root"]
    assert_error("SIX_PLAN_EXTRA_ROLE", validate_six_plan_set, plans, projection, root)


def test_six_plan_rejects_wrong_assigned_mode(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plans = six_plans(identity)
    wrong = json_bytes(valid_plan(identity, "offline"))
    plans["production_root"] = wrong
    plans["production_duplicate"] = wrong
    assert_error("SIX_PLAN_ASSIGNED_MODE_MISMATCH", validate_six_plan_set, plans, projection, root)


def test_six_plan_rejects_root_duplicate_byte_mismatch(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plans = six_plans(identity)
    plans["production_duplicate"] = plans["production_root"] + b"\n"
    assert_error("SIX_PLAN_BYTE_MISMATCH", validate_six_plan_set, plans, projection, root)


def test_six_plan_rejects_one_different_runtime_identity(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plans = six_plans(identity)
    wrong = json_bytes(valid_plan("0" * 64, "offline"))
    plans["offline_root"] = wrong
    plans["offline_duplicate"] = wrong
    assert_error("PLAN_RUNTIME_IDENTITY_MISMATCH", validate_six_plan_set, plans, projection, root)


def test_six_plan_rejects_one_invalid_path_base(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plans = six_plans(identity)
    plan = valid_plan(identity, "test_only")
    plan["runtime_root_role"] = "EVIDENCE_TOOLING_ROOT"
    raw = json_bytes(plan)
    plans["test_only_root"] = raw
    plans["test_only_duplicate"] = raw
    assert_error("PLAN_RUNTIME_ROOT_ROLE", validate_six_plan_set, plans, projection, root)


def test_six_plan_rejects_one_stale_identity(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plans = six_plans(identity)
    assert_error(
        "PLAN_STALE_OPERATIVE_IDENTITY",
        validate_six_plan_set,
        plans,
        projection,
        root,
        forbidden_identities=[identity],
    )


# Security and contract revision regressions --------------------------------


def test_rejects_unclassified_relative_argv_token(tmp_path):
    _, _, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["prospective_argv"].append("../../live-repository/config.yaml")
    assert_error("PLAN_ARGV_CLASSIFICATION_COVERAGE", load_runtime_plan, plan)


@pytest.mark.parametrize(
    ("kind", "code"),
    [
        ("flag", "PLAN_ARGV_LITERAL_PATH"),
        ("identifier", "PLAN_ARGV_LITERAL_PATH"),
        ("integer", "PLAN_ARGV_LITERAL_PATH"),
        ("numeric", "PLAN_ARGV_LITERAL_PATH"),
        ("ros_name", "PLAN_ARGV_ROS_NAME"),
        ("assignment", "PLAN_ARGV_ASSIGNMENT_PATH"),
    ],
)
def test_rejects_traversal_falsely_classified_as_literal(tmp_path, kind, code):
    _, _, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    index = len(plan["prospective_argv"])
    token = "../../live-repository/config.yaml"
    plan["prospective_argv"].append(token)
    plan["argv_classifications"].append({"argv_index": index, "kind": kind, "value": token})
    assert_error(code, load_runtime_plan, plan)


def test_rejects_missing_argv_classification_index(tmp_path):
    _, _, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["argv_classifications"].pop(2)
    assert_error("PLAN_ARGV_CLASSIFICATION_COVERAGE", load_runtime_plan, plan)


def test_rejects_duplicate_argv_classification_index(tmp_path):
    _, _, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["argv_classifications"].append(deepcopy(plan["argv_classifications"][2]))
    assert_error("PLAN_DUPLICATE_ARGV_CLASSIFICATION", load_runtime_plan, plan)


def test_rejects_unknown_argv_classification_kind(tmp_path):
    _, _, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["argv_classifications"][2]["kind"] = "arbitrary_literal"
    assert_error("PLAN_ARGV_CLASSIFICATION_KIND", load_runtime_plan, plan)


def test_rejects_classified_token_value_mismatch(tmp_path):
    _, _, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["argv_classifications"][2]["value"] = "--different"
    assert_error("PLAN_ARGV_CLASSIFICATION_VALUE_MISMATCH", load_runtime_plan, plan)


def test_rejects_path_bearing_assignment(tmp_path):
    _, _, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    index = len(plan["prospective_argv"])
    token = "config:=../../live/config.yaml"
    plan["prospective_argv"].append(token)
    plan["argv_classifications"].append(
        {"argv_index": index, "kind": "assignment", "value": token}
    )
    assert_error("PLAN_ARGV_ASSIGNMENT_PATH", load_runtime_plan, plan)


def test_accepts_explicit_valid_ros_name_classification(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    index = len(plan["prospective_argv"])
    plan["prospective_argv"].append("/ctr/state")
    plan["argv_classifications"].append(
        {"argv_index": index, "kind": "ros_name", "value": "/ctr/state"}
    )
    parsed = validate_runtime_plan(plan, projection, root, ros_name_authority={index: "/ctr/state"})
    assert parsed.argv_classifications[-1].kind == "ros_name"


def test_descriptor_authentication_detects_pathname_replacement(tmp_path, monkeypatch):
    root, projection, _ = runtime_fixture(tmp_path)
    target = root / "config/robot.yaml"
    parent = target.parent
    original_read = runtime_validation.os.read
    replaced = False

    def replacing_read(descriptor, count):
        nonlocal replaced
        block = original_read(descriptor, count)
        if block and not replaced:
            replaced = True
            parent.chmod(0o755)
            target.rename(parent / "robot.original")
            target.write_bytes(b"x" * len(block))
            target.chmod(0o444)
            parent.chmod(0o555)
        return block

    with immutable_runtime(root):
        monkeypatch.setattr(runtime_validation.os, "read", replacing_read)
        result = _raw_reconcile_runtime_projection(projection, root, complete_inventory=False)
    assert replaced
    assert "PHYSICAL_INODE_CHANGED" in issue_codes(result)


def test_descriptor_authentication_detects_concurrent_content_change(tmp_path, monkeypatch):
    root, projection, _ = runtime_fixture(tmp_path)
    target = root / "config/robot.yaml"
    original_read = runtime_validation.os.read
    changed = False

    def mutating_read(descriptor, count):
        nonlocal changed
        block = original_read(descriptor, count)
        if block and not changed:
            changed = True
            target.chmod(0o644)
            target.write_bytes(b"y" * len(block))
            target.chmod(0o444)
        return block

    with immutable_runtime(root):
        monkeypatch.setattr(runtime_validation.os, "read", mutating_read)
        result = _raw_reconcile_runtime_projection(projection, root, complete_inventory=False)
    assert changed
    assert "PHYSICAL_CHANGED_DURING_READ" in issue_codes(result)


def test_rejects_writable_runtime_root(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    result = _raw_reconcile_runtime_projection(projection, root, complete_inventory=False)
    assert "RUNTIME_ROOT_WRITABLE" in issue_codes(result)


def test_fails_closed_when_nofollow_primitives_are_unavailable(tmp_path, monkeypatch):
    root, projection, _ = runtime_fixture(tmp_path)
    monkeypatch.setattr(runtime_validation, "_descriptor_primitives_available", lambda: False)
    with immutable_runtime(root):
        result = _raw_reconcile_runtime_projection(projection, root, complete_inventory=False)
    assert issue_codes(result) == {"PHYSICAL_NOFOLLOW_UNAVAILABLE"}


def test_rejects_writable_intermediate_directory(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    with immutable_runtime(root):
        (root / "config").chmod(0o755)
        result = _raw_reconcile_runtime_projection(projection, root, complete_inventory=False)
    assert "PHYSICAL_DIRECTORY_WRITABLE" in issue_codes(result)


def test_rejects_writable_runtime_member(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    with immutable_runtime(root):
        (root / "config/robot.yaml").chmod(0o644)
        result = _raw_reconcile_runtime_projection(projection, root, complete_inventory=False)
    assert "PHYSICAL_FILE_WRITABLE" in issue_codes(result)


def test_rejects_final_symlink_substitution(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    target = root / "config/robot.yaml"
    outside = tmp_path / "outside.yaml"
    outside.write_text("outside: true\n", encoding="utf-8")
    target.unlink()
    target.symlink_to(outside)
    with immutable_runtime(root):
        result = _raw_reconcile_runtime_projection(projection, root, complete_inventory=False)
    assert "PHYSICAL_SYMLINK" in issue_codes(result)


def test_descriptor_closure_on_success_and_failure(tmp_path, monkeypatch):
    root, projection, _ = runtime_fixture(tmp_path)
    original_open = runtime_validation.os.open
    original_dup = runtime_validation.os.dup
    original_close = runtime_validation.os.close
    opened = set()
    closed = set()

    def tracking_open(*args, **kwargs):
        descriptor = original_open(*args, **kwargs)
        opened.add(descriptor)
        return descriptor

    def tracking_dup(descriptor):
        duplicate = original_dup(descriptor)
        opened.add(duplicate)
        return duplicate

    def tracking_close(descriptor):
        closed.add(descriptor)
        return original_close(descriptor)

    monkeypatch.setattr(runtime_validation.os, "open", tracking_open)
    monkeypatch.setattr(runtime_validation.os, "dup", tracking_dup)
    monkeypatch.setattr(runtime_validation.os, "close", tracking_close)
    monkeypatch.setattr(runtime_validation, "_descriptor_primitives_available", lambda: True)
    with immutable_runtime(root):
        success = _raw_reconcile_runtime_projection(projection, root, complete_inventory=True)
    assert success.issues == ()
    assert opened <= closed

    opened.clear()
    closed.clear()
    (root / "config/robot.yaml").unlink()
    with immutable_runtime(root):
        failure = _raw_reconcile_runtime_projection(projection, root, complete_inventory=False)
    assert "PHYSICAL_MEMBER_MISSING" in issue_codes(failure)
    assert opened <= closed


def test_list_valued_mode_has_stable_error(tmp_path):
    _, _, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["mode"] = []
    assert_error("PLAN_MODE_TYPE", load_runtime_plan, plan)


def test_list_valued_member_role_has_stable_error(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    assert root.is_dir()
    projection["members"][0]["role"] = []
    assert_error("MEMBER_ROLE_TYPE", load_runtime_projection, projection)


def test_mixed_environment_key_has_stable_error(tmp_path):
    _, _, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["prospective_environment"][1] = "bad"
    assert_error("PLAN_ENVIRONMENT_KEY_TYPE", load_runtime_plan, plan)


def test_lone_surrogate_text_has_stable_error():
    assert_error("JSON_UNICODE_ENCODING", load_runtime_projection, "{\"bad\":\"\ud800\"}")


def test_direct_runtime_projection_detaches_mutable_members(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    member = load_runtime_projection(projection).members[0]
    mutable_members = [member]
    direct = RuntimeProjection(PROJECTION_SCHEMA_VERSION, mutable_members)
    mutable_members.clear()
    assert direct.members == (member,)
    assert (root / member.path).is_file()


def test_direct_runtime_plan_recursively_detaches_nested_mappings(tmp_path):
    _, _, identity = runtime_fixture(tmp_path)
    source = valid_plan(identity)
    direct = RuntimePlan(
        schema_version=source["schema_version"],
        mode=source["mode"],
        production_runtime_identity=source["production_runtime_identity"],
        runtime_root_role=source["runtime_root_role"],
        prospective_argv=source["prospective_argv"],
        project_owned_argv_indices=source["project_owned_argv_indices"],
        argv_bindings=source["argv_bindings"],
        external_commands=source["external_commands"],
        argv_classifications=source["argv_classifications"],
        prospective_environment=source["prospective_environment"],
        external_dependencies=source["external_dependencies"],
        policy=source["policy"],
    )
    source["prospective_environment"]["ROS_DOMAIN_ID"] = "999"
    source["argv_classifications"][2]["value"] = "--changed"
    assert direct.environment["ROS_DOMAIN_ID"] == "232"
    assert direct.argv_classifications[2].value == "--domain"


def test_direct_result_records_detach_mutable_sequences():
    issue_list = [RuntimeIssue("EXAMPLE")]
    physical = RuntimeProjectionReconciliation(1, 1, 1, issue_list)
    closure = RuntimeDependencyClosure(["entry.py"], ["entry.py"], ["entry.py"], [], issue_list)
    roles = ["production_root", "production_duplicate", "offline_root", "offline_duplicate", "test_only_root", "test_only_duplicate"]
    plan_hashes = [(role, "0" * 64) for role in roles]
    modes = [(role, role.rsplit("_", 1)[0]) for role in roles]
    reconciled = RuntimeReconciliation("0" * 64, roles, plan_hashes, modes)
    issue_list.clear()
    plan_hashes.clear()
    modes.clear()
    assert len(physical.issues) == 1
    assert len(closure.issues) == 1
    assert len(reconciled.plan_sha256) == 6


@pytest.mark.parametrize("character", ["\n", "\r", "\t", "\x1b", "\x7f"])
def test_rejects_control_character_member_paths(tmp_path, character):
    root, projection, _ = runtime_fixture(tmp_path)
    assert root.is_dir()
    projection["members"][0]["path"] = f"bad{character}name"
    assert_error("PATH_CONTROL_CHARACTER", load_runtime_projection, projection)


def test_rejects_unused_external_dependency_declaration(tmp_path):
    _, projection, _ = runtime_fixture(tmp_path)
    result = validate_runtime_dependency_closure(
        projection,
        entrypoints=["launch/simulation.launch.py"],
        project_nodes=["launch/simulation.launch.py", "pkg/main.py", "config/robot.yaml"],
        dependencies=[
            RuntimeDependency("launch/simulation.launch.py", "pkg/main.py"),
            RuntimeDependency("launch/simulation.launch.py", "config/robot.yaml"),
            RuntimeDependency("launch/simulation.launch.py", "launch_ros", "external"),
        ],
        declared_external_dependencies=["launch_ros", "unused_external"],
    )
    assert "DEPENDENCY_UNUSED_EXTERNAL_DECLARATION" in issue_codes(result)
    assert result.external_dependencies == ("launch_ros",)


def test_explicit_empty_expected_identity_is_rejected(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    assert_error(
        "SHA256_FORMAT",
        validate_runtime_plan,
        valid_plan(identity),
        projection,
        root,
        expected_runtime_identity="",
    )


@pytest.mark.parametrize(
    ("target", "value", "code"),
    [
        ("plan_version", True, "PLAN_SCHEMA_VERSION_TYPE"),
        ("projection_version", True, "PROJECTION_SCHEMA_VERSION_TYPE"),
        ("classification_index", True, "PLAN_ARGV_CLASSIFICATION_INDEX"),
        ("classification_container", {}, "ARGV_CLASSIFICATION_TYPE"),
    ],
)
def test_malformed_version_index_and_container_have_stable_errors(tmp_path, target, value, code):
    _, projection, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    if target == "plan_version":
        plan["schema_version"] = value
        function, subject = load_runtime_plan, plan
    elif target == "projection_version":
        projection["schema_version"] = value
        function, subject = load_runtime_projection, projection
    elif target == "classification_index":
        plan["argv_classifications"][0]["argv_index"] = value
        function, subject = load_runtime_plan, plan
    else:
        plan["argv_classifications"] = value
        function, subject = load_runtime_plan, plan
    assert_error(code, function, subject)


def test_all_six_plans_have_safe_complete_argv_classifications(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plans = six_plans(identity)
    result = validate_six_plan_set(plans, projection, root)
    assert len(result.roles) == 6
    for raw in plans.values():
        parsed = load_runtime_plan(raw)
        assert tuple(item.argv_index for item in parsed.argv_classifications) == tuple(
            range(len(parsed.prospective_argv))
        )


def test_authenticated_snapshot_lifecycle_and_descriptor_bytes(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    with immutable_runtime(root):
        snapshot = open_authenticated_runtime_snapshot(projection, root, complete_inventory=False)
        assert snapshot.authoritative
        original = (root / "launch/simulation.launch.py").read_bytes()
        assert snapshot.read_member_bytes("launch/simulation.launch.py") == original
        snapshot.close(); snapshot.close()
        with pytest.raises(RuntimeValidationError) as exc:
            snapshot.read_member_bytes("launch/simulation.launch.py")
        assert exc.value.code == "SNAPSHOT_CLOSED"


def test_snapshot_path_divergence_keeps_original_descriptor(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    with immutable_runtime(root):
        snapshot = open_authenticated_runtime_snapshot(projection, root, complete_inventory=False)
        target = root / "launch/simulation.launch.py"
        original = target.read_bytes()
        replacement = tmp_path / "replacement.py"; replacement.write_bytes(b"replacement\n")
        target.parent.chmod(0o755)
        target.unlink(); target.symlink_to(replacement)
        assert snapshot.read_member_bytes("launch/simulation.launch.py") == original
        assert snapshot.verify_current_paths()
        snapshot.close()


@pytest.mark.parametrize("value", [[], {}, 0, 1, ""])
def test_snapshot_security_switches_require_exact_booleans(tmp_path, value):
    root, projection, _ = runtime_fixture(tmp_path)
    with pytest.raises(RuntimeValidationError) as exc:
        open_authenticated_runtime_snapshot(projection, root, complete_inventory=value)
    assert exc.value.code == "RECONCILE_COMPLETE_INVENTORY_TYPE"


def test_ros_name_requires_external_authority(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    plan = valid_plan(identity)
    plan["prospective_argv"].append("/tmp/x")
    plan["argv_classifications"].append({"argv_index": 4, "kind": "ros_name", "value": "/tmp/x"})
    assert_error("ROS_NAME_AUTHORITY_REQUIRED", validate_runtime_plan, plan, projection, root)


def test_impossible_projection_reconciliation_is_rejected():
    with pytest.raises(RuntimeValidationError) as exc:
        RuntimeProjectionReconciliation(1, 0, 2, ())
    assert exc.value.code == "RECONCILIATION_COUNT_INCONSISTENT"


def test_impossible_dependency_closure_is_rejected():
    with pytest.raises(RuntimeValidationError) as exc:
        RuntimeDependencyClosure(("a.py",), ("a.py", "b.py"), ("a.py",), (), ())
    assert exc.value.code == "DEPENDENCY_CLEAN_INCONSISTENT"


def test_invalid_six_plan_result_is_rejected():
    with pytest.raises(RuntimeValidationError) as exc:
        RuntimeReconciliation("0" * 64, ("production_root",), (), ())
    assert exc.value.code == "SIX_PLAN_ROLES_INCONSISTENT"


def test_fake_snapshot_construction_is_rejected():
    with pytest.raises(RuntimeValidationError) as exc:
        runtime_validation.AuthenticatedRuntimeSnapshot("/tmp", None, None, {}, (), _token=None)
    assert exc.value.code == "SNAPSHOT_CONSTRUCTION_FORBIDDEN"


def test_snapshot_metadata_cannot_be_replaced(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    with immutable_runtime(root):
        snapshot = open_authenticated_runtime_snapshot(projection, root, complete_inventory=False)
        with pytest.raises(AttributeError):
            snapshot._projection = None
        snapshot.close()


def test_closed_snapshot_rejected_by_production_validation(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    with immutable_runtime(root):
        snapshot = open_authenticated_runtime_snapshot(projection, root, complete_inventory=False)
        snapshot.close()
        with pytest.raises(RuntimeValidationError) as exc:
            _raw_validate_runtime_plan(valid_plan(identity), projection, snapshot)
        assert exc.value.code == "SNAPSHOT_CLOSED"


def test_string_external_dependency_collection_rejected(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    assert_error("EXTERNAL_DEPENDENCIES_CONTAINER_TYPE", validate_runtime_plan, valid_plan(identity), projection, root, allowed_external_dependencies="ros2")


def test_raw_object_new_snapshot_is_rejected_by_authority_operations(tmp_path):
    fake = object.__new__(runtime_validation.AuthenticatedRuntimeSnapshot)
    for operation in (lambda: fake.close(), lambda: fake._ensure_open(), lambda: fake.verify_current_paths()):
        with pytest.raises(RuntimeValidationError) as exc:
            operation()
        assert exc.value.code == "SNAPSHOT_PROVENANCE_INVALID"


def test_attribute_transplant_and_fake_plan_authority_rejected(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    with immutable_runtime(root):
        genuine = open_authenticated_runtime_snapshot(projection, root, complete_inventory=False)
        fake = object.__new__(runtime_validation.AuthenticatedRuntimeSnapshot)
        for name in ("_projection", "_root_fd", "_member_fds", "_member_parents", "_member_stats", "_directory_stats", "_issues", "_closed", "_authoritative"):
            try: setattr(fake, name, getattr(genuine, name))
            except Exception: pass
        with pytest.raises(RuntimeValidationError) as exc:
            runtime_validation.validate_runtime_plan(valid_plan(identity), projection, fake)
        assert exc.value.code == "SNAPSHOT_PROVENANCE_INVALID"
        genuine.close()


def test_snapshot_copy_pickle_and_subclass_rejected(tmp_path):
    import copy, pickle
    root, projection, _ = runtime_fixture(tmp_path)
    with immutable_runtime(root):
        snap = open_authenticated_runtime_snapshot(projection, root, complete_inventory=False)
        with pytest.raises(RuntimeValidationError) as exc:
            copy.copy(snap)
        assert exc.value.code == "SNAPSHOT_COPY_FORBIDDEN"
        with pytest.raises(RuntimeValidationError):
            copy.deepcopy(snap)
        with pytest.raises(RuntimeValidationError):
            pickle.dumps(snap)
        snap.close()
    with pytest.raises(TypeError):
        class Derived(runtime_validation.AuthenticatedRuntimeSnapshot):
            pass


def test_genuine_close_registry_invalidation_and_idempotence(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    with immutable_runtime(root):
        snap = open_authenticated_runtime_snapshot(projection, root, complete_inventory=False)
        snap.close(); snap.close()
        with pytest.raises(RuntimeValidationError) as exc:
            snap.read_member_bytes("launch/simulation.launch.py")
        assert exc.value.code == "SNAPSHOT_CLOSED"


def test_phase_a_authority_state_owns_descriptors(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    with immutable_runtime(root):
        snap = open_authenticated_runtime_snapshot(projection, root, complete_inventory=False)
        state = snap._provenance()
        assert state.member_fds
        snap.close()
        assert state.closed and state.cleaned and not state.member_fds


def test_phase_a_registry_entry_uses_exact_weakref(tmp_path):
    root, projection, _ = runtime_fixture(tmp_path)
    with immutable_runtime(root):
        snap = open_authenticated_runtime_snapshot(projection, root, complete_inventory=False)
        entry = runtime_validation.AuthenticatedRuntimeSnapshot._REGISTRY[id(snap)]
        assert entry[0]() is snap
        snap.close()


def _locked_snapshot(tmp_path):
    root, projection, identity = runtime_fixture(tmp_path)
    guard = immutable_runtime(root); guard.__enter__()
    snap = open_authenticated_runtime_snapshot(projection, root, complete_inventory=False)
    return guard, snap, projection, identity


def test_phase_b2a_read_close_ordered(tmp_path):
    guard, snap, _, _ = _locked_snapshot(tmp_path)
    state = snap._provenance(); state.lock.acquire(); done = threading.Event()
    t = threading.Thread(target=lambda: (snap.close(), done.set())); t.start()
    assert not done.wait(0.05); state.lock.release(); t.join(1); assert not t.is_alive(); assert snap.closed
    guard.__exit__(None, None, None)


def test_phase_b2a_duplicate_close_ordered(tmp_path):
    guard, snap, _, _ = _locked_snapshot(tmp_path); state = snap._provenance(); state.lock.acquire(); done = threading.Event()
    t = threading.Thread(target=lambda: (snap.duplicate_member_fd("launch/simulation.launch.py"), done.set())); t.start()
    state.lock.release(); t.join(1); assert not t.is_alive(); snap.close(); guard.__exit__(None, None, None)


def test_phase_b2a_verify_close_ordered(tmp_path):
    guard, snap, _, _ = _locked_snapshot(tmp_path); state = snap._provenance(); state.lock.acquire(); done = threading.Event()
    t = threading.Thread(target=lambda: (snap.verify_current_paths(), done.set())); t.start()
    assert not done.wait(0.05); state.lock.release(); t.join(1); assert not t.is_alive(); snap.close(); guard.__exit__(None, None, None)


def test_phase_b2a_plan_close_ordered(tmp_path):
    guard, snap, projection, identity = _locked_snapshot(tmp_path); state = snap._provenance(); state.lock.acquire(); done = threading.Event()
    t = threading.Thread(target=lambda: (runtime_validation.validate_runtime_plan(valid_plan(identity), projection, snap, allowed_external_dependencies={"launch_ros", "ros2"}), done.set())); t.start()
    assert not done.wait(0.05); state.lock.release(); t.join(1); assert not t.is_alive(); snap.close(); guard.__exit__(None, None, None)


def test_phase_b2a_multiple_close_callers(tmp_path):
    guard, snap, _, _ = _locked_snapshot(tmp_path); barrier = threading.Barrier(4); errors=[]
    def close():
        try: barrier.wait(); snap.close()
        except Exception as exc: errors.append(exc)
    ts=[threading.Thread(target=close) for _ in range(3)]
    [t.start() for t in ts]; barrier.wait(); [t.join(1) for t in ts]
    assert all(not t.is_alive() for t in ts) and not errors and snap.closed; guard.__exit__(None,None,None)


def test_phase_b2a_gc_without_close(tmp_path):
    guard, snap, _, _ = _locked_snapshot(tmp_path); ref=weakref.ref(snap); snap=None; gc.collect(); assert ref() is None; guard.__exit__(None,None,None)


def test_phase_b2a_explicit_close_then_gc(tmp_path):
    guard, snap, _, _ = _locked_snapshot(tmp_path); snap.close(); ref=weakref.ref(snap); snap=None; gc.collect(); assert ref() is None; guard.__exit__(None,None,None)


def test_phase_b2a_repeated_gc_cycles(tmp_path):
    guard, _, projection, _ = _locked_snapshot(tmp_path)
    # close the helper snapshot before bounded create/drop cycles
    guard.__exit__(None,None,None)
    for i in range(5):
        cycle = tmp_path / str(i); cycle.mkdir()
        g, snap, _, _ = _locked_snapshot(cycle); snap=None; gc.collect(); g.__exit__(None,None,None)


def test_phase_b2a_forged_finalization_isolation(tmp_path):
    guard, genuine, _, _ = _locked_snapshot(tmp_path); fake=object.__new__(runtime_validation.AuthenticatedRuntimeSnapshot); fake=None; gc.collect()
    assert genuine.read_member_bytes("launch/simulation.launch.py"); genuine.close(); guard.__exit__(None,None,None)


@pytest.mark.parametrize("failure", ["missing_root", "missing_member", "bad_hash", "bad_size", "bad_mode", "symlink_root", "inventory_extra"])
def test_factory_fail_closed_regressions(tmp_path, failure):
    root, projection, _ = runtime_fixture(tmp_path)
    if failure == "missing_root": root = tmp_path / "missing"
    elif failure == "missing_member": (root / "launch/simulation.launch.py").unlink()
    elif failure == "bad_hash": projection["members"][0]["sha256"] = "0" * 64
    elif failure == "bad_size": projection["members"][0]["size_bytes"] += 1
    elif failure == "bad_mode": projection["members"][0]["role"] = "unsupported"
    elif failure == "symlink_root":
        other = tmp_path / "other"; other.mkdir(); root.rename(other); root.symlink_to(other, target_is_directory=True)
    elif failure == "inventory_extra": (root / "extra.txt").write_text("x")
    with pytest.raises(RuntimeValidationError):
        open_authenticated_runtime_snapshot(projection, root)
