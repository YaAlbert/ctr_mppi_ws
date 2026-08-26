"""Installed-runtime, process, environment, ACL, and graph authority for Slice 7G.

All constructors are pure or operate only on a caller-supplied candidate tree.  The
production fixed paths are imported from :mod:`slice_7g_authority_protocol`; this
module never provisions them and exposes no public production-path override.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import stat
from types import MappingProxyType
from typing import Any, Callable
import unicodedata
import re

from .slice_7g_authority_protocol import (
    CAMPAIGN_ACCOUNT,
    CAMPAIGN_SYSTEMD_UNIT,
    ENVIRONMENT_MANIFEST_SCHEMA,
    INSTALLED_RUNTIME_MANIFEST_SCHEMA,
    INSTALLED_RUNTIME_PARENT,
    OUTPUT_PARENT,
    PROCESS_MANIFEST_SCHEMA,
    RUNTIME_GROUP,
    Slice7GAuthorityProtocolError,
    Slice7GPeerProcess,
    authority_record_identity,
    validate_authority_record,
)
from .slice_7g_privileged_protocol import (
    CLEANUP_AUTHORITY_EXECUTABLE,
    CLEANUP_AUTHORITY_SOCKET,
    CLEANUP_AUTHORITY_STATE_ROOT,
    CLEANUP_RECOVERY_SOCKET,
    INSTALLED_RUNTIME_MANIFEST_V2_SCHEMA,
    INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA,
    OBSERVER_SUPERVISOR_CGROUP,
    OBSERVER_SUPERVISOR_EXECUTABLE,
    OBSERVER_SUPERVISOR_SOCKET,
    PRIVILEGED_SERVICE_MANIFEST_SCHEMA,
    record_identity as privileged_record_identity,
    validate_record as validate_privileged_record,
)


TREE_IDENTITY_ALGORITHM = "sha256:ctr-slice-7g-installed-runtime-tree-canonical-1"
TREE_IDENTITY_DOMAIN = b"ctr-slice-7g-installed-runtime-tree-canonical-1\0"
PHYSICAL_TREE_IDENTITY_DOMAIN = b"ctr-slice-7g-installed-runtime-physical-tree-canonical-1\0"
PROCESS_IDENTITY_ALGORITHM = "sha256:ctr-slice-7g-process-manifest-canonical-1"
ENVIRONMENT_IDENTITY_ALGORITHM = "sha256:ctr-slice-7g-environment-manifest-canonical-1"
ACL_POLICY_SCHEMA = "ctr-slice-7g-output-parent-acl-policy-1"
ACL_POLICY_DOMAIN = b"ctr-slice-7g-output-parent-acl-policy-canonical-1\0"
MAX_INSTALLED_MEMBERS = 100_000
MAX_INSTALLED_BYTES = 4 * 1024 * 1024 * 1024

EXPECTED_ROS_NODES = frozenset({
    "/parameter_validator",
    "/ctr_simulator",
    "/safety_supervisor",
    "/mppi_controller",
    "/reference_manager",
    "/evaluation_node",
    "/ctr_run_evaluation_monitor",
})
SAFETY_SUPERVISOR_NODE = "/safety_supervisor"
CAMPAIGN_CGROUP = "/system.slice/ctr-slice7g-campaign.service"
SYSTEMD_TEMPLATE_NAMES = (
    "ctr-slice7g-authority.service.in",
    "ctr-slice7g-campaign.service.in",
    "ctr-slice7g-cleanup-authority.service.in",
    "ctr-slice7g-observer-supervisor.service.in",
    "ctr-slice7g-revocation.path.in",
    "ctr-slice7g-revocation.service.in",
)

GOVERNED_ENVIRONMENT_KEYS = frozenset({
    "PATH", "PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH",
    "LD_LIBRARY_PATH", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_HOME",
    "ROS_DISTRO", "ROS_LOG_DIR", "ROS_LOCALHOST_ONLY", "HOME", "XDG_CACHE_HOME",
    "MPLCONFIGDIR", "PYTHONDONTWRITEBYTECODE",
    "PYTHONNOUSERSITE", "CTR_SLICE_7G_CAMPAIGN_ID", "CTR_SLICE_7G_CELL_ID",
    "CTR_SLICE_7G_OUTPUT_ROOT", "CTR_SLICE_7G_AUTHORIZATION_IDENTITY",
    "CTR_SLICE_7G_RECEIPT_IDENTITY", "CTR_SLICE_7G_CHARTER_IDENTITY",
    "CTR_SLICE_7G_ATTEMPT_LEDGER_IDENTITY", "CTR_SLICE_7G_ATTEMPT_LEDGER_REVISION",
    "CTR_SLICE_7G_PROCESS_START_EVENT_IDENTITY", "CTR_SLICE_7G_CAMPAIGN_PLAN_IDENTITY",
    "CTR_SLICE_7G_DOMAIN_LEASE_IDENTITY", "CTR_SLICE_7G_DOMAIN_COMMITTED_BINDING_IDENTITY",
    "CTR_SLICE_7G_CAMPAIGN_OUTPUT_ROOT", "CTR_SLICE_7G_CELL_OUTPUT_ROOT",
    "CTR_SLICE_7G_WORKING_DIRECTORY",
})


class Slice7GInstalledRuntimeError(RuntimeError):
    def __init__(self, code: str, message: str, *, path: str = "$" ) -> None:
        self.code = code
        self.path = path
        super().__init__(f"{code}:{path}:{message}")


@dataclass(frozen=True)
class InstalledTreeInspection:
    candidate_root: str
    root_device: int
    root_inode: int
    members: tuple[MappingProxyType, ...]
    physical_inodes: tuple[tuple[str, int, int], ...]
    canonical_projection: bytes
    logical_identity: str
    physical_identity: str


@dataclass(frozen=True)
class OutputParentAclPolicy:
    canonical_bytes: bytes
    logical_identity: str
    data: MappingProxyType


@dataclass(frozen=True)
class RosNodeObservation:
    node_name: str
    process: Slice7GPeerProcess
    reconciled_process: Slice7GPeerProcess
    ready: bool
    fault_free: bool
    publishers: tuple[str, ...]
    subscribers: tuple[str, ...]


def inspect_installed_runtime_candidate(root: str | os.PathLike[str]) -> InstalledTreeInspection:
    """Create a path-independent, mode-bound two-pass inventory."""

    root_text = _absolute_path(root, "candidate_root")
    root_fd = _open_directory_nofollow(root_text)
    try:
        root_before = os.fstat(root_fd)
        if stat.S_ISLNK(root_before.st_mode):
            _fail("installed_root_type", "installed root must not be a symlink")
        first, first_physical = _inventory(root_fd)
        second, second_physical = _inventory(root_fd)
        if first != second or first_physical != second_physical:
            _fail("installed_tree_changed", "installed tree changed between authentication passes")
        path_check = _open_directory_nofollow(root_text)
        try:
            reopened = os.fstat(path_check)
            if (reopened.st_dev, reopened.st_ino) != (root_before.st_dev, root_before.st_ino):
                _fail("installed_root_replaced", "installed root pathname was replaced")
        finally:
            os.close(path_check)
        projection = {
            "schema_version": "ctr-slice-7g-installed-runtime-tree-1",
            "identity_algorithm": TREE_IDENTITY_ALGORITHM,
            # The established path-independent tree identity remains the
            # content/type/mode identity.  Charter-v7 manifest v3 separately
            # and canonically binds each member's root ownership without
            # changing the historical v1/v2 tree-identity algorithm.
            "members": [
                {key: value for key, value in item.items()
                 if key not in {"owner_uid", "owner_gid"}}
                for item in first
            ],
        }
        canonical = _canonical(projection)
        identity = hashlib.sha256(TREE_IDENTITY_DOMAIN + canonical).hexdigest()
        physical_projection = _canonical({
            "members": [
                {"device": device, "inode": inode, "path": path}
                for path, device, inode in first_physical
            ],
            "root_device": root_before.st_dev,
            "root_inode": root_before.st_ino,
            "schema_version": "ctr-slice-7g-installed-runtime-physical-tree-1",
        })
        physical_identity = hashlib.sha256(
            PHYSICAL_TREE_IDENTITY_DOMAIN + physical_projection,
        ).hexdigest()
        return InstalledTreeInspection(
            root_text,
            root_before.st_dev,
            root_before.st_ino,
            tuple(MappingProxyType(dict(item)) for item in first),
            tuple(first_physical),
            canonical,
            identity,
            physical_identity,
        )
    finally:
        os.close(root_fd)


def make_installed_runtime_manifest(
    inspection: InstalledTreeInspection,
    *,
    console_entrypoints: list[dict[str, Any]],
    python_modules: list[dict[str, Any]],
    generated_interfaces: list[dict[str, Any]],
    elf_members: list[dict[str, Any]],
    process_manifest_identity: str,
    environment_manifest_identity: str,
    source_snapshot: dict[str, Any],
    build_test_approval_identity: str,
) -> MappingProxyType:
    if type(inspection) is not InstalledTreeInspection:
        _fail("installed_inspection", "installed inspection must be exact")
    data = {
        "schema_version": INSTALLED_RUNTIME_MANIFEST_SCHEMA,
        "identity_algorithm": TREE_IDENTITY_ALGORITHM,
        "installed_runtime_identity": inspection.logical_identity,
        "root_path": f"{INSTALLED_RUNTIME_PARENT}/{inspection.logical_identity}",
        "root_device": inspection.root_device,
        "root_inode": inspection.root_inode,
        "physical_tree_identity": inspection.physical_identity,
        "member_count": len(inspection.members),
        "members": [
            {key: value for key, value in item.items() if key not in {"owner_uid", "owner_gid"}}
            for item in inspection.members
        ],
        "console_entrypoints": _plain_records(console_entrypoints, "console_entrypoints"),
        "python_modules": _plain_records(python_modules, "python_modules"),
        "generated_interfaces": _plain_records(generated_interfaces, "generated_interfaces"),
        "elf_members": _plain_records(elf_members, "elf_members"),
        "process_manifest_identity": process_manifest_identity,
        "environment_manifest_identity": environment_manifest_identity,
        "source_snapshot": _plain_dict(source_snapshot, "source_snapshot"),
        "build_test_approval_identity": build_test_approval_identity,
    }
    return validate_authority_record(data, expected_schema=INSTALLED_RUNTIME_MANIFEST_SCHEMA).data


def make_installed_runtime_manifest_v2(
    inspection: InstalledTreeInspection,
    *,
    console_entrypoints: list[dict[str, Any]],
    python_modules: list[dict[str, Any]],
    generated_interfaces: list[dict[str, Any]],
    elf_members: list[dict[str, Any]],
    process_manifest_identity: str,
    environment_manifest_identity: str,
    source_snapshot: dict[str, Any],
    build_test_approval_identity: str,
    privileged_service_manifest: dict[str, Any],
) -> MappingProxyType:
    """Construct the charter-v7 manifest without changing v1 history."""
    if type(inspection) is not InstalledTreeInspection:
        _fail("installed_inspection", "installed inspection must be exact")
    service = validate_privileged_service_manifest(privileged_service_manifest)
    data = {
        "schema_version": INSTALLED_RUNTIME_MANIFEST_V2_SCHEMA,
        "identity_algorithm": TREE_IDENTITY_ALGORITHM,
        "installed_runtime_identity": inspection.logical_identity,
        "root_path": f"{INSTALLED_RUNTIME_PARENT}/{inspection.logical_identity}",
        "root_device": inspection.root_device,
        "root_inode": inspection.root_inode,
        "physical_tree_identity": inspection.physical_identity,
        "member_count": len(inspection.members),
        "members": [
            {key: value for key, value in item.items() if key not in {"owner_uid", "owner_gid"}}
            for item in inspection.members
        ],
        "console_entrypoints": _plain_records(console_entrypoints, "console_entrypoints"),
        "python_modules": _plain_records(python_modules, "python_modules"),
        "generated_interfaces": _plain_records(generated_interfaces, "generated_interfaces"),
        "elf_members": _plain_records(elf_members, "elf_members"),
        "process_manifest_identity": process_manifest_identity,
        "environment_manifest_identity": environment_manifest_identity,
        "source_snapshot": _plain_dict(source_snapshot, "source_snapshot"),
        "build_test_approval_identity": build_test_approval_identity,
        "privileged_service_manifest_identity": privileged_record_identity(
            dict(service), expected_schema=PRIVILEGED_SERVICE_MANIFEST_SCHEMA,
        ),
    }
    return validate_privileged_record(
        data, expected_schema=INSTALLED_RUNTIME_MANIFEST_V2_SCHEMA,
    ).data


def make_installed_runtime_manifest_v3(
    inspection: InstalledTreeInspection,
    *,
    console_entrypoints: list[dict[str, Any]],
    python_modules: list[dict[str, Any]],
    generated_interfaces: list[dict[str, Any]],
    elf_members: list[dict[str, Any]],
    process_manifest_identity: str,
    environment_manifest_identity: str,
    source_snapshot: dict[str, Any],
    build_test_approval_identity: str,
    privileged_service_manifest: dict[str, Any],
) -> MappingProxyType:
    """Construct the recursively closed, ownership-bound charter-v7 manifest."""
    if type(inspection) is not InstalledTreeInspection:
        _fail("installed_inspection", "installed inspection must be exact")
    service = validate_privileged_service_manifest(privileged_service_manifest)
    data = {
        "schema_version": INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA,
        "identity_algorithm": TREE_IDENTITY_ALGORITHM,
        "installed_runtime_identity": inspection.logical_identity,
        "root_path": f"{INSTALLED_RUNTIME_PARENT}/{inspection.logical_identity}",
        "root_device": inspection.root_device,
        "root_inode": inspection.root_inode,
        "physical_tree_identity": inspection.physical_identity,
        "member_count": len(inspection.members),
        "members": [dict(item) for item in inspection.members],
        "console_entrypoints": _plain_records(console_entrypoints, "console_entrypoints"),
        "python_modules": _plain_records(python_modules, "python_modules"),
        "generated_interfaces": _plain_records(generated_interfaces, "generated_interfaces"),
        "elf_members": _plain_records(elf_members, "elf_members"),
        "process_manifest_identity": process_manifest_identity,
        "environment_manifest_identity": environment_manifest_identity,
        "source_snapshot": _plain_dict(source_snapshot, "source_snapshot"),
        "build_test_approval_identity": build_test_approval_identity,
        "privileged_service_manifest_identity": privileged_record_identity(
            dict(service), expected_schema=PRIVILEGED_SERVICE_MANIFEST_SCHEMA,
        ),
    }
    return validate_privileged_record(
        data, expected_schema=INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA,
    ).data


def validate_privileged_service_manifest(value: dict[str, Any]) -> MappingProxyType:
    """Authenticate the two root helpers and their fixed authority surface."""
    record = validate_privileged_record(
        value, expected_schema=PRIVILEGED_SERVICE_MANIFEST_SCHEMA,
    )
    if tuple(record.data["cleanup_schemas"]) != (
        "ctr-slice-7g-cleanup-authority-revision-1",
        "ctr-slice-7g-cleanup-authority-anchor-1",
        "ctr-slice-7g-cleanup-authority-head-1",
    ):
        _fail("privileged_service_manifest", "cleanup schema inventory differs")
    if (
        len(record.data["service_executable_identities"]) != 2
        or len(set(record.data["service_executable_identities"])) != 2
    ):
        _fail("privileged_service_manifest", "service executable inventory differs")
    if (
        len(record.data["systemd_unit_identities"]) != 2
        or len(set(record.data["systemd_unit_identities"])) != 2
    ):
        _fail("privileged_service_manifest", "privileged unit inventory differs")
    return record.data


def authenticate_installed_runtime(
    manifest: dict[str, Any],
) -> InstalledTreeInspection:
    """Authenticate the final fixed-parent tree and reject candidate-path substitution."""

    if type(manifest) is not dict:
        _fail("installed_manifest", "installed manifest must be an exact dictionary")
    schema = manifest.get("schema_version")
    if schema in {
        INSTALLED_RUNTIME_MANIFEST_V2_SCHEMA, INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA,
    }:
        record = validate_privileged_record(
            manifest, expected_schema=schema,
        )
    elif schema == INSTALLED_RUNTIME_MANIFEST_SCHEMA:
        record = validate_authority_record(
            manifest, expected_schema=INSTALLED_RUNTIME_MANIFEST_SCHEMA,
        )
    else:
        _fail("installed_manifest_schema", "installed manifest schema is unsupported")
    expected_root = record.data["root_path"]
    if PurePosixPath(expected_root).parent.as_posix() != INSTALLED_RUNTIME_PARENT:
        _fail("installed_root_parent", "installed root parent differs")
    _authenticate_immutable_parent_chain(expected_root)
    observed = inspect_installed_runtime_candidate(expected_root)
    if observed.logical_identity != record.data["installed_runtime_identity"]:
        _fail("installed_runtime_identity", "installed runtime logical identity differs")
    if (
        observed.root_device != record.data["root_device"]
        or observed.root_inode != record.data["root_inode"]
        or observed.physical_identity != record.data["physical_tree_identity"]
    ):
        _fail("installed_runtime_physical_identity", "installed runtime physical identity differs")
    expected_members = tuple(dict(member) for member in record.data["members"])
    if tuple(dict(member) for member in observed.members) != expected_members:
        _fail("installed_runtime_inventory", "installed runtime inventory differs")
    root_fd = _open_directory_nofollow(expected_root)
    try:
        root_info = os.fstat(root_fd)
        if root_info.st_uid != 0 or stat.S_IMODE(root_info.st_mode) & 0o222:
            _fail("installed_runtime_ownership", "final installed root is not root-owned and immutable")
        for member in expected_members:
            descriptor = root_fd
            opened: list[int] = []
            try:
                parts = PurePosixPath(member["path"]).parts
                for index, part in enumerate(parts):
                    flags = os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW
                    if index < len(parts) - 1 or member["type"] == "directory":
                        flags |= os.O_DIRECTORY
                    child = os.open(part, flags, dir_fd=descriptor)
                    opened.append(child)
                    descriptor = child
                info = os.fstat(descriptor)
                if info.st_uid != 0 or stat.S_IMODE(info.st_mode) & 0o222:
                    _fail("installed_runtime_ownership", "final installed member is not root-owned and immutable", path=member["path"])
            finally:
                for descriptor in reversed(opened):
                    os.close(descriptor)
    finally:
        os.close(root_fd)
    return observed


def _authenticate_immutable_parent_chain(path: str) -> None:
    descriptor = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        parts = PurePosixPath(path).parts[1:]
        for index, part in enumerate(parts):
            child = os.open(
                part, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW,
                dir_fd=descriptor,
            )
            info = os.fstat(child)
            if info.st_uid != 0 or (stat.S_IMODE(info.st_mode) & (0o222 if index == len(parts) - 1 else 0o022)):
                os.close(child)
                _fail("installed_parent_authority", "installed-runtime parent chain is caller/service writable", path=part)
            os.close(descriptor)
            descriptor = child
    finally:
        os.close(descriptor)


def make_environment_manifest(
    *,
    fixed_values: dict[str, str],
    transaction_values: dict[str, str],
    required_absent_keys: list[str],
    path_keys: list[str],
) -> MappingProxyType:
    fixed = _plain_string_map(fixed_values, "fixed_values")
    generated = _plain_string_map(transaction_values, "transaction_values")
    if set(fixed) | set(generated) != set(GOVERNED_ENVIRONMENT_KEYS) - set(required_absent_keys):
        _fail("environment_coverage", "environment manifest does not exactly govern every permitted key")
    data = {
        "schema_version": ENVIRONMENT_MANIFEST_SCHEMA,
        "identity_algorithm": ENVIRONMENT_IDENTITY_ALGORITHM,
        "allowed_keys": sorted(set(fixed) | set(generated)),
        "required_keys": sorted(set(fixed) | set(generated)),
        "required_absent_keys": sorted(required_absent_keys),
        "fixed_values": fixed,
        "transaction_values": generated,
        "path_keys": list(path_keys),
        "path_order": list(path_keys),
        "inherit_parent_environment": False,
    }
    record = validate_authority_record(data, expected_schema=ENVIRONMENT_MANIFEST_SCHEMA)
    return record.data


def instantiate_closed_environment(
    manifest: dict[str, Any] | MappingProxyType, transaction: dict[str, str],
) -> MappingProxyType:
    source = _thaw(manifest) if type(manifest) is MappingProxyType else manifest
    record = validate_authority_record(source, expected_schema=ENVIRONMENT_MANIFEST_SCHEMA)
    supplied = _plain_string_map(transaction, "transaction")
    required_slots = set(record.data["transaction_values"].values())
    if set(supplied) != required_slots:
        _fail("environment_slots", "transaction environment slots differ")
    environment = dict(record.data["fixed_values"])
    for key, slot in record.data["transaction_values"].items():
        environment[key] = supplied[slot]
    if set(environment) != set(record.data["required_keys"]):
        _fail("environment_keys", "instantiated environment keys differ")
    if set(environment) & set(record.data["required_absent_keys"]):
        _fail("environment_absent", "required-absent environment key is present")
    for key in record.data["path_keys"]:
        _validate_path_list(environment[key], key)
    return MappingProxyType(environment)


def make_process_manifest(
    *,
    interpreter: dict[str, Any],
    interpreter_flags: list[str],
    entrypoint: dict[str, Any],
    executables: list[dict[str, Any]],
    argv_template: list[str],
    transaction_slots: dict[str, str],
    environment_manifest_identity: str,
    working_directory: str,
    allowed_descendants: list[dict[str, Any]],
    timeouts: dict[str, float],
    output_ownership: dict[str, Any],
    required_receipts: list[str],
) -> MappingProxyType:
    data = {
        "schema_version": PROCESS_MANIFEST_SCHEMA,
        "identity_algorithm": PROCESS_IDENTITY_ALGORITHM,
        "interpreter": _plain_dict(interpreter, "interpreter"),
        "interpreter_flags": list(interpreter_flags),
        "entrypoint": _plain_dict(entrypoint, "entrypoint"),
        "executables": _plain_records(executables, "executables"),
        "argv_template": list(argv_template),
        "transaction_slots": _plain_string_map(transaction_slots, "transaction_slots"),
        "environment_manifest_identity": environment_manifest_identity,
        "working_directory": working_directory,
        "shell": False,
        "systemd_unit": CAMPAIGN_SYSTEMD_UNIT,
        "cgroup": CAMPAIGN_CGROUP,
        "allowed_descendants": _plain_records(allowed_descendants, "allowed_descendants"),
        "timeouts": _plain_dict(timeouts, "timeouts"),
        "output_ownership": _plain_dict(output_ownership, "output_ownership"),
        "required_receipts": list(required_receipts),
    }
    return validate_authority_record(data, expected_schema=PROCESS_MANIFEST_SCHEMA).data


def instantiate_process(
    process_manifest: dict[str, Any] | MappingProxyType,
    environment_manifest: dict[str, Any] | MappingProxyType,
    slots: dict[str, str],
) -> MappingProxyType:
    process_source = _thaw(process_manifest) if type(process_manifest) is MappingProxyType else process_manifest
    environment_source = _thaw(environment_manifest) if type(environment_manifest) is MappingProxyType else environment_manifest
    process = validate_authority_record(process_source, expected_schema=PROCESS_MANIFEST_SCHEMA)
    environment = validate_authority_record(environment_source, expected_schema=ENVIRONMENT_MANIFEST_SCHEMA)
    if authority_record_identity(dict(environment.data), expected_schema=ENVIRONMENT_MANIFEST_SCHEMA) != process.data["environment_manifest_identity"]:
        _fail("process_environment_identity", "process and environment manifest identities differ")
    supplied = _plain_string_map(slots, "process_slots")
    if set(supplied) != set(process.data["transaction_slots"].values()):
        _fail("process_slots", "process transaction slots differ")
    argv = []
    for token in process.data["argv_template"]:
        if token.startswith("{") and token.endswith("}"):
            name = token[1:-1]
            if name not in process.data["transaction_slots"]:
                _fail("process_slot", "argv contains an unbound slot")
            argv.append(supplied[process.data["transaction_slots"][name]])
        else:
            argv.append(token)
    entrypoint = process.data["entrypoint"]
    interpreter = process.data["interpreter"]
    if not entrypoint["path"].startswith(f"{INSTALLED_RUNTIME_PARENT}/"):
        _fail("process_entrypoint_origin", "entrypoint is not beneath installed-runtime authority")
    return MappingProxyType({
        "executable": interpreter["path"],
        "argv": tuple(argv),
        "environment": instantiate_closed_environment(dict(environment.data), supplied),
        "working_directory": process.data["working_directory"],
        "shell": False,
        "cgroup": process.data["cgroup"],
    })


def output_parent_acl_policy() -> OutputParentAclPolicy:
    """Return the deterministic ACL that a later privileged task must apply."""

    data = {
        "schema_version": ACL_POLICY_SCHEMA,
        "output_parent": OUTPUT_PARENT,
        "parent_owner": "ankid",
        "parent_group": "ankid",
        "parent_mode": 0o750,
        "access_entries": [
            "user::rwx", "user:ctr7g-authority:rwx", "user:ctr7g-campaign:--x",
            "group::---", "mask::rwx", "other::---",
        ],
        "default_entries": [],
        "campaign_root_owner": "ctr7g-authority",
        "campaign_root_group": RUNTIME_GROUP,
        "campaign_root_mode": 0o750,
        "cell_output_owner": CAMPAIGN_ACCOUNT,
        "cell_output_group": RUNTIME_GROUP,
        "cell_output_mode": 0o770,
        "campaign_parent_create_remove_list": False,
        "authority_creates_campaign_root": True,
    }
    canonical = _canonical(data)
    return OutputParentAclPolicy(canonical, hashlib.sha256(ACL_POLICY_DOMAIN + canonical).hexdigest(), _freeze(data))


def validate_output_parent_acl_policy(value: dict[str, Any]) -> OutputParentAclPolicy:
    if type(value) is not dict:
        _fail("acl_policy_type", "ACL policy must be an exact dictionary")
    expected = output_parent_acl_policy()
    if _canonical(value) != expected.canonical_bytes:
        _fail("acl_policy", "output-parent ACL policy differs")
    return expected


def render_and_verify_systemd_templates(
    templates: dict[str, str],
    *,
    installed_runtime_root: str,
    working_directory: str,
    environment_values: dict[str, str],
    timeouts: dict[str, float],
) -> MappingProxyType:
    """Render the six reviewed unit templates from authenticated manifest values."""

    if type(templates) is not dict or set(templates) != set(SYSTEMD_TEMPLATE_NAMES):
        _fail("systemd_templates", "exactly six named templates are required")
    if any(type(key) is not str or type(value) is not str for key, value in templates.items()):
        _fail("systemd_templates", "template keys and values must be exact strings")
    root = _absolute_path(installed_runtime_root, "installed_runtime_root")
    if PurePosixPath(root).parent.as_posix() != INSTALLED_RUNTIME_PARENT:
        _fail("systemd_installed_root", "systemd installed root differs from fixed parent")
    cwd = _absolute_path(working_directory, "working_directory")
    values = _plain_string_map(environment_values, "environment_values")
    required_environment = {
        "PATH", "PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH",
        "LD_LIBRARY_PATH", "RMW_IMPLEMENTATION", "ROS_HOME", "ROS_LOG_DIR",
        "ROS_LOCALHOST_ONLY",
    }
    if set(values) != required_environment:
        _fail("systemd_environment", "systemd environment renderer keys differ")
    timeout_values = _plain_dict(timeouts, "timeouts")
    if set(timeout_values) != {"sigint_seconds", "sigterm_seconds", "sigkill_seconds", "cell_seconds"}:
        _fail("systemd_timeouts", "systemd timeout fields differ")
    if any(type(value) not in (int, float) or type(value) is bool or value <= 0 for value in timeout_values.values()):
        _fail("systemd_timeouts", "systemd timeout values must be positive exact numbers")
    total = sum(float(timeout_values[key]) for key in ("sigint_seconds", "sigterm_seconds", "sigkill_seconds"))
    substitutions = {
        "INSTALLED_RUNTIME_ROOT": root,
        "WORKING_DIRECTORY": cwd,
        "TOTAL_STOP_TIMEOUT_SECONDS": _canonical_number(total),
        **values,
    }
    rendered: dict[str, str] = {}
    for name in SYSTEMD_TEMPLATE_NAMES:
        content = templates[name]
        observed = set(re.findall(r"@([A-Z][A-Z0-9_]*)@", content))
        unknown = observed - set(substitutions)
        if unknown:
            _fail("systemd_placeholder", f"unknown systemd placeholders: {sorted(unknown)!r}")
        for key in sorted(observed):
            replacement = substitutions[key]
            if any(character in replacement for character in "\r\n\0"):
                _fail("systemd_placeholder", "systemd replacement contains a control character")
            content = content.replace(f"@{key}@", replacement)
        if re.search(r"@[A-Z][A-Z0-9_]*@", content):
            _fail("systemd_placeholder", "systemd template retains a placeholder")
        _verify_systemd_unit(name, content, root, cwd, _canonical_number(total))
        rendered[name[:-3]] = content
    return MappingProxyType(rendered)


def _verify_systemd_unit(name: str, content: str, root: str, cwd: str, total: str) -> None:
    if "EnvironmentFile=" in content or "ExecStart=/bin/sh" in content or "ExecStart=/usr/bin/env" in content:
        _fail("systemd_shell_or_override", "systemd unit permits a shell or caller environment")
    if name == "ctr-slice7g-campaign.service.in":
        required = (
            "User=ctr7g-campaign", "Group=ctr7g-runtime", "SupplementaryGroups=",
            "NoNewPrivileges=yes", "CapabilityBoundingSet=\n", "AmbientCapabilities=\n",
            "KillMode=control-group", "Delegate=no", "ProtectControlGroups=yes",
            "SendSIGKILL=yes", f"TimeoutStopSec={total}s", f"WorkingDirectory={cwd}",
            f"ExecStart=/usr/bin/python3.10 -I {root}/lib/python3.10/site-packages/ctr_evaluation/slice_7g_runtime.py",
            "InaccessiblePaths=/run/systemd/private /run/dbus/system_bus_socket",
        )
        if any(item not in content for item in required):
            _fail("systemd_campaign", "campaign systemd unit lacks a required invariant")
        for prohibited in ("sudo", " su ", "systemctl", "docker", "lxc", "Delegate=yes"):
            if prohibited in content:
                _fail("systemd_campaign", "campaign systemd unit contains a prohibited capability")
    elif name == "ctr-slice7g-authority.service.in":
        if "User=ctr7g-authority" not in content or "ExecStart=/usr/libexec/ctr-mppi/ctr-slice7g-authorityd" not in content:
            _fail("systemd_authority", "authority systemd unit differs")
        if "CAP_KILL" in content or "sudo" in content:
            _fail("systemd_authority", "authority unit grants a prohibited privilege")
        if "Delegate=yes" in content or "ProtectControlGroups=yes" not in content:
            _fail("systemd_authority", "authority unit cgroup protection differs")
    elif name == "ctr-slice7g-cleanup-authority.service.in":
        required = (
            "User=root", "Group=root",
            "ExecStart=/usr/libexec/ctr-mppi/ctr-slice7g-cleanupd",
            "NoNewPrivileges=yes", "CapabilityBoundingSet=CAP_CHOWN CAP_DAC_READ_SEARCH",
            "PrivateNetwork=yes", "RestrictAddressFamilies=AF_UNIX",
            "ProtectControlGroups=yes", "Delegate=no",
            "SystemCallFilter=~fork vfork clone clone3",
            "ReadWritePaths=/var/lib/ctr-mppi/slice-7g-cleanup-authority /run/ctr-mppi/slice-7g-cleanup-authority",
            "RuntimeDirectory=ctr-mppi/slice-7g-cleanup-authority",
            "RuntimeDirectoryMode=0755",
        )
        if any(item not in content for item in required):
            _fail("systemd_cleanup_authority", "cleanup authority unit differs")
    elif name == "ctr-slice7g-observer-supervisor.service.in":
        required = (
            "User=root", "Group=root",
            "ExecStart=/usr/libexec/ctr-mppi/ctr-slice7g-observerd",
            "NoNewPrivileges=yes", "Delegate=yes", "ProtectControlGroups=no",
            "CapabilityBoundingSet=CAP_CHOWN CAP_DAC_READ_SEARCH CAP_SETUID CAP_SETGID CAP_KILL",
            "KillMode=control-group", "SendSIGKILL=yes", "TimeoutStopSec=5s",
            "RuntimeDirectory=ctr-mppi/slice-7g-observer-supervisor ctr-slice7g-observer",
            "RuntimeDirectoryMode=0755",
            "InaccessiblePaths=/run/systemd/private /run/dbus/system_bus_socket",
        )
        if any(item not in content for item in required):
            _fail("systemd_observer_supervisor", "observer supervisor unit differs")
        if "AmbientCapabilities=CAP_" in content:
            _fail("systemd_observer_supervisor", "observer supervisor has ambient capability")
    elif name == "ctr-slice7g-revocation.path.in":
        if "DirectoryNotEmpty=/var/lib/ctr-mppi/slice-7g-authority/revocation/pending" not in content or "MakeDirectory=no" not in content:
            _fail("systemd_revocation_path", "revocation path contract differs")
    elif name == "ctr-slice7g-revocation.service.in":
        if "User=root" not in content or "--enforce-revocation" not in content:
            _fail("systemd_revocation_service", "revocation service contract differs")


def _canonical_number(value: float) -> str:
    if not value.is_integer():
        return format(value, ".15g")
    return str(int(value))


def authenticate_ros_node_authority(
    observations: list[RosNodeObservation],
    *,
    campaign_cgroup: str = CAMPAIGN_CGROUP,
) -> tuple[RosNodeObservation, ...]:
    """Require the exact seven-node set and an owned, ready supervisor child."""

    if type(observations) is not list or any(type(item) is not RosNodeObservation for item in observations):
        _fail("ros_node_observations", "ROS node observations must be an exact list of exact records")
    names = [item.node_name for item in observations]
    if len(names) != len(set(names)):
        _fail("ros_node_duplicate", "ROS node observations contain a duplicate")
    if set(names) != set(EXPECTED_ROS_NODES):
        _fail("ros_node_set", "ROS node set differs from the charter authority")
    process_owners: set[tuple[int, int]] = set()
    for item in observations:
        if type(item.ready) is not bool or type(item.fault_free) is not bool:
            _fail("ros_node_state", "ROS node readiness must use exact booleans")
        if item.process != item.reconciled_process:
            _fail("ros_node_process_replaced", "ROS node PID/start-time/executable observation changed")
        if item.process.cgroup != campaign_cgroup:
            _fail("ros_node_cgroup", "ROS node process is outside the campaign cgroup")
        owner = (item.process.credentials.pid, item.process.start_time_ticks)
        if owner in process_owners:
            _fail("ros_node_process_duplicate", "two expected ROS nodes claim one process identity")
        process_owners.add(owner)
        executable = item.process.executable
        if not (
            executable.startswith(INSTALLED_RUNTIME_PARENT + "/")
            or executable.startswith("/opt/ros/humble/")
            or executable == "/usr/bin/python3"
        ):
            _fail("ros_node_executable", "ROS node executable origin is not authorized")
    supervisor = next(item for item in observations if item.node_name == SAFETY_SUPERVISOR_NODE)
    if not supervisor.ready or not supervisor.fault_free:
        _fail("safety_supervisor_state", "safety supervisor is not ready and fault-free")
    if "/ctr/mppi_command" not in supervisor.subscribers or "/ctr/safe_command" not in supervisor.publishers:
        _fail("safety_supervisor_route", "safety-supervisor command route differs")
    simulator = next(item for item in observations if item.node_name == "/ctr_simulator")
    if "/ctr/safe_command" not in simulator.subscribers:
        _fail("simulator_safe_route", "simulator does not consume the supervised command")
    return tuple(observations)


class OwnedResourceRollback:
    """LIFO pre-commit cleanup with identity checks and BaseException preservation."""

    def __init__(self) -> None:
        self._steps: list[tuple[str, Callable[[], None]]] = []
        self.committed = False
        self.closed = False

    def own(self, name: str, cleanup: Callable[[], None]) -> None:
        if self.committed or self.closed or type(name) is not str or not name or not callable(cleanup):
            _fail("rollback_registration", "rollback resource registration is invalid")
        self._steps.append((name, cleanup))

    def mark_committed(self) -> None:
        if self.closed:
            _fail("rollback_closed", "rollback transaction is closed")
        self.committed = True

    def rollback(self, primary: BaseException | None = None) -> tuple[tuple[str, str], ...]:
        if self.closed:
            return ()
        self.closed = True
        if self.committed:
            return ()
        issues: list[tuple[str, str]] = []
        for name, cleanup in reversed(self._steps):
            try:
                cleanup()
            except BaseException as exc:  # cleanup must continue for every registered resource
                issues.append((name, type(exc).__name__))
        if primary is not None:
            if issues:
                try:
                    primary.add_note(f"Slice 7G rollback issues: {issues!r}")
                except (AttributeError, TypeError):
                    pass
            raise primary
        return tuple(issues)


def _inventory(root_fd: int) -> tuple[list[dict[str, Any]], list[tuple[str, int, int]]]:
    members: list[dict[str, Any]] = []
    physical: set[tuple[int, int]] = set()
    physical_by_path: dict[str, tuple[int, int]] = {}
    total_bytes = 0

    def descend(directory_fd: int, prefix: str) -> None:
        nonlocal total_bytes
        entries = sorted(os.scandir(directory_fd), key=lambda entry: entry.name)
        for entry in entries:
            if entry.name in (".", "..") or "/" in entry.name or "\0" in entry.name:
                _fail("installed_member_name", "installed member name is unsafe")
            relative = f"{prefix}/{entry.name}" if prefix else entry.name
            _safe_relative(relative)
            observed = entry.stat(follow_symlinks=False)
            identity = (observed.st_dev, observed.st_ino)
            if identity in physical:
                _fail("installed_inode_alias", "installed tree contains a physical inode alias", path=relative)
            physical.add(identity)
            physical_by_path[relative] = identity
            mode = stat.S_IMODE(observed.st_mode)
            if stat.S_ISLNK(observed.st_mode):
                _fail("installed_symlink", "installed tree contains a symlink", path=relative)
            if stat.S_ISDIR(observed.st_mode):
                members.append({
                    "path": relative, "type": "directory", "mode": mode,
                    "owner_uid": observed.st_uid, "owner_gid": observed.st_gid,
                    "link_count": observed.st_nlink, "size": None, "sha256": None,
                })
                child = os.open(entry.name, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=directory_fd)
                try:
                    descend(child, relative)
                    after = os.fstat(child)
                    if (after.st_dev, after.st_ino, stat.S_IMODE(after.st_mode)) != (observed.st_dev, observed.st_ino, mode):
                        _fail("installed_directory_changed", "installed directory changed during scan", path=relative)
                finally:
                    os.close(child)
            elif stat.S_ISREG(observed.st_mode):
                if observed.st_nlink != 1:
                    _fail("installed_hardlink", "installed file must be single-link", path=relative)
                descriptor = os.open(entry.name, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=directory_fd)
                try:
                    digest = hashlib.sha256()
                    size = 0
                    while True:
                        chunk = os.read(descriptor, 1_048_576)
                        if not chunk:
                            break
                        digest.update(chunk)
                        size += len(chunk)
                        total_bytes += len(chunk)
                        if total_bytes > MAX_INSTALLED_BYTES:
                            _fail("installed_tree_size", "installed tree exceeds byte ceiling")
                    after = os.fstat(descriptor)
                    if (after.st_dev, after.st_ino, after.st_size, stat.S_IMODE(after.st_mode)) != (observed.st_dev, observed.st_ino, observed.st_size, mode) or size != observed.st_size:
                        _fail("installed_file_changed", "installed file changed during hash", path=relative)
                finally:
                    os.close(descriptor)
                members.append({
                    "path": relative, "type": "regular", "mode": mode,
                    "owner_uid": observed.st_uid, "owner_gid": observed.st_gid,
                    "link_count": observed.st_nlink, "size": size,
                    "sha256": digest.hexdigest(),
                })
            else:
                _fail("installed_member_type", "installed tree contains a non-file member", path=relative)
            if len(members) > MAX_INSTALLED_MEMBERS:
                _fail("installed_member_count", "installed tree exceeds member ceiling")

    descend(root_fd, "")
    ordered = sorted(members, key=lambda item: item["path"])
    inode_projection = [
        (item["path"], physical_by_path[item["path"]][0], physical_by_path[item["path"]][1])
        for item in ordered
    ]
    return ordered, inode_projection


def _open_directory_nofollow(path: str) -> int:
    parts = PurePosixPath(path).parts[1:]
    descriptor = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        for part in parts:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def _validate_path_list(value: str, key: str) -> None:
    entries = value.split(":")
    if not entries or any(not item for item in entries) or len(entries) != len(set(entries)):
        _fail("environment_path_list", "path list is empty or duplicated", path=key)
    for item in entries:
        normalized = _absolute_path(item, key)
        if normalized.startswith("/home/ankid/ctr_mppi_ws") or "/build/" in normalized:
            _fail("environment_path_origin", "path list contains source/build authority", path=key)


def _plain_records(value: Any, path: str) -> list[dict[str, Any]]:
    if type(value) is not list:
        _fail("record_list", "record list must be exact", path=path)
    return [_plain_dict(item, f"{path}[{index}]") for index, item in enumerate(value)]


def _plain_dict(value: Any, path: str) -> dict[str, Any]:
    if type(value) is not dict:
        _fail("record_type", "record must be an exact dictionary", path=path)
    return json.loads(json.dumps(value, ensure_ascii=False, allow_nan=False))


def _plain_string_map(value: Any, path: str) -> dict[str, str]:
    if type(value) is not dict or any(type(key) is not str or type(member) is not str for key, member in value.items()):
        _fail("string_map", "value must be an exact string map", path=path)
    return dict(value)


def _absolute_path(value: str | os.PathLike[str], path: str) -> str:
    if type(value) is not str:
        _fail("path_type", "path must be an exact string", path=path)
    candidate = PurePosixPath(value)
    if not candidate.is_absolute() or "\\" in value or "//" in value or any(part in (".", "..") for part in candidate.parts):
        _fail("absolute_path", "path must be absolute and normalized", path=path)
    return value


def _safe_relative(value: str) -> str:
    if type(value) is not str or unicodedata.normalize("NFC", value) != value:
        _fail("relative_path", "relative path must be an exact NFC string")
    candidate = PurePosixPath(value)
    if candidate.is_absolute() or value.startswith("./") or "\\" in value or "//" in value or any(part in ("", ".", "..") for part in candidate.parts):
        _fail("relative_path", "relative path is unsafe", path=value)
    return value


def _canonical(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _freeze(value: Any) -> Any:
    if type(value) is dict:
        return MappingProxyType({key: _freeze(member) for key, member in value.items()})
    if type(value) is list:
        return tuple(_freeze(member) for member in value)
    return value


def _thaw(value: Any) -> Any:
    if type(value) is MappingProxyType:
        return {key: _thaw(member) for key, member in value.items()}
    if type(value) is tuple:
        return [_thaw(member) for member in value]
    return value


def _fail(code: str, message: str, *, path: str = "$") -> None:
    raise Slice7GInstalledRuntimeError(code, message, path=path)


__all__ = [
    "ACL_POLICY_SCHEMA", "CAMPAIGN_CGROUP", "EXPECTED_ROS_NODES", "InstalledTreeInspection",
    "OutputParentAclPolicy", "OwnedResourceRollback", "RosNodeObservation",
    "SAFETY_SUPERVISOR_NODE", "Slice7GInstalledRuntimeError", "authenticate_installed_runtime",
    "authenticate_ros_node_authority", "inspect_installed_runtime_candidate",
    "instantiate_closed_environment", "instantiate_process", "make_environment_manifest",
    "make_installed_runtime_manifest", "make_installed_runtime_manifest_v2",
    "make_installed_runtime_manifest_v3",
    "make_process_manifest", "output_parent_acl_policy",
    "render_and_verify_systemd_templates", "validate_output_parent_acl_policy",
    "validate_privileged_service_manifest",
]
