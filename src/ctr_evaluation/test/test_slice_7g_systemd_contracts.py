from __future__ import annotations

import hashlib
import os
from pathlib import Path
import runpy

import pytest


ROOT = Path(__file__).parents[1]
SYSTEMD = ROOT / "resource" / "systemd"
LIBEXEC = ROOT / "resource" / "libexec"


def _unit(name):
    return (SYSTEMD / name).read_text()


def test_exact_six_systemd_templates_exist():
    assert {item.name for item in SYSTEMD.glob("ctr-slice7g-*.in")} == {
        "ctr-slice7g-authority.service.in",
        "ctr-slice7g-campaign.service.in",
        "ctr-slice7g-cleanup-authority.service.in",
        "ctr-slice7g-observer-supervisor.service.in",
        "ctr-slice7g-revocation.path.in",
        "ctr-slice7g-revocation.service.in",
    }


def test_cleanup_service_is_root_fixed_and_non_delegated():
    unit = _unit("ctr-slice7g-cleanup-authority.service.in")
    for required in (
        "User=root", "Group=root",
        "ExecStart=/usr/libexec/ctr-mppi/ctr-slice7g-cleanupd",
        "NoNewPrivileges=yes", "CapabilityBoundingSet=CAP_CHOWN CAP_DAC_READ_SEARCH",
        "AmbientCapabilities=\n",
        "PrivateNetwork=yes", "RestrictAddressFamilies=AF_UNIX",
        "ProtectControlGroups=yes", "Delegate=no", "Restart=no",
    ):
        assert required in unit
    assert "CAP_KILL" not in unit


def test_observer_supervisor_is_only_delegated_slice7g_unit():
    delegated = []
    for path in SYSTEMD.glob("ctr-slice7g-*.in"):
        if "Delegate=yes" in path.read_text():
            delegated.append(path.name)
    assert delegated == ["ctr-slice7g-observer-supervisor.service.in"]


def test_observer_supervisor_capabilities_and_cgroup_policy_are_narrow():
    unit = _unit("ctr-slice7g-observer-supervisor.service.in")
    for required in (
        "User=root", "Group=root",
        "ExecStart=/usr/libexec/ctr-mppi/ctr-slice7g-observerd",
        "CapabilityBoundingSet=CAP_CHOWN CAP_DAC_READ_SEARCH CAP_SETUID CAP_SETGID CAP_KILL",
        "AmbientCapabilities=\n", "Delegate=yes", "ProtectControlGroups=no",
        "KillMode=control-group", "SendSIGKILL=yes", "TimeoutStopSec=5s",
    ):
        assert required in unit
    assert "systemctl" not in unit
    assert "sudo" not in unit


def test_authority_and_campaign_remain_non_delegated():
    for name in (
        "ctr-slice7g-authority.service.in",
        "ctr-slice7g-campaign.service.in",
    ):
        unit = _unit(name)
        assert "Delegate=no" in unit
        assert "ProtectControlGroups=yes" in unit


def test_wrappers_are_fixed_isolated_python_without_shell():
    for name in ("ctr-slice7g-cleanupd", "ctr-slice7g-observerd"):
        source = (LIBEXEC / name).read_text()
        assert source.startswith("#!/usr/bin/python3.10 -I")
        assert "subprocess" not in source
        assert "/bin/sh" not in source
        assert "os.system" not in source


def test_setup_registers_all_three_libexec_resources():
    source = (ROOT / "setup.py").read_text()
    for name in (
        "ctr-slice7g-authorityd", "ctr-slice7g-cleanupd", "ctr-slice7g-observerd",
    ):
        assert f'"resource/libexec/{name}"' in source


@pytest.mark.parametrize("wrapper", ["ctr-slice7g-cleanupd", "ctr-slice7g-observerd"])
def test_root_wrapper_rejects_authority_manifest_code_redirection(wrapper):
    namespace = runpy.run_path(str(LIBEXEC / wrapper))
    trusted = {"privileged_code": {"installed_root": "/opt/ctr-mppi/slice-7g/trusted"}}
    assert namespace["_select_privileged_root"](
        trusted, {"root_path": "/opt/ctr-mppi/slice-7g/trusted"},
    ) == "/opt/ctr-mppi/slice-7g/trusted"
    with pytest.raises(RuntimeError):
        namespace["_select_privileged_root"](
            trusted, {"root_path": "/tmp/authority-controlled-code"},
        )


def test_root_wrapper_final_barrier_rejects_same_byte_inode_replacement(tmp_path):
    namespace = runpy.run_path(str(LIBEXEC / "ctr-slice7g-cleanupd"))
    path = tmp_path / "trusted.py"
    path.write_bytes(b"trusted code\n")
    path.chmod(0o444)
    info = path.stat()
    expected = {
        "device": info.st_dev, "inode": info.st_ino, "link_count": 1,
        "mode": 0o444, "owner_gid": info.st_gid, "owner_uid": info.st_uid,
        "path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "size": info.st_size, "type": "regular",
    }
    replacement = tmp_path / "replacement"
    replacement.write_bytes(path.read_bytes())
    replacement.chmod(0o444)
    os.replace(replacement, path)
    with pytest.raises(RuntimeError, match="final pathname identity"):
        namespace["_final_barrier"](str(path), expected, True)


def test_service_private_runtime_directories_and_socket_access_are_independent():
    authority = _unit("ctr-slice7g-authority.service.in")
    cleanup = _unit("ctr-slice7g-cleanup-authority.service.in")
    observer = _unit("ctr-slice7g-observer-supervisor.service.in")
    assert "RuntimeDirectory=ctr-mppi/slice-7g-authority" in authority
    assert "RuntimeDirectory=ctr-mppi/slice-7g-cleanup-authority" in cleanup
    assert "RuntimeDirectory=ctr-mppi/slice-7g-observer-supervisor ctr-slice7g-observer" in observer
    assert "RuntimeDirectoryMode=0755" in authority
    assert "RuntimeDirectoryMode=0755" in cleanup
    assert "RuntimeDirectoryMode=0755" in observer
    assert "Wants=ctr-slice7g-cleanup-authority.service" in observer
    assert "Requires=ctr-slice7g-cleanup-authority.service" not in observer
    assert "/run/ctr-mppi/slice-7g-cleanup-authority" not in observer
    assert "/run/ctr-mppi/slice-7g-observer-supervisor" not in cleanup


def test_root_service_isolation_blocks_manager_bus_and_cleanup_forking():
    cleanup = _unit("ctr-slice7g-cleanup-authority.service.in")
    observer = _unit("ctr-slice7g-observer-supervisor.service.in")
    assert "SystemCallFilter=~fork vfork clone clone3" in cleanup
    assert "CapabilityBoundingSet=CAP_CHOWN CAP_DAC_READ_SEARCH" in cleanup
    assert "AmbientCapabilities=\n" in cleanup
    assert "CapabilityBoundingSet=CAP_CHOWN CAP_DAC_READ_SEARCH CAP_SETUID CAP_SETGID CAP_KILL" in observer
    assert "AmbientCapabilities=\n" in observer
    assert "InaccessiblePaths=/run/systemd/private /run/dbus/system_bus_socket" in observer
    assert "CAP_DAC_OVERRIDE" not in cleanup + observer
    for wrapper in ("ctr-slice7g-cleanupd", "ctr-slice7g-observerd"):
        source = (LIBEXEC / wrapper).read_text()
        assert "def _authority_record(" in source
        assert "authority_parent=(" in source
        assert 'stat.S_IMODE(parent.st_mode) != 0o700' in source
