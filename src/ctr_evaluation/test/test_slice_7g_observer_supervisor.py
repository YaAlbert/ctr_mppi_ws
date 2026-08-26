from __future__ import annotations

import os
from pathlib import Path
from types import MappingProxyType

import pytest

from ctr_evaluation import slice_7g_observer_supervisor as supervisor


def _make_synthetic_root(path: Path):
    path.mkdir(mode=0o755)
    (path / "cgroup.procs").write_text("")
    (path / "cgroup.kill").write_text("")


def test_leaf_grammar_is_exact_and_caller_path_is_rejected(tmp_path):
    _make_synthetic_root(tmp_path / "cg")
    containment = supervisor.CgroupV2Containment._for_test(str(tmp_path / "cg"))
    try:
        leaf, identity = containment.create_leaf("observer-00000000000000000001-" + "a" * 32)
        assert leaf.endswith("observer-00000000000000000001-" + "a" * 32)
        assert len(identity) == 64
        with pytest.raises(supervisor.Slice7GObserverSupervisorError):
            containment.create_leaf("../escape")
    finally:
        containment.close()


def test_synthetic_cgroup_members_ignore_pgid_and_sid(tmp_path):
    _make_synthetic_root(tmp_path / "cg")
    containment = supervisor.CgroupV2Containment._for_test(str(tmp_path / "cg"))
    leaf, _ = containment.create_leaf("observer-00000000000000000001-" + "b" * 32)
    try:
        (Path(leaf) / "cgroup.procs").write_text("101\n202\n")
        assert containment.members(leaf) == (101, 202)
    finally:
        (Path(leaf) / "cgroup.procs").write_text("")
        containment.remove_leaf(leaf)
        containment.close()


def test_cgroup_identity_replacement_is_rejected(tmp_path):
    _make_synthetic_root(tmp_path / "cg")
    containment = supervisor.CgroupV2Containment._for_test(str(tmp_path / "cg"))
    leaf, identity = containment.create_leaf("observer-00000000000000000001-" + "c" * 32)
    old_leaf = leaf + ".retained"
    os.rename(leaf, old_leaf)
    Path(leaf).mkdir()
    try:
        with pytest.raises(supervisor.Slice7GObserverSupervisorError) as caught:
            containment.reconcile(leaf, identity)
        assert caught.value.code == "cgroup_leaf_replaced"
    finally:
        Path(leaf).rmdir()
        Path(old_leaf).rmdir()
        containment.close()


def test_fixed_observer_contract_has_no_shell_or_daemon():
    assert supervisor.OBSERVER_EXECUTABLE == "/opt/ros/humble/bin/ros2"
    assert supervisor.OBSERVER_ARGV == ("node", "list", "--no-daemon")
    source = Path(supervisor.__file__).read_text()
    assert "shell=True" not in source
    assert "ros2 daemon" not in " ".join(supervisor.OBSERVER_ARGV)


def test_observer_limits_are_exact():
    assert supervisor.OBSERVER_TIMEOUT_SECONDS == 10.0
    assert supervisor.SIGINT_GRACE_SECONDS == 1.0
    assert supervisor.SIGTERM_GRACE_SECONDS == 1.0
    assert supervisor.CLEANUP_CEILING_SECONDS == 5.0
    assert supervisor.STABLE_EMPTY_SAMPLES == 2
    assert supervisor.STABLE_EMPTY_SPAN_SECONDS == 0.5
    assert supervisor.MAX_OUTPUT_BYTES == 1_048_576


def test_blocked_stub_is_not_released_implicitly(monkeypatch):
    read_fd, write_fd = os.pipe()
    stdout_fd = os.memfd_create("slice7g-test-out", os.MFD_CLOEXEC | os.MFD_ALLOW_SEALING)
    stderr_fd = os.memfd_create("slice7g-test-err", os.MFD_CLOEXEC | os.MFD_ALLOW_SEALING)
    process = supervisor._ForkedObserver(999999, write_fd, stdout_fd, stderr_fd)
    try:
        assert process._released is False
        with pytest.raises(ProcessLookupError):
            os.kill(process.pid, 0)
    finally:
        process._waited = True
        process.close()
        os.close(read_fd)


@pytest.mark.parametrize("escape_kind", ["setsid", "setpgid", "double_fork"])
def test_containment_contract_is_cgroup_not_session_or_pgid(escape_kind):
    source = Path(supervisor.__file__).read_text()
    assert "cgroup.procs" in source
    assert "cgroup.kill" in source
    assert "self.containment.members" in source
    assert escape_kind not in {"authority", "caller"}


def test_numeric_pid_alone_is_not_signal_authority(monkeypatch):
    monkeypatch.setattr(supervisor, "_proc_identity", lambda pid: (222, pid, pid))
    monkeypatch.setattr(supervisor, "_proc_cgroup", lambda pid: "/wrong")
    assert supervisor._pid_matches(123, 222, "/sys/fs/cgroup/right") is False


def test_cleanup_enumerates_leaf_members_independently_of_session_and_pgid(
    tmp_path, monkeypatch,
):
    _make_synthetic_root(tmp_path / "cg")
    containment = supervisor.CgroupV2Containment._for_test(str(tmp_path / "cg"))
    leaf, leaf_identity = containment.create_leaf(
        "observer-00000000000000000001-" + "d" * 32
    )
    try:
        (Path(leaf) / "cgroup.procs").write_text("101\n202\n")
        monkeypatch.setattr(supervisor, "_proc_identity", lambda pid: (pid + 1000, 999, 888))
        monkeypatch.setattr(
            supervisor, "_proc_cgroup",
            lambda pid: leaf.removeprefix("/sys/fs/cgroup"),
        )
        instance = object.__new__(supervisor.ObserverSupervisor)
        instance.containment = containment
        members = instance._authenticated_members(leaf)
        assert members == ((101, 1101), (202, 1202))
        assert {pid for pid, _ in members} == {101, 202}
        assert all(pgid != pid and sid != pid for pid, (_, pgid, sid) in (
            (101, supervisor._proc_identity(101)),
            (202, supervisor._proc_identity(202)),
        ))
        containment.reconcile(leaf, leaf_identity)
    finally:
        (Path(leaf) / "cgroup.procs").write_text("")
        containment.remove_leaf(leaf)
        containment.close()


def test_production_supervisor_requires_root(monkeypatch):
    monkeypatch.setattr(os, "geteuid", lambda: 1000)
    with pytest.raises(supervisor.Slice7GObserverSupervisorError) as caught:
        supervisor.ObserverSupervisor._production()
    assert caught.value.code == "supervisor_principal"


def test_observer_service_rejects_public_arbitrary_fields():
    source = Path(supervisor.__file__).read_text()
    assert "os.execve(OBSERVER_EXECUTABLE" in source
    assert "request.data[\"executable\"]" not in source
    assert "request.data[\"argv\"]" not in source
    assert "request.data[\"cgroup\"]" not in source


def _postexec_subject(tmp_path, monkeypatch):
    proc = tmp_path / "retained-proc"
    proc.mkdir()
    procfd = os.open(proc, os.O_RDONLY | os.O_DIRECTORY)
    info = os.fstat(procfd)
    environment = {
        "PATH": "/usr/bin:/opt/ros/humble/bin",
        "PYTHONPATH": "/fixed/python",
        "AMENT_PREFIX_PATH": "/fixed:/opt/ros/humble",
        "CMAKE_PREFIX_PATH": "/fixed:/opt/ros/humble",
        "LD_LIBRARY_PATH": "/fixed/lib:/opt/ros/humble/lib",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", "ROS_HOME": "/fixed/ros",
        "HOME": "/fixed/home", "XDG_CACHE_HOME": "/fixed/cache",
        "ROS_DISTRO": "humble", "ROS_LOG_DIR": "/fixed/log",
        "ROS_LOCALHOST_ONLY": "1", "MPLCONFIGDIR": "/fixed/mpl",
        "PYTHONDONTWRITEBYTECODE": "1", "PYTHONNOUSERSITE": "1",
        "ROS_DOMAIN_ID": "100",
    }
    observed = supervisor.PostExecObservation(
        77, 4321, 4321, "/usr/bin/python3.10",
        ("/usr/bin/python3.10", supervisor.OBSERVER_EXECUTABLE, *supervisor.OBSERVER_ARGV),
        MappingProxyType(environment),
        (105, 105, 105, 105, 106, 106, 106, 106, ()),
        "/synthetic/leaf", "/fixed",
        (info.st_dev, info.st_ino, info.st_mode & 0o170000),
    )

    class Process:
        def exited_without_reaping(self):
            return False

    process = Process()
    process.pid = 4321
    process.pidfd = 99
    process.procfd = procfd

    instance = object.__new__(supervisor.ObserverSupervisor)
    instance.environment = {key: value for key, value in environment.items() if key != "ROS_DOMAIN_ID"}
    instance.working_directory = "/fixed"
    instance.observer_uid = 105
    instance.observer_gid = 106
    instance.executable_identity = "a" * 64
    instance.interpreter_path = "/usr/bin/python3.10"
    instance.interpreter_identity = "b" * 64
    instance.environment_identity = "f" * 64
    ticks = [0.0]

    def clock():
        ticks[0] += 0.4
        return ticks[0]

    instance.clock = clock
    instance.postexec_provider = lambda pid: observed
    provenance = supervisor.ProcessProvenance(
        4321, 77, 4321, 4321, "/synthetic/leaf", "c" * 64, "d" * 64,
        "a" * 64, "e" * 64, "f" * 64,
    )
    return instance, process, provenance, observed, procfd


@pytest.mark.parametrize(
    "defect",
    [
        "executable", "argv", "environment", "credentials", "session",
        "cgroup", "pid_reuse",
    ],
)
def test_postexec_observer_identity_mismatch_fails_closed(tmp_path, monkeypatch, defect):
    instance, process, provenance, observed, procfd = _postexec_subject(tmp_path, monkeypatch)
    values = dict(observed.__dict__)
    if defect == "executable":
        values["executable"] = "/bin/sh"
    elif defect == "argv":
        values["argv"] = ("/usr/bin/python3.10", supervisor.OBSERVER_EXECUTABLE, "daemon", "start")
    elif defect == "environment":
        values["environment"] = MappingProxyType({**dict(observed.environment), "PATH": "/tmp"})
    elif defect == "credentials":
        values["credentials"] = (999, 999, 999, 999, 106, 106, 106, 106, ())
    elif defect == "session":
        values["session_id"] = 999
    elif defect == "cgroup":
        values["cgroup"] = "/outside"
    else:
        values["proc_identity"] = (observed.proc_identity[0], observed.proc_identity[1] + 1, observed.proc_identity[2])
    instance.postexec_provider = lambda pid: supervisor.PostExecObservation(**values)
    try:
        with pytest.raises(supervisor.Slice7GObserverSupervisorError):
            instance._authenticate_postexec(
                process, provenance, "/sys/fs/cgroup/synthetic/leaf", 100,
            )
    finally:
        os.close(procfd)


def test_postexec_early_exit_before_authentication_fails_closed(tmp_path, monkeypatch):
    instance, process, provenance, observed, procfd = _postexec_subject(tmp_path, monkeypatch)
    process.exited_without_reaping = lambda: True
    try:
        with pytest.raises(supervisor.Slice7GObserverSupervisorError) as caught:
            instance._authenticate_postexec(
                process, provenance, "/sys/fs/cgroup/synthetic/leaf", 100,
            )
        assert caught.value.code == "observer_postexec_early_exit"
    finally:
        os.close(procfd)


def test_postexec_observer_identity_positive_binds_dedicated_session(tmp_path, monkeypatch):
    instance, process, provenance, observed, procfd = _postexec_subject(tmp_path, monkeypatch)
    try:
        identity = instance._authenticate_postexec(
            process, provenance, "/sys/fs/cgroup/synthetic/leaf", 100,
        )
        assert set(identity) == {
            "executable_identity", "interpreter_identity", "argv_identity",
            "environment_identity", "postexec_identity",
        }
        assert all(len(value) == 64 for value in identity.values())
    finally:
        os.close(procfd)
