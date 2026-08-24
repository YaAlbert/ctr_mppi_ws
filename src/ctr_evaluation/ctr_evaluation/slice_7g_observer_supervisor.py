"""Root-owned exclusive cgroup-v2 observer supervisor for Slice 7G.

The production command is fixed in source and in the privileged-service
manifest.  Requests select only a governed observation transaction; they can
never select an executable, arguments, environment, signal, PID, or cgroup.
"""

from __future__ import annotations

from dataclasses import dataclass
import ctypes
import errno
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import resource
import secrets
import select
import signal
import socket
import stat
import time
from types import MappingProxyType
from typing import Any, Callable, Protocol

from .slice_7g_cleanup_authority import (
    CleanupAuthorityLedger,
    CleanupAuthorityRPCClient,
    CleanupLedgerObservation,
    Slice7GCleanupAuthorityError,
)
from .slice_7g_privileged_protocol import (
    OBSERVER_ACCOUNT,
    OBSERVER_ARGV,
    OBSERVER_CONTAINMENT_RECEIPT_SCHEMA,
    OBSERVER_EXECUTABLE,
    OBSERVER_LEAF_PATTERN,
    OBSERVER_SUPERVISOR_CGROUP,
    OBSERVER_SUPERVISOR_EXECUTABLE,
    OBSERVER_SUPERVISOR_SOCKET,
    PRIVILEGED_RECEIPT_SCHEMA,
    PRIVILEGED_REQUEST_SCHEMA,
    PrivilegedRecord,
    ReplayWindow,
    Slice7GPrivilegedProtocolError,
    authenticate_sealed_output,
    make_sealed_memfd,
    observe_peer,
    peer_credentials,
    receive_packet,
    reconcile_peer,
    record_identity,
    send_packet,
    validate_record,
    verify_response_binding,
)


OBSERVER_TIMEOUT_SECONDS = 10.0
SIGINT_GRACE_SECONDS = 1.0
SIGTERM_GRACE_SECONDS = 1.0
CLEANUP_CEILING_SECONDS = 5.0
STABLE_EMPTY_SAMPLES = 2
STABLE_EMPTY_SPAN_SECONDS = 0.5
MAX_OUTPUT_BYTES = 1_048_576
_FACTORY_TOKEN = object()


class Slice7GObserverSupervisorError(RuntimeError):
    """Stable observer supervisor error."""

    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(f"{code}:{message}")


@dataclass(frozen=True)
class ProcessProvenance:
    pid: int
    start_time_ticks: int
    process_group_id: int
    session_id: int
    cgroup: str
    pidfd_identity: str
    procfd_identity: str
    executable_identity: str
    argv_identity: str
    environment_identity: str


@dataclass(frozen=True)
class PostExecObservation:
    start_time_ticks: int
    process_group_id: int
    session_id: int
    executable: str
    argv: tuple[str, ...]
    environment: MappingProxyType
    credentials: tuple[int, int, int, int, int, int, int, int, tuple[int, ...]]
    cgroup: str
    working_directory: str
    proc_identity: tuple[int, int, int]


@dataclass(frozen=True)
class ObserverResult:
    receipt: PrivilegedRecord
    stdout_fd: int
    stderr_fd: int

    def close(self) -> None:
        os.close(self.stdout_fd)
        os.close(self.stderr_fd)


class ContainmentBackend(Protocol):
    root_identity: str

    def create_leaf(self, name: str) -> tuple[str, str]: ...
    def place(self, leaf: str, pid: int) -> None: ...
    def members(self, leaf: str) -> tuple[int, ...]: ...
    def kill_all(self, leaf: str) -> None: ...
    def remove_leaf(self, leaf: str) -> None: ...
    def reconcile(self, leaf: str, identity: str) -> None: ...


class CleanupTransitionAuthority(Protocol):
    def query(self, **bindings: Any) -> CleanupLedgerObservation: ...
    def begin_unbound(self, **bindings: Any) -> CleanupLedgerObservation: ...
    def bind(
        self, prior: CleanupLedgerObservation, *, containment_identity: str,
        process_identity: str, **bindings: Any,
    ) -> CleanupLedgerObservation: ...
    def terminate(
        self, prior: CleanupLedgerObservation, *, state: str,
        disposition_identity: str,
        recovery_authorization_identity: str | None = None,
        **bindings: Any,
    ) -> CleanupLedgerObservation: ...


class CgroupV2Containment:
    """Descriptor-confined cgroup-v2 leaf authority owned by the supervisor."""

    def __init__(
        self,
        root_path: str = "/sys/fs/cgroup" + OBSERVER_SUPERVISOR_CGROUP,
        *,
        _test: bool = False,
    ) -> None:
        expected = "/sys/fs/cgroup" + OBSERVER_SUPERVISOR_CGROUP
        if not _test and root_path != expected:
            _fail("cgroup_root_override", "production observer cgroup root is fixed")
        if not _test and os.geteuid() != 0:
            _fail("cgroup_principal", "production containment requires root supervisor")
        self.path = _absolute(root_path)
        self._test = _test
        self._fd = _open_directory_path(self.path)
        info = os.fstat(self._fd)
        if not stat.S_ISDIR(info.st_mode):
            self.close()
            _fail("cgroup_root", "containment root is not a directory")
        self._stat = _directory_identity(info)
        if not _test:
            for required in ("cgroup.controllers", "cgroup.procs", "cgroup.subtree_control", "cgroup.kill"):
                _authenticate_control(self._fd, required, require_writable=required in {"cgroup.procs", "cgroup.subtree_control", "cgroup.kill"})
        self.root_identity = _identity(
            b"ctr-slice-7g-observer-cgroup-root-physical-canonical-1\0",
            {"device": info.st_dev, "inode": info.st_ino, "path": self.path},
        )

    @classmethod
    def _for_test(cls, root_path: str) -> "CgroupV2Containment":
        return cls(root_path, _test=True)

    def close(self) -> None:
        if getattr(self, "_fd", None) is not None:
            os.close(self._fd)
            self._fd = None

    def create_leaf(self, name: str) -> tuple[str, str]:
        if not _leaf_name(name):
            _fail("cgroup_leaf", "observer leaf name differs")
        self._barrier()
        os.mkdir(name, mode=0o755, dir_fd=self._fd)
        leaf_fd = _open_directory_at(self._fd, name)
        try:
            info = os.fstat(leaf_fd)
            # Kernel cgroup pseudo-file inventory depends on enabled
            # controllers.  Authenticate only fixed controls at their point
            # of use, and require the new boundary to begin empty.
            if self.members(self.path + "/" + name):
                _fail("cgroup_leaf_inventory", "new observer leaf is not empty")
            identity = _identity(
                b"ctr-slice-7g-observer-leaf-physical-canonical-1\0",
                {"device": info.st_dev, "inode": info.st_ino, "name": name, "root_identity": self.root_identity},
            )
            return self.path + "/" + name, identity
        finally:
            os.close(leaf_fd)

    def place(self, leaf: str, pid: int) -> None:
        leaf_fd = self._open_leaf(leaf)
        try:
            _write_control(leaf_fd, "cgroup.procs", f"{pid}\n".encode(), test=self._test)
        finally:
            os.close(leaf_fd)

    def members(self, leaf: str) -> tuple[int, ...]:
        leaf_fd = self._open_leaf(leaf)
        try:
            try:
                raw = _read_control(leaf_fd, "cgroup.procs", test=self._test)
            except FileNotFoundError:
                if self._test:
                    return ()
                raise
        finally:
            os.close(leaf_fd)
        try:
            values = tuple(sorted(int(line) for line in raw.decode("ascii", "strict").splitlines() if line))
        except (UnicodeError, ValueError) as exc:
            raise Slice7GObserverSupervisorError("cgroup_members", type(exc).__name__) from exc
        if any(value <= 0 for value in values) or len(values) != len(set(values)):
            _fail("cgroup_members", "cgroup membership is malformed")
        return values

    def kill_all(self, leaf: str) -> None:
        leaf_fd = self._open_leaf(leaf)
        try:
            if self._test:
                for pid in self.members(leaf):
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                _write_control(leaf_fd, "cgroup.procs", b"", test=True)
            else:
                _write_control(leaf_fd, "cgroup.kill", b"1\n", test=False)
        finally:
            os.close(leaf_fd)

    def remove_leaf(self, leaf: str) -> None:
        name = PurePosixPath(leaf).name
        if self.members(leaf):
            _fail("cgroup_residual", "observer leaf still has members")
        if self._test:
            leaf_path = Path(leaf)
            for child in tuple(leaf_path.iterdir()):
                child.unlink()
        os.rmdir(name, dir_fd=self._fd)

    def reconcile(self, leaf: str, identity: str) -> None:
        leaf_fd = self._open_leaf(leaf)
        try:
            info = os.fstat(leaf_fd)
            observed = _identity(
                b"ctr-slice-7g-observer-leaf-physical-canonical-1\0",
                {"device": info.st_dev, "inode": info.st_ino, "name": PurePosixPath(leaf).name, "root_identity": self.root_identity},
            )
            if observed != identity:
                _fail("cgroup_leaf_replaced", "observer leaf identity changed")
        finally:
            os.close(leaf_fd)

    def _open_leaf(self, leaf: str) -> int:
        expected_parent = self.path + "/"
        if type(leaf) is not str or not leaf.startswith(expected_parent):
            _fail("cgroup_leaf", "observer leaf escapes delegated root")
        name = leaf[len(expected_parent):]
        if "/" in name or not _leaf_name(name):
            _fail("cgroup_leaf", "observer leaf name differs")
        return _open_directory_at(self._fd, name)

    def _barrier(self) -> None:
        if self._fd is None or _directory_identity(os.fstat(self._fd)) != self._stat:
            _fail("cgroup_root_replaced", "cgroup root descriptor changed")
        reopened = _open_directory_path(self.path)
        try:
            if _directory_identity(os.fstat(reopened)) != self._stat:
                _fail("cgroup_root_replaced", "cgroup root pathname changed")
        finally:
            os.close(reopened)


class ObserverSupervisor:
    """One-observer supervisor with a durable cleanup authority dependency."""

    def __init__(
        self,
        *,
        cleanup_authority: CleanupTransitionAuthority,
        containment: ContainmentBackend,
        runtime_authorization_identity: str,
        installed_runtime_identity: str,
        budget_identity: str,
        observer_contract_identity: str,
        executable_identity: str,
        interpreter_path: str,
        interpreter_identity: str,
        environment_identity: str,
        closed_environment: dict[str, str],
        working_directory: str,
        observer_uid: int | None,
        observer_gid: int | None,
        dds_clear_provider: Callable[[int], tuple[int, ...]] | None = None,
        postexec_provider: Callable[[int], PostExecObservation] | None = None,
        service_generation_identity: str | None = None,
        clock: Callable[[], float] = time.monotonic,
        utc_now: Callable[[], str] | None = None,
        _factory_token: object | None = None,
    ) -> None:
        if _factory_token is not _FACTORY_TOKEN:
            _fail("supervisor_factory", "supervisor must use production or private test assembly")
        if not all(
            callable(getattr(cleanup_authority, name, None))
            for name in ("query", "begin_unbound", "bind", "terminate")
        ):
            _fail("supervisor_cleanup_authority", "cleanup transition authority differs")
        self.cleanup_authority = cleanup_authority
        self.containment = containment
        self.runtime_authorization_identity = _digest(runtime_authorization_identity)
        self.installed_runtime_identity = _digest(installed_runtime_identity)
        self.budget_identity = _digest(budget_identity)
        self.observer_contract_identity = _digest(observer_contract_identity)
        self.executable_identity = _digest(executable_identity)
        if interpreter_path != "/usr/bin/python3.10":
            _fail("observer_interpreter", "root-trusted observer interpreter path differs")
        self.interpreter_path = interpreter_path
        self.interpreter_identity = _digest(interpreter_identity)
        self.environment_identity = _digest(environment_identity)
        if type(closed_environment) is not dict or any(type(k) is not str or type(v) is not str for k, v in closed_environment.items()):
            _fail("observer_environment", "observer environment must be an exact string map")
        if set(closed_environment) != {
            "PATH", "PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH",
            "LD_LIBRARY_PATH", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_HOME",
            "HOME", "XDG_CACHE_HOME", "ROS_DISTRO", "ROS_LOG_DIR", "ROS_LOCALHOST_ONLY",
            "MPLCONFIGDIR", "PYTHONDONTWRITEBYTECODE", "PYTHONNOUSERSITE",
        }:
            _fail("observer_environment", "observer environment key set differs")
        self.environment = dict(closed_environment)
        self.working_directory = _absolute(working_directory)
        self.observer_uid = observer_uid
        self.observer_gid = observer_gid
        self.dds_clear_provider = dds_clear_provider or _production_dds_ports
        self.postexec_provider = postexec_provider or _observe_postexec_process
        self.service_generation_identity = service_generation_identity or hashlib.sha256(secrets.token_bytes(32)).hexdigest()
        self.clock = clock
        self.utc_now = utc_now or _utc_now
        self._active_token: str | None = None

    @classmethod
    def _for_test(cls, **kwargs: Any) -> "ObserverSupervisor":
        if "ledger" in kwargs:
            ledger = kwargs.pop("ledger")
            if type(ledger) is not CleanupAuthorityLedger:
                _fail("supervisor_ledger", "test cleanup ledger differs")
            kwargs["cleanup_authority"] = _LedgerTransitionAdapter(ledger)
        return cls(_factory_token=_FACTORY_TOKEN, **kwargs)

    @classmethod
    def _production(cls, **kwargs: Any) -> "ObserverSupervisor":
        if os.geteuid() != 0:
            _fail("supervisor_principal", "production observer supervisor requires root")
        return cls(_factory_token=_FACTORY_TOKEN, **kwargs)

    def observe(self, request_value: dict[str, Any]) -> ObserverResult:
        request = validate_record(request_value, expected_schema=PRIVILEGED_REQUEST_SCHEMA)
        if request.data["operation"] != "OBSERVE_START":
            _fail("observer_operation", "observer supervisor accepts only OBSERVE_START here")
        if self._active_token is not None:
            _fail("observer_concurrency", "exactly one observer may be active")
        if (
            request.data["runtime_authorization_identity"] != self.runtime_authorization_identity
            or request.data["installed_runtime_identity"] != self.installed_runtime_identity
            or request.data["budget_identity"] != self.budget_identity
        ):
            _fail("observer_binding", "observer authority binding differs")
        domain = request.data["domain_id"]
        if type(domain) is not int:
            _fail("observer_domain", "observer domain is missing")
        token = request.data["operation_token"] or secrets.token_hex(16)
        self._active_token = token
        active: CleanupLedgerObservation | None = None
        bound: CleanupLedgerObservation | None = None
        leaf: str | None = None
        leaf_identity: str | None = None
        process: _ForkedObserver | None = None
        primary: BaseException | None = None
        terminal: CleanupLedgerObservation | None = None
        stdout_fd: int | None = None
        stderr_fd: int | None = None
        receipt: PrivilegedRecord | None = None
        try:
            clear = self.cleanup_authority.query(
                runtime_authorization_identity=self.runtime_authorization_identity,
                installed_runtime_identity=self.installed_runtime_identity,
                budget_identity=self.budget_identity,
                service_generation_identity=self.service_generation_identity,
                session_binding_identity=request.data["session_binding_identity"],
            )
            if clear.state not in {"CLEARED", "RECOVERED"}:
                _fail("cleanup_blocked", "cleanup authority blocks observer start")
            active = self.cleanup_authority.begin_unbound(
                runtime_authorization_identity=self.runtime_authorization_identity,
                installed_runtime_identity=self.installed_runtime_identity,
                budget_identity=self.budget_identity,
                service_generation_identity=self.service_generation_identity,
                session_binding_identity=request.data["session_binding_identity"],
                phase=request.data["phase"],
                phase_local_ordinal=request.data["phase_local_ordinal"],
                transaction_observer_ordinal=request.data["transaction_observer_ordinal"],
                domain_id=domain,
                observer_contract_identity=self.observer_contract_identity,
                cleanup_head_identity=clear.head.logical_identity,
            )
            leaf_name = f"observer-{int(active.revision.data['revision']):020d}-{token[:32].lower()}"
            if not _leaf_name(leaf_name):
                _fail("observer_token", "operation token cannot form an observer leaf")
            leaf, leaf_identity = self.containment.create_leaf(leaf_name)
            process = self._fork_blocked(domain)
            baseline_ports = self.dds_clear_provider(domain)
            self.containment.place(leaf, process.pid)
            self.containment.reconcile(leaf, leaf_identity)
            provenance = self._capture_provenance(process, leaf)
            containment_identity = _identity(
                b"ctr-slice-7g-observer-containment-canonical-1\0",
                {
                    "leaf_cgroup_identity": leaf_identity,
                    "process_identity": _process_identity(provenance),
                    "root_identity": self.containment.root_identity,
                },
            )
            bound = self.cleanup_authority.bind(
                active, containment_identity=containment_identity,
                process_identity=_process_identity(provenance),
                runtime_authorization_identity=self.runtime_authorization_identity,
                installed_runtime_identity=self.installed_runtime_identity,
                budget_identity=self.budget_identity,
                service_generation_identity=self.service_generation_identity,
                session_binding_identity=request.data["session_binding_identity"],
            )
            process.release()
            postexec = self._authenticate_postexec(process, provenance, leaf, domain)
            started_ns = time.monotonic_ns()
            returncode, terminating_signal = process.wait(OBSERVER_TIMEOUT_SECONDS)
            ended_ns = time.monotonic_ns()
            cleanup_identity, stable_samples, stable_span = self._cleanup_leaf(
                leaf, leaf_identity, process, domain, baseline_ports,
            )
            stdout_fd = process.seal_stdout()
            stderr_fd = process.seal_stderr()
            stdout_size, stdout_sha = _fd_size_digest(stdout_fd)
            stderr_size, stderr_sha = _fd_size_digest(stderr_fd)
            disposition = _identity(
                b"ctr-slice-7g-observer-cleanup-disposition-canonical-1\0",
                {
                    "cleanup_barrier_identity": cleanup_identity,
                    "leaf_cgroup_identity": leaf_identity,
                    "returncode": returncode,
                },
            )
            terminal = self.cleanup_authority.terminate(
                bound, state="CLEARED", disposition_identity=disposition,
                runtime_authorization_identity=self.runtime_authorization_identity,
                installed_runtime_identity=self.installed_runtime_identity,
                budget_identity=self.budget_identity,
                service_generation_identity=self.service_generation_identity,
                session_binding_identity=request.data["session_binding_identity"],
            )
            value = {
                "schema_version": OBSERVER_CONTAINMENT_RECEIPT_SCHEMA,
                "operation_token": token,
                "service_generation_identity": self.service_generation_identity,
                "session_binding_identity": request.data["session_binding_identity"],
                "runtime_authorization_identity": self.runtime_authorization_identity,
                "budget_identity": self.budget_identity,
                "cleanup_active_head_identity": bound.head.logical_identity,
                "cleanup_terminal_head_identity": terminal.head.logical_identity,
                "domain_id": domain,
                "phase": request.data["phase"],
                "phase_local_ordinal": request.data["phase_local_ordinal"],
                "transaction_observer_ordinal": request.data["transaction_observer_ordinal"],
                "leaf_cgroup": leaf,
                "leaf_cgroup_identity": leaf_identity,
                "pid": provenance.pid,
                "process_start_time_ticks": provenance.start_time_ticks,
                "process_group_id": provenance.process_group_id,
                "session_id": provenance.session_id,
                "pidfd_identity": provenance.pidfd_identity,
                "procfd_identity": provenance.procfd_identity,
                "executable_identity": postexec["executable_identity"],
                "interpreter_identity": postexec["interpreter_identity"],
                "argv_identity": postexec["argv_identity"],
                "environment_identity": postexec["environment_identity"],
                "postexec_identity": postexec["postexec_identity"],
                "working_directory_identity": _identity(
                    b"ctr-slice-7g-working-directory-canonical-1\0",
                    {"path": self.working_directory},
                ),
                "started_monotonic_ns": started_ns,
                "ended_monotonic_ns": ended_ns,
                "exit_status": returncode,
                "terminating_signal": terminating_signal,
                "stdout_size": stdout_size,
                "stdout_sha256": stdout_sha,
                "stderr_size": stderr_size,
                "stderr_sha256": stderr_sha,
                "cleanup_barrier_identity": cleanup_identity,
                "stable_empty_samples": stable_samples,
                "stable_empty_span_ns": stable_span,
                "leaf_removed": True,
                "disposition": "CLEARED",
            }
            receipt = validate_record(value, expected_schema=OBSERVER_CONTAINMENT_RECEIPT_SCHEMA)
        except BaseException as exc:
            primary = exc
            cleanup_error: BaseException | None = None
            if leaf is not None and leaf_identity is not None:
                try:
                    self._force_cleanup_leaf(leaf, leaf_identity)
                except BaseException as cleanup_exc:
                    cleanup_error = cleanup_exc
            latest = None
            try:
                latest = self.cleanup_authority.query(
                    runtime_authorization_identity=self.runtime_authorization_identity,
                    installed_runtime_identity=self.installed_runtime_identity,
                    budget_identity=self.budget_identity,
                    service_generation_identity=self.service_generation_identity,
                    session_binding_identity=request.data["session_binding_identity"],
                )
                if latest.state in {"ACTIVE_UNBOUND", "ACTIVE_BOUND"}:
                    quarantine_identity = _identity(
                        b"ctr-slice-7g-observer-quarantine-canonical-1\0",
                        {
                            "primary": type(primary).__name__,
                            "cleanup": None if cleanup_error is None else type(cleanup_error).__name__,
                            "operation_token": token,
                        },
                    )
                    self.cleanup_authority.terminate(
                        latest, state="QUARANTINED", disposition_identity=quarantine_identity,
                        runtime_authorization_identity=self.runtime_authorization_identity,
                        installed_runtime_identity=self.installed_runtime_identity,
                        budget_identity=self.budget_identity,
                        service_generation_identity=self.service_generation_identity,
                        session_binding_identity=request.data["session_binding_identity"],
                    )
            except BaseException as ledger_exc:
                if hasattr(primary, "add_note"):
                    primary.add_note("quarantine transition failed: " + type(ledger_exc).__name__)
            if cleanup_error is not None and hasattr(primary, "add_note"):
                primary.add_note("containment cleanup failed: " + type(cleanup_error).__name__)
            raise primary
        finally:
            self._active_token = None
            if process is not None:
                process.close()
            if primary is not None:
                for descriptor in (stdout_fd, stderr_fd):
                    if descriptor is not None:
                        os.close(descriptor)
        assert receipt is not None and stdout_fd is not None and stderr_fd is not None
        return ObserverResult(receipt, stdout_fd, stderr_fd)

    def _fork_blocked(self, domain: int) -> "_ForkedObserver":
        if self.observer_uid is None or self.observer_gid is None:
            _fail("observer_principal_unprovisioned", "numeric ctr7g-observer UID/GID are absent")
        environment = dict(self.environment)
        environment["ROS_DOMAIN_ID"] = str(domain)
        return _ForkedObserver.create(
            environment=environment, working_directory=self.working_directory,
            observer_uid=self.observer_uid, observer_gid=self.observer_gid,
        )

    def _capture_provenance(self, process: "_ForkedObserver", leaf: str) -> ProcessProvenance:
        pid = process.pid
        procfd = os.open(f"/proc/{pid}", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW)
        pidfd = None
        try:
            start, pgid, sid = _proc_identity(pid)
            if pgid != pid or sid != pid:
                _fail("observer_session", "blocked observer lacks its dedicated process session")
            cgroup = _proc_cgroup(pid)
            if cgroup != leaf.removeprefix("/sys/fs/cgroup"):
                _fail("observer_cgroup", "blocked observer is outside exclusive leaf")
            pidfd_identity = None
            if hasattr(os, "pidfd_open"):
                pidfd = os.pidfd_open(pid, 0)
                info = os.fstat(pidfd)
                pidfd_identity = _identity(
                    b"ctr-slice-7g-pidfd-canonical-1\0",
                    {"device": info.st_dev, "inode": info.st_ino, "pid": pid, "start": start},
                )
            else:
                _fail("pidfd_unavailable", "platform lacks pidfd_open")
            info = os.fstat(procfd)
            procfd_identity = _identity(
                b"ctr-slice-7g-procfd-canonical-1\0",
                {"device": info.st_dev, "inode": info.st_ino, "pid": pid, "start": start},
            )
            provenance = ProcessProvenance(
                pid, start, pgid, sid, cgroup, pidfd_identity, procfd_identity,
                self.executable_identity,
                _identity(
                    b"ctr-slice-7g-observer-argv-canonical-1\0",
                    {"argv": [OBSERVER_EXECUTABLE, *OBSERVER_ARGV]},
                ),
                self.environment_identity,
            )
            process.retain_provenance(procfd, pidfd)
            procfd = -1
            pidfd = None
            return provenance
        finally:
            if pidfd is not None:
                os.close(pidfd)
            if procfd >= 0:
                os.close(procfd)

    def _authenticate_postexec(
        self, process: "_ForkedObserver", provenance: ProcessProvenance,
        leaf: str, domain: int,
    ) -> dict[str, str]:
        """Reconcile the released child after exec and before accepting any result."""
        deadline = self.clock() + 1.0
        expected_argv = (OBSERVER_EXECUTABLE, *OBSERVER_ARGV)
        while self.clock() <= deadline:
            if process.exited_without_reaping():
                _fail("observer_postexec_early_exit", "observer exited before post-exec authentication")
            try:
                observed = self.postexec_provider(process.pid)
                if type(observed) is not PostExecObservation:
                    _fail("observer_postexec_provider", "post-exec provider returned a non-record")
                start = observed.start_time_ticks
                pgid = observed.process_group_id
                sid = observed.session_id
                executable = observed.executable
                argv = observed.argv
                environment = dict(observed.environment)
                credentials = observed.credentials
                cgroup = observed.cgroup
                cwd = observed.working_directory
            except Slice7GObserverSupervisorError:
                raise
            except (OSError, UnicodeError, ValueError, IndexError) as exc:
                if self.clock() < deadline:
                    time.sleep(0.005)
                    continue
                raise Slice7GObserverSupervisorError(
                    "observer_postexec_identity", type(exc).__name__,
                ) from exc
            except Exception as exc:
                raise Slice7GObserverSupervisorError(
                    "observer_postexec_provider", type(exc).__name__,
                ) from exc
            normalized_argv = argv
            if argv and os.path.realpath(argv[0]) == executable:
                normalized_argv = argv[1:]
            if normalized_argv == expected_argv:
                break
            time.sleep(0.005)
        else:
            _fail("observer_postexec_argv", "observer argv never reached the fixed exec contract")
        expected_environment = dict(self.environment)
        expected_environment["ROS_DOMAIN_ID"] = str(domain)
        if (
            start != provenance.start_time_ticks
            or pgid != provenance.pid or sid != provenance.pid
            or os.path.realpath(executable) != os.path.realpath(self.interpreter_path)
            or cgroup != leaf.removeprefix("/sys/fs/cgroup")
            or environment != expected_environment
            or credentials != (
                self.observer_uid, self.observer_uid, self.observer_uid, self.observer_uid,
                self.observer_gid, self.observer_gid, self.observer_gid, self.observer_gid, (),
            )
            or cwd != self.working_directory
        ):
            _fail("observer_postexec_identity", "observer post-exec process identity differs")
        if process.procfd is None or process.pidfd is None:
            _fail("observer_postexec_handle", "observer provenance handles are unavailable")
        retained_info = os.fstat(process.procfd)
        if observed.proc_identity != (
            retained_info.st_dev, retained_info.st_ino,
            stat.S_IFMT(retained_info.st_mode),
        ):
            _fail("observer_postexec_pid_reuse", "observer proc identity was replaced")
        interpreter_identity = _identity(
            b"ctr-slice-7g-observer-interpreter-postexec-canonical-1\0",
            {"path": executable, "trusted_identity": self.interpreter_identity},
        )
        argv_identity = _identity(
            b"ctr-slice-7g-observer-argv-canonical-2\0",
            {"argv": list(expected_argv), "kernel_argv": list(argv)},
        )
        environment_identity = _identity(
            b"ctr-slice-7g-observer-environment-postexec-canonical-1\0",
            {"environment": environment, "trusted_identity": self.environment_identity},
        )
        postexec_identity = _identity(
            b"ctr-slice-7g-observer-postexec-process-canonical-1\0",
            {
                "argv_identity": argv_identity, "cgroup": cgroup,
                "credentials": list(credentials[:-1]) + [list(credentials[-1])],
                "environment_identity": environment_identity,
                "executable": OBSERVER_EXECUTABLE,
                "executable_identity": self.executable_identity,
                "interpreter": executable,
                "interpreter_identity": interpreter_identity,
                "pid": process.pid, "session_id": sid, "start_time_ticks": start,
            },
        )
        return {
            "executable_identity": self.executable_identity,
            "interpreter_identity": interpreter_identity,
            "argv_identity": argv_identity,
            "environment_identity": environment_identity,
            "postexec_identity": postexec_identity,
        }

    def _cleanup_leaf(
        self, leaf: str, leaf_identity: str, process: "_ForkedObserver",
        domain: int, baseline_ports: tuple[int, ...],
    ) -> tuple[str, int, int]:
        deadline = self.clock() + CLEANUP_CEILING_SECONDS
        for sent_signal, grace in (
            (signal.SIGINT, SIGINT_GRACE_SECONDS),
            (signal.SIGTERM, SIGTERM_GRACE_SECONDS),
        ):
            members = self._authenticated_members(leaf)
            if not members:
                break
            for pid, start in members:
                if _pid_matches(pid, start, leaf):
                    os.kill(pid, sent_signal)
            until = min(deadline, self.clock() + grace)
            while self.clock() < until and self.containment.members(leaf):
                time.sleep(0.01)
        if self.containment.members(leaf):
            self.containment.kill_all(leaf)
        first_ns = None
        samples = 0
        while self.clock() <= deadline:
            self.containment.reconcile(leaf, leaf_identity)
            ports = self.dds_clear_provider(domain)
            if not self.containment.members(leaf) and ports == baseline_ports:
                now_ns = time.monotonic_ns()
                if first_ns is None:
                    first_ns = now_ns
                    samples = 1
                elif now_ns - first_ns >= int(STABLE_EMPTY_SPAN_SECONDS * 1e9):
                    samples += 1
                    self.containment.remove_leaf(leaf)
                    identity = _identity(
                        b"ctr-slice-7g-containment-empty-barrier-canonical-1\0",
                        {
                            "dds_baseline": list(baseline_ports),
                            "leaf_identity": leaf_identity,
                            "samples": samples,
                            "span_ns": now_ns - first_ns,
                        },
                    )
                    return identity, samples, now_ns - first_ns
            else:
                first_ns = None
                samples = 0
            time.sleep(0.01)
        _fail("containment_cleanup_uncertain", "observer leaf did not reach stable empty state")

    def _force_cleanup_leaf(self, leaf: str, leaf_identity: str) -> None:
        self.containment.reconcile(leaf, leaf_identity)
        self.containment.kill_all(leaf)
        deadline = self.clock() + CLEANUP_CEILING_SECONDS
        while self.clock() <= deadline:
            if not self.containment.members(leaf):
                self.containment.remove_leaf(leaf)
                return
            time.sleep(0.01)
        _fail("containment_cleanup_uncertain", "forced containment cleanup is uncertain")

    def _authenticated_members(self, leaf: str) -> tuple[tuple[int, int], ...]:
        result = []
        for pid in self.containment.members(leaf):
            start, _, _ = _proc_identity(pid)
            if _proc_cgroup(pid) != leaf.removeprefix("/sys/fs/cgroup"):
                _fail("containment_escape", "cgroup member provenance differs")
            result.append((pid, start))
        return tuple(result)


class _ForkedObserver:
    """Trusted blocked fork stub.  Untrusted bytes cannot execute before release."""

    def __init__(self, pid: int, release_fd: int, stdout_fd: int, stderr_fd: int) -> None:
        self.pid = pid
        self._release_fd = release_fd
        self._stdout_fd = stdout_fd
        self._stderr_fd = stderr_fd
        self._released = False
        self._waited = False
        self.procfd: int | None = None
        self.pidfd: int | None = None

    @classmethod
    def create(
        cls, *, environment: dict[str, str], working_directory: str,
        observer_uid: int, observer_gid: int,
    ) -> "_ForkedObserver":
        read_fd, write_fd = os.pipe2(os.O_CLOEXEC)
        ready_read_fd, ready_write_fd = os.pipe2(os.O_CLOEXEC)
        stdout_fd = os.memfd_create("slice7g-observer-stdout", os.MFD_CLOEXEC | os.MFD_ALLOW_SEALING)
        stderr_fd = os.memfd_create("slice7g-observer-stderr", os.MFD_CLOEXEC | os.MFD_ALLOW_SEALING)
        pid = os.fork()
        if pid == 0:  # pragma: no cover - exercised only by controlled local helper tests
            try:
                os.close(write_fd)
                os.close(ready_read_fd)
                os.setsid()
                if os.write(ready_write_fd, b"S") != 1:
                    os._exit(126)
                os.close(ready_write_fd)
                if os.read(read_fd, 1) != b"R":
                    os._exit(126)
                os.close(read_fd)
                os.setgroups([])
                os.setgid(observer_gid)
                os.setuid(observer_uid)
                _set_no_new_privileges()
                resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
                resource.setrlimit(resource.RLIMIT_NOFILE, (64, 64))
                os.chdir(working_directory)
                os.dup2(stdout_fd, 1)
                os.dup2(stderr_fd, 2)
                os.closerange(3, 64)
                os.execve(OBSERVER_EXECUTABLE, (OBSERVER_EXECUTABLE, *OBSERVER_ARGV), environment)
            except BaseException:
                os._exit(127)
        os.close(read_fd)
        os.close(ready_write_fd)
        try:
            ready, _, _ = select.select((ready_read_fd,), (), (), 1.0)
            if not ready or os.read(ready_read_fd, 1) != b"S":
                _fail("observer_session", "observer did not establish its dedicated session")
        except BaseException:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                os.waitpid(pid, 0)
            except ChildProcessError:
                pass
            for descriptor in (write_fd, stdout_fd, stderr_fd):
                os.close(descriptor)
            raise
        finally:
            os.close(ready_read_fd)
        return cls(pid, write_fd, stdout_fd, stderr_fd)

    def retain_provenance(self, procfd: int, pidfd: int) -> None:
        if self.procfd is not None or self.pidfd is not None:
            _fail("observer_provenance", "observer provenance handles were already retained")
        self.procfd = procfd
        self.pidfd = pidfd

    def exited_without_reaping(self) -> bool:
        if self.pidfd is None:
            _fail("observer_provenance", "observer pidfd is unavailable")
        ready, _, _ = select.select((self.pidfd,), (), (), 0)
        return bool(ready)

    def release(self) -> None:
        if self._released:
            _fail("observer_release", "observer start barrier cannot be released twice")
        if os.write(self._release_fd, b"R") != 1:
            _fail("observer_release", "observer release token write failed")
        os.close(self._release_fd)
        self._release_fd = -1
        self._released = True

    def wait(self, timeout: float) -> tuple[int, int | None]:
        deadline = time.monotonic() + timeout
        while time.monotonic() <= deadline:
            waited, status = os.waitpid(self.pid, os.WNOHANG)
            if waited == self.pid:
                self._waited = True
                if os.WIFEXITED(status):
                    return os.WEXITSTATUS(status), None
                if os.WIFSIGNALED(status):
                    return 128 + os.WTERMSIG(status), os.WTERMSIG(status)
                _fail("observer_exit", "observer exit disposition differs")
            time.sleep(0.01)
        return 124, signal.SIGKILL

    def seal_stdout(self) -> int:
        return self._seal(self._stdout_fd)

    def seal_stderr(self) -> int:
        return self._seal(self._stderr_fd)

    @staticmethod
    def _seal(descriptor: int) -> int:
        import fcntl
        os.fsync(descriptor)
        seals = fcntl.F_SEAL_WRITE | fcntl.F_SEAL_GROW | fcntl.F_SEAL_SHRINK | fcntl.F_SEAL_SEAL
        fcntl.fcntl(descriptor, fcntl.F_ADD_SEALS, seals)
        os.lseek(descriptor, 0, os.SEEK_SET)
        return os.dup(descriptor)

    def close(self) -> None:
        if self._release_fd >= 0:
            os.close(self._release_fd)
            self._release_fd = -1
        for name in ("_stdout_fd", "_stderr_fd"):
            descriptor = getattr(self, name)
            if descriptor >= 0:
                os.close(descriptor)
                setattr(self, name, -1)
        for name in ("procfd", "pidfd"):
            descriptor = getattr(self, name)
            if descriptor is not None:
                os.close(descriptor)
                setattr(self, name, None)
        if not self._waited:
            try:
                os.kill(self.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                os.waitpid(self.pid, 0)
            except ChildProcessError:
                pass
            self._waited = True


class _LedgerTransitionAdapter:
    """Private synthetic adapter; production uses CleanupAuthorityRPCClient."""

    def __init__(self, ledger: CleanupAuthorityLedger) -> None:
        self._ledger = ledger

    def query(self, **_: Any) -> CleanupLedgerObservation:
        return self._ledger.reconstruct()

    def begin_unbound(self, **bindings: Any) -> CleanupLedgerObservation:
        bindings.pop("installed_runtime_identity", None)
        bindings.pop("cleanup_head_identity", None)
        return self._ledger.begin_unbound(timestamp=_utc_now(), **bindings)

    def bind(
        self, prior: CleanupLedgerObservation, *, containment_identity: str,
        process_identity: str, **_: Any,
    ) -> CleanupLedgerObservation:
        return self._ledger.bind(
            prior, containment_identity=containment_identity,
            process_identity=process_identity, timestamp=_utc_now(),
        )

    def terminate(
        self, prior: CleanupLedgerObservation, *, state: str,
        disposition_identity: str,
        recovery_authorization_identity: str | None = None,
        **_: Any,
    ) -> CleanupLedgerObservation:
        return self._ledger.terminate(
            prior, state=state, disposition_identity=disposition_identity,
            recovery_authorization_identity=recovery_authorization_identity,
            timestamp=_utc_now(),
        )


class ObserverSupervisorService:
    """Closed root service dispatcher for the single fixed observer class."""

    def __init__(
        self, supervisor: ObserverSupervisor, *, authority_uid: int,
        service_generation_identity: str | None = None,
    ) -> None:
        if type(supervisor) is not ObserverSupervisor:
            _fail("observer_service", "observer service assembly differs")
        if type(authority_uid) is not int or authority_uid < 0:
            _fail("observer_service", "authority UID differs")
        self.supervisor = supervisor
        self.authority_uid = authority_uid
        self.service_generation_identity = (
            service_generation_identity or supervisor.service_generation_identity
        )
        self._replay = ReplayWindow(self.service_generation_identity)

    def handle(
        self, request_value: dict[str, Any], peer: Any,
    ) -> tuple[MappingProxyType, tuple[int, ...]]:
        request = validate_record(request_value, expected_schema=PRIVILEGED_REQUEST_SCHEMA)
        reconcile_peer(peer)
        if request.data["service_generation_identity"] not in (
            None, self.service_generation_identity,
        ):
            _fail("observer_generation", "observer request service generation differs")
        self._replay.claim(request)
        if (
            peer.credentials.uid != self.authority_uid
            or peer.cgroup != "/system.slice/ctr-slice7g-authority.service"
            or "/usr/libexec/ctr-mppi/ctr-slice7g-authorityd" not in peer.argv
            or not peer.executable.startswith("/usr/bin/python3")
        ):
            _fail("observer_peer", "observer request peer is not the fixed authority daemon")
        if request.data["operation"] == "OBSERVE_START":
            result = self.supervisor.observe(dict(request.data))
            receipt = result.receipt
            response = _service_receipt(
                request, self.service_generation_identity, result="CLEANED",
                cleanup_head_identity=receipt.data["cleanup_terminal_head_identity"],
                containment=receipt, descriptor_count=2,
            )
            return response, (result.stdout_fd, result.stderr_fd)
        if request.data["operation"] in {
            "OBSERVE_STATUS", "OBSERVE_CANCEL_AND_CLEANUP",
        }:
            _fail("observer_operation_state", "no matching active observer operation")
        _fail("observer_operation", "operation is unavailable on observer service")


class ObserverSupervisorRPCClient:
    """Authority-daemon client for the single fixed observer operation."""

    def __init__(
        self, *, socket_path: str = OBSERVER_SUPERVISOR_SOCKET,
        _test: bool = False,
    ) -> None:
        if not _test and socket_path != OBSERVER_SUPERVISOR_SOCKET:
            _fail("observer_socket_override", "production observer socket is fixed")
        self._test = _test
        self._channel = socket.socket(
            socket.AF_UNIX, socket.SOCK_SEQPACKET | socket.SOCK_CLOEXEC,
        )
        self._channel.settimeout(20.0)
        try:
            self._channel.connect(socket_path)
        except OSError as exc:
            self.close()
            raise Slice7GObserverSupervisorError(
                "observer_connection", type(exc).__name__,
            ) from exc
        peer = observe_peer(peer_credentials(self._channel))
        if not _test and not (
            peer.credentials.uid == 0
            and peer.cgroup == OBSERVER_SUPERVISOR_CGROUP
            and OBSERVER_SUPERVISOR_EXECUTABLE in peer.argv
            and peer.executable.startswith("/usr/bin/python3")
        ):
            self.close()
            _fail("observer_service_peer", "observer service peer identity differs")
        self._connection_nonce = secrets.token_hex(16)
        self._sequence = 0
        self._peer = peer
        self._service_generation_identity: str | None = None

    @classmethod
    def _for_test(cls, socket_path: str) -> "ObserverSupervisorRPCClient":
        return cls(socket_path=socket_path, _test=True)

    def close(self) -> None:
        channel = getattr(self, "_channel", None)
        if channel is not None:
            channel.close()
            self._channel = None

    def observe(
        self, *, runtime_authorization_identity: str,
        installed_runtime_identity: str, budget_identity: str,
        cleanup_head_identity: str, session_binding_identity: str,
        domain_id: int, phase: str, phase_local_ordinal: int,
        transaction_observer_ordinal: int,
        privileged_service_generation_identity: str | None = None,
    ) -> dict[str, Any]:
        if self._channel is None:
            _fail("observer_connection", "observer service connection is closed")
        request = {
            "schema_version": PRIVILEGED_REQUEST_SCHEMA,
            "operation": "OBSERVE_START",
            "sequence": self._sequence,
            "connection_nonce": self._connection_nonce,
            "request_nonce": secrets.token_hex(16),
            "operation_token": secrets.token_hex(16),
            "service_generation_identity": self._service_generation_identity,
            "runtime_authorization_identity": runtime_authorization_identity,
            "installed_runtime_identity": installed_runtime_identity,
            "budget_identity": budget_identity,
            "cleanup_head_identity": cleanup_head_identity,
            "session_binding_identity": session_binding_identity,
            "domain_id": domain_id,
            "phase": phase,
            "phase_local_ordinal": phase_local_ordinal,
            "transaction_observer_ordinal": transaction_observer_ordinal,
            "transition": None,
            "observer_contract_identity": None,
            "containment_identity": None,
            "process_identity": None,
            "disposition_identity": None,
            "recovery_authorization_identity": None,
        }
        send_packet(self._channel, request, expected_schema=PRIVILEGED_REQUEST_SCHEMA)
        response, descriptors = receive_packet(
            self._channel, expected_schema=PRIVILEGED_RECEIPT_SCHEMA,
            expected_descriptors=None,
        )
        self._sequence += 1
        try:
            expected_descriptors = 0 if response.data["result"] == "ERROR" else 2
            generation = verify_response_binding(
                validate_record(request, expected_schema=PRIVILEGED_REQUEST_SCHEMA), response,
                expected_service_generation_identity=self._service_generation_identity,
                expected_descriptor_count=expected_descriptors, descriptors=descriptors,
                peer=self._peer,
            )
            self._service_generation_identity = generation
            if response.data["result"] == "ERROR":
                _fail(response.data["error_code"], "observer service rejected observation")
            nested = response.data["containment_receipt"]
            if nested is None:
                _fail("observer_response", "observer response omits containment receipt")
            receipt = validate_record(
                dict(nested), expected_schema=OBSERVER_CONTAINMENT_RECEIPT_SCHEMA,
            )
            if (
                receipt.logical_identity != response.data["containment_receipt_identity"]
                or receipt.logical_identity != response.data["payload_identity"]
                or response.data["output_descriptor_count"] != 2
                or len(descriptors) != 2
            ):
                _fail("observer_response", "observer containment/output binding differs")
            for request_field, receipt_field in (
                ("operation_token", "operation_token"),
                ("runtime_authorization_identity", "runtime_authorization_identity"),
                ("budget_identity", "budget_identity"),
                ("session_binding_identity", "session_binding_identity"),
                ("domain_id", "domain_id"),
                ("phase", "phase"),
                ("phase_local_ordinal", "phase_local_ordinal"),
                ("transaction_observer_ordinal", "transaction_observer_ordinal"),
            ):
                if receipt.data[receipt_field] != request[request_field]:
                    _fail("observer_response_binding", f"observer response {receipt_field} differs")
            if receipt.data["service_generation_identity"] != generation:
                _fail("observer_response_binding", "observer response service generation differs")
            stdout = authenticate_sealed_output(
                descriptors[0], expected_size=receipt.data["stdout_size"],
                expected_sha256=receipt.data["stdout_sha256"],
            )
            stderr = authenticate_sealed_output(
                descriptors[1], expected_size=receipt.data["stderr_size"],
                expected_sha256=receipt.data["stderr_sha256"],
            )
            return {
                "containment_receipt": receipt,
                "stdout": stdout,
                "stderr": stderr,
                "pid": receipt.data["pid"],
                "pgid": receipt.data["process_group_id"],
                "start_time_ticks": receipt.data["process_start_time_ticks"],
                "started_monotonic_ns": receipt.data["started_monotonic_ns"],
                "ended_monotonic_ns": receipt.data["ended_monotonic_ns"],
                "exit_status": receipt.data["exit_status"],
                "cleanup_barrier_identity": receipt.data["cleanup_barrier_identity"],
                "cleanup_head_identity": receipt.data["cleanup_terminal_head_identity"],
            }
        finally:
            for descriptor in descriptors:
                os.close(descriptor)


def serve_observer_supervisor(
    *, authority_uid: int, authority_gid: int, observer_uid: int,
    observer_gid: int, runtime_authorization_identity: str,
    installed_runtime_identity: str, budget_identity: str,
    observer_contract_identity: str, executable_identity: str,
    interpreter_path: str,
    interpreter_identity: str,
    environment_identity: str, closed_environment: dict[str, str],
    working_directory: str,
) -> None:
    """Serve the fixed supervisor socket without provisioning any resource."""
    cleanup = CleanupAuthorityRPCClient()
    containment = CgroupV2Containment()
    supervisor = ObserverSupervisor._production(
        cleanup_authority=cleanup,
        containment=containment,
        runtime_authorization_identity=runtime_authorization_identity,
        installed_runtime_identity=installed_runtime_identity,
        budget_identity=budget_identity,
        observer_contract_identity=observer_contract_identity,
        executable_identity=executable_identity,
        interpreter_path=interpreter_path,
        interpreter_identity=interpreter_identity,
        environment_identity=environment_identity,
        closed_environment=closed_environment,
        working_directory=working_directory,
        observer_uid=observer_uid,
        observer_gid=observer_gid,
    )
    service = ObserverSupervisorService(supervisor, authority_uid=authority_uid)
    listener = socket.socket(
        socket.AF_UNIX, socket.SOCK_SEQPACKET | socket.SOCK_CLOEXEC,
    )
    try:
        if os.path.lexists(OBSERVER_SUPERVISOR_SOCKET):
            _fail("observer_socket_exists", "observer socket path already exists")
        listener.bind(OBSERVER_SUPERVISOR_SOCKET)
        os.chown(OBSERVER_SUPERVISOR_SOCKET, 0, authority_gid)
        os.chmod(OBSERVER_SUPERVISOR_SOCKET, 0o660)
        listener.listen(8)
        while True:
            channel, _ = listener.accept()
            try:
                _serve_observer_connection(channel, service)
            except (Slice7GObserverSupervisorError, Slice7GPrivilegedProtocolError):
                continue
    finally:
        listener.close()
        containment.close()
        cleanup.close()


def _serve_observer_connection(
    channel: socket.socket, service: ObserverSupervisorService,
) -> None:
    peer = observe_peer(peer_credentials(channel))
    expected_sequence = 0
    connection_nonce: str | None = None
    try:
        for _ in range(128):
            request, descriptors = receive_packet(
                channel, expected_schema=PRIVILEGED_REQUEST_SCHEMA,
                expected_descriptors=0,
            )
            if descriptors or request.data["sequence"] != expected_sequence:
                _fail("observer_protocol_sequence", "observer request sequence differs")
            if connection_nonce is None:
                connection_nonce = request.data["connection_nonce"]
            elif request.data["connection_nonce"] != connection_nonce:
                _fail("observer_protocol_nonce", "observer connection nonce changed")
            expected_sequence += 1
            try:
                response, outputs = service.handle(dict(request.data), peer)
            except (Slice7GObserverSupervisorError, Slice7GPrivilegedProtocolError) as exc:
                response = _service_receipt(
                    request, service.service_generation_identity,
                    result="ERROR", error_code=exc.code,
                )
                outputs = ()
            try:
                send_packet(
                    channel, dict(response), expected_schema=PRIVILEGED_RECEIPT_SCHEMA,
                    descriptors=outputs,
                )
            finally:
                for descriptor in outputs:
                    os.close(descriptor)
    finally:
        channel.close()


def _service_receipt(
    request: PrivilegedRecord, generation: str, *, result: str,
    error_code: str | None = None,
    cleanup_head_identity: str | None = None,
    containment: PrivilegedRecord | None = None,
    descriptor_count: int = 0,
) -> MappingProxyType:
    value = {
        "schema_version": PRIVILEGED_RECEIPT_SCHEMA,
        "operation": request.data["operation"],
        "sequence": request.data["sequence"],
        "connection_nonce": request.data["connection_nonce"],
        "request_nonce": request.data["request_nonce"],
        "operation_token": request.data["operation_token"],
        "service_generation_identity": generation,
        "result": result,
        "error_code": error_code,
        "cleanup_head_identity": cleanup_head_identity,
        "containment_receipt_identity": (
            None if containment is None else containment.logical_identity
        ),
        "output_descriptor_count": descriptor_count,
        "payload_identity": None if containment is None else containment.logical_identity,
        "cleanup_revision": None,
        "cleanup_anchor": None,
        "cleanup_head": None,
        "containment_receipt": None if containment is None else dict(containment.data),
    }
    return validate_record(value, expected_schema=PRIVILEGED_RECEIPT_SCHEMA).data


def _proc_identity(pid: int) -> tuple[int, int, int]:
    raw = _read_path(f"/proc/{pid}/stat", 65_536).decode("ascii", "strict")
    close = raw.rfind(")")
    fields = raw[close + 2:].split()
    if close < 0 or len(fields) < 20:
        _fail("process_identity", "process stat is malformed")
    return int(fields[19]), int(fields[2]), int(fields[3])


def _observe_postexec_process(pid: int) -> PostExecObservation:
    """Module-owned post-exec observation; no serialized request can replace it."""
    start, pgid, sid = _proc_identity(pid)
    executable = os.readlink(f"/proc/{pid}/exe")
    argv = tuple(
        item.decode("utf-8", "strict")
        for item in _read_path(f"/proc/{pid}/cmdline", 65_536).split(b"\0")
        if item
    )
    environment: dict[str, str] = {}
    for item in _read_path(f"/proc/{pid}/environ", 262_144).split(b"\0"):
        if not item:
            continue
        key, separator, value = item.partition(b"=")
        if not separator:
            _fail("observer_postexec_environment", "observer environment entry differs")
        name = key.decode("utf-8", "strict")
        if name in environment:
            _fail("observer_postexec_environment", "observer environment contains a duplicate")
        environment[name] = value.decode("utf-8", "strict")
    credentials = _status_credentials(
        _read_path(f"/proc/{pid}/status", 65_536).decode("ascii", "strict"),
    )
    descriptor = os.open(
        f"/proc/{pid}", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW,
    )
    try:
        info = os.fstat(descriptor)
        proc_identity = (info.st_dev, info.st_ino, stat.S_IFMT(info.st_mode))
    finally:
        os.close(descriptor)
    return PostExecObservation(
        start, pgid, sid, executable, argv, MappingProxyType(environment),
        credentials, _proc_cgroup(pid), os.readlink(f"/proc/{pid}/cwd"),
        proc_identity,
    )


def _status_credentials(
    raw: str,
) -> tuple[int, int, int, int, int, int, int, int, tuple[int, ...]]:
    values: dict[str, tuple[int, ...]] = {}
    for line in raw.splitlines():
        name, separator, payload = line.partition(":")
        if separator and name in {"Uid", "Gid", "Groups"}:
            try:
                values[name] = tuple(int(item) for item in payload.split())
            except ValueError as exc:
                raise Slice7GObserverSupervisorError(
                    "observer_postexec_credentials", type(exc).__name__,
                ) from exc
    if len(values.get("Uid", ())) != 4 or len(values.get("Gid", ())) != 4:
        _fail("observer_postexec_credentials", "observer credentials are malformed")
    return (*values["Uid"], *values["Gid"], values.get("Groups", ()))


def _proc_cgroup(pid: int) -> str:
    lines = _read_path(f"/proc/{pid}/cgroup", 65_536).decode("utf-8", "strict").splitlines()
    if len(lines) != 1 or not lines[0].startswith("0::/"):
        _fail("process_cgroup", "process cgroup is malformed")
    return lines[0][3:]


def _production_dds_ports(domain: int) -> tuple[int, ...]:
    """Observe the kernel UDP tables without creating a network endpoint."""
    base = 7_400 + 250 * domain
    expected = {
        base, base + 1,
        *(base + 10 + offset for offset in range(0, 241, 2)),
        *(base + 11 + offset for offset in range(0, 241, 2)),
    }
    observed: set[int] = set()
    for path in ("/proc/net/udp", "/proc/net/udp6"):
        raw = _read_path(path, 8 * 1024 * 1024).decode("ascii", "strict")
        for line in raw.splitlines()[1:]:
            fields = line.split()
            if len(fields) < 2 or ":" not in fields[1]:
                _fail("dds_residual_provider", "UDP inventory is malformed")
            try:
                port = int(fields[1].rsplit(":", 1)[1], 16)
            except ValueError as exc:
                raise Slice7GObserverSupervisorError(
                    "dds_residual_provider", type(exc).__name__,
                ) from exc
            if port in expected:
                observed.add(port)
    return tuple(sorted(observed))


def _pid_matches(pid: int, start: int, leaf: str) -> bool:
    try:
        current, _, _ = _proc_identity(pid)
        return current == start and _proc_cgroup(pid) == leaf.removeprefix("/sys/fs/cgroup")
    except (FileNotFoundError, ProcessLookupError):
        return False


def _process_identity(value: ProcessProvenance) -> str:
    return _identity(
        b"ctr-slice-7g-observer-process-canonical-1\0",
        {
            "pid": value.pid,
            "start_time_ticks": value.start_time_ticks,
            "process_group_id": value.process_group_id,
            "session_id": value.session_id,
            "cgroup": value.cgroup,
            "pidfd_identity": value.pidfd_identity,
            "procfd_identity": value.procfd_identity,
            "executable_identity": value.executable_identity,
            "argv_identity": value.argv_identity,
            "environment_identity": value.environment_identity,
        },
    )


def _set_no_new_privileges() -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(38, 1, 0, 0, 0) != 0:  # PR_SET_NO_NEW_PRIVS
        err = ctypes.get_errno()
        raise OSError(err, os.strerror(err))


def _fd_size_digest(descriptor: int) -> tuple[int, str]:
    info = os.fstat(descriptor)
    if not stat.S_ISREG(info.st_mode) or info.st_size > MAX_OUTPUT_BYTES:
        _fail("observer_output", "observer output identity differs")
    os.lseek(descriptor, 0, os.SEEK_SET)
    digest = hashlib.sha256()
    remaining = info.st_size
    while remaining:
        chunk = os.read(descriptor, min(65_536, remaining))
        if not chunk:
            _fail("observer_output", "observer output ended early")
        digest.update(chunk)
        remaining -= len(chunk)
    return info.st_size, digest.hexdigest()


def _leaf_name(value: str) -> bool:
    return re.fullmatch(r"observer-[0-9]{20}-[0-9a-f]{32}", value) is not None


def _authenticate_control(directory: int, name: str, *, require_writable: bool) -> None:
    descriptor = os.open(name, os.O_RDWR if require_writable else os.O_RDONLY, dir_fd=directory)
    os.close(descriptor)


def _write_control(directory: int, name: str, payload: bytes, *, test: bool) -> None:
    flags = os.O_WRONLY | os.O_CLOEXEC
    if test:
        flags |= os.O_CREAT | os.O_TRUNC
    descriptor = os.open(name, flags, 0o600, dir_fd=directory)
    try:
        if payload and os.write(descriptor, payload) != len(payload):
            _fail("cgroup_write", "cgroup control write was partial")
        if not payload:
            os.ftruncate(descriptor, 0)
    finally:
        os.close(descriptor)


def _read_control(directory: int, name: str, *, test: bool) -> bytes:
    descriptor = os.open(name, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=directory)
    try:
        return os.read(descriptor, 1_048_577)
    finally:
        os.close(descriptor)


def _open_directory_path(path: str) -> int:
    current = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        for part in PurePosixPath(path).parts[1:]:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=current)
            os.close(current)
            current = child
        return current
    except BaseException:
        os.close(current)
        raise


def _open_directory_at(parent: int, name: str) -> int:
    return os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=parent)


def _directory_identity(info: os.stat_result) -> tuple[int, ...]:
    return (info.st_dev, info.st_ino, stat.S_IFMT(info.st_mode), stat.S_IMODE(info.st_mode), info.st_uid, info.st_gid)


def _read_path(path: str, maximum: int) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        result = os.read(descriptor, maximum + 1)
        if len(result) > maximum:
            _fail("process_record", "process record exceeds maximum")
        return result
    finally:
        os.close(descriptor)


def _identity(domain: bytes, value: dict[str, Any]) -> str:
    return hashlib.sha256(domain + json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _digest(value: Any) -> str:
    if type(value) is not str or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        _fail("digest", "expected lowercase SHA-256")
    return value


def _absolute(value: Any) -> str:
    if type(value) is not str or not value.startswith("/") or os.path.normpath(value) != value:
        _fail("path", "path must be normalized absolute")
    return value


def _utc_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _fail(code: str, message: str) -> None:
    raise Slice7GObserverSupervisorError(code, message)


__all__ = [
    "CgroupV2Containment", "ObserverResult", "ObserverSupervisor", "ProcessProvenance",
    "Slice7GObserverSupervisorError",
]
