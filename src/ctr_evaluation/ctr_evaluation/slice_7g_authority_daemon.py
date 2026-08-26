"""Local OS-principal authority service and global Slice 7G attempt budget.

The production constructor has no path/provider arguments.  Underscored test
factories are the sole way to use temporary authority roots.  Importing this
module performs no I/O and starts no service.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from datetime import datetime, timezone
import ctypes
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import selectors
import secrets
import signal
import socket
import stat
import struct
import subprocess
import time
from types import MappingProxyType
from typing import Any, Callable
import unicodedata

from .slice_7g_authority_protocol import (
    AUTHORITY_BOOTSTRAP_SCHEMA,
    BUILD_TEST_APPROVAL_SCHEMA,
    AUTHORITY_RECEIPT_SCHEMA,
    AUTHORITY_REQUEST_SCHEMA,
    AUTHORITY_REVOCATION_SCHEMA,
    AUTHORITY_SOCKET_PATH,
    AUTHORITY_STATE_ROOT,
    ENVIRONMENT_MANIFEST_SCHEMA,
    GLOBAL_ATTEMPT_BUDGET_SCHEMA,
    INSTALLED_RUNTIME_MANIFEST_SCHEMA,
    MAX_FRAME_BYTES,
    MAX_PRECOMMIT_OBSERVERS,
    MAX_SESSION_REQUESTS,
    OBSERVATION_SESSION_LIFETIME_SECONDS,
    OBSERVATION_SESSION_SCHEMA,
    PROCESS_MANIFEST_SCHEMA,
    PREPARE_TOKEN_LIFETIME_SECONDS,
    ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA,
    FOUR_SOURCE_OBSERVATION_SCHEMA,
    GLOBAL_LEASE_OBSERVATION_SCHEMA,
    RUNTIME_AUTHORIZATION_SCHEMA,
    OBSERVER_CLEANUP_GUARD_SCHEMA,
    OBSERVER_CLEANUP_RECOVERY_SCHEMA,
    Slice7GAuthorityProtocolError,
    Slice7GAuthorityRecord,
    Slice7GPeerCredentials,
    Slice7GPeerProcess,
    authority_record_identity,
    authenticate_file_identity,
    canonical_authority_record_bytes,
    load_production_bootstrap,
    observe_peer_process,
    peer_credentials,
    receive_authority_frame,
    reconcile_peer_process,
    send_authority_frame,
    validate_authority_record,
)
from .slice_7g_installed_runtime import authenticate_installed_runtime
from .slice_7g_cleanup_authority import (
    CleanupAuthorityRPCClient,
    CleanupLedgerObservation,
    Slice7GCleanupAuthorityError,
)
from .slice_7g_observer_supervisor import (
    ObserverSupervisorRPCClient,
    Slice7GObserverSupervisorError,
)
from .slice_7g_privileged_protocol import (
    AUTHORITY_BOOTSTRAP_V2_SCHEMA,
    AUTHORITY_BOOTSTRAP_V3_SCHEMA,
    FOUR_SOURCE_OBSERVATION_V4_SCHEMA,
    GLOBAL_ATTEMPT_BUDGET_V4_SCHEMA,
    INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA,
    OBSERVATION_SESSION_V3_SCHEMA,
    OBSERVER_CONTAINMENT_RECEIPT_SCHEMA,
    PROCESS_MANIFEST_V2_SCHEMA,
    PRIVILEGED_SERVICE_MANIFEST_SCHEMA,
    RUNTIME_AUTHORIZATION_V3_SCHEMA,
    ROS_GRAPH_RECEIPT_V3_SCHEMA,
    RUNTIME_AUTHORITY_RECEIPT_V4_SCHEMA,
    RUNTIME_AUTHORITY_REQUEST_V4_SCHEMA,
    GLOBAL_LEASE_OBSERVATION_V2_SCHEMA,
    PrivilegedRecord,
    Slice7GPrivilegedProtocolError,
    canonical_bytes as canonical_privileged_bytes,
    record_identity as privileged_record_identity,
    validate_record as validate_privileged_record,
)


_PRODUCTION_FACTORY_TOKEN = object()


BUDGET_DIRECTORY_NAME = "global-budget"
BUDGET_LOCK_NAME = "budget.lock"
BUDGET_REVISION_PREFIX = "revision-"
BUDGET_REVISION_WIDTH = 20
GLOBAL_LEASE_REGISTRY_NAME = ".ctr_slice_7g_domain_leases"
GLOBAL_LEASE_LOCK_NAME = "registry.lock"
CLEANUP_GUARD_DIRECTORY_NAME = "observer-cleanup-guard"
CLEANUP_GUARD_LOCK_NAME = "guard.lock"
CLEANUP_GUARD_REVISION_PREFIX = "revision-"
CLEANUP_GUARD_REVISION_WIDTH = 20
CLEANUP_RECOVERY_AUTHORIZATION_NAME = "recovery-authorization.json"
REVOCATION_PENDING_NAME = "revocation/pending"
REVOCATION_PROCESSED_NAME = "revocation/processed"
RECEIPT_DIRECTORY_NAME = "receipts"
CAMPAIGN_UNIT = "ctr-slice7g-campaign.service"
SYSTEMCTL_PATH = "/usr/bin/systemctl"
OUTPUT_PARENT = "/home/ankid/ctr_mppi_evidence/slice_7g"
CAMPAIGN_CELLS = tuple(
    f"{scenario}.seed_{seed:010d}"
    for scenario in ("centerline", "lateral_offset", "near_safety_boundary")
    for seed in (11, 22, 33, 44, 55)
)


class Slice7GAuthorityDaemonError(RuntimeError):
    def __init__(self, code: str, message: str, *, path: str = "$") -> None:
        self.code = code
        self.path = path
        super().__init__(f"{code}:{path}:{message}")


@dataclass(frozen=True)
class PreparedCampaign:
    token: str
    connection_identity: str
    peer: Slice7GPeerProcess
    authorization_identity: str
    campaign_id: str
    campaign_identity: str
    campaign_template_identity: str
    observation_session_identity: str
    four_source_observation_identity: str
    precommit_receipt_identities: tuple[str, ...]
    precommit_observer_count: int
    selected_domain: int
    lease_identity: str
    expires_monotonic: float
    cleanup_head_identity: str | None = None
    containment_receipt_identity: str | None = None


@dataclass(frozen=True)
class ObservationSession:
    nonce: str
    identity: str
    connection_identity: str
    peer: Slice7GPeerProcess
    authorization_identity: str
    installed_runtime_identity: str
    process_manifest_identity: str
    environment_manifest_identity: str
    created_monotonic_ns: int
    deadline_monotonic_ns: int
    candidate_domains: tuple[int, ...]
    receipt_identities: tuple[str, ...]
    receipt_records: tuple[Slice7GAuthorityRecord, ...]
    four_source_records: tuple[Slice7GAuthorityRecord, ...]
    selected_domain: int | None
    lease_identity: str | None
    four_source_observation_identity: str | None
    finalized: bool
    privileged_service_manifest_identity: str | None = None
    cleanup_head_identity: str | None = None
    containment_receipt_identity: str | None = None


@dataclass(frozen=True)
class DaemonObservationEvidence:
    """Raw result from the daemon-owned provider seam.

    This type is deliberately not serializable by the public protocol.  The
    production daemon creates it from its installed-runtime-bound providers;
    tests can supply it only through ``RuntimeAuthorityStateMachine._for_test``.
    """

    active_process_identity: str
    active_process_clear: bool
    dds_port_identity: str
    dds_port_clear: bool
    global_lease_identity: str
    global_lease_registry_identity: str
    global_lease_revision_identity: str
    global_lease_state: str
    global_lease_clear: bool
    peer_process_identity: str
    observation_interval_identity: str
    graph_provider_identity: str
    executable: str
    executable_identity: str
    interpreter: str
    interpreter_identity: str
    module_origin_identities: tuple[str, ...]
    argv: tuple[str, ...]
    environment_identity: str
    working_directory: str
    cgroup: str
    pid: int
    process_group_id: int
    process_start_time_ticks: int
    started_monotonic_ns: int
    ended_monotonic_ns: int
    exit_status: int | None
    terminating_signal: int | None
    stdout: bytes
    stderr: bytes
    nodes: tuple[str, ...]
    cleanup_barrier_identity: str
    unexpected_descendants: int
    ros_daemon_started: bool
    observed_monotonic_ns: int
    cleanup_head_identity: str | None = None
    containment_receipt_identity: str | None = None


@dataclass(frozen=True)
class ProvisionalAllocation:
    campaign_id: str
    campaign_identity: str
    domain_id: int
    output_root_path: str
    output_root_identity: str
    cleanup: Callable[[], None]
    final_barrier: Callable[[], None]
    close: Callable[[], None]


@dataclass(frozen=True)
class BudgetObservation:
    revision: int
    record: Slice7GAuthorityRecord
    path: str
    device: int
    inode: int


@dataclass(frozen=True)
class GlobalLeaseObservation:
    record: Slice7GAuthorityRecord | PrivilegedRecord

    @property
    def clear(self) -> bool:
        return bool(self.record.data["clear"])


class GlobalLeaseStateObserver:
    """Read-only, descriptor-confined observer for the established lease ledger."""

    _RESERVATION_SCHEMA = "ctr-slice-7g-domain-reservation-1"
    _RESERVATION_DOMAIN = b"ctr-slice-7g-domain-reservation-canonical-1\0"
    _BINDING_SCHEMA = "ctr-slice-7g-domain-committed-binding-1"
    _BINDING_DOMAIN = b"ctr-slice-7g-domain-committed-binding-canonical-1\0"
    _RELEASE_SCHEMA = "ctr-slice-7g-domain-release-1"
    _RELEASE_DOMAIN = b"ctr-slice-7g-domain-release-canonical-1\0"
    _HISTORY_NAME = re.compile(
        r"^(reservation|binding|release)\.([0-9a-f]{64})\.json$"
    )

    def __init__(
        self,
        registry_path: str = OUTPUT_PARENT + "/" + GLOBAL_LEASE_REGISTRY_NAME,
        *,
        expected_owner_uid: int | None = None,
        _test: bool = False,
    ) -> None:
        expected = OUTPUT_PARENT + "/" + GLOBAL_LEASE_REGISTRY_NAME
        if not _test and registry_path != expected:
            _fail("global_lease_root_override", "production lease registry path is fixed")
        self._test = _test
        self._path = _absolute_path(registry_path)
        self._owner_uid = os.geteuid() if expected_owner_uid is None else expected_owner_uid
        self._registry_fd = _open_directory_path(self._path)
        info = os.fstat(self._registry_fd)
        if (
            not stat.S_ISDIR(info.st_mode)
            or info.st_uid != self._owner_uid
            or info.st_nlink < 2
            or stat.S_IMODE(info.st_mode) != 0o700
        ):
            os.close(self._registry_fd)
            _fail("global_lease_registry_identity", "global lease registry identity differs")
        self._registry_identity = (
            info.st_dev, info.st_ino, stat.S_IMODE(info.st_mode), info.st_uid, info.st_gid,
        )
        self._lock_fd = os.open(
            GLOBAL_LEASE_LOCK_NAME,
            os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW,
            dir_fd=self._registry_fd,
        )
        lock = os.fstat(self._lock_fd)
        by_name = os.stat(
            GLOBAL_LEASE_LOCK_NAME, dir_fd=self._registry_fd, follow_symlinks=False,
        )
        if (
            not stat.S_ISREG(lock.st_mode)
            or lock.st_uid != self._owner_uid
            or lock.st_nlink != 1
            or stat.S_IMODE(lock.st_mode) != 0o600
            or _file_identity(lock) != _file_identity(by_name)
        ):
            self.close()
            _fail("global_lease_lock_identity", "global lease registry lock identity differs")
        self._lock_identity = _file_identity(lock)

    @classmethod
    def _for_test(cls, registry_path: str) -> "GlobalLeaseStateObserver":
        return cls(registry_path, expected_owner_uid=os.geteuid(), _test=True)

    @staticmethod
    def _provision_test_registry(registry_path: str) -> None:
        root = Path(_absolute_path(registry_path))
        root.mkdir(mode=0o700)
        lock = root / GLOBAL_LEASE_LOCK_NAME
        descriptor = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_CLOEXEC, 0o600)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        _fsync_directory(str(root))

    def close(self) -> None:
        for name in ("_lock_fd", "_registry_fd"):
            descriptor = getattr(self, name, None)
            if descriptor is not None:
                os.close(descriptor)
                setattr(self, name, None)

    def observe(
        self, domain: int, observed_monotonic_ns: int, *,
        session_binding_identity: str | None = None,
        service_nonce: str | None = None,
        phase: str | None = None,
        phase_local_ordinal: int | None = None,
        transaction_observer_ordinal: int | None = None,
        observation_interval_identity: str | None = None,
    ) -> GlobalLeaseObservation:
        if type(domain) is not int or not 100 <= domain <= 199:
            _fail("global_lease_domain", "global lease domain must be 100 through 199")
        if type(observed_monotonic_ns) is not int or observed_monotonic_ns < 0:
            _fail("global_lease_time", "global lease observation time is malformed")
        if self._test and session_binding_identity is None:
            session_binding_identity = _domain_identity(
                b"ctr-slice-7g-test-lease-session-canonical-1\0",
                {"domain_id": domain},
            )
            service_nonce = "synthetic-test-service"
            phase = "PRECOMMIT"
            phase_local_ordinal = 1
            transaction_observer_ordinal = 1
            observation_interval_identity = _domain_identity(
                b"ctr-slice-7g-test-lease-interval-canonical-1\0",
                {"observed_monotonic_ns": observed_monotonic_ns},
            )
        self._barrier()
        try:
            fcntl.flock(self._lock_fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise Slice7GAuthorityDaemonError(
                "global_lease_busy", "global lease registry lock is busy",
            ) from exc
        try:
            before = self._inventory(domain)
            after = self._inventory(domain)
            if before != after:
                _fail("global_lease_changed", "global lease registry changed during observation")
            retained = self._retain_final_inventory(domain, after)
            try:
                self._final_inventory_barrier(domain, after, retained)
            finally:
                self._close_retained_inventory(retained)
        finally:
            fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
        active = tuple(item["identity"] for item in before["active"])
        committed = tuple(item["identity"] for item in before["bindings"])
        state = before["lease_state"]
        registry_identity = _domain_identity(
            b"ctr-slice-7g-global-lease-registry-physical-canonical-1\0",
            {
                "device": self._registry_identity[0], "inode": self._registry_identity[1],
                "mode": self._registry_identity[2], "path": self._path,
            },
        )
        revision_identity = _domain_identity(
            b"ctr-slice-7g-global-lease-registry-revision-canonical-1\0", before,
        )
        value = {
            "schema_version": GLOBAL_LEASE_OBSERVATION_SCHEMA,
            "registry_identity": registry_identity,
            "registry_revision_identity": revision_identity,
            "domain_id": domain,
            "state": state,
            "active_reservation_identities": list(active),
            "committed_binding_identities": list(committed),
            "stale_invalid_identities": [],
            "clear": state == "CLEAR",
            "observed_monotonic_ns": observed_monotonic_ns,
        }
        if session_binding_identity is not None:
            physical = sorted({
                _domain_identity(
                    b"ctr-slice-7g-lease-record-physical-canonical-1\0",
                    item["_physical"],
                )
                for category in ("active", "bindings", "history")
                for item in before[category]
            })
            owner_bindings = sorted({
                item.get("campaign_identity")
                for category in ("active", "bindings")
                for item in before[category]
                if item.get("campaign_identity") is not None
            })
            output_bindings = sorted({
                item.get("output_root")
                for item in before["bindings"]
                if item.get("output_root") is not None
            })
            physical_observation_identity = _domain_identity(
                b"ctr-slice-7g-global-lease-physical-observation-canonical-2\0",
                {
                    "inventory": before,
                    "record_physical_identities": physical,
                    "registry_identity": registry_identity,
                },
            )
            v2 = {
                "schema_version": GLOBAL_LEASE_OBSERVATION_V2_SCHEMA,
                "registry_identity": registry_identity,
                "registry_revision_identity": revision_identity,
                "physical_observation_identity": physical_observation_identity,
                "record_physical_identities": physical,
                "domain_id": domain,
                "state": state,
                "owner_bindings": owner_bindings,
                "output_root_bindings": output_bindings,
                "active_reservation_identities": list(active),
                "committed_binding_identities": list(committed),
                "stale_invalid_identities": (
                    [] if state not in {"STALE_INVALID", "INDETERMINATE"}
                    else sorted({item["identity"] for item in before["history"]})
                ),
                "clear": state == "CLEAR",
                "session_binding_identity": session_binding_identity,
                "service_nonce": service_nonce,
                "phase": phase,
                "phase_local_ordinal": phase_local_ordinal,
                "transaction_observer_ordinal": transaction_observer_ordinal,
                "observation_interval_identity": observation_interval_identity,
                "observed_monotonic_ns": observed_monotonic_ns,
            }
            return GlobalLeaseObservation(validate_privileged_record(
                v2, expected_schema=GLOBAL_LEASE_OBSERVATION_V2_SCHEMA,
            ))
        return GlobalLeaseObservation(validate_authority_record(
            value, expected_schema=GLOBAL_LEASE_OBSERVATION_SCHEMA,
        ))

    def _barrier(self) -> None:
        if self._registry_fd is None or self._lock_fd is None:
            _fail("global_lease_closed", "global lease observer is closed")
        current = os.fstat(self._registry_fd)
        if (
            current.st_dev, current.st_ino, stat.S_IMODE(current.st_mode),
            current.st_uid, current.st_gid,
        ) != self._registry_identity:
            _fail("global_lease_registry_replaced", "global lease registry descriptor changed")
        if _file_identity(os.fstat(self._lock_fd)) != self._lock_identity:
            _fail("global_lease_lock_replaced", "global lease lock descriptor changed")
        reopened = _open_directory_path(self._path)
        try:
            current = os.fstat(reopened)
            if (
                current.st_dev, current.st_ino, stat.S_IMODE(current.st_mode),
                current.st_uid, current.st_gid,
            ) != self._registry_identity:
                _fail("global_lease_registry_replaced", "global lease registry pathname changed")
        finally:
            os.close(reopened)
        by_name = os.stat(
            GLOBAL_LEASE_LOCK_NAME, dir_fd=self._registry_fd, follow_symlinks=False,
        )
        if _file_identity(by_name) != self._lock_identity:
            _fail("global_lease_lock_replaced", "global lease lock pathname changed")

    def _retain_final_inventory(
        self, domain: int, authenticated: dict[str, Any],
    ) -> tuple[int | None, tuple[tuple[str, int, tuple[int, ...], str], ...]]:
        """Retain final record authority until the last pathname/inode barrier."""
        name = f"domain_{domain:03d}"
        try:
            directory = _open_directory_at(self._registry_fd, name)
        except FileNotFoundError:
            if authenticated["directory"] is not None:
                _fail("global_lease_record_replaced", "lease domain disappeared before final barrier")
            return None, ()
        retained: list[tuple[str, int, tuple[int, ...], str]] = []
        try:
            info = os.fstat(directory)
            expected_directory = authenticated["directory"]
            observed_directory = {
                "device": info.st_dev, "inode": info.st_ino,
                "mode": stat.S_IMODE(info.st_mode), "link_count": info.st_nlink,
            }
            if expected_directory != observed_directory:
                _fail("global_lease_domain_replaced", "lease domain physical identity changed")
            names = tuple(sorted(os.listdir(directory)))
            expected_physical = sorted({
                tuple(sorted(item["_physical"].items()))
                for category in ("active", "bindings", "history")
                for item in authenticated[category]
            })
            observed_physical = []
            for entry in names:
                descriptor = os.open(
                    entry, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW,
                    dir_fd=directory,
                )
                try:
                    record = os.fstat(descriptor)
                    raw = _read_fd(descriptor, MAX_FRAME_BYTES)
                    physical = {
                        "device": record.st_dev, "inode": record.st_ino,
                        "mode": stat.S_IMODE(record.st_mode),
                        "link_count": record.st_nlink, "size": record.st_size,
                        "mtime_ns": record.st_mtime_ns, "ctime_ns": record.st_ctime_ns,
                        "sha256": hashlib.sha256(raw).hexdigest(),
                    }
                    if (
                        not stat.S_ISREG(record.st_mode)
                        or record.st_uid != self._owner_uid
                        or record.st_nlink != 1
                        or stat.S_IMODE(record.st_mode) != 0o444
                    ):
                        _fail("global_lease_record_identity", "retained lease record differs")
                    identity = _file_identity(record)
                    retained.append((entry, descriptor, identity, physical["sha256"]))
                    observed_physical.append(tuple(sorted(physical.items())))
                    descriptor = -1
                finally:
                    if descriptor >= 0:
                        os.close(descriptor)
            if sorted(observed_physical) != expected_physical:
                _fail("global_lease_record_replaced", "lease records changed before retention")
            return directory, tuple(retained)
        except BaseException:
            for _, descriptor, _, _ in retained:
                os.close(descriptor)
            os.close(directory)
            raise

    def _final_inventory_barrier(
        self, domain: int, authenticated: dict[str, Any],
        retained: tuple[int | None, tuple[tuple[str, int, tuple[int, ...], str], ...]],
    ) -> None:
        directory, records = retained
        name = f"domain_{domain:03d}"
        if directory is None:
            try:
                probe = _open_directory_at(self._registry_fd, name)
            except FileNotFoundError:
                probe = None
            if probe is not None:
                os.close(probe)
                _fail("global_lease_record_replaced", "lease domain appeared before final barrier")
            self._barrier()
            return
        reopened = _open_directory_at(self._registry_fd, name)
        try:
            if _directory_identity(os.fstat(reopened)) != _directory_identity(os.fstat(directory)):
                _fail("global_lease_domain_replaced", "lease domain pathname was replaced")
        finally:
            os.close(reopened)
        if tuple(sorted(os.listdir(directory))) != tuple(item[0] for item in records):
            _fail("global_lease_inventory", "lease record membership changed at final barrier")
        for entry, descriptor, identity, expected_sha256 in records:
            current = os.fstat(descriptor)
            if _file_identity(current) != identity:
                _fail("global_lease_record_replaced", "retained lease descriptor changed")
            os.lseek(descriptor, 0, os.SEEK_SET)
            raw = _read_fd(descriptor, MAX_FRAME_BYTES)
            if hashlib.sha256(raw).hexdigest() != expected_sha256:
                _fail("global_lease_record_replaced", "retained lease bytes changed")
            by_name = os.stat(entry, dir_fd=directory, follow_symlinks=False)
            if _file_identity(by_name) != identity:
                _fail("global_lease_record_replaced", "lease pathname inode was substituted")
        self._barrier()

    @staticmethod
    def _close_retained_inventory(
        retained: tuple[int | None, tuple[tuple[str, int, tuple[int, ...], str], ...]],
    ) -> None:
        directory, records = retained
        for _, descriptor, _, _ in records:
            os.close(descriptor)
        if directory is not None:
            os.close(directory)

    def _inventory(self, domain: int) -> dict[str, Any]:
        name = f"domain_{domain:03d}"
        try:
            directory = _open_directory_at(self._registry_fd, name)
        except FileNotFoundError:
            return {
                "domain_id": domain, "directory": None, "active": [],
                "bindings": [], "history": [], "lease_state": "CLEAR",
            }
        try:
            info = os.fstat(directory)
            if info.st_uid != self._owner_uid or stat.S_IMODE(info.st_mode) != 0o700:
                _fail("global_lease_domain_identity", "global lease domain directory differs")
            names = sorted(os.listdir(directory))
            if any(
                item != "active.json" and self._HISTORY_NAME.fullmatch(item) is None
                for item in names
            ):
                _fail("global_lease_inventory", "global lease domain inventory contains an unknown entry")
            records: dict[str, dict[str, Any]] = {}
            for item in names:
                records[item] = self._read_record(directory, item, domain)
            active_records = [records["active.json"]] if "active.json" in records else []
            reservation_history = {
                records[item]["identity"]: records[item]
                for item in names if item.startswith("reservation.")
            }
            bindings_by_reservation: dict[str, list[dict[str, Any]]] = {}
            releases_by_reservation: dict[str, list[dict[str, Any]]] = {}
            for item in names:
                if item.startswith("binding."):
                    record = records[item]
                    bindings_by_reservation.setdefault(
                        record["domain_reservation_identity"], [],
                    ).append(record)
                elif item.startswith("release."):
                    record = records[item]
                    releases_by_reservation.setdefault(
                        record["domain_reservation_identity"], [],
                    ).append(record)
            active_identity = active_records[0]["identity"] if active_records else None
            known_reservations = set(reservation_history)
            if active_identity is not None:
                known_reservations.add(active_identity)
            orphan_binding = set(bindings_by_reservation) - known_reservations
            orphan_release = set(releases_by_reservation) - set(reservation_history)
            multiple_binding = any(len(items) != 1 for items in bindings_by_reservation.values())
            multiple_release = any(len(items) != 1 for items in releases_by_reservation.values())
            unreleased_history = {
                identity for identity in reservation_history
                if identity not in releases_by_reservation
            }
            active_released = active_identity is not None and active_identity in releases_by_reservation
            bindings = (
                [] if active_identity is None
                else bindings_by_reservation.get(active_identity, [])
            )
            if multiple_binding or multiple_release:
                lease_state = "CONFLICTING"
            elif orphan_binding or orphan_release or active_released:
                lease_state = "STALE_INVALID"
            elif unreleased_history:
                lease_state = "INDETERMINATE"
            elif active_identity is None:
                lease_state = "CLEAR"
            elif len(bindings) == 1:
                lease_state = "COMMITTED"
            elif not bindings:
                lease_state = "RESERVED"
            else:
                lease_state = "CONFLICTING"
            histories = [records[item] for item in names if item != "active.json"]
            for item, record in records.items():
                match = self._HISTORY_NAME.fullmatch(item)
                if match is None:
                    continue
                kind, suffix = match.groups()
                expected = (
                    record["identity"] if kind == "reservation"
                    else record["domain_reservation_identity"]
                )
                if suffix != expected:
                    _fail(
                        "global_lease_history_binding",
                        "global lease history filename does not bind its record",
                    )
            return {
                "domain_id": domain,
                "directory": {
                    "device": info.st_dev, "inode": info.st_ino,
                    "mode": stat.S_IMODE(info.st_mode), "link_count": info.st_nlink,
                },
                "active": active_records,
                "bindings": bindings,
                "history": histories,
                "lease_state": lease_state,
            }
        finally:
            os.close(directory)

    def _read_record(self, directory: int, name: str, domain: int) -> dict[str, Any]:
        descriptor = os.open(name, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=directory)
        try:
            before = os.fstat(descriptor)
            by_name = os.stat(name, dir_fd=directory, follow_symlinks=False)
            if (
                not stat.S_ISREG(before.st_mode)
                or before.st_uid != self._owner_uid
                or before.st_nlink != 1
                or stat.S_IMODE(before.st_mode) != 0o444
                or _file_identity(before) != _file_identity(by_name)
                or before.st_size <= 0
                or before.st_size > MAX_FRAME_BYTES
            ):
                _fail("global_lease_record_identity", "global lease record identity differs")
            raw = _read_fd(descriptor, MAX_FRAME_BYTES)
            after = os.fstat(descriptor)
            if _file_identity(before) != _file_identity(after):
                _fail("global_lease_record_changed", "global lease record changed while reading")
        finally:
            os.close(descriptor)
        try:
            value = json.loads(raw.decode("utf-8", "strict"), object_pairs_hook=_reject_duplicate_pairs)
        except (UnicodeError, ValueError, json.JSONDecodeError) as exc:
            raise Slice7GAuthorityDaemonError("global_lease_record_json", type(exc).__name__) from exc
        if type(value) is not dict or raw != json.dumps(
            value, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":"),
        ).encode("utf-8"):
            _fail("global_lease_record_canonical", "global lease record is noncanonical")
        if type(value.get("domain_id")) is not int or value["domain_id"] != domain:
            _fail("global_lease_record_domain", "global lease record domain differs")
        if name == "active.json" or name.startswith("reservation."):
            fields = {"schema_version", "domain_id", "runtime_authorization_identity", "campaign_identity", "reserved_at_utc", "identity"}
            schema, identity_domain = self._RESERVATION_SCHEMA, self._RESERVATION_DOMAIN
        elif name.startswith("binding."):
            fields = {
                "schema_version", "domain_lease_identity", "domain_reservation_identity",
                "final_domain_observation_identity", "runtime_authorization_identity",
                "campaign_identity", "campaign_plan_identity", "attempt_ledger_identity",
                "attempt_ledger_revision", "process_start_event_identity", "domain_id",
                "output_root", "identity",
            }
            schema, identity_domain = self._BINDING_SCHEMA, self._BINDING_DOMAIN
        else:
            fields = {"schema_version", "domain_id", "domain_lease_identity", "domain_reservation_identity", "released_at_utc", "identity"}
            schema, identity_domain = self._RELEASE_SCHEMA, self._RELEASE_DOMAIN
        if set(value) != fields or value.get("schema_version") != schema:
            _fail("global_lease_record_schema", "global lease record schema differs")
        identity = value.get("identity")
        if type(identity) is not str or len(identity) != 64:
            _fail("global_lease_record_identity", "global lease record identity is malformed")
        projection = {key: item for key, item in value.items() if key != "identity"}
        if hashlib.sha256(identity_domain + json.dumps(
            projection, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":"),
        ).encode("utf-8")).hexdigest() != identity:
            _fail("global_lease_record_identity", "global lease record identity differs")
        reopened = os.open(
            name, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=directory,
        )
        try:
            final = os.fstat(reopened)
            final_raw = _read_fd(reopened, MAX_FRAME_BYTES)
            if (
                _file_identity(final) != _file_identity(before)
                or hashlib.sha256(final_raw).digest() != hashlib.sha256(raw).digest()
            ):
                _fail(
                    "global_lease_record_replaced",
                    "global lease record physical identity changed before final barrier",
                )
        finally:
            os.close(reopened)
        value["_physical"] = {
            "device": before.st_dev,
            "inode": before.st_ino,
            "mode": stat.S_IMODE(before.st_mode),
            "link_count": before.st_nlink,
            "size": before.st_size,
            "mtime_ns": before.st_mtime_ns,
            "ctime_ns": before.st_ctime_ns,
            "sha256": hashlib.sha256(raw).hexdigest(),
        }
        return value


@dataclass(frozen=True)
class CleanupGuardObservation:
    revision: int
    record: Slice7GAuthorityRecord


class ObserverCleanupGuardStore:
    """Durable non-consuming observer cleanup/quarantine revision chain."""

    def __init__(self, state_root: str = AUTHORITY_STATE_ROOT, *, _test: bool = False) -> None:
        if not _test and state_root != AUTHORITY_STATE_ROOT:
            _fail("cleanup_guard_root_override", "production cleanup guard root is fixed")
        self._state_root = _absolute_path(state_root)
        self._root_fd = _open_directory_path(self._state_root)
        root = os.fstat(self._root_fd)
        if not _test and (root.st_uid != os.geteuid() or stat.S_IMODE(root.st_mode) != 0o700):
            os.close(self._root_fd)
            _fail("cleanup_guard_root_identity", "cleanup guard authority root differs")
        self._root_identity = _directory_identity(root)
        self._authority_root_identity = _domain_identity(
            b"ctr-slice-7g-cleanup-authority-root-physical-canonical-1\0",
            {
                "device": root.st_dev, "inode": root.st_ino,
                "mode": stat.S_IMODE(root.st_mode), "path": self._state_root,
            },
        )
        self._guard_fd = _open_directory_at(self._root_fd, CLEANUP_GUARD_DIRECTORY_NAME)
        guard = os.fstat(self._guard_fd)
        if (
            guard.st_uid != os.geteuid()
            or stat.S_IMODE(guard.st_mode) != 0o700
            or guard.st_nlink < 2
        ):
            self.close()
            _fail("cleanup_guard_root_identity", "cleanup guard directory identity differs")
        self._guard_identity = _directory_identity(guard)
        self._lock_fd = os.open(
            CLEANUP_GUARD_LOCK_NAME,
            os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW,
            dir_fd=self._guard_fd,
        )
        lock = os.fstat(self._lock_fd)
        by_name = os.stat(CLEANUP_GUARD_LOCK_NAME, dir_fd=self._guard_fd, follow_symlinks=False)
        if (
            not stat.S_ISREG(lock.st_mode) or lock.st_nlink != 1
            or stat.S_IMODE(lock.st_mode) != 0o600
            or _file_identity(lock) != _file_identity(by_name)
        ):
            self.close()
            _fail("cleanup_guard_lock_identity", "cleanup guard lock identity differs")
        self._lock_identity = _file_identity(lock)

    @classmethod
    def _for_test(cls, state_root: str) -> "ObserverCleanupGuardStore":
        return cls(state_root, _test=True)

    @staticmethod
    def _provision_test_root(state_root: str, timestamp: str) -> None:
        root = Path(_absolute_path(state_root))
        guard = root / CLEANUP_GUARD_DIRECTORY_NAME
        guard.mkdir(mode=0o700)
        lock_fd = os.open(
            guard / CLEANUP_GUARD_LOCK_NAME,
            os.O_CREAT | os.O_EXCL | os.O_RDWR | os.O_CLOEXEC,
            0o600,
        )
        os.close(lock_fd)
        initial = _empty_cleanup_guard(timestamp)
        payload = canonical_authority_record_bytes(initial, expected_schema=OBSERVER_CLEANUP_GUARD_SCHEMA)
        descriptor = os.open(
            guard / _cleanup_guard_revision_name(0),
            os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_CLOEXEC,
            0o600,
        )
        try:
            _write_all(descriptor, payload)
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        _fsync_directory(str(guard))

    def close(self) -> None:
        for name in ("_lock_fd", "_guard_fd", "_root_fd"):
            descriptor = getattr(self, name, None)
            if descriptor is not None:
                os.close(descriptor)
                setattr(self, name, None)

    def __del__(self) -> None:
        try:
            self.close()
        except BaseException:
            pass

    def observe(self) -> CleanupGuardObservation:
        self._barrier()
        with _locked(self._lock_fd):
            return self._load_latest_locked()

    def require_clear(self) -> CleanupGuardObservation:
        current = self.observe()
        state = current.record.data["state"]
        if state not in {"CLEARED", "RECOVERED"}:
            code = (
                "observation_cleanup_uncertain"
                if state == "QUARANTINED" else "cleanup_guard_active"
            )
            _fail(code, "durable observer cleanup state blocks new authority work")
        return current

    def begin(
        self, *, authorization_identity: str, budget_identity: str,
        service_generation_identity: str, session_binding_identity: str,
        phase: str, phase_local_ordinal: int, transaction_observer_ordinal: int,
        domain_id: int, executable_identity: str, argv_identity: str,
        environment_identity: str, timestamp: str,
    ) -> CleanupGuardObservation:
        with _locked(self._lock_fd):
            current = self._load_latest_locked()
            if current.record.data["state"] not in {"CLEARED", "RECOVERED"}:
                _fail("cleanup_guard_active", "durable observer cleanup guard is not clear")
            value = {
                "schema_version": OBSERVER_CLEANUP_GUARD_SCHEMA,
                "revision": current.revision + 1,
                "predecessor_identity": current.record.logical_identity,
                "state": "ACTIVE_UNBOUND",
                "authorization_identity": _digest(authorization_identity),
                "budget_identity": _digest(budget_identity),
                "service_generation_identity": _digest(service_generation_identity),
                "session_binding_identity": _digest(session_binding_identity),
                "phase": phase,
                "phase_local_ordinal": phase_local_ordinal,
                "transaction_observer_ordinal": transaction_observer_ordinal,
                "domain_id": domain_id,
                "executable_identity": _digest(executable_identity),
                "argv_identity": _digest(argv_identity),
                "environment_identity": _digest(environment_identity),
                "pid": None, "process_start_time_ticks": None, "process_group_id": None,
                "session_id": None, "cgroup": None, "pidfd_identity": None,
                "disposition_identity": None, "recovery_authorization_identity": None,
                "updated_at_utc": timestamp,
            }
            return self._write_successor_locked(current, value)

    def bind_process(
        self, active_identity: str, *, pid: int, process_start_time_ticks: int,
        process_group_id: int, session_id: int, cgroup: str, pidfd_identity: str,
        timestamp: str,
    ) -> CleanupGuardObservation:
        with _locked(self._lock_fd):
            current = self._load_latest_locked()
            if current.record.logical_identity != active_identity or current.record.data["state"] != "ACTIVE_UNBOUND":
                _fail("cleanup_guard_binding", "cleanup guard no longer names the active observer")
            value = _builtin_authority_value(current.record.data)
            value.update({
                "revision": current.revision + 1,
                "predecessor_identity": current.record.logical_identity,
                "state": "ACTIVE_BOUND", "pid": pid,
                "process_start_time_ticks": process_start_time_ticks,
                "process_group_id": process_group_id, "session_id": session_id,
                "cgroup": cgroup, "pidfd_identity": _digest(pidfd_identity),
                "updated_at_utc": timestamp,
            })
            return self._write_successor_locked(current, value)

    def clear(self, bound_identity: str, disposition_identity: str, timestamp: str) -> CleanupGuardObservation:
        return self._terminal(bound_identity, "CLEARED", disposition_identity, timestamp)

    def quarantine(
        self, expected_identity: str, disposition_identity: str, timestamp: str,
    ) -> CleanupGuardObservation:
        return self._terminal(expected_identity, "QUARANTINED", disposition_identity, timestamp)

    def recover_for_test(
        self, recovery: dict[str, Any], *, process_clear: bool, dds_clear: bool,
        lease_clear: bool, graph_clear: bool, disposition_identity: str, timestamp: str,
        current_service_generation_identity: str,
    ) -> CleanupGuardObservation:
        record = validate_authority_record(
            recovery, expected_schema=OBSERVER_CLEANUP_RECOVERY_SCHEMA,
        )
        with _locked(self._lock_fd):
            current = self._load_latest_locked()
            if current.record.data["state"] != "QUARANTINED":
                _fail("cleanup_recovery_state", "cleanup guard is not quarantined")
            if record.data["quarantine_identity"] != current.record.logical_identity:
                _fail("cleanup_recovery_binding", "cleanup recovery targets another quarantine")
            if record.data["authority_root_identity"] != self._authority_root_identity:
                _fail("cleanup_recovery_binding", "cleanup recovery authority root differs")
            if record.data["runtime_authorization_identity"] != current.record.data["authorization_identity"]:
                _fail("cleanup_recovery_binding", "cleanup recovery authorization differs")
            if record.data["budget_identity"] != current.record.data["budget_identity"]:
                _fail("cleanup_recovery_binding", "cleanup recovery budget differs")
            if record.data["service_generation_identity"] != _digest(
                current_service_generation_identity,
            ):
                _fail("cleanup_recovery_binding", "cleanup recovery service generation differs")
            observed = _parse_utc(timestamp)
            if not (
                _parse_utc(record.data["not_before_utc"])
                <= observed
                < _parse_utc(record.data["not_after_utc"])
            ):
                _fail("cleanup_recovery_expired", "cleanup recovery authority is not current")
            if not all(type(item) is bool and item for item in (process_clear, dds_clear, lease_clear, graph_clear)):
                _fail("cleanup_recovery_residual", "fresh recovery clearance is incomplete")
            value = _builtin_authority_value(current.record.data)
            value.update({
                "revision": current.revision + 1,
                "predecessor_identity": current.record.logical_identity,
                "state": "RECOVERED",
                "disposition_identity": _digest(disposition_identity),
                "recovery_authorization_identity": record.logical_identity,
                "service_generation_identity": record.data["service_generation_identity"],
                "updated_at_utc": timestamp,
            })
            return self._write_successor_locked(current, value)

    def _terminal(
        self, expected_identity: str, state: str, disposition_identity: str, timestamp: str,
    ) -> CleanupGuardObservation:
        with _locked(self._lock_fd):
            current = self._load_latest_locked()
            if current.record.logical_identity != expected_identity or current.record.data["state"] not in {"ACTIVE_UNBOUND", "ACTIVE_BOUND"}:
                _fail("cleanup_guard_binding", "cleanup guard terminal transition differs")
            if state == "CLEARED" and current.record.data["state"] != "ACTIVE_BOUND":
                _fail("cleanup_guard_clear", "an unbound observer cannot be proven clear")
            value = _builtin_authority_value(current.record.data)
            value.update({
                "revision": current.revision + 1,
                "predecessor_identity": current.record.logical_identity,
                "state": state,
                "disposition_identity": _digest(disposition_identity),
                "updated_at_utc": timestamp,
            })
            return self._write_successor_locked(current, value)

    def _load_latest_locked(self) -> CleanupGuardObservation:
        inventory = sorted(os.listdir(self._guard_fd))
        names = sorted(
            item for item in inventory
            if item.startswith(CLEANUP_GUARD_REVISION_PREFIX)
        )
        if set(inventory) != {CLEANUP_GUARD_LOCK_NAME, *names}:
            _fail("cleanup_guard_history", "cleanup guard inventory contains an unknown entry")
        if not names or names != [_cleanup_guard_revision_name(index) for index in range(len(names))]:
            _fail("cleanup_guard_history", "cleanup guard revisions are noncontiguous")
        predecessor: str | None = None
        prior_state: str | None = None
        seen: set[tuple[int, int]] = set()
        latest: CleanupGuardObservation | None = None
        transitions = {
            "CLEARED": {"ACTIVE_UNBOUND"}, "RECOVERED": {"ACTIVE_UNBOUND"},
            "ACTIVE_UNBOUND": {"ACTIVE_BOUND", "QUARANTINED"},
            "ACTIVE_BOUND": {"CLEARED", "QUARANTINED"},
            "QUARANTINED": {"RECOVERED"},
        }
        for index, name in enumerate(names):
            descriptor = os.open(name, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=self._guard_fd)
            try:
                before = os.fstat(descriptor)
                by_name = os.stat(name, dir_fd=self._guard_fd, follow_symlinks=False)
                physical = (before.st_dev, before.st_ino)
                if (
                    not stat.S_ISREG(before.st_mode) or before.st_nlink != 1
                    or stat.S_IMODE(before.st_mode) != 0o600
                    or _file_identity(before) != _file_identity(by_name)
                    or physical in seen
                ):
                    _fail("cleanup_guard_revision_identity", "cleanup guard revision identity differs")
                seen.add(physical)
                raw = _read_fd(descriptor, MAX_FRAME_BYTES)
                if _file_identity(before) != _file_identity(os.fstat(descriptor)):
                    _fail("cleanup_guard_revision_changed", "cleanup guard revision changed")
            finally:
                os.close(descriptor)
            record = validate_authority_record(raw, expected_schema=OBSERVER_CLEANUP_GUARD_SCHEMA)
            if record.data["revision"] != index or record.data["predecessor_identity"] != predecessor:
                _fail("cleanup_guard_chain", "cleanup guard predecessor chain differs")
            state = record.data["state"]
            if index and state not in transitions[prior_state]:
                _fail("cleanup_guard_chain", "cleanup guard state transition differs")
            predecessor, prior_state = record.logical_identity, state
            latest = CleanupGuardObservation(index, record)
        assert latest is not None
        self._barrier()
        return latest

    def _write_successor_locked(
        self, current: CleanupGuardObservation, value: dict[str, Any],
    ) -> CleanupGuardObservation:
        payload = canonical_authority_record_bytes(value, expected_schema=OBSERVER_CLEANUP_GUARD_SCHEMA)
        name = _cleanup_guard_revision_name(value["revision"])
        try:
            descriptor = os.open(
                name, os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_CLOEXEC | os.O_NOFOLLOW,
                0o600, dir_fd=self._guard_fd,
            )
        except FileExistsError as exc:
            raise Slice7GAuthorityDaemonError(
                "cleanup_guard_concurrent_transition", "cleanup guard successor already exists",
            ) from exc
        try:
            _write_all(descriptor, payload)
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        os.fsync(self._guard_fd)
        observed = self._load_latest_locked()
        if observed.revision != value["revision"] or observed.record.data["predecessor_identity"] != current.record.logical_identity:
            _fail("cleanup_guard_commit", "cleanup guard successor differs after commit")
        return observed

    def _barrier(self) -> None:
        if any(getattr(self, name, None) is None for name in ("_root_fd", "_guard_fd", "_lock_fd")):
            _fail("cleanup_guard_closed", "cleanup guard store is closed")
        if _directory_identity(os.fstat(self._root_fd)) != self._root_identity:
            _fail("cleanup_guard_root_replaced", "cleanup guard authority root changed")
        if _directory_identity(os.fstat(self._guard_fd)) != self._guard_identity:
            _fail("cleanup_guard_root_replaced", "cleanup guard directory changed")
        if _file_identity(os.fstat(self._lock_fd)) != self._lock_identity:
            _fail("cleanup_guard_lock_replaced", "cleanup guard lock changed")
        reopened = _open_directory_at(self._root_fd, CLEANUP_GUARD_DIRECTORY_NAME)
        try:
            if _directory_identity(os.fstat(reopened)) != self._guard_identity:
                _fail("cleanup_guard_root_replaced", "cleanup guard pathname changed")
        finally:
            os.close(reopened)
        by_name = os.stat(CLEANUP_GUARD_LOCK_NAME, dir_fd=self._guard_fd, follow_symlinks=False)
        if _file_identity(by_name) != self._lock_identity:
            _fail("cleanup_guard_lock_replaced", "cleanup guard lock pathname changed")


class GlobalAttemptBudgetStore:
    """Descriptor-confined contiguous no-replace global 0/1 state."""

    def __init__(
        self, state_root: str = AUTHORITY_STATE_ROOT, *, _test: bool = False,
        _schema: str = GLOBAL_ATTEMPT_BUDGET_SCHEMA,
    ) -> None:
        if not _test and state_root != AUTHORITY_STATE_ROOT:
            _fail("budget_root_override", "production budget root is fixed")
        self._closed = False
        self._state_root = _absolute_path(state_root)
        self._test = _test
        if _schema not in {GLOBAL_ATTEMPT_BUDGET_SCHEMA, "ctr-slice-7g-global-attempt-budget-4"}:
            _fail("budget_schema", "global budget schema is unsupported")
        self._schema = _schema
        self._root_fd = _open_directory_path(self._state_root)
        root = os.fstat(self._root_fd)
        if not _test and (root.st_uid != os.geteuid() or stat.S_IMODE(root.st_mode) != 0o700):
            self.close()
            _fail("authority_state_identity", "authority state root ownership or mode differs")
        self._root_identity = (root.st_dev, root.st_ino)
        self._budget_fd = _open_directory_at(self._root_fd, BUDGET_DIRECTORY_NAME)
        self._budget_identity = _directory_identity(os.fstat(self._budget_fd))
        self._lock_fd = os.open(BUDGET_LOCK_NAME, os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=self._budget_fd)
        lock = os.fstat(self._lock_fd)
        if not stat.S_ISREG(lock.st_mode) or lock.st_nlink != 1 or stat.S_IMODE(lock.st_mode) != 0o600:
            self.close()
            _fail("budget_lock_identity", "budget lock identity differs")
        self._lock_identity = _file_identity(lock)

    @classmethod
    def _for_test(cls, state_root: str) -> "GlobalAttemptBudgetStore":
        return cls(state_root, _test=True)

    @staticmethod
    def _provision_test_root(state_root: str, timestamp: str) -> None:
        """Private fixture helper; production never creates revision zero."""

        root = Path(_absolute_path(state_root))
        root.mkdir(mode=0o700)
        budget = root / BUDGET_DIRECTORY_NAME
        budget.mkdir(mode=0o700)
        lock_fd = os.open(budget / BUDGET_LOCK_NAME, os.O_CREAT | os.O_EXCL | os.O_RDWR, 0o600)
        os.close(lock_fd)
        initial = {
            "schema_version": GLOBAL_ATTEMPT_BUDGET_SCHEMA,
            "revision": 0,
            "predecessor_identity": None,
            "state": "UNCONSUMED",
            "attempts_consumed": 0,
            "attempts_maximum": 1,
            "retries_authorized": 0,
            "authorization_identity": None,
            "process_start_commitment": None,
            "observation_session_identity": None,
            "four_source_observation_identity": None,
            "precommit_observer_count": 0,
            "precommit_receipt_identities": [],
            "postcommit_observer_count": 0,
            "postcommit_receipt_identity": None,
            "postcommit_four_source_observation_identity": None,
            "transaction_observer_count": 0,
            "updated_at_utc": timestamp,
        }
        payload = canonical_authority_record_bytes(initial, expected_schema=GLOBAL_ATTEMPT_BUDGET_SCHEMA)
        descriptor = os.open(
            budget / _revision_name(0), os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_CLOEXEC, 0o600,
        )
        try:
            _write_all(descriptor, payload)
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        directory_fd = os.open(budget, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        ObserverCleanupGuardStore._provision_test_root(str(root), timestamp)

    def observe(self) -> BudgetObservation:
        self._barrier()
        with _locked(self._lock_fd):
            return self._load_latest_locked()

    def commit(
        self,
        *,
        authorization_identity: str,
        commitment: dict[str, Any],
        observation_session_identity: str | None = None,
        four_source_observation_identity: str | None = None,
        precommit_receipt_identities: tuple[str, ...] = (),
        timestamp: str,
    ) -> BudgetObservation:
        with _locked(self._lock_fd):
            current = self._load_latest_locked()
            if current.record.data["state"] != "UNCONSUMED" or current.record.data["attempts_consumed"] != 0:
                _fail("budget_consumed", "global Slice 7G attempt is already consumed")
            if observation_session_identity is None:
                observation_session_identity = commitment.get("observation_session_identity")
            if four_source_observation_identity is None:
                four_source_observation_identity = commitment.get("four_source_observation_identity")
            if not precommit_receipt_identities:
                supplied = commitment.get("precommit_receipt_identities", ())
                if type(supplied) in {list, tuple}:
                    precommit_receipt_identities = tuple(supplied)
            successor = {
                "schema_version": self._schema,
                "revision": current.revision + 1,
                "predecessor_identity": current.record.logical_identity,
                "state": "COMMITTED",
                "attempts_consumed": 1,
                "attempts_maximum": 1,
                "retries_authorized": 0,
                "authorization_identity": authorization_identity,
                "process_start_commitment": _plain_dict(commitment),
                "observation_session_identity": observation_session_identity,
                "four_source_observation_identity": four_source_observation_identity,
                "precommit_observer_count": len(precommit_receipt_identities),
                "precommit_receipt_identities": list(precommit_receipt_identities),
                "postcommit_observer_count": 0,
                "postcommit_receipt_identity": None,
                "postcommit_four_source_observation_identity": None,
                "transaction_observer_count": len(precommit_receipt_identities),
                "updated_at_utc": timestamp,
            }
            if self._schema == "ctr-slice-7g-global-attempt-budget-4":
                successor.update({
                    "cleanup_head_identity": commitment.get("cleanup_head_identity"),
                    "containment_receipt_identity": commitment.get(
                        "containment_receipt_identity"
                    ),
                })
            return self._write_successor_locked(current, successor)

    def finalize(
        self,
        state: str,
        *,
        timestamp: str,
        postcommit_receipt_identity: str | None = None,
        postcommit_four_source_observation_identity: str | None = None,
    ) -> BudgetObservation:
        if state not in {"COMPLETED", "FAILED_AFTER_COMMIT"}:
            _fail("budget_transition", "unsupported final budget state")
        with _locked(self._lock_fd):
            current = self._load_latest_locked()
            if current.record.data["state"] != "COMMITTED":
                _fail("budget_transition", "only COMMITTED may transition to a final state")
            successor = _builtin_authority_value(current.record.data)
            successor.update({
                "revision": current.revision + 1,
                "predecessor_identity": current.record.logical_identity,
                "state": state,
                "updated_at_utc": timestamp,
                "process_start_commitment": _builtin_authority_value(
                    current.record.data["process_start_commitment"]
                ),
            })
            if postcommit_receipt_identity is not None:
                successor.update({
                    "postcommit_observer_count": 1,
                    "postcommit_receipt_identity": postcommit_receipt_identity,
                    "postcommit_four_source_observation_identity": postcommit_four_source_observation_identity,
                    "transaction_observer_count": current.record.data["precommit_observer_count"] + 1,
                })
            return self._write_successor_locked(current, successor)

    def _validate_record(self, value: dict[str, Any] | bytes) -> Any:
        if self._schema == "ctr-slice-7g-global-attempt-budget-4":
            return validate_privileged_record(
                value, expected_schema=self._schema,
            )
        return validate_authority_record(value, expected_schema=self._schema)

    def _canonical_bytes(self, value: dict[str, Any]) -> bytes:
        if self._schema == "ctr-slice-7g-global-attempt-budget-4":
            return validate_privileged_record(
                value, expected_schema=self._schema,
            ).canonical_bytes
        return canonical_authority_record_bytes(value, expected_schema=self._schema)

    def close(self) -> None:
        if getattr(self, "_closed", True):
            return
        self._closed = True
        for name in ("_lock_fd", "_budget_fd", "_root_fd"):
            descriptor = getattr(self, name, None)
            if descriptor is not None:
                try:
                    os.close(descriptor)
                finally:
                    setattr(self, name, None)

    def _load_latest_locked(self) -> BudgetObservation:
        names = sorted(name for name in os.listdir(self._budget_fd) if name.startswith(BUDGET_REVISION_PREFIX))
        if not names or names != [_revision_name(index) for index in range(len(names))]:
            _fail("budget_revision_history", "budget revisions are missing, duplicated, or noncontiguous")
        predecessor: str | None = None
        latest: BudgetObservation | None = None
        seen_inodes: set[tuple[int, int]] = set()
        previous_state: str | None = None
        for index, name in enumerate(names):
            descriptor = os.open(name, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=self._budget_fd)
            try:
                before = os.fstat(descriptor)
                if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1 or stat.S_IMODE(before.st_mode) != 0o600:
                    _fail("budget_revision_identity", "budget revision identity differs", path=name)
                physical = (before.st_dev, before.st_ino)
                if physical in seen_inodes:
                    _fail("budget_revision_alias", "budget revisions share an inode")
                seen_inodes.add(physical)
                raw = _read_fd(descriptor, MAX_FRAME_BYTES)
                after = os.fstat(descriptor)
                if _file_identity(before) != _file_identity(after):
                    _fail("budget_revision_changed", "budget revision changed during read", path=name)
            finally:
                os.close(descriptor)
            record = self._validate_record(raw)
            if record.data["revision"] != index or record.data["predecessor_identity"] != predecessor:
                _fail("budget_revision_chain", "budget revision chain differs", path=name)
            state = record.data["state"]
            if index == 0 and state != "UNCONSUMED":
                _fail("budget_revision_chain", "revision zero is not UNCONSUMED", path=name)
            if index > 0:
                permitted = {
                    "UNCONSUMED": {"COMMITTED"},
                    "COMMITTED": {"COMPLETED", "FAILED_AFTER_COMMIT"},
                    "COMPLETED": set(),
                    "FAILED_AFTER_COMMIT": set(),
                }
                if state not in permitted[previous_state]:
                    _fail("budget_revision_chain", "budget state transition differs", path=name)
            predecessor = record.logical_identity
            previous_state = state
            latest = BudgetObservation(index, record, f"{self._state_root}/{BUDGET_DIRECTORY_NAME}/{name}", before.st_dev, before.st_ino)
        assert latest is not None
        self._barrier()
        return latest

    def _write_successor_locked(self, current: BudgetObservation, data: dict[str, Any]) -> BudgetObservation:
        payload = self._canonical_bytes(data)
        name = _revision_name(data["revision"])
        try:
            descriptor = os.open(name, os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600, dir_fd=self._budget_fd)
        except FileExistsError as exc:
            raise Slice7GAuthorityDaemonError("budget_concurrent_commit", "budget successor already exists") from exc
        try:
            _write_all(descriptor, payload)
            os.fsync(descriptor)
            info = os.fstat(descriptor)
        except BaseException:
            try:
                os.unlink(name, dir_fd=self._budget_fd)
                os.fsync(self._budget_fd)
            except OSError:
                pass
            raise
        finally:
            os.close(descriptor)
        os.fsync(self._budget_fd)
        record = self._validate_record(payload)
        observed = BudgetObservation(data["revision"], record, f"{self._state_root}/{BUDGET_DIRECTORY_NAME}/{name}", info.st_dev, info.st_ino)
        latest = self._load_latest_locked()
        if latest.record.canonical_bytes != observed.record.canonical_bytes:
            _fail("budget_commit_barrier", "durable budget successor differs after commit")
        return latest

    def _barrier(self) -> None:
        if self._closed:
            _fail("budget_closed", "budget authority is closed")
        root = os.fstat(self._root_fd)
        budget = os.fstat(self._budget_fd)
        lock = os.fstat(self._lock_fd)
        if (root.st_dev, root.st_ino) != self._root_identity or _directory_identity(budget) != self._budget_identity or _file_identity(lock) != self._lock_identity:
            _fail("budget_authority_replaced", "budget authority descriptor identity changed")
        reopened_root = _open_directory_path(self._state_root)
        try:
            if (os.fstat(reopened_root).st_dev, os.fstat(reopened_root).st_ino) != self._root_identity:
                _fail("budget_authority_replaced", "authority state pathname was replaced")
            reopened_budget = _open_directory_at(reopened_root, BUDGET_DIRECTORY_NAME)
            try:
                if _directory_identity(os.fstat(reopened_budget)) != self._budget_identity:
                    _fail("budget_authority_replaced", "budget directory pathname was replaced")
                lock_by_name = os.stat(BUDGET_LOCK_NAME, dir_fd=reopened_budget, follow_symlinks=False)
                if _file_identity(lock_by_name) != self._lock_identity:
                    _fail("budget_authority_replaced", "budget lock pathname was replaced")
            finally:
                os.close(reopened_budget)
        finally:
            os.close(reopened_root)


class AuthorityOutputProvisioner:
    """Authority-owned provisional output roots retained through the final barrier."""

    def __init__(self, *, authority_uid: int, runtime_gid: int, campaign_uid: int) -> None:
        self.authority_uid = authority_uid
        self.runtime_gid = runtime_gid
        self.campaign_uid = campaign_uid
        self._parent_fd = _open_directory_path(OUTPUT_PARENT)
        self._parent_identity = _directory_identity(os.fstat(self._parent_fd))
        self._domains: dict[int, str] = {}

    def allocate(self, prepared: PreparedCampaign, domain_id: int) -> ProvisionalAllocation:
        if type(domain_id) is not int or not 100 <= domain_id <= 199:
            _fail("domain_id", "domain ID is outside 100..199")
        if domain_id in self._domains:
            _fail("domain_contended", "domain already has a provisional Slice 7G owner")
        self._barrier()
        leaf = f"campaign-{prepared.campaign_id}"
        root_fd: int | None = None
        root_identity: tuple[int, int] | None = None
        authority_identity: tuple[int, int] | None = None
        cells_identity: tuple[int, int] | None = None
        cell_identities: dict[str, tuple[int, int]] = {}
        cells_fd: int | None = None
        try:
            os.mkdir(leaf, 0o750, dir_fd=self._parent_fd)
            root_fd = _open_directory_at(self._parent_fd, leaf)
            os.fchown(root_fd, self.authority_uid, self.runtime_gid)
            os.fchmod(root_fd, 0o750)
            root_info = os.fstat(root_fd)
            root_identity = (root_info.st_dev, root_info.st_ino)
            os.mkdir("authority", 0o700, dir_fd=root_fd)
            authority_fd = _open_directory_at(root_fd, "authority")
            os.fchown(authority_fd, self.authority_uid, -1)
            authority_identity = _path_inode(os.fstat(authority_fd))
            os.close(authority_fd)
            os.mkdir("cells", 0o750, dir_fd=root_fd)
            cells_fd = _open_directory_at(root_fd, "cells")
            os.fchown(cells_fd, self.authority_uid, self.runtime_gid)
            os.fchmod(cells_fd, 0o750)
            cells_identity = _path_inode(os.fstat(cells_fd))
            for cell in CAMPAIGN_CELLS:
                os.mkdir(cell, 0o770, dir_fd=cells_fd)
                cell_fd = _open_directory_at(cells_fd, cell)
                os.fchown(cell_fd, self.campaign_uid, self.runtime_gid)
                os.fchmod(cell_fd, 0o770)
                cell_identities[cell] = _path_inode(os.fstat(cell_fd))
                os.close(cell_fd)
            os.fsync(cells_fd)
            os.close(cells_fd)
            cells_fd = None
            os.fsync(root_fd)
            os.fsync(self._parent_fd)
            path = f"{OUTPUT_PARENT}/{leaf}"
            identity_payload = json.dumps(
                {
                    "campaign_id": prepared.campaign_id,
                    "campaign_identity": prepared.campaign_identity,
                    "device": root_info.st_dev,
                    "domain_id": domain_id,
                    "inode": root_info.st_ino,
                    "path": path,
                    "schema_version": "ctr-slice-7g-output-root-allocation-1",
                },
                ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":"),
            ).encode("utf-8")
            identity = hashlib.sha256(b"ctr-slice-7g-output-root-allocation-canonical-1\0" + identity_payload).hexdigest()
            self._domains[domain_id] = prepared.token
            closed = False
            final_read_barrier = False

            def barrier() -> None:
                nonlocal final_read_barrier
                if closed or final_read_barrier:
                    _fail("output_final_barrier", "output final barrier is closed or repeated")
                self._barrier()
                observed = os.stat(leaf, dir_fd=self._parent_fd, follow_symlinks=False)
                if _path_inode(observed) != root_identity or _path_inode(os.fstat(root_fd)) != root_identity:
                    _fail("output_root_replaced", "campaign output root was replaced")
                _assert_tree_nofollow(root_fd)
                final_read_barrier = True

            def cleanup() -> None:
                if closed:
                    return
                observed = os.stat(leaf, dir_fd=self._parent_fd, follow_symlinks=False)
                if _path_inode(observed) != root_identity:
                    _fail("output_root_replaced", "provisional output root was replaced")
                _remove_owned_empty_tree(root_fd, CAMPAIGN_CELLS)
                os.close(root_fd)
                os.rmdir(leaf, dir_fd=self._parent_fd)
                os.fsync(self._parent_fd)

            def close() -> None:
                nonlocal closed
                if closed:
                    return
                closed = True
                self._domains.pop(domain_id, None)
                try:
                    os.close(root_fd)
                except OSError:
                    pass

            return ProvisionalAllocation(
                prepared.campaign_id, prepared.campaign_identity, domain_id, path, identity,
                cleanup, barrier, close,
            )
        except BaseException as primary:
            cleanup_issues: list[str] = []
            try:
                if root_fd is not None and root_identity is not None:
                    _rollback_partial_output(
                        self._parent_fd, leaf, root_fd, root_identity,
                        authority_identity, cells_fd, cells_identity, cell_identities,
                    )
                    root_fd = None
                    cells_fd = None
            except BaseException as cleanup_exc:
                cleanup_issues.append(type(cleanup_exc).__name__)
            finally:
                for descriptor in (cells_fd, root_fd):
                    if descriptor is not None:
                        try:
                            os.close(descriptor)
                        except OSError:
                            pass
            if cleanup_issues:
                try:
                    primary.add_note(f"Slice 7G provisional output rollback issues: {cleanup_issues!r}")
                except (AttributeError, TypeError):
                    pass
            raise primary

    def close(self) -> None:
        if getattr(self, "_parent_fd", None) is not None:
            os.close(self._parent_fd)
            self._parent_fd = None

    def _barrier(self) -> None:
        if self._parent_fd is None or _directory_identity(os.fstat(self._parent_fd)) != self._parent_identity:
            _fail("output_parent_replaced", "output parent descriptor identity changed")
        reopened = _open_directory_path(OUTPUT_PARENT)
        try:
            if _directory_identity(os.fstat(reopened))[:2] != self._parent_identity[:2]:
                _fail("output_parent_replaced", "output parent pathname was replaced")
        finally:
            os.close(reopened)


class PrivilegedCleanupGuardView:
    """Query-only view of the root-owned cleanup ledger.

    The unprivileged authority daemon never receives an append operation or a
    filesystem descriptor for the ledger.  Normal and quarantine transitions
    are owned by the fixed root observer supervisor over its independent
    authenticated connection.
    """

    def __init__(self, client: CleanupAuthorityRPCClient) -> None:
        if type(client) is not CleanupAuthorityRPCClient:
            _fail("cleanup_client", "cleanup authority client type differs")
        self._client = client

    def observe(self) -> CleanupLedgerObservation:
        return self._client.query()

    def require_clear(self) -> CleanupLedgerObservation:
        observed = self.observe()
        if observed.state not in {"CLEARED", "RECOVERED"}:
            code = (
                "observation_cleanup_uncertain"
                if observed.state == "QUARANTINED" else "cleanup_guard_active"
            )
            _fail(code, "root cleanup authority blocks runtime work")
        return observed

    def close(self) -> None:
        self._client.close()


class RuntimeAuthorityStateMachine:
    """In-memory prepare state coupled to one durable global budget."""

    def __init__(
        self,
        *,
        bootstrap: dict[str, Any],
        authorization: dict[str, Any],
        budget: GlobalAttemptBudgetStore,
        cleanup_guard: ObserverCleanupGuardStore | None = None,
        service_instance_identity: str,
        peer_matcher: Callable[[Slice7GPeerProcess, Slice7GAuthorityRecord], None],
        peer_reconciler: Callable[[Slice7GPeerProcess], Slice7GPeerProcess] = reconcile_peer_process,
        provisioner: Callable[[PreparedCampaign, int], ProvisionalAllocation] | None = None,
        process_instance_validator: Callable[
            [PreparedCampaign, ProvisionalAllocation, Slice7GPeerProcess, str], None
        ] | None = None,
        observation_provider: Callable[
            [ObservationSession, str, int, int, int, Slice7GPeerProcess],
            DaemonObservationEvidence,
        ] | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        utc_now: Callable[[], str] | None = None,
        _factory_token: object | None = None,
    ) -> None:
        if _factory_token is not _PRODUCTION_FACTORY_TOKEN:
            _fail("authority_factory", "runtime authority state must use the production constructor")
        self._v7 = bootstrap.get("schema_version") == AUTHORITY_BOOTSTRAP_V3_SCHEMA
        if self._v7:
            self.bootstrap = validate_privileged_record(
                bootstrap, expected_schema=AUTHORITY_BOOTSTRAP_V3_SCHEMA,
            )
            self.authorization = validate_privileged_record(
                authorization, expected_schema=RUNTIME_AUTHORIZATION_V3_SCHEMA,
            )
            if budget._schema != GLOBAL_ATTEMPT_BUDGET_V4_SCHEMA:
                _fail("budget_schema", "charter-v7 requires global budget v4")
            self._request_schema = RUNTIME_AUTHORITY_REQUEST_V4_SCHEMA
            self._receipt_schema = RUNTIME_AUTHORITY_RECEIPT_V4_SCHEMA
            self._session_schema = OBSERVATION_SESSION_V3_SCHEMA
            self._graph_schema = ROS_GRAPH_RECEIPT_V3_SCHEMA
            self._four_schema = FOUR_SOURCE_OBSERVATION_V4_SCHEMA
        else:
            self.bootstrap = validate_authority_record(
                bootstrap, expected_schema=AUTHORITY_BOOTSTRAP_SCHEMA,
            )
            self.authorization = validate_authority_record(
                authorization, expected_schema=RUNTIME_AUTHORIZATION_SCHEMA,
            )
            self._request_schema = AUTHORITY_REQUEST_SCHEMA
            self._receipt_schema = AUTHORITY_RECEIPT_SCHEMA
            self._session_schema = OBSERVATION_SESSION_SCHEMA
            self._graph_schema = ROS_GRAPH_OBSERVATION_RECEIPT_SCHEMA
            self._four_schema = FOUR_SOURCE_OBSERVATION_SCHEMA
        self.budget = budget
        if cleanup_guard is None and getattr(budget, "_test", False) and not self._v7:
            cleanup_guard = ObserverCleanupGuardStore._for_test(budget._state_root)
        if type(cleanup_guard) not in {
            ObserverCleanupGuardStore, PrivilegedCleanupGuardView,
        }:
            _fail("cleanup_guard", "runtime authority requires the durable cleanup guard")
        self.cleanup_guard = cleanup_guard
        if self._v7 and type(cleanup_guard) is not PrivilegedCleanupGuardView:
            _fail("cleanup_guard", "charter-v7 requires query-only root cleanup authority")
        self.cleanup_guard.require_clear()
        self.service_instance_identity = _digest(service_instance_identity)
        if not callable(peer_matcher):
            _fail("peer_matcher", "peer matcher must be callable")
        self.peer_matcher = peer_matcher
        if not callable(peer_reconciler):
            _fail("peer_reconciler", "peer reconciler must be callable")
        self.peer_reconciler = peer_reconciler
        self.provisioner = provisioner
        self.process_instance_validator = process_instance_validator
        self.observation_provider = observation_provider
        self.monotonic = monotonic
        self.utc_now = utc_now or _utc_now
        self._observation_sessions: dict[str, ObservationSession] = {}
        self._observation_by_authorization: dict[str, str] = {}
        self._prepared: dict[str, PreparedCampaign] = {}
        self._allocations: dict[str, ProvisionalAllocation] = {}
        self._committed: dict[str, PreparedCampaign] = {}
        self._postcommit: dict[str, tuple[str, str]] = {}

    @classmethod
    def _for_test(cls, **kwargs: Any) -> "RuntimeAuthorityStateMachine":
        return cls(_factory_token=_PRODUCTION_FACTORY_TOKEN, **kwargs)

    @classmethod
    def _production(cls, **kwargs: Any) -> "RuntimeAuthorityStateMachine":
        """Assemble only the fixed, authenticated production provider graph."""
        bootstrap = kwargs.get("bootstrap")
        cleanup_guard = kwargs.get("cleanup_guard")
        if (
            type(bootstrap) is not dict
            or bootstrap.get("schema_version") != AUTHORITY_BOOTSTRAP_V3_SCHEMA
            or type(cleanup_guard) is not PrivilegedCleanupGuardView
        ):
            _fail("authority_factory", "production assembly requires Charter-v7 root helpers")
        return cls(_factory_token=_PRODUCTION_FACTORY_TOKEN, **kwargs)

    def _validate_runtime_record(self, value: dict[str, Any] | bytes, schema: str) -> Any:
        if self._v7:
            return validate_privileged_record(value, expected_schema=schema)
        return validate_authority_record(value, expected_schema=schema)

    def handle(
        self,
        request_value: dict[str, Any],
        peer: Slice7GPeerProcess,
        connection_identity: str,
    ) -> MappingProxyType:
        try:
            request = self._validate_runtime_record(
                request_value, self._request_schema,
            )
        except BaseException:
            self.invalidate_connection_observations(connection_identity, cleanup_uncertain=False)
            raise
        method = request.data["method"]
        try:
            self._authorize_peer(method, peer)
            if method not in {"status", "revoke"}:
                self.peer_reconciler(peer)
                self.peer_matcher(peer, request)
            self._expire_observations()
            self._expire_prepares()
            if method not in {"status", "revoke"}:
                self.cleanup_guard.require_clear()
            if method == "begin_observation":
                return self._begin_observation(request, peer, connection_identity)
            if method == "record_precommit_observation":
                return self._record_precommit_observation(request, peer, connection_identity)
            if method == "finalize_observation":
                return self._finalize_observation(request, peer, connection_identity)
            if method == "prepare":
                return self._prepare(request, peer, connection_identity)
            if method == "allocate_provisional":
                return self._allocate_provisional(request, peer, connection_identity)
            if method == "cancel":
                return self._cancel(request, peer, connection_identity)
            if method == "commit":
                return self._commit(request, peer, connection_identity)
            if method == "record_postcommit_observation":
                return self._record_postcommit_observation(request, peer, connection_identity)
            if method == "complete":
                return self._finalize(request, peer, connection_identity, "COMPLETED")
            if method == "fail_after_commit":
                return self._finalize(request, peer, connection_identity, "FAILED_AFTER_COMMIT")
            if method == "status":
                return self._status(request)
            if method == "revoke":
                return self._revoke(request)
            _fail("authority_method", "unsupported authority method")
        except BaseException as exc:
            if method in {
                "begin_observation", "record_precommit_observation",
                "finalize_observation", "prepare", "allocate_provisional",
                "commit",
            }:
                uncertain = (
                    isinstance(exc, Slice7GAuthorityDaemonError)
                    and exc.code in {
                        "observer_cleanup_uncertain", "observer_residual",
                        "observer_process_ownership", "observer_descriptor_residual",
                        "observer_dds_residual",
                    }
                )
                try:
                    self.invalidate_connection_observations(
                        connection_identity, cleanup_uncertain=uncertain,
                    )
                except BaseException as cleanup_exc:
                    _add_cleanup_note(exc, cleanup_exc, "session invalidation")
            elif method == "record_postcommit_observation":
                uncertain = (
                    isinstance(exc, Slice7GAuthorityDaemonError)
                    and exc.code in {
                        "observer_cleanup_uncertain", "observer_residual",
                        "observer_process_ownership", "observer_descriptor_residual",
                        "observer_dds_residual",
                    }
                )
                try:
                    self.invalidate_connection_observations(
                        connection_identity, cleanup_uncertain=uncertain,
                    )
                except BaseException as cleanup_exc:
                    _add_cleanup_note(exc, cleanup_exc, "session invalidation")
                try:
                    if self.budget.observe().record.data["state"] == "COMMITTED":
                        self.budget.finalize(
                            "FAILED_AFTER_COMMIT", timestamp=self.utc_now(),
                        )
                except BaseException as cleanup_exc:
                    try:
                        exc.add_note(
                            "Slice 7G postcommit failure finalization issue: "
                            + type(cleanup_exc).__name__
                        )
                    except (AttributeError, TypeError):
                        pass
            raise

    def invalidate_connection_observations(
        self, connection_identity: str, *, cleanup_uncertain: bool,
    ) -> None:
        affected = tuple(
            session for session in self._observation_sessions.values()
            if session.connection_identity == connection_identity
        )
        for nonce in tuple(self._observation_sessions):
            if self._observation_sessions[nonce].connection_identity == connection_identity:
                self._discard_observation(nonce)
        if cleanup_uncertain:
            current = self.cleanup_guard.observe()
            if type(self.cleanup_guard) is PrivilegedCleanupGuardView:
                if current.state in {"CLEARED", "RECOVERED"}:
                    _fail(
                        "cleanup_quarantine_missing",
                        "privileged supervisor did not durably quarantine uncertain cleanup",
                    )
            elif current.record.data["state"] in {"CLEARED", "RECOVERED"}:
                if not affected:
                    _fail(
                        "cleanup_quarantine_binding",
                        "cleanup uncertainty lacks an authenticated observation session",
                    )
                session = affected[0]
                active = self.cleanup_guard.begin(
                    authorization_identity=session.authorization_identity,
                    budget_identity=self.budget.observe().record.logical_identity,
                    service_generation_identity=self.service_instance_identity,
                    session_binding_identity=session.identity,
                    phase="PRECOMMIT", phase_local_ordinal=max(1, len(session.receipt_identities) + 1),
                    transaction_observer_ordinal=max(1, len(session.receipt_identities) + 1),
                    domain_id=session.selected_domain or (100 + len(session.receipt_identities)),
                    executable_identity=session.process_manifest_identity,
                    argv_identity=_domain_identity(
                        b"ctr-slice-7g-observer-argv-canonical-1\0",
                        {"argv": ["/opt/ros/humble/bin/ros2", "node", "list", "--no-daemon"]},
                    ),
                    environment_identity=session.environment_manifest_identity,
                    timestamp=self.utc_now(),
                )
                self.cleanup_guard.quarantine(
                    active.record.logical_identity,
                    _domain_identity(
                        b"ctr-slice-7g-cleanup-uncertain-canonical-1\0",
                        {"connection_identity": connection_identity, "session_binding_identity": session.identity},
                    ),
                    self.utc_now(),
                )

    def disconnect(self, connection_identity: str) -> None:
        errors: list[BaseException] = []
        for nonce in [
            key for key, value in self._observation_sessions.items()
            if value.connection_identity == connection_identity
        ]:
            self._discard_observation(nonce)
        for token in [key for key, value in self._prepared.items() if value.connection_identity == connection_identity]:
            try:
                self._discard_prepare(token)
            except BaseException as exc:
                errors.append(exc)
        # Any retained allocation without prepare state is post-commit and can
        # never restore the attempt.  A lost session is a durable failure.
        for token, prepared in tuple(self._committed.items()):
            if prepared.connection_identity == connection_identity:
                allocation = self._allocations.get(token)
                try:
                    current = self.budget.observe()
                    if current.record.data["state"] == "COMMITTED":
                        self.budget.finalize("FAILED_AFTER_COMMIT", timestamp=self.utc_now())
                except BaseException as exc:
                    errors.append(exc)
                finally:
                    if allocation is not None:
                        try:
                            allocation.close()
                        except BaseException as exc:
                            errors.append(exc)
                        self._allocations.pop(token, None)
                    self._postcommit.pop(token, None)
                    for nonce, observation in tuple(self._observation_sessions.items()):
                        if observation.identity == prepared.observation_session_identity:
                            self._discard_observation(nonce)
                    self._committed.pop(token, None)
        _raise_cleanup_errors(errors, "disconnect cleanup")

    def recover_abandoned_commit(self) -> BudgetObservation:
        """Durably fail a committed attempt abandoned by an authority restart.

        A COMMITTED revision can only exist after the permanent process-start
        boundary.  The in-memory connection/preparation state deliberately
        does not survive an authority-service restart, so startup must retain
        the consumed attempt, request fixed-unit cgroup termination, and add a
        contiguous FAILED_AFTER_COMMIT successor before serving new clients.
        """

        current = self.budget.observe()
        if current.record.data["state"] != "COMMITTED":
            return current
        if current.record.data["authorization_identity"] != self.authorization.logical_identity:
            _fail(
                "restart_authorization_binding",
                "abandoned committed budget belongs to another authorization",
            )
        revocation_id = f"service-restart-revision-{current.revision:020d}"
        existing = _read_optional_private_revision(
            self.budget._root_fd, REVOCATION_PENDING_NAME,
            revocation_id + ".json", AUTHORITY_REVOCATION_SCHEMA,
        )
        if existing is None:
            requested_at = self.utc_now()
            trigger_identity = _revocation_trigger_identity(
                revocation_id, self.authorization.logical_identity,
                current.revision, requested_at,
            )
            value = {
                "schema_version": AUTHORITY_REVOCATION_SCHEMA,
                "revocation_id": revocation_id,
                "authorization_identity": self.authorization.logical_identity,
                "budget_revision": current.revision,
                "state": "TRIGGERED_POSTCOMMIT",
                "requested_at_utc": requested_at,
                "requested_by_uid": self.bootstrap.data["authority_uid"],
                "trigger_identity": trigger_identity,
                "processed_trigger_identity": None,
                "termination_receipt_identity": None,
            }
            payload = canonical_authority_record_bytes(
                value, expected_schema=AUTHORITY_REVOCATION_SCHEMA,
            )
            _write_private_revision(
                self.budget._root_fd, REVOCATION_PENDING_NAME,
                revocation_id + ".json", payload,
            )
        elif (
            existing.data["revocation_id"] != revocation_id
            or existing.data["authorization_identity"]
            != self.authorization.logical_identity
            or existing.data["budget_revision"] != current.revision
            or existing.data["state"] != "TRIGGERED_POSTCOMMIT"
        ):
            _fail("restart_revocation_binding", "restart revocation trigger differs")
        return self.budget.finalize("FAILED_AFTER_COMMIT", timestamp=self.utc_now())

    def _authorize_peer(self, method: str, peer: Slice7GPeerProcess) -> None:
        credentials = peer.credentials
        if method in {"status", "revoke"}:
            if credentials.uid != 0:
                _fail("admin_peer", "administrative method requires UID 0")
            return
        if credentials.uid != self.bootstrap.data["campaign_uid"] or credentials.gid != self.bootstrap.data["runtime_gid"]:
            _fail("campaign_peer", "campaign peer numeric UID/GID differs")

    def _begin_observation(
        self, request: Slice7GAuthorityRecord, peer: Slice7GPeerProcess, connection: str,
    ) -> MappingProxyType:
        self._check_authorization(request)
        if self._authorization_revoked():
            _fail("authorization_revoked", "runtime authorization is revoked")
        latest = self._require_unconsumed_authority()
        if self.authorization.logical_identity in self._observation_by_authorization:
            _fail("observation_session_active", "authorization already has an active observation session")
        if any(request.data[field] is not None for field in (
            "observation_session_identity", "observation_session_nonce", "domain_id",
        )):
            _fail("observation_authority_injection", "observation request contains caller authority")
        nonce = "o" + secrets.token_hex(24)
        created_ns = int(self.monotonic() * 1_000_000_000)
        deadline_ns = created_ns + OBSERVATION_SESSION_LIFETIME_SECONDS * 1_000_000_000
        cleanup_head_identity = None
        privileged_service_identity = None
        if self._v7:
            cleanup = self.cleanup_guard.require_clear()
            cleanup_head_identity = cleanup.head.logical_identity
            privileged_service_identity = self.authorization.data[
                "privileged_service_manifest_identity"
            ]
        # This immutable binding is deliberately computed before, and without,
        # candidate or receipt state.  Receipts point to it; the finalized
        # observation subsequently points to the ordered receipt identities.
        session_binding_identity = _domain_identity(
            b"ctr-slice-7g-observation-session-binding-canonical-1\0",
            {
                "authorization_identity": self.authorization.logical_identity,
                "installed_runtime_identity": self.authorization.data[
                    "installed_runtime_identity"
                ],
                "process_manifest_identity": self.authorization.data[
                    "process_manifest_identity"
                ],
                "environment_manifest_identity": self.authorization.data[
                    "environment_manifest_identity"
                ],
                "connection_identity": connection,
                "peer_uid": peer.credentials.uid,
                "peer_gid": peer.credentials.gid,
                "peer_pid": peer.credentials.pid,
                "peer_start_time_ticks": peer.start_time_ticks,
                "campaign_cgroup": peer.cgroup,
                "service_nonce": nonce,
                "daemon_generation_identity": self.service_instance_identity,
                "privileged_service_manifest_identity": privileged_service_identity,
                "cleanup_head_identity": cleanup_head_identity,
                "created_monotonic_ns": created_ns,
                "deadline_monotonic_ns": deadline_ns,
                "domain_minimum": 100,
                "domain_maximum": 199,
            },
        )
        value = {
            "schema_version": self._session_schema,
            "authorization_identity": self.authorization.logical_identity,
            "installed_runtime_identity": self.authorization.data["installed_runtime_identity"],
            "process_manifest_identity": self.authorization.data["process_manifest_identity"],
            "environment_manifest_identity": self.authorization.data["environment_manifest_identity"],
            "connection_identity": connection,
            "peer_uid": peer.credentials.uid, "peer_gid": peer.credentials.gid,
            "peer_pid": peer.credentials.pid, "peer_start_time_ticks": peer.start_time_ticks,
            "campaign_cgroup": peer.cgroup, "service_nonce": nonce,
            "daemon_generation_identity": self.service_instance_identity,
            "created_monotonic_ns": created_ns, "deadline_monotonic_ns": deadline_ns,
            "domain_minimum": 100, "domain_maximum": 199,
            "maximum_precommit_observers": MAX_PRECOMMIT_OBSERVERS,
            "precommit_observer_count": 0, "postcommit_observer_count": 0,
            "transaction_observer_count": 0, "candidate_domains": [],
            "precommit_receipt_identities": [], "selected_domain": None,
            "lease_identity": None, "four_source_observation_identity": None,
            "state": "OPEN",
        }
        if self._v7:
            value.update({
                "privileged_service_manifest_identity": privileged_service_identity,
                "cleanup_head_identity": cleanup_head_identity,
            })
        self._validate_runtime_record(value, self._session_schema)
        session = ObservationSession(
            nonce, session_binding_identity, connection, peer,
            self.authorization.logical_identity,
            self.authorization.data["installed_runtime_identity"],
            self.authorization.data["process_manifest_identity"],
            self.authorization.data["environment_manifest_identity"],
            created_ns, deadline_ns, (), (), (), (), None, None, None, False,
            privileged_service_identity, cleanup_head_identity, None,
        )
        self._observation_sessions[nonce] = session
        self._observation_by_authorization[self.authorization.logical_identity] = nonce
        return self._receipt(
            request, "OBSERVATION_STARTED", latest,
            authorization_identity=self.authorization.logical_identity,
            observation=session,
        )

    def _record_precommit_observation(
        self, request: Slice7GAuthorityRecord, peer: Slice7GPeerProcess, connection: str,
    ) -> MappingProxyType:
        session = self._require_observation(request, peer, connection, finalized=False)
        if session.selected_domain is not None:
            _fail("observer_after_selection", "selected domain cannot be observed again")
        domain = request.data["domain_id"]
        expected_domain = 100 + len(session.receipt_identities)
        if domain != expected_domain:
            _fail("observer_candidate_order", "daemon observations require ascending candidates")
        phase_ordinal = len(session.receipt_identities) + 1
        validated, four = self._observe_authoritatively(
            session, "PRECOMMIT", domain, phase_ordinal, phase_ordinal, peer,
        )
        candidates = (*session.candidate_domains, domain)
        if len(candidates) > MAX_PRECOMMIT_OBSERVERS:
            _fail("observer_counter", "precommit observer count exceeds 100")
        receipts = (*session.receipt_identities, validated.logical_identity)
        candidate_clear = bool(four.data["all_sources_clear"] and not validated.data["nodes"])
        updated = replace(
            session, candidate_domains=candidates, receipt_identities=receipts,
            receipt_records=(*session.receipt_records, validated),
            four_source_records=(*session.four_source_records, four),
            selected_domain=domain if candidate_clear else None,
            lease_identity=(four.data["global_lease_identity"] if candidate_clear else None),
            cleanup_head_identity=(
                validated.data["cleanup_head_identity"] if self._v7
                else session.cleanup_head_identity
            ),
            containment_receipt_identity=(
                validated.data["containment_receipt_identity"] if self._v7
                else session.containment_receipt_identity
            ),
        )
        self._observation_sessions[session.nonce] = updated
        return self._receipt(
            request, "OBSERVATION_RECORDED", self.budget.observe(),
            authorization_identity=session.authorization_identity, observation=updated,
            candidate_clear=candidate_clear,
        )

    def _finalize_observation(
        self, request: Slice7GAuthorityRecord, peer: Slice7GPeerProcess, connection: str,
    ) -> MappingProxyType:
        session = self._require_observation(request, peer, connection, finalized=False)
        if not session.receipt_identities:
            _fail("observation_incomplete", "successful domain selection requires an observer receipt")
        four = session.four_source_records[-1]
        selected = session.selected_domain
        if (
            selected is None
            or
            four.data["phase"] != "PRECOMMIT"
            or four.data["session_binding_identity"] != session.identity
            or four.data["service_nonce"] != session.nonce
            or session.receipt_records[-1].data["four_source_observation_identity"]
            != four.logical_identity
            or session.receipt_records[-1].data["nodes"]
            or not four.data["all_sources_clear"]
            or selected != session.candidate_domains[-1]
            or request.data["domain_id"] not in {None, selected}
        ):
            _fail("four_source_binding", "four-source observation binding differs")
        final_identity = _domain_identity(
            b"ctr-slice-7g-final-domain-observation-canonical-1\0",
            {
                "session_binding_identity": session.identity,
                "service_nonce": session.nonce,
                "candidate_domains": list(session.candidate_domains),
                "precommit_receipt_identities": list(session.receipt_identities),
                "precommit_observer_count": len(session.receipt_identities),
                "selected_domain": selected,
                "selected_four_source_identity": four.logical_identity,
            },
        )
        updated = replace(
            session, four_source_observation_identity=final_identity, finalized=True,
        )
        self._observation_sessions[session.nonce] = updated
        return self._receipt(
            request, "OBSERVATION_COMPLETE", self.budget.observe(),
            authorization_identity=session.authorization_identity, observation=updated,
        )

    def _observe_authoritatively(
        self,
        session: ObservationSession,
        phase: str,
        domain: int,
        phase_ordinal: int,
        transaction_ordinal: int,
        peer: Slice7GPeerProcess,
    ) -> tuple[Slice7GAuthorityRecord, Slice7GAuthorityRecord]:
        if self.observation_provider is None:
            _fail("observation_provider", "daemon-owned observation provider is unavailable")
        try:
            evidence = self.observation_provider(
                session, phase, domain, phase_ordinal, transaction_ordinal, peer,
            )
        except (Slice7GAuthorityDaemonError, Slice7GAuthorityProtocolError):
            raise
        except BaseException as exc:
            if not isinstance(exc, Exception):
                raise
            raise Slice7GAuthorityDaemonError(
                "observation_provider_failed", type(exc).__name__,
            ) from exc
        if type(evidence) is not DaemonObservationEvidence:
            _fail("observation_provider_type", "daemon provider returned an unsupported value")
        four_value = {
            "schema_version": self._four_schema,
            "session_binding_identity": session.identity,
            "service_nonce": session.nonce,
            "phase": phase,
            "phase_local_ordinal": phase_ordinal,
            "transaction_observer_ordinal": transaction_ordinal,
            "domain_id": domain,
            "peer_process_identity": evidence.peer_process_identity,
            "observation_interval_identity": evidence.observation_interval_identity,
            "cleanup_disposition_identity": evidence.cleanup_barrier_identity,
            "active_process_identity": evidence.active_process_identity,
            "dds_port_identity": evidence.dds_port_identity,
            "global_lease_identity": evidence.global_lease_identity,
            "global_lease_registry_identity": evidence.global_lease_registry_identity,
            "global_lease_revision_identity": evidence.global_lease_revision_identity,
            "global_lease_state": evidence.global_lease_state,
            "global_lease_clear": evidence.global_lease_clear,
            "ros_graph_provider_identity": evidence.graph_provider_identity,
            "all_sources_clear": (
                evidence.active_process_clear
                and evidence.dds_port_clear
                and evidence.global_lease_clear
                and not evidence.nodes
            ),
            "observed_monotonic_ns": evidence.observed_monotonic_ns,
        }
        if self._v7:
            if evidence.cleanup_head_identity is None or evidence.containment_receipt_identity is None:
                _fail("privileged_observation_binding", "helper evidence lacks cleanup/containment authority")
            four_value.update({
                "cleanup_head_identity": evidence.cleanup_head_identity,
                "containment_receipt_identity": evidence.containment_receipt_identity,
            })
        four = self._validate_runtime_record(four_value, self._four_schema)
        parsed_identity = _domain_identity(
            b"ctr-slice-7g-ros-node-set-canonical-1\0", {"nodes": list(evidence.nodes)},
        )
        receipt_value = {
            "schema_version": self._graph_schema,
            "session_binding_identity": session.identity,
            "service_nonce": session.nonce,
            "phase": phase,
            "phase_local_ordinal": phase_ordinal,
            "transaction_observer_ordinal": transaction_ordinal,
            "four_source_observation_identity": four.logical_identity,
            "observer_class": "PRECOMMIT_ROS_GRAPH_OBSERVER",
            "executable": evidence.executable,
            "executable_identity": evidence.executable_identity,
            "interpreter": evidence.interpreter,
            "interpreter_identity": evidence.interpreter_identity,
            "module_origin_identities": list(evidence.module_origin_identities),
            "argv": list(evidence.argv),
            "environment_identity": evidence.environment_identity,
            "working_directory": evidence.working_directory,
            "cgroup": evidence.cgroup,
            "shell": False,
            "domain_id": domain,
            "pid": evidence.pid,
            "process_group_id": evidence.process_group_id,
            "process_start_time_ticks": evidence.process_start_time_ticks,
            "started_monotonic_ns": evidence.started_monotonic_ns,
            "ended_monotonic_ns": evidence.ended_monotonic_ns,
            "exit_status": evidence.exit_status,
            "terminating_signal": evidence.terminating_signal,
            "stdout_size": len(evidence.stdout),
            "stdout_sha256": hashlib.sha256(evidence.stdout).hexdigest(),
            "stderr_size": len(evidence.stderr),
            "stderr_sha256": hashlib.sha256(evidence.stderr).hexdigest(),
            "nodes": list(evidence.nodes),
            "parsed_node_set_identity": parsed_identity,
            "cleanup_barrier_identity": evidence.cleanup_barrier_identity,
            "unexpected_descendants": evidence.unexpected_descendants,
            "ros_daemon_started": evidence.ros_daemon_started,
        }
        if self._v7:
            receipt_value.update({
                "cleanup_head_identity": evidence.cleanup_head_identity,
                "containment_receipt_identity": evidence.containment_receipt_identity,
            })
        receipt = self._validate_runtime_record(receipt_value, self._graph_schema)
        if (
            receipt.data["exit_status"] != 0
            or receipt.data["terminating_signal"] is not None
            or receipt.data["stderr_size"] != 0
            or receipt.data["unexpected_descendants"] != 0
            or receipt.data["ros_daemon_started"]
        ):
            _fail("observer_result", "daemon observer did not complete cleanly")
        return receipt, four

    def _prepare(self, request: Slice7GAuthorityRecord, peer: Slice7GPeerProcess, connection: str) -> MappingProxyType:
        session = self._require_observation(request, peer, connection, finalized=True)
        latest = self._require_unconsumed_authority()
        if any(request.data[field] is not None for field in (
            "campaign_id", "campaign_identity", "prepare_token", "campaign_template_identity",
            "output_root_path", "output_root_identity", "process_instance_identity",
        )):
            _fail("prepare_authority_injection", "prepare request contains caller authority values")
        if (
            request.data["authorization_identity"] not in {
                None, self.authorization.logical_identity,
            }
            or request.data["process_manifest_identity"] not in {
                None, session.process_manifest_identity,
            }
            or request.data["domain_id"] not in {None, session.selected_domain}
        ):
            _fail("prepare_observation_binding", "prepare request differs from finalized observation")
        token = "p" + secrets.token_hex(24)
        campaign_id = "c" + secrets.token_hex(16)
        campaign_identity = _runtime_campaign_identity(
            self.authorization.logical_identity, campaign_id,
            self.authorization.data["campaign"]["plan_identity"],
        )
        lifetime = self.authorization.data["prepare_token_lifetime_seconds"]
        if lifetime != PREPARE_TOKEN_LIFETIME_SECONDS:
            _fail("prepare_token_lifetime", "prepare token lifetime must be exactly 300 seconds")
        prepared = PreparedCampaign(
            token, connection, peer, self.authorization.logical_identity, campaign_id,
            campaign_identity, self.authorization.data["campaign"]["plan_identity"],
            session.identity, session.four_source_observation_identity,
            session.receipt_identities, len(session.receipt_identities),
            session.selected_domain, session.lease_identity,
            self.monotonic() + lifetime,
            session.cleanup_head_identity, session.containment_receipt_identity,
        )
        self._prepared[token] = prepared
        self._observation_sessions[session.nonce] = replace(session, finalized=True)
        return self._receipt(
            request, "PREPARED", latest, prepare_token=token,
            authorization_identity=prepared.authorization_identity,
            campaign_id=campaign_id, campaign_identity=campaign_identity, observation=session,
            prepare_expires_monotonic_ns=int(prepared.expires_monotonic * 1_000_000_000),
        )

    def _allocate_provisional(
        self, request: Slice7GAuthorityRecord, peer: Slice7GPeerProcess, connection: str,
    ) -> MappingProxyType:
        prepared = self._require_prepare(request, peer, connection)
        if self.provisioner is None:
            _fail("provisioner_unavailable", "authority provisional allocator is unavailable")
        if request.data["domain_id"] not in {None, prepared.selected_domain}:
            _fail("allocation_observation_binding", "allocation differs from observed domain or lease")
        if any(request.data[field] is not None for field in (
            "output_root_path", "output_root_identity", "process_instance_identity",
        )):
            _fail("allocation_authority_injection", "allocation request contains caller output authority")
        if prepared.token in self._allocations:
            _fail("allocation_replay", "prepare token already owns an allocation")
        allocation = self.provisioner(prepared, prepared.selected_domain)
        if type(allocation) is not ProvisionalAllocation:
            _fail("allocation_type", "provisional allocator returned an unsupported record")
        self._allocations[prepared.token] = allocation
        return self._receipt(
            request, "PREPARED", self.budget.observe(), prepare_token=prepared.token,
            authorization_identity=prepared.authorization_identity,
            campaign_id=prepared.campaign_id, campaign_identity=prepared.campaign_identity,
            output_root_path=allocation.output_root_path,
            output_root_identity=allocation.output_root_identity,
            observation=self._observation_for_prepared(prepared),
        )

    def _cancel(self, request: Slice7GAuthorityRecord, peer: Slice7GPeerProcess, connection: str) -> MappingProxyType:
        if request.data["prepare_token"] is None:
            observation = self._require_observation(
                request, peer, connection,
                finalized=bool(
                    request.data["domain_id"] is not None
                    and self._observation_sessions.get(
                        request.data["observation_session_nonce"],
                    ) is not None
                    and self._observation_sessions[
                        request.data["observation_session_nonce"]
                    ].finalized
                ),
            )
            self._discard_observation(observation.nonce)
            return self._receipt(request, "CANCELLED", self.budget.observe())
        self._require_prepare(request, peer, connection)
        self._discard_prepare(request.data["prepare_token"])
        return self._receipt(request, "CANCELLED", self.budget.observe())

    def _commit(self, request: Slice7GAuthorityRecord, peer: Slice7GPeerProcess, connection: str) -> MappingProxyType:
        prepared = self._require_prepare(request, peer, connection)
        allocation = self._allocations.get(prepared.token)
        if allocation is None:
            _fail("commit_allocation", "commit requires an authority-owned provisional allocation")
        for field in ("domain_id", "output_root_identity", "process_manifest_identity", "process_instance_identity"):
            if request.data[field] is None:
                _fail("commit_binding", "commit request lacks a permanent process-start binding", path=f"$.{field}")
        if request.data["process_manifest_identity"] != self.authorization.data["process_manifest_identity"]:
            _fail("commit_process_manifest", "commit process manifest differs from authorization")
        if self.process_instance_validator is None:
            _fail("process_instance_validator", "process instance validator is unavailable")
        self.process_instance_validator(
            prepared, allocation, peer, request.data["process_instance_identity"],
        )
        if (
            request.data["campaign_id"] != prepared.campaign_id
            or request.data["campaign_identity"] != prepared.campaign_identity
            or request.data["domain_id"] != allocation.domain_id
            or request.data["output_root_path"] != allocation.output_root_path
            or request.data["output_root_identity"] != allocation.output_root_identity
            or request.data["observation_session_identity"]
            != prepared.observation_session_identity
        ):
            _fail("commit_allocation", "commit differs from the prepared allocation")
        commitment = {
            "campaign_identity": prepared.campaign_identity,
            "campaign_template_identity": prepared.campaign_template_identity,
            "domain_id": request.data["domain_id"],
            "output_root_identity": request.data["output_root_identity"],
            "process_manifest_identity": request.data["process_manifest_identity"],
            "process_instance_identity": request.data["process_instance_identity"],
            "observation_session_identity": prepared.observation_session_identity,
            "four_source_observation_identity": prepared.four_source_observation_identity,
            "precommit_receipt_identities": list(prepared.precommit_receipt_identities),
            "precommit_observer_count": prepared.precommit_observer_count,
            "prepare_token_identity": hashlib.sha256(prepared.token.encode("utf-8")).hexdigest(),
            "lease_identity": prepared.lease_identity,
            "peer_pid": peer.credentials.pid,
            "peer_start_time_ticks": peer.start_time_ticks,
            "peer_executable": peer.executable,
            "committed_at_utc": self.utc_now(),
            "service_instance_identity": self.service_instance_identity,
        }
        if self._v7:
            if prepared.cleanup_head_identity is None or prepared.containment_receipt_identity is None:
                _fail("privileged_observation_binding", "commit lacks cleanup/containment authority")
            commitment.update({
                "cleanup_head_identity": prepared.cleanup_head_identity,
                "containment_receipt_identity": prepared.containment_receipt_identity,
            })
        observed = self.budget.commit(
            authorization_identity=prepared.authorization_identity,
            commitment=commitment,
            observation_session_identity=prepared.observation_session_identity,
            four_source_observation_identity=prepared.four_source_observation_identity,
            precommit_receipt_identities=prepared.precommit_receipt_identities,
            timestamp=commitment["committed_at_utc"],
        )
        del self._prepared[prepared.token]
        self._committed[prepared.token] = prepared
        self._allocations.pop(prepared.token, None)
        receipt = self._receipt(
            request, "COMMITTED", observed, committed_at=commitment["committed_at_utc"],
            observation=self._observation_for_prepared(prepared),
        )
        self._persist_receipt(dict(receipt))
        # The descriptor remains retained by the service until finalization.
        self._allocations[prepared.token] = allocation
        return receipt

    def _record_postcommit_observation(
        self, request: Slice7GAuthorityRecord, peer: Slice7GPeerProcess, connection: str,
    ) -> MappingProxyType:
        token = request.data["prepare_token"]
        prepared = self._committed.get(token)
        if prepared is None or prepared.connection_identity != connection or prepared.peer != peer:
            _fail("postcommit_binding", "postcommit observation lacks committed authority")
        if token in self._postcommit:
            _fail("postcommit_replay", "postcommit observer cannot be retried")
        session = self._observation_for_prepared(prepared)
        if request.data["domain_id"] not in {None, prepared.selected_domain}:
            _fail("postcommit_binding", "postcommit candidate domain differs")
        receipt, four = self._observe_authoritatively(
            session, "POSTCOMMIT", prepared.selected_domain, 1,
            prepared.precommit_observer_count + 1, peer,
        )
        if (
            receipt.data["nodes"] or not four.data["all_sources_clear"]
            or receipt.data["session_binding_identity"]
            != prepared.observation_session_identity
            or receipt.data["service_nonce"] != session.nonce
            or receipt.data["four_source_observation_identity"] != four.logical_identity
        ):
            _fail("postcommit_binding", "postcommit observation binding differs")
        self._postcommit[token] = (receipt.logical_identity, four.logical_identity)
        return self._receipt(
            request, "POSTCOMMIT_RECORDED", self.budget.observe(), observation=session,
            postcommit_observer_count=1,
        )

    def _finalize(self, request: Slice7GAuthorityRecord, peer: Slice7GPeerProcess, connection: str, state: str) -> MappingProxyType:
        token = request.data["prepare_token"]
        prepared = self._committed.get(token)
        allocation = self._allocations.get(token)
        if prepared is None or allocation is None:
            _fail("final_barrier", "finalization lacks retained committed authority")
        if prepared.connection_identity != connection or prepared.peer != peer:
            _fail("finalization_binding", "finalization peer or connection differs")
        if (
            request.data["authorization_identity"] != prepared.authorization_identity
            or request.data["campaign_id"] != prepared.campaign_id
            or request.data["campaign_identity"] != prepared.campaign_identity
            or request.data["campaign_template_identity"] != prepared.campaign_template_identity
            or request.data["domain_id"] != allocation.domain_id
            or request.data["output_root_path"] != allocation.output_root_path
            or request.data["output_root_identity"] != allocation.output_root_identity
            or request.data["observation_session_identity"] != prepared.observation_session_identity
        ):
            _fail("finalization_binding", "finalization authority bindings differ")
        postcommit = self._postcommit.get(token)
        if state == "COMPLETED" and postcommit is None:
            _fail("postcommit_required", "campaign cannot complete before the mandatory postcommit observation")
        try:
            allocation.final_barrier()
            observed = self.budget.finalize(
                state, timestamp=self.utc_now(),
                postcommit_receipt_identity=None if postcommit is None else postcommit[0],
                postcommit_four_source_observation_identity=None if postcommit is None else postcommit[1],
            )
        except BaseException:
            if self.budget.observe().record.data["state"] == "COMMITTED":
                self.budget.finalize("FAILED_AFTER_COMMIT", timestamp=self.utc_now())
            raise
        finally:
            allocation.close()
            self._allocations.pop(token, None)
            self._committed.pop(token, None)
            self._postcommit.pop(token, None)
            for nonce, observation in tuple(self._observation_sessions.items()):
                if observation.identity == prepared.observation_session_identity:
                    self._discard_observation(nonce)
        return self._receipt(request, state, observed)

    def _status(self, request: Slice7GAuthorityRecord) -> MappingProxyType:
        return self._receipt(request, "STATUS", self.budget.observe())

    def _revoke(self, request: Slice7GAuthorityRecord) -> MappingProxyType:
        if request.data["authorization_identity"] not in {None, self.authorization.logical_identity}:
            _fail("revocation_authorization", "revocation authorization differs")
        current = self.budget.observe()
        errors: list[BaseException] = []
        for nonce in [
            key for key, value in self._observation_sessions.items()
            if value.authorization_identity == self.authorization.logical_identity
        ]:
            self._discard_observation(nonce)
        for token in [key for key, value in self._prepared.items() if value.authorization_identity == self.authorization.logical_identity]:
            try:
                self._discard_prepare(token)
            except BaseException as exc:
                errors.append(exc)
        try:
            self._write_revocation(
                request, current, postcommit=current.record.data["state"] != "UNCONSUMED",
            )
        except BaseException as exc:
            errors.append(exc)
        _raise_cleanup_errors(errors, "revocation cleanup")
        result = "REVOKED"
        return self._receipt(request, result, current)

    def _check_authorization(self, request: Slice7GAuthorityRecord) -> None:
        if request.data["authorization_identity"] is not None:
            _fail("prepare_authority_injection", "prepare cannot select an authorization identity")
        if request.data["campaign_template_identity"] is not None:
            _fail("prepare_authority_injection", "prepare cannot select a campaign template")
        if self._v7 and request.data["privileged_service_manifest_identity"] != self.authorization.data[
            "privileged_service_manifest_identity"
        ]:
            _fail(
                "privileged_service_binding",
                "runtime request privileged-service identity differs from authorization",
            )

    def _require_unconsumed_authority(self) -> BudgetObservation:
        if self._authorization_revoked():
            _fail("authorization_revoked", "runtime authorization is revoked")
        latest = self.budget.observe()
        if latest.record.data["state"] != "UNCONSUMED":
            _fail("budget_consumed", "global attempt is not unconsumed")
        if latest.revision != 0 or latest.record.logical_identity != self.authorization.data["global_budget_identity"]:
            _fail("global_budget_binding", "authorization global-budget authority differs")
        now = _parse_utc(self.utc_now())
        if not (
            _parse_utc(self.authorization.data["not_before_utc"])
            <= now
            < _parse_utc(self.authorization.data["not_after_utc"])
        ):
            _fail("authorization_time", "runtime authorization is not currently valid")
        return latest

    def _require_observation(
        self,
        request: Slice7GAuthorityRecord,
        peer: Slice7GPeerProcess,
        connection: str,
        *,
        finalized: bool,
    ) -> ObservationSession:
        nonce = request.data["observation_session_nonce"]
        session = self._observation_sessions.get(nonce)
        if session is None:
            _fail("observation_session", "observation session is missing, expired, or invalidated")
        if session.connection_identity != connection or session.peer != peer:
            _fail("observation_binding", "observation session peer or connection differs")
        if session.deadline_monotonic_ns <= int(self.monotonic() * 1_000_000_000):
            self._discard_observation(nonce)
            _fail("observation_session_expired", "observation session is expired")
        if request.data["observation_session_identity"] != session.identity:
            _fail("observation_binding", "observation-session identity differs")
        if session.finalized is not finalized:
            _fail(
                "observation_state",
                "observation session must be finalized" if finalized else "observation session is already finalized",
            )
        return session

    def _observation_for_prepared(self, prepared: PreparedCampaign) -> ObservationSession:
        for session in self._observation_sessions.values():
            if session.identity == prepared.observation_session_identity:
                return session
        _fail("observation_session", "prepared observation session is no longer active")

    def _expire_observations(self) -> None:
        now_ns = int(self.monotonic() * 1_000_000_000)
        for nonce in [
            key for key, value in self._observation_sessions.items()
            if value.deadline_monotonic_ns <= now_ns
            and all(
                prepared.observation_session_identity != value.identity
                for prepared in (*self._prepared.values(), *self._committed.values())
            )
        ]:
            self._discard_observation(nonce)

    def _discard_observation(self, nonce: str) -> None:
        session = self._observation_sessions.pop(nonce, None)
        if session is not None:
            self._observation_by_authorization.pop(session.authorization_identity, None)

    def _require_prepare(self, request: Slice7GAuthorityRecord, peer: Slice7GPeerProcess, connection: str) -> PreparedCampaign:
        token = request.data["prepare_token"]
        prepared = self._prepared.get(token)
        if prepared is None:
            _fail("prepare_token", "prepare token is missing, expired, cancelled, or replayed")
        if prepared.connection_identity != connection or prepared.peer != peer:
            _fail("prepare_binding", "prepare token binding differs")
        if prepared.expires_monotonic <= self.monotonic():
            self._discard_prepare(token)
            _fail("prepare_token", "prepare token is expired")
        if (
            request.data["authorization_identity"] != prepared.authorization_identity
            or request.data["campaign_id"] != prepared.campaign_id
            or request.data["campaign_identity"] != prepared.campaign_identity
            or request.data["campaign_template_identity"] != prepared.campaign_template_identity
            or request.data["observation_session_identity"] != prepared.observation_session_identity
        ):
            _fail("prepare_binding", "prepare request identity binding differs")
        return prepared

    def _expire_prepares(self) -> None:
        now = self.monotonic()
        errors: list[BaseException] = []
        for token in [key for key, value in self._prepared.items() if value.expires_monotonic <= now]:
            try:
                self._discard_prepare(token)
            except BaseException as exc:
                errors.append(exc)
        _raise_cleanup_errors(errors, "expired prepare cleanup")

    def _discard_prepare(self, token: str) -> None:
        allocation = self._allocations.pop(token, None)
        prepared = self._prepared.pop(token, None)
        if prepared is not None:
            for nonce, session in tuple(self._observation_sessions.items()):
                if session.identity == prepared.observation_session_identity:
                    self._discard_observation(nonce)
        errors: list[BaseException] = []
        if allocation is not None:
            try:
                allocation.cleanup()
            except BaseException as exc:
                errors.append(exc)
            try:
                allocation.close()
            except BaseException as exc:
                errors.append(exc)
        _raise_cleanup_errors(errors, "provisional allocation cleanup")

    def _write_revocation(
        self, request: Slice7GAuthorityRecord, budget: BudgetObservation, *, postcommit: bool,
    ) -> None:
        requested_at = self.utc_now()
        trigger_identity = None
        if postcommit:
            trigger_identity = _revocation_trigger_identity(
                request.data["request_id"], self.authorization.logical_identity,
                budget.revision, requested_at,
            )
        record = {
            "schema_version": AUTHORITY_REVOCATION_SCHEMA,
            "revocation_id": request.data["request_id"],
            "authorization_identity": self.authorization.logical_identity,
            "budget_revision": budget.revision,
            "state": "TRIGGERED_POSTCOMMIT" if postcommit else "REQUESTED_PRECOMMIT",
            "requested_at_utc": requested_at,
            "requested_by_uid": 0,
            "trigger_identity": trigger_identity,
            "processed_trigger_identity": None,
            "termination_receipt_identity": None,
        }
        payload = canonical_authority_record_bytes(record, expected_schema=AUTHORITY_REVOCATION_SCHEMA)
        relative = REVOCATION_PENDING_NAME if postcommit else "revocation/records"
        _write_private_revision(
            self.budget._root_fd, relative, request.data["request_id"] + ".json", payload,
        )

    def _authorization_revoked(self) -> bool:
        directory = _open_nested_directory(self.budget._root_fd, "revocation/records")
        try:
            for name in sorted(os.listdir(directory)):
                if not name.endswith(".json"):
                    _fail("revocation_inventory", "revocation directory contains an unexpected member")
                descriptor = os.open(
                    name, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=directory,
                )
                try:
                    record = validate_authority_record(
                        _read_fd(descriptor, MAX_FRAME_BYTES),
                        expected_schema=AUTHORITY_REVOCATION_SCHEMA,
                    )
                finally:
                    os.close(descriptor)
                if record.data["authorization_identity"] == self.authorization.logical_identity:
                    return True
            return False
        finally:
            os.close(directory)

    def _persist_receipt(self, value: dict[str, Any]) -> None:
        builtin = _builtin_authority_value(value)
        if self._v7:
            payload = canonical_privileged_bytes(
                builtin, expected_schema=self._receipt_schema,
            )
            identity = privileged_record_identity(
                builtin, expected_schema=self._receipt_schema,
            )
        else:
            payload = canonical_authority_record_bytes(
                builtin, expected_schema=self._receipt_schema,
            )
            identity = authority_record_identity(
                builtin, expected_schema=self._receipt_schema,
            )
        _write_private_revision(
            self.budget._root_fd, RECEIPT_DIRECTORY_NAME, identity + ".json", payload,
        )

    def _receipt(
        self,
        request: Slice7GAuthorityRecord,
        result: str,
        budget: BudgetObservation,
        *,
        prepare_token: str | None = None,
        committed_at: str | None = None,
        authorization_identity: str | None = None,
        campaign_id: str | None = None,
        campaign_identity: str | None = None,
        output_root_path: str | None = None,
        output_root_identity: str | None = None,
        observation: ObservationSession | None = None,
        prepare_expires_monotonic_ns: int | None = None,
        postcommit_observer_count: int | None = None,
        candidate_clear: bool | None = None,
    ) -> MappingProxyType:
        previous = budget.revision - 1 if result in {"COMMITTED", "COMPLETED", "FAILED_AFTER_COMMIT"} else budget.revision
        data = {
            "schema_version": self._receipt_schema,
            "method": request.data["method"],
            "request_id": request.data["request_id"],
            "result": result,
            "authorization_identity": authorization_identity or request.data["authorization_identity"],
            "service_instance_identity": self.service_instance_identity,
            "service_nonce": observation.nonce if observation is not None else None,
            "prepare_token": prepare_token,
            "previous_budget_revision": previous,
            "budget_revision": budget.revision,
            "budget_identity": budget.record.logical_identity,
            "campaign_id": campaign_id or request.data["campaign_id"],
            "campaign_identity": campaign_identity or request.data["campaign_identity"],
            "campaign_template_identity": request.data["campaign_template_identity"] or self.authorization.data["campaign"]["plan_identity"],
            "domain_id": request.data["domain_id"],
            "output_root_path": output_root_path or request.data["output_root_path"],
            "output_root_identity": output_root_identity or request.data["output_root_identity"],
            "process_manifest_identity": request.data["process_manifest_identity"],
            "process_instance_identity": request.data["process_instance_identity"],
            "observation_session_identity": (
                observation.identity if observation is not None
                else request.data["observation_session_identity"]
            ),
            "observation_session_nonce": (
                observation.nonce if observation is not None
                else request.data["observation_session_nonce"]
            ),
            "observation_session_deadline_monotonic_ns": (
                observation.deadline_monotonic_ns if observation is not None else None
            ),
            "four_source_observation_identity": (
                observation.four_source_observation_identity if observation is not None
                else None
            ),
            "precommit_receipt_identities": (
                list(observation.receipt_identities) if observation is not None
                else []
            ),
            "precommit_observer_count": (
                len(observation.receipt_identities) if observation is not None
                else 0
            ),
            "postcommit_observer_count": (
                0 if postcommit_observer_count is None else postcommit_observer_count
            ),
            "transaction_observer_count": (
                (len(observation.receipt_identities) if observation is not None else 0)
                + (0 if postcommit_observer_count is None else postcommit_observer_count)
            ),
            "lease_identity": (
                observation.lease_identity if observation is not None else None
            ),
            "prepare_expires_monotonic_ns": prepare_expires_monotonic_ns,
            "committed_at_utc": committed_at,
            "candidate_clear": candidate_clear,
            "error_code": None,
        }
        if self._v7:
            data.update({
                "cleanup_head_identity": (
                    observation.cleanup_head_identity if observation is not None
                    else budget.record.data.get("cleanup_head_identity")
                ),
                "containment_receipt_identity": (
                    observation.containment_receipt_identity if observation is not None
                    else budget.record.data.get("containment_receipt_identity")
                ),
            })
        return self._validate_runtime_record(data, self._receipt_schema).data


class Slice7GAuthorityDaemon:
    """Production fixed-locator AF_UNIX daemon assembly."""

    def __init__(self) -> None:
        bootstrap = _load_v7_production_bootstrap()
        if os.geteuid() != bootstrap.data["authority_uid"] or os.getegid() != bootstrap.data["authority_gid"]:
            _fail("daemon_credentials", "daemon numeric UID/GID differs from bootstrap")
        state_root = bootstrap.data["authority_state_root"]
        budget = GlobalAttemptBudgetStore(_schema=GLOBAL_ATTEMPT_BUDGET_V4_SCHEMA)
        cleanup_client = CleanupAuthorityRPCClient()
        observer_client = ObserverSupervisorRPCClient()
        cleanup_guard = PrivilegedCleanupGuardView(cleanup_client)
        lease_observer = GlobalLeaseStateObserver(
            expected_owner_uid=bootstrap.data["authority_uid"],
        )
        authorization = _read_named_privileged_record(
            budget._root_fd, state_root,
            bootstrap.data["record_paths"]["runtime_authorization"],
            RUNTIME_AUTHORIZATION_V3_SCHEMA,
            expected_owner_uid=bootstrap.data["authority_uid"],
        )
        installed = _read_named_privileged_record(
            budget._root_fd, state_root,
            bootstrap.data["record_paths"]["installed_runtime_manifest"],
            INSTALLED_RUNTIME_MANIFEST_V3_SCHEMA,
            expected_owner_uid=bootstrap.data["authority_uid"],
        )
        process = _read_named_privileged_record(
            budget._root_fd, state_root,
            bootstrap.data["record_paths"]["process_manifest"], PROCESS_MANIFEST_V2_SCHEMA,
            expected_owner_uid=bootstrap.data["authority_uid"],
        )
        privileged_services = _read_named_privileged_record(
            budget._root_fd, state_root,
            bootstrap.data["record_paths"]["privileged_service_manifest"],
            PRIVILEGED_SERVICE_MANIFEST_SCHEMA,
            expected_owner_uid=bootstrap.data["authority_uid"],
        )
        environment = _read_named_record(
            budget._root_fd, state_root,
            bootstrap.data["record_paths"]["environment_manifest"],
            ENVIRONMENT_MANIFEST_SCHEMA,
            expected_owner_uid=bootstrap.data["authority_uid"],
        )
        approval = _read_named_record(
            budget._root_fd, state_root,
            bootstrap.data["record_paths"]["build_test_approval"],
            BUILD_TEST_APPROVAL_SCHEMA,
            expected_owner_uid=bootstrap.data["authority_uid"],
        )
        authenticate_installed_runtime(json.loads(installed.canonical_bytes))
        for executable in (
            process.data["interpreter"], process.data["entrypoint"], *process.data["executables"],
        ):
            authenticate_file_identity(
                dict(executable), expected_mode=executable["mode"],
                expected_owner_uid=0, expected_owner_gid=executable["owner_gid"],
            )
        if installed.data["installed_runtime_identity"] != authorization.data["installed_runtime_identity"]:
            _fail("installed_runtime_binding", "installed runtime and authorization differ")
        if not (
            privileged_services.logical_identity
            == installed.data["privileged_service_manifest_identity"]
            == process.data["privileged_service_manifest_identity"]
            == authorization.data["privileged_service_manifest_identity"]
        ):
            _fail("privileged_service_binding", "privileged-service manifest bindings differ")
        if (
            approval.logical_identity != authorization.data["build_test_approval_identity"]
            or approval.logical_identity != installed.data["build_test_approval_identity"]
            or approval.data["installed_runtime_proposal_identity"] != installed.data["installed_runtime_identity"]
        ):
            _fail("build_approval_binding", "build approval identity differs")
        if not (
            approval.data["source_snapshot"] == authorization.data["source_snapshot"]
            == installed.data["source_snapshot"]
        ):
            _fail("source_snapshot_binding", "source snapshot bindings differ")
        for field in ("branch", "head", "tracked_diff_sha256"):
            if approval.data[field] != authorization.data[field]:
                _fail("build_approval_binding", f"build approval {field} differs")
        if (
            approval.data["applicable_test_nodes"] != authorization.data["applicable_test_nodes"]
            or approval.data["node_id_sha256"] != authorization.data["node_id_sha256"]
            or approval.data["git_command_manifest_sha256"]
            != authorization.data["git_command_manifest_sha256"]
            or approval.data["tests_passed"] != approval.data["applicable_test_nodes"]
        ):
            _fail("build_test_binding", "build/test node or Git authority differs")
        _authenticate_bound_canonical_json(dict(authorization.data["source_snapshot"]), member_count=True)
        _authenticate_bound_canonical_json(dict(authorization.data["charter"]), member_count=False)
        if process.logical_identity != authorization.data["process_manifest_identity"]:
            _fail("process_manifest_binding", "process manifest and authorization differ")
        if environment.logical_identity != authorization.data["environment_manifest_identity"]:
            _fail("environment_manifest_binding", "environment manifest and authorization differ")
        if environment.logical_identity != process.data["environment_manifest_identity"]:
            _fail("process_environment_binding", "process and environment manifest differ")
        self.bootstrap = bootstrap
        self.installed = installed
        self.process = process
        self.environment = environment
        self.privileged_services = privileged_services
        self.cleanup_guard = cleanup_guard
        self.cleanup_client = cleanup_client
        self.observer_client = observer_client
        self.lease_observer = lease_observer
        self.output_provisioner = AuthorityOutputProvisioner(
            authority_uid=bootstrap.data["authority_uid"],
            runtime_gid=bootstrap.data["runtime_gid"],
            campaign_uid=bootstrap.data["campaign_uid"],
        )
        self.state = RuntimeAuthorityStateMachine._production(
            bootstrap=json.loads(bootstrap.canonical_bytes),
            authorization=json.loads(authorization.canonical_bytes),
            budget=budget, cleanup_guard=cleanup_guard,
            service_instance_identity=hashlib.sha256(secrets.token_bytes(32)).hexdigest(),
            peer_matcher=self._match_peer,
            provisioner=self.output_provisioner.allocate,
            process_instance_validator=self._validate_process_instance,
            observation_provider=self._observe_domain,
        )
        self.state.recover_abandoned_commit()
        self._socket: socket.socket | None = None

    def serve_forever(self) -> None:
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._socket = listener
        try:
            listener.bind(AUTHORITY_SOCKET_PATH)
            os.chown(AUTHORITY_SOCKET_PATH, self.bootstrap.data["authority_uid"], self.bootstrap.data["runtime_gid"])
            os.chmod(AUTHORITY_SOCKET_PATH, 0o660)
            listener.listen(8)
            while True:
                channel, _ = listener.accept()
                try:
                    self._serve_channel(channel)
                except (
                    Slice7GAuthorityProtocolError,
                    Slice7GPrivilegedProtocolError,
                    Slice7GAuthorityDaemonError,
                ):
                    # Malformed or disconnected clients lose only their own
                    # bounded session; they cannot terminate the authority
                    # service or change the durable budget.
                    continue
        finally:
            listener.close()
            self.state.budget.close()
            self.cleanup_guard.close()
            self.observer_client.close()
            self.lease_observer.close()
            self.output_provisioner.close()

    def _serve_channel(self, channel: socket.socket) -> None:
        connection = "c" + secrets.token_hex(16)
        try:
            credentials = peer_credentials(channel)
            peer = observe_peer_process(credentials)
            for _ in range(MAX_SESSION_REQUESTS):
                if _clean_eof(channel):
                    return
                try:
                    request = _receive_v7_runtime_frame(channel)
                except BaseException:
                    self.state.invalidate_connection_observations(
                        connection, cleanup_uncertain=False,
                    )
                    raise
                try:
                    response = self.state.handle(dict(request.data), peer, connection)
                except (
                    Slice7GAuthorityDaemonError, Slice7GAuthorityProtocolError,
                    Slice7GPrivilegedProtocolError,
                ) as exc:
                    response = _error_receipt(request, self.state.service_instance_identity, exc.code)
                _send_v7_runtime_frame(channel, dict(response))
            if not _clean_eof(channel):
                _fail("authority_session_bound", "authority session request bound exceeded")
        finally:
            self.state.disconnect(connection)
            channel.close()

    def _match_peer(self, peer: Slice7GPeerProcess, request: Slice7GAuthorityRecord) -> None:
        authorization = self.state.authorization.data
        entrypoint = self.process.data["entrypoint"]
        interpreter = self.process.data["interpreter"]
        if entrypoint["sha256"] != authorization["entrypoint_identity"]:
            _fail("peer_entrypoint_binding", "entrypoint and authorization identity differ")
        if peer.executable != interpreter["path"]:
            _fail("peer_interpreter", "peer interpreter path differs")
        _authenticate_peer_executable(peer, dict(interpreter))
        authenticate_file_identity(
            dict(entrypoint), expected_mode=entrypoint["mode"],
            expected_owner_uid=0, expected_owner_gid=entrypoint["owner_gid"],
        )
        if tuple(peer.argv) != tuple(self.process.data["argv_template"]):
            _fail("peer_argv", "peer argv differs from the closed coordinator invocation")
        if peer.working_directory != self.process.data["working_directory"]:
            _fail("peer_cwd", "peer working directory differs")
        if peer.cgroup != self.process.data["cgroup"]:
            _fail("peer_cgroup", "peer is outside the campaign cgroup")
        environment = dict(peer.environment)
        expected = dict(self.environment.data["fixed_values"])
        if self.environment.data["transaction_values"]:
            _fail("peer_environment_manifest", "coordinator environment cannot contain transaction slots")
        if environment != expected:
            _fail("peer_environment", "peer environment differs from the closed manifest")
        installed_root = self.installed.data["root_path"]
        entrypoint_index = 1 + len(self.process.data["interpreter_flags"])
        if (
            len(peer.argv) <= entrypoint_index
            or peer.argv[entrypoint_index] != entrypoint["path"]
            or not peer.argv[entrypoint_index].startswith(installed_root + "/")
        ):
            _fail("peer_installed_origin", "coordinator script does not originate in installed runtime")

    def _validate_process_instance(
        self,
        prepared: PreparedCampaign,
        allocation: ProvisionalAllocation,
        peer: Slice7GPeerProcess,
        supplied_identity: str,
    ) -> None:
        projection = {
            "authorization_identity": prepared.authorization_identity,
            "campaign_id": prepared.campaign_id,
            "campaign_identity": prepared.campaign_identity,
            "cgroup": self.process.data["cgroup"],
            "domain_id": allocation.domain_id,
            "environment_manifest_identity": self.environment.logical_identity,
            "executable": dict(self.process.data["entrypoint"]),
            "output_root_identity": allocation.output_root_identity,
            "output_root_path": allocation.output_root_path,
            "process_manifest_identity": self.process.logical_identity,
            "schema_version": "ctr-slice-7g-process-instance-1",
            "systemd_unit": self.process.data["systemd_unit"],
            "working_directory": self.process.data["working_directory"],
        }
        payload = json.dumps(
            projection, ensure_ascii=False, allow_nan=False, sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        expected = hashlib.sha256(
            b"ctr-slice-7g-process-instance-canonical-1\0" + payload,
        ).hexdigest()
        if supplied_identity != expected:
            _fail("process_instance_identity", "process instance identity differs")

    def _observe_domain(
        self,
        session: ObservationSession,
        phase: str,
        domain: int,
        phase_ordinal: int,
        transaction_ordinal: int,
        peer: Slice7GPeerProcess,
    ) -> DaemonObservationEvidence:
        """Run all four fixed providers and return raw daemon-owned evidence."""

        self.state.peer_reconciler(peer)
        ros2_matches = [
            dict(item) for item in self.process.data["executables"]
            if item["path"] == "/opt/ros/humble/bin/ros2"
        ]
        if len(ros2_matches) != 1:
            _fail("observer_executable_origin", "ros2 executable is not uniquely manifest-bound")
        ros2 = ros2_matches[0]
        authenticate_file_identity(
            ros2, expected_mode=ros2["mode"], expected_owner_uid=0,
            expected_owner_gid=ros2["owner_gid"],
        )
        interpreter = dict(self.process.data["interpreter"])
        authenticate_file_identity(
            interpreter, expected_mode=interpreter["mode"], expected_owner_uid=0,
            expected_owner_gid=interpreter["owner_gid"],
        )
        fixed = dict(self.environment.data["fixed_values"])
        generated = dict(self.environment.data["transaction_values"])
        if generated.get("ROS_DOMAIN_ID") != "domain_id":
            _fail("observer_environment", "ROS_DOMAIN_ID is not a manifest-bound transaction slot")
        environment = {**fixed, "ROS_DOMAIN_ID": str(domain)}
        if set(environment) != set(self.environment.data["required_keys"]):
            _fail("observer_environment", "observer environment key set differs")
        active_identity, active_clear = _observe_active_process_source(domain)
        dds_identity, dds_clear = _observe_dds_port_source(domain)
        observation_interval_identity = _domain_identity(
            b"ctr-slice-7g-v7-observation-interval-authority-canonical-1\0",
            {
                "session_binding_identity": session.identity,
                "phase": phase,
                "phase_local_ordinal": phase_ordinal,
                "transaction_observer_ordinal": transaction_ordinal,
                "domain_id": domain,
            },
        )
        lease = self.lease_observer.observe(
            domain, time.monotonic_ns(),
            session_binding_identity=session.identity,
            service_nonce=session.nonce,
            phase=phase,
            phase_local_ordinal=phase_ordinal,
            transaction_observer_ordinal=transaction_ordinal,
            observation_interval_identity=observation_interval_identity,
        )
        if not active_clear or not dds_clear or not lease.clear:
            _fail(
                "candidate_occupied_non_ros",
                "candidate is occupied by an active process, DDS port, or global lease",
            )
        executable_identity = _domain_identity(
            b"ctr-slice-7g-file-identity-canonical-1\0", ros2,
        )
        argv = (ros2["path"], "node", "list", "--no-daemon")
        cleanup = self.cleanup_guard.require_clear()
        execution = self.observer_client.observe(
            runtime_authorization_identity=session.authorization_identity,
            installed_runtime_identity=session.installed_runtime_identity,
            budget_identity=self.state.budget.observe().record.logical_identity,
            cleanup_head_identity=cleanup.head.logical_identity,
            session_binding_identity=session.identity,
            domain_id=domain,
            phase=phase,
            phase_local_ordinal=phase_ordinal,
            transaction_observer_ordinal=transaction_ordinal,
            privileged_service_generation_identity=None,
        )
        nodes = _parse_server_ros_nodes(execution["stdout"])
        graph_identity = _domain_identity(
            b"ctr-slice-7g-ros-graph-provider-canonical-1\0",
            {
                "domain_id": domain,
                "nodes": list(nodes),
                "stderr_sha256": hashlib.sha256(execution["stderr"]).hexdigest(),
                "stdout_sha256": hashlib.sha256(execution["stdout"]).hexdigest(),
            },
        )
        module_origins = tuple(sorted(
            _domain_identity(b"ctr-slice-7g-python-module-origin-canonical-1\0", dict(item))
            for item in self.installed.data["python_modules"]
            if item["module"].split(".", 1)[0] in {"ros2cli", "ros2node", "rclpy"}
        ))
        if not module_origins:
            _fail("observer_module_origins", "installed ROS CLI module origins are unavailable")
        peer_identity = _domain_identity(
            b"ctr-slice-7g-peer-process-canonical-1\0",
            {
                "cgroup": peer.cgroup,
                "executable": peer.executable,
                "pid": peer.credentials.pid,
                "start_time_ticks": peer.start_time_ticks,
            },
        )
        interval_identity = observation_interval_identity
        containment = execution["containment_receipt"]
        return DaemonObservationEvidence(
            active_process_identity=active_identity,
            active_process_clear=active_clear,
            dds_port_identity=dds_identity,
            dds_port_clear=dds_clear,
            global_lease_identity=lease.record.logical_identity,
            global_lease_registry_identity=lease.record.data["registry_identity"],
            global_lease_revision_identity=lease.record.data["registry_revision_identity"],
            global_lease_state=lease.record.data["state"],
            global_lease_clear=lease.record.data["clear"],
            peer_process_identity=peer_identity,
            observation_interval_identity=interval_identity,
            graph_provider_identity=graph_identity,
            executable=ros2["path"], executable_identity=executable_identity,
            interpreter=interpreter["path"],
            interpreter_identity=_domain_identity(
                b"ctr-slice-7g-file-identity-canonical-1\0", interpreter,
            ),
            module_origin_identities=module_origins, argv=argv,
            environment_identity=self.environment.logical_identity,
            working_directory=self.process.data["working_directory"],
            cgroup=containment.data["leaf_cgroup"], pid=execution["pid"],
            process_group_id=execution["pgid"],
            process_start_time_ticks=execution["start_time_ticks"],
            started_monotonic_ns=execution["started_monotonic_ns"],
            ended_monotonic_ns=execution["ended_monotonic_ns"],
            exit_status=execution["exit_status"], terminating_signal=None,
            stdout=execution["stdout"], stderr=execution["stderr"], nodes=nodes,
            cleanup_barrier_identity=execution["cleanup_barrier_identity"],
            unexpected_descendants=0, ros_daemon_started=False,
            observed_monotonic_ns=execution["ended_monotonic_ns"],
            cleanup_head_identity=execution["cleanup_head_identity"],
            containment_receipt_identity=containment.logical_identity,
        )

def enforce_pending_revocations() -> int:
    """Root-only fixed-purpose helper for the root-owned revocation unit."""

    if os.geteuid() != 0:
        _fail("revocation_root", "revocation enforcement requires UID 0")
    bootstrap = load_production_bootstrap()
    root_fd = _open_directory_path(bootstrap.data["state_root"])
    try:
        root_info = os.fstat(root_fd)
        if (
            root_info.st_uid != bootstrap.data["authority_uid"]
            or stat.S_IMODE(root_info.st_mode) != 0o700
        ):
            _fail("authority_state_identity", "authority state root ownership or mode differs")
        process = _read_named_record(
            root_fd, bootstrap.data["state_root"],
            bootstrap.data["record_paths"]["process_manifest"], PROCESS_MANIFEST_SCHEMA,
            expected_owner_uid=bootstrap.data["authority_uid"],
        )
        return _enforce_pending_revocations(
            root_fd, bootstrap.data["authority_uid"], process, subprocess.run, _utc_now,
        )
    finally:
        os.close(root_fd)


def _enforce_pending_revocations_for_test(
    state_root: str,
    authority_uid: int,
    process_manifest: dict[str, Any],
    runner: Callable[..., Any],
    utc_now: Callable[[], str],
) -> int:
    """Private synthetic provider seam; it is unreachable from the CLI."""

    root_fd = _open_directory_path(_absolute_path(state_root))
    try:
        process = validate_authority_record(
            process_manifest, expected_schema=PROCESS_MANIFEST_SCHEMA,
        )
        return _enforce_pending_revocations(root_fd, authority_uid, process, runner, utc_now)
    finally:
        os.close(root_fd)


def _enforce_pending_revocations(
    root_fd: int,
    authority_uid: int,
    process: Slice7GAuthorityRecord,
    runner: Callable[..., Any],
    utc_now: Callable[[], str],
) -> int:
    pending_fd = _open_nested_directory(root_fd, REVOCATION_PENDING_NAME)
    processed_fd = _open_nested_directory(root_fd, REVOCATION_PROCESSED_NAME)
    try:
        names = sorted(os.listdir(pending_fd))
        triggers: list[tuple[str, Slice7GAuthorityRecord]] = []
        for name in names:
            if not name.endswith(".json") or "/" in name:
                _fail("revocation_inventory", "pending revocation inventory differs")
            descriptor = os.open(
                name, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=pending_fd,
            )
            try:
                before = os.fstat(descriptor)
                if (
                    not stat.S_ISREG(before.st_mode)
                    or before.st_nlink != 1
                    or before.st_uid != authority_uid
                    or stat.S_IMODE(before.st_mode) != 0o600
                ):
                    _fail("revocation_trigger_identity", "revocation trigger identity differs")
                record = validate_authority_record(
                    _read_fd(descriptor, MAX_FRAME_BYTES),
                    expected_schema=AUTHORITY_REVOCATION_SCHEMA,
                )
                if _file_identity(before) != _file_identity(os.fstat(descriptor)):
                    _fail("revocation_trigger_changed", "revocation trigger changed during read")
            finally:
                os.close(descriptor)
            if record.data["state"] != "TRIGGERED_POSTCOMMIT":
                _fail("revocation_trigger_state", "pending revocation is not post-commit")
            expected_trigger = _revocation_trigger_identity(
                record.data["revocation_id"], record.data["authorization_identity"],
                record.data["budget_revision"], record.data["requested_at_utc"],
            )
            if record.data["trigger_identity"] != expected_trigger:
                _fail("revocation_trigger_identity", "revocation trigger binding differs")
            triggers.append((name, record))
        if not triggers:
            return 0
        timeouts = process.data["timeouts"]
        stop_timeout = sum(float(timeouts[key]) for key in (
            "sigint_seconds", "sigterm_seconds", "sigkill_seconds",
        ))
        completed = runner(
            [SYSTEMCTL_PATH, "stop", CAMPAIGN_UNIT],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            shell=False, check=False, timeout=stop_timeout,
            env={"PATH": "/usr/bin:/bin", "SYSTEMD_PAGER": "cat"}, cwd="/",
        )
        if type(completed.returncode) is not int or completed.returncode != 0:
            _fail("revocation_stop", "fixed campaign unit stop failed")
        stdout = _exact_completed_stream(completed.stdout, "stdout")
        stderr = _exact_completed_stream(completed.stderr, "stderr")
        terminated_at = utc_now()
        _parse_utc(terminated_at)
        for name, trigger in triggers:
            termination_projection = {
                "argv": [SYSTEMCTL_PATH, "stop", CAMPAIGN_UNIT],
                "exit_status": completed.returncode,
                "processed_trigger_identity": trigger.logical_identity,
                "stderr_sha256": hashlib.sha256(stderr).hexdigest(),
                "stdout_sha256": hashlib.sha256(stdout).hexdigest(),
                "terminated_at_utc": terminated_at,
            }
            termination_identity = _domain_identity(
                b"ctr-slice-7g-revocation-termination-receipt-canonical-1\0",
                termination_projection,
            )
            enforced = dict(trigger.data)
            enforced.update({
                "state": "ENFORCED_POSTCOMMIT",
                "processed_trigger_identity": trigger.logical_identity,
                "termination_receipt_identity": termination_identity,
            })
            payload = canonical_authority_record_bytes(
                enforced, expected_schema=AUTHORITY_REVOCATION_SCHEMA,
            )
            _write_private_revision_fd(processed_fd, name, payload)
            current = os.stat(name, dir_fd=pending_fd, follow_symlinks=False)
            if current.st_nlink != 1 or current.st_uid != authority_uid:
                _fail("revocation_trigger_replaced", "revocation trigger changed before removal")
            os.unlink(name, dir_fd=pending_fd)
            os.fsync(pending_fd)
        return len(triggers)
    finally:
        os.close(processed_fd)
        os.close(pending_fd)


def main(argv: list[str] | None = None) -> int:
    arguments = [] if argv is None else argv
    if type(arguments) is not list or any(type(item) is not str for item in arguments):
        return 2
    if arguments == ["--enforce-revocation"]:
        try:
            enforce_pending_revocations()
        except (Slice7GAuthorityDaemonError, Slice7GAuthorityProtocolError):
            return 2
        return 0
    if arguments:
        return 2
    try:
        Slice7GAuthorityDaemon().serve_forever()
    except (
        Slice7GAuthorityDaemonError, Slice7GAuthorityProtocolError,
        Slice7GCleanupAuthorityError, Slice7GObserverSupervisorError,
        Slice7GPrivilegedProtocolError, OSError,
    ):
        return 2
    return 0


class _locked:
    def __init__(self, descriptor: int) -> None:
        self.descriptor = descriptor

    def __enter__(self) -> None:
        fcntl.flock(self.descriptor, fcntl.LOCK_EX)

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        fcntl.flock(self.descriptor, fcntl.LOCK_UN)


def _read_named_record(
    root_fd: int,
    root: str,
    path: str,
    schema: str,
    *,
    expected_owner_uid: int | None = None,
) -> Slice7GAuthorityRecord:
    root_path = PurePosixPath(root)
    candidate = PurePosixPath(path)
    if candidate == root_path or root_path not in candidate.parents:
        _fail("authority_record_path", "authority record escapes fixed state root")
    relative = candidate.relative_to(root_path)
    if len(relative.parts) < 2:
        _fail("authority_record_path", "authority record must be beneath a confined state subdirectory")
    parent = _open_nested_directory(root_fd, PurePosixPath(*relative.parts[:-1]).as_posix())
    descriptor: int | None = None
    try:
        descriptor = os.open(
            relative.parts[-1], os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW,
            dir_fd=parent,
        )
        info = os.fstat(descriptor)
        by_name = os.stat(relative.parts[-1], dir_fd=parent, follow_symlinks=False)
        owner_uid = os.geteuid() if expected_owner_uid is None else expected_owner_uid
        if (
            not stat.S_ISREG(info.st_mode)
            or info.st_nlink != 1
            or info.st_uid != owner_uid
            or stat.S_IMODE(info.st_mode) not in {0o440, 0o600}
            or _file_identity(info) != _file_identity(by_name)
        ):
            _fail("authority_record_identity", "authority record physical identity differs")
        raw = _read_fd(descriptor, MAX_FRAME_BYTES)
        if _file_identity(info) != _file_identity(os.fstat(descriptor)):
            _fail("authority_record_changed", "authority record changed during read")
    finally:
        if descriptor is not None:
            os.close(descriptor)
        os.close(parent)
    return validate_authority_record(raw, expected_schema=schema)


def _read_named_privileged_record(
    root_fd: int,
    root: str,
    path: str,
    schema: str,
    *,
    expected_owner_uid: int,
) -> PrivilegedRecord:
    """Read one v7 record without allowing path, inode, or owner substitution."""

    root_path = PurePosixPath(root)
    candidate = PurePosixPath(path)
    if candidate == root_path or root_path not in candidate.parents:
        _fail("authority_record_path", "authority record escapes fixed state root")
    relative = candidate.relative_to(root_path)
    if len(relative.parts) < 2:
        _fail("authority_record_path", "authority record must be confined below a state subdirectory")
    parent = _open_nested_directory(
        root_fd, PurePosixPath(*relative.parts[:-1]).as_posix(),
    )
    descriptor: int | None = None
    try:
        descriptor = os.open(
            relative.parts[-1], os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW,
            dir_fd=parent,
        )
        before = os.fstat(descriptor)
        by_name = os.stat(relative.parts[-1], dir_fd=parent, follow_symlinks=False)
        if (
            not stat.S_ISREG(before.st_mode)
            or before.st_nlink != 1
            or before.st_uid != expected_owner_uid
            or stat.S_IMODE(before.st_mode) not in {0o440, 0o600}
            or _file_identity(before) != _file_identity(by_name)
        ):
            _fail("authority_record_identity", "v7 authority record physical identity differs")
        raw = _read_fd(descriptor, MAX_FRAME_BYTES)
        after = os.fstat(descriptor)
        final_name = os.stat(relative.parts[-1], dir_fd=parent, follow_symlinks=False)
        if _file_identity(before) != _file_identity(after) or _file_identity(before) != _file_identity(final_name):
            _fail("authority_record_changed", "v7 authority record changed during authentication")
        return validate_privileged_record(raw, expected_schema=schema)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        os.close(parent)


def _load_v7_production_bootstrap() -> PrivilegedRecord:
    """Authenticate the fixed root-owned bootstrap without creating anything."""

    path = "/etc/ctr-mppi/slice-7g-authority/bootstrap.json"
    descriptor = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        before = os.fstat(descriptor)
        by_name = os.stat(path, follow_symlinks=False)
        if (
            not stat.S_ISREG(before.st_mode)
            or before.st_uid != 0
            or before.st_gid != 0
            or before.st_nlink != 1
            or stat.S_IMODE(before.st_mode) not in {0o400, 0o440, 0o444}
            or _file_identity(before) != _file_identity(by_name)
        ):
            _fail("bootstrap_identity", "v7 bootstrap physical identity differs")
        raw = _read_fd(descriptor, MAX_FRAME_BYTES)
        if _file_identity(before) != _file_identity(os.fstat(descriptor)):
            _fail("bootstrap_changed", "v7 bootstrap changed during authentication")
        return validate_privileged_record(raw, expected_schema=AUTHORITY_BOOTSTRAP_V3_SCHEMA)
    finally:
        os.close(descriptor)


def _write_private_revision(root_fd: int, relative_directory: str, name: str, payload: bytes) -> None:
    current = _open_nested_directory(root_fd, relative_directory)
    try:
        _write_private_revision_fd(current, name, payload)
    finally:
        os.close(current)


def _read_optional_private_revision(
    root_fd: int,
    relative_directory: str,
    name: str,
    schema: str,
) -> Slice7GAuthorityRecord | None:
    directory = _open_nested_directory(root_fd, relative_directory)
    descriptor: int | None = None
    try:
        try:
            descriptor = os.open(
                name, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW,
                dir_fd=directory,
            )
        except FileNotFoundError:
            return None
        before = os.fstat(descriptor)
        by_name = os.stat(name, dir_fd=directory, follow_symlinks=False)
        if (
            not stat.S_ISREG(before.st_mode)
            or before.st_nlink != 1
            or stat.S_IMODE(before.st_mode) != 0o600
            or _file_identity(before) != _file_identity(by_name)
        ):
            _fail("authority_revision_identity", "authority revision identity differs")
        raw = _read_fd(descriptor, MAX_FRAME_BYTES)
        if _file_identity(before) != _file_identity(os.fstat(descriptor)):
            _fail("authority_revision_changed", "authority revision changed during read")
        return validate_authority_record(raw, expected_schema=schema)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        os.close(directory)


def _write_private_revision_fd(directory_fd: int, name: str, payload: bytes) -> None:
    if type(name) is not str or not name or "/" in name or name in {".", ".."}:
        _fail("authority_path", "authority revision name is unsafe")
    descriptor = os.open(
        name, os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_CLOEXEC | os.O_NOFOLLOW,
        0o600, dir_fd=directory_fd,
    )
    try:
        _write_all(descriptor, payload)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    os.fsync(directory_fd)


def _domain_identity(domain: bytes, value: dict[str, Any]) -> str:
    if type(domain) is not bytes or type(value) is not dict:
        _fail("authority_identity", "identity inputs must use exact built-in types")
    payload = json.dumps(
        value, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(domain + payload).hexdigest()


def _revocation_trigger_identity(
    revocation_id: str,
    authorization_identity: str,
    budget_revision: int,
    requested_at_utc: str,
) -> str:
    return _domain_identity(
        b"ctr-slice-7g-revocation-trigger-canonical-1\0",
        {
            "authorization_identity": authorization_identity,
            "budget_revision": budget_revision,
            "requested_at_utc": requested_at_utc,
            "revocation_id": revocation_id,
            "schema_version": "ctr-slice-7g-revocation-trigger-1",
        },
    )


def _exact_completed_stream(value: Any, name: str) -> bytes:
    if type(value) is not bytes:
        _fail("revocation_stop_stream", f"systemd stop {name} must be exact bytes")
    if len(value) > MAX_FRAME_BYTES:
        _fail("revocation_stop_stream", f"systemd stop {name} exceeds its bound")
    return value


def _authenticate_peer_executable(
    peer: Slice7GPeerProcess,
    identity: dict[str, Any],
) -> None:
    descriptor: int | None = None
    try:
        descriptor = os.open(
            f"/proc/{peer.credentials.pid}/exe",
            os.O_RDONLY | os.O_CLOEXEC,
        )
        before = os.fstat(descriptor)
        expected = (
            identity["device"], identity["inode"], identity["mode"],
            identity["link_count"], identity["size"], identity["owner_uid"],
            identity["owner_gid"],
        )
        observed = (
            before.st_dev, before.st_ino, stat.S_IMODE(before.st_mode),
            before.st_nlink, before.st_size, before.st_uid, before.st_gid,
        )
        if observed != expected or not stat.S_ISREG(before.st_mode):
            _fail("peer_executable_identity", "peer executable physical identity differs")
        digest = hashlib.sha256()
        while True:
            chunk = os.read(descriptor, 1_048_576)
            if not chunk:
                break
            digest.update(chunk)
        after = os.fstat(descriptor)
        after_observed = (
            after.st_dev, after.st_ino, stat.S_IMODE(after.st_mode),
            after.st_nlink, after.st_size, after.st_uid, after.st_gid,
        )
        if after_observed != expected or digest.hexdigest() != identity["sha256"]:
            _fail("peer_executable_identity", "peer executable changed or digest differs")
    except OSError as exc:
        raise Slice7GAuthorityDaemonError(
            "peer_executable_identity", f"peer executable authentication failed: {type(exc).__name__}",
        ) from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)


def _raise_cleanup_errors(errors: list[BaseException], context: str) -> None:
    if not errors:
        return
    primary = errors[0]
    if len(errors) > 1:
        try:
            primary.add_note(
                f"Slice 7G {context} secondary errors: "
                f"{[type(item).__name__ for item in errors[1:]]!r}"
            )
        except (AttributeError, TypeError):
            pass
    if isinstance(primary, Exception) and not isinstance(
        primary, (Slice7GAuthorityDaemonError, Slice7GAuthorityProtocolError),
    ):
        raise Slice7GAuthorityDaemonError(
            "authority_cleanup", f"{context} failed: {type(primary).__name__}",
        ) from primary
    raise primary


def _add_cleanup_note(primary: BaseException, secondary: BaseException, context: str) -> None:
    try:
        primary.add_note(f"Slice 7G {context}: {type(secondary).__name__}")
    except (AttributeError, TypeError):
        pass


def _authenticate_bound_canonical_json(binding: dict[str, Any], *, member_count: bool) -> None:
    if type(binding) is not dict:
        _fail("bound_record_binding", "bound record authority must be an exact dictionary")
    path = _absolute_path(binding["path"])
    descriptor = _open_regular_path_nofollow(path)
    try:
        before = os.fstat(descriptor)
        if (
            not stat.S_ISREG(before.st_mode)
            or before.st_nlink != 1
            or stat.S_IMODE(before.st_mode) & 0o222
            or before.st_size <= 0
        ):
            _fail("bound_record_identity", "bound record is writable, empty, aliased, or non-regular")
        raw = _read_fd(descriptor, MAX_FRAME_BYTES)
        after = os.fstat(descriptor)
        if _file_identity(before) != _file_identity(after):
            _fail("bound_record_changed", "bound record changed during authentication")
    finally:
        os.close(descriptor)
    if hashlib.sha256(raw).hexdigest() != binding["physical_sha256"]:
        _fail("bound_record_physical", "bound record physical digest differs")
    try:
        data = json.loads(
            raw.decode("utf-8", "strict"),
            object_pairs_hook=_reject_duplicate_pairs,
            parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)),
        )
    except (UnicodeError, ValueError, json.JSONDecodeError) as exc:
        raise Slice7GAuthorityDaemonError("bound_record_json", str(exc)) from exc
    if type(data) is not dict or data.get("schema_version") != binding["schema_version"]:
        _fail("bound_record_schema", "bound record schema differs")
    canonical = json.dumps(
        data, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":"),
    ).encode("utf-8")
    if raw != canonical or "logical_identity" in data:
        _fail("bound_record_canonical", "bound record is noncanonical or self-identifying")
    algorithm = binding["logical_identity_algorithm"]
    if type(algorithm) is not str or not algorithm.startswith("sha256:"):
        _fail("bound_record_algorithm", "bound record logical algorithm differs")
    domain = algorithm.removeprefix("sha256:").encode("ascii", "strict") + b"\0"
    if hashlib.sha256(domain + canonical).hexdigest() != binding["logical_identity"]:
        _fail("bound_record_logical", "bound record logical identity differs")
    if member_count:
        members = data.get("members")
        if type(members) is not list or len(members) != binding["member_count"]:
            _fail("bound_record_members", "bound snapshot member count differs")
    reopened = _open_regular_path_nofollow(path)
    try:
        if _file_identity(os.fstat(reopened)) != _file_identity(before):
            _fail("bound_record_replaced", "bound record pathname was replaced")
    finally:
        os.close(reopened)


def _reject_duplicate_pairs(items: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in items:
        if type(key) is not str or key in result:
            raise ValueError("duplicate or non-string JSON key")
        result[key] = value
    return result


def _open_nested_directory(root_fd: int, relative_directory: str) -> int:
    path = PurePosixPath(relative_directory)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        _fail("authority_path", "authority relative directory is unsafe")
    current = os.dup(root_fd)
    try:
        for part in path.parts:
            child = _open_directory_at(current, part)
            os.close(current)
            current = child
        return current
    except BaseException:
        os.close(current)
        raise


def _read_proc_bounded(path: str, maximum: int) -> bytes:
    try:
        with open(path, "rb", buffering=0) as stream:
            raw = stream.read(maximum + 1)
    except OSError:
        raise
    if len(raw) > maximum:
        _fail("proc_record_size", "process record exceeds its bound")
    return raw


def _proc_start_time(root: str) -> int:
    raw = _read_proc_bounded(root + "/stat", 65_536).decode("ascii", "strict")
    close = raw.rfind(")")
    fields = raw[close + 2:].split()
    if close < 0 or len(fields) < 20:
        _fail("observer_process_identity", "process stat record is malformed")
    return int(fields[19])


def _observe_active_process_source(domain: int) -> tuple[str, bool]:
    observations: list[dict[str, Any]] = []
    for name in sorted(item for item in os.listdir("/proc") if item.isdigit()):
        try:
            raw = _read_proc_bounded(f"/proc/{name}/environ", MAX_FRAME_BYTES)
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        marker = b"ROS_DOMAIN_ID=" + str(domain).encode("ascii")
        if marker not in raw.split(b"\0"):
            continue
        try:
            start = _proc_start_time(f"/proc/{name}")
            executable = os.readlink(f"/proc/{name}/exe")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            _fail("active_process_provider", "domain process changed during observation")
        observations.append({"executable": executable, "pid": int(name), "start_time_ticks": start})
    identity = _domain_identity(
        b"ctr-slice-7g-active-process-observation-canonical-1\0",
        {"records": observations},
    )
    return identity, not observations


def _domain_udp_ports(domain: int) -> frozenset[int]:
    base = 7_400 + 250 * domain
    return frozenset({base, base + 1, *(base + 10 + offset for offset in range(0, 241, 2)),
                      *(base + 11 + offset for offset in range(0, 241, 2))})


def _observed_udp_ports(domain: int) -> tuple[int, ...]:
    expected = _domain_udp_ports(domain)
    observed: set[int] = set()
    for path in ("/proc/net/udp", "/proc/net/udp6"):
        try:
            lines = Path(path).read_text(encoding="ascii").splitlines()[1:]
        except OSError as exc:
            raise Slice7GAuthorityDaemonError("dds_port_provider", type(exc).__name__) from exc
        for line in lines:
            fields = line.split()
            if len(fields) < 2 or ":" not in fields[1]:
                _fail("dds_port_provider", "UDP socket inventory is malformed")
            try:
                port = int(fields[1].rsplit(":", 1)[1], 16)
            except ValueError as exc:
                raise Slice7GAuthorityDaemonError("dds_port_provider", "invalid UDP port") from exc
            if port in expected:
                observed.add(port)
    return tuple(sorted(observed))


def _observe_dds_port_source(domain: int) -> tuple[str, bool]:
    ports = _observed_udp_ports(domain)
    identity = _domain_identity(
        b"ctr-slice-7g-dds-port-observation-canonical-1\0", {"ports": list(ports)},
    )
    return identity, not ports


def _server_group_members(
    pgid: int, cgroup: str, minimum_start_ticks: int, session_id: int | None = None,
    *, retained_zombie: tuple[int, int] | None = None,
) -> tuple[tuple[int, int], ...]:
    members: list[tuple[int, int]] = []
    for name in sorted(item for item in os.listdir("/proc") if item.isdigit()):
        root = f"/proc/{name}"
        try:
            raw = _read_proc_bounded(root + "/stat", 65_536).decode("ascii", "strict")
            close = raw.rfind(")")
            fields = raw[close + 2:].split()
            state = fields[0]
            member_group = int(fields[2])
            member_session = int(fields[3])
            start = int(fields[19])
        except (FileNotFoundError, ProcessLookupError):
            continue
        except (IndexError, UnicodeError, ValueError) as exc:
            raise Slice7GAuthorityDaemonError("observer_process_ownership", type(exc).__name__) from exc
        if member_group != pgid:
            continue
        try:
            lines = _read_proc_bounded(root + "/cgroup", 65_536).decode("utf-8", "strict").splitlines()
        except (FileNotFoundError, ProcessLookupError):
            continue
        if (
            lines != ["0::" + cgroup]
            or start < minimum_start_ticks
            or (session_id is not None and member_session != session_id)
        ):
            _fail("observer_process_ownership", "observer PGID was reused or escaped its cgroup")
        if retained_zombie == (int(name), start) and state == "Z":
            continue
        members.append((int(name), start))
    return tuple(members)


def _cleanup_server_observer_group(
    pgid: int, cgroup: str, minimum_start_ticks: int, domain: int,
    baseline_ports: tuple[int, ...], authenticated_members: tuple[tuple[int, int], ...],
    *,
    port_observer: Callable[[int], tuple[int, ...]] = _observed_udp_ports,
    session_id: int | None = None,
    retained_zombie: tuple[int, int] | None = None,
) -> str:
    authenticated = frozenset(authenticated_members)
    if retained_zombie is not None and retained_zombie not in authenticated:
        _fail("observer_process_ownership", "retained leader provenance is absent")

    def owned_members() -> tuple[tuple[int, int], ...]:
        _reap_owned_group_zombies(
            pgid, cgroup, minimum_start_ticks, session_id,
            retained_zombie=retained_zombie,
        )
        members = _server_group_members(
            pgid, cgroup, minimum_start_ticks, session_id,
            retained_zombie=retained_zombie,
        )
        if retained_zombie is None and any(member not in authenticated for member in members):
            _fail(
                "observer_process_ownership",
                "observer PGID contains a process without retained identity provenance",
            )
        return members

    def signal_members(members: tuple[tuple[int, int], ...], sent_signal: int) -> None:
        for pid, expected_start in members:
            current = _server_group_members(
                pgid, cgroup, minimum_start_ticks, session_id,
                retained_zombie=retained_zombie,
            )
            if (pid, expected_start) not in current:
                continue
            try:
                os.kill(pid, sent_signal)
            except ProcessLookupError:
                pass

    deadline = time.monotonic() + 5.0
    for sent_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        members = owned_members()
        if not members:
            break
        signal_members(members, sent_signal)
        step_deadline = min(deadline, time.monotonic() + 1.0)
        while time.monotonic() < step_deadline:
            if not owned_members():
                break
            time.sleep(0.02)
    clean_samples: list[dict[str, Any]] = []
    first_clean: float | None = None
    while time.monotonic() <= deadline:
        members = owned_members()
        ports = port_observer(domain)
        if not members and ports == baseline_ports:
            now = time.monotonic()
            if first_clean is None:
                first_clean = now
                clean_samples = [{"members": [], "ports": list(ports)}]
            elif now - first_clean >= 0.5:
                clean_samples.append({"members": [], "ports": list(ports)})
                return _domain_identity(
                    b"ctr-slice-7g-observer-cleanup-barrier-canonical-1\0",
                    {"stable_samples": clean_samples},
                )
        else:
            first_clean = None
            clean_samples = []
        time.sleep(0.02)
    _fail("observer_cleanup_uncertain", "observer process/DDS residual barrier did not clear")


def _run_server_owned_graph_observer(
    argv: tuple[str, ...], environment: dict[str, str], cwd: str, cgroup: str,
    *, cleanup_guard: ObserverCleanupGuardStore, guard_context: dict[str, Any],
    utc_now: Callable[[], str] | None = None,
    port_observer: Callable[[int], tuple[int, ...]] = _observed_udp_ports,
) -> dict[str, Any]:
    if type(guard_context) is not dict:
        _fail("cleanup_guard_context", "observer cleanup guard context must be exact")
    timestamp = _utc_now if utc_now is None else utc_now
    domain = int(environment["ROS_DOMAIN_ID"])
    baseline_ports = port_observer(domain)
    started = time.monotonic_ns()
    active = cleanup_guard.begin(timestamp=timestamp(), **guard_context)
    process: subprocess.Popen[bytes] | None = None
    bound: CleanupGuardObservation | None = None
    pidfd: int | None = None
    procfd: int | None = None
    selector = selectors.DefaultSelector()
    stdout = bytearray()
    stderr = bytearray()
    primary: BaseException | None = None
    cleanup_error: BaseException | None = None
    cleanup_identity: str | None = None
    unexpected_group_member = False
    returncode: int | None = None
    previous_subreaper: bool | None = None
    try:
        previous_subreaper = _set_child_subreaper(True)
        process = subprocess.Popen(
            argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            cwd=cwd, env=environment, shell=False, close_fds=True, start_new_session=True,
        )
        # Popen has not polled or waited: even a leader that exits immediately
        # remains an unreaped zombie, retaining PID/PGID/session provenance.
        procfd = os.open(
            f"/proc/{process.pid}", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW,
        )
        start_ticks = _proc_start_time(f"/proc/{process.pid}")
        pgid = os.getpgid(process.pid)
        session_id = os.getsid(process.pid)
        if pgid != process.pid or session_id != process.pid:
            _fail("observer_process_group", "observer does not own its dedicated process session")
        observed_cgroup = _read_proc_bounded(
            f"/proc/{process.pid}/cgroup", 65_536,
        ).decode("utf-8", "strict").splitlines()
        if observed_cgroup != ["0::" + cgroup]:
            _fail("observer_process_ownership", "observer cgroup differs")
        if hasattr(os, "pidfd_open"):
            pidfd = os.pidfd_open(process.pid, 0)
            handle = os.fstat(pidfd)
            handle_kind = "pidfd"
        else:
            handle = os.fstat(procfd)
            handle_kind = "proc_directory"
        pidfd_identity = _domain_identity(
            b"ctr-slice-7g-observer-process-handle-canonical-1\0",
            {
                "device": handle.st_dev, "inode": handle.st_ino, "kind": handle_kind,
                "pid": process.pid, "start_time_ticks": start_ticks,
            },
        )
        bound = cleanup_guard.bind_process(
            active.record.logical_identity, pid=process.pid,
            process_start_time_ticks=start_ticks, process_group_id=pgid,
            session_id=session_id, cgroup=cgroup, pidfd_identity=pidfd_identity,
            timestamp=timestamp(),
        )
        assert process.stdout is not None and process.stderr is not None
        for stream, name in ((process.stdout, "stdout"), (process.stderr, "stderr")):
            os.set_blocking(stream.fileno(), False)
            selector.register(stream, selectors.EVENT_READ, name)
        deadline = time.monotonic() + 10.0
        while selector.get_map():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                _fail("observer_timeout", "ROS graph observer exceeded 10 seconds")
            for key, _ in selector.select(min(remaining, 0.1)):
                chunk = os.read(key.fileobj.fileno(), 65_536)
                if not chunk:
                    selector.unregister(key.fileobj)
                    key.fileobj.close()
                    continue
                target = stdout if key.data == "stdout" else stderr
                target.extend(chunk)
                if len(target) > 1_048_576:
                    _fail("observer_output_size", f"observer {key.data} exceeds 1048576 bytes")
    except BaseException as exc:
        primary = exc
    finally:
        selector.close()
        if process is not None:
            for stream in (process.stdout, process.stderr):
                if stream is not None and not stream.closed:
                    stream.close()
        if process is not None and bound is not None:
            try:
                retained = (process.pid, start_ticks)
                unexpected_group_member = bool(_server_group_members(
                    pgid, cgroup, start_ticks, session_id,
                    retained_zombie=retained,
                ))
                cleanup_identity = _cleanup_server_observer_group(
                    pgid, cgroup, start_ticks, domain, baseline_ports, (retained,),
                    port_observer=port_observer, session_id=session_id,
                    retained_zombie=retained,
                )
            except BaseException as exc:
                cleanup_error = exc
            try:
                returncode = process.wait(timeout=1.0)
            except BaseException as exc:
                if cleanup_error is None:
                    cleanup_error = exc
                else:
                    _add_cleanup_note(cleanup_error, exc, "observer reap")
        elif process is not None:
            # The unreaped Popen relationship and start_new_session contract
            # are sufficient to stop the task-owned group, but not to claim
            # clearance; the durable guard remains quarantined.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=1.0)
            except BaseException as exc:
                cleanup_error = exc
        terminal_error: BaseException | None = None
        try:
            latest = cleanup_guard.observe()
            if cleanup_error is None and bound is not None and cleanup_identity is not None:
                cleanup_guard.clear(bound.record.logical_identity, cleanup_identity, timestamp())
            elif latest.record.data["state"] in {"ACTIVE_UNBOUND", "ACTIVE_BOUND"}:
                cleanup_guard.quarantine(
                    latest.record.logical_identity,
                    _domain_identity(
                        b"ctr-slice-7g-observer-cleanup-quarantine-canonical-1\0",
                        {
                            "primary": None if primary is None else type(primary).__name__,
                            "cleanup": None if cleanup_error is None else type(cleanup_error).__name__,
                            "session_binding_identity": guard_context["session_binding_identity"],
                        },
                    ),
                    timestamp(),
                )
        except BaseException as exc:
            terminal_error = exc
        for descriptor in (pidfd, procfd):
            if descriptor is not None:
                try:
                    os.close(descriptor)
                except BaseException as exc:
                    if terminal_error is None:
                        terminal_error = exc
        if previous_subreaper is not None:
            try:
                _set_child_subreaper(previous_subreaper)
            except BaseException as exc:
                if terminal_error is None:
                    terminal_error = exc
        if primary is not None:
            if cleanup_error is not None:
                _add_cleanup_note(primary, cleanup_error, "observer cleanup")
            if terminal_error is not None:
                _add_cleanup_note(primary, terminal_error, "cleanup guard transition")
            raise primary
        if cleanup_error is not None:
            if terminal_error is not None:
                _add_cleanup_note(cleanup_error, terminal_error, "cleanup guard transition")
            raise cleanup_error
        if terminal_error is not None:
            raise terminal_error
        if unexpected_group_member:
            _fail(
                "observer_unexpected_descendant",
                "observer descendant survived immediate parent exit and was removed",
            )
    return {
        "pidfd_identity": pidfd_identity,
        "session_id": session_id,
        "pid": process.pid, "pgid": pgid, "start_time_ticks": start_ticks,
        "started_monotonic_ns": started, "ended_monotonic_ns": time.monotonic_ns(),
        "exit_status": int(returncode), "stdout": bytes(stdout), "stderr": bytes(stderr),
        "cleanup_barrier_identity": cleanup_identity,
    }


def _set_child_subreaper(enabled: bool) -> bool:
    if type(enabled) is not bool:
        _fail("observer_subreaper", "subreaper state must be an exact Boolean")
    libc = ctypes.CDLL(None, use_errno=True)
    current = ctypes.c_int()
    if libc.prctl(37, ctypes.byref(current), 0, 0, 0) != 0:  # PR_GET_CHILD_SUBREAPER
        _fail("observer_subreaper", f"prctl get failed with errno {ctypes.get_errno()}")
    if bool(current.value) != enabled and libc.prctl(36, int(enabled), 0, 0, 0) != 0:  # PR_SET_CHILD_SUBREAPER
        _fail("observer_subreaper", f"prctl set failed with errno {ctypes.get_errno()}")
    return bool(current.value)


def _reap_owned_group_zombies(
    pgid: int, cgroup: str, minimum_start_ticks: int, session_id: int | None,
    *, retained_zombie: tuple[int, int] | None,
) -> None:
    members = _server_group_members(
        pgid, cgroup, minimum_start_ticks, session_id,
        retained_zombie=retained_zombie,
    )
    for pid, expected_start in members:
        try:
            raw = _read_proc_bounded(f"/proc/{pid}/stat", 65_536).decode("ascii", "strict")
            fields = raw[raw.rfind(")") + 2:].split()
            state, start = fields[0], int(fields[19])
        except (FileNotFoundError, ProcessLookupError):
            continue
        if state != "Z" or start != expected_start:
            continue
        try:
            waited, _ = os.waitpid(pid, os.WNOHANG)
        except ChildProcessError:
            continue
        if waited not in {0, pid}:
            _fail("observer_process_ownership", "unexpected child was reaped")


def _parse_server_ros_nodes(raw: bytes) -> tuple[str, ...]:
    if len(raw) > 1_048_576 or b"\r" in raw or b"\0" in raw:
        _fail("observer_output", "ROS graph output contains prohibited bytes")
    try:
        text = raw.decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        raise Slice7GAuthorityDaemonError("observer_output_utf8", type(exc).__name__) from exc
    if text.endswith("\n"):
        text = text[:-1]
    if "\n\n" in text or text.endswith("\n"):
        _fail("observer_output", "ROS graph output contains empty or extra terminal lines")
    if not text:
        return ()
    nodes = tuple(text.split("\n"))
    if len(nodes) > 65_536 or len(nodes) != len(set(nodes)):
        _fail("observer_output", "ROS graph node inventory is duplicate or oversized")
    for node in nodes:
        if len(node.encode("utf-8")) > 8_192 or unicodedata.normalize("NFC", node) != node:
            _fail("observer_output", "ROS node name is noncanonical")
        if not node.startswith("/") or "//" in node or node.endswith("/"):
            _fail("observer_output", "ROS node name is not an absolute normalized name")
        if any(ord(character) < 0x20 or ord(character) == 0x7F for character in node):
            _fail("observer_output", "ROS node name contains a control character")
    return nodes


def _error_receipt(request: Slice7GAuthorityRecord, service_identity: str, code: str) -> MappingProxyType:
    v7 = request.data["schema_version"] == RUNTIME_AUTHORITY_REQUEST_V4_SCHEMA
    data = {
        "schema_version": (
            RUNTIME_AUTHORITY_RECEIPT_V4_SCHEMA if v7 else AUTHORITY_RECEIPT_SCHEMA
        ),
        "method": request.data["method"], "request_id": request.data["request_id"],
        "result": "ERROR", "authorization_identity": request.data["authorization_identity"],
        "service_instance_identity": service_identity, "service_nonce": None,
        "prepare_token": None,
        "previous_budget_revision": None, "budget_revision": None, "budget_identity": None,
        "campaign_id": request.data["campaign_id"],
        "campaign_identity": request.data["campaign_identity"], "domain_id": request.data["domain_id"],
        "campaign_template_identity": request.data["campaign_template_identity"],
        "output_root_path": request.data["output_root_path"],
        "output_root_identity": request.data["output_root_identity"],
        "process_manifest_identity": request.data["process_manifest_identity"],
        "process_instance_identity": request.data["process_instance_identity"],
        "observation_session_identity": request.data["observation_session_identity"],
        "observation_session_nonce": request.data["observation_session_nonce"],
        "observation_session_deadline_monotonic_ns": None,
        "four_source_observation_identity": None,
        "precommit_receipt_identities": [],
        "precommit_observer_count": 0,
        "postcommit_observer_count": 0,
        "transaction_observer_count": 0,
        "lease_identity": None,
        "prepare_expires_monotonic_ns": None,
        "committed_at_utc": None, "candidate_clear": None, "error_code": code,
    }
    if v7:
        data.update({
            "cleanup_head_identity": None,
            "containment_receipt_identity": None,
        })
        return validate_privileged_record(
            data, expected_schema=RUNTIME_AUTHORITY_RECEIPT_V4_SCHEMA,
        ).data
    return validate_authority_record(data, expected_schema=AUTHORITY_RECEIPT_SCHEMA).data


def _clean_eof(channel: socket.socket) -> bool:
    try:
        value = channel.recv(1, socket.MSG_PEEK)
    except OSError as exc:
        raise Slice7GAuthorityProtocolError("authority_socket_read", str(exc)) from exc
    return value == b""


def _receive_v7_runtime_frame(channel: socket.socket) -> PrivilegedRecord:
    if type(channel) is not socket.socket or channel.family != socket.AF_UNIX:
        _fail("authority_socket_type", "runtime authority channel must be AF_UNIX")
    header = _recv_exact_v7(channel, 4)
    size = struct.unpack("!I", header)[0]
    if size == 0 or size > MAX_FRAME_BYTES:
        _fail("authority_frame_size", "runtime authority frame size differs")
    return validate_privileged_record(
        _recv_exact_v7(channel, size),
        expected_schema=RUNTIME_AUTHORITY_REQUEST_V4_SCHEMA,
    )


def _send_v7_runtime_frame(channel: socket.socket, value: dict[str, Any]) -> None:
    payload = canonical_privileged_bytes(
        value, expected_schema=RUNTIME_AUTHORITY_RECEIPT_V4_SCHEMA,
    )
    view = memoryview(struct.pack("!I", len(payload)) + payload)
    while view:
        written = channel.send(view)
        if written <= 0:
            _fail("authority_socket_write", "runtime authority socket made no progress")
        view = view[written:]


def _recv_exact_v7(channel: socket.socket, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining:
        part = channel.recv(remaining)
        if not part:
            _fail("authority_frame_truncated", "runtime authority frame ended early")
        chunks.append(part)
        remaining -= len(part)
    return b"".join(chunks)


def _open_directory_path(path: str) -> int:
    descriptor = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        for part in PurePosixPath(path).parts[1:]:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def _open_regular_path_nofollow(path: str) -> int:
    absolute = _absolute_path(path)
    parts = PurePosixPath(absolute).parts[1:]
    directory = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        for part in parts[:-1]:
            child = os.open(
                part, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW,
                dir_fd=directory,
            )
            os.close(directory)
            directory = child
        return os.open(
            parts[-1], os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=directory,
        )
    finally:
        os.close(directory)


def _open_directory_at(parent: int, name: str) -> int:
    if type(name) is not str or not name or "/" in name or name in {".", ".."}:
        _fail("authority_path", "directory component is unsafe")
    return os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=parent)


def _revision_name(revision: int) -> str:
    return f"{BUDGET_REVISION_PREFIX}{revision:0{BUDGET_REVISION_WIDTH}d}.json"


def _cleanup_guard_revision_name(revision: int) -> str:
    return (
        f"{CLEANUP_GUARD_REVISION_PREFIX}"
        f"{revision:0{CLEANUP_GUARD_REVISION_WIDTH}d}.json"
    )


def _empty_cleanup_guard(timestamp: str) -> dict[str, Any]:
    return {
        "schema_version": OBSERVER_CLEANUP_GUARD_SCHEMA,
        "revision": 0, "predecessor_identity": None, "state": "CLEARED",
        "authorization_identity": None, "budget_identity": None,
        "service_generation_identity": None, "session_binding_identity": None,
        "phase": None, "phase_local_ordinal": None,
        "transaction_observer_ordinal": None, "domain_id": None,
        "executable_identity": None, "argv_identity": None,
        "environment_identity": None, "pid": None, "process_start_time_ticks": None,
        "process_group_id": None, "session_id": None, "cgroup": None,
        "pidfd_identity": None, "disposition_identity": None,
        "recovery_authorization_identity": None, "updated_at_utc": timestamp,
    }


def _directory_identity(info: os.stat_result) -> tuple[int, int, int, int, int, int]:
    if not stat.S_ISDIR(info.st_mode):
        _fail("authority_directory", "authority member is not a directory")
    return (info.st_dev, info.st_ino, stat.S_IMODE(info.st_mode), info.st_nlink, info.st_uid, info.st_gid)


def _path_inode(info: os.stat_result) -> tuple[int, int]:
    if not stat.S_ISDIR(info.st_mode):
        _fail("output_member_type", "output member is not a directory")
    return (info.st_dev, info.st_ino)


def _remove_owned_empty_tree(root_fd: int, cell_names: tuple[str, ...]) -> None:
    cells_fd = _open_directory_at(root_fd, "cells")
    try:
        observed = sorted(os.listdir(cells_fd))
        if observed != sorted(cell_names):
            _fail("rollback_inventory", "provisional cell inventory differs")
        for name in cell_names:
            child = _open_directory_at(cells_fd, name)
            try:
                if os.listdir(child):
                    _fail("rollback_nonempty", "provisional cell output is not empty", path=name)
            finally:
                os.close(child)
            os.rmdir(name, dir_fd=cells_fd)
        os.fsync(cells_fd)
    finally:
        os.close(cells_fd)
    os.rmdir("cells", dir_fd=root_fd)
    authority_fd = _open_directory_at(root_fd, "authority")
    try:
        if os.listdir(authority_fd):
            _fail("rollback_nonempty", "provisional authority directory is not empty")
    finally:
        os.close(authority_fd)
    os.rmdir("authority", dir_fd=root_fd)
    os.fsync(root_fd)


def _rollback_partial_output(
    parent_fd: int,
    leaf: str,
    root_fd: int,
    root_identity: tuple[int, int],
    authority_identity: tuple[int, int] | None,
    cells_fd: int | None,
    cells_identity: tuple[int, int] | None,
    cell_identities: dict[str, tuple[int, int]],
) -> None:
    """Remove only descriptor-reconciled resources created by one failed allocation."""

    named_root = os.stat(leaf, dir_fd=parent_fd, follow_symlinks=False)
    if _path_inode(named_root) != root_identity or _path_inode(os.fstat(root_fd)) != root_identity:
        _fail("rollback_root_replaced", "provisional output root changed before rollback")
    owned_cells_fd = cells_fd
    if cells_identity is not None and owned_cells_fd is None:
        owned_cells_fd = _open_directory_at(root_fd, "cells")
    try:
        if cells_identity is not None:
            assert owned_cells_fd is not None
            if _path_inode(os.fstat(owned_cells_fd)) != cells_identity:
                _fail("rollback_cells_replaced", "provisional cells directory changed")
            if set(os.listdir(owned_cells_fd)) != set(cell_identities):
                _fail("rollback_inventory", "provisional cells inventory contains an unowned member")
            for name in reversed(tuple(cell_identities)):
                observed = os.stat(name, dir_fd=owned_cells_fd, follow_symlinks=False)
                if _path_inode(observed) != cell_identities[name]:
                    _fail("rollback_cell_replaced", "provisional cell directory changed", path=name)
                child = _open_directory_at(owned_cells_fd, name)
                try:
                    if os.listdir(child):
                        _fail("rollback_nonempty", "provisional cell directory is not empty", path=name)
                finally:
                    os.close(child)
                os.rmdir(name, dir_fd=owned_cells_fd)
            os.fsync(owned_cells_fd)
    finally:
        if owned_cells_fd is not None:
            os.close(owned_cells_fd)
    if cells_identity is not None:
        os.rmdir("cells", dir_fd=root_fd)
    if authority_identity is not None:
        observed = os.stat("authority", dir_fd=root_fd, follow_symlinks=False)
        if _path_inode(observed) != authority_identity:
            _fail("rollback_authority_replaced", "provisional authority directory changed")
        authority_fd = _open_directory_at(root_fd, "authority")
        try:
            if os.listdir(authority_fd):
                _fail("rollback_nonempty", "provisional authority directory is not empty")
        finally:
            os.close(authority_fd)
        os.rmdir("authority", dir_fd=root_fd)
    if os.listdir(root_fd):
        _fail("rollback_inventory", "provisional output root contains an unowned member")
    os.fsync(root_fd)
    os.close(root_fd)
    os.rmdir(leaf, dir_fd=parent_fd)
    os.fsync(parent_fd)


def _assert_tree_nofollow(root_fd: int) -> None:
    stack = [os.dup(root_fd)]
    try:
        while stack:
            directory = stack.pop()
            try:
                for entry in os.scandir(directory):
                    info = entry.stat(follow_symlinks=False)
                    if stat.S_ISLNK(info.st_mode):
                        _fail("output_symlink", "output tree contains a symlink", path=entry.name)
                    if stat.S_ISDIR(info.st_mode):
                        stack.append(_open_directory_at(directory, entry.name))
                    elif not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                        _fail("output_member_type", "output tree contains a non-regular or aliased member", path=entry.name)
            finally:
                os.close(directory)
    finally:
        for descriptor in stack:
            os.close(descriptor)


def _file_identity(info: os.stat_result) -> tuple[int, int, int, int, int, int, int]:
    return (info.st_dev, info.st_ino, stat.S_IMODE(info.st_mode), info.st_nlink, info.st_size, info.st_uid, info.st_gid)


def _read_fd(descriptor: int, maximum: int) -> bytes:
    result = bytearray()
    while True:
        chunk = os.read(descriptor, min(1_048_576, maximum + 1 - len(result)))
        if not chunk:
            return bytes(result)
        result.extend(chunk)
        if len(result) > maximum:
            _fail("authority_record_size", "authority record exceeds bound")


def _write_all(descriptor: int, payload: bytes) -> None:
    view = memoryview(payload)
    while view:
        count = os.write(descriptor, view)
        if count <= 0:
            _fail("authority_write", "authority write made no progress")
        view = view[count:]


def _plain_dict(value: Any) -> dict[str, Any]:
    if type(value) is not dict:
        _fail("authority_mapping", "value must be an exact dictionary")
    return json.loads(json.dumps(value, ensure_ascii=False, allow_nan=False))


def _builtin_authority_value(value: Any) -> Any:
    """Thaw only values produced by the protocol's immutable validator.

    Public callers still must supply exact built-in JSON values.  This helper
    exists solely at the internal boundary where a validated authority record
    has deliberately converted dictionaries/lists to mapping proxies/tuples.
    """

    if isinstance(value, MappingProxyType) or type(value) is dict:
        return {key: _builtin_authority_value(member) for key, member in value.items()}
    if type(value) in {tuple, list}:
        return [_builtin_authority_value(member) for member in value]
    if type(value) in {str, int, float, bool} or value is None:
        return value
    _fail("authority_internal_value", "validated authority value has an unsupported type")


def _digest(value: Any) -> str:
    if type(value) is not str or len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
        _fail("authority_digest", "value must be a lowercase SHA-256 digest")
    return value


def _absolute_path(value: Any) -> str:
    if type(value) is not str:
        _fail("authority_path", "authority path must be an exact string")
    path = PurePosixPath(value)
    if not path.is_absolute() or "\\" in value or "//" in value or any(part in {".", ".."} for part in path.parts):
        _fail("authority_path", "authority path is not normalized")
    return value


def _parse_utc(value: str) -> datetime:
    try:
        return datetime.strptime(value, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
    except (TypeError, ValueError) as exc:
        raise Slice7GAuthorityDaemonError("authority_time", "authority timestamp is invalid") from exc


def _utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _runtime_campaign_identity(
    authorization_identity: str, campaign_id: str, campaign_template_identity: str,
) -> str:
    payload = json.dumps(
        {
            "authorization_identity": authorization_identity,
            "campaign_id": campaign_id,
            "campaign_template_identity": campaign_template_identity,
            "schema_version": "ctr-slice-7g-runtime-campaign-1",
        },
        ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(b"ctr-slice-7g-runtime-campaign-canonical-1\0" + payload).hexdigest()


def _fsync_directory(path: str) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _fail(code: str, message: str, *, path: str = "$") -> None:
    raise Slice7GAuthorityDaemonError(code, message, path=path)


__all__ = [
    "AuthorityOutputProvisioner", "BUDGET_DIRECTORY_NAME", "BudgetObservation", "GlobalAttemptBudgetStore",
    "PreparedCampaign", "ProvisionalAllocation", "RuntimeAuthorityStateMachine", "Slice7GAuthorityDaemon",
    "Slice7GAuthorityDaemonError", "enforce_pending_revocations", "main",
]
