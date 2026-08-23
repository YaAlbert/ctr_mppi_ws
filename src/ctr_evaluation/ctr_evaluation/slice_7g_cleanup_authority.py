"""Root-owned monotonic cleanup authority for Slice 7G.

Only the fixed cleanup helper may mutate this ledger in production.  The
unprivileged authority daemon has query-only protocol access and no direct
filesystem access to the root-owned 0700 state tree.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import secrets
import select
import socket
import stat
from types import MappingProxyType
from typing import Any, Callable

from .slice_7g_privileged_protocol import (
    AUTHORITY_EXECUTABLE,
    CLEANUP_ANCHOR_SCHEMA,
    CLEANUP_AUTHORITY_SOCKET,
    CLEANUP_AUTHORITY_STATE_ROOT,
    CLEANUP_RECOVERY_SOCKET,
    CLEANUP_HEAD_SCHEMA,
    CLEANUP_RECOVERY_AUTHORIZATION_V2_SCHEMA,
    CLEANUP_RECOVERY_OBSERVATION_SCHEMA,
    CLEANUP_RECOVERY_PROVIDER_RECEIPT_SCHEMA,
    CLEANUP_REVISION_SCHEMA,
    OBSERVER_SUPERVISOR_CGROUP,
    OBSERVER_SUPERVISOR_EXECUTABLE,
    PRIVILEGED_RECEIPT_SCHEMA,
    PRIVILEGED_REQUEST_SCHEMA,
    PeerProcess,
    PrivilegedRecord,
    Slice7GPrivilegedProtocolError,
    ReplayWindow,
    canonical_bytes,
    peer_credentials,
    observe_peer,
    receive_packet,
    reconcile_peer,
    record_identity,
    send_packet,
    validate_record,
    verify_response_binding,
)


LOCK_NAME = "ledger.lock"
REVISION_DIRECTORY = "revisions"
ANCHOR_DIRECTORY = "anchors"
HEAD_DIRECTORY = "heads"
REVISION_PATTERN = re.compile(r"^revision-([0-9]{20})\.json$")
ANCHOR_PATTERN = re.compile(r"^anchor-([0-9]{20})\.json$")
HEAD_PATTERN = re.compile(r"^head-([0-9]{20})\.json$")
MAX_LEDGER_REVISIONS = 100_000
_RECOVERY_INTERNAL_TOKEN = object()


class Slice7GCleanupAuthorityError(RuntimeError):
    """Stable cleanup-ledger/service error."""

    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(f"{code}:{message}")


@dataclass(frozen=True)
class CleanupLedgerObservation:
    revision: PrivilegedRecord
    anchor: PrivilegedRecord
    head: PrivilegedRecord

    @property
    def state(self) -> str:
        return str(self.revision.data["state"])


@dataclass(frozen=True)
class RecoveryProviderEvidence:
    """Raw fixed-provider evidence; it is not caller-serializable authority."""

    provider: str
    evidence_identity: str
    residual_count: int
    lease_state: str | None
    started_monotonic_ns: int
    ended_monotonic_ns: int
    cleanup_disposition_identity: str


class CleanupAuthorityLedger:
    """Descriptor-confined immutable revision/anchor/head chain."""

    def __init__(
        self,
        state_root: str = CLEANUP_AUTHORITY_STATE_ROOT,
        *,
        _test: bool = False,
        _expected_owner_uid: int | None = None,
    ) -> None:
        if not _test and state_root != CLEANUP_AUTHORITY_STATE_ROOT:
            _fail("cleanup_root_override", "production cleanup root is fixed")
        if not _test and os.geteuid() != 0:
            _fail("cleanup_root_principal", "production cleanup ledger requires root")
        self._test = _test
        self._path = _absolute(state_root)
        self._owner_uid = 0 if _expected_owner_uid is None and not _test else (
            os.geteuid() if _expected_owner_uid is None else _expected_owner_uid
        )
        self._root_fd = _open_directory_path(self._path)
        root = os.fstat(self._root_fd)
        if (
            not stat.S_ISDIR(root.st_mode)
            or root.st_uid != self._owner_uid
            or stat.S_IMODE(root.st_mode) != 0o700
            or root.st_nlink < 2
        ):
            self.close()
            _fail("cleanup_root_identity", "cleanup state root identity differs")
        self._root_stat = _directory_identity(root)
        self._root_identity = _identity(
            b"ctr-slice-7g-cleanup-root-physical-canonical-1\0",
            {"device": root.st_dev, "inode": root.st_ino, "mode": 0o700, "path": self._path},
        )
        self._revision_fd = _open_directory_at(self._root_fd, REVISION_DIRECTORY)
        self._anchor_fd = _open_directory_at(self._root_fd, ANCHOR_DIRECTORY)
        self._head_fd = _open_directory_at(self._root_fd, HEAD_DIRECTORY)
        self._directory_stats = tuple(
            _directory_identity(os.fstat(fd))
            for fd in (self._revision_fd, self._anchor_fd, self._head_fd)
        )
        for fd in (self._revision_fd, self._anchor_fd, self._head_fd):
            info = os.fstat(fd)
            if info.st_uid != self._owner_uid or stat.S_IMODE(info.st_mode) != 0o700:
                self.close()
                _fail("cleanup_directory_identity", "cleanup ledger directory differs")
        self._lock_fd = os.open(
            LOCK_NAME, os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=self._root_fd,
        )
        lock = os.fstat(self._lock_fd)
        by_name = os.stat(LOCK_NAME, dir_fd=self._root_fd, follow_symlinks=False)
        if (
            not stat.S_ISREG(lock.st_mode) or lock.st_uid != self._owner_uid
            or stat.S_IMODE(lock.st_mode) != 0o600 or lock.st_nlink != 1
            or _physical_tuple(lock) != _physical_tuple(by_name)
        ):
            self.close()
            _fail("cleanup_lock_identity", "cleanup ledger lock differs")
        self._lock_stat = _physical_tuple(lock)
        self._retained_physical: dict[tuple[str, int], tuple[int, ...]] = {}
        if not (_test and self._inventory_counts() == (0, 0, 0)):
            self.reconstruct()

    @classmethod
    def _for_test(cls, state_root: str) -> "CleanupAuthorityLedger":
        return cls(state_root, _test=True, _expected_owner_uid=os.geteuid())

    @staticmethod
    def _provision_test_root(state_root: str, *, timestamp: str = "2026-08-23T00:00:00Z") -> None:
        root = Path(_absolute(state_root))
        root.mkdir(mode=0o700)
        for name in (REVISION_DIRECTORY, ANCHOR_DIRECTORY, HEAD_DIRECTORY):
            (root / name).mkdir(mode=0o700)
        lock = os.open(root / LOCK_NAME, os.O_CREAT | os.O_EXCL | os.O_RDWR | os.O_CLOEXEC, 0o600)
        try:
            os.fsync(lock)
        finally:
            os.close(lock)
        for name in (REVISION_DIRECTORY, ANCHOR_DIRECTORY, HEAD_DIRECTORY):
            _fsync_path(str(root / name))
        _fsync_path(str(root))
        ledger = CleanupAuthorityLedger._for_test(str(root))
        try:
            if ledger._inventory_counts() != (0, 0, 0):
                _fail("cleanup_provision", "test ledger was not empty")
            ledger._append_locked({
                "schema_version": CLEANUP_REVISION_SCHEMA,
                "revision": 0,
                "predecessor_identity": None,
                "state": "CLEARED",
                "runtime_authorization_identity": None,
                "budget_identity": None,
                "service_generation_identity": None,
                "session_binding_identity": None,
                "phase": None,
                "phase_local_ordinal": None,
                "transaction_observer_ordinal": None,
                "domain_id": None,
                "observer_contract_identity": None,
                "containment_identity": None,
                "process_identity": None,
                "disposition_identity": _identity(
                    b"ctr-slice-7g-cleanup-initial-canonical-1\0", {"state": "CLEARED"},
                ),
                "recovery_authorization_identity": None,
                "created_at_utc": timestamp,
            })
        finally:
            ledger.close()

    @property
    def root_identity(self) -> str:
        return self._root_identity

    def close(self) -> None:
        for name in ("_lock_fd", "_head_fd", "_anchor_fd", "_revision_fd", "_root_fd"):
            descriptor = getattr(self, name, None)
            if descriptor is not None:
                try:
                    os.close(descriptor)
                finally:
                    setattr(self, name, None)

    def __enter__(self) -> "CleanupAuthorityLedger":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()

    def reconstruct(self) -> CleanupLedgerObservation:
        self._barrier()
        with _nonblocking_lock(self._lock_fd):
            return self._reconstruct_locked()

    def require_clear(self) -> CleanupLedgerObservation:
        observed = self.reconstruct()
        if observed.state not in {"CLEARED", "RECOVERED"}:
            _fail("cleanup_blocked", f"cleanup state {observed.state} blocks new work")
        return observed

    def append(self, value: dict[str, Any]) -> CleanupLedgerObservation:
        self._barrier()
        with _nonblocking_lock(self._lock_fd):
            current = self._reconstruct_locked()
            detached = _exact_dict(value)
            expected_revision = int(current.revision.data["revision"]) + 1
            if detached.get("revision") != expected_revision:
                _fail("cleanup_revision", "cleanup successor revision differs")
            if detached.get("predecessor_identity") != current.revision.logical_identity:
                _fail("cleanup_predecessor", "cleanup successor predecessor differs")
            self._validate_transition(current.state, detached.get("state"))
            return self._append_locked(detached)

    def begin_unbound(
        self, *, runtime_authorization_identity: str, budget_identity: str,
        service_generation_identity: str, session_binding_identity: str, phase: str,
        phase_local_ordinal: int, transaction_observer_ordinal: int, domain_id: int,
        observer_contract_identity: str, timestamp: str,
    ) -> CleanupLedgerObservation:
        current = self.require_clear()
        return self.append({
            "schema_version": CLEANUP_REVISION_SCHEMA,
            "revision": int(current.revision.data["revision"]) + 1,
            "predecessor_identity": current.revision.logical_identity,
            "state": "ACTIVE_UNBOUND",
            "runtime_authorization_identity": runtime_authorization_identity,
            "budget_identity": budget_identity,
            "service_generation_identity": service_generation_identity,
            "session_binding_identity": session_binding_identity,
            "phase": phase,
            "phase_local_ordinal": phase_local_ordinal,
            "transaction_observer_ordinal": transaction_observer_ordinal,
            "domain_id": domain_id,
            "observer_contract_identity": observer_contract_identity,
            "containment_identity": None,
            "process_identity": None,
            "disposition_identity": None,
            "recovery_authorization_identity": None,
            "created_at_utc": timestamp,
        })

    def bind(
        self, predecessor: CleanupLedgerObservation, *, containment_identity: str,
        process_identity: str, timestamp: str,
    ) -> CleanupLedgerObservation:
        value = dict(predecessor.revision.data)
        value.update({
            "revision": int(value["revision"]) + 1,
            "predecessor_identity": predecessor.revision.logical_identity,
            "state": "ACTIVE_BOUND",
            "containment_identity": containment_identity,
            "process_identity": process_identity,
            "created_at_utc": timestamp,
        })
        return self.append(value)

    def terminate(
        self, predecessor: CleanupLedgerObservation, *, state: str,
        disposition_identity: str, timestamp: str,
        recovery_authorization_identity: str | None = None,
    ) -> CleanupLedgerObservation:
        value = dict(predecessor.revision.data)
        value.update({
            "revision": int(value["revision"]) + 1,
            "predecessor_identity": predecessor.revision.logical_identity,
            "state": state,
            "disposition_identity": disposition_identity,
            "recovery_authorization_identity": recovery_authorization_identity,
            "created_at_utc": timestamp,
        })
        return self.append(value)

    def _recover_authenticated(
        self,
        authorization: dict[str, Any],
        provider_receipts: list[dict[str, Any]],
        observation: dict[str, Any],
        *,
        timestamp: str,
        _internal_token: object,
    ) -> CleanupLedgerObservation:
        if _internal_token is not _RECOVERY_INTERNAL_TOKEN:
            _fail("recovery_authority", "recovery requires daemon-owned evidence authority")
        current = self.reconstruct()
        if current.state != "QUARANTINED":
            _fail("recovery_state", "recovery requires current QUARANTINED state")
        auth = validate_record(
            authorization, expected_schema=CLEANUP_RECOVERY_AUTHORIZATION_V2_SCHEMA,
        )
        if auth.data["quarantine_head_identity"] != current.head.logical_identity:
            _fail("recovery_head", "recovery authorization targets another head")
        if auth.data["quarantine_anchor_identity"] != current.anchor.logical_identity:
            _fail("recovery_anchor", "recovery authorization targets another anchor")
        now = _parse_utc(timestamp)
        if not (_parse_utc(auth.data["not_before_utc"]) <= now < _parse_utc(auth.data["not_after_utc"])):
            _fail("recovery_time", "recovery authorization is not current")
        if auth.data["one_shot"] is not True:
            _fail("recovery_one_shot", "recovery authorization is not one-shot")
        if type(provider_receipts) is not list or len(provider_receipts) != 4:
            _fail("recovery_receipts", "recovery requires four daemon/helper receipts")
        validated = [
            validate_record(item, expected_schema=CLEANUP_RECOVERY_PROVIDER_RECEIPT_SCHEMA)
            for item in provider_receipts
        ]
        providers = [item.data["provider"] for item in validated]
        if providers != ["process", "dds", "lease", "graph"]:
            _fail("recovery_receipts", "recovery provider order or inventory differs")
        if any(
            item.data["recovery_authorization_identity"] != auth.logical_identity
            or item.data["quarantine_head_identity"] != current.head.logical_identity
            or item.data["quarantine_anchor_identity"] != current.anchor.logical_identity
            or item.data["clear"] is not True
            for item in validated
        ):
            _fail("recovery_receipts", "recovery provider binding/clearance differs")
        observed = validate_record(
            observation, expected_schema=CLEANUP_RECOVERY_OBSERVATION_SCHEMA,
        )
        if tuple(observed.data["provider_receipt_identities"]) != tuple(
            item.logical_identity for item in validated
        ) or observed.data["all_sources_clear"] is not True:
            _fail("recovery_observation", "recovery observation differs")
        disposition = _identity(
            b"ctr-slice-7g-cleanup-recovery-disposition-canonical-1\0",
            {
                "authorization_identity": auth.logical_identity,
                "observation_identity": observed.logical_identity,
                "quarantine_head_identity": current.head.logical_identity,
            },
        )
        return self.terminate(
            current, state="RECOVERED", disposition_identity=disposition,
            recovery_authorization_identity=auth.logical_identity, timestamp=timestamp,
        )

    def _append_locked(self, value: dict[str, Any]) -> CleanupLedgerObservation:
        revision_record = validate_record(value, expected_schema=CLEANUP_REVISION_SCHEMA)
        number = int(revision_record.data["revision"])
        revision_name = f"revision-{number:020d}.json"
        revision_info, revision_sha = self._write_immutable(
            self._revision_fd, revision_name, revision_record.canonical_bytes,
        )
        previous_anchor = None
        previous_head = None
        if number:
            prior = self._load_triple(number - 1)
            previous_anchor = prior.anchor.logical_identity
            previous_head = prior.head.logical_identity
        anchor_value = {
            "schema_version": CLEANUP_ANCHOR_SCHEMA,
            "revision": number,
            "authority_root_identity": self._root_identity,
            "revision_identity": revision_record.logical_identity,
            "revision_device": revision_info.st_dev,
            "revision_inode": revision_info.st_ino,
            "revision_mode": stat.S_IMODE(revision_info.st_mode),
            "revision_link_count": revision_info.st_nlink,
            "revision_size": revision_info.st_size,
            "revision_sha256": revision_sha,
            "predecessor_anchor_identity": previous_anchor,
        }
        anchor_record = validate_record(anchor_value, expected_schema=CLEANUP_ANCHOR_SCHEMA)
        anchor_name = f"anchor-{number:020d}.json"
        anchor_info, anchor_sha = self._write_immutable(
            self._anchor_fd, anchor_name, anchor_record.canonical_bytes,
        )
        head_value = {
            "schema_version": CLEANUP_HEAD_SCHEMA,
            "revision": number,
            "authority_root_identity": self._root_identity,
            "revision_identity": revision_record.logical_identity,
            "anchor_identity": anchor_record.logical_identity,
            "anchor_device": anchor_info.st_dev,
            "anchor_inode": anchor_info.st_ino,
            "anchor_mode": stat.S_IMODE(anchor_info.st_mode),
            "anchor_link_count": anchor_info.st_nlink,
            "anchor_size": anchor_info.st_size,
            "anchor_sha256": anchor_sha,
            "predecessor_head_identity": previous_head,
        }
        head_record = validate_record(head_value, expected_schema=CLEANUP_HEAD_SCHEMA)
        head_name = f"head-{number:020d}.json"
        self._write_immutable(self._head_fd, head_name, head_record.canonical_bytes)
        for fd in (self._revision_fd, self._anchor_fd, self._head_fd, self._root_fd):
            os.fsync(fd)
        observed = self._reconstruct_locked()
        if observed.head.logical_identity != head_record.logical_identity:
            _fail("cleanup_commit_barrier", "cleanup head differs after commit")
        return observed

    def _write_immutable(self, directory: int, name: str, payload: bytes) -> tuple[os.stat_result, str]:
        descriptor = os.open(
            name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC | os.O_NOFOLLOW,
            0o400,
            dir_fd=directory,
        )
        try:
            offset = 0
            while offset < len(payload):
                written = os.write(descriptor, payload[offset:])
                if written <= 0:
                    _fail("cleanup_write", "cleanup immutable write made no progress")
                offset += written
            os.fsync(descriptor)
            info = os.fstat(descriptor)
            if stat.S_IMODE(info.st_mode) != 0o400 or info.st_nlink != 1:
                _fail("cleanup_record_identity", "cleanup immutable record differs")
            return info, hashlib.sha256(payload).hexdigest()
        finally:
            os.close(descriptor)

    def _reconstruct_locked(self) -> CleanupLedgerObservation:
        revision_names = _exact_names(self._revision_fd, REVISION_PATTERN)
        anchor_names = _exact_names(self._anchor_fd, ANCHOR_PATTERN)
        head_names = _exact_names(self._head_fd, HEAD_PATTERN)
        if not revision_names or not (len(revision_names) == len(anchor_names) == len(head_names)):
            _fail("cleanup_inventory", "cleanup triple inventory is incomplete")
        expected = list(range(len(revision_names)))
        if (
            [number for number, _ in revision_names] != expected
            or [number for number, _ in anchor_names] != expected
            or [number for number, _ in head_names] != expected
            or len(expected) > MAX_LEDGER_REVISIONS
        ):
            _fail("cleanup_inventory", "cleanup triple inventory has a gap/fork/rollback")
        previous_revision = None
        previous_anchor = None
        previous_head = None
        latest = None
        seen_inodes: set[tuple[int, int]] = set()
        for number in expected:
            observed = self._load_triple(number, seen_inodes=seen_inodes)
            if observed.revision.data["predecessor_identity"] != previous_revision:
                _fail("cleanup_chain", "cleanup revision predecessor differs")
            if observed.anchor.data["predecessor_anchor_identity"] != previous_anchor:
                _fail("cleanup_chain", "cleanup anchor predecessor differs")
            if observed.head.data["predecessor_head_identity"] != previous_head:
                _fail("cleanup_chain", "cleanup head predecessor differs")
            if number:
                prior_state = latest.state
                self._validate_transition(prior_state, observed.state)
            previous_revision = observed.revision.logical_identity
            previous_anchor = observed.anchor.logical_identity
            previous_head = observed.head.logical_identity
            latest = observed
        assert latest is not None
        self._barrier()
        return latest

    def _load_triple(
        self, number: int, *, seen_inodes: set[tuple[int, int]] | None = None,
    ) -> CleanupLedgerObservation:
        revision, revision_info, revision_sha = self._read_immutable(
            self._revision_fd, f"revision-{number:020d}.json", CLEANUP_REVISION_SCHEMA,
        )
        anchor, anchor_info, anchor_sha = self._read_immutable(
            self._anchor_fd, f"anchor-{number:020d}.json", CLEANUP_ANCHOR_SCHEMA,
        )
        head, head_info, _ = self._read_immutable(
            self._head_fd, f"head-{number:020d}.json", CLEANUP_HEAD_SCHEMA,
        )
        if seen_inodes is not None:
            for info in (revision_info, anchor_info, head_info):
                inode = (info.st_dev, info.st_ino)
                if inode in seen_inodes:
                    _fail("cleanup_hardlink", "cleanup records share a physical inode")
                seen_inodes.add(inode)
        if (
            revision.data["revision"] != number
            or anchor.data["revision"] != number
            or head.data["revision"] != number
            or anchor.data["authority_root_identity"] != self._root_identity
            or head.data["authority_root_identity"] != self._root_identity
            or anchor.data["revision_identity"] != revision.logical_identity
            or head.data["revision_identity"] != revision.logical_identity
            or head.data["anchor_identity"] != anchor.logical_identity
            or tuple(anchor.data[field] for field in (
                "revision_device", "revision_inode", "revision_mode", "revision_link_count",
                "revision_size", "revision_sha256",
            )) != (
                revision_info.st_dev, revision_info.st_ino, stat.S_IMODE(revision_info.st_mode),
                revision_info.st_nlink, revision_info.st_size, revision_sha,
            )
            or tuple(head.data[field] for field in (
                "anchor_device", "anchor_inode", "anchor_mode", "anchor_link_count",
                "anchor_size", "anchor_sha256",
            )) != (
                anchor_info.st_dev, anchor_info.st_ino, stat.S_IMODE(anchor_info.st_mode),
                anchor_info.st_nlink, anchor_info.st_size, anchor_sha,
            )
        ):
            _fail("cleanup_anchor", "cleanup physical anchor/head binding differs")
        return CleanupLedgerObservation(revision, anchor, head)

    def _read_immutable(
        self, directory: int, name: str, schema: str,
    ) -> tuple[PrivilegedRecord, os.stat_result, str]:
        descriptor = os.open(name, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=directory)
        try:
            before = os.fstat(descriptor)
            by_name = os.stat(name, dir_fd=directory, follow_symlinks=False)
            if (
                not stat.S_ISREG(before.st_mode) or before.st_uid != self._owner_uid
                or before.st_nlink != 1 or stat.S_IMODE(before.st_mode) != 0o400
                or before.st_size <= 0 or before.st_size > 262_144
                or _physical_tuple(before) != _physical_tuple(by_name)
            ):
                _fail("cleanup_record_identity", "cleanup immutable record differs")
            raw = _read_fd(descriptor, 262_144)
            after = os.fstat(descriptor)
            if _physical_tuple(after) != _physical_tuple(before):
                _fail("cleanup_record_changed", "cleanup record changed while reading")
            key = (name.rsplit("-", 1)[0], int(name.rsplit("-", 1)[1].split(".")[0]))
            retained = self._retained_physical.get(key)
            physical = _physical_tuple(before)
            if retained is not None and retained != physical:
                _fail("cleanup_same_byte_replacement", "cleanup record inode was replaced")
            self._retained_physical[key] = physical
            return validate_record(raw, expected_schema=schema), before, hashlib.sha256(raw).hexdigest()
        finally:
            os.close(descriptor)

    def _inventory_counts(self) -> tuple[int, int, int]:
        return tuple(len(os.listdir(fd)) for fd in (self._revision_fd, self._anchor_fd, self._head_fd))

    def _barrier(self) -> None:
        if self._root_fd is None:
            _fail("cleanup_closed", "cleanup ledger is closed")
        if _directory_identity(os.fstat(self._root_fd)) != self._root_stat:
            _fail("cleanup_root_replaced", "cleanup root descriptor changed")
        reopened = _open_directory_path(self._path)
        try:
            if _directory_identity(os.fstat(reopened)) != self._root_stat:
                _fail("cleanup_root_replaced", "cleanup root pathname changed")
        finally:
            os.close(reopened)
        for fd, expected in zip(
            (self._revision_fd, self._anchor_fd, self._head_fd), self._directory_stats,
        ):
            if _directory_identity(os.fstat(fd)) != expected:
                _fail("cleanup_directory_replaced", "cleanup ledger directory changed")
        if _physical_tuple(os.fstat(self._lock_fd)) != self._lock_stat:
            _fail("cleanup_lock_replaced", "cleanup lock changed")
        by_name = os.stat(LOCK_NAME, dir_fd=self._root_fd, follow_symlinks=False)
        if _physical_tuple(by_name) != self._lock_stat:
            _fail("cleanup_lock_replaced", "cleanup lock pathname changed")

    @staticmethod
    def _validate_transition(current: str, successor: Any) -> None:
        allowed = {
            "CLEARED": {"ACTIVE_UNBOUND"},
            "RECOVERED": {"ACTIVE_UNBOUND"},
            "ACTIVE_UNBOUND": {"ACTIVE_BOUND", "QUARANTINED"},
            "ACTIVE_BOUND": {"CLEARED", "QUARANTINED"},
            "QUARANTINED": {"RECOVERED"},
        }
        if successor not in allowed.get(current, set()):
            _fail("cleanup_transition", f"transition {current!r}->{successor!r} is forbidden")


class CleanupAuthorityService:
    """Closed request dispatcher.  Socket activation/provisioning is external."""

    def __init__(
        self,
        ledger: CleanupAuthorityLedger,
        *,
        authority_uid: int,
        observer_supervisor_uid: int = 0,
        recovery_uid: int | None = None,
        service_generation_identity: str | None = None,
        utc_now: Callable[[], str] | None = None,
        recovery_controller: "CleanupRecoveryController | None" = None,
    ) -> None:
        if type(ledger) is not CleanupAuthorityLedger:
            _fail("cleanup_service", "cleanup service requires exact ledger authority")
        self.ledger = ledger
        self.authority_uid = _exact_nonnegative(authority_uid, "authority_uid")
        self.observer_uid = _exact_nonnegative(observer_supervisor_uid, "observer_uid")
        if recovery_uid is not None:
            _exact_nonnegative(recovery_uid, "recovery_uid")
        self.recovery_uid = recovery_uid
        self.service_generation_identity = service_generation_identity or hashlib.sha256(
            secrets.token_bytes(32)
        ).hexdigest()
        self.utc_now = utc_now or _utc_now
        self.recovery_controller = recovery_controller
        self._replay = ReplayWindow(self.service_generation_identity)

    def handle(self, request_value: dict[str, Any], peer: PeerProcess) -> MappingProxyType:
        request = validate_record(request_value, expected_schema=PRIVILEGED_REQUEST_SCHEMA)
        reconcile_peer(peer)
        if request.data["service_generation_identity"] not in (
            None, self.service_generation_identity,
        ):
            _fail("cleanup_generation", "cleanup request service generation differs")
        self._replay.claim(request)
        operation = request.data["operation"]
        if operation == "CLEANUP_STATE_QUERY":
            if (
                peer.credentials.uid != self.authority_uid
                or peer.cgroup != "/system.slice/ctr-slice7g-authority.service"
                or AUTHORITY_EXECUTABLE not in peer.argv
                or not peer.executable.startswith("/usr/bin/python3")
            ):
                _fail("cleanup_peer", "cleanup query peer UID differs")
            observed = self.ledger.reconstruct()
            return self._receipt(request, "OK", observed)
        if operation == "CLEANUP_REVISION_APPEND":
            if (
                peer.credentials.uid != self.observer_uid
                or peer.credentials.gid != 0
                or not peer.executable.startswith("/usr/bin/python3")
                or OBSERVER_SUPERVISOR_EXECUTABLE not in peer.argv
                or peer.cgroup != OBSERVER_SUPERVISOR_CGROUP
            ):
                _fail("cleanup_writer_peer", "normal cleanup writer is not the fixed root supervisor")
            current = self.ledger.reconstruct()
            if request.data["cleanup_head_identity"] != current.head.logical_identity:
                _fail("cleanup_head", "cleanup append targets a stale head")
            transition = request.data["transition"]
            if transition == "ACTIVE_UNBOUND":
                observed = self.ledger.begin_unbound(
                    runtime_authorization_identity=request.data["runtime_authorization_identity"],
                    budget_identity=request.data["budget_identity"],
                    service_generation_identity=request.data["service_generation_identity"],
                    session_binding_identity=request.data["session_binding_identity"],
                    phase=request.data["phase"],
                    phase_local_ordinal=request.data["phase_local_ordinal"],
                    transaction_observer_ordinal=request.data["transaction_observer_ordinal"],
                    domain_id=request.data["domain_id"],
                    observer_contract_identity=request.data["observer_contract_identity"],
                    timestamp=self.utc_now(),
                )
            elif transition == "ACTIVE_BOUND":
                observed = self.ledger.bind(
                    current,
                    containment_identity=request.data["containment_identity"],
                    process_identity=request.data["process_identity"],
                    timestamp=self.utc_now(),
                )
            elif transition in {"CLEARED", "QUARANTINED"}:
                observed = self.ledger.terminate(
                    current, state=transition,
                    disposition_identity=request.data["disposition_identity"],
                    recovery_authorization_identity=request.data["recovery_authorization_identity"],
                    timestamp=self.utc_now(),
                )
            else:
                _fail("cleanup_transition", "normal cleanup transition differs")
            return self._receipt(request, "OK", observed)
        if operation in {"RECOVERY_OBSERVE", "RECOVERY_COMMIT"}:
            if self.recovery_uid is None:
                _fail("recovery_unprovisioned", "numeric recovery authority is not provisioned")
            if peer.credentials.uid != self.recovery_uid:
                _fail("recovery_peer", "recovery peer UID differs")
            if self.recovery_controller is None:
                _fail("recovery_unprovisioned", "production recovery providers are unavailable")
            if operation == "RECOVERY_OBSERVE":
                observed = self.recovery_controller.observe(request)
                return self._receipt(request, "OK", observed)
            observed = self.recovery_controller.commit(request)
            return self._receipt(request, "RECOVERED", observed)
        _fail("cleanup_operation", "operation is not available on cleanup service")

    def _receipt(
        self, request: PrivilegedRecord, result: str, observation: CleanupLedgerObservation,
    ) -> MappingProxyType:
        value = {
            "schema_version": PRIVILEGED_RECEIPT_SCHEMA,
            "operation": request.data["operation"],
            "sequence": request.data["sequence"],
            "connection_nonce": request.data["connection_nonce"],
            "request_nonce": request.data["request_nonce"],
            "operation_token": request.data["operation_token"],
            "service_generation_identity": self.service_generation_identity,
            "result": result,
            "error_code": None,
            "cleanup_head_identity": observation.head.logical_identity,
            "containment_receipt_identity": None,
            "output_descriptor_count": 0,
            "payload_identity": observation.revision.logical_identity,
            "cleanup_revision": dict(observation.revision.data),
            "cleanup_anchor": dict(observation.anchor.data),
            "cleanup_head": dict(observation.head.data),
            "containment_receipt": None,
        }
        return validate_record(value, expected_schema=PRIVILEGED_RECEIPT_SCHEMA).data


class CleanupRecoveryController:
    """Private fixed-provider recovery evidence authority.

    Neither the authorization record nor provider results are accepted from a
    protocol mapping.  A future root provisioning task supplies a root-owned
    authorization loader and the four fixed helper providers.
    """

    def __init__(
        self, ledger: CleanupAuthorityLedger, *,
        authorization_loader: Callable[[str], dict[str, Any]],
        providers: dict[str, Callable[[MappingProxyType], RecoveryProviderEvidence]],
        service_generation_identity: str,
        utc_now: Callable[[], str] | None = None,
    ) -> None:
        if type(providers) is not dict or tuple(providers) != (
            "process", "dds", "lease", "graph",
        ):
            _fail("recovery_providers", "fixed recovery provider inventory differs")
        if not callable(authorization_loader) or any(not callable(item) for item in providers.values()):
            _fail("recovery_providers", "recovery providers must be private callables")
        self.ledger = ledger
        self.authorization_loader = authorization_loader
        self.providers = dict(providers)
        self.service_generation_identity = _digest(service_generation_identity)
        self.utc_now = utc_now or _utc_now
        self._pending: dict[str, tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any], str]] = {}

    def observe(self, request: PrivilegedRecord) -> CleanupLedgerObservation:
        current = self.ledger.reconstruct()
        if current.state != "QUARANTINED":
            _fail("recovery_state", "recovery observation requires quarantine")
        requested = request.data["recovery_authorization_identity"]
        if requested is None:
            _fail("recovery_authorization", "recovery authorization identity is missing")
        authorization_value = self.authorization_loader(requested)
        if type(authorization_value) is not dict:
            _fail("recovery_authorization", "root recovery loader returned a non-record")
        authorization = validate_record(
            authorization_value, expected_schema=CLEANUP_RECOVERY_AUTHORIZATION_V2_SCHEMA,
        )
        if authorization.logical_identity != requested:
            _fail("recovery_authorization", "loaded recovery authorization identity differs")
        if (
            authorization.data["quarantine_head_identity"] != current.head.logical_identity
            or authorization.data["quarantine_anchor_identity"] != current.anchor.logical_identity
            or authorization.data["cleanup_service_generation_identity"]
            != self.service_generation_identity
        ):
            _fail("recovery_binding", "recovery authorization binding differs")
        domain = current.revision.data["domain_id"]
        if type(domain) is not int:
            _fail("recovery_domain", "quarantine lacks an exact domain")
        receipts: list[dict[str, Any]] = []
        for ordinal, (name, provider) in enumerate(self.providers.items(), 1):
            evidence = provider(authorization.data)
            if type(evidence) is not RecoveryProviderEvidence or evidence.provider != name:
                _fail("recovery_provider", "fixed recovery provider result differs")
            clear = evidence.residual_count == 0 and (
                name != "lease" or evidence.lease_state == "CLEAR"
            )
            value = {
                "schema_version": CLEANUP_RECOVERY_PROVIDER_RECEIPT_SCHEMA,
                "provider": name,
                "provider_identity": _identity(
                    b"ctr-slice-7g-recovery-provider-canonical-1\0",
                    {"provider": name, "service_generation_identity": self.service_generation_identity},
                ),
                "recovery_nonce": authorization.data["recovery_nonce"],
                "quarantine_head_identity": current.head.logical_identity,
                "quarantine_anchor_identity": current.anchor.logical_identity,
                "recovery_authorization_identity": authorization.logical_identity,
                "service_generation_identity": self.service_generation_identity,
                "domain_id": domain,
                "runtime_authorization_identity": authorization.data["runtime_authorization_identity"],
                "budget_identity": authorization.data["budget_identity"],
                "phase": "RECOVERY",
                "ordinal": ordinal,
                "started_monotonic_ns": evidence.started_monotonic_ns,
                "ended_monotonic_ns": evidence.ended_monotonic_ns,
                "evidence_identity": evidence.evidence_identity,
                "clear": clear,
                "cleanup_disposition_identity": evidence.cleanup_disposition_identity,
            }
            receipts.append(dict(validate_record(
                value, expected_schema=CLEANUP_RECOVERY_PROVIDER_RECEIPT_SCHEMA,
            ).data))
        identities = [record_identity(
            value, expected_schema=CLEANUP_RECOVERY_PROVIDER_RECEIPT_SCHEMA,
        ) for value in receipts]
        all_clear = all(value["clear"] is True for value in receipts)
        observation_value = {
            "schema_version": CLEANUP_RECOVERY_OBSERVATION_SCHEMA,
            "recovery_nonce": authorization.data["recovery_nonce"],
            "quarantine_head_identity": current.head.logical_identity,
            "quarantine_anchor_identity": current.anchor.logical_identity,
            "recovery_authorization_identity": authorization.logical_identity,
            "runtime_authorization_identity": authorization.data["runtime_authorization_identity"],
            "budget_identity": authorization.data["budget_identity"],
            "service_generation_identity": self.service_generation_identity,
            "domain_id": domain,
            "provider_receipt_identities": identities,
            "all_sources_clear": all_clear,
            "observed_monotonic_ns": max(item["ended_monotonic_ns"] for item in receipts),
        }
        observation = validate_record(
            observation_value, expected_schema=CLEANUP_RECOVERY_OBSERVATION_SCHEMA,
        )
        if not all_clear:
            _fail("recovery_residual", "fresh recovery providers are not all clear")
        token = request.data["operation_token"]
        if token in self._pending:
            _fail("recovery_replay", "recovery operation token was already observed")
        self._pending[token] = (
            json.loads(authorization.canonical_bytes), receipts,
            json.loads(observation.canonical_bytes),
            current.head.logical_identity,
        )
        return current

    def commit(self, request: PrivilegedRecord) -> CleanupLedgerObservation:
        token = request.data["operation_token"]
        pending = self._pending.pop(token, None)
        if pending is None:
            _fail("recovery_replay", "recovery commit lacks fresh observed evidence")
        authorization, receipts, observation, expected_head = pending
        current = self.ledger.reconstruct()
        if current.head.logical_identity != expected_head:
            _fail("recovery_head", "quarantine head changed before recovery commit")
        return self.ledger._recover_authenticated(
            authorization, receipts, observation, timestamp=self.utc_now(),
            _internal_token=_RECOVERY_INTERNAL_TOKEN,
        )


class CleanupAuthorityRPCClient:
    """Fixed-socket client; exposes cleanup transitions, never ledger paths."""

    def __init__(
        self, *, socket_path: str = CLEANUP_AUTHORITY_SOCKET,
        _test: bool = False,
    ) -> None:
        if not _test and socket_path != CLEANUP_AUTHORITY_SOCKET:
            _fail("cleanup_socket_override", "production cleanup socket is fixed")
        self._socket_path = _absolute(socket_path)
        self._channel = socket.socket(
            socket.AF_UNIX, socket.SOCK_SEQPACKET | socket.SOCK_CLOEXEC,
        )
        self._channel.settimeout(15.0)
        try:
            self._channel.connect(self._socket_path)
        except OSError as exc:
            self.close()
            raise Slice7GCleanupAuthorityError("cleanup_connection", type(exc).__name__) from exc
        peer = observe_peer(peer_credentials(self._channel))
        if not _test and not _fixed_helper_peer(
            peer, "/usr/libexec/ctr-mppi/ctr-slice7g-cleanupd",
            "/system.slice/ctr-slice7g-cleanup-authority.service",
        ):
            self.close()
            _fail("cleanup_service_peer", "cleanup service peer identity differs")
        self._connection_nonce = secrets.token_hex(16)
        self._sequence = 0
        self._peer = peer
        self._service_generation_identity: str | None = None

    @classmethod
    def _for_test(cls, socket_path: str) -> "CleanupAuthorityRPCClient":
        return cls(socket_path=socket_path, _test=True)

    def close(self) -> None:
        channel = getattr(self, "_channel", None)
        if channel is not None:
            channel.close()
            self._channel = None

    def __enter__(self) -> "CleanupAuthorityRPCClient":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()

    def query(self, **bindings: Any) -> CleanupLedgerObservation:
        return self._transition("CLEANUP_STATE_QUERY", None, **bindings)

    def begin_unbound(self, **bindings: Any) -> CleanupLedgerObservation:
        return self._transition("CLEANUP_REVISION_APPEND", "ACTIVE_UNBOUND", **bindings)

    def bind(
        self, prior: CleanupLedgerObservation, *, containment_identity: str,
        process_identity: str, **bindings: Any,
    ) -> CleanupLedgerObservation:
        return self._transition(
            "CLEANUP_REVISION_APPEND", "ACTIVE_BOUND",
            cleanup_head_identity=prior.head.logical_identity,
            containment_identity=containment_identity,
            process_identity=process_identity,
            **bindings,
        )

    def terminate(
        self, prior: CleanupLedgerObservation, *, state: str,
        disposition_identity: str, recovery_authorization_identity: str | None = None,
        **bindings: Any,
    ) -> CleanupLedgerObservation:
        return self._transition(
            "CLEANUP_REVISION_APPEND", state,
            cleanup_head_identity=prior.head.logical_identity,
            disposition_identity=disposition_identity,
            recovery_authorization_identity=recovery_authorization_identity,
            **bindings,
        )

    def _transition(
        self, operation: str, transition: str | None, **bindings: Any,
    ) -> CleanupLedgerObservation:
        if self._channel is None:
            _fail("cleanup_connection", "cleanup service connection is closed")
        defaults = {
            "service_generation_identity": self._service_generation_identity,
            "runtime_authorization_identity": None,
            "installed_runtime_identity": None,
            "budget_identity": None,
            "cleanup_head_identity": None,
            "session_binding_identity": None,
            "domain_id": None,
            "phase": None,
            "phase_local_ordinal": None,
            "transaction_observer_ordinal": None,
            "observer_contract_identity": None,
            "containment_identity": None,
            "process_identity": None,
            "disposition_identity": None,
            "recovery_authorization_identity": None,
        }
        unknown = set(bindings) - set(defaults)
        if unknown:
            _fail("cleanup_client_fields", f"unknown cleanup binding fields: {sorted(unknown)!r}")
        defaults.update(bindings)
        request = {
            "schema_version": PRIVILEGED_REQUEST_SCHEMA,
            "operation": operation,
            "sequence": self._sequence,
            "connection_nonce": self._connection_nonce,
            "request_nonce": secrets.token_hex(16),
            "operation_token": secrets.token_hex(16),
            **defaults,
            "transition": transition,
        }
        send_packet(self._channel, request, expected_schema=PRIVILEGED_REQUEST_SCHEMA)
        response, descriptors = receive_packet(
            self._channel, expected_schema=PRIVILEGED_RECEIPT_SCHEMA,
            expected_descriptors=0,
        )
        generation = verify_response_binding(
            validate_record(request, expected_schema=PRIVILEGED_REQUEST_SCHEMA), response,
            expected_service_generation_identity=self._service_generation_identity,
            expected_descriptor_count=0, descriptors=descriptors, peer=self._peer,
        )
        self._service_generation_identity = generation
        self._sequence += 1
        if response.data["result"] == "ERROR":
            _fail(response.data["error_code"], "cleanup service rejected transition")
        records = (
            response.data["cleanup_revision"], response.data["cleanup_anchor"],
            response.data["cleanup_head"],
        )
        if any(item is None for item in records):
            _fail("cleanup_response", "cleanup response omits ledger records")
        observation = CleanupLedgerObservation(
            validate_record(dict(records[0]), expected_schema=CLEANUP_REVISION_SCHEMA),
            validate_record(dict(records[1]), expected_schema=CLEANUP_ANCHOR_SCHEMA),
            validate_record(dict(records[2]), expected_schema=CLEANUP_HEAD_SCHEMA),
        )
        if (
            response.data["cleanup_head_identity"] != observation.head.logical_identity
            or response.data["payload_identity"] != observation.revision.logical_identity
        ):
            _fail("cleanup_response_binding", "cleanup response record identities differ")
        if operation == "CLEANUP_REVISION_APPEND":
            if observation.state != transition:
                _fail("cleanup_response_binding", "cleanup response transition differs")
            for request_field, revision_field in (
                ("runtime_authorization_identity", "runtime_authorization_identity"),
                ("budget_identity", "budget_identity"),
                ("session_binding_identity", "session_binding_identity"),
                ("domain_id", "domain_id"),
                ("phase", "phase"),
                ("phase_local_ordinal", "phase_local_ordinal"),
                ("transaction_observer_ordinal", "transaction_observer_ordinal"),
            ):
                expected = request[request_field]
                if expected is not None and observation.revision.data[revision_field] != expected:
                    _fail("cleanup_response_binding", f"cleanup response {revision_field} differs")
        return observation


def serve_cleanup_authority(
    *, authority_uid: int, authority_gid: int, recovery_uid: int | None,
    recovery_gid: int | None,
) -> None:
    """Serve both fixed root-owned SOCK_SEQPACKET endpoints.

    Socket parents and the ledger must already have been provisioned.  This
    function never initializes production authority state.
    """
    ledger = CleanupAuthorityLedger()
    service = CleanupAuthorityService(
        ledger, authority_uid=authority_uid, recovery_uid=recovery_uid,
    )
    listeners: list[socket.socket] = []
    try:
        endpoints = ((CLEANUP_AUTHORITY_SOCKET, authority_gid),)
        if recovery_uid is not None and recovery_gid is not None:
            endpoints += ((CLEANUP_RECOVERY_SOCKET, recovery_gid),)
        for path, group in endpoints:
            if os.path.lexists(path):
                _fail("cleanup_socket_exists", "cleanup socket path already exists")
            listener = socket.socket(
                socket.AF_UNIX, socket.SOCK_SEQPACKET | socket.SOCK_CLOEXEC,
            )
            listener.bind(path)
            os.chown(path, 0, group)
            os.chmod(path, 0o660)
            listener.listen(8)
            listeners.append(listener)
        while True:
            ready, _, _ = select.select(listeners, (), ())
            for listener in ready:
                channel, _ = listener.accept()
                _serve_cleanup_connection(channel, service)
    finally:
        for listener in listeners:
            listener.close()
        ledger.close()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _serve_cleanup_connection(
    channel: socket.socket, service: CleanupAuthorityService,
) -> None:
    peer: PeerProcess | None = None
    active_binding: dict[str, Any] | None = None
    expected_sequence = 0
    connection_nonce: str | None = None
    primary: BaseException | None = None
    try:
        peer = observe_peer(peer_credentials(channel))
        for _ in range(128):
            request, descriptors = receive_packet(
                channel, expected_schema=PRIVILEGED_REQUEST_SCHEMA,
                expected_descriptors=0,
            )
            if descriptors or request.data["sequence"] != expected_sequence:
                _fail("cleanup_protocol_sequence", "cleanup request sequence differs")
            if connection_nonce is None:
                connection_nonce = request.data["connection_nonce"]
            elif request.data["connection_nonce"] != connection_nonce:
                _fail("cleanup_protocol_nonce", "cleanup connection nonce changed")
            expected_sequence += 1
            if request.data["operation"] == "CLEANUP_REVISION_APPEND":
                active_binding = dict(request.data)
            try:
                response = service.handle(dict(request.data), peer)
            except (Slice7GCleanupAuthorityError, Slice7GPrivilegedProtocolError) as exc:
                response = _error_receipt(request, service.service_generation_identity, exc.code)
            send_packet(channel, dict(response), expected_schema=PRIVILEGED_RECEIPT_SCHEMA)
    except EOFError:
        pass
    except BaseException as exc:
        primary = exc
    finally:
        if active_binding is not None and peer is not None:
            try:
                current = service.ledger.reconstruct()
                if current.state in {"ACTIVE_UNBOUND", "ACTIVE_BOUND"}:
                    disposition = _identity(
                        b"ctr-slice-7g-cleanup-disconnect-quarantine-canonical-1\0",
                        {
                            "connection_nonce": connection_nonce,
                            "head_identity": current.head.logical_identity,
                        },
                    )
                    service.ledger.terminate(
                        current, state="QUARANTINED",
                        disposition_identity=disposition,
                        timestamp=service.utc_now(),
                    )
            except BaseException as cleanup_error:
                if primary is not None and hasattr(primary, "add_note"):
                    primary.add_note(
                        "cleanup disconnect quarantine failed: "
                        + type(cleanup_error).__name__
                    )
        channel.close()
    if primary is not None:
        raise primary


def _error_receipt(
    request: PrivilegedRecord, generation: str, code: str,
) -> MappingProxyType:
    value = {
        "schema_version": PRIVILEGED_RECEIPT_SCHEMA,
        "operation": request.data["operation"],
        "sequence": request.data["sequence"],
        "connection_nonce": request.data["connection_nonce"],
        "request_nonce": request.data["request_nonce"],
        "operation_token": request.data["operation_token"],
        "service_generation_identity": generation,
        "result": "ERROR",
        "error_code": code,
        "cleanup_head_identity": None,
        "containment_receipt_identity": None,
        "output_descriptor_count": 0,
        "payload_identity": None,
        "cleanup_revision": None,
        "cleanup_anchor": None,
        "cleanup_head": None,
        "containment_receipt": None,
    }
    return validate_record(value, expected_schema=PRIVILEGED_RECEIPT_SCHEMA).data


def _fixed_helper_peer(peer: PeerProcess, script: str, cgroup: str) -> bool:
    return (
        peer.credentials.uid == 0
        and peer.cgroup == cgroup
        and script in peer.argv
        and peer.executable.startswith("/usr/bin/python3")
    )


class _nonblocking_lock:
    def __init__(self, descriptor: int) -> None:
        self.descriptor = descriptor

    def __enter__(self) -> None:
        try:
            fcntl.flock(self.descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise Slice7GCleanupAuthorityError("cleanup_busy", "cleanup ledger lock is busy") from exc

    def __exit__(self, *_: Any) -> None:
        fcntl.flock(self.descriptor, fcntl.LOCK_UN)


def _exact_names(descriptor: int, pattern: re.Pattern[str]) -> list[tuple[int, str]]:
    names = sorted(os.listdir(descriptor))
    result: list[tuple[int, str]] = []
    for name in names:
        match = pattern.fullmatch(name)
        if match is None:
            _fail("cleanup_inventory", "cleanup directory contains an unknown entry")
        result.append((int(match.group(1)), name))
    return result


def _open_directory_path(path: str) -> int:
    parts = PurePosixPath(path).parts
    if not parts or parts[0] != "/" or any(part in ("", ".", "..") for part in parts[1:]):
        _fail("cleanup_path", "cleanup path is not normalized absolute")
    current = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        for part in parts[1:]:
            child = os.open(
                part, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW,
                dir_fd=current,
            )
            os.close(current)
            current = child
        return current
    except BaseException:
        os.close(current)
        raise


def _open_directory_at(parent: int, name: str) -> int:
    return os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW, dir_fd=parent)


def _read_fd(descriptor: int, maximum: int) -> bytes:
    os.lseek(descriptor, 0, os.SEEK_SET)
    chunks: list[bytes] = []
    total = 0
    while True:
        chunk = os.read(descriptor, min(65_536, maximum + 1 - total))
        if not chunk:
            break
        chunks.append(chunk)
        total += len(chunk)
        if total > maximum:
            _fail("cleanup_record_size", "cleanup record exceeds maximum")
    return b"".join(chunks)


def _physical_tuple(info: os.stat_result) -> tuple[int, ...]:
    return (
        info.st_dev, info.st_ino, stat.S_IFMT(info.st_mode), stat.S_IMODE(info.st_mode),
        info.st_nlink, info.st_size, info.st_uid, info.st_gid, info.st_mtime_ns, info.st_ctime_ns,
    )


def _directory_identity(info: os.stat_result) -> tuple[int, ...]:
    return (
        info.st_dev, info.st_ino, stat.S_IFMT(info.st_mode), stat.S_IMODE(info.st_mode),
        info.st_nlink, info.st_uid, info.st_gid,
    )


def _identity(domain: bytes, value: dict[str, Any]) -> str:
    return hashlib.sha256(
        domain + json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def _absolute(value: str) -> str:
    if type(value) is not str or not value.startswith("/") or os.path.normpath(value) != value:
        _fail("cleanup_path", "cleanup path must be normalized absolute")
    return value


def _exact_dict(value: Any) -> dict[str, Any]:
    if type(value) is not dict:
        _fail("cleanup_record_type", "cleanup record must be exact dictionary")
    return dict(value)


def _exact_nonnegative(value: Any, name: str) -> int:
    if type(value) is not int or value < 0:
        _fail("cleanup_numeric_identity", f"{name} must be an exact nonnegative integer")
    return value


def _digest(value: Any) -> str:
    if (
        type(value) is not str or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        _fail("cleanup_digest", "value must be a lowercase SHA-256")
    return value


def _parse_utc(value: Any) -> datetime:
    if type(value) is not str or not value.endswith("Z"):
        _fail("cleanup_timestamp", "timestamp must be canonical UTC")
    try:
        parsed = datetime.fromisoformat(value[:-1] + "+00:00")
    except ValueError as exc:
        raise Slice7GCleanupAuthorityError("cleanup_timestamp", "timestamp is malformed") from exc
    if parsed.tzinfo != timezone.utc:
        _fail("cleanup_timestamp", "timestamp must be UTC")
    return parsed


def _fsync_path(path: str) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _fail(code: str, message: str) -> None:
    raise Slice7GCleanupAuthorityError(code, message)


__all__ = [
    "CleanupAuthorityLedger", "CleanupAuthorityService", "CleanupLedgerObservation",
    "Slice7GCleanupAuthorityError", "serve_cleanup_authority",
]
