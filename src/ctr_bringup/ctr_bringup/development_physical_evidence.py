"""Authenticated latest physical evidence for explicit development evaluation.

The public ROS state and tactile topics remain unchanged.  This module provides
one same-user, per-run, descriptor-authenticated memfd channel so the safety
process can evaluate the genuine simulator timestamp without depending on ROS
serialization or delivery latency.  It is unavailable unless the runner creates
an authenticated session root and explicitly selects the transport.
"""

from __future__ import annotations

from array import array
from dataclasses import dataclass
import hashlib
import hmac
import json
import math
import mmap
import os
from pathlib import Path
import re
import socket
import stat
import struct
import threading
import time


TRANSPORT_ROS = "ros"
TRANSPORT_AUTHENTICATED_SHARED_MEMORY = "authenticated_shared_memory"
TRANSPORT_VALUES = (TRANSPORT_ROS, TRANSPORT_AUTHENTICATED_SHARED_MEMORY)
PRODUCTION_HARDWARE_FRESHNESS_TIMEOUT_S = 0.10
SIMULATOR_PAPER_EVALUATION_FRESHNESS_TIMEOUT_S = 0.20
PHYSICAL_EVIDENCE_STABLE_READ_TIMEOUT_S = 0.050
PHYSICAL_EVIDENCE_STABLE_READ_POLL_S = 0.0005
TRANSPORT_ENV = "CTR_DEVELOPMENT_PHYSICAL_EVIDENCE_TRANSPORT"
SESSION_ENV = "CTR_DEVELOPMENT_PHYSICAL_EVIDENCE_SESSION"
ROOT_ENV = "CTR_DEVELOPMENT_PHYSICAL_EVIDENCE_ROOT"
SOCKET_FILENAME = "physical-evidence.sock"
SESSION_PATTERN = re.compile(r"[0-9a-f]{64}")

MAGIC = b"CTRPEV01"
SCHEMA_VERSION = 1
MAPPING_SIZE = 4096
_SEQUENCE = struct.Struct("<Q")
_PAYLOAD = struct.Struct("<8sII32sIIQQQQII6d6d3d6d")
_DIGEST_SIZE = hashlib.sha256().digest_size
RECORD_LENGTH = _PAYLOAD.size + _DIGEST_SIZE
_RECORD_OFFSET = _SEQUENCE.size

FLAG_SOURCE_VALID = 1 << 0
FLAG_SIMULATION = 1 << 1
FLAG_FRAME_VALID = 1 << 2
FLAG_PHYSICAL_COLLISION = 1 << 3
FLAG_SAFETY_MARGIN_VIOLATION = 1 << 4
FLAG_TACTILE_VALID = 1 << 5
FLAG_CONTACT = 1 << 6
FLAG_WARNING = 1 << 7
FLAG_STOP = 1 << 8
_ALL_FLAGS = (1 << 9) - 1

_HELLO_SCHEMA = "ctr-development-physical-evidence-hello-1"
_RECEIPT_SCHEMA = "ctr-development-physical-evidence-receipt-1"


class PhysicalEvidenceError(RuntimeError):
    """Stable fail-closed channel error."""


@dataclass(frozen=True, slots=True)
class PhysicalEvidenceRecord:
    session_id: str
    producer_pid: int
    producer_uid: int
    generated_sequence: int
    source_monotonic_ns: int
    source_stamp_ns: int
    command_sequence: int
    q: tuple[float, ...]
    q_dot: tuple[float, ...]
    tip_position: tuple[float, float, float]
    whole_backbone_physical_clearance_m: float
    whole_backbone_safety_clearance_m: float
    raw_tactile: float
    filtered_tactile: float
    tactile_force_n: float
    tactile_clearance_m: float
    tactile_region: int
    source_valid: bool
    simulation: bool
    frame_valid: bool
    physical_collision: bool
    safety_margin_violation: bool
    tactile_valid: bool
    contact: bool
    warning: bool
    stop: bool

    def __post_init__(self) -> None:
        _session_bytes(self.session_id)
        for name in (
            "producer_pid",
            "producer_uid",
            "generated_sequence",
            "source_monotonic_ns",
            "source_stamp_ns",
            "command_sequence",
            "tactile_region",
        ):
            value = getattr(self, name)
            if type(value) is not int or value < 0:
                raise TypeError(f"{name} must be a nonnegative exact int")
        if len(self.q) != 6 or len(self.q_dot) != 6:
            raise ValueError("physical evidence q and q_dot must each contain six values")
        numeric = (
            *self.q,
            *self.q_dot,
            *self.tip_position,
            self.whole_backbone_physical_clearance_m,
            self.whole_backbone_safety_clearance_m,
            self.raw_tactile,
            self.filtered_tactile,
            self.tactile_force_n,
            self.tactile_clearance_m,
        )
        if any(type(value) is not float or not math.isfinite(value) for value in numeric):
            raise TypeError("physical evidence numeric values must be finite exact floats")
        for name in (
            "source_valid",
            "simulation",
            "frame_valid",
            "physical_collision",
            "safety_margin_violation",
            "tactile_valid",
            "contact",
            "warning",
            "stop",
        ):
            if type(getattr(self, name)) is not bool:
                raise TypeError(f"{name} must be an exact bool")


def selected_transport(environment: dict[str, str] | None = None) -> str:
    values = os.environ if environment is None else environment
    value = values.get(TRANSPORT_ENV, TRANSPORT_ROS)
    if value not in TRANSPORT_VALUES:
        raise PhysicalEvidenceError("physical_evidence_transport_invalid")
    return value


def authenticated_session_from_environment(
    environment: dict[str, str] | None = None,
) -> tuple[Path, str]:
    values = os.environ if environment is None else environment
    if selected_transport(values) != TRANSPORT_AUTHENTICATED_SHARED_MEMORY:
        raise PhysicalEvidenceError("physical_evidence_transport_not_selected")
    root_text = values.get(ROOT_ENV, "")
    session_id = values.get(SESSION_ENV, "")
    if type(root_text) is not str or not root_text:
        raise PhysicalEvidenceError("physical_evidence_root_missing")
    _session_bytes(session_id)
    root = Path(root_text)
    _authenticate_root(root)
    return root, session_id


class PhysicalEvidenceProducer:
    """Single writer and authenticated descriptor broker."""

    def __init__(
        self,
        root: Path,
        session_id: str,
        *,
        expected_reader_token: str = "safety_supervisor_node",
    ) -> None:
        _authenticate_root(root)
        self.root = root
        self.session_id = session_id
        self._session_bytes = _session_bytes(session_id)
        self._expected_reader_token = _nonempty_token(expected_reader_token)
        self._stop = threading.Event()
        self._active_lock = threading.Lock()
        self._active_connection: socket.socket | None = None
        self._memfd = os.memfd_create("ctr-development-physical-evidence", os.MFD_CLOEXEC)
        os.fchmod(self._memfd, 0o600)
        os.ftruncate(self._memfd, MAPPING_SIZE)
        self._mapping = mmap.mmap(
            self._memfd,
            MAPPING_SIZE,
            flags=mmap.MAP_SHARED,
            prot=mmap.PROT_READ | mmap.PROT_WRITE,
        )
        self._socket_path = root / SOCKET_FILENAME
        if self._socket_path.exists() or self._socket_path.is_symlink():
            self.close()
            raise PhysicalEvidenceError("physical_evidence_socket_exists")
        self._server = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self._server.settimeout(0.25)
        try:
            self._server.bind(str(self._socket_path))
        except BaseException:
            self.close()
            raise
        os.chmod(self._socket_path, 0o600)
        socket_stat = os.lstat(self._socket_path)
        if not stat.S_ISSOCK(socket_stat.st_mode) or socket_stat.st_uid != os.getuid():
            self.close()
            raise PhysicalEvidenceError("physical_evidence_socket_identity_invalid")
        self._socket_identity = (socket_stat.st_dev, socket_stat.st_ino)
        self._server.listen(1)
        self._broker = threading.Thread(
            target=self._serve,
            name="ctr-physical-evidence-broker",
            daemon=False,
        )
        self._broker.start()

    @classmethod
    def from_environment(cls) -> "PhysicalEvidenceProducer":
        root, session_id = authenticated_session_from_environment()
        return cls(root, session_id)

    def write(self, record: PhysicalEvidenceRecord) -> None:
        if type(record) is not PhysicalEvidenceRecord:
            raise TypeError("physical evidence writer requires an exact record")
        if record.session_id != self.session_id:
            raise PhysicalEvidenceError("physical_evidence_session_mismatch")
        if record.producer_pid != os.getpid() or record.producer_uid != os.getuid():
            raise PhysicalEvidenceError("physical_evidence_producer_identity_mismatch")
        payload = _pack_record(record)
        generation = _SEQUENCE.unpack_from(self._mapping, 0)[0]
        if generation & 1:
            generation += 1
        _SEQUENCE.pack_into(self._mapping, 0, generation + 1)
        self._mapping[_RECORD_OFFSET : _RECORD_OFFSET + len(payload)] = payload
        _SEQUENCE.pack_into(self._mapping, 0, generation + 2)

    def _serve(self) -> None:
        while not self._stop.is_set():
            try:
                connection, _address = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                continue
            try:
                self._serve_connection(connection)
            except Exception:
                # One malformed or unauthorized connection must not destroy
                # the single-purpose broker.  The peer receives no descriptor
                # and the producer remains available to the expected reader.
                pass
            finally:
                with self._active_lock:
                    if self._active_connection is connection:
                        self._active_connection = None
                connection.close()

    def _serve_connection(self, connection: socket.socket) -> None:
        pid, uid, _gid = _peer_credentials(connection)
        if uid != os.getuid() or not _process_command_contains(
            pid, self._expected_reader_token
        ):
            raise PhysicalEvidenceError("physical_evidence_reader_identity_rejected")
        packet = connection.recv(4096)
        hello = _closed_json(
            packet,
            expected_keys={"schema", "role", "session_id"},
        )
        if hello != {
            "schema": _HELLO_SCHEMA,
            "role": "safety_reader",
            "session_id": self.session_id,
        }:
            raise PhysicalEvidenceError("physical_evidence_reader_handshake_rejected")
        with self._active_lock:
            if self._active_connection is not None:
                raise PhysicalEvidenceError("physical_evidence_reader_already_connected")
            self._active_connection = connection
        read_fd = os.open(
            f"/proc/self/fd/{self._memfd}", os.O_RDONLY | os.O_CLOEXEC
        )
        try:
            member = os.fstat(read_fd)
            receipt = _canonical_json(
                {
                    "schema": _RECEIPT_SCHEMA,
                    "session_id": self.session_id,
                    "producer_pid": os.getpid(),
                    "producer_uid": os.getuid(),
                    "device": member.st_dev,
                    "inode": member.st_ino,
                    "mode": stat.S_IMODE(member.st_mode),
                    "link_count": member.st_nlink,
                    "size": member.st_size,
                }
            )
            sent = connection.sendmsg(
                [receipt],
                [(socket.SOL_SOCKET, socket.SCM_RIGHTS, array("i", [read_fd]))],
            )
            if sent != len(receipt):
                raise PhysicalEvidenceError("physical_evidence_receipt_short_send")
        finally:
            os.close(read_fd)
        connection.settimeout(0.25)
        while not self._stop.is_set():
            try:
                unexpected = connection.recv(1)
            except socket.timeout:
                continue
            if not unexpected:
                return
            raise PhysicalEvidenceError("physical_evidence_control_packet_unexpected")

    def close(self) -> None:
        stop = getattr(self, "_stop", None)
        if stop is not None:
            stop.set()
        active_lock = getattr(self, "_active_lock", None)
        if active_lock is not None:
            with active_lock:
                active = getattr(self, "_active_connection", None)
                self._active_connection = None
            if active is not None:
                active.close()
        server = getattr(self, "_server", None)
        if server is not None:
            server.close()
            self._server = None
        broker = getattr(self, "_broker", None)
        if broker is not None and broker is not threading.current_thread():
            broker.join(timeout=1.0)
            self._broker = None
        mapping = getattr(self, "_mapping", None)
        if mapping is not None:
            mapping.close()
            self._mapping = None
        memfd = getattr(self, "_memfd", -1)
        if memfd >= 0:
            os.close(memfd)
            self._memfd = -1
        socket_path = getattr(self, "_socket_path", None)
        identity = getattr(self, "_socket_identity", None)
        if socket_path is not None and identity is not None:
            try:
                current = os.lstat(socket_path)
            except FileNotFoundError:
                return
            if (current.st_dev, current.st_ino) != identity or not stat.S_ISSOCK(
                current.st_mode
            ):
                raise PhysicalEvidenceError("physical_evidence_socket_replaced")
            os.unlink(socket_path)


class PhysicalEvidenceReader:
    """Authenticated read-only safety mapping and live control connection."""

    def __init__(
        self,
        root: Path,
        session_id: str,
        *,
        expected_producer_token: str = "simulator_node",
        connect_timeout_s: float = 10.0,
    ) -> None:
        _authenticate_root(root)
        self.session_id = session_id
        self._session_bytes = _session_bytes(session_id)
        expected_producer_token = _nonempty_token(expected_producer_token)
        deadline = time.monotonic() + connect_timeout_s
        self._control = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        socket_path = root / SOCKET_FILENAME
        while True:
            try:
                self._control.connect(str(socket_path))
                break
            except (FileNotFoundError, ConnectionRefusedError):
                if time.monotonic() >= deadline:
                    self.close()
                    raise PhysicalEvidenceError("physical_evidence_connect_timeout")
                time.sleep(0.01)
        pid, uid, _gid = _peer_credentials(self._control)
        if uid != os.getuid() or not _process_or_parent_command_contains(
            pid, expected_producer_token
        ):
            self.close()
            raise PhysicalEvidenceError("physical_evidence_producer_identity_rejected")
        hello = _canonical_json(
            {
                "schema": _HELLO_SCHEMA,
                "role": "safety_reader",
                "session_id": session_id,
            }
        )
        if self._control.send(hello) != len(hello):
            self.close()
            raise PhysicalEvidenceError("physical_evidence_hello_short_send")
        packet, ancillary, flags, _address = self._control.recvmsg(
            4096, socket.CMSG_SPACE(array("i", [0]).itemsize * 2)
        )
        received_fds: list[int] = []
        try:
            if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
                raise PhysicalEvidenceError("physical_evidence_receipt_truncated")
            for level, kind, data in ancillary:
                if level != socket.SOL_SOCKET or kind != socket.SCM_RIGHTS:
                    raise PhysicalEvidenceError("physical_evidence_ancillary_unknown")
                values = array("i")
                values.frombytes(data[: len(data) - len(data) % values.itemsize])
                received_fds.extend(values)
            if len(received_fds) != 1:
                raise PhysicalEvidenceError("physical_evidence_descriptor_count_invalid")
            receipt = _closed_json(
                packet,
                expected_keys={
                    "schema",
                    "session_id",
                    "producer_pid",
                    "producer_uid",
                    "device",
                    "inode",
                    "mode",
                    "link_count",
                    "size",
                },
            )
            descriptor = received_fds.pop()
            member = os.fstat(descriptor)
            expected = {
                "schema": _RECEIPT_SCHEMA,
                "session_id": session_id,
                "producer_pid": pid,
                "producer_uid": uid,
                "device": member.st_dev,
                "inode": member.st_ino,
                "mode": stat.S_IMODE(member.st_mode),
                "link_count": member.st_nlink,
                "size": member.st_size,
            }
            if receipt != expected:
                raise PhysicalEvidenceError("physical_evidence_receipt_identity_mismatch")
            if (
                not stat.S_ISREG(member.st_mode)
                or member.st_size != MAPPING_SIZE
                or member.st_nlink != 0
                or stat.S_IMODE(member.st_mode) != 0o600
            ):
                raise PhysicalEvidenceError("physical_evidence_descriptor_identity_invalid")
            self.producer_pid = pid
            self.producer_uid = uid
            self._descriptor = descriptor
            self._mapping = mmap.mmap(
                descriptor,
                MAPPING_SIZE,
                flags=mmap.MAP_SHARED,
                prot=mmap.PROT_READ,
            )
            self._last_sequence = 0
            self._last_source_stamp_ns = 0
        except BaseException:
            for descriptor in received_fds:
                os.close(descriptor)
            self.close()
            raise

    @classmethod
    def from_environment(cls) -> "PhysicalEvidenceReader":
        root, session_id = authenticated_session_from_environment()
        return cls(root, session_id)

    def read(self) -> PhysicalEvidenceRecord:
        if not self.producer_alive():
            raise PhysicalEvidenceError("physical_evidence_producer_disconnected")
        deadline = time.monotonic() + PHYSICAL_EVIDENCE_STABLE_READ_TIMEOUT_S
        while True:
            generation_before = _SEQUENCE.unpack_from(self._mapping, 0)[0]
            if generation_before == 0:
                raise PhysicalEvidenceError("physical_evidence_unavailable")
            if generation_before & 1:
                if time.monotonic() >= deadline:
                    break
                time.sleep(PHYSICAL_EVIDENCE_STABLE_READ_POLL_S)
                continue
            raw = bytes(
                self._mapping[
                    _RECORD_OFFSET : _RECORD_OFFSET + RECORD_LENGTH
                ]
            )
            generation_after = _SEQUENCE.unpack_from(self._mapping, 0)[0]
            if generation_before != generation_after or generation_after & 1:
                if time.monotonic() >= deadline:
                    break
                time.sleep(PHYSICAL_EVIDENCE_STABLE_READ_POLL_S)
                continue
            record = _unpack_record(raw)
            if record.session_id != self.session_id:
                raise PhysicalEvidenceError("physical_evidence_session_mismatch")
            if (
                record.producer_pid != self.producer_pid
                or record.producer_uid != self.producer_uid
            ):
                raise PhysicalEvidenceError("physical_evidence_producer_identity_mismatch")
            if record.generated_sequence < self._last_sequence:
                raise PhysicalEvidenceError("physical_evidence_sequence_rollback")
            if record.source_stamp_ns < self._last_source_stamp_ns:
                raise PhysicalEvidenceError("physical_evidence_timestamp_rollback")
            if record.generated_sequence > self._last_sequence:
                self._last_sequence = record.generated_sequence
                self._last_source_stamp_ns = record.source_stamp_ns
            elif record.source_stamp_ns != self._last_source_stamp_ns:
                raise PhysicalEvidenceError("physical_evidence_duplicate_sequence_changed")
            return record
        raise PhysicalEvidenceError("physical_evidence_torn_read")

    def producer_alive(self) -> bool:
        control = getattr(self, "_control", None)
        if control is None:
            return False
        try:
            packet = control.recv(1, socket.MSG_PEEK | socket.MSG_DONTWAIT)
        except BlockingIOError:
            return True
        except OSError:
            return False
        return bool(packet)

    def close(self) -> None:
        mapping = getattr(self, "_mapping", None)
        if mapping is not None:
            mapping.close()
            self._mapping = None
        descriptor = getattr(self, "_descriptor", -1)
        if descriptor >= 0:
            os.close(descriptor)
            self._descriptor = -1
        control = getattr(self, "_control", None)
        if control is not None:
            control.close()
            self._control = None


def _pack_record(record: PhysicalEvidenceRecord) -> bytes:
    flags = 0
    for enabled, flag in (
        (record.source_valid, FLAG_SOURCE_VALID),
        (record.simulation, FLAG_SIMULATION),
        (record.frame_valid, FLAG_FRAME_VALID),
        (record.physical_collision, FLAG_PHYSICAL_COLLISION),
        (record.safety_margin_violation, FLAG_SAFETY_MARGIN_VIOLATION),
        (record.tactile_valid, FLAG_TACTILE_VALID),
        (record.contact, FLAG_CONTACT),
        (record.warning, FLAG_WARNING),
        (record.stop, FLAG_STOP),
    ):
        if enabled:
            flags |= flag
    payload = _PAYLOAD.pack(
        MAGIC,
        SCHEMA_VERSION,
        RECORD_LENGTH,
        _session_bytes(record.session_id),
        record.producer_pid,
        record.producer_uid,
        record.generated_sequence,
        record.source_monotonic_ns,
        record.source_stamp_ns,
        record.command_sequence,
        flags,
        record.tactile_region,
        *record.q,
        *record.q_dot,
        *record.tip_position,
        record.whole_backbone_physical_clearance_m,
        record.whole_backbone_safety_clearance_m,
        record.raw_tactile,
        record.filtered_tactile,
        record.tactile_force_n,
        record.tactile_clearance_m,
    )
    return payload + hashlib.sha256(payload).digest()


def _unpack_record(raw: bytes) -> PhysicalEvidenceRecord:
    if type(raw) is not bytes or len(raw) != RECORD_LENGTH:
        raise PhysicalEvidenceError("physical_evidence_record_length_invalid")
    payload, digest = raw[:-_DIGEST_SIZE], raw[-_DIGEST_SIZE:]
    if not hmac.compare_digest(hashlib.sha256(payload).digest(), digest):
        raise PhysicalEvidenceError("physical_evidence_integrity_invalid")
    values = _PAYLOAD.unpack(payload)
    if values[0] != MAGIC or values[1] != SCHEMA_VERSION or values[2] != RECORD_LENGTH:
        raise PhysicalEvidenceError("physical_evidence_schema_invalid")
    flags = values[10]
    if flags & ~_ALL_FLAGS:
        raise PhysicalEvidenceError("physical_evidence_flags_invalid")
    q_start = 12
    q = tuple(float(value) for value in values[q_start : q_start + 6])
    q_dot = tuple(float(value) for value in values[q_start + 6 : q_start + 12])
    tip = tuple(float(value) for value in values[q_start + 12 : q_start + 15])
    scalars = values[q_start + 15 : q_start + 21]
    return PhysicalEvidenceRecord(
        session_id=values[3].hex(),
        producer_pid=values[4],
        producer_uid=values[5],
        generated_sequence=values[6],
        source_monotonic_ns=values[7],
        source_stamp_ns=values[8],
        command_sequence=values[9],
        q=q,
        q_dot=q_dot,
        tip_position=tip,
        whole_backbone_physical_clearance_m=float(scalars[0]),
        whole_backbone_safety_clearance_m=float(scalars[1]),
        raw_tactile=float(scalars[2]),
        filtered_tactile=float(scalars[3]),
        tactile_force_n=float(scalars[4]),
        tactile_clearance_m=float(scalars[5]),
        tactile_region=values[11],
        source_valid=bool(flags & FLAG_SOURCE_VALID),
        simulation=bool(flags & FLAG_SIMULATION),
        frame_valid=bool(flags & FLAG_FRAME_VALID),
        physical_collision=bool(flags & FLAG_PHYSICAL_COLLISION),
        safety_margin_violation=bool(flags & FLAG_SAFETY_MARGIN_VIOLATION),
        tactile_valid=bool(flags & FLAG_TACTILE_VALID),
        contact=bool(flags & FLAG_CONTACT),
        warning=bool(flags & FLAG_WARNING),
        stop=bool(flags & FLAG_STOP),
    )


def _session_bytes(session_id: str) -> bytes:
    if type(session_id) is not str or SESSION_PATTERN.fullmatch(session_id) is None:
        raise PhysicalEvidenceError("physical_evidence_session_invalid")
    return bytes.fromhex(session_id)


def _authenticate_root(root: Path) -> os.stat_result:
    if not isinstance(root, Path) or not root.is_absolute():
        raise PhysicalEvidenceError("physical_evidence_root_invalid")
    member = os.lstat(root)
    if (
        not stat.S_ISDIR(member.st_mode)
        or stat.S_ISLNK(member.st_mode)
        or member.st_uid != os.getuid()
        or stat.S_IMODE(member.st_mode) != 0o700
        or root.resolve(strict=True) != root
    ):
        raise PhysicalEvidenceError("physical_evidence_root_identity_invalid")
    return member


def _peer_credentials(connection: socket.socket) -> tuple[int, int, int]:
    try:
        raw = connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12)
        pid, uid, gid = struct.unpack("3i", raw)
    except (OSError, struct.error) as exc:
        raise PhysicalEvidenceError("physical_evidence_peer_credentials_unavailable") from exc
    if pid <= 0 or uid < 0 or gid < 0:
        raise PhysicalEvidenceError("physical_evidence_peer_credentials_invalid")
    return pid, uid, gid


def _process_command_contains(pid: int, token: str) -> bool:
    try:
        raw = (Path("/proc") / str(pid) / "cmdline").read_bytes()
    except OSError:
        return False
    return token.encode("utf-8") in raw.split(b"\0") or token.encode("utf-8") in raw


def _process_or_parent_command_contains(pid: int, token: str) -> bool:
    """Authenticate a multiprocessing worker through its fixed node parent.

    Python's spawn helper replaces the child's visible argv with
    ``multiprocessing.spawn``.  The physical source is nevertheless an exact
    direct child of the fixed simulator entry point.  Bind that relationship
    at connection time instead of weakening the role check to generic Python.
    """

    if _process_command_contains(pid, token):
        return True
    try:
        status = (Path("/proc") / str(pid) / "status").read_text(
            encoding="utf-8", errors="strict"
        )
    except (OSError, UnicodeError):
        return False
    parent_lines = [line for line in status.splitlines() if line.startswith("PPid:")]
    if len(parent_lines) != 1:
        return False
    fields = parent_lines[0].split()
    if len(fields) != 2 or not fields[1].isdigit():
        return False
    parent_pid = int(fields[1])
    return parent_pid > 1 and _process_command_contains(parent_pid, token)


def _closed_json(payload: bytes, *, expected_keys: set[str]) -> dict:
    if type(payload) is not bytes or not payload or len(payload) > 4096:
        raise PhysicalEvidenceError("physical_evidence_control_frame_invalid")
    try:
        text = payload.decode("utf-8", errors="strict")
        result = json.loads(text)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PhysicalEvidenceError("physical_evidence_control_json_invalid") from exc
    if type(result) is not dict or set(result) != expected_keys:
        raise PhysicalEvidenceError("physical_evidence_control_schema_invalid")
    if _canonical_json(result) != payload:
        raise PhysicalEvidenceError("physical_evidence_control_noncanonical")
    return result


def _canonical_json(value: dict) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _nonempty_token(value: str) -> str:
    if type(value) is not str or not value or "\0" in value:
        raise ValueError("peer command token must be a nonempty string")
    return value
