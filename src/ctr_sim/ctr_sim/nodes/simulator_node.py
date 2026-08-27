"""ROS2 simulation loop for Milestone 3."""

from __future__ import annotations

from dataclasses import dataclass, replace
import json
import math
import gc
import multiprocessing
import os
import pickle
from pathlib import Path
import re
import signal
import stat
import threading
import time
from typing import Iterable

import numpy as np

import rclpy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as NavPath
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ctr_bringup.parameter_validation import (
    load_parameter_files,
    parse_launch_bool,
    validate_config_paths,
    validate_or_raise,
)
from ctr_bringup.development_physical_evidence import (
    PhysicalEvidenceProducer,
    PhysicalEvidenceRecord,
    TRANSPORT_AUTHENTICATED_SHARED_MEMORY,
    TRANSPORT_ROS,
    TRANSPORT_VALUES,
    selected_transport,
)
from ctr_bringup.slice_7g_profile import (
    apply_slice_7g_development_simulation_profile,
    apply_slice_7g_simulation_profile,
)
from ctr_interfaces.msg import CtrBackbone, CtrJointCommand, CtrJointState, CtrState, CtrTactileState
from ctr_model.approximate_model import ApproximateCTRModel
from ctr_mppi_controller.curved_lumen import CurvedLumen
from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen, goal_position_from_config
from ctr_mppi_controller.lumen_factory import (
    config_with_lumen_overrides,
    lumen_geometry_fingerprint,
    lumen_geometry_log_line,
    lumen_geometry_from_config,
    lumen_mode_from_config,
)
from ctr_mppi_controller.nodes.reference_manager_node import reference_path_qos_profile
from ctr_sim.lumen_markers import (
    BoundedTipTrajectory,
    build_actual_tip_path_marker,
    build_dynamic_lumen_delete_markers,
    build_dynamic_lumen_diagnostic_markers,
    LumenMarkerConfig,
    build_curved_static_lumen_markers,
    build_reference_path_markers,
    build_static_lumen_delete_markers,
    marker_keys,
    markers_with_stamp,
    static_lumen_cache_key,
)
from ctr_sim.lumen_diagnostics import LumenRuntimeDiagnostic, build_lumen_runtime_diagnostic
from ctr_sim.simulation_core import CTRSimulationCore
from ctr_tactile.simulated_tactile import SimulatedTactileParameters, simulate_tactile
from ctr_tactile.tactile_processing import TactileProcessor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


_STATIC_DEVELOPMENT_MARKER_NAMESPACES = frozenset(
    {"lumen_surface", "lumen_wireframe", "lumen_centerline"}
)
_DEVELOPMENT_EVALUATION_CPU_PARTITION_ENV = "CTR_DEVELOPMENT_EVALUATION_CPU_PARTITION"
_DEVELOPMENT_SIMULATOR_CPU_LIST_ENV = "CTR_DEVELOPMENT_SIMULATOR_CPU_LIST"
_EVAL007_TIMING_TRACE_ROOT_ENV = "CTR_EVAL007_TIMING_TRACE_ROOT"


class _BoundedProcessTimingTrace:
    """One-writer fixed-width trace that never waits on a file or ROS path.

    The trace is diagnostic evidence only. It uses fixed-size shared integer
    arrays so the physical source never queues or serializes a trace record,
    acquires a process lock, or waits for a trace consumer. Readers inspect it
    only after the owning process has stopped.
    """

    _CAPACITY = 8192

    def __init__(self, context, fields: tuple[str, ...]) -> None:
        if not fields or len(set(fields)) != len(fields):
            raise ValueError("timing trace fields must be nonempty and unique")
        self.fields = tuple(fields)
        self._values = tuple(
            context.RawArray("Q", self._CAPACITY) for _field in self.fields
        )
        self._slot_generations = context.RawArray("Q", self._CAPACITY)
        self._write_count = context.RawValue("Q", 0)

    def append(self, values: tuple[int, ...]) -> None:
        if len(values) != len(self.fields) or any(
            type(value) is not int or value < 0 for value in values
        ):
            raise TypeError("timing trace requires one nonnegative exact int per field")
        absolute_index = int(self._write_count.value)
        slot = absolute_index % self._CAPACITY
        generation = int(self._slot_generations[slot])
        if generation & 1:
            generation += 1
        self._slot_generations[slot] = generation + 1
        for column, value in zip(self._values, values, strict=True):
            column[slot] = value
        self._slot_generations[slot] = generation + 2
        self._write_count.value = absolute_index + 1

    def snapshot(self) -> dict[str, object]:
        """Return the stable retained suffix after the writer has terminated."""

        write_count = int(self._write_count.value)
        retained = min(write_count, self._CAPACITY)
        first = write_count - retained
        rows: list[list[int]] = []
        for absolute_index in range(first, write_count):
            slot = absolute_index % self._CAPACITY
            generation_before = int(self._slot_generations[slot])
            if generation_before == 0 or generation_before & 1:
                raise RuntimeError("timing trace contains an incomplete slot")
            row = [int(column[slot]) for column in self._values]
            generation_after = int(self._slot_generations[slot])
            if generation_before != generation_after or generation_after & 1:
                raise RuntimeError("timing trace changed during reconstruction")
            rows.append(row)
        return {
            "schema": "ctr-eval007-bounded-process-timing-trace-1",
            "fields": list(self.fields),
            "capacity": self._CAPACITY,
            "write_count": write_count,
            "overwritten_count": max(0, write_count - self._CAPACITY),
            "rows": rows,
        }


_PHYSICAL_SOURCE_TRACE_FIELDS = (
    "generated_sequence",
    "source_stamp_ns",
    "expected_monotonic_ns",
    "generation_entry_monotonic_ns",
    "generation_exit_monotonic_ns",
    "generation_thread_cpu_ns",
    "generation_process_cpu_ns",
    "mailbox_commit_begin_monotonic_ns",
    "mailbox_commit_end_monotonic_ns",
    "mailbox_commit_thread_cpu_ns",
    "shared_commit_begin_monotonic_ns",
    "shared_commit_end_monotonic_ns",
    "command_sequence",
    "committed_sequence",
    "mailbox_version",
    "schedule_missed_periods",
)
_PUBLICATION_TRACE_FIELDS = (
    "observed_sequence",
    "source_stamp_ns",
    "mailbox_version",
    "observation_monotonic_ns",
    "publish_begin_monotonic_ns",
    "publish_end_monotonic_ns",
    "skipped_source_sequences",
    "publisher_pid",
)


def development_marker_qos_profile(namespace: str) -> QoSProfile:
    """Keep static visual geometry available to RViz without periodic large republishes."""

    static = namespace in _STATIC_DEVELOPMENT_MARKER_NAMESPACES
    return QoSProfile(
        depth=1 if static else 10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL if static else DurabilityPolicy.VOLATILE
        ),
    )


def source_evidence_qos_profile(*, reliable: bool = True) -> QoSProfile:
    """Keep the latest reliable physical sample without a publisher backlog."""

    return QoSProfile(
        depth=1,
        reliability=(
            ReliabilityPolicy.RELIABLE
            if reliable
            else ReliabilityPolicy.BEST_EFFORT
        ),
        durability=DurabilityPolicy.VOLATILE,
    )


@dataclass(frozen=True, slots=True)
class _PhysicalTactileSample:
    raw_signal: float
    filtered_signal: float
    force_n: float
    clearance_m: float
    contact: bool
    warning: bool
    stop: bool
    valid: bool
    diagnostic_status: str
    region: int


@dataclass(frozen=True, slots=True)
class _PhysicalSample:
    """Immutable physical-source result handed to non-authoritative publishers."""

    sequence: int
    stamp_sec: int
    stamp_nanosec: int
    expected_monotonic_s: float
    source_start_monotonic_s: float
    source_complete_monotonic_s: float
    source_lateness_s: float
    source_duration_s: float
    q: tuple[float, ...]
    q_dot: tuple[float, ...]
    tip_position: tuple[float, float, float]
    backbone_points: tuple[tuple[float, float, float], ...]
    command_age_s: float
    command_saturated: bool
    command_valid: bool
    diagnostic_status: str
    model_status: str
    tactile: _PhysicalTactileSample | None
    source_timing: tuple[tuple[str, float | int], ...]

    def stamp(self) -> Time:
        return Time(sec=self.stamp_sec, nanosec=self.stamp_nanosec)


class _LatestPhysicalSampleMailbox:
    """A bounded immutable latest-value handoff with no publication backpressure."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._sample: _PhysicalSample | None = None
        self._version = 0
        self._closed = False

    def put(self, sample: _PhysicalSample) -> int:
        if type(sample) is not _PhysicalSample:
            raise TypeError("mailbox accepts exact _PhysicalSample values")
        with self._condition:
            if self._closed:
                raise RuntimeError("mailbox is closed")
            self._sample = sample
            self._version += 1
            version = self._version
            # Every publication process owns an independent read cursor over
            # this one immutable latest-value slot.
            self._condition.notify_all()
            return version

    def take_after(
        self,
        version: int,
        stop_event: threading.Event,
    ) -> tuple[int, _PhysicalSample] | None:
        with self._condition:
            while not self._closed and self._version <= version:
                if stop_event.is_set():
                    return None
                self._condition.wait(timeout=0.25)
            if self._closed or stop_event.is_set():
                return None
            sample = self._sample
            if sample is None:
                raise RuntimeError("mailbox version has no sample")
            return self._version, sample

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()


class _LatestProcessPhysicalSampleMailbox:
    """Lock-free latest sample shared by one source and independent readers.

    The source is the sole writer. It writes the inactive member of a
    two-buffer seqlock and publishes the new active index only after the bytes
    are complete. Readers retry a changing generation. A slow reader may skip
    superseded samples, but it cannot observe torn bytes or block the source.
    """

    _CAPACITY_BYTES = 262_144

    def __init__(self, context) -> None:
        self._buffers = (
            context.RawArray("B", self._CAPACITY_BYTES),
            context.RawArray("B", self._CAPACITY_BYTES),
        )
        self._lengths = (context.RawValue("I", 0), context.RawValue("I", 0))
        self._generations = (
            context.RawValue("Q", 0),
            context.RawValue("Q", 0),
        )
        self._active = context.RawValue("B", 0)
        self._version = context.RawValue("Q", 0)
        self._closed = context.RawValue("b", 0)

    def put(self, sample: _PhysicalSample) -> int:
        if type(sample) is not _PhysicalSample:
            raise TypeError("mailbox accepts exact _PhysicalSample values")
        payload = pickle.dumps(sample, protocol=5)
        if len(payload) > self._CAPACITY_BYTES:
            raise RuntimeError("physical sample exceeds mailbox capacity")
        if self._closed.value:
            raise RuntimeError("mailbox is closed")
        slot = 1 - int(self._active.value)
        generation = int(self._generations[slot].value)
        if generation & 1:
            generation += 1
        self._generations[slot].value = generation + 1
        self._buffers[slot][: len(payload)] = payload
        self._lengths[slot].value = len(payload)
        self._generations[slot].value = generation + 2
        version = int(self._version.value) + 1
        self._active.value = slot
        self._version.value = version
        return version

    def take_after(self, version: int, stop_event) -> tuple[int, _PhysicalSample] | None:
        while not self._closed.value and not stop_event.is_set():
            delivery = self.snapshot_after(version)
            if delivery is None:
                stop_event.wait(0.001)
                continue
            return delivery
        return None

    def snapshot_after(self, version: int) -> tuple[int, _PhysicalSample] | None:
        """Return one stable latest snapshot without waiting or consuming it."""

        for _attempt in range(8):
            next_version = int(self._version.value)
            if self._closed.value or next_version <= version:
                return None
            slot = int(self._active.value)
            generation_before = int(self._generations[slot].value)
            if generation_before & 1:
                continue
            length = int(self._lengths[slot].value)
            if length <= 0 or length > self._CAPACITY_BYTES:
                continue
            payload = bytes(self._buffers[slot][:length])
            generation_after = int(self._generations[slot].value)
            if (
                generation_before != generation_after
                or generation_after & 1
                or slot != int(self._active.value)
                or next_version != int(self._version.value)
            ):
                continue
            sample = pickle.loads(payload)
            if type(sample) is not _PhysicalSample:
                raise RuntimeError("mailbox payload type is invalid")
            return next_version, sample
        return None

    def close(self) -> None:
        self._closed.value = 1


class _LatestProcessCommandMailbox:
    """Lock-free latest safe-command snapshot for the source process."""

    _WIDTH = 6

    def __init__(self, context) -> None:
        self._values = (
            context.RawArray("d", self._WIDTH),
            context.RawArray("d", self._WIDTH),
        )
        self._valid = (context.RawValue("b", 0), context.RawValue("b", 0))
        self._stamp_ns = (context.RawValue("Q", 0), context.RawValue("Q", 0))
        self._generations = (
            context.RawValue("Q", 0),
            context.RawValue("Q", 0),
        )
        self._active = context.RawValue("B", 0)
        self._version = context.RawValue("Q", 0)

    def put(self, command: Iterable[float], *, valid: bool, stamp_ns: int) -> int:
        values = tuple(float(value) for value in command)
        if len(values) != self._WIDTH or not all(math.isfinite(value) for value in values):
            raise ValueError("safe command mailbox requires six finite values")
        if type(valid) is not bool or type(stamp_ns) is not int or stamp_ns < 0:
            raise TypeError("safe command mailbox metadata has invalid exact types")
        slot = 1 - int(self._active.value)
        generation = int(self._generations[slot].value)
        if generation & 1:
            generation += 1
        self._generations[slot].value = generation + 1
        self._values[slot][:] = values
        self._valid[slot].value = int(valid)
        self._stamp_ns[slot].value = stamp_ns
        self._generations[slot].value = generation + 2
        version = int(self._version.value) + 1
        self._active.value = slot
        self._version.value = version
        return version

    def snapshot(self) -> tuple[int, tuple[float, ...], bool, int]:
        for _attempt in range(8):
            version = int(self._version.value)
            slot = int(self._active.value)
            generation_before = int(self._generations[slot].value)
            if generation_before & 1:
                continue
            values = tuple(float(value) for value in self._values[slot])
            valid = bool(self._valid[slot].value)
            stamp_ns = int(self._stamp_ns[slot].value)
            generation_after = int(self._generations[slot].value)
            if (
                generation_before == generation_after
                and not generation_after & 1
                and slot == int(self._active.value)
                and version == int(self._version.value)
            ):
                return version, values, valid, stamp_ns
        raise RuntimeError("safe command mailbox remained unstable")


def _wait_for_physical_deadline(deadline_mono: float, stop_event) -> bool:
    """Actively wait on the source's reserved development physical core.

    Sleeping introduced 20--50 ms CFS wake/scheduling intervals on the test
    host even though source computation used roughly 1.6 ms of thread CPU.
    This helper is used only by the explicit development source process after
    its exclusive CPU partition is authenticated. Production continues to use
    the established ROS timer. Returning false means shutdown was requested.
    """

    checks = 0
    while time.monotonic() < deadline_mono:
        checks += 1
        if checks & 0xFF:
            continue
        if stop_event.is_set():
            return False
    return not stop_event.is_set()


def _next_physical_deadline(
    previous_deadline_mono: float,
    period_s: float,
    completed_mono: float,
) -> float:
    """Advance one period and cap catch-up backlog at the completion instant."""

    values = (previous_deadline_mono, period_s, completed_mono)
    if not all(math.isfinite(value) for value in values) or period_s <= 0.0:
        raise ValueError("physical deadline inputs must be finite with positive period")
    return max(previous_deadline_mono + period_s, completed_mono)


def _retain_eval007_timing_trace(
    trace_root: Path,
    source_trace: _BoundedProcessTimingTrace,
    publication_traces: dict[str, _BoundedProcessTimingTrace],
) -> Path:
    """Retain one post-shutdown diagnostic bundle in an authenticated 0700 root."""

    if not isinstance(trace_root, Path) or not trace_root.is_absolute():
        raise ValueError("EVAL-007 timing trace root must be an absolute Path")
    root_stat = os.lstat(trace_root)
    if not stat.S_ISDIR(root_stat.st_mode) or stat.S_ISLNK(root_stat.st_mode):
        raise ValueError("EVAL-007 timing trace root must be a real directory")
    if root_stat.st_uid != os.getuid() or stat.S_IMODE(root_stat.st_mode) != 0o700:
        raise PermissionError("EVAL-007 timing trace root must be current-user mode 0700")
    resolved = trace_root.resolve(strict=True)
    if resolved != trace_root:
        raise ValueError("EVAL-007 timing trace root must be a canonical path")

    document = {
        "schema": "ctr-eval007-simulator-timing-bundle-1",
        "simulator_pid": os.getpid(),
        "captured_after_worker_shutdown_monotonic_ns": time.monotonic_ns(),
        "physical_source": source_trace.snapshot(),
        "publication_workers": {
            kind: publication_traces[kind].snapshot()
            for kind in sorted(publication_traces)
        },
    }
    payload = json.dumps(
        document,
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    filename = f"simulator-timing-{os.getpid()}-{time.time_ns()}.json"
    directory_fd = os.open(
        trace_root,
        os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW,
    )
    descriptor = -1
    try:
        descriptor = os.open(
            filename,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC | os.O_NOFOLLOW,
            0o600,
            dir_fd=directory_fd,
        )
        written = 0
        while written < len(payload):
            count = os.write(descriptor, payload[written:])
            if count <= 0:
                raise OSError("short EVAL-007 timing trace write")
            written += count
        os.fsync(descriptor)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        os.close(directory_fd)
    return trace_root / filename


def _generate_physical_tactile_sample(
    tip_position: Iterable[float],
    lumen,
    tactile_parameters: SimulatedTactileParameters,
    tactile_processor: TactileProcessor,
    timestamp_s: float,
) -> _PhysicalTactileSample:
    tip = np.asarray(tuple(tip_position), dtype=np.float64)
    if lumen is None:
        sample = simulate_tactile(None, tactile_parameters)
    else:
        clearance = lumen.point_clearance(tip).physical_clearance
        sample = simulate_tactile(clearance, tactile_parameters)
    processed = tactile_processor.process(
        [sample.raw_signal] if sample.valid else None,
        clearance_m=sample.clearance_m,
        geometric_contact=sample.contact,
        timestamp_s=timestamp_s,
    )
    return _PhysicalTactileSample(
        raw_signal=float(sample.raw_signal),
        filtered_signal=float(processed.filtered_signal),
        force_n=float(processed.force_n),
        clearance_m=float(processed.clearance_m),
        contact=bool(processed.contact),
        warning=bool(processed.warning),
        stop=bool(processed.stop),
        valid=bool(processed.valid),
        diagnostic_status=str(processed.diagnostic_status),
        region=int(processed.region),
    )


def _physical_source_process_main(
    config: dict,
    dt: float,
    command_timeout_s: float,
    sample_mailbox: _LatestProcessPhysicalSampleMailbox,
    command_mailbox: _LatestProcessCommandMailbox,
    stop_event,
    ready_event,
    diagnostics_enabled: bool,
    timing_trace: _BoundedProcessTimingTrace | None = None,
) -> None:
    """Generate authoritative physical samples outside every ROS interpreter."""

    # The ROS node parent owns process-group shutdown.  Ignoring the terminal
    # SIGINT in workers lets the parent set the shared stop event, join every
    # worker, retain the bounded trace, and close the channel deterministically.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    affinity = _apply_development_source_affinity(os.environ)
    physical_evidence_producer = None
    try:
        physical_evidence_producer = (
            PhysicalEvidenceProducer.from_environment()
            if selected_transport() == TRANSPORT_AUTHENTICATED_SHARED_MEMORY
            else None
        )
        model = ApproximateCTRModel(config)
        core = CTRSimulationCore(config)
        tactile_enabled = bool(config.get("tactile", {}).get("enabled", False))
        lumen = lumen_geometry_from_config(config) if tactile_enabled else None
        tactile_parameters = (
            SimulatedTactileParameters.from_mapping(config)
            if tactile_enabled
            else None
        )
        tactile_processor = (
            TactileProcessor.from_mapping(config) if tactile_enabled else None
        )
    except BaseException:
        if physical_evidence_producer is not None:
            physical_evidence_producer.close()
        ready_event.set()
        raise
    expected_mono = time.monotonic() + dt
    sequence = 0
    previous_handoff_wall_s = 0.0
    previous_handoff_thread_cpu_s = 0.0
    ready_event.set()
    try:
        while not stop_event.is_set():
            if not _wait_for_physical_deadline(expected_mono, stop_event):
                return
            callback_start_mono_ns = time.monotonic_ns()
            callback_start_thread_cpu_ns = time.thread_time_ns()
            callback_start_process_cpu_ns = time.process_time_ns()
            callback_start_mono = callback_start_mono_ns * 1.0e-9
            timing: dict[str, float | int] = {}
            if diagnostics_enabled:
                timing["previous_handoff_wall_s"] = previous_handoff_wall_s
                timing["previous_handoff_thread_cpu_s"] = (
                    previous_handoff_thread_cpu_s
                )

            def mark_stage(name: str) -> None:
                # The EVAL-007 focused roots retain the detailed substage
                # traces used to localize scheduler delay. Paper diagnostics
                # keep only bounded end-to-end source timing so evidence
                # collection cannot perturb the source/publication contract.
                del name

            lateness_s = max(0.0, callback_start_mono - expected_mono)
            expected_for_sample = expected_mono
            skipped_periods = int(lateness_s / dt)
            if diagnostics_enabled:
                timing["schedule_missed_periods"] = skipped_periods
            sequence += 1

            command_version, command_values, command_valid, command_stamp_ns = (
                command_mailbox.snapshot()
            )
            now_wall_ns = time.time_ns()
            command_age_s = max(0.0, (now_wall_ns - command_stamp_ns) * 1.0e-9)
            command_active = command_valid and command_age_s <= command_timeout_s
            command = (
                np.asarray(command_values, dtype=np.float64)
                if command_active
                else np.zeros(6, dtype=np.float64)
            )
            diagnostic_status = (
                "Safe command accepted"
                if command_active
                else (
                    "Command timed out; applying zero velocity"
                    if command_valid
                    else "Initialized without command"
                )
            )
            mark_stage("command")

            step = core.step(command, dt)
            mark_stage("state_step")
            model_result = model.forward_kinematics(step.q)
            mark_stage("forward_kinematics")

            # Generate the source stamp exactly once after physical-state
            # computation. Publication processes receive and preserve it.
            source_stamp_ns = time.time_ns()
            stamp_sec, stamp_nanosec = divmod(source_stamp_ns, 1_000_000_000)
            q = tuple(float(value) for value in step.q)
            q_dot = tuple(float(value) for value in step.q_dot)
            tip_position = tuple(float(value) for value in model_result.tip_position)
            tactile = (
                _generate_physical_tactile_sample(
                    tip_position,
                    lumen,
                    tactile_parameters,
                    tactile_processor,
                    source_stamp_ns * 1.0e-9,
                )
                if tactile_parameters is not None and tactile_processor is not None
                else None
            )
            shared_clearance = (
                lumen.backbone_clearance(model_result.backbone_points)
                if physical_evidence_producer is not None and lumen is not None
                else None
            )
            if physical_evidence_producer is not None and (
                tactile is None or shared_clearance is None
            ):
                raise RuntimeError(
                    "authenticated shared physical evidence requires tactile and lumen geometry"
                )
            mark_stage("tactile_generation")
            mark_stage("immutable_sample")
            callback_end_mono_ns = time.monotonic_ns()
            callback_end_thread_cpu_ns = time.thread_time_ns()
            callback_end_process_cpu_ns = time.process_time_ns()
            callback_end_mono = callback_end_mono_ns * 1.0e-9
            if diagnostics_enabled:
                callback_wall_s = callback_end_mono - callback_start_mono
                callback_thread_cpu_s = (
                    callback_end_thread_cpu_ns - callback_start_thread_cpu_ns
                ) * 1.0e-9
                # Detailed CPU/scheduler traces were retained in the focused
                # diagnostic roots. They are deliberately not copied through
                # the high-rate publication mailbox after localization.
                _ = callback_wall_s
                _ = callback_thread_cpu_s
                _ = callback_start_process_cpu_ns
                _ = affinity

            sample = _PhysicalSample(
                sequence=sequence,
                stamp_sec=int(stamp_sec),
                stamp_nanosec=int(stamp_nanosec),
                expected_monotonic_s=expected_for_sample,
                source_start_monotonic_s=callback_start_mono,
                source_complete_monotonic_s=callback_end_mono,
                source_lateness_s=lateness_s,
                source_duration_s=callback_end_mono - callback_start_mono,
                q=q,
                q_dot=q_dot,
                tip_position=tip_position,
                # Publication processes deterministically reconstruct the
                # backbone from this exact q. Keeping the variable-length
                # backbone out of the source mailbox prevents IPC pickling
                # and copying from extending the authoritative source period.
                backbone_points=(),
                command_age_s=command_age_s,
                command_saturated=bool(step.command_saturated),
                command_valid=bool(command_valid),
                diagnostic_status=diagnostic_status,
                model_status=str(model_result.diagnostic_status),
                tactile=tactile,
                source_timing=(),
            )
            shared_commit_begin_ns = 0
            shared_commit_end_ns = 0
            if physical_evidence_producer is not None:
                shared_commit_begin_ns = time.monotonic_ns()
                physical_evidence_producer.write(
                    PhysicalEvidenceRecord(
                        session_id=physical_evidence_producer.session_id,
                        producer_pid=os.getpid(),
                        producer_uid=os.getuid(),
                        generated_sequence=sequence,
                        source_monotonic_ns=callback_end_mono_ns,
                        source_stamp_ns=source_stamp_ns,
                        command_sequence=int(command_version),
                        q=q,
                        q_dot=q_dot,
                        tip_position=tip_position,
                        whole_backbone_physical_clearance_m=float(
                            shared_clearance.minimum_clearance
                        ),
                        whole_backbone_safety_clearance_m=float(
                            np.min(shared_clearance.safety_margin_clearances)
                        ),
                        raw_tactile=float(tactile.raw_signal),
                        filtered_tactile=float(tactile.filtered_signal),
                        tactile_force_n=float(tactile.force_n),
                        tactile_clearance_m=float(tactile.clearance_m),
                        tactile_region=int(tactile.region),
                        source_valid=True,
                        simulation=True,
                        frame_valid=True,
                        physical_collision=bool(shared_clearance.collision_count),
                        safety_margin_violation=bool(
                            shared_clearance.safety_margin_violation_count
                        ),
                        tactile_valid=bool(tactile.valid),
                        contact=bool(tactile.contact),
                        warning=bool(tactile.warning),
                        stop=bool(tactile.stop),
                    )
                )
                shared_commit_end_ns = time.monotonic_ns()
            handoff_wall_start_ns = time.monotonic_ns()
            handoff_cpu_start_ns = time.thread_time_ns()
            mailbox_version = sample_mailbox.put(sample)
            handoff_wall_end_ns = time.monotonic_ns()
            handoff_cpu_end_ns = time.thread_time_ns()
            previous_handoff_wall_s = (
                handoff_wall_end_ns - handoff_wall_start_ns
            ) * 1.0e-9
            previous_handoff_thread_cpu_s = (
                handoff_cpu_end_ns - handoff_cpu_start_ns
            ) * 1.0e-9
            if timing_trace is not None:
                timing_trace.append(
                    (
                        sequence,
                        source_stamp_ns,
                        int(expected_for_sample * 1_000_000_000),
                        callback_start_mono_ns,
                        callback_end_mono_ns,
                        callback_end_thread_cpu_ns - callback_start_thread_cpu_ns,
                        callback_end_process_cpu_ns - callback_start_process_cpu_ns,
                        handoff_wall_start_ns,
                        handoff_wall_end_ns,
                        handoff_cpu_end_ns - handoff_cpu_start_ns,
                        shared_commit_begin_ns,
                        shared_commit_end_ns,
                        int(command_version),
                        sequence,
                        int(mailbox_version),
                        skipped_periods,
                    )
                )
            # Do not turn one scheduler interruption into an additional full
            # source gap by waiting for the next aligned slot. Produce one
            # genuine immediate catch-up sample when late, then rebase. This
            # bounds backlog (unlike an unbounded catch-up loop) and never
            # duplicates or rewrites a physical timestamp.
            expected_mono = _next_physical_deadline(
                expected_for_sample,
                dt,
                time.monotonic(),
            )
    finally:
        if physical_evidence_producer is not None:
            physical_evidence_producer.close()
        ready_event.set()


def _publication_process_main(
    kind: str,
    mailbox: _LatestProcessPhysicalSampleMailbox,
    stop_event,
    ready_event,
    config: dict,
    frame_id: str,
    diagnostics_enabled: bool,
    timing_trace: _BoundedProcessTimingTrace | None = None,
) -> None:
    """Own one ROS publication family outside the physical-source interpreter."""

    signal.signal(signal.SIGINT, signal.SIG_IGN)
    node = None
    try:
        publisher_affinity = _apply_development_publication_affinity(
            kind, os.environ
        )
        publication_model = (
            ApproximateCTRModel(config)
            if kind in {"state", "auxiliary"}
            else None
        )
        rclpy.init(args=None)
        node = Node(f"ctr_simulator_{kind}_publisher")
        if kind == "state":
            publishers = {
                "state": node.create_publisher(
                    CtrState,
                    "/ctr/state",
                    source_evidence_qos_profile(reliable=False),
                )
            }
        elif kind == "tactile":
            publishers = {
                "tactile": node.create_publisher(
                    CtrTactileState,
                    "/ctr/tactile/state",
                    source_evidence_qos_profile(),
                ),
                # The compact deterministic state and tactile evidence are
                # emitted by one worker from one immutable physical sample.
                # Safety therefore cannot observe independent DDS scheduling
                # gaps between its two freshness inputs.
                "safety_state": node.create_publisher(
                    CtrJointState,
                    "/ctr/safety/joint_state",
                    source_evidence_qos_profile(),
                ),
            }
        elif kind == "auxiliary":
            publishers = {
                "joint": node.create_publisher(CtrJointState, "/ctr/joint_state", 10),
                "standard_joint": node.create_publisher(JointState, "/joint_states", 10),
                "backbone": node.create_publisher(CtrBackbone, "/ctr/backbone", 10),
                "tip": node.create_publisher(PoseStamped, "/ctr/tip", 10),
                "diagnostics": node.create_publisher(DiagnosticArray, "/diagnostics", 10),
            }
        else:
            raise RuntimeError(f"unknown publication worker kind: {kind}")

        ready_event.set()
        previous_version = 0
        previous_publish_duration_s = 0.0
        last_diagnostics_publish_mono = time.monotonic() - 0.10
        while not stop_event.is_set():
            delivery = mailbox.take_after(previous_version, stop_event)
            if delivery is None:
                break
            version, sample = delivery
            observation_ns = time.monotonic_ns()
            publish_start_ns = time.monotonic_ns()
            publish_start = publish_start_ns * 1.0e-9
            if kind == "state":
                message = _state_message_from_sample(
                    sample,
                    frame_id,
                    model=publication_model,
                )
                if diagnostics_enabled:
                    message.diagnostic_status = _with_evaluation_timing_schema(
                        message.diagnostic_status,
                        "ctr_state_timing_v1",
                        {
                            "sequence": sample.sequence,
                            "publish_monotonic_s": publish_start,
                            "mailbox_version": version,
                            "mailbox_overwrites": max(
                                0, version - previous_version - 1
                            ),
                            "publisher_pid": os.getpid(),
                        },
                    )
                publishers["state"].publish(message)
            elif kind == "tactile":
                safety_state = _safety_joint_state_message_from_sample(sample, frame_id)
                if diagnostics_enabled:
                    safety_state.diagnostic_status = _with_evaluation_timing_schema(
                        safety_state.diagnostic_status,
                        "ctr_state_timing_v1",
                        {
                            "sequence": sample.sequence,
                            "publish_monotonic_s": publish_start,
                            "mailbox_version": version,
                            "mailbox_overwrites": max(
                                0, version - previous_version - 1
                            ),
                            "publisher_pid": os.getpid(),
                        },
                    )
                publishers["safety_state"].publish(safety_state)
                message = _tactile_message_from_sample(sample, frame_id)
                if diagnostics_enabled:
                    timing = {
                            "sequence": sample.sequence,
                            "expected_monotonic_s": sample.expected_monotonic_s,
                            "callback_start_monotonic_s": sample.source_start_monotonic_s,
                            "publish_monotonic_s": publish_start,
                            "callback_lateness_s": sample.source_lateness_s,
                            "skipped_periods": 0,
                            "late_periods": int(
                                sample.source_lateness_s
                                * float(config["simulation"]["update_frequency"])
                            ),
                            "previous_callback_duration_s": previous_publish_duration_s,
                            "state_source_monotonic_s": sample.source_complete_monotonic_s,
                            "state_age_s": max(0.0, publish_start - sample.source_complete_monotonic_s),
                            "physics_sequence": sample.sequence,
                            "physics_callback_start_monotonic_s": sample.source_start_monotonic_s,
                            "physics_callback_lateness_s": sample.source_lateness_s,
                            "physics_callback_duration_s": sample.source_duration_s,
                            "mailbox_version": version,
                            "mailbox_overwrites": max(0, version - previous_version - 1),
                            "publisher_pid": os.getpid(),
                            "publisher_thread_id": threading.get_native_id(),
                            "publisher_affinity_count": len(publisher_affinity),
                        }
                    for index, cpu in enumerate(publisher_affinity):
                        timing[f"publisher_affinity_cpu_{index}"] = cpu
                    message.diagnostic_status = _with_evaluation_timing(
                        message.diagnostic_status, timing
                    )
                publishers["tactile"].publish(message)
            else:
                _publish_auxiliary_process_sample(
                    sample,
                    frame_id,
                    publishers,
                    last_diagnostics_publish_mono,
                    model=publication_model,
                )
                if time.monotonic() - last_diagnostics_publish_mono >= 0.10:
                    last_diagnostics_publish_mono = time.monotonic()
            publish_end_ns = time.monotonic_ns()
            previous_publish_duration_s = (publish_end_ns - publish_start_ns) * 1.0e-9
            if timing_trace is not None:
                timing_trace.append(
                    (
                        int(sample.sequence),
                        int(sample.stamp_sec) * 1_000_000_000
                        + int(sample.stamp_nanosec),
                        int(version),
                        observation_ns,
                        publish_start_ns,
                        publish_end_ns,
                        max(0, int(version) - int(previous_version) - 1),
                        os.getpid(),
                    )
                )
            previous_version = version
    finally:
        ready_event.set()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class CTRSimulatorNode(Node):
    """Receive safe q_dot commands, update q, and publish simulated CTR state."""

    def __init__(self):
        super().__init__("simulator_node")
        self.declare_parameter("config_paths", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("runtime_mode", "simulation")
        self.declare_parameter("target_position", [0.0, 0.0, 0.08])
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("enable_cylindrical_lumen", False)
        self.declare_parameter("enable_curved_lumen", False)
        self.declare_parameter("curved_lumen_type", "")
        self.declare_parameter("cylinder_target_position", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("tactile_enabled", False)
        self.declare_parameter("slice_7g_profile", False)
        self.declare_parameter("development_simulation", False)
        self.declare_parameter("enable_development_visualization", False)
        self.declare_parameter("evaluation_diagnostics_enabled", False)
        self.declare_parameter("physical_evidence_transport", TRANSPORT_ROS)

        config_paths = validate_config_paths(self.get_parameter("config_paths").value)

        raw_config = load_parameter_files(config_paths)
        enable_lumen = parse_launch_bool(
            self.get_parameter("enable_cylindrical_lumen").value,
            "enable_cylindrical_lumen",
        )
        enable_curved_lumen = parse_launch_bool(
            self.get_parameter("enable_curved_lumen").value,
            "enable_curved_lumen",
        )
        self.config = config_with_lumen_overrides(
            raw_config,
            enable_cylindrical_lumen=enable_lumen,
            enable_curved_lumen=enable_curved_lumen,
            curved_lumen_type=str(self.get_parameter("curved_lumen_type").value or ""),
            target=_optional_vector3_parameter(self.get_parameter("cylinder_target_position").value),
        )
        self.config.setdefault("runtime", {})["mode"] = str(
            self.get_parameter("runtime_mode").value
        )
        slice_7g_enabled = parse_launch_bool(
            self.get_parameter("slice_7g_profile").value,
            "slice_7g_profile",
        )
        development_enabled = parse_launch_bool(
            self.get_parameter("development_simulation").value,
            "development_simulation",
        )
        development_visualization_enabled = parse_launch_bool(
            self.get_parameter("enable_development_visualization").value,
            "enable_development_visualization",
        )
        evaluation_diagnostics_enabled = parse_launch_bool(
            self.get_parameter("evaluation_diagnostics_enabled").value,
            "evaluation_diagnostics_enabled",
        )
        physical_evidence_transport = str(
            self.get_parameter("physical_evidence_transport").value
        )
        if physical_evidence_transport not in TRANSPORT_VALUES:
            raise ValueError("physical_evidence_transport is invalid")
        if (
            physical_evidence_transport == TRANSPORT_AUTHENTICATED_SHARED_MEMORY
            and (not development_enabled or not evaluation_diagnostics_enabled)
        ):
            raise ValueError(
                "authenticated shared physical evidence requires explicit development diagnostics"
            )
        if physical_evidence_transport != selected_transport():
            raise ValueError(
                "physical_evidence_transport differs from the runner-bound environment"
            )
        if development_visualization_enabled and not development_enabled:
            raise ValueError(
                "enable_development_visualization requires development_simulation=true"
            )
        self.development_simulation = development_enabled
        self.development_visualization = development_visualization_enabled
        self.evaluation_diagnostics_enabled = evaluation_diagnostics_enabled
        self.physical_evidence_transport = physical_evidence_transport
        self.config = (
            apply_slice_7g_development_simulation_profile(self.config, enabled=True)
            if development_enabled
            else apply_slice_7g_simulation_profile(self.config, enabled=slice_7g_enabled)
        )
        validate_or_raise(self.config)
        self.lumen_mode = lumen_mode_from_config(self.config)

        self.core = CTRSimulationCore(self.config)
        self.model = ApproximateCTRModel(self.config)
        self._state_lock = threading.RLock()
        self._command_lock = threading.Lock()

        simulation = self.config["simulation"]
        self.update_frequency = float(simulation["update_frequency"])
        self.dt = 1.0 / self.update_frequency
        self.tactile_source_state_timeout = float(self.config["safety"]["tactile_timeout"])
        self.lumen_marker_config = LumenMarkerConfig.from_mapping(
            simulation.get("visualization", {})
        )
        if self.development_visualization:
            self.lumen_marker_config = replace(
                self.lumen_marker_config,
                publish_lumen_surface=True,
            )
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.frame_id = self.config["robot"]["frames"]["base"]
        self.world_frame_id = self.config["robot"]["frames"]["world"]
        self.tip_frame_id = self.config["robot"]["frames"]["tip"]
        self.target_position = (
            goal_position_from_config(self.config)
            if self.lumen_mode != "none"
            else _vector3(self.get_parameter("target_position").value, "target_position")
        )
        self.lumen_geometry = lumen_geometry_from_config(self.config)
        self.lumen = self.lumen_geometry
        self.tactile_enabled = slice_7g_enabled or parse_launch_bool(
            self.get_parameter("tactile_enabled").value,
            "tactile_enabled",
        )
        self.tactile_parameters = (
            SimulatedTactileParameters.from_mapping(self.config) if self.tactile_enabled else None
        )
        self.tactile_processor = (
            TactileProcessor.from_mapping(self.config) if self.tactile_enabled else None
        )
        self._static_lumen_cache_key = None
        self._static_lumen_markers: list[Marker] = []
        self._static_lumen_marker_keys: tuple[tuple[str, int], ...] = ()
        self._static_lumen_marker_frame_id = self.frame_id
        self._last_static_lumen_publish_time_s: float | None = None
        self._last_development_visualization_publish_time_s: float | None = None
        self._last_runtime_marker_publish_time_s: float | None = None
        self._static_lumen_build_count = 0
        self._static_lumen_cache_hit_logged = False
        self._dynamic_lumen_marker_keys: tuple[tuple[str, int], ...] = ()
        self._dynamic_lumen_marker_frame_id = self.frame_id
        self._last_lumen_diagnostic_log_signature: tuple[str, ...] | None = None
        self._lumen_diagnostic_update_count = 0
        self._reference_path_points = np.empty((0, 3), dtype=np.float64)
        self._reference_path_frame_id = self.frame_id
        self._tip_trajectory = (
            BoundedTipTrajectory(
                max_points=self.lumen_marker_config.actual_tip_history_max_points,
                minimum_interval=self.lumen_marker_config.actual_tip_history_min_interval,
            )
            if self.development_visualization
            else None
        )

        self.latest_command = np.zeros(6, dtype=float)
        self.command_valid = False
        self.command_saturated = False
        self.last_command_time = self.get_clock().now()
        self.last_diagnostic_status = "Initialized without command"
        initial_model_result = self.model.forward_kinematics(self.core.q)
        initial_mono = time.monotonic()
        self._latest_tactile_tip = np.asarray(
            initial_model_result.tip_position, dtype=np.float64
        ).copy()
        self._latest_state_source_mono = initial_mono
        self._physics_sequence = 0
        self._physics_expected_mono = initial_mono + self.dt
        self._physics_callback_start_mono = initial_mono
        self._physics_callback_lateness_s = 0.0
        self._physics_callback_duration_s = 0.0
        self._tactile_sequence = 0
        self._tactile_expected_mono = initial_mono + self.dt
        self._tactile_previous_callback_duration_s = 0.0
        self._previous_source_timing: dict[str, float | int] = {}
        self._source_gc_pause_wall_s = 0.0
        self._source_gc_pause_thread_s = 0.0
        self._source_gc_collections = 0
        self._source_gc_started: dict[int, tuple[float, float]] = {}
        self._source_gc_callback = self._on_source_gc_event
        if self.evaluation_diagnostics_enabled:
            gc.callbacks.append(self._source_gc_callback)
        self._diagnostics_publish_period_s = 0.10
        self._last_diagnostics_publish_mono = initial_mono - self._diagnostics_publish_period_s
        self._physics_callback_group = MutuallyExclusiveCallbackGroup()
        self._tactile_callback_group = MutuallyExclusiveCallbackGroup()
        self._visualization_callback_group = MutuallyExclusiveCallbackGroup()
        self._latest_visualization_backbone = np.asarray(
            initial_model_result.backbone_points, dtype=np.float64
        ).copy()
        self._visualization_mailbox_version = 0
        self._backbone_point_cache: list[Point] = []
        self._tip_pose_cache: PoseStamped | None = None
        self._state_message_cache: CtrState | None = None
        self._publication_context = (
            multiprocessing.get_context("spawn") if self.development_simulation else None
        )
        trace_root_text = os.environ.get(_EVAL007_TIMING_TRACE_ROOT_ENV, "")
        self._eval007_trace_root = (
            Path(trace_root_text)
            if self.development_simulation
            and self.evaluation_diagnostics_enabled
            and trace_root_text
            else None
        )
        self._source_timing_trace = (
            _BoundedProcessTimingTrace(
                self._publication_context, _PHYSICAL_SOURCE_TRACE_FIELDS
            )
            if self._publication_context is not None
            and self._eval007_trace_root is not None
            else None
        )
        self._publication_timing_traces: dict[
            str, _BoundedProcessTimingTrace
        ] = {}
        self._source_stop_event = (
            self._publication_context.Event()
            if self._publication_context is not None
            else threading.Event()
        )
        self._source_thread: threading.Thread | None = None
        self._source_process: multiprocessing.Process | None = None
        self._source_failure: BaseException | None = None
        self._source_affinity: tuple[int, ...] = ()
        if self._publication_context is not None:
            # One shared immutable slot gives each process an independent read
            # cursor while the source performs only one serialization/copy.
            shared_mailbox = _LatestProcessPhysicalSampleMailbox(
                self._publication_context
            )
            self._state_mailbox = shared_mailbox
            self._tactile_mailbox = shared_mailbox
            self._auxiliary_mailbox = shared_mailbox
            self._command_mailbox = _LatestProcessCommandMailbox(
                self._publication_context
            )
            self._command_mailbox.put(
                np.zeros(6, dtype=np.float64),
                valid=False,
                stamp_ns=int(self.get_clock().now().nanoseconds),
            )
        else:
            self._state_mailbox = _LatestPhysicalSampleMailbox()
            self._tactile_mailbox = _LatestPhysicalSampleMailbox()
            self._auxiliary_mailbox = _LatestPhysicalSampleMailbox()
            self._command_mailbox = None
        self._publication_processes: list[multiprocessing.Process] = []
        self._publication_failures: dict[str, BaseException] = {}

        self.command_sub = self.create_subscription(
            CtrJointCommand,
            "/ctr/safe_command",
            self._on_safe_command,
            10,
        )
        self.target_sub = self.create_subscription(
            PoseStamped,
            "/ctr/reference/tip",
            self._on_target,
            10,
        )
        self.reference_path_sub = (
            self.create_subscription(
                NavPath,
                "/ctr/reference/path",
                self._on_reference_path,
                reference_path_qos_profile(),
            )
            if self.development_visualization
            else None
        )

        if self.development_simulation:
            self.joint_pub = None
            self.standard_joint_pub = None
            self.backbone_pub = None
            self.tip_pub = None
            self.state_pub = None
            self.tactile_pub = None
            self.diagnostics_pub = None
        else:
            self.joint_pub = self.create_publisher(CtrJointState, "/ctr/joint_state", 10)
            self.standard_joint_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.backbone_pub = self.create_publisher(CtrBackbone, "/ctr/backbone", 10)
            self.tip_pub = self.create_publisher(PoseStamped, "/ctr/tip", 10)
            self.state_pub = self.create_publisher(CtrState, "/ctr/state", 10)
            self.tactile_pub = (
                self.create_publisher(CtrTactileState, "/ctr/tactile/state", 10)
                if self.tactile_enabled
                else None
            )
            self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/ctr/visualization",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.development_marker_pubs = (
            {
                namespace: self.create_publisher(
                    MarkerArray,
                    f"/ctr/development_visualization/{namespace}",
                    development_marker_qos_profile(namespace),
                )
                for namespace in (
                    "lumen_surface",
                    "lumen_wireframe",
                    "lumen_centerline",
                    "ctr_backbone",
                    "reference_path",
                    "actual_tip_path",
                    "tip_marker",
                    "target_marker",
                )
            }
            if self.development_visualization
            else {}
        )

        self.timer = (
            None
            if self.development_simulation
            else self.create_timer(
                self.dt, self._on_timer, callback_group=self._physics_callback_group
            )
        )
        self.tactile_timer = None
        initial_stamp = self.get_clock().now().to_msg()
        initial_backbone = self._latest_visualization_backbone.copy()
        self._publish_visualization_snapshot(
            initial_stamp,
            initial_backbone,
            include_static=True,
        )
        self.visualization_timer = self.create_timer(
            1.0 / self.lumen_marker_config.marker_publish_rate,
            self._on_visualization_timer,
            callback_group=self._visualization_callback_group,
        )
        if self.development_simulation:
            assert self._publication_context is not None
            publication_specs = [
                ("state", self._state_mailbox),
                ("auxiliary", self._auxiliary_mailbox),
            ]
            if self.tactile_enabled:
                publication_specs.append(("tactile", self._tactile_mailbox))
            ready_events = []
            for kind, mailbox in publication_specs:
                ready_event = self._publication_context.Event()
                timing_trace = (
                    _BoundedProcessTimingTrace(
                        self._publication_context, _PUBLICATION_TRACE_FIELDS
                    )
                    if self._eval007_trace_root is not None
                    else None
                )
                if timing_trace is not None:
                    self._publication_timing_traces[kind] = timing_trace
                process = self._publication_context.Process(
                    target=_publication_process_main,
                    name=f"ctr-simulator-{kind}-publisher",
                    args=(
                        kind,
                        mailbox,
                        self._source_stop_event,
                        ready_event,
                        self.config,
                        self.frame_id,
                        self.evaluation_diagnostics_enabled,
                        timing_trace,
                    ),
                    daemon=False,
                )
                process.start()
                self._publication_processes.append(process)
                ready_events.append((kind, ready_event, process))
            for kind, ready_event, process in ready_events:
                if not ready_event.wait(timeout=10.0) or not process.is_alive():
                    self._source_stop_event.set()
                    self._close_publication_mailboxes()
                    raise RuntimeError(f"{kind} publication process failed to initialize")
            source_ready_event = self._publication_context.Event()
            assert self._command_mailbox is not None
            self._source_process = self._publication_context.Process(
                target=_physical_source_process_main,
                name="ctr-simulator-physical-source",
                args=(
                    self.config,
                    self.dt,
                    self.command_timeout,
                    self._state_mailbox,
                    self._command_mailbox,
                    self._source_stop_event,
                    source_ready_event,
                    self.evaluation_diagnostics_enabled,
                    self._source_timing_trace,
                ),
                daemon=False,
            )
            self._source_process.start()
            if (
                not source_ready_event.wait(timeout=10.0)
                or not self._source_process.is_alive()
            ):
                self._source_stop_event.set()
                self._close_publication_mailboxes()
                raise RuntimeError("physical source process failed to initialize")
        self.get_logger().info(
            "CTR simulator started: /ctr/safe_command -> /ctr/joint_state, /ctr/backbone, /ctr/tip, /ctr/state."
        )
        self.get_logger().info(lumen_geometry_log_line(self.config, role="simulator"))

    def _on_safe_command(self, msg: CtrJointCommand) -> None:
        try:
            command = np.asarray(msg.q_dot, dtype=float)
            if command.shape != (6,) or not np.all(np.isfinite(command)):
                raise ValueError("safe command must contain 6 finite values")
            if not msg.valid:
                raise ValueError("safe command validity flag is false")
        except ValueError as exc:
            command_stamp = self.get_clock().now()
            with self._command_lock:
                self.command_valid = False
                self.latest_command = np.zeros(6, dtype=float)
                self.last_command_time = command_stamp
                self.last_diagnostic_status = f"Rejected command: {exc}"
                if self._command_mailbox is not None:
                    self._command_mailbox.put(
                        self.latest_command,
                        valid=False,
                        stamp_ns=int(command_stamp.nanoseconds),
                    )
            self.get_logger().warn(self.last_diagnostic_status)
            return

        command_stamp = self.get_clock().now()
        with self._command_lock:
            self.latest_command = command
            self.command_valid = True
            self.last_command_time = command_stamp
            self.last_diagnostic_status = msg.diagnostic_status or "Safe command accepted"
            if self._command_mailbox is not None:
                self._command_mailbox.put(
                    command,
                    valid=True,
                    stamp_ns=int(command_stamp.nanoseconds),
                )

    def _on_target(self, msg: PoseStamped) -> None:
        self.target_position = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=float,
        )

    def _on_reference_path(self, msg: NavPath) -> None:
        """Retain only the exact finite reference data published by the controller path owner."""

        try:
            frame_id = str(msg.header.frame_id)
            if not frame_id:
                raise ValueError("reference path frame is empty")
            points = np.asarray(
                [
                    (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
                    for pose in msg.poses
                ],
                dtype=np.float64,
            )
            if points.ndim != 2 or points.shape[1:] != (3,) or points.shape[0] < 1:
                raise ValueError("reference path must contain at least one 3D pose")
            if not np.all(np.isfinite(points)):
                raise ValueError("reference path contains non-finite coordinates")
            pose_frames = {pose.header.frame_id for pose in msg.poses if pose.header.frame_id}
            if pose_frames and pose_frames != {frame_id}:
                raise ValueError("reference path pose frames do not match its header")
        except (TypeError, ValueError) as exc:
            self._reference_path_points = np.empty((0, 3), dtype=np.float64)
            self.get_logger().warn(f"Reference path visualization rejected: {exc}")
            return
        self._reference_path_points = points.copy()
        self._reference_path_frame_id = frame_id

    def _on_timer(self) -> None:
        """Generate one authoritative physical sample without ROS publication in development."""

        callback_start_mono = time.monotonic()
        callback_start_thread_cpu = time.thread_time()
        callback_start_process_cpu = time.process_time()
        gc_pause_wall_start = self._source_gc_pause_wall_s
        gc_pause_thread_start = self._source_gc_pause_thread_s
        gc_collections_start = self._source_gc_collections
        source_timing: dict[str, float | int] = {}
        stage_wall = callback_start_mono
        stage_thread_cpu = callback_start_thread_cpu

        def mark_stage(name: str) -> None:
            nonlocal stage_wall, stage_thread_cpu
            if not self.evaluation_diagnostics_enabled:
                return
            now_wall = time.monotonic()
            now_thread_cpu = time.thread_time()
            source_timing[f"{name}_wall_s"] = now_wall - stage_wall
            source_timing[f"{name}_thread_cpu_s"] = now_thread_cpu - stage_thread_cpu
            stage_wall = now_wall
            stage_thread_cpu = now_thread_cpu

        expected_mono = self._physics_expected_mono
        lateness_s = max(0.0, callback_start_mono - expected_mono)
        skipped_periods = int(lateness_s / self.dt)
        self._physics_expected_mono = expected_mono + (skipped_periods + 1) * self.dt
        self._physics_sequence += 1
        callback_now = self.get_clock().now()
        with self._command_lock:
            command_age = (callback_now - self.last_command_time).nanoseconds * 1e-9
            command_valid = self.command_valid
            command = (
                self.latest_command.copy()
                if command_valid and command_age <= self.command_timeout
                else np.zeros(6)
            )
            if command_valid and command_age > self.command_timeout:
                self.last_diagnostic_status = "Command timed out; applying zero velocity"
        mark_stage("command")

        step = self.core.step(command, self.dt)
        mark_stage("state_step")
        self.command_saturated = step.command_saturated
        model_result = self.model.forward_kinematics(step.q)
        mark_stage("forward_kinematics")
        lock_wait_start = time.monotonic()
        lock_wait_thread_cpu_start = time.thread_time()
        with self._state_lock:
            if self.evaluation_diagnostics_enabled:
                source_timing["state_lock_wait_wall_s"] = time.monotonic() - lock_wait_start
                source_timing["state_lock_wait_thread_cpu_s"] = (
                    time.thread_time() - lock_wait_thread_cpu_start
                )
            self._latest_tactile_tip = np.asarray(
                model_result.tip_position, dtype=np.float64
            ).copy()
            self._latest_visualization_backbone = np.asarray(
                model_result.backbone_points, dtype=np.float64
            ).copy()
            self._latest_state_source_mono = callback_start_mono
            self._physics_callback_start_mono = callback_start_mono
            self._physics_callback_lateness_s = lateness_s
        mark_stage("state_snapshot")

        # The source timestamp is generated once when the genuine physical
        # sample is complete.  Publication workers must never rewrite it.
        stamp = self.get_clock().now().to_msg()
        q = tuple(float(value) for value in step.q)
        q_dot = tuple(float(value) for value in step.q_dot)
        tip_position = tuple(float(value) for value in model_result.tip_position)
        backbone = tuple(
            tuple(float(value) for value in point)
            for point in model_result.backbone_points
        )
        tactile = (
            _generate_physical_tactile_sample(
                tip_position,
                self.lumen,
                self.tactile_parameters,
                self.tactile_processor,
                stamp.sec + stamp.nanosec * 1.0e-9,
            )
            if self.tactile_parameters is not None
            and self.tactile_processor is not None
            else None
        )
        mark_stage("tactile_generation")
        mark_stage("immutable_sample")
        callback_end_mono = time.monotonic()
        callback_end_thread_cpu = time.thread_time()
        callback_end_process_cpu = time.process_time()
        if self.evaluation_diagnostics_enabled:
            callback_wall_s = callback_end_mono - callback_start_mono
            callback_thread_cpu_s = callback_end_thread_cpu - callback_start_thread_cpu
            source_timing.update(
                {
                    "callback_wall_s": callback_wall_s,
                    "callback_thread_cpu_s": callback_thread_cpu_s,
                    "callback_process_cpu_s": callback_end_process_cpu - callback_start_process_cpu,
                    "callback_scheduler_or_blocked_s": max(0.0, callback_wall_s - callback_thread_cpu_s),
                    "gc_pause_wall_s": self._source_gc_pause_wall_s - gc_pause_wall_start,
                    "gc_pause_thread_cpu_s": self._source_gc_pause_thread_s - gc_pause_thread_start,
                    "gc_collections": self._source_gc_collections - gc_collections_start,
                    "source_thread_id": threading.get_native_id(),
                    "thread_id": threading.get_native_id(),
                    "pid": os.getpid(),
                    "source_affinity_count": len(self._source_affinity),
                }
            )
            for index, cpu in enumerate(self._source_affinity):
                source_timing[f"source_affinity_cpu_{index}"] = cpu
            source_timing.update(
                {
                    f"previous_{key}": value
                    for key, value in self._previous_source_timing.items()
                }
            )
            self._previous_source_timing = {
                key: value
                for key, value in source_timing.items()
                if not key.startswith("previous_")
            }

        sample = _PhysicalSample(
            sequence=self._physics_sequence,
            stamp_sec=int(stamp.sec),
            stamp_nanosec=int(stamp.nanosec),
            expected_monotonic_s=expected_mono,
            source_start_monotonic_s=callback_start_mono,
            source_complete_monotonic_s=callback_end_mono,
            source_lateness_s=lateness_s,
            source_duration_s=callback_end_mono - callback_start_mono,
            q=q,
            q_dot=q_dot,
            tip_position=tip_position,
            backbone_points=backbone,
            command_age_s=command_age,
            command_saturated=bool(step.command_saturated),
            command_valid=bool(command_valid),
            diagnostic_status=str(self.last_diagnostic_status),
            model_status=str(model_result.diagnostic_status),
            tactile=tactile,
            source_timing=tuple(sorted(source_timing.items())),
        )

        if self.development_simulation:
            # These constant-time single-slot replacements are the only source
            # thread handoff.  ROS message creation and publish calls occur on
            # independent workers and can neither block nor mutate the source.
            try:
                if self._source_stop_event.is_set():
                    return
                self._state_mailbox.put(sample)
            except RuntimeError:
                if not self._source_stop_event.is_set():
                    raise
        else:
            self._publish_sample_synchronously(sample)

        with self._state_lock:
            self._physics_callback_duration_s = time.monotonic() - callback_start_mono

    def _publish_sample_synchronously(self, sample: _PhysicalSample) -> None:
        """Preserve the established non-development publication ordering."""

        self._publish_state_sample(sample)
        if self.tactile_pub is not None:
            self._publish_tactile_for_physical_state(sample, mailbox_version=sample.sequence)
        self._publish_auxiliary_sample(sample)

    def _on_visualization_timer(self) -> None:
        if not self._visualization_consumers_present():
            return
        if self.development_simulation:
            delivery = self._state_mailbox.snapshot_after(
                self._visualization_mailbox_version
            )
            if delivery is not None:
                version, sample = delivery
                self._visualization_mailbox_version = version
                backbone_points = _backbone_points_from_sample(sample, self.model)
                with self._state_lock:
                    self._latest_visualization_backbone = np.asarray(
                        backbone_points, dtype=np.float64
                    )
                    self._latest_tactile_tip = np.asarray(
                        sample.tip_position, dtype=np.float64
                    )
        stamp = self.get_clock().now().to_msg()
        with self._state_lock:
            backbone = self._latest_visualization_backbone.copy()
            tip = self._latest_tactile_tip.copy()
        if self._tip_trajectory is not None:
            self._tip_trajectory.append(tip, _stamp_seconds(stamp))
        self._publish_visualization_snapshot(stamp, backbone, include_static=False)

    def _visualization_consumers_present(self) -> bool:
        """Avoid constructing MarkerArrays when no visualization client can consume them."""

        if self.marker_pub.get_subscription_count() > 0:
            return True
        return any(
            publisher.get_subscription_count() > 0
            for publisher in self.development_marker_pubs.values()
        )

    @staticmethod
    def _publisher_has_consumers(publisher) -> bool:
        return int(publisher.get_subscription_count()) > 0

    def _publish_visualization_snapshot(
        self,
        stamp,
        backbone_array: np.ndarray,
        *,
        include_static: bool,
    ) -> None:
        backbone_points = [_point_from_array(point) for point in backbone_array]
        marker_array = self._marker_array_msg(
            stamp,
            backbone_points,
            backbone_array,
            include_development=self.development_visualization,
            include_static=include_static,
        )
        self.marker_pub.publish(
            MarkerArray(
                markers=[marker for marker in marker_array.markers if marker.ns != "lumen_surface"]
            )
        )
        if self.development_visualization:
            self._publish_development_marker_topics(marker_array)

    def _run_state_publisher(self) -> None:
        self._run_publication_worker(
            "state",
            self._state_mailbox,
            lambda sample, _version, _previous: self._publish_state_sample(sample),
        )

    def _run_tactile_publisher(self) -> None:
        self._run_publication_worker(
            "tactile",
            self._tactile_mailbox,
            lambda sample, version, previous: self._publish_tactile_for_physical_state(
                sample,
                mailbox_version=version,
                previous_mailbox_version=previous,
            ),
        )

    def _run_auxiliary_publisher(self) -> None:
        self._run_publication_worker(
            "auxiliary",
            self._auxiliary_mailbox,
            lambda sample, _version, _previous: self._publish_auxiliary_sample(sample),
        )

    def _run_publication_worker(self, name, mailbox, publish_sample) -> None:
        """Drain only the latest immutable sample; fail closed on worker loss."""

        previous_version = 0
        try:
            while not self._source_stop_event.is_set():
                delivery = mailbox.take_after(previous_version, self._source_stop_event)
                if delivery is None:
                    return
                version, sample = delivery
                publish_sample(sample, version, previous_version)
                previous_version = version
        except BaseException as exc:
            self._publication_failures[name] = exc
            self._source_stop_event.set()
            self._close_publication_mailboxes()
            self.get_logger().error(
                f"{name} publication worker stopped fail-closed: "
                f"{type(exc).__name__}: {exc}"
            )

    def _publish_state_sample(self, sample: _PhysicalSample) -> None:
        stamp = sample.stamp()
        backbone_points = [_point_from_array(point) for point in sample.backbone_points]
        tip_pose = _tip_pose_message(stamp, self.frame_id, sample.tip_position)
        self.state_pub.publish(
            self._state_msg(
                stamp,
                np.asarray(sample.q, dtype=np.float64),
                np.asarray(sample.q_dot, dtype=np.float64),
                backbone_points,
                tip_pose,
                diagnostic_status=sample.diagnostic_status,
            )
        )

    def _publish_auxiliary_sample(self, sample: _PhysicalSample) -> None:
        stamp = sample.stamp()
        q = np.asarray(sample.q, dtype=np.float64)
        q_dot = np.asarray(sample.q_dot, dtype=np.float64)
        backbone_array = np.asarray(sample.backbone_points, dtype=np.float64)
        backbone_points = [_point_from_array(point) for point in sample.backbone_points]
        tip_pose = _tip_pose_message(stamp, self.frame_id, sample.tip_position)
        if self._tip_trajectory is not None:
            self._tip_trajectory.append(sample.tip_position, _stamp_seconds(stamp))
        if self._publisher_has_consumers(self.tip_pub):
            self.tip_pub.publish(tip_pose)
        if self._publisher_has_consumers(self.joint_pub):
            self.joint_pub.publish(
                self._joint_state_msg(
                    stamp,
                    q,
                    q_dot,
                    diagnostic_status=sample.diagnostic_status,
                )
            )
        if self._publisher_has_consumers(self.standard_joint_pub):
            self.standard_joint_pub.publish(self._standard_joint_state_msg(stamp, q, q_dot))
        if self._publisher_has_consumers(self.backbone_pub):
            self.backbone_pub.publish(
                self._backbone_msg(
                    stamp,
                    backbone_points,
                    diagnostic_status=sample.diagnostic_status,
                )
            )
        diagnostics_now = time.monotonic()
        if (
            self._publisher_has_consumers(self.diagnostics_pub)
            and diagnostics_now - self._last_diagnostics_publish_mono
            >= self._diagnostics_publish_period_s
        ):
            self.diagnostics_pub.publish(
                self._diagnostics_msg(
                    stamp,
                    sample.command_age_s,
                    sample.model_status,
                    command_valid=sample.command_valid,
                    command_saturated=sample.command_saturated,
                    diagnostic_status=sample.diagnostic_status,
                )
            )
            self._last_diagnostics_publish_mono = diagnostics_now

    def _publish_tactile_for_physical_state(
        self,
        sample: _PhysicalSample,
        *,
        mailbox_version: int,
        previous_mailbox_version: int = 0,
    ) -> None:
        """Measure tactile evidence from the exact state integrated this cycle."""

        publish_mono = time.monotonic()
        self._tactile_sequence = sample.sequence
        timing = {
            "sequence": sample.sequence,
            "expected_monotonic_s": sample.expected_monotonic_s,
            "callback_start_monotonic_s": sample.source_start_monotonic_s,
            "publish_monotonic_s": publish_mono,
            "callback_lateness_s": sample.source_lateness_s,
            "skipped_periods": int(sample.source_lateness_s / self.dt),
            "previous_callback_duration_s": self._tactile_previous_callback_duration_s,
            "state_source_monotonic_s": sample.source_complete_monotonic_s,
            "state_age_s": max(0.0, publish_mono - sample.source_complete_monotonic_s),
            "physics_sequence": sample.sequence,
            "physics_callback_start_monotonic_s": sample.source_start_monotonic_s,
            "physics_callback_lateness_s": sample.source_lateness_s,
            "physics_callback_duration_s": sample.source_duration_s,
            "mailbox_version": mailbox_version,
            "mailbox_overwrites": max(0, mailbox_version - previous_mailbox_version - 1),
            "pid": os.getpid(),
            "publisher_thread_id": threading.get_native_id(),
            "source_affinity_count": len(getattr(self, "_source_affinity", ())),
        }
        for index, cpu in enumerate(getattr(self, "_source_affinity", ())):
            timing[f"source_affinity_cpu_{index}"] = cpu
        if self.evaluation_diagnostics_enabled:
            timing.update(dict(sample.source_timing))
        msg = _tactile_message_from_sample(sample, self.frame_id)
        if self.evaluation_diagnostics_enabled:
            msg.diagnostic_status = _with_evaluation_timing(msg.diagnostic_status, timing)
        self.tactile_pub.publish(msg)
        self._tactile_previous_callback_duration_s = time.monotonic() - publish_mono

    def _on_source_gc_event(self, phase: str, _info: dict[str, int]) -> None:
        """Measure GC pauses without changing collection policy or triggering collection."""

        thread_id = threading.get_native_id()
        if phase == "start":
            self._source_gc_started[thread_id] = (time.monotonic(), time.thread_time())
            return
        if phase != "stop":
            return
        started = self._source_gc_started.pop(thread_id, None)
        if started is None:
            return
        self._source_gc_pause_wall_s += time.monotonic() - started[0]
        self._source_gc_pause_thread_s += time.thread_time() - started[1]
        self._source_gc_collections += 1

    def _run_bounded_source(self) -> None:
        """Run physical integration independently of ROS callbacks and publishers."""

        try:
            self._source_affinity = _apply_development_source_affinity(os.environ)
            while not self._source_stop_event.is_set():
                for process in self._publication_processes:
                    if process.exitcode is not None:
                        raise RuntimeError(
                            f"publication process {process.name} exited unexpectedly "
                            f"with code {process.exitcode}"
                        )
                wait_s = max(0.0, self._physics_expected_mono - time.monotonic())
                if self._source_stop_event.wait(wait_s):
                    break
                self._on_timer()
        except BaseException as exc:
            self._source_failure = exc
            self._source_stop_event.set()
            self._close_publication_mailboxes()
            self.get_logger().error(
                f"physical source stopped fail-closed: {type(exc).__name__}: {exc}"
            )

    def _close_publication_mailboxes(self) -> None:
        closed: set[int] = set()
        for mailbox_name in ("_state_mailbox", "_tactile_mailbox", "_auxiliary_mailbox"):
            mailbox = getattr(self, mailbox_name, None)
            if mailbox is not None and id(mailbox) not in closed:
                closed.add(id(mailbox))
                mailbox.close()

    def destroy_node(self):
        previous_mask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})
        try:
            stop_event = getattr(self, "_source_stop_event", None)
            source_thread = getattr(self, "_source_thread", None)
            source_process = getattr(self, "_source_process", None)
            if stop_event is not None:
                stop_event.set()
            if source_thread is not None and source_thread is not threading.current_thread():
                source_thread.join(timeout=2.0)
                if source_thread.is_alive():
                    self.get_logger().error("physical source thread did not stop within 2 seconds")
            if source_process is not None:
                source_process.join(timeout=2.0)
                if source_process.is_alive():
                    self.get_logger().error(
                        "physical source process did not stop within 2 seconds"
                    )
                    source_process.terminate()
                    source_process.join(timeout=2.0)
                source_process.close()
            self._close_publication_mailboxes()
            for process in getattr(self, "_publication_processes", ()):
                process.join(timeout=2.0)
                if process.is_alive():
                    self.get_logger().error(
                        f"publication process {process.name} did not stop within 2 seconds"
                    )
                    process.terminate()
                    process.join(timeout=2.0)
                process.close()
            trace_root = getattr(self, "_eval007_trace_root", None)
            source_trace = getattr(self, "_source_timing_trace", None)
            publication_traces = getattr(self, "_publication_timing_traces", {})
            if trace_root is not None and source_trace is not None:
                try:
                    _retain_eval007_timing_trace(
                        trace_root,
                        source_trace,
                        publication_traces,
                    )
                except Exception as exc:
                    self.get_logger().error(
                        "EVAL-007 timing trace could not be retained: "
                        f"{type(exc).__name__}: {exc}"
                    )
            callback = getattr(self, "_source_gc_callback", None)
            if callback in gc.callbacks:
                gc.callbacks.remove(callback)
            return super().destroy_node()
        finally:
            # A launch process may deliver a second SIGINT while the first one
            # is already driving rclpy shutdown.  Do not re-deliver that
            # pending duplicate into completed teardown.
            if signal.SIGINT not in signal.sigpending():
                signal.pthread_sigmask(signal.SIG_SETMASK, previous_mask)

    def _joint_state_msg(
        self,
        stamp,
        q: np.ndarray,
        q_dot: np.ndarray,
        *,
        diagnostic_status: str | None = None,
    ) -> CtrJointState:
        msg = CtrJointState()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.insertion_position = [float(value) for value in q[:3]]
        msg.rotation_position = [float(value) for value in q[3:]]
        msg.joint_velocity = [float(value) for value in q_dot]
        msg.valid = True
        msg.diagnostic_status = (
            self.last_diagnostic_status if diagnostic_status is None else diagnostic_status
        )
        return msg

    def _standard_joint_state_msg(self, stamp, q: np.ndarray, q_dot: np.ndarray) -> JointState:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [
            "rho1",
            "rho2",
            "rho3",
            "theta1",
            "theta2",
            "theta3",
        ]
        msg.position = [float(value) for value in q]
        msg.velocity = [float(value) for value in q_dot]
        return msg

    def _backbone_msg(
        self,
        stamp,
        points: list[Point],
        *,
        diagnostic_status: str | None = None,
    ) -> CtrBackbone:
        msg = CtrBackbone()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.points = points
        msg.valid = True
        msg.diagnostic_status = (
            self.last_diagnostic_status if diagnostic_status is None else diagnostic_status
        )
        return msg

    def _backbone_points_for_publish(self, points: np.ndarray) -> list[Point]:
        if len(self._backbone_point_cache) != len(points):
            self._backbone_point_cache = [Point() for _ in range(len(points))]
        for message_point, source_point in zip(self._backbone_point_cache, points):
            message_point.x = float(source_point[0])
            message_point.y = float(source_point[1])
            message_point.z = float(source_point[2])
        return self._backbone_point_cache

    def _tip_pose(self, stamp, tip_position: np.ndarray) -> PoseStamped:
        msg = self._tip_pose_cache
        if msg is None:
            msg = PoseStamped()
            self._tip_pose_cache = msg
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(tip_position[0])
        msg.pose.position.y = float(tip_position[1])
        msg.pose.position.z = float(tip_position[2])
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        return msg

    def _state_msg(
        self,
        stamp,
        q: np.ndarray,
        q_dot: np.ndarray,
        backbone_points: list[Point],
        tip_pose: PoseStamped,
        *,
        diagnostic_status: str | None = None,
    ) -> CtrState:
        msg = self._state_message_cache
        if msg is None:
            msg = CtrState()
            self._state_message_cache = msg
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.q = [float(value) for value in q]
        msg.q_dot = [float(value) for value in q_dot]
        msg.backbone = backbone_points
        msg.tip_pose = tip_pose.pose
        msg.tactile_force.x = 0.0
        msg.tactile_force.y = 0.0
        msg.tactile_force.z = 0.0
        msg.contact = False
        msg.valid = True
        msg.diagnostic_status = (
            self.last_diagnostic_status if diagnostic_status is None else diagnostic_status
        )
        return msg

    def _tactile_msg(self, stamp, tip_position: np.ndarray) -> CtrTactileState:
        if self.lumen is None:
            sample = simulate_tactile(None, self.tactile_parameters)
        else:
            clearance = self.lumen.point_clearance(tip_position).physical_clearance
            sample = simulate_tactile(clearance, self.tactile_parameters)
        processed = self.tactile_processor.process(
            [sample.raw_signal] if sample.valid else None,
            clearance_m=sample.clearance_m,
            geometric_contact=sample.contact,
            timestamp_s=stamp.sec + stamp.nanosec * 1e-9,
        )

        msg = CtrTactileState()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.raw_values = [float(sample.raw_signal)]
        msg.filtered_values = [float(processed.filtered_signal)]
        msg.force.x = 0.0
        msg.force.y = 0.0
        msg.force.z = 0.0
        msg.force_magnitude = float(processed.force_n)
        msg.contact = bool(processed.contact)
        msg.warning = bool(processed.warning)
        msg.stop = bool(processed.stop)
        msg.valid = bool(processed.valid)
        msg.diagnostic_status = processed.diagnostic_status
        msg.clearance_m = float(processed.clearance_m)
        msg.source = "simulated"
        msg.region = int(processed.region)
        return msg

    def _diagnostics_msg(
        self,
        stamp,
        command_age: float,
        model_status: str,
        *,
        command_valid: bool | None = None,
        command_saturated: bool | None = None,
        diagnostic_status: str | None = None,
    ) -> DiagnosticArray:
        valid = self.command_valid if command_valid is None else command_valid
        saturated = self.command_saturated if command_saturated is None else command_saturated
        status_text = (
            self.last_diagnostic_status if diagnostic_status is None else diagnostic_status
        )
        status = DiagnosticStatus()
        status.name = "ctr_sim/simulator_node"
        status.hardware_id = "simulation"
        status.level = DiagnosticStatus.OK if valid else DiagnosticStatus.WARN
        status.message = status_text
        status.values = [
            KeyValue(key="runtime_mode", value=str(self.get_parameter("runtime_mode").value)),
            KeyValue(key="command_age_s", value=f"{command_age:.6f}"),
            KeyValue(key="command_saturated", value=str(saturated)),
            KeyValue(key="model_status", value=model_status),
            KeyValue(key="TODO-SIM-001", value="Actuator nonidealities are not implemented."),
        ]
        msg = DiagnosticArray()
        msg.header.stamp = stamp
        msg.status = [status]
        return msg

    def _marker_array_msg(
        self,
        stamp,
        backbone_points: list[Point],
        backbone_array: np.ndarray,
        *,
        include_development: bool | None = None,
        include_static: bool = True,
    ) -> MarkerArray:
        backbone = Marker()
        backbone.header.stamp = stamp
        backbone.header.frame_id = self.frame_id
        backbone.ns = "ctr_backbone"
        backbone.id = 0
        backbone.type = Marker.LINE_STRIP
        backbone.action = Marker.ADD
        backbone.scale.x = 0.002
        backbone.color = ColorRGBA(r=0.1, g=0.45, b=1.0, a=1.0)
        backbone.points = backbone_points

        tip = Marker()
        tip.header.stamp = stamp
        tip.header.frame_id = self.frame_id
        tip.ns = "tip_marker"
        tip.id = 1
        tip.type = Marker.SPHERE
        tip.action = Marker.ADD
        tip.scale.x = 0.008
        tip.scale.y = 0.008
        tip.scale.z = 0.008
        tip.color = ColorRGBA(r=1.0, g=0.15, b=0.1, a=1.0)
        if backbone_points:
            tip.pose.position = backbone_points[-1]
        tip.pose.orientation.w = 1.0

        target = Marker()
        target.header.stamp = stamp
        target.header.frame_id = self.frame_id
        target.ns = "target_marker"
        target.id = 2
        target.type = Marker.LINE_STRIP
        target.action = Marker.ADD
        target.scale.x = 0.0015
        target.color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=1.0)
        target.pose.orientation.w = 1.0
        target.points = [
            _point_from_array(
                self.target_position
                + 0.006
                * np.array(
                    [
                        math.cos(2.0 * math.pi * index / 32.0),
                        math.sin(2.0 * math.pi * index / 32.0),
                        0.0,
                    ],
                    dtype=float,
                )
            )
            for index in range(33)
        ]

        markers = [backbone, tip, target]
        if include_static:
            markers.extend(self._static_lumen_markers_for_publish(stamp))
        markers.extend(self._dynamic_lumen_markers_for_publish(stamp, backbone_array))
        if self.lumen_mode == "cylindrical" and isinstance(self.lumen, CylindricalLumen):
            markers.extend(self._lumen_markers(stamp, backbone_array))
        if include_development is None:
            include_development = self.development_visualization
        if include_development and self._reference_path_points.shape[0] > 0:
            markers.extend(
                build_reference_path_markers(
                    self._reference_path_points,
                    self._reference_path_frame_id,
                    stamp,
                )
            )
        if include_development and self._tip_trajectory is not None:
            actual_path = build_actual_tip_path_marker(
                self._tip_trajectory.points(),
                self.frame_id,
                stamp,
            )
            if actual_path is not None:
                markers.append(actual_path)
        return MarkerArray(markers=markers)

    def _development_visualization_publish_due(self, stamp) -> bool:
        if not self.development_visualization:
            return False
        stamp_s = _stamp_seconds(stamp)
        last_publish = self._last_development_visualization_publish_time_s
        period = 1.0 / self.lumen_marker_config.marker_publish_rate
        if last_publish is None or stamp_s < last_publish or stamp_s - last_publish >= period - 1.0e-12:
            self._last_development_visualization_publish_time_s = stamp_s
            return True
        return False

    def _runtime_marker_publication_due(self, stamp) -> bool:
        """Honor the configured marker rate independently of the physics/sensor rate.

        Marker construction is presentation work, not a physical sample.  Keeping
        it at ``simulation.visualization.marker_publish_rate`` prevents the large
        marker array from starving state and tactile evidence while preserving the
        configured visual update cadence in both ordinary and diagnostic runs.
        """

        stamp_s = _stamp_seconds(stamp)
        last_publish = self._last_runtime_marker_publish_time_s
        period = 1.0 / self.lumen_marker_config.marker_publish_rate
        if (
            last_publish is None
            or stamp_s < last_publish
            or stamp_s - last_publish >= period - 1.0e-12
        ):
            self._last_runtime_marker_publish_time_s = stamp_s
            return True
        return False

    def _publish_development_marker_topics(self, marker_array: MarkerArray) -> None:
        if not self.development_marker_pubs:
            return
        by_namespace: dict[str, list[Marker]] = {
            namespace: [] for namespace in self.development_marker_pubs
        }
        for marker in marker_array.markers:
            if marker.ns in by_namespace:
                by_namespace[marker.ns].append(marker)
        for namespace, publisher in self.development_marker_pubs.items():
            if by_namespace[namespace]:
                publisher.publish(MarkerArray(markers=by_namespace[namespace]))

    def _static_lumen_markers_for_publish(self, stamp) -> list[Marker]:
        config = getattr(self, "lumen_marker_config", LumenMarkerConfig())
        if self.lumen_mode != "curved" or not isinstance(self.lumen, CurvedLumen):
            return self._clear_static_lumen_markers(stamp, reason=f"mode_{self.lumen_mode}")
        if not config.publish_lumen_markers:
            return self._clear_static_lumen_markers(stamp, reason="visualization_disabled")

        fingerprint = lumen_geometry_fingerprint(getattr(self, "config", self.lumen))
        cache_key = static_lumen_cache_key(fingerprint, config)
        if cache_key != self._static_lumen_cache_key:
            deletes = self._clear_static_lumen_markers(stamp, reason="cache_rebuild")
            try:
                markers = build_curved_static_lumen_markers(
                    self.lumen,
                    fingerprint,
                    self.lumen.frame_id,
                    config,
                    stamp,
                )
            except ValueError as exc:
                self._static_lumen_cache_key = None
                self._static_lumen_markers = []
                self._static_lumen_marker_keys = ()
                self._static_lumen_marker_frame_id = self.lumen.frame_id
                self._log_lumen_markers(
                    mode="curved",
                    frame=self.lumen.frame_id,
                    fingerprint=fingerprint,
                    centerline_points=int(self.lumen.centerline_points.shape[0]),
                    rings=0,
                    segments=config.ring_segments,
                    cached=False,
                    reason=f"generation_failed:{exc}",
                )
                return deletes
            self._static_lumen_cache_key = cache_key
            self._static_lumen_markers = markers
            self._static_lumen_marker_keys = marker_keys(markers)
            self._static_lumen_marker_frame_id = self.lumen.frame_id
            self._static_lumen_build_count += 1
            self._static_lumen_cache_hit_logged = False
            self._last_static_lumen_publish_time_s = _stamp_seconds(stamp)
            self._log_lumen_markers(
                mode="curved",
                frame=self.lumen.frame_id,
                fingerprint=fingerprint,
                centerline_points=int(self.lumen.centerline_points.shape[0]),
                rings=self._static_lumen_ring_count(config),
                segments=config.ring_segments,
                cached=False,
                reason="built",
            )
            return deletes + markers_with_stamp(markers, stamp)

        if not self._static_lumen_publish_due(stamp, config):
            return []
        if not self._static_lumen_cache_hit_logged:
            self._log_lumen_markers(
                mode="curved",
                frame=self.lumen.frame_id,
                fingerprint=fingerprint,
                centerline_points=int(self.lumen.centerline_points.shape[0]),
                rings=self._static_lumen_ring_count(config),
                segments=config.ring_segments,
                cached=True,
                reason="cache_hit",
            )
            self._static_lumen_cache_hit_logged = True
        return markers_with_stamp(self._static_lumen_markers, stamp)

    def _static_lumen_publish_due(self, stamp, config: LumenMarkerConfig) -> bool:
        if self.development_visualization:
            # Static development topics are transient-local, so one authenticated
            # publication is sufficient even when RViz joins after the simulator.
            return False
        stamp_s = _stamp_seconds(stamp)
        last_publish = self._last_static_lumen_publish_time_s
        period = 1.0 / config.marker_publish_rate
        if last_publish is None or stamp_s < last_publish or stamp_s - last_publish >= period - 1.0e-12:
            self._last_static_lumen_publish_time_s = stamp_s
            return True
        return False

    def _clear_static_lumen_markers(self, stamp, *, reason: str) -> list[Marker]:
        keys = tuple(getattr(self, "_static_lumen_marker_keys", ()))
        if not keys:
            self._static_lumen_cache_key = None
            self._static_lumen_markers = []
            self._static_lumen_marker_keys = ()
            self._last_static_lumen_publish_time_s = None
            return []
        frame = getattr(self, "_static_lumen_marker_frame_id", self.frame_id)
        deletes = build_static_lumen_delete_markers(keys, frame, stamp)
        self._log_lumen_markers(
            mode=self.lumen_mode,
            frame=frame,
            fingerprint="none",
            centerline_points=0,
            rings=0,
            segments=getattr(getattr(self, "lumen_marker_config", None), "ring_segments", 0),
            cached=False,
            reason=reason,
        )
        self._static_lumen_cache_key = None
        self._static_lumen_markers = []
        self._static_lumen_marker_keys = ()
        self._last_static_lumen_publish_time_s = None
        self._static_lumen_cache_hit_logged = False
        return deletes

    def _static_lumen_ring_count(self, config: LumenMarkerConfig) -> int:
        if not isinstance(self.lumen, CurvedLumen):
            return 0
        point_count = int(self.lumen.centerline_points.shape[0])
        return len(tuple(dict.fromkeys(list(range(0, point_count, config.ring_stride)) + [point_count - 1])))

    def _log_lumen_markers(
        self,
        *,
        mode: str,
        frame: str,
        fingerprint: str,
        centerline_points: int,
        rings: int,
        segments: int,
        cached: bool,
        reason: str,
    ) -> None:
        try:
            logger = self.get_logger()
        except Exception:
            return
        logger.info(
            "LUMEN_MARKERS "
            f"mode={mode} "
            f"frame={frame} "
            f"fingerprint={fingerprint} "
            f"centerline_points={centerline_points} "
            f"rings={rings} "
            f"segments={segments} "
            f"cached={str(cached).lower()} "
            f"build_count={self._static_lumen_build_count} "
            f"reason={reason}"
        )

    def _dynamic_lumen_markers_for_publish(self, stamp, backbone_array: np.ndarray) -> list[Marker]:
        config = getattr(self, "lumen_marker_config", LumenMarkerConfig())
        if not config.publish_lumen_markers:
            return self._clear_dynamic_lumen_markers(stamp, reason="visualization_disabled")
        if not config.publish_lumen_diagnostics:
            return self._clear_dynamic_lumen_markers(stamp, reason="diagnostics_disabled")
        if self.lumen_mode != "curved" or not isinstance(self.lumen, CurvedLumen):
            return self._clear_dynamic_lumen_markers(stamp, reason=f"mode_{self.lumen_mode}")

        try:
            diagnostic = build_lumen_runtime_diagnostic(self.lumen, backbone_array, self.lumen_mode)
            markers = build_dynamic_lumen_diagnostic_markers(diagnostic, stamp)
        except ValueError as exc:
            self._log_lumen_diagnostic_unavailable(
                mode=self.lumen_mode,
                frame=getattr(self.lumen, "frame_id", self.frame_id),
                reason=f"generation_failed:{exc}",
            )
            return self._clear_dynamic_lumen_markers(stamp, reason="generation_failed")

        new_keys = marker_keys(markers)
        old_keys = tuple(getattr(self, "_dynamic_lumen_marker_keys", ()))
        obsolete_keys = tuple(key for key in old_keys if key not in new_keys)
        deletes: list[Marker] = []
        if obsolete_keys:
            deletes = build_dynamic_lumen_delete_markers(
                obsolete_keys,
                getattr(self, "_dynamic_lumen_marker_frame_id", diagnostic.frame_id),
                stamp,
            )
        self._dynamic_lumen_marker_keys = new_keys
        self._dynamic_lumen_marker_frame_id = diagnostic.frame_id
        self._lumen_diagnostic_update_count += 1
        self._log_lumen_diagnostic(diagnostic, reason="updated")
        return deletes + markers

    def _clear_dynamic_lumen_markers(self, stamp, *, reason: str) -> list[Marker]:
        keys = tuple(getattr(self, "_dynamic_lumen_marker_keys", ()))
        if not keys:
            return []
        frame = getattr(self, "_dynamic_lumen_marker_frame_id", self.frame_id)
        deletes = build_dynamic_lumen_delete_markers(keys, frame, stamp)
        self._dynamic_lumen_marker_keys = ()
        self._log_lumen_diagnostic_unavailable(mode=self.lumen_mode, frame=frame, reason=reason)
        return deletes

    def _log_lumen_diagnostic(self, diagnostic: LumenRuntimeDiagnostic, *, reason: str) -> None:
        signature = (
            diagnostic.geometry_mode,
            diagnostic.constraint_type,
            diagnostic.status,
        )
        if signature == self._last_lumen_diagnostic_log_signature:
            return
        self._last_lumen_diagnostic_log_signature = signature
        try:
            logger = self.get_logger()
        except Exception:
            return
        logger.info(
            "LUMEN_DIAGNOSTIC "
            f"mode={diagnostic.geometry_mode} "
            f"constraint={diagnostic.constraint_type} "
            f"state={diagnostic.status} "
            f"backbone_index={diagnostic.backbone_index} "
            f"physical_clearance={diagnostic.physical_clearance:.9f} "
            f"safety_clearance={diagnostic.safety_clearance:.9f} "
            f"collision={str(diagnostic.physical_collision).lower()} "
            f"margin_violation={str(diagnostic.safety_margin_violation).lower()} "
            f"frame={diagnostic.frame_id} "
            f"reason={reason}"
        )

    def _log_lumen_diagnostic_unavailable(self, *, mode: str, frame: str, reason: str) -> None:
        signature = (str(mode), "unavailable", str(reason))
        if signature == self._last_lumen_diagnostic_log_signature:
            return
        self._last_lumen_diagnostic_log_signature = signature
        try:
            logger = self.get_logger()
        except Exception:
            return
        logger.info(
            "LUMEN_DIAGNOSTIC "
            f"mode={mode} "
            "constraint=unavailable "
            "state=UNAVAILABLE "
            "backbone_index=-1 "
            "physical_clearance=unavailable "
            "safety_clearance=unavailable "
            "collision=false "
            "margin_violation=false "
            f"frame={frame} "
            f"reason={reason}"
        )

    def _lumen_markers(self, stamp, backbone_array: np.ndarray) -> list[Marker]:
        assert self.lumen is not None
        clearance = self.lumen.backbone_clearance(backbone_array)
        center = self.lumen.axis_origin + 0.5 * self.lumen.length * self.lumen.axis_direction
        orientation = _quaternion_from_z_axis(self.lumen.axis_direction)

        wall = Marker()
        wall.header.stamp = stamp
        wall.header.frame_id = self.lumen.frame_id
        wall.ns = "cylindrical_lumen"
        wall.id = 10
        wall.type = Marker.CYLINDER
        wall.action = Marker.ADD
        wall.pose.position = _point_from_array(center)
        wall.pose.orientation.x = orientation[0]
        wall.pose.orientation.y = orientation[1]
        wall.pose.orientation.z = orientation[2]
        wall.pose.orientation.w = orientation[3]
        wall.scale.x = 2.0 * self.lumen.radius
        wall.scale.y = 2.0 * self.lumen.radius
        wall.scale.z = self.lumen.length
        wall.color = ColorRGBA(r=0.2, g=0.7, b=1.0, a=0.16)

        safety = Marker()
        safety.header.stamp = stamp
        safety.header.frame_id = self.lumen.frame_id
        safety.ns = "cylindrical_lumen"
        safety.id = 11
        safety.type = Marker.CYLINDER
        safety.action = Marker.ADD
        safety.pose = wall.pose
        safety.scale.x = 2.0 * self.lumen.preferred_radius
        safety.scale.y = 2.0 * self.lumen.preferred_radius
        safety.scale.z = self.lumen.length
        safety.color = ColorRGBA(r=0.1, g=0.9, b=0.4, a=0.10)

        axis = Marker()
        axis.header.stamp = stamp
        axis.header.frame_id = self.lumen.frame_id
        axis.ns = "cylindrical_lumen"
        axis.id = 12
        axis.type = Marker.LINE_STRIP
        axis.action = Marker.ADD
        axis.scale.x = 0.001
        axis.color = ColorRGBA(r=0.05, g=0.05, b=0.05, a=1.0)
        axis.points = [
            _point_from_array(self.lumen.axis_origin),
            _point_from_array(self.lumen.axis_origin + self.lumen.length * self.lumen.axis_direction),
        ]

        closest = Marker()
        closest.header.stamp = stamp
        closest.header.frame_id = self.lumen.frame_id
        closest.ns = "cylindrical_lumen"
        closest.id = 13
        closest.type = Marker.SPHERE
        closest.action = Marker.ADD
        closest.scale.x = 0.006
        closest.scale.y = 0.006
        closest.scale.z = 0.006
        if clearance.collision_count:
            closest.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        elif clearance.safety_margin_violation_count:
            closest.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=1.0)
        else:
            closest.color = ColorRGBA(r=0.0, g=0.8, b=0.2, a=1.0)
        closest.pose.position = _point_from_array(backbone_array[clearance.closest_backbone_point_index])
        closest.pose.orientation.w = 1.0

        return [wall, safety, axis, closest]


def _backbone_points_from_sample(
    sample: _PhysicalSample,
    model: ApproximateCTRModel | None,
) -> tuple[tuple[float, float, float], ...]:
    if sample.backbone_points:
        return sample.backbone_points
    if model is None:
        raise RuntimeError("backbone reconstruction model is unavailable")
    result = model.forward_kinematics(sample.q)
    return tuple(
        tuple(float(value) for value in point)
        for point in result.backbone_points
    )


def _state_message_from_sample(
    sample: _PhysicalSample,
    frame_id: str,
    *,
    model: ApproximateCTRModel | None = None,
) -> CtrState:
    stamp = sample.stamp()
    backbone_points = _backbone_points_from_sample(sample, model)
    msg = CtrState()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.q = list(sample.q)
    msg.q_dot = list(sample.q_dot)
    msg.backbone = [_point_from_array(point) for point in backbone_points]
    msg.tip_pose = _tip_pose_message(stamp, frame_id, sample.tip_position).pose
    msg.tactile_force.x = 0.0
    msg.tactile_force.y = 0.0
    msg.tactile_force.z = 0.0
    msg.contact = False
    msg.valid = True
    msg.diagnostic_status = sample.diagnostic_status
    return msg


def _safety_joint_state_message_from_sample(
    sample: _PhysicalSample,
    frame_id: str,
) -> CtrJointState:
    """Return compact exact-q safety evidence from one physical sample."""

    msg = CtrJointState()
    msg.header.stamp = sample.stamp()
    msg.header.frame_id = frame_id
    msg.insertion_position = list(sample.q[:3])
    msg.rotation_position = list(sample.q[3:])
    msg.joint_velocity = list(sample.q_dot)
    msg.valid = True
    msg.diagnostic_status = sample.diagnostic_status
    return msg


def _tactile_message_from_sample(
    sample: _PhysicalSample,
    frame_id: str,
) -> CtrTactileState:
    tactile = sample.tactile
    if tactile is None:
        raise RuntimeError("physical sample has no tactile evidence")
    msg = CtrTactileState()
    msg.header.stamp = sample.stamp()
    msg.header.frame_id = frame_id
    msg.raw_values = [tactile.raw_signal]
    msg.filtered_values = [tactile.filtered_signal]
    msg.force.x = 0.0
    msg.force.y = 0.0
    msg.force.z = 0.0
    msg.force_magnitude = tactile.force_n
    msg.contact = tactile.contact
    msg.warning = tactile.warning
    msg.stop = tactile.stop
    msg.valid = tactile.valid
    msg.diagnostic_status = tactile.diagnostic_status
    msg.clearance_m = tactile.clearance_m
    msg.source = "simulated"
    msg.region = tactile.region
    return msg


def _publish_auxiliary_process_sample(
    sample: _PhysicalSample,
    frame_id: str,
    publishers: dict,
    last_diagnostics_publish_mono: float,
    *,
    model: ApproximateCTRModel | None,
) -> None:
    stamp = sample.stamp()
    backbone_points = _backbone_points_from_sample(sample, model)
    if publishers["tip"].get_subscription_count() > 0:
        publishers["tip"].publish(_tip_pose_message(stamp, frame_id, sample.tip_position))
    if publishers["joint"].get_subscription_count() > 0:
        joint = CtrJointState()
        joint.header.stamp = stamp
        joint.header.frame_id = frame_id
        joint.insertion_position = list(sample.q[:3])
        joint.rotation_position = list(sample.q[3:])
        joint.joint_velocity = list(sample.q_dot)
        joint.valid = True
        joint.diagnostic_status = sample.diagnostic_status
        publishers["joint"].publish(joint)
    if publishers["standard_joint"].get_subscription_count() > 0:
        joint = JointState()
        joint.header.stamp = stamp
        joint.name = ["rho1", "rho2", "rho3", "theta1", "theta2", "theta3"]
        joint.position = list(sample.q)
        joint.velocity = list(sample.q_dot)
        publishers["standard_joint"].publish(joint)
    if publishers["backbone"].get_subscription_count() > 0:
        backbone = CtrBackbone()
        backbone.header.stamp = stamp
        backbone.header.frame_id = frame_id
        backbone.points = [_point_from_array(point) for point in backbone_points]
        backbone.valid = True
        backbone.diagnostic_status = sample.diagnostic_status
        publishers["backbone"].publish(backbone)
    if (
        publishers["diagnostics"].get_subscription_count() > 0
        and time.monotonic() - last_diagnostics_publish_mono >= 0.10
    ):
        status = DiagnosticStatus()
        status.name = "ctr_sim/simulator_node"
        status.hardware_id = "simulation"
        status.level = DiagnosticStatus.OK if sample.command_valid else DiagnosticStatus.WARN
        status.message = sample.diagnostic_status
        status.values = [
            KeyValue(key="runtime_mode", value="simulation"),
            KeyValue(key="command_age_s", value=f"{sample.command_age_s:.6f}"),
            KeyValue(key="command_saturated", value=str(sample.command_saturated)),
            KeyValue(key="model_status", value=sample.model_status),
            KeyValue(key="TODO-SIM-001", value="Actuator nonidealities are not implemented."),
        ]
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = stamp
        diagnostic.status = [status]
        publishers["diagnostics"].publish(diagnostic)


def _point_from_array(values: Iterable[float]) -> Point:
    x, y, z = [float(value) for value in values]
    point = Point()
    point.x = x
    point.y = y
    point.z = z
    return point


def _tip_pose_message(stamp, frame_id: str, position: Iterable[float]) -> PoseStamped:
    x, y, z = (float(value) for value in position)
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    return msg


def _vector3(values, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain 3 finite values")
    return array


def _optional_vector3_parameter(values) -> list[float] | None:
    if values is None or values == "":
        return None
    if isinstance(values, (list, tuple)) and len(values) == 0:
        return None
    return [float(value) for value in _vector3(values, "cylinder_target_position")]


def _apply_development_source_affinity(environment: dict[str, str]) -> tuple[int, ...]:
    """Apply the runner-provided exclusive physical-core group to this thread."""

    if environment.get(_DEVELOPMENT_EVALUATION_CPU_PARTITION_ENV) != "1":
        return ()
    raw = environment.get(_DEVELOPMENT_SIMULATOR_CPU_LIST_ENV, "")
    if re.fullmatch(r"[0-9]+(?:,[0-9]+)*", raw) is None:
        raise RuntimeError("development simulator CPU list is missing or malformed")
    reserved_core_cpus = tuple(int(value) for value in raw.split(","))
    cpus = reserved_core_cpus[:1]
    if len(set(reserved_core_cpus)) != len(reserved_core_cpus):
        raise RuntimeError("development simulator CPU list contains duplicates")
    try:
        os.sched_setaffinity(0, set(cpus))
        observed = set(os.sched_getaffinity(0))
    except (AttributeError, OSError) as exc:
        raise RuntimeError("development simulator CPU affinity could not be applied") from exc
    if observed != set(cpus):
        raise RuntimeError("development simulator CPU affinity did not reconcile")
    return cpus


def _apply_development_publication_affinity(
    kind: str,
    environment: dict[str, str],
    topology_root: Path = Path("/sys/devices/system/cpu"),
) -> tuple[int, ...]:
    """Give tactile and state publication separate inherited physical cores."""

    if environment.get(_DEVELOPMENT_EVALUATION_CPU_PARTITION_ENV) != "1":
        return ()
    if kind not in {"state", "tactile", "auxiliary"}:
        raise RuntimeError("unknown development publication affinity kind")
    try:
        allowed = sorted(int(cpu) for cpu in os.sched_getaffinity(0))
    except (AttributeError, OSError) as exc:
        raise RuntimeError("development publisher affinity is unavailable") from exc
    cores: dict[tuple[int, int], list[int]] = {}
    for cpu in allowed:
        topology = topology_root / f"cpu{cpu}" / "topology"
        try:
            package_id = int((topology / "physical_package_id").read_text().strip())
            core_id = int((topology / "core_id").read_text().strip())
        except (OSError, ValueError) as exc:
            raise RuntimeError("development publisher CPU topology is unavailable") from exc
        cores.setdefault((package_id, core_id), []).append(cpu)
    groups = [tuple(sorted(values)) for _key, values in sorted(cores.items())]
    if len(groups) < 2:
        raise RuntimeError("development publisher affinity requires two physical cores")
    # State serialization owns one complete base core. Tactile publication is
    # lightweight because physical tactile generation already occurred in the
    # source process; it may share the other core with optional auxiliary data.
    if kind == "tactile":
        selected = groups[1][:1]
    elif kind == "auxiliary":
        selected = groups[0][-1:]
    else:
        selected = groups[0][:1]
    try:
        os.sched_setaffinity(0, set(selected))
        observed = set(os.sched_getaffinity(0))
    except (AttributeError, OSError) as exc:
        raise RuntimeError("development publisher affinity could not be applied") from exc
    if observed != set(selected):
        raise RuntimeError("development publisher affinity did not reconcile")
    return selected


def _stamp_seconds(stamp) -> float:
    return float(getattr(stamp, "sec", 0)) + 1.0e-9 * float(getattr(stamp, "nanosec", 0))


def _with_evaluation_timing(status: str, values: dict[str, float | int]) -> str:
    return _with_evaluation_timing_schema(
        status,
        "ctr_tactile_timing_v1",
        values,
    )


def _with_evaluation_timing_schema(
    status: str,
    schema: str,
    values: dict[str, float | int],
) -> str:
    fields = ";".join(f"{key}={values[key]}" for key in sorted(values))
    return f"{status}|{schema}|{fields}"


def _quaternion_from_z_axis(axis: np.ndarray) -> tuple[float, float, float, float]:
    target = np.asarray(axis, dtype=float)
    target = target / max(float(np.linalg.norm(target)), 1.0e-12)
    source = np.array([0.0, 0.0, 1.0], dtype=float)
    dot = float(np.clip(np.dot(source, target), -1.0, 1.0))
    if dot > 1.0 - 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    if dot < -1.0 + 1.0e-12:
        return (1.0, 0.0, 0.0, 0.0)
    cross = np.cross(source, target)
    quat = np.array([cross[0], cross[1], cross[2], 1.0 + dot], dtype=float)
    quat /= np.linalg.norm(quat)
    return (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))


def main(args=None):
    rclpy.init(args=args)
    node = CTRSimulatorNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
