import copy
from dataclasses import FrozenInstanceError
import inspect
import json
import os
import sys
import threading
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_model"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))


try:
    import ctr_interfaces.msg as ctr_interfaces_msg_module  # noqa: F401
    for required_name in ("CtrBackbone", "CtrJointCommand", "CtrJointState", "CtrState", "CtrTactileState"):
        if not hasattr(ctr_interfaces_msg_module, required_name):
            raise ImportError(required_name)
except ImportError:
    ctr_interfaces_module = types.ModuleType("ctr_interfaces")
    ctr_interfaces_msg_module = types.ModuleType("ctr_interfaces.msg")
    for name in ("CtrBackbone", "CtrJointCommand", "CtrJointState", "CtrState", "CtrTactileState"):
        setattr(ctr_interfaces_msg_module, name, type(name, (), {}))
    sys.modules["ctr_interfaces"] = ctr_interfaces_module
    sys.modules["ctr_interfaces.msg"] = ctr_interfaces_msg_module


from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Path as NavPath  # noqa: E402
from ctr_bringup.parameter_validation import load_parameter_files  # noqa: E402
from ctr_mppi_controller.curved_lumen import CurvedLumen  # noqa: E402
from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen  # noqa: E402
from ctr_mppi_controller.lumen_factory import (  # noqa: E402
    config_with_lumen_overrides,
    lumen_geometry_from_config,
    lumen_mode_from_config,
)
from ctr_sim.lumen_markers import (  # noqa: E402
    BoundedTipTrajectory,
    CURVED_STATIC_LUMEN_MARKER_KEYS,
    DYNAMIC_LUMEN_MARKER_KEYS,
    LumenMarkerConfig,
)
from ctr_sim.lumen_diagnostics import STATUS_COLLISION, STATUS_MARGIN  # noqa: E402
import ctr_sim.nodes.simulator_node as simulator_node_module  # noqa: E402
from ctr_sim.nodes.simulator_node import (  # noqa: E402
    CTRSimulatorNode,
    _BoundedProcessTimingTrace,
    _LatestPhysicalSampleMailbox,
    _LatestProcessCommandMailbox,
    _LatestProcessPhysicalSampleMailbox,
    _PhysicalSample,
    _PhysicalTactileSample,
    _PHYSICAL_SOURCE_TRACE_FIELDS,
    _PUBLICATION_TRACE_FIELDS,
    _apply_development_publication_affinity,
    _apply_development_source_affinity,
    _next_physical_deadline,
    _point_from_array,
    _retain_eval007_timing_trace,
    _wait_for_physical_deadline,
    development_marker_qos_profile,
    source_evidence_qos_profile,
)
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402


CONFIG_FILES = [
    REPO_ROOT / "config" / "robot_params.yaml",
    REPO_ROOT / "config" / "model_params.yaml",
    REPO_ROOT / "config" / "mppi_params.yaml",
    REPO_ROOT / "config" / "simulation_params.yaml",
    REPO_ROOT / "config" / "safety_params.yaml",
    REPO_ROOT / "config" / "tactile_params.yaml",
    REPO_ROOT / "config" / "hardware_params.yaml",
]


def load_config():
    return copy.deepcopy(load_parameter_files(CONFIG_FILES))


def no_lumen_config():
    return config_with_lumen_overrides(
        load_config(),
        enable_cylindrical_lumen=False,
        enable_curved_lumen=False,
    )


def cylinder_config():
    return config_with_lumen_overrides(
        load_config(),
        enable_cylindrical_lumen=True,
        enable_curved_lumen=False,
    )


def curved_config(lumen_type="circular_arc"):
    return config_with_lumen_overrides(
        load_config(),
        enable_cylindrical_lumen=False,
        enable_curved_lumen=True,
        curved_lumen_type=lumen_type,
    )


def simulator_shell(config):
    node = object.__new__(CTRSimulatorNode)
    node.config = config
    node.frame_id = config["robot"]["frames"]["base"]
    node.target_position = np.asarray(config["goal"]["position"], dtype=float)
    node.lumen_mode = lumen_mode_from_config(config)
    node.lumen_geometry = lumen_geometry_from_config(config)
    node.lumen = node.lumen_geometry
    node.lumen_marker_config = LumenMarkerConfig.from_mapping(config["simulation"].get("visualization", {}))
    node._static_lumen_cache_key = None
    node._static_lumen_markers = []
    node._static_lumen_marker_keys = ()
    node._static_lumen_marker_frame_id = node.frame_id
    node._last_static_lumen_publish_time_s = None
    node._last_development_visualization_publish_time_s = None
    node._last_runtime_marker_publish_time_s = None
    node._static_lumen_build_count = 0
    node._static_lumen_cache_hit_logged = False
    node._dynamic_lumen_marker_keys = ()
    node._dynamic_lumen_marker_frame_id = node.frame_id
    node._last_lumen_diagnostic_log_signature = None
    node._lumen_diagnostic_update_count = 0
    node.development_simulation = False
    node.development_visualization = False
    node.evaluation_diagnostics_enabled = False
    node.development_marker_pubs = {}
    node._reference_path_points = np.empty((0, 3), dtype=np.float64)
    node._reference_path_frame_id = node.frame_id
    node._tip_trajectory = None
    return node


def sample_backbone():
    return np.array(
        [
            [0.0, 0.0, 0.0],
            [0.003, 0.0, 0.040],
            [0.006, 0.0, 0.080],
        ],
        dtype=float,
    )


def physical_sample(sequence=7):
    return _PhysicalSample(
        sequence=sequence,
        stamp_sec=12,
        stamp_nanosec=34,
        expected_monotonic_s=10.01,
        source_start_monotonic_s=10.012,
        source_complete_monotonic_s=10.013,
        source_lateness_s=0.002,
        source_duration_s=0.001,
        q=(0.0,) * 6,
        q_dot=(0.0,) * 6,
        tip_position=(0.01, 0.0, 0.08),
        backbone_points=((0.0, 0.0, 0.0), (0.01, 0.0, 0.08)),
        command_age_s=0.02,
        command_saturated=False,
        command_valid=True,
        diagnostic_status="safe command accepted",
        model_status="model valid",
        tactile=_PhysicalTactileSample(
            raw_signal=0.0,
            filtered_signal=0.0,
            force_n=0.0,
            clearance_m=0.02,
            contact=False,
            warning=False,
            stop=False,
            valid=True,
            diagnostic_status="simulated tactile valid",
            region=0,
        ),
        source_timing=(("callback_wall_s", 0.001), ("source_thread_id", 42)),
    )


class SimulatorNodeLumenRuntimeTest(unittest.TestCase):
    def test_tactile_sampling_is_bound_to_new_physical_state_and_records_source_timing(self):
        node = simulator_shell(curved_config("circular_arc"))
        node.dt = 0.01
        node.tactile_source_state_timeout = 0.10
        node._state_lock = __import__("threading").RLock()
        node._latest_tactile_tip = np.array([0.01, 0.0, 0.08])
        node._latest_state_source_mono = 10.0
        node._physics_sequence = 7
        node._physics_callback_start_mono = 10.0
        node._physics_callback_lateness_s = 0.002
        node._physics_callback_duration_s = 0.004
        node._tactile_sequence = 0
        node._tactile_previous_callback_duration_s = 0.0
        node.evaluation_diagnostics_enabled = True
        published = []
        node.tactile_pub = SimpleNamespace(publish=published.append)

        with patch.object(
            simulator_node_module.time,
            "monotonic",
            side_effect=(10.014, 10.015),
        ), patch.object(
            simulator_node_module,
            "_tactile_message_from_sample",
            return_value=SimpleNamespace(
                valid=True, diagnostic_status="eligible_no_contact"
            ),
        ):
            node._publish_tactile_for_physical_state(
                physical_sample(),
                mailbox_version=1,
            )

        self.assertEqual(1, len(published))
        status = published[0].diagnostic_status
        self.assertIn("|ctr_tactile_timing_v1|", status)
        self.assertIn("sequence=7", status)
        self.assertIn("physics_sequence=7", status)
        self.assertIn("mailbox_overwrites=0", status)
        self.assertTrue(published[0].valid)

    def test_latest_sample_mailbox_is_single_slot_and_sample_is_deeply_immutable(self):
        mailbox = _LatestPhysicalSampleMailbox()
        first = physical_sample(1)
        second = physical_sample(2)
        self.assertEqual(1, mailbox.put(first))
        self.assertEqual(2, mailbox.put(second))
        delivery = mailbox.take_after(0, threading.Event())
        self.assertEqual((2, second), delivery)
        with self.assertRaises(FrozenInstanceError):
            second.sequence = 3
        with self.assertRaises(TypeError):
            second.q[0] = 1.0
        mailbox.close()
        self.assertIsNone(mailbox.take_after(2, threading.Event()))

    def test_process_mailbox_is_single_slot_and_preserves_exact_immutable_sample(self):
        context = __import__("multiprocessing").get_context("spawn")
        mailbox = _LatestProcessPhysicalSampleMailbox(context)
        stop_event = context.Event()
        self.assertEqual(1, mailbox.put(physical_sample(1)))
        self.assertEqual(2, mailbox.put(physical_sample(2)))
        self.assertEqual((2, physical_sample(2)), mailbox.take_after(0, stop_event))
        self.assertEqual((2, physical_sample(2)), mailbox.take_after(1, stop_event))
        mailbox.close()
        self.assertIsNone(mailbox.take_after(2, stop_event))

    def test_bounded_process_timing_trace_preserves_exact_numeric_layers(self):
        context = __import__("multiprocessing").get_context("spawn")
        trace = _BoundedProcessTimingTrace(context, _PHYSICAL_SOURCE_TRACE_FIELDS)
        first = tuple(range(len(_PHYSICAL_SOURCE_TRACE_FIELDS)))
        second = tuple(value + 100 for value in first)
        trace.append(first)
        trace.append(second)

        snapshot = trace.snapshot()

        self.assertEqual(
            "ctr-eval007-bounded-process-timing-trace-1", snapshot["schema"]
        )
        self.assertEqual(list(_PHYSICAL_SOURCE_TRACE_FIELDS), snapshot["fields"])
        self.assertEqual([list(first), list(second)], snapshot["rows"])
        self.assertEqual(2, snapshot["write_count"])
        self.assertEqual(0, snapshot["overwritten_count"])
        with self.assertRaises(TypeError):
            trace.append(tuple(False for _field in _PHYSICAL_SOURCE_TRACE_FIELDS))

    def test_eval007_trace_retention_requires_owned_0700_root_and_is_closed(self):
        context = __import__("multiprocessing").get_context("spawn")
        source = _BoundedProcessTimingTrace(context, _PHYSICAL_SOURCE_TRACE_FIELDS)
        publication = _BoundedProcessTimingTrace(context, _PUBLICATION_TRACE_FIELDS)
        source.append(tuple(range(len(_PHYSICAL_SOURCE_TRACE_FIELDS))))
        publication.append(tuple(range(len(_PUBLICATION_TRACE_FIELDS))))
        with __import__("tempfile").TemporaryDirectory() as tmp:
            root = Path(tmp)
            root.chmod(0o700)
            retained = _retain_eval007_timing_trace(
                root,
                source,
                {"tactile": publication},
            )
            document = json.loads(retained.read_text(encoding="utf-8"))
            self.assertEqual("ctr-eval007-simulator-timing-bundle-1", document["schema"])
            self.assertEqual(1, document["physical_source"]["write_count"])
            self.assertEqual(1, document["publication_workers"]["tactile"]["write_count"])
            self.assertEqual(0o600, retained.stat().st_mode & 0o777)
            root.chmod(0o755)
            with self.assertRaises(PermissionError):
                _retain_eval007_timing_trace(root, source, {})

    def test_development_process_publishers_share_one_source_handoff(self):
        source = inspect.getsource(CTRSimulatorNode.__init__)
        timer_source = inspect.getsource(CTRSimulatorNode._on_timer)
        self.assertIn("shared_mailbox = _LatestProcessPhysicalSampleMailbox", source)
        self.assertIn("self._state_mailbox = shared_mailbox", source)
        self.assertIn("self._tactile_mailbox = shared_mailbox", source)
        self.assertIn("self._auxiliary_mailbox = shared_mailbox", source)
        self.assertNotIn('(\"safety_state\", self._state_mailbox)', source)
        self.assertEqual(1, timer_source.count("_mailbox.put(sample)"))

    def test_tactile_worker_emits_coherent_compact_safety_state(self):
        source = inspect.getsource(simulator_node_module._publication_process_main)
        self.assertIn('"/ctr/safety/joint_state"', source)
        self.assertIn("CtrJointState", source)
        self.assertIn("source_evidence_qos_profile()", source)
        self.assertIn('publishers["safety_state"].publish(', source)
        self.assertLess(
            source.index('publishers["safety_state"].publish('),
            source.index('publishers["tactile"].publish(message)'),
        )

    def test_process_command_mailbox_returns_one_exact_latest_snapshot(self):
        context = __import__("multiprocessing").get_context("spawn")
        mailbox = _LatestProcessCommandMailbox(context)
        command = (0.1, 0.2, 0.3, -0.1, -0.2, -0.3)
        self.assertEqual(1, mailbox.put(command, valid=True, stamp_ns=1234))
        self.assertEqual((1, command, True, 1234), mailbox.snapshot())
        with self.assertRaises(TypeError):
            mailbox.put(command, valid=1, stamp_ns=1234)

    def test_physical_deadline_wait_is_absolute_and_stop_bounded(self):
        stop_event = threading.Event()
        started = __import__("time").monotonic()
        self.assertTrue(_wait_for_physical_deadline(started + 0.005, stop_event))
        elapsed = __import__("time").monotonic() - started
        self.assertGreaterEqual(elapsed, 0.004)
        stop_event.set()
        self.assertFalse(
            _wait_for_physical_deadline(__import__("time").monotonic() + 1.0, stop_event)
        )

    def test_late_physical_deadline_catches_up_once_without_backlog_or_extra_wait(self):
        self.assertEqual(10.01, _next_physical_deadline(10.0, 0.01, 10.005))
        self.assertEqual(10.2, _next_physical_deadline(10.0, 0.01, 10.2))
        with self.assertRaises(ValueError):
            _next_physical_deadline(10.0, 0.0, 10.0)

    def test_publication_affinity_separates_state_and_tactile_physical_cores(self):
        with __import__("tempfile").TemporaryDirectory() as tmp:
            topology = Path(tmp)
            for cpu, core in ((2, 2), (6, 2), (3, 3), (7, 3)):
                root = topology / f"cpu{cpu}" / "topology"
                root.mkdir(parents=True)
                (root / "physical_package_id").write_text("0", encoding="utf-8")
                (root / "core_id").write_text(str(core), encoding="utf-8")
            observed = []
            current = {2, 3, 6, 7}

            def set_affinity(_pid, cpus):
                current.clear()
                current.update(cpus)
                observed.append(tuple(sorted(cpus)))

            environment = {"CTR_DEVELOPMENT_EVALUATION_CPU_PARTITION": "1"}
            with patch.object(os, "sched_getaffinity", side_effect=lambda _pid: set(current)), patch.object(
                os, "sched_setaffinity", side_effect=set_affinity
            ):
                self.assertEqual(
                    (3,),
                    _apply_development_publication_affinity(
                        "tactile", environment, topology
                    ),
                )
            self.assertEqual([(3,)], observed)

    def test_state_publication_block_cannot_delay_tactile_worker_or_source_handoff(self):
        node = object.__new__(CTRSimulatorNode)
        node._source_stop_event = threading.Event()
        node._publication_failures = {}
        node._state_mailbox = _LatestPhysicalSampleMailbox()
        node._tactile_mailbox = _LatestPhysicalSampleMailbox()
        node._auxiliary_mailbox = _LatestPhysicalSampleMailbox()
        node.get_logger = lambda: SimpleNamespace(error=lambda _message: None)
        state_entered = threading.Event()
        state_release = threading.Event()
        tactile_sequences = []

        def blocked_state(_sample, _version, _previous):
            state_entered.set()
            state_release.wait(timeout=1.0)

        state_worker = threading.Thread(
            target=node._run_publication_worker,
            args=("state", node._state_mailbox, blocked_state),
        )
        tactile_worker = threading.Thread(
            target=node._run_publication_worker,
            args=(
                "tactile",
                node._tactile_mailbox,
                lambda sample, _version, _previous: tactile_sequences.append(sample.sequence),
            ),
        )
        state_worker.start()
        tactile_worker.start()
        node._state_mailbox.put(physical_sample(1))
        node._tactile_mailbox.put(physical_sample(1))
        self.assertTrue(state_entered.wait(timeout=1.0))
        node._state_mailbox.put(physical_sample(2))
        node._tactile_mailbox.put(physical_sample(2))
        for _ in range(100):
            if tactile_sequences and tactile_sequences[-1] == 2:
                break
            threading.Event().wait(0.005)
        self.assertEqual(2, tactile_sequences[-1])
        state_release.set()
        node._source_stop_event.set()
        node._close_publication_mailboxes()
        state_worker.join(timeout=1.0)
        tactile_worker.join(timeout=1.0)
        self.assertFalse(state_worker.is_alive())
        self.assertFalse(tactile_worker.is_alive())
        self.assertEqual({}, node._publication_failures)

    def test_console_keeps_visualization_off_the_physical_state_callback(self):
        source = (PACKAGE_ROOT / "ctr_sim" / "nodes" / "simulator_node.py").read_text(encoding="utf-8")
        self.assertIn("MutuallyExclusiveCallbackGroup", source)
        self.assertIn("SingleThreadedExecutor()", source)
        self.assertIn("target=_physical_source_process_main", source)
        self.assertIn("target=_publication_process_main", source)
        self.assertIn('multiprocessing.get_context("spawn")', source)
        self.assertIn("self._publish_tactile_for_physical_state(", source)
        self.assertIn("callback_group=self._visualization_callback_group", source)
        self.assertIn("self.tactile_timer = None", source)
        self.assertIn("stamp = self.get_clock().now().to_msg()", source)
        timer_body = source[source.index("    def _on_timer"):source.index("    def _publish_sample_synchronously")]
        self.assertNotIn(".publish(", timer_body)

    def test_source_evidence_qos_is_reliable_latest_sample(self):
        qos = source_evidence_qos_profile()
        self.assertEqual(1, qos.depth)
        self.assertEqual(ReliabilityPolicy.RELIABLE, qos.reliability)
        self.assertEqual(DurabilityPolicy.VOLATILE, qos.durability)
        development_state = source_evidence_qos_profile(reliable=False)
        self.assertEqual(ReliabilityPolicy.BEST_EFFORT, development_state.reliability)

    def test_development_source_affinity_is_exact_and_reconciled(self):
        observed = []
        current = {1, 5}

        def set_affinity(pid, cpus):
            current.clear()
            current.update(cpus)
            observed.append((pid, set(cpus)))

        with patch.object(simulator_node_module.os, "sched_setaffinity", side_effect=set_affinity), patch.object(
            simulator_node_module.os, "sched_getaffinity", side_effect=lambda _pid: set(current)
        ):
            result = _apply_development_source_affinity(
                {
                    "CTR_DEVELOPMENT_EVALUATION_CPU_PARTITION": "1",
                    "CTR_DEVELOPMENT_SIMULATOR_CPU_LIST": "1,5",
                }
            )
        self.assertEqual((1,), result)
        self.assertEqual([(0, {1})], observed)

    def test_bounded_source_loop_integrates_on_its_own_thread_until_stopped(self):
        node = object.__new__(CTRSimulatorNode)
        node._source_stop_event = threading.Event()
        node._source_failure = None
        node._source_affinity = ()
        node._publication_processes = []
        node._physics_expected_mono = 0.0
        callbacks = []

        def source_callback():
            callbacks.append(threading.current_thread().name)
            node._source_stop_event.set()

        node._on_timer = source_callback
        node.get_logger = lambda: SimpleNamespace(error=lambda _message: None)
        with patch.object(simulator_node_module, "_apply_development_source_affinity", return_value=(1, 5)):
            worker = threading.Thread(
                target=node._run_bounded_source,
                name="test-bounded-source",
            )
            worker.start()
            worker.join(timeout=1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(["test-bounded-source"], callbacks)
        self.assertEqual((1, 5), node._source_affinity)
        self.assertIsNone(node._source_failure)

    def test_visualization_work_is_skipped_without_a_marker_consumer(self):
        node = simulator_shell(curved_config("circular_arc"))
        node.marker_pub = SimpleNamespace(get_subscription_count=lambda: 0)
        node.development_marker_pubs = {
            "ctr_backbone": SimpleNamespace(get_subscription_count=lambda: 0)
        }
        self.assertFalse(node._visualization_consumers_present())
        node.development_marker_pubs["ctr_backbone"] = SimpleNamespace(
            get_subscription_count=lambda: 1
        )
        self.assertTrue(node._visualization_consumers_present())

    def test_auxiliary_publishers_skip_serialization_without_consumers(self):
        publisher = SimpleNamespace(get_subscription_count=lambda: 0)
        self.assertFalse(simulator_node_module.CTRSimulatorNode._publisher_has_consumers(publisher))
        publisher.get_subscription_count = lambda: 1
        self.assertTrue(simulator_node_module.CTRSimulatorNode._publisher_has_consumers(publisher))

        source = (PACKAGE_ROOT / "ctr_sim" / "nodes" / "simulator_node.py").read_text(encoding="utf-8")
        state_index = source.index("self.state_pub.publish")
        tip_index = source.index("self.tip_pub.publish", state_index)
        self.assertLess(state_index, tip_index)
        self.assertIn("self._diagnostics_publish_period_s = 0.10", source)

    def test_high_rate_state_message_storage_is_reused_without_geometry_changes(self):
        node = simulator_shell(curved_config("circular_arc"))
        node._backbone_point_cache = []
        node._tip_pose_cache = None
        node._state_message_cache = None
        first = node._backbone_points_for_publish(np.asarray([[1.0, 2.0, 3.0]]))
        second = node._backbone_points_for_publish(np.asarray([[4.0, 5.0, 6.0]]))
        self.assertIs(first, second)
        self.assertEqual((4.0, 5.0, 6.0), (second[0].x, second[0].y, second[0].z))

    def test_configured_marker_rate_is_independent_of_diagnostics_mode(self):
        node = simulator_shell(curved_config("circular_arc"))
        node.evaluation_diagnostics_enabled = True
        self.assertTrue(node._runtime_marker_publication_due(_time_msg(1.0)))
        self.assertFalse(node._runtime_marker_publication_due(_time_msg(1.1)))
        self.assertTrue(node._runtime_marker_publication_due(_time_msg(1.2)))

        node._last_runtime_marker_publish_time_s = None
        node.evaluation_diagnostics_enabled = False
        self.assertTrue(node._runtime_marker_publication_due(_time_msg(1.0)))
        self.assertFalse(node._runtime_marker_publication_due(_time_msg(1.1)))
        self.assertTrue(node._runtime_marker_publication_due(_time_msg(1.2)))

    def test_simulator_source_uses_shared_lumen_factory(self):
        source = (PACKAGE_ROOT / "ctr_sim" / "nodes" / "simulator_node.py").read_text(encoding="utf-8")
        self.assertIn("config_with_lumen_overrides", source)
        self.assertIn("lumen_geometry_from_config", source)
        self.assertIn("lumen_mode_from_config", source)
        self.assertIn("parse_launch_bool", source)
        self.assertNotIn("config_with_cylinder_overrides", source)

    def test_factory_modes_construct_expected_geometry_for_simulator(self):
        cases = (
            (no_lumen_config(), "none", type(None)),
            (cylinder_config(), "cylindrical", CylindricalLumen),
            (curved_config("circular_arc"), "curved", CurvedLumen),
            (curved_config("s_curve"), "curved", CurvedLumen),
        )
        for config, expected_mode, expected_type in cases:
            with self.subTest(expected_mode=expected_mode, expected_type=expected_type):
                self.assertEqual(expected_mode, lumen_mode_from_config(config))
                self.assertIsInstance(lumen_geometry_from_config(config), expected_type)

    def test_no_lumen_marker_array_keeps_geometry_independent_markers(self):
        node = simulator_shell(no_lumen_config())
        backbone = sample_backbone()
        msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        self.assertEqual(["ctr_backbone", "tip_marker", "target_marker"], [marker.ns for marker in msg.markers])

    def test_curved_marker_array_publishes_static_and_dynamic_diagnostic_markers(self):
        node = simulator_shell(curved_config("circular_arc"))
        backbone = sample_backbone()
        msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        namespaces = [marker.ns for marker in msg.markers]
        self.assertEqual(["ctr_backbone", "tip_marker", "target_marker"], namespaces[:3])
        self.assertEqual(
            [key[0] for key in CURVED_STATIC_LUMEN_MARKER_KEYS],
            namespaces[3 : 3 + len(CURVED_STATIC_LUMEN_MARKER_KEYS)],
        )
        self.assertEqual(DYNAMIC_LUMEN_MARKER_KEYS, tuple((marker.ns, marker.id) for marker in msg.markers[-4:]))
        self.assertNotIn("cylindrical_lumen", namespaces)
        self.assertIn("lumen_closest_pair", namespaces)
        self.assertIn("lumen_status", namespaces)

    def test_cylindrical_marker_array_retains_existing_lumen_markers(self):
        node = simulator_shell(cylinder_config())
        backbone = sample_backbone()
        msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        namespaces = [marker.ns for marker in msg.markers]
        self.assertEqual(["ctr_backbone", "tip_marker", "target_marker"], namespaces[:3])
        self.assertGreater(namespaces.count("cylindrical_lumen"), 0)
        cylinder_markers = [marker for marker in msg.markers if marker.ns == "cylindrical_lumen"]
        self.assertEqual([10, 11, 12, 13], [marker.id for marker in cylinder_markers])
        self.assertEqual(
            [Marker.CYLINDER, Marker.CYLINDER, Marker.LINE_STRIP, Marker.SPHERE],
            [marker.type for marker in cylinder_markers],
        )
        for namespace, _marker_id in CURVED_STATIC_LUMEN_MARKER_KEYS:
            self.assertNotIn(namespace, namespaces)
        for namespace, _marker_id in DYNAMIC_LUMEN_MARKER_KEYS:
            self.assertNotIn(namespace, namespaces)

    def test_cylindrical_marker_13_color_semantics_are_preserved(self):
        node = simulator_shell(cylinder_config())
        cases = (
            (np.array([[0.010, 0.0, 0.050]], dtype=float), (0.0, 0.8, 0.2)),
            (np.array([[0.0278, 0.0, 0.050]], dtype=float), (1.0, 0.6, 0.0)),
            (np.array([[0.0320, 0.0, 0.050]], dtype=float), (1.0, 0.0, 0.0)),
        )
        for backbone, expected_color in cases:
            with self.subTest(expected_color=expected_color):
                msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
                closest = [marker for marker in msg.markers if marker.ns == "cylindrical_lumen" and marker.id == 13][0]
                self.assertEqual(Marker.SPHERE, closest.type)
                self.assertEqual(expected_color, (closest.color.r, closest.color.g, closest.color.b))
                np.testing.assert_allclose(
                    [closest.pose.position.x, closest.pose.position.y, closest.pose.position.z],
                    backbone[0],
                )

    def test_curved_static_markers_are_cached_and_republished_at_bounded_rate(self):
        node = simulator_shell(curved_config("circular_arc"))
        backbone = sample_backbone()

        first = node._marker_array_msg(_time_msg(1.0), [_point_from_array(point) for point in backbone], backbone)
        first_namespaces = [marker.ns for marker in first.markers]
        self.assertIn("lumen_centerline", first_namespaces)
        self.assertEqual(1, node._static_lumen_build_count)

        second = node._marker_array_msg(_time_msg(1.05), [_point_from_array(point) for point in backbone], backbone)
        second_namespaces = [marker.ns for marker in second.markers]
        self.assertNotIn("lumen_centerline", second_namespaces)
        self.assertEqual(1, node._static_lumen_build_count)

        node.target_position = np.array([0.001, 0.002, 0.003], dtype=float)
        third = node._marker_array_msg(_time_msg(1.25), [_point_from_array(point) for point in backbone], backbone)
        third_namespaces = [marker.ns for marker in third.markers]
        self.assertIn("lumen_centerline", third_namespaces)
        self.assertEqual(1, node._static_lumen_build_count)
        self.assertEqual(3, node._lumen_diagnostic_update_count)

    def test_curved_static_marker_points_are_finite(self):
        node = simulator_shell(curved_config("s_curve"))
        backbone = sample_backbone()
        msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        for marker in msg.markers:
            for point in marker.points:
                self.assertTrue(np.isfinite([point.x, point.y, point.z]).all(), marker.ns)
            self.assertTrue(np.isfinite([marker.scale.x, marker.scale.y, marker.scale.z]).all(), marker.ns)

    def test_visualization_disabled_suppresses_curved_static_markers(self):
        config = curved_config("circular_arc")
        config["simulation"]["visualization"]["publish_lumen_markers"] = False
        node = simulator_shell(config)
        backbone = sample_backbone()
        msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        self.assertEqual(["ctr_backbone", "tip_marker", "target_marker"], [marker.ns for marker in msg.markers])

    def test_development_marker_array_uses_real_reference_and_bounded_tip_history(self):
        node = simulator_shell(curved_config("circular_arc"))
        node.development_simulation = True
        node.development_visualization = True
        node.lumen_marker_config = node.lumen_marker_config.__class__(
            **{
                **node.lumen_marker_config.__dict__,
                "publish_lumen_surface": True,
                "actual_tip_history_max_points": 3,
            }
        )
        node._reference_path_points = np.array([[0.01, 0.0, 0.08]], dtype=float)
        node._tip_trajectory = BoundedTipTrajectory(max_points=3, minimum_interval=0.05)
        node._tip_trajectory.append([0.0, 0.0, 0.04], 1.0)
        node._tip_trajectory.append([0.001, 0.0, 0.05], 1.1)
        backbone = sample_backbone()
        msg = node._marker_array_msg(
            _time_msg(1.1),
            [_point_from_array(point) for point in backbone],
            backbone,
        )
        keys = {(marker.ns, marker.id) for marker in msg.markers}
        self.assertIn(("lumen_surface", 0), keys)
        self.assertIn(("reference_path", 1), keys)
        self.assertIn(("actual_tip_path", 0), keys)

    def test_development_static_marker_qos_is_latched_and_dynamic_marker_qos_is_not(self):
        static_qos = development_marker_qos_profile("lumen_surface")
        dynamic_qos = development_marker_qos_profile("actual_tip_path")
        self.assertEqual(ReliabilityPolicy.RELIABLE, static_qos.reliability)
        self.assertEqual(DurabilityPolicy.TRANSIENT_LOCAL, static_qos.durability)
        self.assertEqual(ReliabilityPolicy.RELIABLE, dynamic_qos.reliability)
        self.assertEqual(DurabilityPolicy.VOLATILE, dynamic_qos.durability)

    def test_development_simulation_without_visual_opt_in_keeps_heavy_markers_off_critical_path(self):
        node = simulator_shell(curved_config("circular_arc"))
        node.development_simulation = True
        node.development_visualization = False
        node.lumen_marker_config = node.lumen_marker_config.__class__(
            **{
                **node.lumen_marker_config.__dict__,
                "publish_lumen_surface": False,
            }
        )
        node._reference_path_points = np.array([[0.01, 0.0, 0.08]], dtype=float)
        node._tip_trajectory = None
        backbone = sample_backbone()
        msg = node._marker_array_msg(
            _time_msg(1.1),
            [_point_from_array(point) for point in backbone],
            backbone,
        )
        namespaces = {marker.ns for marker in msg.markers}
        self.assertNotIn("lumen_surface", namespaces)
        self.assertNotIn("reference_path", namespaces)
        self.assertNotIn("actual_tip_path", namespaces)

    def test_reference_path_callback_retains_exact_nonempty_path_and_common_frame(self):
        node = simulator_shell(curved_config("circular_arc"))
        message = NavPath()
        message.header.frame_id = "base_link"
        for coordinates in ((0.0, 0.0, 0.04), (0.01, 0.0, 0.08)):
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose.position = _point_from_array(np.asarray(coordinates, dtype=float))
            message.poses.append(pose)
        node._on_reference_path(message)
        self.assertEqual("base_link", node._reference_path_frame_id)
        np.testing.assert_allclose(
            node._reference_path_points,
            [[0.0, 0.0, 0.04], [0.01, 0.0, 0.08]],
        )

    def test_accepted_reference_tip_moves_active_target_marker_to_controller_target(self):
        node = simulator_shell(curved_config("circular_arc"))
        message = PoseStamped()
        message.header.frame_id = "base_link"
        message.pose.position = _point_from_array(np.array([0.015, 0.005, 0.100]))
        node._on_target(message)
        backbone = sample_backbone()
        marker_array = node._marker_array_msg(
            Time(), [_point_from_array(point) for point in backbone], backbone
        )
        target = next(marker for marker in marker_array.markers if marker.ns == "target_marker")
        center = np.mean(
            [[point.x, point.y, point.z] for point in target.points[:-1]], axis=0
        )
        np.testing.assert_allclose(center, [0.015, 0.005, 0.100], atol=1.0e-12)

    def test_lumen_diagnostics_disabled_keeps_static_markers_and_dynamic_markers_absent(self):
        config = curved_config("circular_arc")
        config["simulation"]["visualization"]["publish_lumen_diagnostics"] = False
        node = simulator_shell(config)
        backbone = sample_backbone()
        msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        keys = tuple((marker.ns, marker.id) for marker in msg.markers)
        for key in CURVED_STATIC_LUMEN_MARKER_KEYS:
            self.assertIn(key, keys)
        for key in DYNAMIC_LUMEN_MARKER_KEYS:
            self.assertNotIn(key, keys)
        self.assertEqual(0, node._lumen_diagnostic_update_count)

    def test_no_lumen_clears_stale_curved_static_markers_with_targeted_deletes(self):
        node = simulator_shell(curved_config("circular_arc"))
        backbone = sample_backbone()
        node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)

        node.lumen_mode = "none"
        node.lumen = None
        msg = node._marker_array_msg(_time_msg(1.0), [_point_from_array(point) for point in backbone], backbone)
        deletes = [marker for marker in msg.markers if marker.action == Marker.DELETE]
        self.assertEqual(
            CURVED_STATIC_LUMEN_MARKER_KEYS + DYNAMIC_LUMEN_MARKER_KEYS,
            tuple((marker.ns, marker.id) for marker in deletes),
        )
        self.assertTrue(all(marker.action != Marker.DELETEALL for marker in deletes))

    def test_cylindrical_mode_clears_prior_curved_static_markers_without_delete_all(self):
        node = simulator_shell(curved_config("circular_arc"))
        backbone = sample_backbone()
        node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)

        cylinder = lumen_geometry_from_config(cylinder_config())
        node.config = cylinder_config()
        node.lumen_mode = "cylindrical"
        node.lumen = cylinder
        node.lumen_geometry = cylinder
        msg = node._marker_array_msg(_time_msg(1.0), [_point_from_array(point) for point in backbone], backbone)
        deletes = [marker for marker in msg.markers if marker.action == Marker.DELETE]
        self.assertEqual(
            CURVED_STATIC_LUMEN_MARKER_KEYS + DYNAMIC_LUMEN_MARKER_KEYS,
            tuple((marker.ns, marker.id) for marker in deletes),
        )
        self.assertTrue(any(marker.ns == "cylindrical_lumen" and marker.id == 10 for marker in msg.markers))

    def test_dynamic_diagnostic_reuses_full_fk_backbone_without_static_cache_rebuild(self):
        node = simulator_shell(curved_config("circular_arc"))
        backbone = sample_backbone()
        calls = []
        original = simulator_node_module.build_lumen_runtime_diagnostic

        def spy(lumen, points, mode):
            calls.append((lumen, np.asarray(points).copy(), mode))
            return original(lumen, points, mode)

        simulator_node_module.build_lumen_runtime_diagnostic = spy
        try:
            first = node._marker_array_msg(_time_msg(1.0), [_point_from_array(point) for point in backbone], backbone)
            second = node._marker_array_msg(_time_msg(1.01), [_point_from_array(point) for point in backbone], backbone)
        finally:
            simulator_node_module.build_lumen_runtime_diagnostic = original

        self.assertEqual(2, len(calls))
        np.testing.assert_allclose(calls[0][1], backbone)
        self.assertEqual("curved", calls[0][2])
        self.assertEqual(1, node._static_lumen_build_count)
        self.assertIn("lumen_centerline", [marker.ns for marker in first.markers])
        self.assertNotIn("lumen_centerline", [marker.ns for marker in second.markers])
        self.assertIn("lumen_status", [marker.ns for marker in second.markers])

    def test_dynamic_status_transitions_reuse_stable_marker_ids(self):
        node = simulator_shell(curved_config("circular_arc"))
        lumen = node.lumen
        assert isinstance(lumen, CurvedLumen)
        safe = np.asarray([lumen.centerline_points[5] + np.array([0.006, 0.0, 0.0])], dtype=float)
        margin = np.asarray([lumen.centerline_points[5] + np.array([0.0278, 0.0, 0.0])], dtype=float)
        collision = np.asarray([lumen.centerline_points[5] + np.array([0.032, 0.0, 0.0])], dtype=float)
        messages = [
            node._marker_array_msg(_time_msg(1.0), [_point_from_array(point) for point in safe], safe),
            node._marker_array_msg(_time_msg(1.1), [_point_from_array(point) for point in margin], margin),
            node._marker_array_msg(_time_msg(1.2), [_point_from_array(point) for point in collision], collision),
            node._marker_array_msg(_time_msg(1.3), [_point_from_array(point) for point in safe], safe),
        ]
        for msg in messages:
            self.assertEqual(DYNAMIC_LUMEN_MARKER_KEYS, tuple((marker.ns, marker.id) for marker in msg.markers[-4:]))
        status_text = [[marker.text for marker in msg.markers if marker.ns == "lumen_status"][0] for msg in messages]
        self.assertIn(STATUS_MARGIN, status_text[1])
        self.assertIn(STATUS_COLLISION, status_text[2])
        self.assertNotIn(STATUS_COLLISION, status_text[3])

    def test_dynamic_generation_failure_deletes_stale_keys_and_keeps_static_markers(self):
        node = simulator_shell(curved_config("circular_arc"))
        backbone = sample_backbone()
        node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        original = simulator_node_module.build_lumen_runtime_diagnostic

        def failing(*_args, **_kwargs):
            raise ValueError("diagnostic failure")

        simulator_node_module.build_lumen_runtime_diagnostic = failing
        try:
            msg = node._marker_array_msg(_time_msg(1.0), [_point_from_array(point) for point in backbone], backbone)
        finally:
            simulator_node_module.build_lumen_runtime_diagnostic = original

        deletes = tuple((marker.ns, marker.id) for marker in msg.markers if marker.action == Marker.DELETE)
        self.assertEqual(DYNAMIC_LUMEN_MARKER_KEYS, deletes)
        self.assertIn("lumen_centerline", [marker.ns for marker in msg.markers])


def _time_msg(seconds: float) -> Time:
    stamp = Time()
    stamp.sec = int(seconds)
    stamp.nanosec = int(round((seconds - stamp.sec) * 1.0e9))
    return stamp


if __name__ == "__main__":
    unittest.main()
