import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

import ctr_bringup.development_physical_evidence as evidence


def record(session_id, sequence, *, stamp_ns=None):
    return evidence.PhysicalEvidenceRecord(
        session_id=session_id,
        producer_pid=os.getpid(),
        producer_uid=os.getuid(),
        generated_sequence=sequence,
        source_monotonic_ns=time.monotonic_ns(),
        source_stamp_ns=time.time_ns() if stamp_ns is None else stamp_ns,
        command_sequence=17,
        q=(0.01, 0.02, 0.03, 0.1, 0.2, 0.3),
        q_dot=(0.001, 0.002, 0.003, 0.01, 0.02, 0.03),
        tip_position=(0.004, 0.005, 0.090),
        whole_backbone_physical_clearance_m=0.012,
        whole_backbone_safety_clearance_m=0.009,
        raw_tactile=0.1,
        filtered_tactile=0.08,
        tactile_force_n=0.4,
        tactile_clearance_m=0.012,
        tactile_region=0,
        source_valid=True,
        simulation=True,
        frame_valid=True,
        physical_collision=False,
        safety_margin_violation=False,
        tactile_valid=True,
        contact=False,
        warning=False,
        stop=False,
    )


class PhysicalEvidenceChannelTest(unittest.TestCase):
    def setUp(self):
        self._temporary = tempfile.TemporaryDirectory(prefix="physical-evidence-")
        self.root = Path(self._temporary.name)
        os.chmod(self.root, 0o700)
        self.session_id = "ab" * 32
        self.producer = None
        self.reader = None

    def tearDown(self):
        if self.reader is not None:
            self.reader.close()
        if self.producer is not None:
            self.producer.close()
        self._temporary.cleanup()

    def connect(self):
        self.producer = evidence.PhysicalEvidenceProducer(
            self.root,
            self.session_id,
            expected_reader_token="python",
        )
        self.reader = evidence.PhysicalEvidenceReader(
            self.root,
            self.session_id,
            expected_producer_token="python",
            connect_timeout_s=1.0,
        )

    def test_authenticated_round_trip_preserves_exact_physical_values(self):
        self.connect()
        expected = record(self.session_id, 1)
        self.producer.write(expected)
        self.assertEqual(expected, self.reader.read())

    def test_environment_connect_timeout_never_exceeds_committed_limit(self):
        self.assertEqual(10.0, evidence.connect_timeout_from_environment({}))
        self.assertEqual(
            2.5,
            evidence.connect_timeout_from_environment(
                {evidence.CONNECT_TIMEOUT_ENV: "2.5"}
            ),
        )
        for value in ("0", "-1", "nan", "inf", "10.0000001", "30", "invalid"):
            with self.subTest(value=value), self.assertRaisesRegex(
                evidence.PhysicalEvidenceError,
                "physical_evidence_connect_timeout_invalid",
            ):
                evidence.connect_timeout_from_environment(
                    {evidence.CONNECT_TIMEOUT_ENV: value}
                )

        with self.assertRaisesRegex(
            evidence.PhysicalEvidenceError,
            "physical_evidence_connect_timeout_invalid",
        ):
            evidence.PhysicalEvidenceReader(
                self.root,
                self.session_id,
                expected_producer_token="python",
                connect_timeout_s=10.0000001,
            )

    def test_latest_slot_permits_explained_sequence_gap_without_duplication(self):
        self.connect()
        first = record(self.session_id, 1)
        latest = record(self.session_id, 4)
        self.producer.write(first)
        self.producer.write(record(self.session_id, 2))
        self.producer.write(record(self.session_id, 3))
        self.producer.write(latest)
        self.assertEqual(4, self.reader.read().generated_sequence)
        self.assertEqual(4, self.reader.read().generated_sequence)

    def test_sequence_and_timestamp_rollback_fail_closed(self):
        self.connect()
        stamp_ns = time.time_ns()
        self.producer.write(record(self.session_id, 2, stamp_ns=stamp_ns))
        self.reader.read()
        self.producer.write(record(self.session_id, 1, stamp_ns=stamp_ns + 1))
        with self.assertRaisesRegex(
            evidence.PhysicalEvidenceError, "sequence_rollback"
        ):
            self.reader.read()

        self.reader._last_sequence = 2
        self.reader._last_source_stamp_ns = stamp_ns
        self.producer.write(record(self.session_id, 3, stamp_ns=stamp_ns - 1))
        with self.assertRaisesRegex(
            evidence.PhysicalEvidenceError, "timestamp_rollback"
        ):
            self.reader.read()

    def test_torn_or_corrupt_record_fails_integrity_check(self):
        self.connect()
        self.producer.write(record(self.session_id, 1))
        offset = evidence._RECORD_OFFSET + 13
        self.producer._mapping[offset] ^= 0x01
        with self.assertRaisesRegex(
            evidence.PhysicalEvidenceError, "integrity_invalid"
        ):
            self.reader.read()

    def test_reader_waits_for_a_bounded_in_progress_commit(self):
        self.connect()
        expected = record(self.session_id, 1)
        self.producer.write(expected)
        stable_generation = evidence._SEQUENCE.unpack_from(
            self.producer._mapping, 0
        )[0]
        evidence._SEQUENCE.pack_into(
            self.producer._mapping, 0, stable_generation + 1
        )

        def finish_commit():
            time.sleep(0.010)
            evidence._SEQUENCE.pack_into(
                self.producer._mapping, 0, stable_generation
            )

        thread = threading.Thread(target=finish_commit)
        thread.start()
        try:
            self.assertEqual(expected, self.reader.read())
        finally:
            thread.join(timeout=1.0)

    def test_reader_fails_closed_when_commit_never_stabilizes(self):
        self.connect()
        self.producer.write(record(self.session_id, 1))
        stable_generation = evidence._SEQUENCE.unpack_from(
            self.producer._mapping, 0
        )[0]
        evidence._SEQUENCE.pack_into(
            self.producer._mapping, 0, stable_generation + 1
        )
        started = time.monotonic()
        with self.assertRaisesRegex(
            evidence.PhysicalEvidenceError, "physical_evidence_torn_read"
        ):
            self.reader.read()
        self.assertLess(time.monotonic() - started, 0.10)

    def test_initial_zero_generation_is_immediately_unavailable(self):
        self.connect()
        with mock.patch.object(evidence.time, "sleep") as sleep, self.assertRaisesRegex(
            evidence.PhysicalEvidenceError,
            "physical_evidence_unavailable",
        ):
            self.reader.read()
        sleep.assert_not_called()

    def test_post_start_zero_generation_retries_then_fails_closed(self):
        self.connect()
        expected = record(self.session_id, 1)
        self.producer.write(expected)
        self.assertEqual(expected, self.reader.read())
        stable_generation = evidence._SEQUENCE.unpack_from(
            self.producer._mapping, 0
        )[0]
        evidence._SEQUENCE.pack_into(self.producer._mapping, 0, 0)

        def restore_stable_generation():
            time.sleep(0.010)
            evidence._SEQUENCE.pack_into(
                self.producer._mapping, 0, stable_generation
            )

        thread = threading.Thread(target=restore_stable_generation)
        thread.start()
        try:
            self.assertEqual(expected, self.reader.read())
        finally:
            thread.join(timeout=1.0)

        evidence._SEQUENCE.pack_into(self.producer._mapping, 0, 0)
        started = time.monotonic()
        with self.assertRaisesRegex(
            evidence.PhysicalEvidenceError, "physical_evidence_torn_read"
        ):
            self.reader.read()
        self.assertLess(time.monotonic() - started, 0.10)

    def test_wrong_session_receives_no_descriptor(self):
        self.producer = evidence.PhysicalEvidenceProducer(
            self.root,
            self.session_id,
            expected_reader_token="python",
        )
        with self.assertRaises(evidence.PhysicalEvidenceError):
            evidence.PhysicalEvidenceReader(
                self.root,
                "cd" * 32,
                expected_producer_token="python",
                connect_timeout_s=1.0,
            )

    def test_producer_disconnect_is_not_fresh_evidence(self):
        self.connect()
        self.producer.write(record(self.session_id, 1))
        self.reader.read()
        self.producer.close()
        self.producer = None
        with self.assertRaisesRegex(
            evidence.PhysicalEvidenceError, "producer_disconnected"
        ):
            self.reader.read()

    def test_root_must_be_private_and_owned(self):
        os.chmod(self.root, 0o755)
        with self.assertRaisesRegex(
            evidence.PhysicalEvidenceError, "root_identity_invalid"
        ):
            evidence.PhysicalEvidenceProducer(
                self.root,
                self.session_id,
                expected_reader_token="python",
            )


if __name__ == "__main__":
    unittest.main()
