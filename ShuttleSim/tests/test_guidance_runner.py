#!/usr/bin/env python3
"""Runner lifecycle and stream regressions; no KSP connection is made."""
from __future__ import annotations

import io
import gzip
from unittest.mock import patch
import pathlib
import json
import math
import sys
import tempfile
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))
from backend_mailbox import BackendMailbox
from run_guidance import Backend


class MailboxTests(unittest.TestCase):
    def test_telemetry_storage_does_not_grow_with_flight_duration(self):
        mailbox = BackendMailbox()
        mailbox.expect("connect")
        for ut in range(10000):
            mailbox.publish({"type": "snapshot", "snapshot": {"ut": ut, "trajectory": list(range(32))}})
        self.assertEqual(mailbox._pending, {"connect"})
        self.assertEqual(mailbox._responses, {})
        latest = mailbox.wait(lambda m: m.get("type") == "snapshot", 0)
        self.assertEqual(latest["snapshot"]["ut"], 9999)

    def test_out_of_order_replies_survive_unrelated_waits(self):
        mailbox = BackendMailbox()
        for request_id in ("first", "second"):
            mailbox.expect(request_id)
            mailbox.publish({"type": "response", "id": request_id, "ok": True})
        self.assertEqual(mailbox.wait(lambda m: m.get("id") == "second", 0)["id"], "second")
        self.assertEqual(mailbox.wait(lambda m: m.get("id") == "first", 0)["id"], "first")
        self.assertFalse(mailbox._pending)
        self.assertFalse(mailbox._responses)

    def test_duplicate_and_unsolicited_replies_are_not_unbounded(self):
        mailbox = BackendMailbox()
        mailbox.expect("one")
        with self.assertRaises(ValueError):
            mailbox.expect("one")
        for index in range(1000):
            mailbox.publish({"type": "response", "id": str(index)})
        self.assertFalse(mailbox._responses)
        mailbox.cancel("one")
        self.assertFalse(mailbox._pending)

    def test_eof_and_reader_failures_are_observable(self):
        mailbox = BackendMailbox()
        mailbox.finish()
        with self.assertRaises(EOFError):
            mailbox.wait(lambda m: False, 10)
        mailbox = BackendMailbox()
        mailbox.finish(OSError("log is unavailable"))
        with self.assertRaises(RuntimeError):
            mailbox.check()

    def test_timeout_does_not_consume_a_different_reply(self):
        mailbox = BackendMailbox()
        mailbox.expect("other")
        mailbox.publish({"type": "response", "id": "other"})
        with self.assertRaises(TimeoutError):
            mailbox.wait(lambda m: m.get("id") == "missing", 0)
        self.assertEqual(mailbox.wait(lambda m: m.get("id") == "other", 0)["id"], "other")

    def test_backend_integration_preserves_latest_and_logs(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory)
            lines = ''.join('{"type":"snapshot","snapshot":{"phase":"MM304 Entry","telemetry":{"ut":%d}}}\n' % ut
                            for ut in range(1000))
            process = SimpleNamespace(stdout=io.StringIO(lines), stdin=io.StringIO())
            backend = Backend(process, path / "events.jsonl", mirror=False,
                              replay_path=path / "replay.jsonl")
            backend.thread.join(timeout=3)
            self.assertFalse(backend.thread.is_alive())
            self.assertEqual(backend.latest["telemetry"]["ut"], 999)
            self.assertFalse(backend.mailbox._responses)
            backend.close()
            self.assertEqual(len((path / "events.jsonl").read_text().splitlines()), 1000)
            with self.assertRaises(EOFError):
                backend.wait(lambda m: m.get("type") == "response", 10)


class HacGeometryTests(unittest.TestCase):
    @staticmethod
    def record(side=1.0, angle=120.0, radius=3200.0, distance=2800.0):
        sweep=side*math.radians(angle)
        return {"runwayEnd":0,"side":side,"radius":radius,"sweep":sweep,
                "leadLength":radius,"arcLength":radius*abs(sweep),
                "center":[-distance,side*radius],"exit":[-distance,0.0],
                "entry":[-distance-side*radius*math.sin(sweep),side*radius*(1-math.cos(sweep))],
                "finalDistance":distance}

    def evidence(self, *records):
        from run_guidance import fixed_hac_geometry_evidence
        text="\n".join("MM305_ROUTE "+json.dumps(record) for record in records)
        return fixed_hac_geometry_evidence(text,2800.0)

    def test_native_signed_arcs_are_geometric_not_log_wording(self):
        for side in (-1.0,1.0):
            for angle in (45.0,120.0,270.0):
                with self.subTest(side=side,angle=angle):
                    record=self.record(side,angle)
                    self.assertTrue(self.evidence(record,record)["valid"])

    def test_geometry_must_remain_runway_anchored_and_immutable(self):
        self.assertFalse(self.evidence(self.record(),self.record(radius=4000.0))["valid"])
        self.assertFalse(self.evidence(self.record(distance=3000.0))["valid"])
        changed=self.record();changed["center"][0]+=1.0
        self.assertFalse(self.evidence(changed)["valid"])

    def test_invalid_numbers_points_and_extra_circuits_fail_closed(self):
        for key,value in (("radius",math.nan),("radius",2999.0),("sweep",math.tau+0.1),
                          ("leadLength",-1.0),("entry",[]),("arcLength",123.0)):
            with self.subTest(key=key,value=value):
                changed=self.record();changed[key]=value
                self.assertFalse(self.evidence(changed)["valid"])


class ArtifactTests(unittest.TestCase):
    def test_same_label_allocates_distinct_owned_directories(self):
        from run_artifacts import allocate_run
        with tempfile.TemporaryDirectory() as directory:
            root=pathlib.Path(directory)
            first=allocate_run(root,"mm304","stamp")
            (first/"evidence").write_text("keep")
            second=allocate_run(root,"mm304","stamp")
            self.assertNotEqual(first,second)
            self.assertEqual((first/"evidence").read_text(),"keep")
            with self.assertRaises(ValueError):
                allocate_run(root,"../outside","stamp")

    def test_failed_metadata_serialization_preserves_previous_document(self):
        from run_artifacts import write_json
        with tempfile.TemporaryDirectory() as directory:
            path=pathlib.Path(directory)/"manifest.json"
            write_json(path,{"state":"running"})
            with self.assertRaises(ValueError):
                write_json(path,{"invalid":math.nan})
            self.assertEqual(json.loads(path.read_text()),{"state":"running"})
            self.assertEqual(list(path.parent.iterdir()),[path])

    def test_compression_round_trip_and_existing_destination(self):
        from run_artifacts import compress_recording
        with tempfile.TemporaryDirectory() as directory:
            path=pathlib.Path(directory)/"telemetry.jsonl"
            evidence=b'{"ut":1.25}\n'*1000
            path.write_bytes(evidence)
            compressed=compress_recording(path)
            self.assertFalse(path.exists())
            with gzip.open(compressed,"rb") as stream:
                self.assertEqual(stream.read(),evidence)
            path.write_bytes(b"new evidence")
            with self.assertRaises(FileExistsError):
                compress_recording(path)
            self.assertEqual(path.read_bytes(),b"new evidence")

    def test_failed_compression_keeps_complete_uncompressed_evidence(self):
        from run_artifacts import compress_recording
        with tempfile.TemporaryDirectory() as directory:
            path=pathlib.Path(directory)/"events.jsonl"
            path.write_bytes(b"recorded flight")
            with patch("run_artifacts.shutil.copyfileobj",side_effect=OSError("disk full")):
                with self.assertRaises(OSError):
                    compress_recording(path)
            self.assertEqual(path.read_bytes(),b"recorded flight")
            self.assertEqual(list(path.parent.iterdir()),[path])


if __name__ == "__main__":
    unittest.main()
