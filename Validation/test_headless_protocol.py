#!/usr/bin/env python3
"""Exercise backend pipe ordering without connecting to KSP."""
from __future__ import annotations

from contextlib import contextmanager, redirect_stdout
import io
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Tools"))
from headless_flight import BackendProcess


@contextmanager
def backend_fixture(snapshot=None, *, response_first=False, omit_response=False):
    with tempfile.TemporaryDirectory(prefix="sts-backend-protocol-") as directory:
        executable = Path(directory) / "fake_backend.py"
        executable.write_text(
            "#!/usr/bin/env python3\n"
            "import json, sys\n"
            f"snapshot = {snapshot!r}\n"
            "for line in sys.stdin:\n"
            "    request = json.loads(line)\n"
            "    messages = []\n"
            "    if snapshot is not None:\n"
            "        messages.append({'type': 'snapshot', 'snapshot': snapshot})\n"
            f"    if not {omit_response!r}:\n"
            "        messages.append({'type': 'response', 'id': request['id'], 'ok': True, 'result': {}})\n"
            f"    if {response_first!r}:\n"
            "        messages.reverse()\n"
            "    for message in messages:\n"
            "        print(json.dumps(message), flush=True)\n",
            encoding="utf-8",
        )
        executable.chmod(0o700)
        backend = BackendProcess(executable)
        try:
            with redirect_stdout(io.StringIO()):
                yield backend
        finally:
            if backend.process.poll() is None:
                backend.process.terminate()
            backend.process.wait(timeout=3.0)
            backend.close()


class BackendProtocolTests(unittest.TestCase):
    def test_shadow_rejection_before_reply_does_not_become_timeout(self):
        reason = "Reentry rejected by closed-loop shadow: entered=true, MM305=false"
        snapshot = {"phase": "Idle", "automationEngaged": False, "warningMessage": reason}
        with backend_fixture(snapshot) as backend:
            with self.assertRaisesRegex(RuntimeError, "Reentry rejected by closed-loop shadow"):
                backend.engage("engageReentry", timeout=2.0, acknowledgement_timeout=0.05)
            self.assertIsNone(backend.process.poll())

    def test_rejection_after_reply_is_also_consumed(self):
        snapshot = {"phase": "Idle", "automationEngaged": False,
                    "warningMessage": "HAC has no feasible route"}
        with backend_fixture(snapshot, response_first=True) as backend:
            with self.assertRaisesRegex(RuntimeError, "HAC has no feasible route"):
                backend.engage("engageHACTest", timeout=2.0, acknowledgement_timeout=2.0)

    def test_coast_does_not_need_an_extra_snapshot(self):
        snapshot = {"phase": "Coast", "automationEngaged": True}
        with backend_fixture(snapshot) as backend:
            self.assertEqual(backend.engage("engage", timeout=2.0,
                                            acknowledgement_timeout=0.05), snapshot)
            self.assertIsNone(backend.process.poll())

    def test_ok_response_alone_is_not_engagement(self):
        with backend_fixture() as backend:
            with self.assertRaisesRegex(TimeoutError, "engage did not confirm engagement"):
                backend.engage("engage", timeout=2.0, acknowledgement_timeout=0.05)

    def test_method_timeout_preserves_request_context(self):
        snapshot = {"phase": "Idle", "automationEngaged": False}
        with backend_fixture(snapshot, omit_response=True) as backend:
            with self.assertRaisesRegex(TimeoutError, "Backend method engageReentry timed out"):
                backend.send("engageReentry", timeout=0.25)


if __name__ == "__main__":
    unittest.main()
