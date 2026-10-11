"""The public CLI must return when the kRPC handshake stalls."""

import sys
import threading
from types import SimpleNamespace
from unittest import mock

import pytest

from ksl.config import load_config
from ksl.krpc_link import ConnectionFailure, connect


def test_stalled_handshake_times_out_and_closes_late_connection():
    config = load_config(overrides=["connection.timeoutSeconds=0.05"])
    release = threading.Event()
    returned = threading.Event()
    closed = threading.Event()

    def slow_connect(**_kwargs):
        release.wait(2)
        returned.set()
        return SimpleNamespace(close=closed.set)

    with mock.patch.dict(sys.modules, {"krpc": SimpleNamespace(connect=slow_connect)}):
        with pytest.raises(ConnectionFailure, match="handshake.*timed out"):
            with connect(config, "timeout-test"):
                pass
        release.set()
        assert returned.wait(1)
        assert closed.wait(1)
