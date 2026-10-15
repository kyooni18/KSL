"""Keep the offline suite independent of a KSP instance running on this host."""
import pytest


@pytest.fixture(autouse=True)
def _no_live_krpc(monkeypatch):
    def offline(*_args, **_kwargs):
        raise ConnectionRefusedError("offline test: live kRPC disabled")
    monkeypatch.setattr("ksl.krpc_link.connect", offline)
