"""Keep the offline suite independent of a KSP instance running on this host."""
import pytest


@pytest.fixture(autouse=True)
def _no_live_krpc(monkeypatch, tmp_path):
    def offline(*_args, **_kwargs):
        raise ConnectionRefusedError("offline test: live kRPC disabled")
    monkeypatch.setattr("ksl.krpc_link.connect", offline)
    # Fake controllers still exercise flock, but must not discover or contend
    # with a real flight's endpoint ownership file on the developer's host.
    monkeypatch.setattr("ksl.ownership.DATA_ROOT", tmp_path)
