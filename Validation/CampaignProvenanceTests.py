#!/usr/bin/env python3
"""Deterministic regression tests for immutable live-campaign artifacts."""
from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Tools"))

from campaign_provenance import (
    RUNTIME_SOURCE_RELATIVE_PATHS,
    load_pinned_campaign_artifacts,
    pin_campaign_artifacts,
    sha256_file,
    verify_running_executable,
    write_run_manifest,
)


def fake_root(base: Path) -> tuple[Path, Path, Path]:
    root = base / "project"
    (root / "CLanding" / "build").mkdir(parents=True)
    (root / "Configuration").mkdir(parents=True)
    (root / "Tools").mkdir(parents=True)
    for relative in RUNTIME_SOURCE_RELATIVE_PATHS:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"fixture source: {relative}\n", encoding="utf-8")
    (root / "CLanding" / "main.c").write_text("int main(void){return 0;}\n", encoding="utf-8")
    (root / "CLanding" / "landing.h").write_text("#pragma once\n", encoding="utf-8")
    (root / "CLanding" / "Makefile").write_text("all:\n\t@true\n", encoding="utf-8")
    backend = root / "CLanding" / "build" / "landing_backend"
    config = root / "Configuration" / "live.json"
    config.write_text('{"generation":1}\n', encoding="utf-8")
    return root, backend, config


def build_native_fixture(path: Path) -> None:
    """Build an unsigned local executable so macOS system-binary policy is irrelevant."""
    source = path.with_suffix(".c")
    source.write_text(
        "#include <stdio.h>\n"
        "#include <stdlib.h>\n"
        "#include <unistd.h>\n"
        "int main(int argc, char **argv) {\n"
        "    if (argc < 2) return 0;\n"
        "    char *end = NULL;\n"
        "    long seconds = strtol(argv[1], &end, 10);\n"
        "    if (end && *end == '\\0') { sleep((unsigned int)(seconds > 0 ? seconds : 0)); return 0; }\n"
        "    puts(argv[1]);\n"
        "    return 0;\n"
        "}\n",
        encoding="utf-8",
    )
    subprocess.check_call(["cc", "-O2", "-std=c17", str(source), "-o", str(path)])
    source.unlink()


def test_mutable_source_replacement_cannot_change_pinned_restart() -> None:
    with tempfile.TemporaryDirectory(prefix="campaign-provenance-") as raw:
        root, backend, config = fake_root(Path(raw))
        build_native_fixture(backend)
        pinned = pin_campaign_artifacts(root, backend, config)
        manifest = json.loads(pinned.manifest.read_text(encoding="utf-8"))

        assert pinned.backend.parent == pinned.config.parent == pinned.manifest.parent
        assert manifest["identity"] == pinned.identity
        assert manifest["sourceTreeSha256"] == pinned.source_tree_sha256
        assert manifest["backend"]["sha256"] == pinned.backend_sha256
        assert manifest["backend"]["pinnedInode"] == pinned.backend_inode
        assert manifest["configuration"]["sha256"] == pinned.config_sha256
        assert sha256_file(pinned.backend) == pinned.backend_sha256
        assert sha256_file(pinned.config) == pinned.config_sha256
        assert pinned.backend.stat().st_mode & 0o222 == 0
        assert pinned.config.stat().st_mode & 0o222 == 0
        assert pinned.manifest.stat().st_mode & 0o222 == 0

        first = subprocess.check_output([str(pinned.backend), "original"], text=True).strip()
        assert first == "original"

        # Simulate a concurrent relink/config edit at the mutable source paths.
        backend.write_bytes(b"replacement executable bytes\n")
        os.chmod(backend, 0o755)
        config.write_text('{"generation":2}\n', encoding="utf-8")

        # Reopen the immutable identity after mutable replacement. A retry must
        # still execute/read the original candidate rather than the rebuilt paths.
        restarted = load_pinned_campaign_artifacts(root, pinned.identity)
        second = subprocess.check_output([str(restarted.backend), "original"], text=True).strip()
        third = subprocess.check_output([str(restarted.backend), "original"], text=True).strip()
        assert second == third == "original"
        assert json.loads(restarted.config.read_text(encoding="utf-8"))["generation"] == 1
        assert sha256_file(restarted.backend) == pinned.backend_sha256


def test_effective_runtime_configuration_is_the_pinned_identity() -> None:
    with tempfile.TemporaryDirectory(prefix="campaign-provenance-config-") as raw:
        root, backend, config = fake_root(Path(raw))
        build_native_fixture(backend)
        source_config_sha = sha256_file(config)
        effective = {"generation": 1, "guidance": {"useTimeWarp": False}}
        pinned = pin_campaign_artifacts(root, backend, config, effective_configuration=effective)
        manifest = json.loads(pinned.manifest.read_text(encoding="utf-8"))

        assert json.loads(pinned.config.read_text(encoding="utf-8")) == effective
        assert manifest["configuration"]["sourceSha256"] == source_config_sha
        assert manifest["configuration"]["sha256"] == pinned.config_sha256
        assert manifest["configuration"]["effective"] is True
        assert pinned.config_sha256 != source_config_sha


def test_runtime_source_mutation_changes_identity_and_blocks_exact_restart() -> None:
    with tempfile.TemporaryDirectory(prefix="campaign-provenance-runtime-") as raw:
        root, backend, config = fake_root(Path(raw))
        build_native_fixture(backend)
        pinned = pin_campaign_artifacts(root, backend, config)
        manifest = json.loads(pinned.manifest.read_text(encoding="utf-8"))

        runtime_hashes = manifest.get("runtimeSourceHashes") or {}
        assert set(runtime_hashes) == set(RUNTIME_SOURCE_RELATIVE_PATHS)
        for relative in RUNTIME_SOURCE_RELATIVE_PATHS:
            assert manifest["sourceHashes"][relative] == runtime_hashes[relative]

        runner = root / "Tools" / "headless_flight.py"
        runner.write_text("fixture source: changed headless runner\n", encoding="utf-8")
        try:
            load_pinned_campaign_artifacts(root, pinned.identity)
        except RuntimeError as exc:
            assert "runtime sources changed since pinning" in str(exc)
            assert "Tools/headless_flight.py" in str(exc)
        else:
            raise AssertionError("exact restart accepted mutated live runtime source")

        repinned = pin_campaign_artifacts(root, backend, config)
        assert repinned.identity != pinned.identity


def test_headless_live_launch_has_mandatory_pinned_path_contract() -> None:
    source = (ROOT / "Tools" / "headless_flight.py").read_text(encoding="utf-8")
    assert "if args.live and not args.connect_only and not args.no_pin_campaign_artifacts:" in source
    assert "backend_path = campaign_pin.backend" in source
    assert "backend = BackendProcess(backend_path)" in source
    assert "verify_running_executable(backend.process.pid, backend_path)" in source
    assert "write_run_manifest(" in source
    assert '"--campaign-identity"' in source
    assert "config_was_explicit" in source
    assert "Creating a new pinned live KSP campaign requires an explicit --config path" in source
    launcher = (ROOT / "run_headless.command").read_text(encoding="utf-8")
    assert "KSP_LANDER_CAMPAIGN_IDENTITY" in launcher
    assert "--campaign-identity" in launcher
    assert 'if [ "$PINNED_RESTART" -eq 0 ]; then' in launcher


def test_running_process_inode_matches_pinned_native_executable() -> None:
    with tempfile.TemporaryDirectory(prefix="campaign-provenance-pid-") as raw:
        root, backend, config = fake_root(Path(raw))
        build_native_fixture(backend)
        pinned = pin_campaign_artifacts(root, backend, config)
        process = subprocess.Popen([str(pinned.backend), "5"])
        try:
            time.sleep(0.05)
            verified_inode = verify_running_executable(process.pid, pinned.backend)
            assert verified_inode == pinned.backend_inode
            run_manifest = write_run_manifest(
                root / "Runtime" / "Headless",
                pinned,
                ["headless_flight.py", "--live"],
                process.pid,
                verified_inode,
            )
            payload = json.loads(run_manifest.read_text(encoding="utf-8"))
            assert payload["campaignIdentity"] == pinned.identity
            assert payload["sourceTreeSha256"] == pinned.source_tree_sha256
            assert payload["runtimeSourceHashes"] == json.loads(pinned.manifest.read_text(encoding="utf-8"))["runtimeSourceHashes"]
            assert payload["backend"]["sha256"] == pinned.backend_sha256
            assert payload["backend"]["verifiedExecutableInode"] == pinned.backend_inode
            assert payload["configuration"]["sha256"] == pinned.config_sha256
            assert run_manifest.stat().st_mode & 0o222 == 0
        finally:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)


def main() -> int:
    test_mutable_source_replacement_cannot_change_pinned_restart()
    test_effective_runtime_configuration_is_the_pinned_identity()
    test_runtime_source_mutation_changes_identity_and_blocks_exact_restart()
    test_headless_live_launch_has_mandatory_pinned_path_contract()
    test_running_process_inode_matches_pinned_native_executable()
    print("Campaign provenance tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
