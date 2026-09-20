#!/usr/bin/env python3
"""Immutable live-campaign backend/config snapshots and process provenance checks."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any


@dataclass(frozen=True)
class PinnedCampaignArtifacts:
    identity: str
    directory: Path
    backend: Path
    config: Path
    manifest: Path
    backend_sha256: str
    config_sha256: str
    source_tree_sha256: str
    backend_inode: int


# These files influence live control, safety recovery, campaign pinning, or the
# acceptance verdict without being compiled into the pinned native backend.
# They therefore have to remain byte-identical across an exact campaign retry.
RUNTIME_SOURCE_RELATIVE_PATHS = (
    "run_headless.command",
    "Tools/campaign_provenance.py",
    "Tools/headless_flight.py",
    "AgentWork/RunGuard-execution-boundary/detached_runner.py",
    "Tools/ksp_test_guard.py",
    "Tools/postflight_acceptance.py",
    "Tools/postflight_75km_acceptance.py",
)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _read_stable_file(path: Path) -> tuple[bytes, os.stat_result]:
    """Capture bytes and the inode metadata that actually owns those bytes."""
    with path.open("rb") as handle:
        before = os.fstat(handle.fileno())
        data = handle.read()
        after = os.fstat(handle.fileno())
    fingerprint_before = (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
    fingerprint_after = (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns)
    if fingerprint_before != fingerprint_after:
        raise RuntimeError(f"Campaign artifact changed while being captured: {path}")
    return data, before


def runtime_source_hashes(root: Path) -> dict[str, str]:
    """Hash non-native production scripts that affect a countable campaign."""
    root = root.resolve()
    hashes: dict[str, str] = {}
    for relative in RUNTIME_SOURCE_RELATIVE_PATHS:
        path = root / relative
        if not path.is_file():
            raise RuntimeError(f"Production runtime source is missing: {path}")
        hashes[relative] = sha256_file(path)
    return hashes


def production_source_hashes(root: Path) -> dict[str, str]:
    root = root.resolve()
    source_root = root / "CLanding"
    paths = [
        path for path in source_root.iterdir()
        if path.is_file() and (path.suffix in {".c", ".h"} or path.name == "Makefile")
    ]
    hashes = {
        str(path.relative_to(root)): sha256_file(path)
        for path in sorted(paths, key=lambda item: item.name)
    }
    hashes.update(runtime_source_hashes(root))
    return dict(sorted(hashes.items()))


def source_tree_digest(hashes: dict[str, str]) -> str:
    encoded = json.dumps(hashes, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return sha256_bytes(encoded)


def _write_pinned(path: Path, data: bytes, mode: int) -> None:
    expected = sha256_bytes(data)
    if path.exists():
        if sha256_file(path) != expected:
            raise RuntimeError(f"Pinned campaign artifact collision at {path}")
        os.chmod(path, mode)
        return
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    try:
        with temporary.open("xb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, mode)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def pin_campaign_artifacts(
    root: Path,
    backend: Path,
    config: Path,
    artifact_root: Path | None = None,
    *,
    effective_configuration: dict[str, Any] | None = None,
) -> PinnedCampaignArtifacts:
    root = root.resolve()
    backend = backend.resolve()
    config = config.resolve()
    if not backend.is_file():
        raise FileNotFoundError(f"Landing backend does not exist: {backend}")
    if not config.is_file():
        raise FileNotFoundError(f"Configuration does not exist: {config}")

    before_sources = production_source_hashes(root)
    backend_data, backend_source_stat = _read_stable_file(backend)
    source_config_data, config_source_stat = _read_stable_file(config)
    after_sources = production_source_hashes(root)
    if before_sources != after_sources:
        raise RuntimeError("CLanding production source changed while campaign artifacts were being pinned")

    if effective_configuration is None:
        effective_config_data = source_config_data
    else:
        effective_config_data = (
            json.dumps(effective_configuration, indent=2, sort_keys=True, allow_nan=False) + "\n"
        ).encode("utf-8")

    backend_sha = sha256_bytes(backend_data)
    source_config_sha = sha256_bytes(source_config_data)
    config_sha = sha256_bytes(effective_config_data)
    source_sha = source_tree_digest(before_sources)
    identity = f"{source_sha[:16]}-{backend_sha[:16]}-{config_sha[:16]}"
    base = (artifact_root or root / "Runtime" / "Headless" / "acceptance-artifacts").resolve()
    directory = base / identity
    directory.mkdir(parents=True, exist_ok=True)
    pinned_backend = directory / "landing_backend"
    pinned_config = directory / "configuration.json"
    manifest_path = directory / "manifest.json"

    _write_pinned(pinned_backend, backend_data, 0o555)
    _write_pinned(pinned_config, effective_config_data, 0o444)
    if sha256_file(pinned_backend) != backend_sha or sha256_file(pinned_config) != config_sha:
        raise RuntimeError("Pinned campaign artifact verification failed")

    backend_inode = pinned_backend.stat().st_ino
    try:
        current_backend_stat = backend.stat()
        source_path_still_matches_capture = (
            current_backend_stat.st_dev == backend_source_stat.st_dev
            and current_backend_stat.st_ino == backend_source_stat.st_ino
        )
    except FileNotFoundError:
        source_path_still_matches_capture = False

    manifest: dict[str, Any] = {
        "schemaVersion": 2,
        "identity": identity,
        "createdUTC": datetime.now(timezone.utc).isoformat(),
        "root": str(root),
        "sourceTreeSha256": source_sha,
        "sourceHashes": before_sources,
        "runtimeSourceHashes": {
            relative: before_sources[relative] for relative in RUNTIME_SOURCE_RELATIVE_PATHS
        },
        "backend": {
            "sourcePath": str(backend),
            "sourceDevice": backend_source_stat.st_dev,
            "sourceInode": backend_source_stat.st_ino,
            "sourceSize": backend_source_stat.st_size,
            "sourceMtimeNs": backend_source_stat.st_mtime_ns,
            "sourcePathStillMatchesCapture": source_path_still_matches_capture,
            "pinnedPath": str(pinned_backend),
            "pinnedInode": backend_inode,
            "sha256": backend_sha,
            "size": len(backend_data),
        },
        "configuration": {
            "sourcePath": str(config),
            "sourceDevice": config_source_stat.st_dev,
            "sourceInode": config_source_stat.st_ino,
            "sourceSha256": source_config_sha,
            "pinnedPath": str(pinned_config),
            "sha256": config_sha,
            "effective": effective_configuration is not None,
            "size": len(effective_config_data),
        },
    }
    if manifest_path.exists():
        existing = json.loads(manifest_path.read_text(encoding="utf-8"))
        if existing.get("identity") != identity or existing.get("sourceTreeSha256") != source_sha:
            raise RuntimeError(f"Pinned campaign manifest collision at {manifest_path}")
        if (existing.get("backend") or {}).get("sha256") != backend_sha:
            raise RuntimeError(f"Pinned campaign backend hash mismatch at {manifest_path}")
        if (existing.get("configuration") or {}).get("sha256") != config_sha:
            raise RuntimeError(f"Pinned campaign configuration hash mismatch at {manifest_path}")
    else:
        temporary = manifest_path.with_name(f".{manifest_path.name}.tmp-{os.getpid()}")
        try:
            temporary.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            with temporary.open("rb") as handle:
                os.fsync(handle.fileno())
            os.chmod(temporary, 0o444)
            os.replace(temporary, manifest_path)
        finally:
            if temporary.exists():
                temporary.unlink()

    return PinnedCampaignArtifacts(
        identity=identity,
        directory=directory,
        backend=pinned_backend,
        config=pinned_config,
        manifest=manifest_path,
        backend_sha256=backend_sha,
        config_sha256=config_sha,
        source_tree_sha256=source_sha,
        backend_inode=backend_inode,
    )


def load_pinned_campaign_artifacts(
    root: Path,
    identity: str,
    artifact_root: Path | None = None,
) -> PinnedCampaignArtifacts:
    """Reopen one immutable candidate so every retry uses the exact same bytes."""
    parts = identity.strip().lower().split("-")
    if len(parts) != 3 or any(len(part) != 16 or any(ch not in "0123456789abcdef" for ch in part) for part in parts):
        raise RuntimeError(f"Invalid campaign identity: {identity!r}")
    identity = "-".join(parts)
    base = (artifact_root or root.resolve() / "Runtime" / "Headless" / "acceptance-artifacts").resolve()
    directory = base / identity
    manifest_path = directory / "manifest.json"
    if not manifest_path.is_file():
        raise RuntimeError(f"Pinned campaign identity does not exist: {identity}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("identity") != identity:
        raise RuntimeError(f"Pinned campaign manifest identity mismatch: {manifest_path}")
    backend = directory / "landing_backend"
    config = directory / "configuration.json"
    backend_sha = str((manifest.get("backend") or {}).get("sha256") or "")
    config_sha = str((manifest.get("configuration") or {}).get("sha256") or "")
    source_sha = str(manifest.get("sourceTreeSha256") or "")
    source_hashes = manifest.get("sourceHashes")
    if not source_sha:
        raise RuntimeError(f"Pinned campaign source hash missing: {manifest_path}")
    if not isinstance(source_hashes, dict) or not source_hashes:
        raise RuntimeError(f"Pinned campaign source hashes missing: {manifest_path}")
    if source_tree_digest({str(key): str(value) for key, value in source_hashes.items()}) != source_sha:
        raise RuntimeError(f"Pinned campaign source hash manifest mismatch: {manifest_path}")
    pinned_runtime_hashes = manifest.get("runtimeSourceHashes")
    if not isinstance(pinned_runtime_hashes, dict) or not pinned_runtime_hashes:
        raise RuntimeError(
            f"Pinned campaign runtime source hashes missing; identity predates runtime-script binding: {manifest_path}"
        )
    current_runtime_hashes = runtime_source_hashes(root)
    normalized_runtime_hashes = {str(key): str(value) for key, value in pinned_runtime_hashes.items()}
    if current_runtime_hashes != normalized_runtime_hashes:
        changed = sorted(
            key for key in set(current_runtime_hashes) | set(normalized_runtime_hashes)
            if current_runtime_hashes.get(key) != normalized_runtime_hashes.get(key)
        )
        raise RuntimeError(
            "Pinned campaign runtime sources changed since pinning; exact restart is unsafe: "
            + ", ".join(changed)
        )
    if sha256_file(backend) != backend_sha:
        raise RuntimeError(f"Pinned campaign backend hash mismatch: {backend}")
    if sha256_file(config) != config_sha:
        raise RuntimeError(f"Pinned campaign configuration hash mismatch: {config}")
    if not os.access(backend, os.X_OK):
        raise RuntimeError(f"Pinned campaign backend is not executable: {backend}")
    return PinnedCampaignArtifacts(
        identity=identity,
        directory=directory,
        backend=backend,
        config=config,
        manifest=manifest_path,
        backend_sha256=backend_sha,
        config_sha256=config_sha,
        source_tree_sha256=source_sha,
        backend_inode=backend.stat().st_ino,
    )


def write_run_manifest(
    runtime: Path,
    pinned: PinnedCampaignArtifacts,
    argv: list[str],
    backend_pid: int,
    verified_inode: int,
) -> Path:
    """Persist the exact source/config/backend identity used by one live invocation."""
    candidate_manifest = json.loads(pinned.manifest.read_text(encoding="utf-8"))
    run_root = runtime / "run-manifests"
    run_root.mkdir(parents=True, exist_ok=True)
    now = datetime.now(timezone.utc)
    name = f"{now.strftime('%Y%m%dT%H%M%S%fZ')}-{backend_pid}.json"
    path = run_root / name
    payload = {
        "schemaVersion": 1,
        "startedUTC": now.isoformat(),
        "argv": argv,
        "campaignIdentity": pinned.identity,
        "campaignManifest": str(pinned.manifest),
        "sourceTreeSha256": pinned.source_tree_sha256,
        "runtimeSourceHashes": candidate_manifest.get("runtimeSourceHashes"),
        "backend": {
            "path": str(pinned.backend),
            "sha256": pinned.backend_sha256,
            "pinnedInode": pinned.backend_inode,
            "processPid": backend_pid,
            "verifiedExecutableInode": verified_inode,
        },
        "configuration": {
            "path": str(pinned.config),
            "sha256": pinned.config_sha256,
            "sourceSha256": (candidate_manifest.get("configuration") or {}).get("sourceSha256"),
        },
    }
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    try:
        temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        with temporary.open("rb") as handle:
            os.fsync(handle.fileno())
        os.chmod(temporary, 0o444)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return path


def verify_running_executable(pid: int, executable: Path) -> int:
    """Return the verified executable inode or raise if PID is not running that file."""
    executable = executable.resolve()
    expected_inode = executable.stat().st_ino
    if sys.platform == "darwin":
        proc = subprocess.run(
            ["lsof", "-a", "-p", str(pid), "-d", "txt", "-Fnfi"],
            text=True,
            capture_output=True,
            check=True,
        )
        current: dict[str, str] = {}
        records: list[dict[str, str]] = []
        for raw in proc.stdout.splitlines():
            if not raw:
                continue
            tag, value = raw[0], raw[1:]
            if tag == "f":
                if current:
                    records.append(current)
                current = {"f": value}
            elif tag in {"i", "n"}:
                current[tag] = value
        if current:
            records.append(current)
        for record in records:
            if record.get("n") == str(executable) and int(record.get("i", "-1")) == expected_inode:
                return expected_inode
        raise RuntimeError(
            f"Backend PID {pid} is not mapped to pinned executable {executable} inode {expected_inode}"
        )

    proc_exe = Path(f"/proc/{pid}/exe")
    if proc_exe.exists():
        actual = proc_exe.resolve()
        if actual == executable and actual.stat().st_ino == expected_inode:
            return expected_inode
        raise RuntimeError(f"Backend PID {pid} executable {actual} does not match pinned {executable}")
    raise RuntimeError(f"Executable provenance verification is unsupported on platform {sys.platform!r}")
