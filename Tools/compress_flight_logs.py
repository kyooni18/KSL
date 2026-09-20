#!/usr/bin/env python3
"""Losslessly archive KSPShutltleLander JSONL flight logs.

The logger intentionally records large forensic snapshots.  Those snapshots
contain highly repetitive trajectory data, so Zstandard is dramatically more
effective than gzip for these files.  Python 3.14 ships ``compression.zstd``
in the standard library, which keeps this tool dependency-free on the current
project runtime.

By default the script scans the application's FlightLogs directory and keeps
the original JSONL files.  Use --delete-original only when archival storage is
the desired end state.  A source file that changes while it is being archived
is treated as active and is never deleted.
"""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import shutil
import sys
import time

try:
    import compression.zstd as zstd
except ImportError as exc:  # pragma: no cover - project runtime is Python 3.14+
    raise SystemExit(
        "Python 3.14+ is required because this tool uses the standard-library "
        "compression.zstd module."
    ) from exc


DEFAULT_LOG_DIR = Path(__file__).resolve().parents[1] / "FlightLogs"

CHUNK_SIZE = 8 * 1024 * 1024
PROGRESS_INTERVAL = 256 * 1024 * 1024


def human_bytes(value: int) -> str:
    units = ("B", "KiB", "MiB", "GiB", "TiB")
    amount = float(value)
    for unit in units:
        if amount < 1024.0 or unit == units[-1]:
            return f"{amount:.1f} {unit}"
        amount /= 1024.0
    return f"{value} B"


def source_identity(path: Path) -> tuple[int, int, int]:
    stat = path.stat()
    return stat.st_size, stat.st_mtime_ns, stat.st_ino


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while chunk := handle.read(CHUNK_SIZE):
            digest.update(chunk)
    return digest.hexdigest()


def verify_archive(archive: Path, expected_sha256: str, expected_size: int) -> None:
    digest = hashlib.sha256()
    size = 0
    with zstd.open(archive, "rb") as handle:
        while chunk := handle.read(CHUNK_SIZE):
            digest.update(chunk)
            size += len(chunk)

    if size != expected_size:
        raise RuntimeError(
            f"verification failed: decompressed size is {size}, expected {expected_size}"
        )
    if digest.hexdigest() != expected_sha256:
        raise RuntimeError("verification failed: SHA-256 mismatch")


def compress_log(
    source: Path,
    *,
    level: int,
    delete_original: bool,
    verify: bool,
    force: bool,
) -> bool:
    source = source.expanduser().resolve()
    if not source.is_file():
        print(f"skip: not a file: {source}", file=sys.stderr)
        return False
    if source.suffix != ".jsonl" and not force:
        print(f"skip: not a .jsonl flight log: {source}", file=sys.stderr)
        return False

    archive = source.with_suffix(source.suffix + ".zst")
    temporary = archive.with_suffix(archive.suffix + ".tmp")

    if archive.exists() and not force:
        print(f"skip: archive already exists: {archive.name}")
        return False

    start_identity = source_identity(source)
    source_size = start_identity[0]
    digest = hashlib.sha256()
    processed = 0
    next_progress = PROGRESS_INTERVAL
    started = time.monotonic()

    print(f"archive: {source.name} ({human_bytes(source_size)})")
    temporary.unlink(missing_ok=True)

    try:
        with source.open("rb") as reader, zstd.open(
            temporary, "wb", level=level
        ) as writer:
            while chunk := reader.read(CHUNK_SIZE):
                digest.update(chunk)
                writer.write(chunk)
                processed += len(chunk)
                if processed >= next_progress:
                    pct = processed / source_size * 100 if source_size else 100.0
                    print(f"  {pct:5.1f}%  {human_bytes(processed)}")
                    next_progress += PROGRESS_INTERVAL

        # The flight controller may still be appending to the file.  Refuse to
        # publish a misleading archive if the source changed during the read.
        end_identity = source_identity(source)
        if end_identity != start_identity:
            temporary.unlink(missing_ok=True)
            print(
                "  source changed while archiving; it is probably an active "
                "flight log, so the archive was discarded"
            )
            return False

        os.replace(temporary, archive)

        should_verify = verify or delete_original
        if should_verify:
            print("  verifying archive")
            verify_archive(archive, digest.hexdigest(), source_size)

        archive_size = archive.stat().st_size
        ratio = archive_size / source_size * 100 if source_size else 0.0
        elapsed = max(time.monotonic() - started, 0.001)
        print(
            f"  done: {human_bytes(source_size)} -> {human_bytes(archive_size)} "
            f"({ratio:.2f}%), {human_bytes(int(source_size / elapsed))}/s"
        )

        if delete_original:
            source.unlink()
            print("  original removed after successful verification")

        return True
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def collect_logs(inputs: list[Path]) -> list[Path]:
    result: list[Path] = []
    seen: set[Path] = set()

    for item in inputs:
        path = item.expanduser()
        candidates = sorted(path.glob("*.jsonl")) if path.is_dir() else [path]
        for candidate in candidates:
            resolved = candidate.resolve()
            if resolved not in seen:
                seen.add(resolved)
                result.append(resolved)

    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Archive KSPShutltleLander JSONL flight logs with Zstandard."
    )
    parser.add_argument(
        "paths",
        nargs="*",
        type=Path,
        help=(
            "JSONL files or directories. Defaults to the application's "
            "FlightLogs directory."
        ),
    )
    parser.add_argument(
        "--level",
        type=int,
        default=10,
        help="Zstandard compression level (default: 10).",
    )
    parser.add_argument(
        "--delete-original",
        action="store_true",
        help="Delete each JSONL only after the archive has been verified.",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Decompress and SHA-256 verify each archive after writing it.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite an existing .zst archive and allow non-.jsonl inputs.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    inputs = args.paths or [DEFAULT_LOG_DIR]
    logs = collect_logs(inputs)

    if not logs:
        print("No flight logs found.")
        return 0

    archived = 0
    failures = 0
    for log in logs:
        try:
            if compress_log(
                log,
                level=args.level,
                delete_original=args.delete_original,
                verify=args.verify,
                force=args.force,
            ):
                archived += 1
        except Exception as exc:
            failures += 1
            print(f"error: {log}: {exc}", file=sys.stderr)

    print(f"finished: {archived} archived, {failures} failed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
