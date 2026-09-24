#!/usr/bin/env python3
"""Batch compress all simulation runs in ShuttleSim/runs to .gz."""
from __future__ import annotations
import gzip
import json
import os
from pathlib import Path
import shutil
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / "ShuttleSim/runs"


def compress_file(src_path_str: str) -> tuple[str, int, int]:
    src = Path(src_path_str)
    if not src.is_file() or src.suffix != ".jsonl":
        return str(src), 0, 0
    orig_size = src.stat().st_size
    dst = src.with_name(src.name + ".gz")
    tmp = src.with_name(src.name + ".tmp.gz")
    try:
        with src.open("rb") as f_in, gzip.open(tmp, "wb", compresslevel=6) as f_out:
            shutil.copyfileobj(f_in, f_out, length=1024 * 1024)
        # Verify
        with gzip.open(tmp, "rb") as verify_f:
            verify_f.seek(0, 2)
            uncompressed_pos = verify_f.tell()
            if uncompressed_pos != orig_size:
                raise ValueError(f"Integrity check failed: got {uncompressed_pos}, expected {orig_size}")
        tmp.replace(dst)
        src.unlink()
        compressed_size = dst.stat().st_size
        return str(src), orig_size, compressed_size
    except Exception as exc:
        tmp.unlink(missing_ok=True)
        raise RuntimeError(f"Failed to compress {src}: {exc}") from exc


def update_manifests():
    for manifest_path in RUNS.glob("*/manifest.json"):
        try:
            m = json.loads(manifest_path.read_text(encoding="utf-8"))
            changed = False
            for k in ("guidanceSnapshots", "simulatorTelemetry"):
                val = m.get(k)
                if isinstance(val, str) and val.endswith(".jsonl"):
                    # Check if .gz exists
                    p = manifest_path.parent / val
                    gz_p = manifest_path.parent / (val + ".gz")
                    if gz_p.is_file() and not p.is_file():
                        m[k] = val + ".gz"
                        changed = True
            if changed:
                manifest_path.write_text(json.dumps(m, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        except Exception:
            pass


def main():
    jsonls = sorted(list(RUNS.glob("*/*.jsonl")) + list(RUNS.glob("*.jsonl")), key=lambda p: p.stat().st_size)
    print(f"Found {len(jsonls)} .jsonl files to compress...")
    total_orig = 0
    total_compressed = 0
    t0 = time.time()
    
    with ProcessPoolExecutor(max_workers=4) as executor:
        futures = {executor.submit(compress_file, str(p)): p for p in jsonls}
        done = 0
        for future in as_completed(futures):
            src_str, orig_sz, comp_sz = future.result()
            total_orig += orig_sz
            total_compressed += comp_sz
            done += 1
            if done % 50 == 0 or done == len(jsonls):
                elapsed = time.time() - t0
                pct = total_compressed / total_orig * 100 if total_orig > 0 else 0
                saved_mb = (total_orig - total_compressed) / (1024 * 1024)
                print(f"[{done}/{len(jsonls)}] Compressed {total_orig/(1024*1024):.1f} MB -> {total_compressed/(1024*1024):.1f} MB ({pct:.1f}%, saved {saved_mb:.1f} MB) in {elapsed:.1f}s", flush=True)

    print("Updating manifest.json files...")
    update_manifests()
    print("Done!")


if __name__ == "__main__":
    main()
