#!/usr/bin/env python3
"""Import the authoritative 2026-09-02 visual-team handoff into liftrace-sim."""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import zipfile
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PROJECT_ROOT.parent
DEFAULT_ARCHIVE = WORKSPACE_ROOT / "liftrace_vision_to_navigation_handoff_20260902_v2.zip"
DEFAULT_OUTPUT = PROJECT_ROOT / "data" / "vision" / "vsim04_20260902_v2"
ARCHIVE_ROOT = "liftrace_vision_to_navigation_handoff_20260902_v2"
BATCHES = {
    "operating_surface_trials.csv": "B_full100_seed11",
    "lateral_trials.csv": "C_post_fix_seed11",
    "motion_trials.csv": "D_supported16_seed11",
}


def _member(relative: str) -> str:
    return f"{ARCHIVE_ROOT}/{relative}"


def _read_text(archive: zipfile.ZipFile, relative: str) -> str:
    return archive.read(_member(relative)).decode("utf-8-sig")


def _read_csv(archive: zipfile.ZipFile, relative: str) -> list[dict[str, str]]:
    return list(csv.DictReader(io.StringIO(_read_text(archive, relative))))


def _bool(value: str) -> bool | None:
    normalized = value.strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    return None


def _validate_manifest(archive: zipfile.ZipFile) -> int:
    checked = 0
    for line in _read_text(archive, "MANIFEST.sha256").splitlines():
        if not line.strip():
            continue
        expected, relative = line.split(maxsplit=1)
        relative = relative.removeprefix("./")
        actual = hashlib.sha256(archive.read(_member(relative))).hexdigest()
        if actual != expected:
            raise ValueError(f"checksum mismatch: {relative}")
        checked += 1
    return checked


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", nargs="?", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    archive_digest = hashlib.sha256(args.archive.read_bytes()).hexdigest()

    with zipfile.ZipFile(args.archive) as archive:
        bad_member = archive.testzip()
        if bad_member:
            raise ValueError(f"invalid ZIP member: {bad_member}")
        manifest_files_checked = _validate_manifest(archive)
        source_revision = _read_text(archive, "SOURCE_REVISION.txt").strip()
        batch_rows: dict[str, list[dict[str, str]]] = {}
        batch_text: dict[str, str] = {}
        for output_name, batch in BATCHES.items():
            relative = f"02_VSIM04/{batch}/vsim04/vision_search_performance.csv"
            batch_text[output_name] = _read_text(archive, relative)
            batch_rows[batch] = _read_csv(archive, relative)

    b_rows = batch_rows["B_full100_seed11"]
    if len(b_rows) != 100 or len({row["trial_id"] for row in b_rows}) != 100:
        raise ValueError("authoritative B batch must contain 100 unique trials")
    expected_grid = {
        (name, height, speed)
        for name in ("tent", "pillbox", "bridge", "panzer", "red_cross")
        for height in (1.2, 1.8, 2.4, 3.0, 3.6)
        for speed in (0.5, 1.0, 1.5, 2.0)
    }
    actual_grid = {
        (row["class_name"], float(row["height_m"]), float(row["speed_mps"]))
        for row in b_rows
    }
    if actual_grid != expected_grid:
        raise ValueError("authoritative B batch does not cover the expected 5x5x4 grid")
    c_rows = batch_rows["C_post_fix_seed11"]
    d_rows = batch_rows["D_supported16_seed11"]
    if len(c_rows) != 25 or len(d_rows) != 16:
        raise ValueError(f"unexpected C/D row counts: C={len(c_rows)}, D={len(d_rows)}")

    args.output.mkdir(parents=True, exist_ok=True)
    for output_name, payload in batch_text.items():
        (args.output / output_name).write_text(
            payload if payload.endswith("\n") else payload + "\n", encoding="utf-8"
        )

    condition_path = args.output / "condition_success_rates.csv"
    fields = [
        "kind", "class_name", "height_m", "speed_mps", "successes", "trials",
        "p_selected", "p_confirm", "failure_stage", "source_batch", "seed_scope",
        "performance_verdict",
    ]
    with condition_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in sorted(b_rows, key=lambda item: (
            item["class_name"], float(item["height_m"]), float(item["speed_mps"])
        )):
            selected, confirmed = _bool(row["p_selected"]), _bool(row["p_confirm"])
            writer.writerow({
                "kind": "dynamic", "class_name": row["class_name"],
                "height_m": row["height_m"], "speed_mps": row["speed_mps"],
                "successes": int(selected is True), "trials": 1,
                "p_selected": "" if selected is None else int(selected),
                "p_confirm": "" if confirmed is None else int(confirmed),
                "failure_stage": row["failure_stage"] or "none",
                "source_batch": "B_full100_seed11", "seed_scope": "single_seed_11",
                "performance_verdict": row["performance_verdict"],
            })

    provenance = {
        "schema_version": 2,
        "source_archive": args.archive.name,
        "source_archive_sha256": archive_digest,
        "source_revision": source_revision,
        "manifest_files_checked": manifest_files_checked,
        "authoritative_batches": {
            "operating_surface": {"name": "B_full100_seed11", "trials": len(b_rows)},
            "lateral": {"name": "C_post_fix_seed11", "trials": len(c_rows)},
            "motion": {"name": "D_supported16_seed11", "trials": len(d_rows)},
        },
        "simulation_table": condition_path.name,
        "interpretation": {
            "probability_field": "p_selected", "p_interrupt": None,
            "performance_status": "DIAGNOSTIC_ONLY_SINGLE_SEED",
        },
        "limitations": [
            "All authoritative V-SIM-04 batches use seed 11; no confidence interval is implied.",
            "B_full100_seed11 is diagnostic-only because its P95 confirmation processing time exceeds the frozen threshold.",
            "C_post_fix_seed11 is NOT_GATED and shows lateral asymmetry; partial-frame rows are not valid full-frame observations.",
            "D_supported16_seed11 covers only 16 supported single-target designs; the other proposed designs are NOT_RUN, not failures.",
            "Visual p_selected is not navigation P_interrupt; visual-only P_interrupt remains null.",
            "Historical formal23/sparse30/repeat data are intentionally excluded.",
        ],
    }
    (args.output / "provenance.json").write_text(
        json.dumps(provenance, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (args.output / "README.md").write_text(
        "# V-SIM-04 视觉交付导入（2026-09-02 v2）\n\n"
        "仿真查表仅使用 `B_full100_seed11`；未混入历史 formal23、sparse30 或重复运行。\n\n"
        "- `condition_success_rates.csv`：B100 的 5 类 × 5 高度 × 4 速度精确查表；\n"
        "- `operating_surface_trials.csv`：B100 原始逐试验汇总表；\n"
        "- `lateral_trials.csv`：C25 post-fix 横向实验逐试验表；\n"
        "- `motion_trials.csv`：D16 supported 运动实验逐试验表；\n"
        "- `provenance.json`：归档哈希、源版本、批次与解释边界。\n\n"
        f"原始交付包保留在工作区根目录 `{args.archive.name}`，不在数据目录内重复存储。\n\n"
        "注意：这些结果均为单 seed；`p_selected` 不是导航 `P_interrupt`。\n",
        encoding="utf-8",
    )
    print(f"wrote {args.output} (B={len(b_rows)}, C={len(c_rows)}, D={len(d_rows)}, manifest={manifest_files_checked} files verified)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
