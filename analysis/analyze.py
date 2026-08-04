#!/usr/bin/env python3
"""Aggregate batch landing metas into per-group metrics table."""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
BATCH_DIR = PROJECT_ROOT / "data" / "batch"
REPORT_DIR = PROJECT_ROOT / "data" / "report"


def load_metas(batch_dir: Path) -> list[dict]:
    metas: list[dict] = []
    if not batch_dir.is_dir():
        return metas
    for d in sorted(batch_dir.iterdir()):
        meta_path = d / "meta.json"
        if d.is_dir() and meta_path.is_file():
            metas.append(json.loads(meta_path.read_text(encoding="utf-8")))
    return metas


def group_key(meta: dict) -> str:
    s = meta.get("scenario", {})
    if meta.get("tag", "").startswith("baseline"):
        return "baseline"
    if s.get("platform_vx_m_s", 0.0) != 0.0:
        return "platform"
    if s.get("nonidealities"):
        return "nonideal"
    if s.get("offset_x_m", 0.0) != 0.0:
        return f"offset_{s['offset_x_m']}m"
    return f"dist_{s.get('disturbance_preset', 'off')}"


def _stats(values: list[float]) -> dict:
    if not values:
        return {"mean": None, "std": None}
    return {"mean": round(statistics.mean(values), 4),
            "std": round(statistics.stdev(values), 4) if len(values) > 1 else 0.0}


def aggregate(metas: list[dict]) -> list[dict]:
    groups: dict[str, list[dict]] = {}
    for m in metas:
        groups.setdefault(group_key(m), []).append(m)
    rows = []
    for key in sorted(groups):
        group = groups[key]
        landed = [m for m in group if m.get("outcome") == "LANDED"]
        vz = [float(m["landing"]["touchdown_vz_m_s"]) for m in landed]
        err = [float(m["landing"]["final_horizontal_error_m"]) for m in landed]
        roll = [float(m["landing"]["max_abs_roll_deg"]) for m in landed]
        aborts = [str(m["landing"].get("abort_reason", "")) for m in group
                  if m.get("outcome") == "ABORTED"]
        vz_s, err_s, roll_s = _stats(vz), _stats(err), _stats(roll)
        rows.append({
            "group": key,
            "n": len(group),
            "landed": len(landed),
            "success_rate": round(len(landed) / len(group), 4),
            "touchdown_vz_mean": vz_s["mean"],
            "touchdown_vz_std": vz_s["std"],
            "horizontal_error_mean": err_s["mean"],
            "horizontal_error_std": err_s["std"],
            "max_roll_mean": roll_s["mean"],
            "max_roll_std": roll_s["std"],
            "abort_reasons": ";".join(sorted(set(aborts))),
        })
    return rows


def main() -> int:
    p = argparse.ArgumentParser(description="Aggregate batch landing metas.")
    p.add_argument("--batch-dir", default=str(BATCH_DIR))
    p.add_argument("--out-csv", default=str(REPORT_DIR / "metrics.csv"))
    p.add_argument("--out-md", default=str(REPORT_DIR / "batch_summary.md"))
    args = p.parse_args()

    metas = load_metas(Path(args.batch_dir))
    rows = aggregate(metas)
    REPORT_DIR.mkdir(parents=True, exist_ok=True)

    fields = ["group", "n", "landed", "success_rate",
              "touchdown_vz_mean", "touchdown_vz_std",
              "horizontal_error_mean", "horizontal_error_std",
              "max_roll_mean", "max_roll_std", "abort_reasons"]
    import csv
    with Path(args.out_csv).open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    lines = ["# 批量起降结果汇总\n",
             "| 分组 | n | 成功 | 成功率 | 触水 vz(m/s) | 落点偏差(m) | 最大横滚(deg) | 中止原因 |",
             "|---|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(
            f"| {r['group']} | {r['n']} | {r['landed']} | {r['success_rate']:.2f} "
            f"| {r['touchdown_vz_mean']}±{r['touchdown_vz_std']} "
            f"| {r['horizontal_error_mean']}±{r['horizontal_error_std']} "
            f"| {r['max_roll_mean']}±{r['max_roll_std']} | {r['abort_reasons']} |")
    Path(args.out_md).write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"aggregated {len(metas)} metas -> {args.out_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
