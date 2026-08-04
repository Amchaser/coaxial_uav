#!/usr/bin/env python3
"""Batch runner: build a scenario matrix and run each via run_one_flight.py."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUN_ONE = PROJECT_ROOT / "scripts" / "run_one_flight.py"
DISTURBANCE_PRESETS = ["off", "calm", "mild", "strong", "asymmetric"]
OFFSETS = [0.0, 1.0, 2.0, 3.0]


def build_matrix(repeat: int = 10) -> list[dict]:
    scenarios: list[dict] = []
    # 1) 标况基线组
    for i in range(repeat):
        scenarios.append({
            "tag": f"baseline_repeat{i:02d}",
            "disturbance_preset": "off", "offset": 0.0,
            "nonidealities": False, "platform_vx": 0.0, "config_json": "{}",
        })
    # 2) 扰动扫描组
    for preset in DISTURBANCE_PRESETS:
        scenarios.append({
            "tag": f"dist_{preset}", "disturbance_preset": preset,
            "offset": 0.0, "nonidealities": False, "platform_vx": 0.0,
            "config_json": "{}",
        })
    # 3) 偏移扫描组
    for off in OFFSETS:
        scenarios.append({
            "tag": f"offset_{off}m", "disturbance_preset": "off",
            "offset": off, "nonidealities": False, "platform_vx": 0.0,
            "config_json": "{}",
        })
    # 4) 非理想性组
    scenarios.append({
        "tag": "nonideal_on", "disturbance_preset": "off", "offset": 0.0,
        "nonidealities": True, "platform_vx": 0.0, "config_json": "{}",
    })
    # 5) 移动平台组
    scenarios.append({
        "tag": "platform_0.3ms", "disturbance_preset": "off", "offset": 0.0,
        "nonidealities": False, "platform_vx": 0.3, "config_json": "{}",
    })
    return scenarios


def scenario_to_args(s: dict) -> list[str]:
    return [
        "--tag", s["tag"],
        "--disturbance-preset", s["disturbance_preset"],
        "--offset-x", f"{s['offset']}",
        *(["--nonidealities"] if s.get("nonidealities") else []),
        "--platform-vx", f"{s.get('platform_vx', 0.0)}",
        "--config-json", s.get("config_json", "{}"),
    ]


def main() -> int:
    p = argparse.ArgumentParser(description="Run the batch scenario matrix.")
    p.add_argument("--repeat", type=int, default=10)
    p.add_argument("--venv-python", default=str(Path.home() / ".venv-uav" / "bin" / "python"))
    p.add_argument("--dry-run", action="store_true", help="print scenario list only")
    args = p.parse_args()

    scenarios = build_matrix(repeat=args.repeat)
    summary = {"total": len(scenarios), "results": []}
    for i, s in enumerate(scenarios, 1):
        if args.dry_run:
            print(f"[{i}/{len(scenarios)}] {s['tag']}")
            continue
        cmd = [args.venv_python, str(RUN_ONE), *scenario_to_args(s)]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        result = {"tag": s["tag"], "returncode": proc.returncode}
        try:
            meta = json.loads(proc.stdout.strip().splitlines()[-1])
            result["outcome"] = meta.get("outcome")
        except Exception:
            result["outcome"] = "PARSE_FAIL"
            result["stderr"] = proc.stderr[-500:]
        summary["results"].append(result)
        print(f"[{i}/{len(scenarios)}] {s['tag']} -> {result['outcome']}", flush=True)
        if proc.returncode != 0:
            print(f"  stderr: {proc.stderr[-300:]}", flush=True)

    out = PROJECT_ROOT / "data" / "batch" / "batch_summary.json"
    if args.dry_run:
        print("dry-run: no batch_summary.json written")
        return 0
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    n_ok = sum(1 for r in summary["results"] if r.get("outcome") == "LANDED")
    print(f"SUMMARY: {n_ok}/{len(scenarios)} LANDED -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
