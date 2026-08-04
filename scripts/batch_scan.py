#!/usr/bin/env python3
"""Headless parameter scan (B-mode): launch gz sim -s per combo, run one flight.

Usage:
    python3 scripts/batch_scan.py --grid '{"height.kp":[45.0,60.0],"landing.descent_vz":[-0.3,-0.5]}' \
        [--base-tag dist_strong] [--run-one-args '--disturbance-preset strong']
"""

from __future__ import annotations

import argparse
import itertools
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUN_ONE = PROJECT_ROOT / "scripts" / "run_one_flight.py"
SCAN_PARTITION = "coaxial_uav_scan"


def nested_assign(path: str, value: float, config: dict) -> None:
    parts = path.split(".")
    node = config
    for part in parts[:-1]:
        node = node.setdefault(part, {})
    node[parts[-1]] = value


def tag_for(path: str, value: float) -> str:
    key = path.split(".")[-1]
    return f"scan_{key}_{value}"


def expand_grid(grid: dict) -> list[tuple[str, dict]]:
    keys = list(grid.keys())
    value_sets = list(grid.values())
    combos = []
    for values in itertools.product(*value_sets):
        cfg: dict = {}
        parts: list[str] = []
        for k, v in zip(keys, values):
            nested_assign(k, v, cfg)
            parts.append(f"{k.split('.')[-1]}_{v}")
        combos.append(("_".join(["scan"] + parts), cfg))
    return combos


def decide_result(tag: str, returncode: int, stdout: str) -> dict:
    """Decide the recorded result entry for one scanned combo.

    A nonzero exit code is never recorded as success: the outcome is forced to
    "FAILED" even if the flight still emitted a meta JSON line. The meta's own
    outcome (or PARSE_FAIL) is kept visible via ``reported_outcome``.
    """
    try:
        meta = json.loads(stdout.strip().splitlines()[-1])
        reported = meta.get("outcome")
    except Exception:
        reported = "PARSE_FAIL"
    if returncode != 0:
        return {"tag": tag, "rc": returncode, "outcome": "FAILED", "reported_outcome": reported}
    return {"tag": tag, "rc": 0, "outcome": reported}


def main() -> int:
    p = argparse.ArgumentParser(description="Headless PID/landing parameter scan.")
    p.add_argument("--grid", required=True, help="JSON dict, e.g. '{\"height.kp\":[45.0,60.0]}'")
    p.add_argument("--base-tag", default="dist_strong")
    p.add_argument("--run-one-args", default="")
    p.add_argument("--venv-python", default=str(Path.home() / ".venv-uav" / "bin" / "python"))
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()

    grid = json.loads(args.grid)
    combos = expand_grid(grid)
    summary = {"grid": grid, "combos": len(combos), "results": []}

    for i, (tag, overrides) in enumerate(combos, 1):
        full_tag = f"{args.base_tag}_{tag}"
        print(f"[{i}/{len(combos)}] {full_tag} cfg={json.dumps(overrides)}", flush=True)
        if args.dry_run:
            continue
        # 启动 headless 仿真（独立分区）
        env = dict(os.environ)
        env["GZ_PARTITION"] = SCAN_PARTITION
        env["GZ_SIM_RESOURCE_PATH"] = f"{PROJECT_ROOT}/models"
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{PROJECT_ROOT}/build/plugins"
        gz = subprocess.Popen(
            ["gz", "sim", "-s", f"{PROJECT_ROOT}/worlds/static_water_takeoff.sdf"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            time.sleep(10)  # 等世界加载
            cmd = [args.venv_python, str(RUN_ONE),
                   "--partition", SCAN_PARTITION, "--tag", full_tag,
                   "--config-json", json.dumps(overrides)]
            if args.run_one_args:
                cmd += args.run_one_args.split()
            proc = subprocess.run(cmd, capture_output=True, text=True)
            result = decide_result(full_tag, proc.returncode, proc.stdout)
            summary["results"].append(result)
            print(f"  -> {result['outcome']}", flush=True)
        finally:
            gz.terminate()
            try:
                gz.wait(timeout=5)
            except subprocess.TimeoutExpired:
                gz.kill()

    out = PROJECT_ROOT / "data" / "batch" / "scan_summary.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"scan summary -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
