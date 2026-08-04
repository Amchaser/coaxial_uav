#!/usr/bin/env python3
"""Headless parameter scan (B-mode): launch gz sim -s per combo, run one flight.

Usage:
    python3 scripts/batch_scan.py --grid '{"height.kp":[45.0,60.0],"landing.descent_rate_m_s":[0.35,0.45]}' \
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

# 校验白名单：与 dashboard/server._publish_config 实际发布的键保持一致，
# 避免未知键被静默丢弃导致扫出完全相同的组合（参数扫描最糟的失败模式）。
_PID_AXES = {"height", "roll", "pitch", "yaw"}
_PID_KEYS = {"kp", "ki", "kd", "limit"}
_VELPOS_AXES = {"velocity_x", "velocity_y", "position_x", "position_y"}
_VELPOS_KEYS = {"kp", "ki", "kd", "limit", "integral_limit"}

_TOP_LEVEL_KEYS = {
    "target_z_m", "target_roll_rad", "target_pitch_rad", "target_yaw_rad",
    "velocity_control_enabled", "target_vx_m_s", "target_vy_m_s",
    "velocity_tilt_limit_rad", "velocity_accel_limit_m_s2",
    "position_control_enabled", "target_x_m", "target_y_m",
    "position_velocity_limit_m_s", "max_omega_rad_s",
    "attitude_setpoint_rate_limit_rad_s", "yaw_large_signal_kp",
    "yaw_large_signal_kd", "yaw_schedule_start_rad", "yaw_schedule_end_rad",
}

# _publish_config 发布的 landing_* 扁平键（来自 config["landing"] 的对应子键）。
_LANDING_KEYS = {
    "surface_mode", "platform_top_offset_m", "target_x_m", "target_y_m",
    "target_yaw_rad", "moving_target_enabled", "target_vx_m_s", "target_vy_m_s",
    "target_yaw_rate_rad_s", "target_status_timeout_s", "target_speed_limit_m_s",
    "high_hover_z_m", "approach_speed_m_s", "cruise_speed_m_s",
    "position_tolerance_m", "yaw_tolerance_rad", "descent_rate_m_s",
    "flare_clearance_m", "flare_rate_m_s", "touchdown_max_vz_m_s",
    "contact_confirm_s", "spool_down_s",
    "departure_horizontal_speed_limit_m_s", "departure_clearance_margin_m",
    "near_horizontal_speed_limit_m_s", "moving_target_correction_reserve_m_s",
    "approach_braking_accel_m_s2", "abort_position_error_m",
    "near_max_descent_speed_m_s", "go_around_height_m",
    "departure_stable_time_s", "align_stable_time_s", "hover_stable_time_s",
    "approach_relative_speed_tolerance_m_s", "align_relative_speed_tolerance_m_s",
    "hover_relative_speed_tolerance_m_s",
    "departure_horizontal_speed_tolerance_m_s", "height_tolerance_m",
    "approach_vertical_speed_tolerance_m_s",
    "precision_vertical_speed_tolerance_m_s", "near_overspeed_grace_s",
    "contact_submerged_fraction", "settling_vertical_speed_limit_m_s",
    "settling_time_s", "contact_loss_grace_s",
    "go_around_height_tolerance_m", "go_around_vertical_speed_tolerance_m_s",
    "flare_transition_margin_m", "departure_tilt_limit_rad",
    "approach_tilt_limit_rad", "near_tilt_limit_rad", "warning_tilt_rad",
    "abort_tilt_rad", "approach_abort_tilt_rad", "yaw_rate_tolerance_rad_s",
    "contact_tilt_rate_limit_rad_s", "settling_tilt_rate_limit_rad_s",
    "go_around_tilt_tolerance_rad",
}


def validate_grid(grid: dict) -> list[str]:
    """Return grid paths whose final segment is not a real published config key."""
    unknown: list[str] = []
    for path in grid:
        parts = path.split(".")
        if len(parts) == 1:
            if path not in _TOP_LEVEL_KEYS:
                unknown.append(path)
        elif len(parts) == 2:
            section, key = parts
            if section in _PID_AXES and key in _PID_KEYS:
                continue
            if section in _VELPOS_AXES and key in _VELPOS_KEYS:
                continue
            if section == "landing" and key in _LANDING_KEYS:
                continue
            unknown.append(path)
        else:
            unknown.append(path)
    return unknown


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


def sim_command(world: str) -> list[str]:
    """Build the headless gz sim launch command for a scan combo.

    ``-r`` is required: without it gz sim starts paused and the world never
    advances, so the flight never progresses (see run_static_water.sh which
    always appends ``-r``). ``-s`` runs headless (no GUI).
    """
    return ["gz", "sim", "-r", "-s", world]


def main() -> int:
    p = argparse.ArgumentParser(description="Headless PID/landing parameter scan.")
    p.add_argument("--grid", required=True, help="JSON dict, e.g. '{\"height.kp\":[45.0,60.0]}'")
    p.add_argument("--base-tag", default="dist_strong")
    p.add_argument("--run-one-args", default="")
    p.add_argument("--venv-python", default=str(Path.home() / ".venv-uav" / "bin" / "python"))
    p.add_argument("--flight-timeout", type=float, default=150.0,
                   help="per-combo run_one_flight subprocess timeout")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()

    grid = json.loads(args.grid)
    unknown = validate_grid(grid)
    if unknown:
        print("ERROR: unknown grid keys (silently dropped by _publish_config -> identical combos): "
              f"{', '.join(sorted(unknown))}", file=sys.stderr)
        print("  known examples: height.kp, landing.descent_rate_m_s, "
              "landing.flare_rate_m_s, landing.high_hover_z_m, position_x.kp", file=sys.stderr)
        return 2
    combos = expand_grid(grid)
    summary = {"grid": grid, "combos": len(combos), "results": []}

    if not args.dry_run:
        venv_python = Path(args.venv_python)
        if not venv_python.is_file():
            print(f"ERROR: venv python not found: {venv_python}", file=sys.stderr)
            print("  pass --venv-python /path/to/python or create ~/.venv-uav", file=sys.stderr)
            return 1

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
            sim_command(f"{PROJECT_ROOT}/worlds/static_water_takeoff.sdf"),
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            time.sleep(10)  # 等世界加载
            cmd = [args.venv_python, str(RUN_ONE),
                   "--partition", SCAN_PARTITION, "--tag", full_tag,
                   "--config-json", json.dumps(overrides)]
            if args.run_one_args:
                cmd += args.run_one_args.split()
            try:
                proc = subprocess.run(cmd, capture_output=True, text=True, timeout=args.flight_timeout)
            except subprocess.TimeoutExpired:
                result = {"tag": full_tag, "rc": -1, "outcome": "TIMEOUT_RUN",
                          "reported_outcome": "TIMEOUT_RUN",
                          "stderr": f"exceeded {args.flight_timeout}s flight timeout"}
                summary["results"].append(result)
                print("  -> TIMEOUT_RUN", flush=True)
                continue
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
