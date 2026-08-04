#!/usr/bin/env python3
"""Run a single takeoff→hover→landing flight and record scenario CSV + meta.json."""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dashboard.server import (  # noqa: E402
    GazeboPluginController,
    read_plugin_status,
    sample_state,
)

BATCH_DIR = PROJECT_ROOT / "data" / "batch"
INITIAL_Z_M = 0.34
# 高度环为纯 PD（ki=0），悬停存在约 0.05m 稳态偏差；
# 0.05 的判定带过于苛刻会导致 TAKEOFF_TIMEOUT，放宽到 0.08。
STABILIZED_EPS_M = 0.08
STABILIZED_COUNT = 5
LANDED_STATES = {"LANDED", "ABORTED"}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run one takeoff→hover→landing flight.")
    p.add_argument("--partition", default=os.environ.get("GZ_PARTITION", "coaxial_uav_static_water"))
    p.add_argument("--world", default="static_water_takeoff")
    p.add_argument("--model", default="coaxial_uav")
    p.add_argument("--tag", required=True, help="scenario tag, e.g. dist_strong")
    p.add_argument("--out-dir", default=str(BATCH_DIR))
    p.add_argument("--target-x", type=float, default=0.0)
    p.add_argument("--target-y", type=float, default=0.0)
    p.add_argument("--target-z", type=float, default=0.8)
    p.add_argument("--offset-x", type=float, default=0.0, help="initial horizontal offset (m)")
    p.add_argument("--offset-y", type=float, default=0.0)
    p.add_argument("--disturbance-preset", choices=["off", "calm", "mild", "strong", "asymmetric"], default="off")
    p.add_argument("--nonidealities", action="store_true")
    p.add_argument("--platform-vx", type=float, default=0.0, help="moving platform x speed (m/s)")
    p.add_argument("--config-json", default="{}", help="extra nested config overrides, e.g. '{\"height\":{\"kp\":60}}'")
    p.add_argument("--stabilize-timeout", type=float, default=30.0)
    p.add_argument("--landing-timeout", type=float, default=40.0)
    p.add_argument("--settle-delay", type=float, default=3.0, help="seconds to settle on water after reset")
    p.add_argument("--sample-period", type=float, default=0.05)
    p.add_argument("--seed", type=int, default=20260804)
    return p.parse_args()


def build_config(args) -> dict:
    cfg = {
        "target_x_m": args.target_x,
        "target_y_m": args.target_y,
        "target_z_m": args.target_z,
        "disturbance": {
            "enabled": args.disturbance_preset != "off",
            "preset": args.disturbance_preset,
            "seed": getattr(args, "seed", 20260804),
        },
        "nonidealities": {"enabled": bool(args.nonidealities)},
    }
    if abs(args.platform_vx) > 1e-9:
        cfg["moving_target_enabled"] = True
        cfg["target_vx_m_s"] = args.platform_vx
        cfg["target_vy_m_s"] = 0.0
    extra = json.loads(args.config_json) if args.config_json.strip() else {}
    return deep_merge(cfg, extra)


def deep_merge(base: dict, extra: dict) -> dict:
    out = dict(base)
    for k, v in extra.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def record_takeoff(state: dict, prev: dict, t: float, acc: dict) -> dict:
    z = float(state["z"])
    prev_z = float(prev["z"])
    if acc.get("t_liftoff_s") is None and z > prev_z + 0.05 and z > 0.10:
        acc["t_liftoff_s"] = t
    acc["max_abs_roll_rad"] = max(acc.get("max_abs_roll_rad", 0.0), abs(float(state["roll"])))
    acc["max_abs_pitch_rad"] = max(acc.get("max_abs_pitch_rad", 0.0), abs(float(state["pitch"])))
    acc["max_z_m"] = max(acc.get("max_z_m", -1e9), z)
    acc["last_z_m"] = z
    return acc


def takeoff_summary(acc: dict) -> dict:
    target = float(acc.get("target_z_m", 0.8))
    return {
        "t_liftoff_s": acc.get("t_liftoff_s"),
        "max_abs_roll_deg": round(acc.get("max_abs_roll_rad", 0.0) * 180.0 / 3.141592653589793, 4),
        "max_abs_pitch_deg": round(acc.get("max_abs_pitch_rad", 0.0) * 180.0 / 3.141592653589793, 4),
        "overshoot_m": round(max(0.0, acc.get("max_z_m", 0.0) - target), 4),
        "stabilize_time_s": acc.get("stabilize_s"),
    }


def landing_row(st: dict, status: dict | None, t: float) -> dict:
    att = st.get("attitude", {})
    motors = st.get("motors", {})
    stats = st.get("stats", {})
    status = status or {}
    return {
        "t_s": round(t, 3),
        "sim_time_s": float(stats.get("sim_time_s", 0.0)),
        "state": str(status.get("landing_state", "UNKNOWN")),
        "z_m": float(st["position"]["z"]),
        "x_m": float(st["position"].get("x", 0.0)),
        "y_m": float(st["position"].get("y", 0.0)),
        "roll_deg": round(float(att.get("roll_rad", 0.0)) * 57.29578, 4),
        "pitch_deg": round(float(att.get("pitch_rad", 0.0)) * 57.29578, 4),
        "yaw_deg": round(float(att.get("yaw_rad", 0.0)) * 57.29578, 4),
        "upper_rad_s": float(motors.get("upper_rad_s", 0.0)),
        "lower_rad_s": float(motors.get("lower_rad_s", 0.0)),
        "horizontal_error_m": float(status.get("landing_horizontal_error_m", -1.0)),
        "touchdown_vz_m_s": float(status.get("landing_touchdown_vz_m_s", 0.0)),
        "abort_reason": str(status.get("landing_abort_reason", "")),
        "disturbance_active": bool(status.get("disturbance_active", False)),
        "buoyancy_n": float(status.get("buoyancy_compensation_n", 0.0)),
        "slamming_force_n": float(status.get("slamming_force_n", 0.0)),
    }


def wait_stabilized(controller, args, target_z) -> bool:
    ok = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.stabilize_timeout:
        st = sample_state(args.partition, args.world, args.model)
        z = float(st["position"]["z"])
        if abs(z - target_z) < STABILIZED_EPS_M:
            ok += 1
            if ok >= STABILIZED_COUNT:
                return True
        else:
            ok = 0
        time.sleep(args.sample_period)
    return False


def monitor_landing(controller, args) -> list[dict]:
    rows: list[dict] = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.landing_timeout:
        st = sample_state(args.partition, args.world, args.model)
        status = read_plugin_status(args.partition, args.model)
        row = landing_row(st, status, t=time.monotonic() - t0)
        rows.append(row)
        if row["state"] in LANDED_STATES:
            break
        time.sleep(args.sample_period)
    return rows


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out_dir) / args.tag
    out_dir.mkdir(parents=True, exist_ok=True)

    from scripts.reset_pose import reset_pose  # 延迟 import，避免循环

    controller = GazeboPluginController(args.partition, args.world, args.model)
    controller.update_config(build_config(args), persist=False)

    # 1) 位姿重置到 (target + offset)，落水稳定
    rp = reset_pose(args.partition, args.world, args.model,
                    x=args.target_x + args.offset_x,
                    y=args.target_y + args.offset_y,
                    z=INITIAL_Z_M)
    if not rp["ok"]:
        print(json.dumps({"outcome": "RESET_FAILED", "message": rp["message"]}))
        controller.close()
        return 2
    time.sleep(args.settle_delay)

    # 2) 起飞：start → 采样起飞指标 → 等稳定
    takeoff_acc = {"target_z_m": args.target_z, "t_liftoff_s": None}
    prev = {"z": float(sample_state(args.partition, args.world, args.model)["position"]["z"])}
    controller.start()
    t0 = time.monotonic()
    stabilized = False
    while time.monotonic() - t0 < args.stabilize_timeout:
        st = sample_state(args.partition, args.world, args.model)
        z = float(st["position"]["z"])
        att = st.get("attitude", {})
        record_takeoff({"z": z, "roll": att.get("roll_rad", 0.0), "pitch": att.get("pitch_rad", 0.0)},
                       prev, t=time.monotonic() - t0, acc=takeoff_acc)
        prev = {"z": z, "roll": att.get("roll_rad", 0.0), "pitch": att.get("pitch_rad", 0.0)}
        if abs(z - args.target_z) < STABILIZED_EPS_M:
            stabilized = True
            break
        time.sleep(args.sample_period)
    takeoff_acc["stabilize_s"] = round(time.monotonic() - t0, 3)
    if not stabilized:
        controller.stop()
        print(json.dumps({"outcome": "TAKEOFF_TIMEOUT"}))
        controller.close()
        return 3

    # 3) 降落：start_landing → 监测到 LANDED/ABORTED/超时
    controller.start_landing()
    rows = monitor_landing(controller, args)
    controller.stop()
    last = rows[-1] if rows else {}
    outcome = "TIMEOUT"
    if rows and last["state"] == "LANDED":
        outcome = "LANDED"
    elif rows and last["state"] == "ABORTED":
        outcome = "ABORTED"

    # 4) 落盘
    fields = ["t_s", "sim_time_s", "state", "z_m", "x_m", "y_m", "roll_deg",
              "pitch_deg", "yaw_deg", "upper_rad_s", "lower_rad_s",
              "horizontal_error_m", "touchdown_vz_m_s", "abort_reason",
              "disturbance_active", "buoyancy_n", "slamming_force_n"]
    csv_path = out_dir / "samples.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)

    takeoff = takeoff_summary(takeoff_acc)
    meta = {
        "tag": args.tag,
        "scenario": {
            "disturbance_preset": args.disturbance_preset,
            "nonidealities": bool(args.nonidealities),
            "offset_x_m": args.offset_x,
            "offset_y_m": args.offset_y,
            "target_z_m": args.target_z,
            "platform_vx_m_s": args.platform_vx,
            "config_json": args.config_json,
            "seed": args.seed,
        },
        "outcome": outcome,
        "takeoff": takeoff,
        "landing": {
            "final_horizontal_error_m": float(last.get("horizontal_error_m", -1.0)),
            "touchdown_vz_m_s": float(last.get("touchdown_vz_m_s", 0.0)),
            "max_abs_roll_deg": round(max((abs(float(r["roll_deg"])) for r in rows), default=0.0), 4),
            "max_abs_pitch_deg": round(max((abs(float(r["pitch_deg"])) for r in rows), default=0.0), 4),
            "duration_s": round(rows[-1]["t_s"] - rows[0]["t_s"], 3) if len(rows) > 1 else 0.0,
            "abort_reason": str(last.get("abort_reason", "")),
        },
        "samples_csv": str(csv_path.relative_to(PROJECT_ROOT)),
    }
    meta_path = out_dir / "meta.json"
    meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(meta))
    controller.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
