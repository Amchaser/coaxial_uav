#!/usr/bin/env python3
"""Run a single takeoff→hover→landing flight and record scenario CSV + meta.json."""

from __future__ import annotations

import argparse
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
STABILIZED_EPS_M = 0.05
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
