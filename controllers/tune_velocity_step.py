#!/usr/bin/env python3
"""Run a temporary high-rate horizontal velocity step test."""

from __future__ import annotations

import argparse
import json
import sys
import time
from copy import deepcopy
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dashboard.server import (
    GazeboPluginController,
    PerformanceTestRunner,
    load_tuning_config,
    save_tuning_config,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--partition", default="coaxial_uav_velocity_tune")
    parser.add_argument("--axis", choices=("vx", "vy"), default="vx")
    parser.add_argument("--step", type=float, default=1.0)
    parser.add_argument("--kp", type=float, default=2.8)
    parser.add_argument("--ki", type=float, default=0.15)
    parser.add_argument("--accel-limit", type=float, default=2.2)
    parser.add_argument("--tilt-limit-deg", type=float, default=15.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument(
        "--stale-position-mode",
        action="store_true",
        help="inject a conflicting position-mode flag to regression-test axis mode selection",
    )
    parser.add_argument("--baseline", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    original = load_tuning_config()
    config = deepcopy(original)
    loop_key = f"velocity_{args.axis[-1]}"
    config["target_z_m"] = 0.8
    config["target_roll_rad"] = 0.0
    config["target_pitch_rad"] = 0.0
    config["target_yaw_rad"] = args.yaw_deg * 3.141592653589793 / 180.0
    config["target_vx_m_s"] = 0.0
    config["target_vy_m_s"] = 0.0
    config["velocity_control_enabled"] = True
    config["position_control_enabled"] = args.stale_position_mode
    config["velocity_tilt_limit_rad"] = args.tilt_limit_deg * 3.141592653589793 / 180.0
    config["velocity_accel_limit_m_s2"] = args.accel_limit
    config[loop_key]["kp"] = args.kp
    config[loop_key]["ki"] = args.ki
    config[loop_key]["limit"] = args.accel_limit
    config["disturbance"] = {"enabled": False, "preset": "off", "seed": 20260726}

    controller = GazeboPluginController(
        args.partition, "static_water_takeoff", "coaxial_uav"
    )
    runner = PerformanceTestRunner(
        controller, args.partition, "static_water_takeoff", "coaxial_uav"
    )
    try:
        runner.start({
            "config": config,
            "test": {
                "axis": args.axis,
                "step": args.step,
                "baseline_s": args.baseline,
                "duration_s": args.duration,
            },
        })
        deadline = time.monotonic() + args.timeout
        while runner.snapshot().get("running") and time.monotonic() < deadline:
            time.sleep(0.1)
        result = runner.snapshot()
        result_config = result.get("config")
        if not isinstance(result_config, dict):
            result_config = {}
        print(json.dumps({
            "mode": result.get("mode"),
            "message": result.get("message"),
            "axis": result.get("axis"),
            "requested_step": result.get("requested_step"),
            "requested_step_unit": result.get("requested_step_unit"),
            "applied_step": result.get("applied_step"),
            "applied_step_unit": result.get("applied_step_unit"),
            "control_mode": {
                "velocity_control_enabled": result_config.get("velocity_control_enabled"),
                "position_control_enabled": result_config.get("position_control_enabled"),
            },
            "metrics": result.get("metrics"),
            "saved_path": result.get("saved_path"),
        }, indent=2))
        return 0 if result.get("mode") == "complete" else 1
    finally:
        try:
            runner.stop()
        finally:
            controller.close()
            save_tuning_config(original)


if __name__ == "__main__":
    raise SystemExit(main())
