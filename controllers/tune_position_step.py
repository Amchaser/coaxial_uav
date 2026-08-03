#!/usr/bin/env python3
"""Run a temporary high-rate horizontal position step test."""

from __future__ import annotations

import argparse
import json
import sys
import time
from copy import deepcopy
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dashboard.server import (  # noqa: E402
    GazeboPluginController,
    PerformanceTestRunner,
    load_tuning_config,
    save_tuning_config,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--partition", default="coaxial_uav_position_tune")
    parser.add_argument("--axis", choices=("x", "y"), default="x")
    parser.add_argument("--kp", type=float, default=2.5)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=0.95)
    parser.add_argument("--velocity-limit", type=float, default=2.5)
    parser.add_argument("--accel-limit", type=float, default=2.2)
    parser.add_argument("--velocity-kp", type=float, default=2.8)
    parser.add_argument("--velocity-ki", type=float, default=0.15)
    parser.add_argument("--baseline", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    original = load_tuning_config()
    config = deepcopy(original)
    loop_key = f"position_{args.axis}"
    config["target_z_m"] = 0.8
    config["target_roll_rad"] = 0.0
    config["target_pitch_rad"] = 0.0
    config["target_yaw_rad"] = 0.0
    config["target_vx_m_s"] = 0.0
    config["target_vy_m_s"] = 0.0
    config["velocity_control_enabled"] = True
    config["position_control_enabled"] = True
    config["position_velocity_limit_m_s"] = args.velocity_limit
    config["velocity_accel_limit_m_s2"] = args.accel_limit
    for velocity_key in ("velocity_x", "velocity_y"):
        config[velocity_key]["kp"] = args.velocity_kp
        config[velocity_key]["ki"] = args.velocity_ki
        config[velocity_key]["limit"] = args.accel_limit
    config[loop_key]["kp"] = args.kp
    config[loop_key]["ki"] = args.ki
    config[loop_key]["kd"] = args.kd
    config[loop_key]["limit"] = args.velocity_limit
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
                "baseline_s": args.baseline,
                "duration_s": args.duration,
            },
        })
        deadline = time.monotonic() + args.timeout
        while runner.snapshot().get("running") and time.monotonic() < deadline:
            time.sleep(0.1)
        result = runner.snapshot()
        print(json.dumps({
            "mode": result.get("mode"),
            "message": result.get("message"),
            "axis": result.get("axis"),
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
