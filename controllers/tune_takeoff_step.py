#!/usr/bin/env python3
"""Run the Gazebo PID plugin from the command line.

This is a repeatable smoke-test helper for height and attitude-loop tuning in
the static-water Gazebo world. It uses the same plugin-control interface as the
browser dashboard and prints CSV samples for quick stability checks.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dashboard.server import GazeboPluginController, sample_state  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a static-water takeoff PID tuning step.")
    parser.add_argument("--partition", default=os.environ.get("GZ_PARTITION", "coaxial_uav_static_water"))
    parser.add_argument("--world", default="static_water_takeoff")
    parser.add_argument("--model", default="coaxial_uav")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--sample-period", type=float, default=0.5)
    parser.add_argument("--target-z", type=float, default=0.8)
    parser.add_argument("--target-roll", type=float, default=0.0)
    parser.add_argument("--target-pitch", type=float, default=0.0)
    parser.add_argument("--target-yaw", type=float, default=0.0)
    parser.add_argument("--hover-omega", type=float, default=136.362)
    parser.add_argument("--max-omega", type=float, default=150.0)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--setpoint-rate-limit", type=float, default=0.75)
    parser.add_argument("--height-kp", type=float, default=45.0)
    parser.add_argument("--height-ki", type=float, default=0.0)
    parser.add_argument("--height-kd", type=float, default=35.0)
    parser.add_argument("--height-limit", type=float, default=30.0)
    parser.add_argument("--att-kp", type=float, help="legacy override for both roll and pitch Kp")
    parser.add_argument("--att-kd", type=float, help="legacy override for both roll and pitch Kd")
    parser.add_argument("--att-limit", type=float, help="legacy override for both roll and pitch limits")
    parser.add_argument("--roll-kp", type=float, default=193.3)
    parser.add_argument("--roll-kd", type=float, default=8.61)
    parser.add_argument("--roll-limit", type=float, default=2.5)
    parser.add_argument("--pitch-kp", type=float, default=351.3)
    parser.add_argument("--pitch-kd", type=float, default=15.65)
    parser.add_argument("--pitch-limit", type=float, default=2.7)
    parser.add_argument("--yaw-kp", type=float, default=296.1)
    parser.add_argument("--yaw-kd", type=float, default=13.19)
    parser.add_argument("--yaw-limit", type=float, default=0.7)
    parser.add_argument("--yaw-large-signal-kp", type=float, default=20.0)
    parser.add_argument("--yaw-large-signal-kd", type=float, default=3.0)
    parser.add_argument("--yaw-schedule-start", type=float, default=0.02)
    parser.add_argument("--yaw-schedule-end", type=float, default=0.08)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    roll_kp = args.roll_kp if args.att_kp is None else args.att_kp
    pitch_kp = args.pitch_kp if args.att_kp is None else args.att_kp
    roll_kd = args.roll_kd if args.att_kd is None else args.att_kd
    pitch_kd = args.pitch_kd if args.att_kd is None else args.att_kd
    roll_limit = args.roll_limit if args.att_limit is None else args.att_limit
    pitch_limit = args.pitch_limit if args.att_limit is None else args.att_limit
    controller = GazeboPluginController(args.partition, args.world, args.model)
    controller.update_config(
        {
            "target_z_m": args.target_z,
            "target_roll_rad": args.target_roll,
            "target_pitch_rad": args.target_pitch,
            "target_yaw_rad": args.target_yaw,
            "hover_omega_rad_s": args.hover_omega,
            "max_omega_rad_s": args.max_omega,
            "attitude_setpoint_rate_limit_rad_s": args.setpoint_rate_limit,
            "yaw_large_signal_kp": args.yaw_large_signal_kp,
            "yaw_large_signal_kd": args.yaw_large_signal_kd,
            "yaw_schedule_start_rad": args.yaw_schedule_start,
            "yaw_schedule_end_rad": args.yaw_schedule_end,
            "rate_hz": args.rate,
            "height": {
                "kp": args.height_kp,
                "ki": args.height_ki,
                "kd": args.height_kd,
                "limit": args.height_limit,
            },
            "roll": {"kp": roll_kp, "ki": 0.0, "kd": roll_kd, "limit": roll_limit},
            "pitch": {"kp": pitch_kp, "ki": 0.0, "kd": pitch_kd, "limit": pitch_limit},
            "yaw": {"kp": args.yaw_kp, "ki": 0.0, "kd": args.yaw_kd, "limit": args.yaw_limit},
        }
    )
    print("config=" + json.dumps(controller.snapshot()["config"], separators=(",", ":")), flush=True)
    print(
        "time_s,ok,z_m,roll_rad,pitch_rad,yaw_rad,upper_rad_s,lower_rad_s,sim_time_s,rtf",
        flush=True,
    )
    start = time.monotonic()
    controller.start()
    try:
        while time.monotonic() - start < args.duration:
            elapsed = time.monotonic() - start
            state = sample_state(args.partition, args.world, args.model)
            print(
                "{:.3f},{},{:.4f},{:.6f},{:.6f},{:.6f},{:.3f},{:.3f},{:.3f},{:.3f}".format(
                    elapsed,
                    state.get("ok", False),
                    float(state["position"]["z"]),  # type: ignore[index]
                    float(state["attitude"]["roll_rad"]),  # type: ignore[index]
                    float(state["attitude"]["pitch_rad"]),  # type: ignore[index]
                    float(state["attitude"]["yaw_rad"]),  # type: ignore[index]
                    float(state["motors"].get("upper_rad_s", 0.0)),  # type: ignore[union-attr]
                    float(state["motors"].get("lower_rad_s", 0.0)),  # type: ignore[union-attr]
                    float(state["stats"]["sim_time_s"]),  # type: ignore[index]
                    float(state["stats"]["real_time_factor"]),  # type: ignore[index]
                ),
                flush=True,
            )
            time.sleep(max(0.1, args.sample_period))
    finally:
        controller.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
