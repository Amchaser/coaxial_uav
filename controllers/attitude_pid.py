#!/usr/bin/env python3
"""Small-signal attitude PID controller for Gazebo Garden transport topics."""

from __future__ import annotations

import argparse
import math
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass


@dataclass
class Pose:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    limit: float
    integral_limit: float
    integral: float = 0.0
    previous_error: float | None = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            dt = 1e-3
        self.integral += error * dt
        self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        self.previous_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return clamp(output, -self.limit, self.limit)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quat_to_euler(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def run_cmd(args: list[str], env: dict[str, str], timeout: float = 5.0) -> str:
    completed = subprocess.run(
        args,
        check=True,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=timeout,
    )
    return completed.stdout


def pose_blocks(debug_text: str) -> list[str]:
    blocks: list[str] = []
    in_pose = False
    depth = 0
    current: list[str] = []
    for line in debug_text.splitlines():
        stripped = line.strip()
        if not in_pose and stripped == "pose {":
            in_pose = True
            depth = 1
            current = [line]
            continue
        if not in_pose:
            continue
        current.append(line)
        depth += stripped.count("{")
        depth -= stripped.count("}")
        if depth == 0:
            blocks.append("\n".join(current))
            in_pose = False
    return blocks


def field(block: str, name: str, default: float = 0.0) -> float:
    match = re.search(rf"\b{name}\s*:\s*([-+0-9.eE]+)", block)
    return default if match is None else float(match.group(1))


def read_model_pose(topic: str, model_name: str, env: dict[str, str]) -> Pose:
    output = run_cmd(["gz", "topic", "-e", "-t", topic, "-n", "1"], env)
    for block in pose_blocks(output):
        if f'name: "{model_name}"' not in block:
            continue
        position = re.search(r"position\s*\{(?P<body>.*?)\n\s*\}", block, re.S)
        orientation = re.search(r"orientation\s*\{(?P<body>.*?)\n\s*\}", block, re.S)
        pos_body = "" if position is None else position.group("body")
        ori_body = "" if orientation is None else orientation.group("body")
        return Pose(
            x=field(pos_body, "x"),
            y=field(pos_body, "y"),
            z=field(pos_body, "z"),
            qx=field(ori_body, "x"),
            qy=field(ori_body, "y"),
            qz=field(ori_body, "z"),
            qw=field(ori_body, "w", 1.0),
        )
    raise RuntimeError(f"model pose not found in topic {topic}: {model_name}")


def publish_wrench(topic: str, link_name: str, roll_nm: float, pitch_nm: float, yaw_nm: float, env: dict[str, str]) -> None:
    payload = (
        f"entity: {{name: '{link_name}', type: LINK}}, "
        f"wrench: {{torque: {{x: {roll_nm:.6f}, y: {pitch_nm:.6f}, z: {yaw_nm:.6f}}}}}"
    )
    run_cmd(["gz", "topic", "-t", topic, "-m", "gz.msgs.EntityWrench", "-p", payload], env)


def clear_wrench(topic: str, link_name: str, env: dict[str, str]) -> None:
    payload = f"name: '{link_name}', type: LINK"
    run_cmd(["gz", "topic", "-t", topic, "-m", "gz.msgs.Entity", "-p", payload], env)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read Gazebo attitude and publish limited PID attitude torque.")
    parser.add_argument("--partition", default="coaxial_uav_static_water")
    parser.add_argument("--world", default="static_water_takeoff")
    parser.add_argument("--model", default="coaxial_uav")
    parser.add_argument("--link", default="coaxial_uav::base_link")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--rate", type=float, default=4.0)
    parser.add_argument("--roll-sp", type=float, default=0.0)
    parser.add_argument("--pitch-sp", type=float, default=0.0)
    parser.add_argument("--yaw-sp", type=float, default=0.0)
    parser.add_argument("--kp-roll", type=float, default=0.30)
    parser.add_argument("--ki-roll", type=float, default=0.0)
    parser.add_argument("--kd-roll", type=float, default=0.03)
    parser.add_argument("--kp-pitch", type=float, default=0.30)
    parser.add_argument("--ki-pitch", type=float, default=0.0)
    parser.add_argument("--kd-pitch", type=float, default=0.03)
    parser.add_argument("--kp-yaw", type=float, default=0.10)
    parser.add_argument("--ki-yaw", type=float, default=0.0)
    parser.add_argument("--kd-yaw", type=float, default=0.01)
    parser.add_argument("--torque-limit", type=float, default=0.20)
    parser.add_argument("--yaw-torque-limit", type=float, default=0.05)
    parser.add_argument("--integral-limit", type=float, default=0.20)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    env = os.environ.copy()
    env["GZ_PARTITION"] = args.partition
    pose_topic = f"/world/{args.world}/pose/info"
    wrench_topic = f"/world/{args.world}/wrench/persistent"
    clear_topic = f"/world/{args.world}/wrench/clear"

    roll_pid = PID(args.kp_roll, args.ki_roll, args.kd_roll, args.torque_limit, args.integral_limit)
    pitch_pid = PID(args.kp_pitch, args.ki_pitch, args.kd_pitch, args.torque_limit, args.integral_limit)
    yaw_pid = PID(args.kp_yaw, args.ki_yaw, args.kd_yaw, args.yaw_torque_limit, args.integral_limit)

    stopped = False

    def stop_handler(_signum: int, _frame: object) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    start = time.monotonic()
    previous = start
    period = 1.0 / max(args.rate, 0.1)
    print("time_s,z_m,roll_rad,pitch_rad,yaw_rad,roll_nm,pitch_nm,yaw_nm")
    try:
        while not stopped and time.monotonic() - start < args.duration:
            now = time.monotonic()
            pose = read_model_pose(pose_topic, args.model, env)
            roll, pitch, yaw = quat_to_euler(pose.qx, pose.qy, pose.qz, pose.qw)
            dt = now - previous
            previous = now

            roll_cmd = roll_pid.update(wrap_pi(args.roll_sp - roll), dt)
            pitch_cmd = pitch_pid.update(wrap_pi(args.pitch_sp - pitch), dt)
            yaw_cmd = yaw_pid.update(wrap_pi(args.yaw_sp - yaw), dt)
            publish_wrench(wrench_topic, args.link, roll_cmd, pitch_cmd, yaw_cmd, env)

            print(
                f"{now - start:.3f},{pose.z:.4f},{roll:.6f},{pitch:.6f},{yaw:.6f},"
                f"{roll_cmd:.6f},{pitch_cmd:.6f},{yaw_cmd:.6f}",
                flush=True,
            )
            elapsed = time.monotonic() - now
            time.sleep(max(0.0, period - elapsed))
    finally:
        clear_wrench(clear_topic, args.link, env)
    return 0


if __name__ == "__main__":
    sys.exit(main())
