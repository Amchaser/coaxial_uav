#!/usr/bin/env python3
"""Reset the coaxial UAV model pose via the Garden UserCommands set_pose service.

Usage:
    python3 scripts/reset_pose.py [--x X] [--y Y] [--z Z] [--yaw RAD] [--model coaxial_uav]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys


def set_pose_service_request(name: str, x: float, y: float, z: float,
                             yaw: float = 0.0) -> dict:
    """构建 gz.msgs.Pose 的 dict 表示。

    gz.msgs.Pose 为扁平结构：name/position/orientation 直接挂在消息顶层，
    没有嵌套的 pose 字段（见 /usr/include/gz/msgs9/gz/msgs/pose.proto）。
    朝向用单位四元数表示（z=sin(yaw/2), w=cos(yaw/2)）。
    """
    return {
        "name": name,
        "position": {"x": x, "y": y, "z": z},
        "orientation": {
            "x": 0.0, "y": 0.0,
            "z": math.sin(yaw / 2.0),
            "w": math.cos(yaw / 2.0),
        },
    }


def format_pose_request_text(req: dict) -> str:
    """把 gz.msgs.Pose 的 dict 表示转成 protobuf 文本格式。

    Garden 的 ``gz service --req`` 只接受 protobuf 文本格式，不接受 JSON
    （传 JSON 会报 "Expected identifier, got: {" 并静默变成空请求）。
    """
    p = req["position"]
    o = req["orientation"]
    return (
        'name: "{name}"\n'
        "position: {{ x: {px} y: {py} z: {pz} }}\n"
        "orientation: {{ x: {ox} y: {oy} z: {oz} w: {ow} }}"
    ).format(
        name=req["name"],
        px=p["x"], py=p["y"], pz=p["z"],
        ox=o["x"], oy=o["y"], oz=o["z"], ow=o["w"],
    )


def _parse_boolean_reply(stdout: object) -> bool | None:
    """Parse ``data: true`` / ``data: false`` from the gz.msgs.Boolean reply.

    Returns True/False when the field is present, None when it is absent (in
    which case the caller falls back to the process returncode).
    """
    if not isinstance(stdout, str):
        return None
    match = re.search(r"\bdata\s*:\s*(true|false)\b", stdout)
    if match is None:
        return None
    return match.group(1) == "true"


def reset_pose(partition: str, world: str, model: str,
               x: float, y: float, z: float, yaw: float = 0.0,
               timeout_ms: int = 5000) -> dict:
    env = dict(os.environ)
    env["GZ_PARTITION"] = partition
    req = format_pose_request_text(set_pose_service_request(model, x, y, z, yaw))
    cmd = [
        "gz", "service", "-s", f"/world/{world}/set_pose",
        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
        "--req", req, "--timeout", str(timeout_ms),
    ]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, env=env,
                              timeout=(timeout_ms // 1000) + 5)
    except subprocess.TimeoutExpired:
        return {"ok": False, "message": "set_pose service call timed out"}
    except FileNotFoundError:
        return {"ok": False, "message": "gz binary not found (is Gazebo installed / on PATH?)"}
    if proc.returncode != 0:
        return {"ok": False, "message": proc.stderr.strip() or proc.stdout.strip()}
    # 解析 Boolean 回复本体：data:false 意味着服务端拒绝了置位，returncode 仍是 0。
    ok = _parse_boolean_reply(proc.stdout)
    if ok is False:
        return {"ok": False, "message": proc.stdout.strip() or "set_pose replied data: false"}
    return {"ok": True, "message": proc.stdout.strip()}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Reset the coaxial UAV model pose.")
    p.add_argument("--partition", default=os.environ.get("GZ_PARTITION", "coaxial_uav_static_water"))
    p.add_argument("--world", default="static_water_takeoff")
    p.add_argument("--model", default="coaxial_uav")
    p.add_argument("--x", type=float, default=0.0)
    p.add_argument("--y", type=float, default=0.0)
    p.add_argument("--z", type=float, default=0.34)
    p.add_argument("--yaw", type=float, default=0.0)
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    result = reset_pose(args.partition, args.world, args.model,
                        args.x, args.y, args.z, args.yaw)
    print(json.dumps(result))
    sys.exit(0 if result["ok"] else 1)
