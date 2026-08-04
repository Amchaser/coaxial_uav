#!/usr/bin/env python3
"""Run a single takeoff→hover→landing flight and record scenario CSV + meta.json."""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
import traceback
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dashboard.server import (  # noqa: E402
    GazeboPluginController,
    StreamingStatusReader,
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
    p.add_argument("--sample-period", type=float, default=0.05,
                   help="upper bound on landing CSV row spacing (sim seconds); achieved "
                        "cadence is higher because samples come from a single persistent "
                        "control/status stream instead of one gz subprocess per tick")
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
        # 移动平台由 landing.* 键配置（_publish_config 只发布这些 landing_ 前缀键）。
        # 静态基线显式置 False，避免从持久化的 tuning_config 继承旧的移动平台配置。
        "landing": {
            "moving_target_enabled": abs(args.platform_vx) > 1e-9,
            "target_vx_m_s": args.platform_vx,
            "target_vy_m_s": 0.0,
        },
    }
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
        "hover_error_m": acc.get("hover_error_m"),
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
        "target_x_m": float(status.get("landing_target_x_m", 0.0)),
        "target_y_m": float(status.get("landing_target_y_m", 0.0)),
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


def landing_row_from_status(status: dict, t: float) -> dict:
    """Build a landing sample row from a single /<model>/control/status snapshot.

    The control/status JSON already carries pose, attitude, motors, landing state
    and errors, so a full flight phase can be sampled from ONE persistent topic
    reader instead of spawning ~5 gz subprocesses per tick.
    """
    return {
        "t_s": round(t, 3),
        "sim_time_s": float(status.get("sim_time_s", 0.0)),
        "state": str(status.get("landing_state", "UNKNOWN")),
        "z_m": float(status.get("z_m", 0.0)),
        "x_m": float(status.get("world_x_m", 0.0)),
        "y_m": float(status.get("world_y_m", 0.0)),
        "target_x_m": float(status.get("landing_target_x_m", 0.0)),
        "target_y_m": float(status.get("landing_target_y_m", 0.0)),
        "roll_deg": round(float(status.get("roll_rad", 0.0)) * 57.29578, 4),
        "pitch_deg": round(float(status.get("pitch_rad", 0.0)) * 57.29578, 4),
        "yaw_deg": round(float(status.get("yaw_rad", 0.0)) * 57.29578, 4),
        "upper_rad_s": float(status.get("upper_motor_rad_s", 0.0)),
        "lower_rad_s": float(status.get("lower_motor_rad_s", 0.0)),
        "horizontal_error_m": float(status.get("landing_horizontal_error_m", -1.0)),
        "touchdown_vz_m_s": float(status.get("landing_touchdown_vz_m_s", 0.0)),
        "abort_reason": str(status.get("landing_abort_reason", "")),
        "disturbance_active": bool(status.get("disturbance_active", False)),
        "buoyancy_n": float(status.get("buoyancy_compensation_n", 0.0)),
        "slamming_force_n": float(status.get("slamming_force_n", 0.0)),
    }


def _control_env(partition: str) -> dict:
    env = os.environ.copy()
    env["GZ_PARTITION"] = partition
    return env


def _scenario_dict(args) -> dict:
    return {
        "disturbance_preset": args.disturbance_preset,
        "nonidealities": bool(args.nonidealities),
        "offset_x_m": args.offset_x,
        "offset_y_m": args.offset_y,
        "target_z_m": args.target_z,
        "platform_vx_m_s": args.platform_vx,
        "config_json": args.config_json,
        "seed": args.seed,
    }


def _landing_snapshot(cfg: dict) -> dict:
    """Snapshot of the effective landing config actually sent to the plugins."""
    landing = cfg.get("landing", {}) if isinstance(cfg, dict) else {}
    return {
        "moving_target_enabled": bool(landing.get("moving_target_enabled", False)),
        "target_vx_m_s": float(landing.get("target_vx_m_s", 0.0)),
        "target_vy_m_s": float(landing.get("target_vy_m_s", 0.0)),
    }


def write_failure_meta(out_dir: Path, args, landing_snapshot: dict,
                       outcome: str, reason: str, takeoff: dict | None = None) -> None:
    """Write a meta.json for failed groups so analyze.py still counts them (n, 0 landed)."""
    meta = {
        "tag": args.tag,
        "scenario": _scenario_dict(args),
        "landing_config": landing_snapshot,
        "outcome": outcome,
        "reason": reason,
        "takeoff": takeoff or {},
    }
    meta_path = out_dir / "meta.json"
    meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")


def monitor_landing(reader: StreamingStatusReader, args) -> list[dict]:
    rows: list[dict] = []
    start_sim: float | None = None
    last_sim: float | None = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.landing_timeout:
        try:
            status = reader.get(timeout=0.5)
        except RuntimeError:
            break  # stream died / topic unavailable
        sim = float(status.get("sim_time_s", 0.0))
        if start_sim is None:
            start_sim = sim
        if sim - start_sim >= args.landing_timeout:
            break
        # decimate: at most one row per sample_period of *sim* time (upper bound),
        # but no subprocess is spawned, so the achieved cadence stays far below 1s.
        if last_sim is not None and sim - last_sim < args.sample_period:
            continue
        last_sim = sim
        row = landing_row_from_status(status, t=sim - start_sim)
        rows.append(row)
        if row["state"] in LANDED_STATES:
            break
    return rows


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out_dir) / args.tag
    out_dir.mkdir(parents=True, exist_ok=True)

    from scripts.reset_pose import reset_pose  # 延迟 import，避免循环

    controller = GazeboPluginController(args.partition, args.world, args.model)
    reader: StreamingStatusReader | None = None
    landing_snapshot: dict = {}
    try:
        cfg = build_config(args)
        controller.update_config(cfg, persist=False)
        landing_snapshot = _landing_snapshot(cfg)

        # 1) 位姿重置到 (target + offset)，落水稳定
        rp = reset_pose(args.partition, args.world, args.model,
                        x=args.target_x + args.offset_x,
                        y=args.target_y + args.offset_y,
                        z=INITIAL_Z_M)
        if not rp["ok"]:
            print(json.dumps({"outcome": "RESET_FAILED", "message": rp["message"]}))
            write_failure_meta(out_dir, args, landing_snapshot, "RESET_FAILED",
                               f"reset_pose: {rp['message']}")
            return 2
        time.sleep(args.settle_delay)

        # 2) 起飞：单个 /control/status 流（不再每 tick spawn 子进程）。
        #    /control/status 自带 z/roll/pitch/motors，起飞指标全速累积；
        #    稳定判定按 sample_period 节奏做 N 次连续带内（与 STABILIZED_COUNT 一致）。
        reader = StreamingStatusReader(f"/{args.model}/control/status", _control_env(args.partition))
        reader.start()
        # 起飞前先取一个静止样本作为 liftoff 检测基准（prev），否则 controller.start()
        # 后的第一帧可能已是爬升高度，t_liftoff_s 将无法触发。
        rest = None
        try:
            rest = reader.get(timeout=2.0)
        except RuntimeError:
            pass
        prev = None
        start_sim: float | None = None
        if rest is not None:
            prev = {"z": float(rest.get("z_m", 0.0)),
                    "roll": float(rest.get("roll_rad", 0.0)),
                    "pitch": float(rest.get("pitch_rad", 0.0))}
            start_sim = float(rest.get("sim_time_s", 0.0))
        controller.start()
        takeoff_acc = {"target_z_m": args.target_z, "t_liftoff_s": None}
        sim: float | None = None
        last_check_sim: float | None = None
        ok_count = 0
        stab_zs: list[float] = []
        stabilized = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.stabilize_timeout:
            try:
                status = reader.get(timeout=0.5)
            except RuntimeError:
                break
            sim = float(status.get("sim_time_s", 0.0))
            if start_sim is None:
                start_sim = sim
            z = float(status.get("z_m", 0.0))
            roll = float(status.get("roll_rad", 0.0))
            pitch = float(status.get("pitch_rad", 0.0))
            if prev is None:
                prev = {"z": z, "roll": roll, "pitch": pitch}
                continue
            # prev 固定为起飞前静止基准：密集采样下相邻帧 z 差远小于 0.05，
            # 若逐帧更新 prev，t_liftoff_s 将永远无法触发。
            record_takeoff({"z": z, "roll": roll, "pitch": pitch}, prev,
                           t=sim - start_sim, acc=takeoff_acc)
            if last_check_sim is None or sim - last_check_sim >= args.sample_period:
                last_check_sim = sim
                if abs(z - args.target_z) < STABILIZED_EPS_M:
                    ok_count += 1
                    stab_zs.append(z)
                    if ok_count >= STABILIZED_COUNT:
                        stabilized = True
                        break
                else:
                    ok_count = 0
                    stab_zs = []
            if sim - start_sim >= args.stabilize_timeout:
                break
        if sim is not None and start_sim is not None:
            takeoff_acc["stabilize_s"] = round(sim - start_sim, 3)
        else:
            takeoff_acc["stabilize_s"] = round(time.monotonic() - t0, 3)
        if stabilized and stab_zs:
            # 稳态高度取稳定窗口 z 均值，体现 ~7cm 的下垂（overshoot_m 只能捕获超调）。
            takeoff_acc["hover_error_m"] = round(sum(stab_zs) / len(stab_zs) - args.target_z, 4)
        takeoff = takeoff_summary(takeoff_acc)
        if not stabilized:
            controller.stop()
            print(json.dumps({"outcome": "TAKEOFF_TIMEOUT"}))
            write_failure_meta(out_dir, args, landing_snapshot, "TAKEOFF_TIMEOUT",
                               f"not stabilized within {args.stabilize_timeout}s "
                               f"(target_z={args.target_z}, eps={STABILIZED_EPS_M})",
                               takeoff=takeoff)
            return 3

        # 3) 降落：start_landing → 监测到 LANDED/ABORTED/超时
        controller.start_landing()
        rows = monitor_landing(reader, args)
        controller.stop()
        last = rows[-1] if rows else {}
        outcome = "TIMEOUT"
        if rows and last["state"] == "LANDED":
            outcome = "LANDED"
        elif rows and last["state"] == "ABORTED":
            outcome = "ABORTED"

        # 4) 落盘
        fields = ["t_s", "sim_time_s", "state", "z_m", "x_m", "y_m", "target_x_m",
                  "target_y_m", "roll_deg",
                  "pitch_deg", "yaw_deg", "upper_rad_s", "lower_rad_s",
                  "horizontal_error_m", "touchdown_vz_m_s", "abort_reason",
                  "disturbance_active", "buoyancy_n", "slamming_force_n"]
        csv_path = out_dir / "samples.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)

        meta = {
            "tag": args.tag,
            "scenario": _scenario_dict(args),
            "landing_config": landing_snapshot,
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
        return 0
    except Exception:
        traceback.print_exc()
        try:
            write_failure_meta(out_dir, args, landing_snapshot, "ERROR",
                               "unhandled exception; see stderr traceback")
        except Exception:
            pass
        raise
    finally:
        if reader is not None:
            reader.close()
        controller.close()


if __name__ == "__main__":
    sys.exit(main())
