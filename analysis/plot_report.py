#!/usr/bin/env python3
"""Generate report charts from batch landing data (matplotlib)."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _setup_cjk_font() -> None:
    """Best-effort: register a CJK font so Chinese labels render instead of tofu.

    Tries common Windows/WSL and Linux font paths; no-op (charts fall back to
    DejaVu Sans with missing-glyph warnings) when none is available.
    """
    from matplotlib import font_manager

    candidates = (
        "/mnt/c/Windows/Fonts/msyh.ttc",
        "/mnt/c/Windows/Fonts/simhei.ttf",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    )
    for fp in candidates:
        if not Path(fp).is_file():
            continue
        try:
            font_manager.fontManager.addfont(fp)
            family = font_manager.FontProperties(fname=fp).get_name()
            plt.rcParams["font.sans-serif"] = [family, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            return
        except Exception:
            continue


_setup_cjk_font()

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
BATCH_DIR = PROJECT_ROOT / "data" / "batch"
REPORT_DIR = PROJECT_ROOT / "data" / "report"

# 中性图表配色（dataviz 风格）
C_BLUE = "#2563eb"
C_ORANGE = "#ea580c"
C_GREEN = "#16a34a"
C_GRAY = "#94a3b8"


def collect_timeseries(batch_dir: Path) -> dict:
    series = {}
    for d in sorted(batch_dir.iterdir()):
        csv_path = d / "samples.csv"
        if not (d.is_dir() and csv_path.is_file()):
            continue
        rows = list(csv.DictReader(csv_path.open(encoding="utf-8")))
        series[d.name] = {
            "t": [float(r["t_s"]) for r in rows],
            "z": [float(r["z_m"]) for r in rows],
            "roll_deg": [float(r["roll_deg"]) for r in rows],
            "pitch_deg": [float(r["pitch_deg"]) for r in rows],
            "upper": [float(r["upper_rad_s"]) for r in rows],
        }
    return series


def build_landing_points(batch_dir: Path) -> list[dict]:
    pts = []
    for d in sorted(batch_dir.iterdir()):
        csv_path = d / "samples.csv"
        meta_path = d / "meta.json"
        if not (d.is_dir() and csv_path.is_file() and meta_path.is_file()):
            continue
        rows = list(csv.DictReader(csv_path.open(encoding="utf-8")))
        if not rows:
            continue
        last = rows[-1]
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        # 优先用目标相对坐标：x/y 相对降落目标，验收圈原点即目标（世界系不再混用）
        has_target = "target_x_m" in last and "target_y_m" in last
        if has_target:
            rx = float(last["x_m"]) - float(last["target_x_m"])
            ry = float(last["y_m"]) - float(last["target_y_m"])
        else:
            # 旧数据（无目标列）：退化为 horizontal_error_m，y 置 0，仍落在验收圈语义内
            rx = float(last.get("horizontal_error_m", 0.0))
            ry = 0.0
        pt = {
            "tag": d.name,
            "x": rx,
            "y": ry,
            "vz": float(last["touchdown_vz_m_s"]),
            "relative": has_target,
            "outcome": meta.get("outcome", "UNKNOWN"),
            "group": meta.get("scenario", {}).get("disturbance_preset", "off"),
        }
        if "horizontal_error_m" in last:
            pt["horizontal_error_m"] = float(last["horizontal_error_m"])
        pts.append(pt)
    return pts


def plot_timeseries(series: dict, out_png: str, tag: str) -> None:
    s = series.get(tag)
    if not s:
        return
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(s["t"], s["z"], color=C_BLUE)
    axes[0].set_ylabel("z (m)")
    axes[0].set_title(f"scenario: {tag}")
    axes[1].plot(s["t"], s["roll_deg"], color=C_ORANGE, label="roll")
    axes[1].plot(s["t"], s["pitch_deg"], color=C_GREEN, label="pitch")
    axes[1].set_ylabel("angle (deg)")
    axes[1].legend()
    axes[2].plot(s["t"], s["upper"], color=C_GRAY)
    axes[2].set_ylabel("upper motor (rad/s)")
    axes[2].set_xlabel("t (s)")
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def plot_landing_scatter(points: list[dict], out_png: str) -> None:
    fig, ax = plt.subplots(figsize=(7, 7))
    xs = [p["x"] for p in points]
    ys = [p["y"] for p in points]
    ax.scatter(xs, ys, c=C_BLUE, s=30, alpha=0.7)
    ax.add_patch(plt.Circle((0, 0), 0.3, fill=False, color=C_ORANGE, lw=1.5,
                            label="0.3m 验收圈"))
    ax.set_xlabel("落点 x 相对目标 (m)")
    ax.set_ylabel("落点 y 相对目标 (m)")
    ax.set_aspect("equal")
    ax.legend()
    title = f"落点分布（{len(points)} 次）"
    errs = [p["horizontal_error_m"] for p in points if "horizontal_error_m" in p]
    if errs:
        title += f"\n水平误差 mean±std: {np.mean(errs):.3f}±{np.std(errs):.3f} m"
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def plot_disturbance_boxplot(points: list[dict], out_png: str) -> None:
    by_group: dict[str, list[float]] = {}
    for p in points:
        by_group.setdefault(p["group"], []).append(abs(p["vz"]))
    groups = sorted(by_group)
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.boxplot([by_group[g] for g in groups], labels=groups)
    ax.set_ylabel("|touchdown vz| (m/s)")
    ax.set_title("扰动预设 vs 触水速度")
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def plot_success_rate(rows: list[dict], out_png: str) -> None:
    Path(out_png).parent.mkdir(parents=True, exist_ok=True)
    groups = [r["group"] for r in rows]
    rates = [r["success_rate"] * 100.0 for r in rows]
    colors = [C_GREEN if v >= 90 else (C_ORANGE if v >= 50 else "#dc2626") for v in rates]
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar(groups, rates, color=colors)
    ax.axhline(90, color=C_GRAY, ls="--", lw=1, label="90% 目标")
    ax.set_ylabel("成功率 (%)")
    ax.set_ylim(0, 105)
    ax.set_xticklabels(groups, rotation=30, ha="right")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def plot_touchdown_safety(points: list[dict], out_png: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 6))
    xs = [p.get("horizontal_error_m", abs(p["x"])) for p in points]
    vs = [abs(p["vz"]) for p in points]
    ax.scatter(xs, vs, c=C_BLUE, s=30, alpha=0.7)
    ax.axhline(0.35, color=C_GREEN, ls="--", lw=1.5, label="vz 安全边界 0.35 m/s")
    ax.set_xlabel("落点水平误差 (m)")
    ax.set_ylabel("触水 |vz| (m/s)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def main() -> int:
    p = argparse.ArgumentParser(description="Generate report charts.")
    p.add_argument("--batch-dir", default=str(BATCH_DIR))
    p.add_argument("--out-dir", default=str(REPORT_DIR))
    p.add_argument("--timeseries-tags", nargs="*", default=["baseline_repeat00", "dist_strong"])
    args = p.parse_args()

    batch = Path(args.batch_dir)
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    series = collect_timeseries(batch)
    for tag in args.timeseries_tags:
        plot_timeseries(series, str(out / f"timeseries_{tag}.png"), tag)

    points = build_landing_points(batch)
    plot_landing_scatter(points, str(out / "landing_scatter.png"))
    plot_disturbance_boxplot(points, str(out / "disturbance_boxplot.png"))
    plot_touchdown_safety(points, str(out / "touchdown_safety.png"))

    from analysis.analyze import aggregate, load_metas
    rows = aggregate(load_metas(batch))
    plot_success_rate(rows, str(out / "success_rate.png"))
    print(f"charts -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
