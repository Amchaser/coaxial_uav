import json
from pathlib import Path
from unittest import mock

import pytest

from analysis.plot_report import collect_timeseries, build_landing_points, plot_success_rate


# 目标 (-5,-5)；落点世界坐标 (-4.95,-5.03) => 相对坐标 (0.05,-0.03)
_TARGET_X = -5.0
_TARGET_Y = -5.0


def _make_batch(tmp_path):
    d = tmp_path / "batch" / "dist_strong"
    d.mkdir(parents=True)
    (d / "samples.csv").write_text(
        "t_s,sim_time_s,state,z_m,x_m,y_m,target_x_m,target_y_m,roll_deg,pitch_deg,yaw_deg,"
        "upper_rad_s,lower_rad_s,horizontal_error_m,touchdown_vz_m_s,abort_reason,"
        "disturbance_active,buoyancy_n,slamming_force_n\n"
        f"0.0,1.0,HIGH_HOVER,0.8,0.1,0.2,{_TARGET_X},{_TARGET_Y},1.5,-2.0,0.0,100,95,0.1,0.0,,True,0,0\n"
        f"1.0,2.0,LANDED,0.02,-4.95,-5.03,{_TARGET_X},{_TARGET_Y},0.5,0.4,0.0,60,55,0.058,-0.2,,True,80,1.0\n"
    )
    (d / "meta.json").write_text(json.dumps({
        "tag": "dist_strong",
        "scenario": {"disturbance_preset": "strong", "offset_x_m": 0.0,
                     "nonidealities": False, "platform_vx_m_s": 0.0},
        "outcome": "LANDED",
        "landing": {"touchdown_vz_m_s": -0.2, "final_horizontal_error_m": 0.058,
                    "max_abs_roll_deg": 1.5, "abort_reason": ""},
    }))
    return tmp_path / "batch"


def test_collect_timeseries(tmp_path):
    batch = _make_batch(tmp_path)
    series = collect_timeseries(batch)
    assert "dist_strong" in series
    assert series["dist_strong"]["z"] == [0.8, 0.02]
    assert series["dist_strong"]["roll_deg"] == [1.5, 0.5]


def test_build_landing_points(tmp_path):
    batch = _make_batch(tmp_path)
    pts = build_landing_points(batch)
    assert pts[0]["tag"] == "dist_strong"
    # 相对坐标：验收圈原点即目标 (-5,-5)
    assert pts[0]["x"] == pytest.approx(0.05)
    assert pts[0]["y"] == pytest.approx(-0.03)
    assert pts[0]["relative"] is True
    assert pts[0]["horizontal_error_m"] == pytest.approx(0.058)


def test_build_landing_points_fallback_without_target(tmp_path):
    d = tmp_path / "batch" / "legacy"
    d.mkdir(parents=True)
    # 旧数据：samples.csv 无 target_x_m/target_y_m 列
    (d / "samples.csv").write_text(
        "t_s,sim_time_s,state,z_m,x_m,y_m,roll_deg,pitch_deg,yaw_deg,upper_rad_s,lower_rad_s,"
        "horizontal_error_m,touchdown_vz_m_s,abort_reason,disturbance_active,buoyancy_n,slamming_force_n\n"
        "1.0,2.0,LANDED,0.02,-4.95,-5.03,0.5,0.4,0.0,60,55,0.058,-0.2,,True,80,1.0\n"
    )
    (d / "meta.json").write_text(json.dumps({
        "tag": "legacy",
        "scenario": {"disturbance_preset": "strong"},
        "outcome": "LANDED",
    }))
    pts = build_landing_points(tmp_path / "batch")
    assert pts[0]["relative"] is False
    # 退化：x 取 horizontal_error_m，y 置 0
    assert pts[0]["x"] == pytest.approx(0.058)
    assert pts[0]["y"] == 0.0


def test_plot_success_rate_writes_png(tmp_path):
    batch = _make_batch(tmp_path)
    out = tmp_path / "report"
    with mock.patch("matplotlib.pyplot.savefig"):
        from analysis.plot_report import plot_success_rate
        plot_success_rate([{"group": "baseline", "n": 10, "success_rate": 0.9},
                           {"group": "dist_strong", "n": 10, "success_rate": 0.4}],
                          str(out / "success_rate.png"))
    assert (out / "success_rate.png").exists()
