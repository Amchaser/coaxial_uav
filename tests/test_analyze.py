import json
import statistics
from pathlib import Path
from analysis.analyze import aggregate, group_key


def _meta(tag, outcome="LANDED", vz=0.2, err=0.1, roll=2.0):
    return {
        "tag": tag,
        "scenario": {"disturbance_preset": "off", "offset_x_m": 0.0,
                     "nonidealities": False, "platform_vx_m_s": 0.0},
        "outcome": outcome,
        "landing": {"touchdown_vz_m_s": vz, "final_horizontal_error_m": err,
                    "max_abs_roll_deg": roll, "abort_reason": ""},
    }


def test_group_key_baseline():
    assert group_key(_meta("baseline_repeat00")) == "baseline"


def test_group_key_disturbance():
    m = _meta("dist_strong")
    m["scenario"]["disturbance_preset"] = "strong"
    assert group_key(m) == "dist_strong"


def test_aggregate_success_rate():
    metas = [_meta("a"), _meta("b", outcome="ABORTED")]
    rows = aggregate(metas)
    row = rows[0]
    assert row["n"] == 2
    assert row["success_rate"] == 0.5
    assert row["touchdown_vz_mean"] == 0.2


def test_aggregate_std():
    metas = [_meta("a", vz=0.2), _meta("b", vz=0.4)]
    rows = aggregate(metas)
    # 实现使用样本标准差 statistics.stdev 并保留 4 位小数（[0.2,0.4] -> 0.1414），而非总体标准差 0.1
    expected = round(statistics.stdev([0.2, 0.4]), 4)
    assert abs(rows[0]["touchdown_vz_std"] - expected) < 1e-9
