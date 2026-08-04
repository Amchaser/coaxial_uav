import json

from scripts.batch_scan import decide_result, expand_grid, sim_command, tag_for, validate_grid


def test_expand_grid_single_axis():
    grid = {"height.kp": [45.0, 60.0]}
    combos = expand_grid(grid)
    assert len(combos) == 2
    assert combos[0][0] == "scan_kp_45.0"
    assert combos[0][1] == {"height": {"kp": 45.0}}


def test_expand_grid_two_axes_cartesian():
    grid = {"height.kp": [45.0, 60.0], "landing.descent_rate_m_s": [0.35, 0.45]}
    combos = expand_grid(grid)
    assert len(combos) == 4


def test_tag_for():
    assert tag_for("height.kp", 60.0) == "scan_kp_60.0"


def test_validate_grid_accepts_real_keys():
    grid = {"height.kp": [40.0], "height.kd": [35.0],
            "landing.descent_rate_m_s": [0.35], "landing.high_hover_z_m": [1.6],
            "position_x.kp": [4.0], "position_velocity_limit_m_s": [2.0],
            "target_z_m": [0.8]}
    assert validate_grid(grid) == []


def test_validate_grid_rejects_unknown_keys():
    # 计划里的错误示例键：未知键会被 _publish_config 静默丢弃 -> 完全相同的组合
    unknown = validate_grid({"height.kp": [40.0],
                             "landing.descent_vz": [0.3],
                             "landing.near_water_height": [0.2]})
    assert "height.kp" not in unknown
    assert "landing.descent_vz" in unknown
    assert "landing.near_water_height" in unknown


def test_scan_main_records_rc_and_fails_on_nonzero():
    # Flight exits nonzero yet still emits a meta JSON line claiming success:
    # must be recorded as FAILED with the meta outcome kept visible.
    stdout = json.dumps({"outcome": "SUCCESS"}) + "\n"
    result = decide_result("dist_strong_scan_kp_45.0", 1, stdout)
    assert result["rc"] == 1
    assert result["outcome"] == "FAILED"
    assert result["reported_outcome"] == "SUCCESS"


def test_decide_result_success_rc_zero():
    stdout = json.dumps({"outcome": "SUCCESS"}) + "\n"
    result = decide_result("dist_strong_scan_kp_45.0", 0, stdout)
    assert result["rc"] == 0
    assert result["outcome"] == "SUCCESS"
    assert "reported_outcome" not in result


def test_decide_result_nonzero_with_unparseable_stdout():
    result = decide_result("tag", 2, "not a json meta line\n")
    assert result["rc"] == 2
    assert result["outcome"] == "FAILED"
    assert result["reported_outcome"] == "PARSE_FAIL"


def test_sim_command_includes_run_and_headless_flags():
    cmd = sim_command("/worlds/static_water_takeoff.sdf")
    assert cmd[0] == "gz"
    assert cmd[1] == "sim"
    # -r (run, not paused) and -s (headless) must both be present.
    assert "-r" in cmd
    assert "-s" in cmd
    assert "/worlds/static_water_takeoff.sdf" in cmd
