import json

from scripts.batch_scan import decide_result, expand_grid, tag_for


def test_expand_grid_single_axis():
    grid = {"height.kp": [45.0, 60.0]}
    combos = expand_grid(grid)
    assert len(combos) == 2
    assert combos[0][0] == "scan_kp_45.0"
    assert combos[0][1] == {"height": {"kp": 45.0}}


def test_expand_grid_two_axes_cartesian():
    grid = {"height.kp": [45.0, 60.0], "landing.descent_vz": [-0.3, -0.5]}
    combos = expand_grid(grid)
    assert len(combos) == 4


def test_tag_for():
    assert tag_for("height.kp", 60.0) == "scan_kp_60.0"


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
