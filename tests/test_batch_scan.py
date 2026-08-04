from scripts.batch_scan import expand_grid, tag_for


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
