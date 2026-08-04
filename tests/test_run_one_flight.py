from scripts.run_one_flight import build_config, record_takeoff, takeoff_summary


class Args:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def test_build_config_disturbance_off_disabled():
    a = Args(disturbance_preset="off", nonidealities=False,
             platform_vx=0.0, target_x=0.0, target_y=0.0, target_z=0.8,
             config_json="{}")
    cfg = build_config(a)
    assert cfg["disturbance"]["enabled"] is False
    assert cfg["disturbance"]["preset"] == "off"


def test_build_config_disturbance_on_enabled():
    a = Args(disturbance_preset="strong", nonidealities=False,
             platform_vx=0.0, target_x=0.0, target_y=0.0, target_z=0.8,
             config_json="{}")
    cfg = build_config(a)
    assert cfg["disturbance"]["enabled"] is True
    assert cfg["disturbance"]["preset"] == "strong"


def test_build_config_moving_target():
    a = Args(disturbance_preset="off", nonidealities=False,
             platform_vx=0.5, target_x=0.0, target_y=0.0, target_z=0.8,
             config_json="{}")
    cfg = build_config(a)
    assert cfg["moving_target_enabled"] is True
    assert cfg["target_vx_m_s"] == 0.5


def test_build_config_config_json_merge():
    a = Args(disturbance_preset="off", nonidealities=False,
             platform_vx=0.0, target_x=0.0, target_y=0.0, target_z=0.8,
             config_json='{"height": {"kp": 60.0}}')
    cfg = build_config(a)
    assert cfg["height"]["kp"] == 60.0


def test_record_takeoff_liftoff_and_max_attitude():
    acc = {}
    prev = {"z": 0.02, "roll": 0.0, "pitch": 0.0}
    # 离水后一帧：z 上升，姿态偏
    rec = record_takeoff({"z": 0.15, "roll": 0.05, "pitch": -0.03},
                         prev, t=1.0, acc=acc)
    assert acc["t_liftoff_s"] == 1.0
    assert acc["max_abs_roll_rad"] == 0.05
    assert acc["max_abs_pitch_rad"] == 0.03
    assert acc["max_z_m"] == 0.15


def test_takeoff_summary_overshoot():
    acc = {"t_liftoff_s": 2.0, "max_abs_roll_rad": 0.08,
           "max_abs_pitch_rad": 0.06, "max_z_m": 0.9, "target_z_m": 0.8,
           "stabilize_s": 5.5}
    s = takeoff_summary(acc)
    assert s["overshoot_m"] == 0.1
    assert s["t_liftoff_s"] == 2.0
    assert s["stabilize_time_s"] == 5.5


from scripts.run_one_flight import deep_merge, landing_row


def test_deep_merge_nested():
    base = {"height": {"kp": 45.0, "kd": 35.0}, "z": 0.8}
    extra = {"height": {"kp": 60.0}, "q": 1}
    out = deep_merge(base, extra)
    assert out["height"] == {"kp": 60.0, "kd": 35.0}
    assert out["q"] == 1


def test_landing_row_fields():
    st = {"position": {"z": 0.30}, "attitude": {"roll_rad": 0.01, "pitch_rad": -0.02, "yaw_rad": 0.0},
          "motors": {"upper_rad_s": 100.0, "lower_rad_s": 90.0},
          "stats": {"sim_time_s": 10.0}}
    status = {"landing_state": "SLOW_DESCENT", "landing_horizontal_error_m": 0.2,
              "landing_touchdown_vz_m_s": -0.1}
    row = landing_row(st, status, t=1.0)
    assert row["state"] == "SLOW_DESCENT"
    assert row["z_m"] == 0.30
    assert row["horizontal_error_m"] == 0.2
    assert row["touchdown_vz_m_s"] == -0.1
