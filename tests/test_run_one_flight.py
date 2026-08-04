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
