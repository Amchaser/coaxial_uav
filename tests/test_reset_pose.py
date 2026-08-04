import math
from unittest import mock

import pytest

from scripts.reset_pose import set_pose_service_request


def test_set_pose_service_request_json():
    req = set_pose_service_request("coaxial_uav", 1.0, -2.0, 0.34, 0.0)
    assert req["name"] == "coaxial_uav"
    # gz.msgs.Pose 为扁平结构：position/orientation 直接挂在顶层（无嵌套 pose 字段）
    assert req["position"]["x"] == 1.0
    assert req["position"]["y"] == -2.0
    assert req["position"]["z"] == 0.34
    assert req["orientation"]["z"] == 0.0
    assert req["orientation"]["w"] == 1.0


def test_set_pose_service_request_yaw():
    req = set_pose_service_request("coaxial_uav", 0, 0, 0.34, yaw=0.5)
    # 朝向必须是单位四元数（否则 gz 归一化后 yaw 不准确）
    z, w = req["orientation"]["z"], req["orientation"]["w"]
    assert round(z * z + w * w, 6) == 1.0
    assert z == pytest.approx(math.sin(0.5 / 2.0))
    assert w == pytest.approx(math.cos(0.5 / 2.0))


def test_reset_pose_builds_command():
    with mock.patch("subprocess.run", return_value=mock.Mock(returncode=0)) as m:
        from scripts.reset_pose import reset_pose
        reset_pose("coaxial_uav_static_water", "static_water_takeoff",
                   "coaxial_uav", 1.0, 0.0, 0.34)
    args = m.call_args.args[0]
    assert "/world/static_water_takeoff/set_pose" in args
    assert "--reqtype" in args and "gz.msgs.Pose" in args
    # gz service --req 只接受 protobuf 文本格式（不接受 JSON）
    req = args[args.index("--req") + 1]
    assert 'name: "coaxial_uav"' in req
    assert "position: {" in req
    assert "x: 1.0" in req
    assert "z: 0.34" in req


def test_reset_pose_timeout():
    import subprocess as _subprocess
    from scripts.reset_pose import reset_pose
    with mock.patch("subprocess.run", side_effect=_subprocess.TimeoutExpired("cmd", 5)):
        r = reset_pose("p", "w", "m", 0.0, 0.0, 0.34)
    assert r["ok"] is False
    assert "timed out" in r["message"]


def test_reset_pose_nonzero_returncode():
    from scripts.reset_pose import reset_pose
    with mock.patch("subprocess.run",
                    return_value=mock.Mock(returncode=1, stdout="", stderr="boom")):
        r = reset_pose("p", "w", "m", 0.0, 0.0, 0.34)
    assert r["ok"] is False
    assert "boom" in r["message"]


def test_reset_pose_data_false_reply():
    from scripts.reset_pose import reset_pose
    proc = mock.Mock(returncode=0, stdout="data: false\n", stderr="")
    with mock.patch("subprocess.run", return_value=proc):
        r = reset_pose("p", "w", "m", 0.0, 0.0, 0.34)
    assert r["ok"] is False


def test_reset_pose_data_true_reply():
    from scripts.reset_pose import reset_pose
    proc = mock.Mock(returncode=0, stdout="data: true\n", stderr="")
    with mock.patch("subprocess.run", return_value=proc):
        r = reset_pose("p", "w", "m", 0.0, 0.0, 0.34)
    assert r["ok"] is True


def test_reset_pose_missing_gz_binary():
    from scripts.reset_pose import reset_pose
    with mock.patch("subprocess.run", side_effect=FileNotFoundError):
        r = reset_pose("p", "w", "m", 0.0, 0.0, 0.34)
    assert r["ok"] is False
    assert "gz binary not found" in r["message"]
