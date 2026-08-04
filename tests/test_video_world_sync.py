from pathlib import Path

WORLDS = Path(__file__).resolve().parents[1] / "worlds"
BASE_WORLD = WORLDS / "static_water_takeoff.sdf"
VIDEO_WORLD = WORLDS / "static_water_takeoff_video.sdf"

BEGIN_MARKER = b"<!-- VIDEO_ONLY_BLOCK_BEGIN: camera recorder; do not edit beyond this marker -->"
END_MARKER = b"<!-- VIDEO_ONLY_BLOCK_END -->"


def _strip_video_block(data: bytes) -> bytes:
    """去掉视频世界中从 VIDEO_ONLY_BLOCK_BEGIN 行首到 VIDEO_ONLY_BLOCK_END 行尾（含）的整段。"""
    begin = data.index(BEGIN_MARKER)
    end = data.index(END_MARKER, begin)
    end_of_line = data.index(b"\n", end)
    line_start = data.rfind(b"\n", 0, begin) + 1
    return data[:line_start] + data[end_of_line + 1:]


def test_video_world_matches_base_plus_camera():
    """视频世界必须等于基础世界 + 录像相机块。

    去掉 VIDEO_ONLY_BLOCK 标记段后，剩余内容应与基础世界逐字节一致，
    防止基础世界（PID 参数/物理/新模型）改动后视频世界静默漂移。
    """
    base = BASE_WORLD.read_bytes()
    video = VIDEO_WORLD.read_bytes()
    assert video.count(BEGIN_MARKER) == 1, "VIDEO_ONLY_BLOCK_BEGIN 标记必须唯一"
    assert video.count(END_MARKER) == 1, "VIDEO_ONLY_BLOCK_END 标记必须唯一"
    assert _strip_video_block(video) == base
