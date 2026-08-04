import subprocess
import sys
from pathlib import Path

from scripts.batch_runner import build_matrix, scenario_to_args

PROJECT_ROOT = Path(__file__).resolve().parents[1]
BATCH_SUMMARY = PROJECT_ROOT / "data" / "batch" / "batch_summary.json"


def test_build_matrix_has_baseline_repeats():
    m = build_matrix(repeat=2)
    tags = [s["tag"] for s in m]
    assert tags.count("baseline_repeat00") == 1
    assert tags.count("baseline_repeat01") == 1


def test_build_matrix_covers_dimensions():
    m = build_matrix(repeat=1)
    tags = [s["tag"] for s in m]
    assert "dist_off" in tags and "dist_strong" in tags
    assert "offset_1.0m" in tags and "offset_3.0m" in tags
    assert "nonideal_on" in tags
    assert "platform_0.3ms" in tags


def test_scenario_to_args_maps_fields():
    args = scenario_to_args({"tag": "x", "disturbance_preset": "mild",
                             "offset": 1.0, "nonidealities": False,
                             "platform_vx": 0.0, "config_json": "{}"})
    assert "--tag" in args and args[args.index("--tag") + 1] == "x"
    assert args[args.index("--disturbance-preset") + 1] == "mild"
    assert args[args.index("--offset-x") + 1] == "1.0"


def test_scenario_to_args_nonidealities_only_when_true():
    on = scenario_to_args({"tag": "n", "disturbance_preset": "off",
                           "offset": 0.0, "nonidealities": True,
                           "platform_vx": 0.0, "config_json": "{}"})
    off = scenario_to_args({"tag": "n", "disturbance_preset": "off",
                            "offset": 0.0, "nonidealities": False,
                            "platform_vx": 0.0, "config_json": "{}"})
    assert "--nonidealities" in on
    assert "--nonidealities" not in off


def test_dry_run_lists_scenarios_and_writes_nothing():
    BATCH_SUMMARY.unlink(missing_ok=True)
    r = subprocess.run(
        [sys.executable, str(PROJECT_ROOT / "scripts" / "batch_runner.py"),
         "--repeat", "1", "--dry-run"],
        cwd=PROJECT_ROOT, capture_output=True, text=True)
    assert r.returncode == 0
    assert "nonideal_on" in r.stdout
    assert "platform_0.3ms" in r.stdout
    assert not BATCH_SUMMARY.exists()

