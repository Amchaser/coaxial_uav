import json
import subprocess
import sys
from pathlib import Path

from scripts.batch_runner import build_matrix, main, scenario_to_args

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


class _FakeProc:
    """Stand-in for subprocess.CompletedProcess; no real subprocess is spawned."""

    def __init__(self, returncode, stdout, stderr=""):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def test_real_run_stub_subprocess_locks_summary(tmp_path, monkeypatch, capsys):
    # Redirect the hardcoded output dir (PROJECT_ROOT/data/batch/...) to a temp
    # dir so the test is hermetic and never touches a real batch_summary.json.
    import scripts.batch_runner as br
    monkeypatch.setattr(br, "PROJECT_ROOT", tmp_path)
    summary_path = tmp_path / "data" / "batch" / "batch_summary.json"

    expected = build_matrix(repeat=1)
    calls = []

    def stub_run(cmd, **kwargs):
        calls.append(cmd)
        tag = cmd[cmd.index("--tag") + 1]
        if tag == "baseline_repeat00":
            return _FakeProc(0, json.dumps({"outcome": "LANDED"}) + "\n")
        return _FakeProc(1, json.dumps({"outcome": "ABORTED"}) + "\n", "sim err\n")

    monkeypatch.setattr(subprocess, "run", stub_run)
    monkeypatch.setattr(sys, "argv", ["batch_runner", "--repeat", "1"])

    assert main() == 0

    # One subprocess spawn per scenario; nothing actually ran.
    assert len(calls) == len(expected)

    assert summary_path.exists()
    data = json.loads(summary_path.read_text(encoding="utf-8"))
    # Lock the summary structure: {"total", "results":[{tag, returncode, outcome}]}.
    assert data["total"] == len(expected)
    assert len(data["results"]) == len(expected)
    for r in data["results"]:
        assert set(r) == {"tag", "returncode", "outcome"}

    # Exactly one LANDED (the baseline), everything else ABORTED.
    outcomes = [r["outcome"] for r in data["results"]]
    assert outcomes.count("LANDED") == 1
    landed = next(r for r in data["results"] if r["outcome"] == "LANDED")
    assert landed["tag"] == "baseline_repeat00"
    assert landed["returncode"] == 0

    # The SUMMARY line reports the same LANDED count.
    out = capsys.readouterr().out
    assert f"SUMMARY: 1/{len(expected)} LANDED" in out

