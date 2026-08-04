import subprocess
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "record_flight.sh"


def test_script_exists_and_syntax():
    assert SCRIPT.is_file()
    proc = subprocess.run(["bash", "-n", str(SCRIPT)], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr


def test_script_usage_message():
    proc = subprocess.run(["bash", str(SCRIPT), "--help"], capture_output=True, text=True)
    assert proc.returncode != 0  # --help 未处理，报错即说明被解析
    assert "tag" in (proc.stdout + proc.stderr).lower() or "usage" in (proc.stdout + proc.stderr).lower()
