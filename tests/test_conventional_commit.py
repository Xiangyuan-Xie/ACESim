import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_commit_msg.py"


def _run_checker(message: str) -> subprocess.CompletedProcess[str]:
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", delete=False) as file:
        file.write(message)
        path = file.name
    try:
        return subprocess.run(
            [sys.executable, str(SCRIPT), path],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
    finally:
        Path(path).unlink(missing_ok=True)


def test_conventional_commit_checker_accepts_required_scope() -> None:
    result = _run_checker("docs(license): adopt Apache 2.0 project docs\n")

    assert result.returncode == 0, result.stderr


def test_conventional_commit_checker_rejects_missing_scope() -> None:
    result = _run_checker("docs: update README\n")

    assert result.returncode == 1
    assert "type(scope): description" in result.stderr


def test_conventional_commit_checker_rejects_unknown_type() -> None:
    result = _run_checker("doc(readme): update README\n")

    assert result.returncode == 1
    assert "Allowed types" in result.stderr
