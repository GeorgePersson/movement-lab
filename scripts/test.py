"""Run the full test suite with fresh observation outputs; Python 3 standard library."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parent.parent
output = Path(tempfile.mkdtemp(prefix="movement-lab-tests-"))
env = dict(os.environ)
env.update(
    WF85_OUTPUT_DIR=str(output / "reload"),
    WF86_OUTPUT_DIR=str(output / "camera"),
    WF88_OUTPUT_DIR=str(output / "slope"),
)
print(f"Test observations: {output}", flush=True)
result = subprocess.run(
    ["cargo", "test", "--locked", "--features", "viewer", "--all-targets", *sys.argv[1:]],
    cwd=root,
    env=env,
)
sys.exit(result.returncode)
