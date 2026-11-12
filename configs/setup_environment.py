"""Install requirements exclusively into the bundled Python environment."""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from configs.config import local_path


def main():
    environment = local_path("python_env")
    if not Path(sys.prefix).resolve().is_relative_to(environment):
        raise RuntimeError("Use python_env/python.exe; installing into system Python is disabled.")
    for args in (["--upgrade", "pip"], ["-r", str(ROOT / "requirements.txt")]):
        subprocess.run([sys.executable, "-s", "-m", "pip", "--isolated", "install",
                        "--no-user", "--cache-dir", str(local_path("outputs/cache/pip")),
                        *args], cwd=ROOT, check=True)
    print("Cai dat xong. Chay START_ALL_SYSTEM_WITH_3D_LIDAR.bat")


if __name__ == "__main__":
    main()

