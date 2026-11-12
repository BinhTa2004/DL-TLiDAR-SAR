"""Install project requirements into the active Python environment."""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def main():
    subprocess.run([sys.executable, "-m", "pip", "install", "--upgrade", "pip"], cwd=ROOT, check=True)
    subprocess.run([sys.executable, "-m", "pip", "install", "-r", str(ROOT / "requirements.txt")], cwd=ROOT, check=True)
    print("Cai dat xong. Chay run.bat hoac: python main.py gui")


if __name__ == "__main__":
    main()
