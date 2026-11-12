"""Real-time 3D Open3D Point Cloud Viewer and Victim Marker Visualization."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from app.lidar_display import LidarDisplayMixin, generate_target_box_wires
from app.lidar_mapping import UAVLiDAR3DReconstruction, main

__all__ = ["LidarDisplayMixin", "generate_target_box_wires", "UAVLiDAR3DReconstruction", "main"]

if __name__ == "__main__":
    main()
