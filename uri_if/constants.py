"""Shared defaults (no heavy dependencies)."""

import os
from pathlib import Path


HOST = "192.168.56.101"  # Default/first robot
HOST_AYAL = "192.168.57.101"  # Robot 1
HOST_URI = "192.168.56.101"   # Robot 2
GRIPPER_PORT = 63352
_RMP_LAB_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CALIB_FILE = Path(_RMP_LAB_ROOT) / "shared" / "calibration.json"