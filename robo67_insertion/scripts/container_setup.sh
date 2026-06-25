#!/usr/bin/env bash
# Robo67 container dependency setup -- uv-managed.
#
# Fixes the NumPy-2 / OpenCV ABI break inside `multipanda-container` and installs
# the vision deps the robo67_insertion nodes need. Uses the `uv` package manager
# (the host's static uv binary is reachable in the container via the /host mount).
#
# WHY: the container ships apt `python3-opencv` 4.5.4 built against NumPy 1.x, but
# `/usr/local/.../dist-packages/numpy` is 2.x -> `import cv2` raises
# "numpy.core.multiarray failed to import". We install a NumPy-2-compatible
# `opencv-python-headless` (>=4.10) into the SAME interpreter ROS/rclpy uses
# (/usr/bin/python3), which shadows the broken apt cv2 (it sits earlier on
# sys.path in /usr/local/lib/python3.10/dist-packages). No cv_bridge is used by
# our nodes, so there is no C++ ABI coupling to worry about.
#
# RUN AS ROOT INSIDE THE CONTAINER (writes to /usr/local):
#   docker exec --user root multipanda-container bash /host/Code/Robo67/robo67_insertion/scripts/container_setup.sh
#
# Idempotent: re-running is safe.
set -euo pipefail

UV=/host/.local/bin/uv
export UV_CACHE_DIR=/tmp/uvcache   # root-writable; avoids root-owned files in the host home
PY=/usr/bin/python3

if [[ ! -x "$UV" ]]; then
  echo "ERROR: uv not found at $UV (expected the host uv via the /host mount)." >&2
  exit 1
fi

echo "== uv: $($UV --version) =="
echo "== installing numpy-2-compatible OpenCV + pyrealsense2 into $PY =="
"$UV" pip install --system --python "$PY" \
  "opencv-python-headless>=4.10,<4.12" \
  "pyrealsense2>=2.54,<2.57"

echo "== verifying =="
"$PY" - <<'PY'
import numpy, cv2
print("numpy", numpy.__version__)
print("cv2  ", cv2.__version__, "->", cv2.__file__)
import numpy as np
cv2.findHomography(np.zeros((4,2)), np.eye(4,2))   # smoke
try:
    import pyrealsense2 as rs
    print("pyrealsense2", getattr(rs, "__version__", "imported OK"))
except Exception as e:
    print("pyrealsense2 NOT available:", e)
print("OK")
PY
echo "== done =="
