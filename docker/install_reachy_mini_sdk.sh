#!/bin/bash
# Install the Pollen Robotics `reachy-mini` SDK into the distro's Python
# (the one rclpy is built for), so ROS nodes can `import reachy_mini`.
#
# Why not plain `pip install reachy-mini`? The SDK pins pygobject<=3.46, which
# has no wheel for this Python and fails to build. The distro's python3-gi
# (pygobject 3.56) is API-compatible, so we install the SDK without
# dependencies, then every dependency except pygobject.
set -euo pipefail
VERSION=${1:-1.10.0}
export PATH=$HOME/.local/bin:$PATH
command -v uv >/dev/null || curl -LsSf https://astral.sh/uv/install.sh | sh >/dev/null
UV="uv pip install --python /usr/bin/python3 --system --break-system-packages --no-cache"
$UV --no-deps "reachy-mini==${VERSION}" packaging
python3 - > /tmp/reachy-mini-deps.txt <<'PY'
import importlib.metadata as m
from packaging.requirements import Requirement
for r in m.requires("reachy-mini"):
    req = Requirement(r)
    if req.marker and not req.marker.evaluate({"extra": ""}):
        continue
    if req.name.lower() == "pygobject":
        continue
    print(f"{req.name}{req.specifier}")
PY
$UV -r /tmp/reachy-mini-deps.txt
python3 -c "import reachy_mini, gi; print('reachy-mini SDK', reachy_mini.__version__ if hasattr(reachy_mini, '__version__') else '${VERSION}', 'with pygobject', gi.__version__)"
