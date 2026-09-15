#!/usr/bin/env bash
# Read-only checks: does not open the camera or serial port.
set -euo pipefail
UMI_RELEASE_ROOT=$(cd -- "$(dirname -- "$(readlink -f -- "$0")")" && pwd)
test "$(uname -m)" = aarch64 || { echo 'Requires aarch64'; exit 1; }
python3 -c 'import sys; assert sys.version_info[:2] == (3, 12), "Requires Python 3.12"'
grep -aq 'rockchip,rk3576' /proc/device-tree/compatible || {
  echo 'Requires RK3576 with a supported Rockchip Linux image'; exit 1;
}
for element in fdsrc rawvideoparse videoconvert tee queue mpph265enc filesink videorate videoscale jpegenc; do
  gst-inspect-1.0 "$element" >/dev/null
done
env PYTHONPATH="$UMI_RELEASE_ROOT/app:$UMI_RELEASE_ROOT/runtime" python3 -c \
  'import pyrealsense2, pyrsutils; from three_device_slam.edge_rk3576.rsusb_capture import capture_rsusb_session; print("HOST_RUNTIME_PASS")'
