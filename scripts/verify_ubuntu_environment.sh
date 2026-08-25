#!/usr/bin/env bash
set -euo pipefail

if [[ ! -r /etc/os-release ]]; then
  echo "FAIL/unsupported_ubuntu" >&2
  exit 2
fi
# shellcheck source=/etc/os-release
source /etc/os-release
if [[ ${ID} != ubuntu || ${VERSION_ID} != 22.04 ]]; then
  echo "FAIL/unsupported_ubuntu" >&2
  exit 2
fi

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "FAIL/ros_humble_missing" >&2
  exit 2
fi
# shellcheck source=/opt/ros/humble/setup.bash
source /opt/ros/humble/setup.bash
if [[ ${ROS_DISTRO:-} != humble ]]; then
  echo "FAIL/ros_humble_missing" >&2
  exit 2
fi

python=/opt/three-device-slam/venv/bin/python
bridge=/opt/three-device-slam/lib/libtwo_uq2_xu.so
if [[ ! -x ${python} ]]; then
  echo "FAIL/venv_missing" >&2
  exit 2
fi
if [[ ! -f ${bridge} ]]; then
  echo "FAIL/xu_bridge_missing" >&2
  exit 2
fi
if [[ ! -f ${bridge}.sha256 ]]; then
  echo "FAIL/xu_bridge_hash" >&2
  exit 2
fi
if [[ ! -d /var/lib/three-device-slam/sessions ]]; then
  echo "FAIL/session_root_missing" >&2
  exit 2
fi

"$python" -B -c 'import three_device_slam' || {
  echo "FAIL/python_package_missing" >&2
  exit 2
}
sha256sum --check --status "${bridge}.sha256" || {
  echo "FAIL/xu_bridge_hash" >&2
  exit 2
}
file "$bridge" | grep -Eq 'ELF 64-bit.*x86-64' || {
  echo "FAIL/xu_bridge_architecture" >&2
  exit 2
}
for symbol in ylx_open ylx_read_imu27 ylx_close; do
  nm -D --defined-only "$bridge" | grep -Eq "[[:space:]]${symbol}$" || {
    echo "FAIL/xu_bridge_symbol_missing" >&2
    exit 2
  }
done

echo "PASS/ubuntu_environment"
