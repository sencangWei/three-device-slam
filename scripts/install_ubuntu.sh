#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "FAIL/root_required" >&2
  exit 2
fi

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

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
vendor_sdk=/home/robot/vendor/2uq2/YLX_XU_API_2026721
if [[ ! -f ${vendor_sdk}/include/V4L2/extunit.h ]]; then
  echo "BLOCKED/2uq2_vendor_sdk_missing" >&2
  exit 3
fi

install -d -o robot -g robot /opt/three-device-slam
install -d -o robot -g robot /opt/three-device-slam/lib
install -d -o robot -g robot /etc/three-device-slam
install -d -o robot -g robot /var/lib/three-device-slam/sessions

if [[ ! -x /opt/three-device-slam/venv/bin/python ]]; then
  python3 -m venv --system-site-packages /opt/three-device-slam/venv
fi
/opt/three-device-slam/venv/bin/python -m pip install "$repo_root"
make -C "$repo_root/native/2uq2_xu_bridge" clean all VENDOR_SDK="$vendor_sdk"

artifact="$repo_root/native/2uq2_xu_bridge/build/libtwo_uq2_xu.so"
bridge=/opt/three-device-slam/lib/libtwo_uq2_xu.so
built_hash=$(sha256sum "$artifact" | awk '{print $1}')
install -m 0755 "$artifact" "$bridge"
installed_hash=$(sha256sum "$bridge" | awk '{print $1}')
if [[ ${built_hash} != "${installed_hash}" ]]; then
  echo "FAIL/xu_bridge_hash" >&2
  exit 2
fi
hash_manifest="${artifact}.sha256"
printf '%s  %s\n' "$built_hash" "$bridge" >"$hash_manifest"
install -m 0644 "$hash_manifest" "${bridge}.sha256"

file "$bridge" | grep -Eq 'ELF 64-bit.*x86-64'
for symbol in ylx_open ylx_read_imu27 ylx_close; do
  nm -D --defined-only "$bridge" | grep -Eq "[[:space:]]${symbol}$"
done
