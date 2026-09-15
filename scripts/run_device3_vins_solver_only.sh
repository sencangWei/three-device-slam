#!/usr/bin/env bash
# Run the signed device3 VINS and loop nodes without opening any sensor.
set -euo pipefail

CONFIG="${1:-/data/active_runtime_calibration/vins_config.yaml}"
ROOT=/home/robot/ego_vio_humble
VINS_WS="$ROOT/.product_live_build/vins_ws"
LOOP_WS="$ROOT/.product_live_build/loop_ws"
VINS_EXECUTABLE="$VINS_WS/build/vins_fusion_ros2/vins_fusion_ros2_node"
VINS_LIBRARY="$VINS_WS/build/vins_fusion_ros2/vins/libvins_lib.so"
LOOP_EXECUTABLE="${EGO_VIO_PRODUCT_LIVE_LOOP_EXECUTABLE:-/opt/umi/device3_candidate_loop_fusion_node}"
HASH_MANIFEST="${EGO_VIO_PRODUCT_LIVE_HASH_MANIFEST:-/opt/umi/device3_candidate_product_live_hashes.env}"

for required in \
  /opt/ros/humble/setup.bash \
  "$VINS_WS/install/setup.bash" \
  "$LOOP_WS/install/setup.bash" \
  "$VINS_EXECUTABLE" "$VINS_LIBRARY" "$LOOP_EXECUTABLE" \
  "$HASH_MANIFEST" "$CONFIG"; do
  [[ -f "$required" ]] || { echo "缺少求解器文件: $required" >&2; exit 5; }
done

manifest_value() {
  local key="$1" value
  value="$(sed -n "s/^${key}=//p" "$HASH_MANIFEST" | head -n 1)"
  [[ "$value" =~ ^[0-9a-f]{64}$ ]] || {
    echo "哈希清单字段无效: $key" >&2
    exit 6
  }
  printf '%s' "$value"
}

verify_hash() {
  local key="$1" path="$2" expected actual
  expected="$(manifest_value "$key")"
  actual="$(sha256sum "$path" | awk '{print $1}')"
  [[ "$actual" == "$expected" ]] || {
    echo "签发哈希不匹配: $path" >&2
    echo "期望: $expected" >&2
    echo "实际: $actual" >&2
    exit 6
  }
}

verify_hash PRODUCT_LIVE_VINS_SHA256 "$VINS_EXECUTABLE"
verify_hash PRODUCT_LIVE_VINS_LIBRARY_SHA256 "$VINS_LIBRARY"
verify_hash PRODUCT_LIVE_LOOP_SHA256 "$LOOP_EXECUTABLE"

set +u
source /opt/ros/humble/setup.bash
source "$VINS_WS/install/setup.bash"
source "$LOOP_WS/install/setup.bash"
set -u

mkdir -p /data/runtime/device3_d405_product_v1/output/pose_graph
LOG_ROOT="${DEVICE3_SOLVER_LOG_ROOT:-/tmp/device3_solver}"
mkdir -p "$LOG_ROOT"

cleanup() {
  local pid
  for pid in "${LOOP_PID:-}" "${VINS_PID:-}"; do
    if [[ -n "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null; then
      kill -- "-$pid" 2>/dev/null || true
    fi
  done
  for _ in {1..20}; do
    local alive=0
    for pid in "${LOOP_PID:-}" "${VINS_PID:-}"; do
      [[ -n "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null && alive=1
    done
    [[ "$alive" -eq 0 ]] && break
    sleep 0.1
  done
  for pid in "${LOOP_PID:-}" "${VINS_PID:-}"; do
    if [[ -n "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null; then
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

cd "$(dirname -- "$CONFIG")"
setsid env \
  LD_LIBRARY_PATH="$VINS_WS/build/vins_fusion_ros2:$VINS_WS/build/vins_fusion_ros2/vins${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  "$VINS_EXECUTABLE" \
  --ros-args -p use_sim_time:=false -p config_file:="$CONFIG" \
  >"$LOG_ROOT/vins.log" 2>&1 &
VINS_PID=$!

setsid env \
  LD_LIBRARY_PATH="$LOOP_WS/build/vins_fusion_ros2:$LOOP_WS/build/vins_fusion_ros2/vins${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  "$LOOP_EXECUTABLE" "$CONFIG" \
  >"$LOG_ROOT/loop_fusion.log" 2>&1 &
LOOP_PID=$!

sleep 3
kill -0 "$VINS_PID" 2>/dev/null || { tail -n 80 "$LOG_ROOT/vins.log" >&2; exit 1; }
kill -0 "$LOOP_PID" 2>/dev/null || { tail -n 80 "$LOG_ROOT/loop_fusion.log" >&2; exit 1; }
echo "SIGNED_DEVICE3_SOLVER_READY vins_pid=$VINS_PID loop_pid=$LOOP_PID config=$CONFIG"

while kill -0 "$VINS_PID" 2>/dev/null && kill -0 "$LOOP_PID" 2>/dev/null; do
  sleep 1
done
echo "设备3 VINS 或回环节点异常退出" >&2
tail -n 80 "$LOG_ROOT/vins.log" >&2 || true
tail -n 80 "$LOG_ROOT/loop_fusion.log" >&2 || true
exit 1
