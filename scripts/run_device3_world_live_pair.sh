#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u
IMAGE="${DEVICE3_WORLD_IMAGE:-umi-ego-vio:device3-cc06413a-d405-product-v1.0.2-20260913}"
DATA_ROOT="${UMI_DEVICE3_D405_DATA_ROOT:-/home/robot/umi_ego_vio_data_device3}"
IMU_BY_ID=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_cc06413ab505f0118b03921272aab386-if00-port0
PYTHON_BIN="${DEVICE3_WORLD_PYTHON:-/opt/three-device-slam/venv/bin/python}"
RUN_DURATION="${DEVICE3_WORLD_DURATION:-0}"
NO_DISPLAY="${DEVICE3_WORLD_NO_DISPLAY:-0}"
VIEWER_CPUSET="${DEVICE3_WORLD_VIEWER_CPUSET:-}"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="${1:-$ROOT/artifacts/spatial_bench/device3_world_live_pair_$RUN_STAMP}"
if [[ "$OUTPUT" != /* ]]; then
  OUTPUT="$ROOT/$OUTPUT"
fi
CONTAINER="device3-world-live-$RUN_STAMP-$$"
DEVICE3_LOG="${OUTPUT}.device3_runtime.log"
VIEWER_LOG="${OUTPUT}.rerun_world3d.log"
SOLVER_ROOT="${OUTPUT}.solver"
SIGNED_CONFIG="$DATA_ROOT/active_runtime_calibration/vins_config.yaml"
WORLD_CONFIG="$SOLVER_ROOT/vins_config.yaml"
MOTION_RELEASE_LIB="${DEVICE3_WORLD_VINS_LIBRARY:-$ROOT/artifacts/runtime/device3-vins-motion-release-v3/image/libvins_lib.so}"
MOTION_RELEASE_HASHES="${DEVICE3_WORLD_VINS_HASHES:-$ROOT/artifacts/runtime/device3-vins-motion-release-v3/image/device3_candidate_product_live_hashes.env}"

if [[ "$NO_DISPLAY" != "1" ]]; then
  [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]] || {
    echo "DISPLAY或X11不可用，无法显示世界坐标实时窗口" >&2
    exit 4
  }
fi
if [[ -z "$VIEWER_CPUSET" ]] && command -v taskset >/dev/null 2>&1; then
  CPU_COUNT="$(nproc)"
  if (( CPU_COUNT >= 8 )); then
    VIEWER_CPUSET="$((CPU_COUNT - 4))-$((CPU_COUNT - 1))"
  fi
fi
[[ "$RUN_DURATION" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
  echo "DEVICE3_WORLD_DURATION必须是非负秒数" >&2
  exit 3
}
[[ ! -e "$OUTPUT" ]] || { echo "输出目录已存在: $OUTPUT" >&2; exit 3; }
[[ ! -e "$SOLVER_ROOT" ]] || { echo "求解器输出目录已存在: $SOLVER_ROOT" >&2; exit 3; }
[[ -x "$PYTHON_BIN" ]] || { echo "缺少Python环境: $PYTHON_BIN" >&2; exit 3; }
[[ -f "$MOTION_RELEASE_LIB" ]] || { echo "缺少设备3运动释放VINS库: $MOTION_RELEASE_LIB" >&2; exit 3; }
[[ -f "$MOTION_RELEASE_HASHES" ]] || { echo "缺少设备3运动释放哈希清单: $MOTION_RELEASE_HASHES" >&2; exit 3; }
[[ -e "$IMU_BY_ID" ]] || { echo "缺少设备3 STM32: $IMU_BY_ID" >&2; exit 3; }
docker image inspect "$IMAGE" >/dev/null
mkdir -p "$DATA_ROOT"/{recordings,realtime_sessions,slam_results,logs,hil_evidence,active_runtime_calibration}
mkdir -p "$(dirname -- "$OUTPUT")"
mkdir -p "$SOLVER_ROOT"

# The signed 5 cm per-output guard is appropriate for static product tests but
# latches during legitimate handheld motion above 0.75 m/s at the 15 Hz output
# rate.  Keep all signed calibration values and use a session-local 20 cm guard;
# this remains below the observed 30--56 cm estimator-failure jumps.
cp "$SIGNED_CONFIG" "$WORLD_CONFIG"
cp "$DATA_ROOT/active_runtime_calibration/left.yaml" "$SOLVER_ROOT/left.yaml"
cp "$DATA_ROOT/active_runtime_calibration/right.yaml" "$SOLVER_ROOT/right.yaml"
python3 - "$WORLD_CONFIG" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
old = "raw_odometry_failure_step_m: 0.05"
if text.count(old) != 1:
    raise SystemExit("签发配置中的位姿保护门槛不是唯一的0.05，拒绝生成会话配置")
path.write_text(text.replace(old, "raw_odometry_failure_step_m: 0.20"), encoding="utf-8")
PY

cleanup() {
  if docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER"; then
    docker stop --time 5 "$CONTAINER" >/dev/null 2>&1 || true
  fi
  if [[ -n "${DOCKER_PID:-}" ]]; then
    wait "$DOCKER_PID" 2>/dev/null || true
  fi
  if [[ -n "${VIEWER_PID:-}" ]] && kill -0 -- "-$VIEWER_PID" 2>/dev/null; then
    kill -- "-$VIEWER_PID" 2>/dev/null || true
    wait "$VIEWER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

docker run --rm \
  --name "$CONTAINER" \
  --network host \
  --ipc host \
  --user "$(id -u):$(id -g)" \
  --env HOME=/home/robot \
  --env EGO_VIO_DEVICE_SET_ID=UMI_DEVICE_03_CC06413A \
  --env EGO_VIO_RELEASE_ID=UMI_DEVICE3_D405_PRODUCT_V1_0_2_20260913 \
  --env DEVICE3_SOLVER_LOG_ROOT=/output/solver \
  --mount "type=bind,src=$DATA_ROOT,dst=/data" \
  --mount "type=bind,src=$SOLVER_ROOT,dst=/output" \
  --mount "type=bind,src=$MOTION_RELEASE_LIB,dst=/home/robot/ego_vio_humble/.product_live_build/vins_ws/build/vins_fusion_ros2/vins/libvins_lib.so,readonly" \
  --mount "type=bind,src=$MOTION_RELEASE_HASHES,dst=/opt/umi/device3_candidate_product_live_hashes.env,readonly" \
  --mount "type=bind,src=$ROOT/scripts/run_device3_vins_solver_only.sh,dst=/opt/umi/run_device3_vins_solver_only.sh,readonly" \
  --entrypoint bash \
  "$IMAGE" /opt/umi/run_device3_vins_solver_only.sh \
    /output/vins_config.yaml \
  >"$DEVICE3_LOG" 2>&1 &
DOCKER_PID=$!

solver_ready=0
for _ in {1..40}; do
  if grep -Fq 'SIGNED_DEVICE3_SOLVER_READY' "$DEVICE3_LOG" 2>/dev/null; then
    solver_ready=1
    break
  fi
  if ! kill -0 "$DOCKER_PID" 2>/dev/null; then
    break
  fi
  sleep 1
done
if [[ "$solver_ready" -ne 1 ]]; then
  echo "设备3实时VINS启动失败" >&2
  tail -n 80 "$DEVICE3_LOG" >&2 || true
  exit 1
fi

echo "设备3签发VINS求解器已启动；双相机将由同一RSUSB进程持有。"
echo "前15秒保持Ego和设备3静止，调整视野，让Ego同时看到AprilGrid与设备3 ID1。"
echo "随后按中文界面移动Ego约20秒完成ORB初始化；设备3和固定AprilGrid不要移动。"
echo "界面提示建锚时，两台设备都保持静止并继续看到AprilGrid与ID1。"
echo "两条世界锚建立后可以移动Ego和设备3；两台设备分别依靠ORB/VINS继续运行。"
if [[ "$RUN_DURATION" == "0" || "$RUN_DURATION" == "0.0" ]]; then
  echo "运行模式: 持续运行，直到按Q/Esc、关闭中文引导窗口或在终端按Ctrl-C。"
else
  echo "运行模式: 限时${RUN_DURATION}秒；到时会正常自动停止，不属于崩溃。"
fi
echo "输出: $OUTPUT"

if [[ "$NO_DISPLAY" != "1" ]]; then
  if [[ -n "$VIEWER_CPUSET" ]]; then
    setsid nice -n 10 taskset -c "$VIEWER_CPUSET" \
      "$PYTHON_BIN" -u "$ROOT/scripts/rerun_world_pair_viewer.py" \
      >"$VIEWER_LOG" 2>&1 &
    echo "Rerun软件渲染已隔离到CPU $VIEWER_CPUSET，避免影响实时SLAM。"
  else
    setsid nice -n 10 "$PYTHON_BIN" -u "$ROOT/scripts/rerun_world_pair_viewer.py" \
      >"$VIEWER_LOG" 2>&1 &
  fi
  VIEWER_PID=$!
  sleep 3
  if ! kill -0 "$VIEWER_PID" 2>/dev/null; then
    echo "三维Rerun查看器启动失败" >&2
    tail -n 80 "$VIEWER_LOG" >&2 || true
    exit 1
  fi
  echo "UMI三维查看器已待命；世界锚建立后自动打开窗口，蓝色=Ego，橙色=设备3。"
fi

ARGS=(--output "$OUTPUT" --duration "$RUN_DURATION")
if [[ "$NO_DISPLAY" == "1" ]]; then
  ARGS+=(--no-display)
fi
export DEVICE3_WORLD_IMAGE="$IMAGE"
export DEVICE3_WORLD_VINS_LIBRARY="$MOTION_RELEASE_LIB"
"$PYTHON_BIN" "$ROOT/scripts/device3_world_live_pair.py" "${ARGS[@]}"
