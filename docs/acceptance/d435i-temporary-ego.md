# D435i 临时 Ego 采集与 coordinator 接入

日期：2026-08-28

## 结论

- `PASS/D435i standalone HIL`：左右红外 `1280x720 Y8 @ 30 Hz`、内置陀螺仪和加速度计 `200 Hz`，默认 5 Hz 最新帧预览。
- `PASS/D435i barrier HIL`：发布 Ego 健康心跳、等待 coordinator 计划开始时刻、按采集时间划分正式段、生成 `session/ego/acceptance.json`。
- `PASS/D435i + one D405 shared-owner HIL`：D435i Ego 与一只 D405 UMI 已在同一进程、同一控制线程下完成 RSUSB 联合采集与时间门禁。
- `BLOCKED/full two-UMI HIL: right_device_not_connected`：正式入口已经要求 Ego、left、right 三个明确身份，但当前只连接一只 D405 和一条外置 IMU；不得把单 UMI PASS 冒充完整三设备 PASS。
- `BLOCKED/D435i end-to-end product offline`：现有 `sync_index` 和最终 session verifier 仍读取 2UQ2 的 `ego.video/ego.xu` 数据契约，尚未切换到 D435i 的左右红外与内置 IMU契约。

这些状态相互独立；上述 PASS 不替代真实双 D405 三设备验收，也不代表最终 2UQ2 产品验收。

## RSUSB 所有权约束

实机 A/B 已证明，D435i 与 D405 不能由不同进程分别拥有 RSUSB 生命周期；即使共享 `rs.context`，由不同线程执行设备 start/stop 也不稳定。D435i 临时 Ego 的正式产品采集入口因此使用一个 owner，在一个控制线程内按 `ego -> left -> right` 顺序启动和反序停止所有 RealSense 设备。左右外部 IMU各自由独立串口读取线程记录，但不会拥有 RealSense 设备。

产品配置为 D435i 时，`run_product_capture` 不再调用旧的三个 worker coordinator；它生成一条包含三个相机序列号、两条 IMU by-id 路径、三个标定证据 ID 和两个明确 IMU 协议的共享-owner命令。任一身份缺失时，在打开流之前返回 `BLOCKED/required_hardware_unavailable`。

## 固定设备与流契约

- 型号：Intel RealSense D435I
- 实测序列号：`327122078613`
- 固件：`5.13.0.55`
- USB：`3.2`
- 左红外：`ego.ir_left`，Y8，`1280x720 @ 30 Hz`
- 右红外：`ego.ir_right`，Y8，`1280x720 @ 30 Hz`
- 陀螺仪：`ego.gyro`，little-endian float32 XYZ，rad/s，`200 Hz`
- 加速度计：`ego.accel`，little-endian float32 XYZ，m/s²，`200 Hz`

每条记录保留 RealSense 原始 `global_time`、映射到主机单调时钟的 `acquisition_ns`、主机 `arrival_ns`、帧号、映射参数、CRC 和有效性。左右图像按帧号配对，最大允许采集时刻差为 1 ms。

## 单机录制命令

```bash
cd /home/robot/three-device-slam

OPENBLAS_NUM_THREADS=1 PYTHONPATH=. \
/opt/three-device-slam/venv/bin/python -u -m \
three_device_slam.devices.d435i_ego.worker \
  --serial 327122078613 \
  --output artifacts/d435i_ego/manual_run1 \
  --duration 30 \
  --warmup 2 \
  --preview-hz 5
```

窗口按 `q` 或 `ESC` 会停止并把本次数据标为操作者中止；`--no-preview` 仅用于无桌面诊断。

## 正式配置中的 Ego 片段

现有未写 `type` 的 Ego 配置继续按 2UQ2 解析。临时 D435i 必须显式写：

```json
{
  "type": "d435i",
  "serial": "327122078613",
  "calibration_id": "d435i-factory-327122078613-v1"
}
```

不能在 D405 身份和两条外置 IMU 稳定路径未知时生成完整 `product.json`。

## HIL 证据

### Standalone preview run1

路径：`artifacts/d435i_ego/hil_720p_200hz_preview_20260828_run1`

- 左/右红外：各 `30.029 Hz`，151/151 正式帧全部配对，最大 skew `0 ms`。
- 陀螺仪：`200.551 Hz`。
- 加速度计：`202.133 Hz`。
- 序列缺口、时间回退、写队列丢失：全部 `0`。
- 独立 CRC/哈希复核：`PASS`，总文件字节数 `403657147`。

### Joint barrier run2

路径：`artifacts/d435i_ego/joint_barrier_720p_200hz_preview_20260828_run2`

- worker 先发布健康心跳，外部测试协调端再写入未来 `scheduled_start_ns`。
- 第一张正式图像采集时刻晚于计划开始 `33.076 ms`，通过 `0–50 ms` 起录覆盖门。
- 左/右红外：各 `29.980 Hz`，63/63 正式帧全部配对。
- 陀螺仪：`200.223 Hz`；加速度计：`201.806 Hz`。
- `session/ego/acceptance.json`：`PASS`。

run1 的首次 barrier 尝试因使用 USB 到达时刻划分正式边界，被独立检查发现首帧采集时刻早于计划开始 `9.03 ms`，应保留为 `FAIL` 诊断证据；run2 已改为按 acquisition time 截断并加入自动 acceptance 门。

### Shared-owner one-UMI run4

路径：`artifacts/pair_hil/shared_owner_emitter_off_d435i_ego_d405_left_20260828_run4`

- D435i 红外投射器显式关闭并回读 `emitter_enabled=0`。
- D435i 双目 `29.941 Hz`，gyro `199.968 Hz`，accel `201.536 Hz`。
- D405 RGB/双 IR `30.007 Hz`，外部 IMU `399.667 Hz`。
- 正式窗口内序列缺口、时间回退、IMU 传输错误和落盘队列丢失全部为 `0`。
- Ego 到 UMI 最近帧最大差 `16.683 ms`，低于 `20 ms` 门限。

### Full-group missing-right run1

路径：`artifacts/pair_hil/formal_group_missing_right_20260828_run1`

- 正式 D435i 产品入口生成单-owner三设备拓扑。
- 当前枚举不到配置中的 right D405，因此未打开任何采集流。
- `coordinator.json` 为 `BLOCKED/required_hardware_unavailable`，`scheduled_start_ns=null`；没有降级成单 UMI 或旧的三进程模式。
