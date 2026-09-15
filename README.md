# three-device-slam

面向 Ubuntu 22.04 / ROS 2 Humble 的三设备同步采集与空间定位项目。目标设备编组为：

- Ego：D435i（当前临时实时世界坐标路线）或正式 2UQ2 Ego；
- 左 UMI：D405 + 独立外部 IMU；
- 右 UMI：D405 + 独立外部 IMU。

仓库包含多设备身份绑定、联合健康门、时间戳证据、同步索引、D405 VINS、D435i ORB-SLAM3、AprilGrid 世界锚、三维实时显示、离线分析与验收工具。采集数据、运行日志、构建产物和设备凭据不进入 Git。

## 主要入口

正式三设备采集入口：

```bash
./run_three_device_slam.sh product
```

当前 Ego + 设备3临时世界坐标实时运行：

```bash
./scripts/run_device3_world_live_pair.sh
```

默认持续运行，直到在引导窗口按 `Q` / `Esc`、关闭窗口或在终端按 `Ctrl+C`。

## 安装与验证

```bash
sudo ./scripts/install_ubuntu.sh
sudo ./scripts/verify_ubuntu_environment.sh
PYTHONPATH=.:/opt/three-device-slam-vendor/librealsense-rsusb-2.58.2/python \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q
```

硬件验收结果与算法边界记录在 `docs/acceptance/`，当前工作进展见 `progress.md`，历史交接入口见 `CODEX_HANDOFF.md`。

## 数据边界

`artifacts/` 保存录包、轨迹、运行时二进制和验收证据，仅保留在受控设备本地。仓库提交源码、配置模板、补丁、测试和可复现构建脚本，不提交相机原始数据、私钥或本机认证信息。
