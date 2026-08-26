# 第一阶段软件验收记录

## 结论

- `PASS/phase1_software`：三设备同步采集第一阶段软件已在 Ubuntu 22.04 / ROS 2 Humble 上完成安装与无硬件验收。
- `BLOCKED/phase1_hil:device_hardware_unavailable`：本次没有两台 D405、两个外部 IMU 和一台 2UQ2，未执行真实三设备采集。
- `BLOCKED/spatial_alignment:phase_not_delivered`：第一帧健康 Ego SLAM 位姿、重力对齐和无标记双 UMI 空间约束属于后续空间阶段，不属于本次软件 PASS。

## 受测版本与环境

| 项目 | 观测值 |
|---|---|
| 验收 UTC | `2026-08-26T03:20:05Z` |
| 主机 | `robot-MS-7E19` |
| 操作系统 | Ubuntu 22.04.5 LTS, x86-64 |
| ROS | ROS 2 Humble |
| 分支 | `feature/acquisition-sync-clean-extraction` |
| 提交 | `4dbe3f1f29c81f6f58403f5a8a8b9e457fe0460b` |
| 干净验收目录 | `/home/robot/worktrees/three-device-slam-phase1-b2ff517` |
| Git bundle SHA256 | `992bdd2b420cecef0637d9cb32f85db516cd6729b4647e8b7049addf4c127f27` |

`git bundle verify` 确认 bundle 包含完整历史；最终 checkout 的 `git status --short --branch` 仅显示分支行，无修改或未跟踪文件。

## 验证证据

| 验证 | 结果 |
|---|---|
| `python3 -m pytest -q`，最终 bundle checkout | `629 passed in 290.91s` |
| 本地 WSL 全量回归 | `628 passed, 1 skipped in 41.60s`；WSL 缺少 Gst introspection，Ubuntu 实机执行该项 |
| `python3 -m compileall -q three_device_slam` | exit `0` |
| `git diff --check` | exit `0` |
| `sudo ./scripts/install_ubuntu.sh` | exit `0`；重复安装成功 |
| `sudo ./scripts/verify_ubuntu_environment.sh` | `PASS/ubuntu_environment` |
| 安装包导入 | `three_device_slam 0.1.0`，来自 `/opt/three-device-slam/venv/...` |
| 采集运行依赖 | `yaml 5.4.1`、`cv2 4.5.4`、`pyrealsense2`、`GStreamer 1.20.3` 均从隔离解释器成功导入 |
| RealSense Python 模块 | 运行路径位于 `/opt/three-device-slam/venv/...`；SHA256 `ec0089d1618732f298048b3f4bb4dccab99c95361f60ccd4ade8d78f1922b0a1` |
| GStreamer 只读探针 | Ubuntu 实机隔离 HOME/XDG 零写；验证前后 `/dev/null` 均为字符设备 `1:3`、模式 `0666`、大小 `0` |
| 2UQ2 XU 桥接库 | x86-64 ELF；导出 `ylx_open`、`ylx_read_imu27`、`ylx_close` |
| 已安装桥接库 SHA256 | `09419497fbb2af4cb28374e607e5f6eb7b6b32629048aef0bcb32e44a6d63227` |
| 原仓运行时引用扫描 | `FINAL_ISOLATION_PASS` |
| 缺配置一键入口 | exit `3`，`BLOCKED/device_config_missing` |
| 产品配置 | 安装器未创建 `/etc/three-device-slam/product.json` |

缺配置验证使用正式入口 `./run_three_device_slam.sh product --config /tmp/three-device-slam-missing.json`。它在打开设备前安全退出，未伪造产品配置或设备身份。

## 远程验收发现并修复的问题

1. ROS 2 的标准 `setup.bash` 在安装脚本的 `set -u` 下读取未定义变量；现仅在加载 ROS 环境期间暂停 nounset，并在返回后恢复。
2. 安装器固定 `RM=rm` 时，首次 `make clean` 会因不存在 `build/` 失败；清理目标现为幂等的 `rm -rf -- build`。
3. Ubuntu 22.04 默认 mawk 不支持 `{64}` 间隔正则，导致正确 SHA256 被拒绝；校验现使用长度和字符集判断。
4. pip 22 + `--system-site-packages` 的构建隔离优先导入 setuptools 59.6，PEP 621 元数据被构建成 `UNKNOWN-0.0.0`；包元数据现使用 setuptools 59.6 可读的 `setup.cfg`，回归测试要求生成 `three_device_slam-0.1.0` wheel。
5. 原环境验证器只验证主包导入，未发现 `pyrealsense2` 缺失；现安装器和只读验证器逐项导入 YAML、OpenCV、RealSense 和 GStreamer，并固定 RealSense 模块路径、架构与 SHA256。
6. GStreamer 初始化可能写入 registry；曾尝试将 registry 指向 `/dev/null`，Ubuntu 实测发现 GStreamer 会替换该设备节点。测试后已立即恢复 `/dev/null` 为字符设备 `1:3`、模式 `0666`；最终探针完全不调用 `Gst.init`，只导入 Gst 并读取版本，且有真实零写与设备节点不变回归。

上述问题均有先失败、再通过的自动化回归测试。

## D405 隔离与主机变更

- `/home/robot/ego_vio_humble` 在验收前后均解析到 `/home/robot/releases/ego_vio_humble/product_v1_20260824`。
- 远程 D405 正式仓库保持在 `release/humble-stm32-product-v1-20260824`，HEAD 为 `a7a143d`，工作树干净；本任务未修改该仓库、分支或虚拟环境。
- 三设备产品独立安装到 `/opt/three-device-slam`，配置目录为 `/etc/three-device-slam`，会话目录为 `/var/lib/three-device-slam/sessions`。
- RealSense Python 二进制从正式 D405 发布包只读复制到 root 管理的独立输入目录 `/opt/three-device-slam-vendor`，随后安装进三设备 venv；运行时不引用 D405 发布目录。
- 为满足 Ubuntu venv 前置条件，安装了官方包 `python3.10-venv`、`python3-pip-whl` 和 `python3-setuptools-whl`；没有替换系统 Python。
- 安装产生的 checkout 内临时 `build/`、`three_device_slam.egg-info/` 和桥接 `build/` 已清理，最终验收 checkout 恢复干净。

## 实机验收入口条件

具备一台 2UQ2、两台 D405、两个独立外部 IMU、真实设备身份和真实标定 ID 后，才能创建 `/etc/three-device-slam/product.json` 并执行 Task 9。真实 HIL 必须验证预热、联合健康门、自动起录、Ctrl+C/故障封存、三设备时间域证据、离线索引和校验；未执行前不得标记 HIL PASS。
