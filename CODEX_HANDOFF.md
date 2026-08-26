# three-device-slam Codex 交接（2026-08-26）

## 新对话的首要任务

先阅读本文件、`PROJECT_LOG.md`、`docs/superpowers/specs/2026-08-25-three-device-slam-repository-design.md`、`docs/superpowers/plans/2026-08-25-acquisition-sync-clean-extraction.md` 和 `docs/acceptance/phase1-software-acceptance.md`，再核对 `git status`、当前分支和 HEAD。不要重新修改或合并 D405 项目。

当前尚未回答的问题是：在没有真实设备、没有 Ego 历史数据的情况下，是否确认先设计“确定性三设备模拟器”，再设计“空间对齐数学仿真”？模拟结果只能标记 `PASS/simulation`，不能替代 HIL。

## 用户目标与不可变约束

- 产品是一个独立的“双 UMI + Ego”三设备系统，不是 D405-MAXIMU 的子项目。
- 正确拓扑：一台 2UQ2 Ego、两台 D405、两个分别连接左右 D405 的独立外部 IMU；三套设备由同一个人同时佩戴采集。
- Ego 是购买的 2UQ2 视频/IMU 产品，遵循原厂文档和 XU SDK；UMI 是 D405 加外部 IMU。
- 用户的时间同步判断：只要各相机图像映射到 PC 的 `global_time` 一致，并且每台设备内部 IMU 与图像已经对齐，三设备时间轴就完成统一。实现必须保留可审计的原始时间戳、映射参数和残差，不能只保留换算结果。
- D405 上电初期可能有坏帧，必须预热和健康过滤；2UQ2 同样必须预热。
- 世界原点取自动起录之后第一帧健康的 Ego SLAM 位姿，并按重力方向对齐；不能把预热坏帧或启动瞬间的不稳定帧作为原点。
- 量产采集不能要求用户每次携带标定板，也不能在产品各面粘贴影响外观的 Tag。
- 后续空间方案应围绕 Ego 主轨迹、无标记双 UMI 6D 观测、左右各自 VIO 增量和左右独立因子图设计；尚未完成真实数据验证。
- 唯一一键入口在联合健康门通过后自动开始录制，不进行第二次点击；结束或故障后自动封存并运行离线索引/校验。
- 正式交付分支统一使用 `main`。
- 任何没有真实硬件证据的检查必须保持 `BLOCKED/device_hardware_unavailable`，不能伪造 HIL PASS。
- 原 D405 仓库、分支、发布目录和虚拟环境不得修改；需要复用时只把必要代码或二进制复制到本独立项目。
- 不要把登录密码、设备秘密或私人凭据写入仓库、交接文件或日志。

## 仓库与主机状态

- Windows 原始仓库：`D:\semg.claude\three-device-slam`。
- Ubuntu 正式工作区目标：`/home/robot/three-device-slam`。
- 正式分支：`main`。
- 迁移前 Windows `main` HEAD：`1b7a6105882734177a0b65871fde02f19c63bdac`；本交接文件会形成一个后续文档提交，因此 Ubuntu 上应以迁移后的 `main` HEAD 为准。
- 当前没有配置 GitHub 远端；跨机器通过经过 SHA256 校验的 Git bundle 迁移完整历史。
- Windows 合并前的旧未跟踪启动副本保存在 `stash@{0}`，说明为 `preserve pre-main-merge bootstrap files 2026-08-26`；它不是当前正式实现。
- Ubuntu 22.04.5 / ROS 2 Humble 主机名为 `robot-MS-7E19`。
- Ubuntu 已有 VS Code 1.133.0 和 OpenAI Codex 扩展 `openai.chatgpt`。

## D405 隔离证据

- Windows D405 仓库：`D:\semg.claude\D405-MAXIMU`，任务结束时仍在用户原有分支 `firmware/esp32-s3-imu-encoder`，HEAD `33b0ad2c580b8ef53c5c48136d7262115d5ec556`；原有未跟踪文件未触碰。
- Ubuntu `/home/robot/ego_vio_humble` 仍链接到 `/home/robot/releases/ego_vio_humble/product_v1_20260824`。
- Ubuntu 正式 D405 发布仓库 HEAD `a7a143df9a138ada481e6c234b03803ab0cae837`，工作树干净。
- `three-device-slam` 的代码和启动脚本没有 D405 仓库或发布目录的运行时引用。

## 第一阶段已经实现

- 三设备会话生命周期、联合健康门、预热、自动起录、Ctrl+C/故障封存。
- 两个 D405+外部 IMU worker；`--serial` 和 `--imu-port` 均显式必填，禁止隐藏默认硬件身份。
- 单个 2UQ2 Ego worker 和 XU 桥接库。
- 时间域证据、帧/IMU 索引、离线一致性校验和质量报告。
- 产品配置解析与唯一入口 `run_three_device_slam.sh`。
- Ubuntu 安装器、只读环境验证器和独立安装目录。
- Shell 文件用 `.gitattributes` 固定 LF，避免 Windows checkout 破坏 Bash。

关键文件：

- `three_device_slam/acquisition/coordinator.py`
- `three_device_slam/devices/d405_umi/worker.py`
- `three_device_slam/devices/two_uq2/worker.py`
- `three_device_slam/quality/verify_session.py`
- `three_device_slam/synchronization/build_index.py`
- `scripts/install_ubuntu.sh`
- `scripts/verify_ubuntu_environment.sh`
- `run_three_device_slam.sh`

## Ubuntu 安装状态

- 产品：`/opt/three-device-slam`
- 配置：`/etc/three-device-slam`
- 会话：`/var/lib/three-device-slam/sessions`
- 独立 RealSense 输入：`/opt/three-device-slam-vendor/librealsense-rsusb-2.58.2/python/pyrealsense2.cpython-310-x86_64-linux-gnu.so`
- RealSense SHA256：`ec0089d1618732f298048b3f4bb4dccab99c95361f60ccd4ade8d78f1922b0a1`
- 2UQ2 SDK：`/home/robot/vendor/2uq2/YLX_XU_API_2026721`
- 已安装桥接库 SHA256：`09419497fbb2af4cb28374e607e5f6eb7b6b32629048aef0bcb32e44a6d63227`
- 安装器不会生成虚假 `/etc/three-device-slam/product.json`；缺配置时入口返回 exit 3 和 `BLOCKED/device_config_missing`。

## 验证证据

- Ubuntu 最终软件回归：`629 passed in 290.91s`。
- Windows 合并到 `main` 并修复 LF 后，WSL 回归：`629 passed, 1 skipped`；跳过项是 WSL 缺少 Gst introspection，Ubuntu 实机已执行并通过。
- `sudo ./scripts/install_ubuntu.sh` 首次和重复安装成功。
- `sudo ./scripts/verify_ubuntu_environment.sh` 返回 `PASS/ubuntu_environment`。
- YAML 5.4.1、OpenCV 4.5.4、独立 venv 内的 pyrealsense2、GStreamer 1.20.3 均实际导入。
- 缺产品配置的一键入口在打开硬件前安全阻塞。
- 独立复审最终结论：Critical 0、Important 0、Architecture CLEAR、可合并。

## 已知风险与不能重复的错误

- 不要在 GStreamer 探针中调用 `Gst.init()`；它可能创建或更新 registry，破坏只读验证契约。
- 绝对不要设置 `GST_REGISTRY=/dev/null`。实机证明 GStreamer 会用普通文件替换 `/dev/null`。该节点已恢复并验证为字符设备 major/minor `1:3`、模式 `0666`、大小 `0`。最终实现只导入 Gst 并读取版本。
- RealSense first-publish、坏目标拒绝和竞态 loser 清理仍缺更强的对抗性单测；实现与远端重复安装证据已通过，属于非阻塞测试加固项。
- D405 身份测试覆盖两参数同时缺失，但还可分别补充只缺 serial、只缺 IMU port 的测试。
- 当前没有真实三设备，也没有任何 Ego 历史录包，不能进入真实 HIL 或声称空间对齐有效。

## 当前阶段与建议的下一步

第一阶段软件为 `PASS/phase1_software`；真实三设备 HIL 和空间对齐仍阻塞。

无设备时建议分两步：

1. 先设计确定性三设备模拟器：生成一台 Ego、左右两台 UMI 的图像/IMU/位姿时间流，可注入预热坏帧、时间偏移、抖动、丢帧和故障，复用正式协调器、健康门、封存和离线校验。所有结果只标记 `PASS/simulation`。
2. 模拟器设计获用户批准后，再设计空间对齐数学仿真：合成已知真值的 Ego 主轨迹、左右 UMI 轨迹、重力方向和外参，验证首帧原点、重力对齐、无标记约束和左右独立因子图。

不要在用户批准设计前直接实现模拟器。新对话应先复述上述边界，然后只问一个问题：

> 是否确认先设计确定性三设备模拟器，再设计空间对齐数学仿真？
