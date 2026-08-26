# three-device-slam 项目日志

## 2026-08-25

- 用户确认 three-device-slam 是双 UMI + Ego 系统的长期产品总仓库，而不是 D405-MAXIMU 的子项目或临时采集仓库。
- 用户选择干净提取方案：从 D405 正式产品代码复制必要能力到新仓，之后独立演进；原 D405 仓库不得修改，也不得成为运行时依赖。
- 已核实 D405 GitHub 正式交付线 main 与 release/humble-stm32-product-v1-20260824 同指向 a7a143df9a138ada481e6c234b03803ab0cae837。
- 用户确认目标运行环境为 Ubuntu 22.04 LTS / ROS 2 Humble，Windows 不作为产品运行环境。
- 用户确认唯一一键入口在联合健康门通过后自动开始录制，不要求第二次点击；结束后自动封存并运行离线处理。
- 用户确认 D405 和 2UQ2 都必须预热，世界原点取自动起录后第一帧健康 Ego SLAM 位姿并对齐重力。
- 用户确认量产不依赖每次拍标定板或外露 tag；空间链路采用 Ego 主轨迹、无标记双 UMI 6D 观测、各自 VIO 增量和左右独立因子图。
- 完整仓库设计已写入 docs/superpowers/specs/2026-08-25-three-device-slam-repository-design.md。

## 2026-08-26

- 第一阶段干净提取已完成：独立实现三设备会话、双 2UQ2 与 D405+外部 IMU 采集适配、联合健康门、预热、自动起录、Ctrl+C/故障封存、时间索引、离线校验和 Ubuntu 一键入口。
- 最终软件提交为 `b2ff517e6d8953957afa91fdd4461d1a7552d1c5`；Ubuntu 22.04 / ROS 2 Humble 干净 checkout 上 `615 passed`，安装器、只读环境验证器和缺配置安全阻塞入口均通过。
- 远程验收发现并修复 ROS nounset、首次 make clean、Ubuntu mawk SHA256 和 setuptools 59 包元数据兼容问题；证据见 `docs/acceptance/phase1-software-acceptance.md`。
- 原 D405 正式仓库未修改，`/home/robot/ego_vio_humble` 仍指向 `product_v1_20260824`。
- 远程机独立安装 `/opt/three-device-slam`；未创建虚假 `product.json`，未写入虚假设备身份或标定 ID。

当前状态：第一阶段软件验收 `PASS`。由于没有真实设备，三设备 HIL 验收保持 `BLOCKED/no_devices`；第一帧健康 Ego SLAM 位姿、重力对齐和无标记双 UMI 空间约束仍是后续空间阶段。

下一步：设备到场后按 Task 9 执行真实三设备一键采集验收；HIL 通过后再进入空间对齐和乌帮图系统阶段。
