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

当前状态：用户已批准仓库设计。仅建立独立本地 Git 仓库、设计和实施计划，尚未迁移或修改任何产品代码，尚未创建远端 GitHub 仓库。

下一步：按 docs/superpowers/plans/2026-08-25-acquisition-sync-clean-extraction.md 执行第一阶段干净提取；执行方式由用户选择。
