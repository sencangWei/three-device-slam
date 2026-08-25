# three-device-slam 长期产品仓库设计

日期：2026-08-25

状态：对话设计已确认，等待用户审阅本文档

目标平台：Ubuntu 22.04 LTS、ROS 2 Humble、Linux x86-64

设备拓扑：一个人同时佩戴头部 Ego，并手持左、右两个 UMI

## 1. 产品目标

three-device-slam 是双 UMI + Ego 三设备系统的长期产品总仓库，不是 D405-MAXIMU 的功能分支，也不是只覆盖当前采集阶段的临时仓库。

仓库最终负责：

- 三设备身份识别、联合预热、健康门和不可变原始录制；
- 三路采集时间到同一 PC 单调时钟域的可验证映射；
- 公共 30 Hz 多视角索引；
- 2UQ2 离线双目惯性 Ego SLAM；
- 左右 D405 + 外部 IMU 的独立 UMI VIO；
- Ego 对左右 UMI 的无标记 6D 相对位姿观测；
- 两条独立手部因子图、共同世界坐标输出和数据集导出；
- 分层 PASS、FAIL、BLOCKED 质检证据；
- Ubuntu 产品安装、启动、发布和恢复。

设备组成固定为：

- Ego：2UQ2 双目相机及其内置 UVC IMU；
- 左 UMI：Intel D405 + 一路外部 IMU；
- 右 UMI：Intel D405 + 一路外部 IMU。

量产流程不要求每次携带或拍摄标定板，不要求规定启动姿态，也不依赖 UMI 外壳上永久外露的 AprilTag。EVT 阶段允许临时 tag 或外部测量系统提供真值，但它们不是产品运行依赖。

## 2. 仓库所有权与来源边界

### 2.1 所有权

- three-device-slam/main 是本系统正式产品交付线。
- D405-MAXIMU 继续独立维护和交付；新系统不得修改它、从它的工作目录运行、在运行时导入它，或把它作为 Git submodule。
- 新仓采用干净提取：复制必要的下层能力，并在新仓独立演进。
- 复制进入新仓的代码从复制时起由新仓负责测试、发布和兼容性，不要求向原仓同步回写。

### 2.2 冻结来源

| 用途 | 来源 | 冻结提交/证据 |
|---|---|---|
| D405 正式产品基线 | GitHub sencangWei/D405-MAXIMU 的 main | a7a143df9a138ada481e6c234b03803ab0cae837 |
| 三设备阶段一迁移参考 | feature/three-device-acquisition | 软件提交 de2b52f；文档 HEAD 15eafa0 |
| 2UQ2 厂商 Linux SDK | 目标机 /home/robot/vendor/2uq2/YLX_XU_API_2026721 | SDK 归档 SHA-256 9b1b9e9d4dac7c3a1df027d90c694791a3cfe3241643f67a4939697520eda41f |
| 当前 D405 时间参数 | 正式产品运行配置 | estimate_td=0，td=-0.009312 s，imu_lead_guard_ms=-6.812 |

2026-08-25 已验证 D405 GitHub main 与 release/humble-stm32-product-v1-20260824 同指向 a7a143d。新仓不得从本地陈旧 main、旧稳定目录或历史 td=-0.0117 s 版本提取产品代码。

厂商 SDK 是否能复制进 Git 仓库取决于其许可证。许可证未确认前，只提交自有桥接代码、版本和哈希校验，不提交厂商二进制或源码。

## 3. 运行环境与发布边界

正式运行环境固定为：

- Ubuntu 22.04 LTS；
- ROS 2 Humble；
- Linux x86-64；
- 当前验收主机 robot@192.168.113.224；
- 产品机已验收的 librealsense/ROS 基础环境；
- 2UQ2 原生桥接构建为 Linux x86-64 共享库。

Windows 只用于代码查看和开发辅助，不是产品运行依赖，也不作为最终 HIL 通过证据。

新仓 main 是正式交付来源，但产品机不直接运行可变的开发 checkout。正式发布应从 main 的冻结提交构建不可变 release 目录，经验收后原子切换 current 指针，并保留上一版本用于回滚。

## 4. 逻辑架构

计划中的职责结构如下：

~~~text
three-device-slam/
├─ devices/
│  ├─ d405_umi/          # D405、外部 IMU 和单设备 VIO 适配
│  └─ two_uq2_ego/       # 2UQ2 双目、UVC IMU 和 XU bridge
├─ acquisition/          # 身份、预热、联合屏障和不可变录制
├─ synchronization/      # 公共时钟证据和 30 Hz 联合索引
├─ slam/
│  ├─ ego/               # 2UQ2 离线双目惯性 SLAM
│  ├─ umi_vio/           # 左右 UMI 独立 VIO
│  └─ fusion/            # 左右独立因子图与共同世界输出
├─ tracking/
│  └─ umi_markerless/    # Ego 到左右 UMI 的无标记 6D 观测
├─ quality/              # 分阶段验收与证据
├─ configs/              # 设备身份、标定和产品配置
├─ scripts/              # 安装、发布和诊断入口
└─ run_three_device_slam.sh
~~~

这是职责设计，不要求实现时为了目录外观机械搬运代码。实施计划应优先保持最小、可测试的模块边界。

### 4.1 D405 双实例约束

两台 UMI 必须由序列号和外部 IMU 稳定设备路径明确绑定为 left 和 right。两套实例必须具有独立：

- ROS namespace 和 topic；
- 进程锁；
- 配置和标定 ID；
- 录制目录、日志和健康状态；
- VIO 初始化、lost、reset、stale 状态。

不得复用原单设备产品中的全局锁、固定 topic 或共享输出目录来假装支持双实例。

## 5. 一键运行语义

正式日常入口为：

~~~bash
./run_three_device_slam.sh product
~~~

单次启动完成以下链路：

~~~text
设备与环境预检
  -> 三路同时启动并保存预热原始数据
  -> 联合连续健康门
  -> 自动进入正式录制
  -> Ctrl+C 或设定时长结束
  -> 原子封存原始数据
  -> 公共时间索引
  -> Ego 离线 SLAM
  -> 左右 UMI VIO
  -> 无标记 6D 观测
  -> 左右独立因子图
  -> 数据集导出和分层质检
~~~

联合健康门通过后自动开始正式录制，不要求操作者第二次点击。Ctrl+C 必须触发有界、可观测的安全退出，不得直接遗留真假不明的会话。

离线阶段失败不得删除或改写原始数据。修复算法后必须能够从失败阶段重跑，而不要求重新采集。

## 6. 会话与不可变数据契约

每次任务生成一个自包含会话：

~~~text
sessions/<session_id>/
├─ manifest.json
├─ raw/
│  ├─ ego_2uq2/
│  ├─ umi_left/
│  └─ umi_right/
├─ sync/
│  └─ common_30hz.*
├─ trajectories/
│  ├─ ego_slam.*
│  ├─ umi_left_vio.*
│  ├─ umi_right_vio.*
│  └─ fused.*
├─ quality/
└─ exports/
~~~

manifest 至少记录：

- session_id、任务起止和状态；
- 三台相机、两路外部 IMU 的稳定身份；
- 固件、标定 ID、外观模型 ID；
- 仓库提交、dirty 状态、产品 release；
- Ubuntu、内核、ROS、librealsense 和桥接版本；
- 使用的配置和原始文件哈希；
- 每个处理阶段的状态、版本、输入和输出。

raw 一经封存即不可变。同步、SLAM、跟踪、融合和导出均为可重建派生物，不能覆盖 raw。

每条传感器记录保留：

- 设备原始时间；
- 与采样或曝光关联、映射到 PC 单调时钟域的 acquisition_time；
- 数据抵达采集进程的 arrival_time；
- 帧号、2UQ2 24 位累计序号或 IMU counter；
- CRC、解析、坏帧、重连和丢弃状态；
- 标定和设备身份。

arrival_time 只用于 USB 和调度抖动诊断，不能替代 acquisition_time 做三设备同步。

## 7. 时间同步模型

三设备时间对齐的定义是三路 acquisition_time 均可验证地映射到同一个 PC 单调时钟域，不是三张图时间戳数值完全相等，也不是三相机硬件同曝光。

- 每台 D405 单独维护设备时钟到公共 PC 时钟的连续映射，并使用自己的相机/IMU td_i。
- 2UQ2 按产品文档使用同帧双目、UVC XU 27 字节 IMU 和 24 位累计序号关系。
- 若某一路只能获得 USB 到达时间，或时钟映射证据不成立，该段为 BLOCKED/common_clock_unverified。
- 公共数据集默认采用 30 Hz 时间网格并保留三路真实 acquisition_time 和 skew。
- 三图 acquisition_time 最大跨度目标不超过 10 ms，硬上限为 16.7 ms。
- 超出硬上限的样本保留证据，但 trainable=false。

以上解决共同时间轴，不自动解决空间坐标对齐。

## 8. 预热、自动起录与世界原点

状态机为：

~~~text
COLD -> PREFLIGHT -> WARMING -> JOINT_READY -> RECORDING -> SEALED -> OFFLINE_QC
~~~

首轮 HIL 冻结条件：

- 至少预热 5 s；
- 最近连续 3 s 三设备全部健康；
- 最长等待 30 s；
- 超时为 BLOCKED/warmup_timeout，不进入正式录制。

D405 和 2UQ2 的预热原始数据都保存并标记 warmup=true，用于诊断、重力和偏置估计，但不进入任务轨迹和训练样本。

JOINT_READY 通过后，启动器自动写入 recording_start_ns 并进入 RECORDING。任务世界 W 定义为离线处理中第一帧 acquisition_time 不早于 recording_start_ns 的健康 Ego SLAM 位姿：

- 原点平移置零；
- 偏航置零；
- 重力方向由预热段 IMU/SLAM 对齐；
- 保留重力确定的俯仰和横滚。

不得使用任一设备第一张原始图或预热坏帧定义世界原点。

## 9. 空间对齐与融合

采用 Ego 主轨迹方案：

~~~text
2UQ2 双目 + UVC IMU -> 离线双目惯性 SLAM -> T_W_E(t)
2UQ2 双目观察左 UMI -> 无标记 6D Z_E_L(t) --+
左 D405 + 外部 IMU -> VIO 相对增量 DeltaT_L --+-> 左因子图 -> T_W_L(t)

2UQ2 双目观察右 UMI -> 无标记 6D Z_E_R(t) --+
右 D405 + 外部 IMU -> VIO 相对增量 DeltaT_R --+-> 右因子图 -> T_W_R(t)
~~~

左右 UMI 的 VIO 原点任意，只使用局部相对增量，不能直接当成 W 中的绝对位姿。Ego 无标记观测把两条手部轨迹锚定到共同 W；短时遮挡由各自 VIO 桥接，重新出现后使用鲁棒约束校正漂移。

左右手图首版独立优化，共同引用 T_W_E。只有实测表明 Ego 漂移是主要误差源时，才升级为同时优化三条轨迹的联合图。

量产无标记观测使用 UMI CAD、双目几何和正常产品外观中的不对称几何或纹理。不得预设 2UQ2 一定提供彩色图像；特征方案以实际彩色或灰度输出能力为准。

人体运动范围和左右身份连续性只可作为异常检测或弱先验，不能在视觉和 VIO 都不可用时凭空恢复 6D 真值。

## 10. 状态和故障语义

每一层只输出 PASS、FAIL 或 BLOCKED：

- BLOCKED：缺少设备、标定、许可证、共同时间证据、真值阈值等前置条件，算法结果不可被合理判定；
- FAIL：前置条件满足，但采集、计算或验收明确失败；
- PASS：该层全部必需检查具有证据且通过。

分层状态至少包括：

- preflight；
- acquisition；
- timing；
- ego_slam；
- umi_left_vio、umi_right_vio；
- markerless_left、markerless_right；
- fusion；
- spatial_accuracy；
- overall。

overall 只有所有必需层均 PASS 时才为 PASS。采集和时间 PASS 不得被宣传为三设备空间系统 PASS。

关键处理：

- 启动前设备缺失或身份冲突：BLOCKED/device_identity；
- 预热超时：BLOCKED/warmup_timeout；
- 录制中设备掉线、时间回退或磁盘写失败：封存已有原始数据并 FAIL；
- 单个坏帧：标坏并跳过，不伪造数据；
- UMI 短时不可见：由该 UMI VIO 桥接；
- UMI VIO reset 且 Ego 不可见：记录轨迹断点，不伪造连续位姿；
- Ego SLAM 在核心区间无法恢复：相关空间输出 FAIL；
- 离线阶段崩溃：raw 保持可重放，阶段可恢复执行。

## 11. Ubuntu 安装、测试与验收

首次安装入口计划为 scripts/install_ubuntu.sh，负责确定性地检查或构建依赖。日常运行不得要求操作者手动 source 多个工作区、手工启动多个终端或修改设备路径。

验收分四层：

1. 无硬件测试：单元测试、数据契约、状态机、故障传播和录制回放。
2. 60 秒三设备联合 HIL：频率、抖动、帧/计数器连续性、时钟证据、三图跨度、队列和持续写盘。
3. 稳定性与故障注入：长时间任务、拔插、进程崩溃、磁盘不足、重复启动和安全退出。
4. EVT 空间真值：临时 tag 或外部真值覆盖距离、快速运动、遮挡、手交叉、弱纹理和重新出现，冻结 DVT/PVT 空间阈值。

当前无三设备硬件，因此只能复用既有软件证据；以下项目保持 BLOCKED/device_hardware_unavailable：

- 真实三设备频率、抖动和掉帧；
- D405 global_time 映射的双机联合实测；
- 2UQ2 XU 与视频帧的硬件关系；
- 持续写盘和长时间稳定性；
- Ego SLAM、无标记 6D、融合和空间精度。

跳过的测试必须记录为 SKIPPED 或 BLOCKED，不能计入通过。

## 12. 干净提取规则

实施迁移必须遵守：

1. 先从 D405 GitHub 正式 main 的 a7a143d 提取经过产品清理的最小代码集合。
2. 再逐项审阅 de2b52f 中的三设备功能，只迁入与新架构直接相关的修改。
3. 每个迁入文件记录原仓、提交、原路径、许可证和迁入后路径。
4. 不复制历史 release、旧运行时、客户数据、录制数据、构建产物或用户未提交文件。
5. 不更改 D405-MAXIMU 的 main、feature 分支、工作树或未跟踪文件。
6. 新仓测试不得通过相对路径访问原 D405 仓库。
7. 在独立 Ubuntu checkout 中证明新仓可安装、测试和启动。

干净提取意味着新仓拥有新的 Git 历史；来源通过文档和逐文件 provenance 保留，而不是通过 submodule 或整仓历史复制保留。

## 13. 分阶段实现边界

后续实施计划按依赖顺序拆分：

1. 建立新仓最小工程、来源清单和 Ubuntu 测试入口；
2. 提取三路不可变采集、联合健康门和同步索引；
3. 完成 D405 双实例 namespace、锁、日志和健康隔离；
4. 验收 2UQ2 bridge、视频/XU 关系和 60 秒联合 HIL；
5. 完成 2UQ2 离线双目惯性 Ego SLAM；
6. 建立 EVT 临时真值链和实际外观测试集；
7. 实现左右 UMI 无标记 6D 观测；
8. 实现两条独立因子图、30 Hz 数据集和空间质检；
9. 冻结量产阈值并完成 Ubuntu 端到端发布验收。

每一阶段必须有可重复的输入、命令、预期证据和停止条件。前一阶段失败时，不用后续看似合理的轨迹掩盖基础采集或时间错误。

## 14. 明确非目标

首版不做：

- 三台相机硬件同曝光；
- 三个完整地图的实时协同建图；
- 每次任务拍标定板；
- 量产外露 AprilTag；
- 指定启动动作；
- 在 EVT 真值实验前承诺毫米级精度；
- 在任意长遮挡下承诺无漂移；
- 修改或合并回 D405-MAXIMU；
- 为 Windows 建立正式产品运行线。

## 15. 设计完成标准

本文档经用户审阅批准后，设计阶段完成。下一步必须先编写独立实施计划，明确逐文件提取顺序、测试先行策略、Ubuntu 验证命令和每阶段提交点；在该计划再次确认前不迁移产品代码。

## 16. 参考资料

- HoMMI paper: https://arxiv.org/html/2603.03243
- iPhUMI: https://github.com/real-stanford/iPhUMI
- HoMMI alignment code: https://github.com/gsanpark/mof_hommi
- HiFi-UMI paper: https://arxiv.org/html/2607.25895
- FoundationPose: https://github.com/NVlabs/FoundationPose
- InfraredTags（未来备选，不是首版依赖）: https://hcie.csail.mit.edu/research/infraredtags/infraredtags.html
