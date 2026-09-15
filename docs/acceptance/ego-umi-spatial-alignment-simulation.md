# Ego 世界下的 UMI 空间对齐与确定性仿真方案

日期：2026-08-27

状态：数学链路已完成确定性仿真；真实 1 Ego + 2 UMI 标定、延迟测量和 HIL 尚未执行

## 1. 当前目标与边界

正式设备拓扑固定为 `Ego + left UMI + right UMI`。仿真和后续一键启动默认同时运行 `Ego -> left UMI`、`Ego -> right UMI` 两条相互独立的链路；单侧运行只用于诊断和故障隔离，不是产品默认模式。

本阶段验证：

- D405/VINS 已有的相机位姿到机身位姿转换方向；
- Ego 定义任务世界坐标系；
- UMI 任意原点 VIO 轨迹锚定到 Ego 世界；
- 相对观测时间延迟未校正时会造成可测误差，校正后应恢复；
- 左右 UMI 共享 Ego 世界，但各自保持独立时钟、外参、tag ID、锚点和健康状态。

本阶段不验证：

- 真实相机内参、相机—IMU 外参或 `td` 的准确度；
- USB、ROS 2/DDS、模型推理和设备时钟的真实延迟；
- 无标记 6D 检测器的精度；
- 遮挡后的真实 VIO 漂移或因子图收敛；
- HIL、量产精度或三实体设备同时运行。

因此唯一允许的通过结论是 `PASS/simulation`，不能写成 HIL PASS。

## 2. 坐标系和唯一变换约定

所有变换统一记作 `T_A_B`：把 B 坐标系中的点变换到 A 坐标系。

| 符号 | 含义 |
|---|---|
| `W` | 由 Ego 定义的任务世界 |
| `S` | Ego SLAM 初始化时的任意局部坐标系 |
| `E` | Ego 机身/跟踪坐标系 |
| `U_i` | 第 i 个 UMI 机身坐标系：`U_left`、`U_right` |
| `C_i` | 第 i 个 UMI 的 D405 相机坐标系 |
| `V_i` | 第 i 个 UMI VIO 的任意局部世界 |

相邻坐标系按消元顺序左乘，例如：

```text
p_A = T_A_B @ p_B
T_A_C = T_A_B @ T_B_C
```

D405 产品配置中的 `body_T_cam` 按 `T_U_C` 使用。若 VIO 给出相机位姿 `T_V_C`，机身位姿为：

```text
T_V_U = T_V_C @ inverse(T_U_C)
```

Ego 直接观察 UMI 相对位姿时：

```text
T_W_U(t) = T_W_E(t) @ T_E_U(t)
```

UMI VIO 原点任意。首次或重定位后的锚点为：

```text
T_W_V = T_W_E(t0) @ T_E_U(t0) @ inverse(T_V_U(t0))
```

后续轨迹传播为：

```text
T_W_U(t) = T_W_V @ T_V_U(t)
```

这个公式允许 UMI 在 Ego 暂时看不见它时继续靠自己的 VIO 桥接；重新看见后，再用新的相对观测约束漂移。当前仿真只验证无噪声单锚点闭环，不冒充完整因子图。

## 3. Ego 世界初始化

世界原点只由第一帧 `acquisition_time >= recording_start_ns` 且健康的 Ego SLAM 位姿定义，预热帧、到达时间和任一 UMI 首帧都不能定义世界。

初始化步骤：

1. 从预热 IMU/SLAM 得到 Ego SLAM 坐标系中的重力向量；
2. 将重力方向映射到世界 `-Z`；
3. 将选中 Ego 位姿的平移置零；
4. 从 2UQ2 跟踪帧契约显式读入 `forward_axis_in_ego`，将该物理前向轴投影到重力水平面并设为世界 `+X`，从而将初始偏航置零；不得默认所有跟踪帧都以机身 `+X` 为物理前向；
5. 若重力模长无效、前向轴近似平行重力或没有健康首帧，则 BLOCKED，不能静默使用单位阵。

## 4. 数据模型

### 4.1 仿真已实现模型

`PoseSample`：

| 字段 | 类型 | 约束 |
|---|---|---|
| `timestamp_ns` | int | 非负、按序列严格递增；语义为采样/曝光对应的公共时间 |
| `transform` | float64 `[4,4]` | 有限值、末行 `[0,0,0,1]`、旋转正交且行列式为 +1 |
| `healthy` | bool | 不健康样本不能作为锚点或插值端点 |

`RelativePoseObservation`：

| 字段 | 类型 | 约束 |
|---|---|---|
| `timestamp_ns` | int | 观测携带的时间戳，不等同 callback 到达时间 |
| `ego_from_umi` | float64 `[4,4]` | `T_E_U`，同样执行 SE(3) 校验 |
| `valid` | bool | 无效观测禁止进入锚定 |

`WorldInitialization`：记录世界初始化使用的 Ego acquisition timestamp 和 `T_W_S`。初始化函数还强制调用方提供 `forward_axis_in_ego`，使 2UQ2 实际轴约定成为可审计输入。

### 4.2 实机链路需要追加的记录字段

每条相机、IMU、相对位姿和 VIO 记录至少保留：

- `device_time_ns`；
- `acquisition_time_ns`；
- `arrival_time_ns`；
- 时钟模型 ID、标定 ID、设备序列号；
- `frame_id`、`child_frame_id` 和明确的变换方向；
- 原始四元数顺序（D405 当前为 `xyzw`）；
- tracking/healthy/lost/reset/relocalized 状态；
- 时间校正值、校正来源、估计时间和不确定度；
- 相对观测置信度、遮挡、几何退化和离群原因。

## 5. 时间模型与对齐时间轴

定义 UMI 相对观测的带符号延迟 `d_i`：

```text
t_acquisition = t_stamped - d_i
```

`d_i > 0` 表示观测时间戳比实际采样时刻晚。偏移也可能为负，因此实机标定必须测量符号，不能只测绝对值。callback/DDS 到达时间只用于诊断，不进入空间融合。

```text
预热                    正式录制                         离线/在线对齐
|----------------------|-----------------------------------------------|
估计重力/偏置/时钟模型  recording_start_ns
                        |
                        +-- 第一健康 Ego pose -> 建立 W
                        |
                        +-- UMI 图像/IMU -> T_V_U(t)
                        |
                        +-- Ego 观察 UMI，携带 t_stamped
                            |
                            +-- t = t_stamped - d_i
                            +-- 在 t 插值 T_W_E 与 T_V_U；超过 max_gap 则拒绝
                            +-- 计算独立 T_W_V_i
                            +-- 输出 T_W_U_i(t)
```

实机延迟测量建议分两层：

1. 采集层：验证每路设备时钟到 PC 单调时钟的线性映射、漂移、回绕、重连和残差；
2. 感知层：让 Ego 与 UMI 同时观察快速且可重复的运动事件，通过多次试验估计 `d_i`，报告均值、标准差、p95 和随时间漂移，而不是只保存一个最佳样本。

若固定偏移不能解释残差，则升级为 `d_i(t) = d0_i + alpha_i * (t - t0)`；在证据不足前不要引入更复杂的时变模型。

## 6. 确定性仿真设计和结果

仿真使用 20 ms 周期、2 s 长度的解析 Ego/UMI 6D 轨迹；最大允许插值间隔为 40 ms，Ego 前向轴在仿真配置中显式设为 `[1,0,0]`。每个 UMI 注入不同的任意 `T_V_W`、不同的 `T_U_C`，相对观测注入 60 ms 时间戳延迟。没有随机数，因此任何提交上的结果都可复现。

运行方式：

```bash
PYTHONPATH=. python3 -m three_device_slam.spatial.simulation
PYTHONPATH=. python3 -m three_device_slam.spatial.simulation --devices left  # 仅诊断
```

2026-08-27 双虚拟 UMI 结果：

| 指标 | left | right |
|---|---:|---:|
| `body_T_cam` 往返最大平移误差 | `4.97e-16 m` | `2.22e-16 m` |
| 60 ms 未校正轨迹平移 RMSE | `0.01154 m` | `0.01946 m` |
| 60 ms 未校正轨迹旋转 RMSE | `1.206 deg` | `0.327 deg` |
| 校正后轨迹平移 RMSE | `9.29e-16 m` | `8.86e-16 m` |
| `body_T_cam` 往返最大旋转误差 | `8.15e-15 deg` | `6.51e-15 deg` |
| 校正后轨迹旋转 RMSE | `4.62e-15 deg` | `3.46e-15 deg` |

该结果的含义是：公式、乘法顺序和延迟符号在无噪声条件下闭环；它不预测实机误差。

## 7. 故障注入清单

| ID | 注入项 | 当前自动化 | 预期结果 |
|---|---|---|---|
| S01 | 把 `T_U_C` 乘法方向写反 | 是，正确往返测试会失败 | FAIL/frame_convention |
| S02 | 非正交旋转或 det 不为 +1 | 是 | FAIL/invalid_se3 |
| S03 | UMI VIO 使用任意大平移/旋转原点 | 是 | 锚定后恢复 W 轨迹 |
| S04 | 相对观测时间戳延迟 60 ms 且不校正 | 是 | 误差必须显著超过阈值 |
| S05 | 使用正确带符号延迟 | 是 | 恢复到数值精度 |
| S06 | 录制开始后的首个 Ego 样本不健康 | 是 | 跳到下一健康 acquisition sample |
| S07 | 时间戳重复、倒退或查询超出范围 | API 已拒绝；部分专项测试待扩充 | FAIL/time_contract |
| S08 | 插值区间含 lost/unhealthy 或超过 `max_gap_ns` | 是 | BLOCKED/pose_gap |
| S09 | 相对观测无效或校正后时间为负 | API 已拒绝；需扩充专项测试 | FAIL/invalid_observation |
| S10 | 左链路 reset/relocalize | 尚未注入 | 只重建左 `T_W_V`，右链路不变 |
| S11 | USB 延迟突增、时钟漂移、计数器回绕 | 尚未注入 | 保留原始证据，超门限样本不可训练 |
| S12 | 重力为零或 Ego 前向与重力退化 | 实现已拒绝；需扩充专项测试 | BLOCKED/world_init_degenerate |
| S13 | 相对观测离群/错误 UMI 身份 | 尚未注入 | 鲁棒门拒绝，不污染另一条链路 |
| S14 | 长遮挡和 VIO 漂移 | 尚未注入 | VIO 桥接；重现后因子图收敛或 FAIL |

## 8. 分阶段验收标准

### A. 数学仿真（本阶段）

- 所有旋转满足 `R.T @ R ~= I` 且 `det(R)=+1`；
- D405 相机/机身往返误差 `< 1e-10 m`；
- D405 相机/机身往返旋转误差 `< 1e-7 deg`；
- Ego 原点误差 `< 1e-10 m`，重力角误差 `< 1e-8 deg`；
- Ego 物理前向轴必须显式提供，任何插值区间必须 `<= max_gap_ns`；
- 无延迟或正确校正后，UMI 世界轨迹平移 RMSE `< 1e-9 m`、旋转 RMSE `< 1e-7 deg`；
- 60 ms 未校正平移 RMSE `> 0.01 m`，证明测试具有辨识力；
- 默认同时运行左右两条虚拟 UMI 链路；单侧诊断与双链运行中的同侧结果一致，每个 `anchor_id` 独立；
- 输出必须为 `PASS/simulation` 且 `hil_status=NOT_RUN`。

### B. 标定后 1 Ego + 2 UMI HIL（下一阶段）

- 所有内外参和 `td` 具有唯一标定 ID、日期、输入哈希和可回放产物；
- 2UQ2 跟踪帧的物理前向轴经过实测并写入帧/标定契约；
- acquisition clock 映射连续，重连/回绕有明确段界；
- 重复延迟测量报告符号、均值、标准差、p95 和漂移；
- 静态测试中转换后的 UMI 速度接近零，不因 Ego 自运动产生伪运动；
- 已知刚体/重复轨迹闭环分别报告位置和角度误差，不用单一综合分数掩盖问题；
- 人为遮挡与恢复不会改变 Ego 世界，也不会静默重置 UMI 世界；
- rosbag、日志、配置、提交和质量报告能从同一 session 复现。

具体毫米/角度 HIL 门限应由标定靶或外部真值系统的测量噪声确定，不能由无噪声仿真反推。

### C. 一键三设备（后续）

已有产品入口目标仍是：

```bash
./run_three_device_slam.sh product
```

空间链路接入后，一次启动应按阶段运行：三设备联合预检和 barrier、不可变录制、公共时间索引、Ego SLAM、left/right 独立 VIO、两个唯一 tag 的相对观测、两条独立锚定/融合、统一质量报告。正式配置固定 `active_umis=[left,right]`；单侧配置只允许进入显式诊断模式，不能产生产品 PASS。

任何单 UMI 失败都必须带设备 ID；不得覆盖另一条链路的锚点或原始数据。离线空间阶段失败不能删除已封存 raw，会话必须支持从失败阶段重跑。

## 9. 开源实现参考及采用范围

### 9.1 HoMMI/iPhUMI 的真实三设备组合

重新核对论文和 iOS/Python 源码后，HoMMI 的组合是三台同时工作的 iPhone：left UMI、right UMI、head Ego。三台先通过 ARKit multi-device collaboration 在采集期间建立共享 AR 世界 `A`，而不是由 head 逐帧检测左右 tag 后再拼接世界。

iPhUMI 程序启用 `isCollaborationEnabled`，通过 MultipeerConnectivity 交换 `ARSession.CollaborationData`。选出的 host 发布单位矩阵 world anchor；非 host 收到共享 anchor 后调用 `setWorldOrigin(relativeTransform:)`。此后三台记录的 `frame.camera.transform` 都是同一 `A` 中的相机位姿：

```text
T_A_I_head(t)
T_A_I_left(t)
T_A_I_right(t)
```

离线 `align.py` 只求 left/right/head 共同时间区间并以 60 Hz 重采样，不再次做空间地图拼接。每台设备再使用自己的固定安装外参：

```text
T_A_E       = T_A_I_head  @ T_I_head_E
T_A_U_left  = T_A_I_left  @ T_I_left_U_left
T_A_U_right = T_A_I_right @ T_I_right_U_right
```

若产品要求由 Ego 首帧定义世界 `W`，则对共享 AR 世界整体做一次共同基变换：

```text
T_W_A = desired(T_W_E(t0)) @ inverse(T_A_E(t0))

T_W_E(t)       = T_W_A @ T_A_E(t)
T_W_U_left(t)  = T_W_A @ T_A_U_left(t)
T_W_U_right(t) = T_W_A @ T_A_U_right(t)
```

三条轨迹必须乘同一个 `T_W_A`，不能分别归零，否则会破坏左右手与头部之间的空间关系。

### 9.2 我们硬件上的临时 tag 替代层

2UQ2 Ego 和两个 D405 UMI 没有 ARKit collaborative map，因此不能原样复用三 iPhone 的共享世界建立程序。顶部临时 tag 是我们的 bootstrap/验证方案，不是 HoMMI 的原算法。对 `i in {left,right}`：

```text
T_W_U_i(t)
  = T_W_E(t)
  @ T_E_CE
  @ T_CE_Tag_i(t)
  @ inverse(T_U_i_Tag_i)

T_W_V_i
  = T_W_U_i(t0) @ inverse(T_V_i_U_i(t0))

T_W_U_i(t)
  = T_W_V_i @ T_V_i_U_i(t)
```

其中左右必须使用不同 tag ID、不同 `T_U_i_Tag_i`、不同延迟和不同 VIO anchor。tag 只提供身份和跨设备空间约束；没有安装外参、时间校正和 UMI VIO 时，tag ID 本身不能直接生成世界轨迹。

- [HoMMI/MoF](https://github.com/gsanpark/mof_hommi)：采用公共时间交集、按采样时间插值、显式坐标转换和单/双侧独立处理思想；不采用“时间很接近就直接覆盖另一设备时间戳”的捷径。
- [HoMMI 论文](https://arxiv.org/html/2603.03243v2)：明确说明使用两台夹爪 iPhone 和一台帽载 iPhone，并通过 ARKit multi-device collaboration 建立共享坐标系。
- [iPhUMI iOS 采集程序](https://github.com/real-stanford/iPhUMI/blob/main/ios_app/iPhUMI/DataCollection/ViewController.swift)：共享 collaboration data、host world anchor、world origin 和每帧 ARKit pose 的具体实现。
- [iPhUMI 三路离线对齐](https://github.com/real-stanford/iPhUMI/blob/main/python_package/iphumi/demonstration_processing/process_stages/align.py)：left/right/head 公共时间区间和 60 Hz 位姿重采样。
- [iPhone 到 TCP 外参](https://github.com/real-stanford/iPhUMI/blob/main/python_package/iphumi/demonstration_processing/utils/gripper_util.py)：实现 `W_T_I @ I_T_TCP`。
- [HoMMI 对齐实现](https://github.com/gsanpark/mof_hommi/blob/main/hommi/demonstration_processing/process_stages/align.py)：用于核对 `world @ relative` 的命名帧组合和有效区间处理。
- [HoMMI 时间同步实现](https://github.com/gsanpark/mof_hommi/blob/main/hommi/demonstration_processing/process_stages/timesync.py)：采用重复观测估计偏移并报告离散程度的证据模式。
- [Stanford UMI 标定脚本](https://github.com/real-stanford/universal_manipulation_interface/blob/d095ba9590df789df5189eea5ee7e431689038a6/scripts/calibrate_slam_tag.py)：用于核对 `T_slam_cam @ T_cam_tag`、时间关联、范围过滤和重复候选鲁棒选择。
- [iPhUMI](https://github.com/real-stanford/iPhUMI)：作为多独立设备共享世界坐标和可选第三设备的架构参考。

这些项目提供设计证据；本仓实现是窄接口的独立实现，没有从外部仓库复制源码。iPhUMI 内部用于测量夹爪宽度的手指 AR tags 与我们贴在 UMI 顶部的世界锚定 tags 用途不同。
