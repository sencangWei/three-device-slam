# D435i 临时 Ego 三设备方案 · 交接总纲（2026-09-10）

> 本文是给接手人的**入口文档**：读完后能知道项目在哪、结论是什么、坑在哪、下一步干什么。
> 每日细节在 `progress.md`（608 行，按日期），空间标定完整推导在
> `docs/handoff_spatial_alignment_20260905.md`（1328 行，内部交叉引用 §49/§62），
> 本文不重复抄录，只做索引和收口。

---

## 0. 一句话现状

**"临时方案"使命已基本完成**：用 D435i 当替身 Ego，把三设备 SLAM 的
**采集、时间同步、索引质检、空间标定四条链全部打通并验收**；
最终安装外参已收官（z 悖论闭环，norm 46.2mm / z −43.4mm，与物理链闭合 0.2mm）。
**下一步的主线是接入 2UQ2 产品 ego 相机**（09-08 已完成两台 2UQ2 标定交付），
D435i 退役后仅作参考。

---

## 1. 项目目标与方案定位

**三设备 SLAM**：Ego（头部第一人称相机）+ 双手 UMI（ wrist 相机）+ SLAM 后端。

| 角色 | 临时硬件（本方案） | 产品形态 |
|---|---|---|
| Ego | Intel D435i（S/N 327122078613） | **2UQ2** 双目+IMU 一体机（1bcf:0b15） |
| UMI 左手 | D405（S/N 260322279785）+ 外置 IMU（CP2102N 串口，stm32_combined_v1，400Hz） | 同左（UMI 硬件不变） |
| UMI 右手 | **缺**（产品入口硬要求，当前 BLOCKED） | 待接 |
| 后端 | VINS/COVINS-G（未接入完成） | 同左 |

**为什么用 D435i 临时代替 2UQ2**：2UQ2 产品相机到货晚/需标定，D435i 有现成
双目+IMU 且 SDK 适配代码已在仓库（`three_device_slam/devices/d435i_ego/`），
先把整条软件链跑通。D435i 的所有标定结论**不能**直接搬到 2UQ2（不同传感器），
但流程、工具链、验收方法全部复用。

**09-08 变量**：两台 2UQ2 已标定交付（`~/桌面/2UQ2_标定交付_20260908.tar.xz`，
含工具包+参数+留出极线验收）。产品 ego 切换已无任何标定侧阻塞。

---

## 2. 硬件清单与"认设备"方法

| 设备 | SDK 序列号 | sysfs/lsusb 序列号 | 备注 |
|---|---|---|---|
| Ego D435i | **327122078613** | 349643060321 | ⚠️ 两者不一致是 RealSense 常态 |
| UMI D405 | **260322279785** | 260423073555 | 同上 |
| 外置 IMU | CP2102N by-id：`/dev/serial/by-id/usb-Silicon_Labs_..._c48df736...-if00-port0` | — | 协议 stm32_combined_v1，400Hz |

**铁律：查设备归属以 `pyrealsense2.query_devices()` 为准，永远别信 sysfs serial。**

### 2.1 已知的硬件级缺陷（D435i 这台）

1. **IR 双目整体虚焦**（Laplacian var ~22 vs color 的 320）——40mm tag 在 0.47m
   仅 ~55px 且全解码失败。所以空间标定全程走 **color 传感器**。
2. **factory IR 双目标定自带 +1.01% 标度误差**（s=1.0099~1.01014，20 帧点阵相似
   拟合，板/间距/背码三处一致）。影响一切 IR 度量/深度用途。color 无此问题。
3. RGB 存储的畸变系数**全零 = 名义占位**（9 种分辨率都零），不能直接用
   （见 §5 决策 k1k2k3）。

### 2.2 固定流契约（D435i）

- 左/右红外：`ego.ir_left` / `ego.ir_right`，Y8，`1280x720 @ 30Hz`
- 陀螺/加速度：`ego.gyro` / `ego.accel`，float32 XYZ，`200Hz`
- 每条记录保留 RealSense `global_time`、映射到主机单调钟的 `acquisition_ns`、
  `arrival_ns`、帧号、CRC。左右图按帧号配对，采集时刻差 ≤1ms。

---

## 3. 运行环境与运维坑（接手先读这条）

仓库：`/home/robot/three-device-slam`，分支 main。**工作树有 ~179 个未提交文件**
（09-04 以来全部成果），用户明确要求**不得擅自提交**，交接后第一件事该是做提交整理。

### 3.1 怎么跑（唯一正确姿势）

```bash
cd /home/robot/three-device-slam   # 必须先进仓库根目录
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. \
  /opt/three-device-slam/venv/bin/python -B -m three_device_slam.cli product ...
```

- ⚠️ **`run_three_device_slam.sh` 的 `-I` 隔离模式会让 /opt venv 里的旧 site-packages
  拷贝优先**（没有单 UMI 支持等新代码），别用它跑产品入口。
- ⚠️ **venv 的 pip shebang 是坏的，且 site-packages 归 root**；同步新代码到 /opt
  需要 sudo rsync——曾被权限策略拒绝。**变通：一律从仓库根目录 + PYTHONPATH=.
  跑仓库代码**（如上），不依赖 /opt 里的拷贝。

### 3.2 周期性掉帧（每次重开机都要重做！）

症状：两台相机每 ~1.15s 同步出现 66ms 掉帧。根因：USB autosuspend(2s) + CPU
powersave。修复：

```bash
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend
# 所有 usb 设备的 power/control 设 on；cpupower frequency-set -g performance
```

**重开机即失效**，建议写进开机脚本。

### 3.3 测试

`cd /home/robot/three-device-slam && /opt/three-device-slam/venv/bin/python -m pytest tests/`
全量 800+（最后记录 872）个测试，每次改动后必须全绿。

---

## 4. 系统架构与数据契约

```
┌─ 采集层    acquisition/   rsusb_pair（单 owner 模式，一个控制线程按 ego→left→right
│            启动/反序停止；左右外置 IMU 各自串口线程，不拥有 RealSense 设备）
├─ 同步层    synchronization/clock_mapping.py  ← 连续时钟映射（见 §5.1）
├─ 索引层    sync_index（pair 拓扑，schema ego.two_device.sync_index.v1，
│            build_pairs 中点采样 → 30Hz 索引）
├─ 质检层    quality/verify_pair_session.py（schema ego.two_device.acceptance.v1，
│            按 capture_topology 自动选质检器）
├─ 空间层    spatial/mount_calibration.py、spatial/pair_tag_alignment
└─ 设备层    devices/d435i_ego/（临时 ego）、devices/d405_umi/、devices/two_uq2/（产品 ego）
```

关键 schemas：`ego.d435i.acceptance.v1`（单机）、`ego.two_device.sync_index.v1`、
`ego.two_device.acceptance.v1`、`ego.two_device.mount_calibration.consensus.v1`、
`ego.realsense.extrinsics_read.v2`。

**RSUSB 所有权约束**（实测 A/B 钉死）：D435i 与 D405 不能由不同进程/线程分别
拥有 RSUSB 生命周期，必须单 owner 单控制线程。

**产品入口约束**：`run_product_capture` 硬要求 left+right 双 UMI，
缺 right 直接 `BLOCKED/required_hardware_unavailable`——单 UMI 会话走
`rsusb_pair` 直跑，不许把单 UMI PASS 冒充三设备 PASS。

---

## 5. 已完成进度（按时间线）

### 5.1 时间同步 ✅（09-03 ~ 09-04）

- 新模块 `synchronization/clock_mapping.py`：`DeviceClockTracker`——启动截距 +
  按设备用帧序号最小二乘拟合设备钟速率误差（对丢帧稳健、与 USB 到达延迟无关）。
  每台 UMI 独立 tracker（只喂 ir_left 参考流），每帧 metadata 带
  clock_rate_ppm/anchor/samples 审计证据；Ego 用固定映射
  （D435i 漂移 ~1.2ms/30s 在预算内）。
- 证据 `artifacts/product_sessions/three_device_20260904T114327.548871Z_0868eae8`：
  sync index **901/901 全 trainable**（之前 727/900），span p50 7.4ms / max 11.6ms，
  UMI 钟差收敛 −439ppm。
- 更早 run2（20260903）：双 IR 30.01Hz、Ego IMU 200Hz、UMI IMU 400Hz、
  跨机最近帧 p50 0.47ms / max 5.79ms。

### 5.2 索引与质检 ✅（09-04）

- config.py 单 UMI 支持（right 可缺省，仅 d435i ego）；coordinator 报告
  `capture_topology: single_umi`；`sync_index` pair 拓扑；
  新质检器 `quality/verify_pair_session.py`。
- 端到端 `three_device_20260904T110715`：acquisition/calibration/index/verification
  全 PASS，`overall=BLOCKED` 是正确语义（当时空间层未交付）。

### 5.3 空间标定 ✅（09-04 深夜 ~ 09-07，主线战斗）

**路线**（每一步都有独立证据）：

1. **09-04 深夜**：发现 IR 糊/color 锐 → 弃 IR 走 color。坑：tag 贴黑支架检测
   不到外框（要白纸基底露 ≥10mm 白边，规范写入 `assets/calibration_tags/README.md`）；
   OpenCV 的 36H11 字典极性与官方相反（反色才解得出）。
   证据 `artifacts/spatial_bench/ego_color_tag_20260904T232857`：601/601 帧 100%
   检出，T_ego_color_tag 平移 rms **0.25mm**，重投影中位 0.39px——两设备刚性
   共空间得证。
2. **09-05 mount_calibration 闭环**：`scripts/pair_color_tag_capture.py`（双 color
   共采 + 各自 calibration.json 含工厂 color→ir_left）+
   `spatial/mount_calibration.py`（公共外 tag 共视：
   T_ego_umi = T_ego_ext·inv(T_umi_ext)）。
   坑修复：D405 YUYV 以 uint16 返回要 `.view(uint8)`；暗场景手动
   exposure=30000µs+gain=96（33000µs 会掉 15fps）；工厂外参 det=1+3e-8 需 SVD 投
   SO(3)；OpenCV 把外 tag 误检 id=1 → 中位数/MAD 平移门 + Markley 四元数均值。
3. **z 悖论（09-05~09-07）**：卡尺 48.4mm vs 算法共识 z=−61.3mm，差 14.6mm。
   逐项排除（标度/旋转/IPPE 四组合/代码审查全绿）后真凶有二：
   - **Ego RGB 零畸变 = 名义占位**（LOSO 交叉验证否决 factory 零 D：
     板 p95 ~1.0px、mount 外推 26.5px）；
   - 旧单外 tag 链弱观测（ego 看外 tag 仅 49px@0.75m，旋转 rms 17°）。
4. **最终决策（`scripts/ego_rgb_model_selection.py`，报告
   `artifacts/spatial_bench/mount_z_decision_run1/report.json`）**：
   采用 **k1k2k3** 内参替代 factory 零 D 占位：

   | 参数 | 值 |
   |---|---|
   | K | fx 892.6243 / fy 892.8378 / cx 664.9773 / cy 376.3447 |
   | D | [0.0787, 0.0179, 0, 0, −0.5453] |
   | 依据 | 联合 12 帧板+背码，板最差 p95 0.78px、mount 外推 2.7px |

   **最终答案：norm 46.2mm / z −43.4mm**（对 Codex 壳体物理链 46.4 闭合 0.2mm；
   对卡尺玻璃链 48.4 差 2.2mm——口径差属测量技术差）。k1k2 备选 45.3/−42.5。
   关键数学事实：标定点缩放不改变 calibrateCamera 目标 → k1k2k3 对 IR +1% 标度
   误差天然免疫，fx 892.62 就是真度量值。
5. **09-07 深夜亚mm重复性**：`scripts/dual_ir_color_submm_capture.py` +
   `dual_ir_color_submm_analysis.py`，跨位姿 z 差 **0.22mm** 达标；s_ego=1.0099
   复现 +1% IR 标度误差。⚠️ 该链绝对值偏 ~6mm（在物理带外）——定位为
   ego.color 85px 小背码 IPPE_SQUARE 系统性角点偏置（0.42m 处 1px≈5mm）。
   **结论：绝对值以联合链 norm 46.2 为准，该会话 = 重复性证据 + 独立交叉验证。**
   报告 `artifacts/spatial_bench/submm_final_2pose/`。

**mount 标定共识文件** `artifacts/spatial_bench/mount_calibration_umic_consensus_20260905.json`：
`T_umi_ir_left_from_mount_tag` = t[0.01956, −0.004426, −0.061342]m、
rpy[−179.531°, 1.727°, 179.17°]（tag 在 UMI 左 IR 后方 6cm、朝后，物理合理）；
同几何连采两会话重复性 ≤6.3mm / ≤1.7°；z=−0.061 三次会话一致。
⚠️ 重复性必须**同几何连采**（动过 UMI 再比，差 24mm 是杠杆臂变了不是漂移）。

### 5.4 2UQ2 产品相机标定 ✅（09-08，在 ego_vio_calib_kit 会话完成）

- 交付 `~/桌面/2UQ2_标定交付_20260908.tar.xz`：两台 2UQ2 参数（ego.2uq2.calibration
  .training.v1）+ 全套复标工具。相机1 基线 73.10mm / 极线 p95 0.855 PASS；
  相机2 初检"右半幅偏软"实为**出厂调焦环位置不对**，转环调焦后重解
  基线 72.48mm / p95 0.761 PASS。
- 关键坑（复用时必读交付包 README）：kalibr 焦点初始化要求单目整张 6×6 板全见
  （36 角点缺一即 skip → NaN）；XU 桥接库必须用单查询新版（旧库双查询卡 102Hz）；
  极线误差对 fx 一阶敏感，候选解仲裁必须用独立留出集。

### 5.5 未动 ❌

- **动态空间对齐**：所有空间证据都是静态刚体；"移动 UMI + sync index 对齐
  tag 观测与 UMI 帧"从未跑过。
- **SLAM 后端**：VINS 889 位姿@30Hz 跑通但静态重跑漂 1.19–2.40°（边缘化先验+
  OpenMP，限线程后仍 2.05°，未定位）；COVINS-G 只在准备中
  （`docs/architecture/ego-umi-covins-integration.md`）。
- **双 UMI**：产品入口硬要求，硬件缺右手。
- **2UQ2 接入 three-device-slam**：`devices/two_uq2/` 已有 capture/calibration_
  capture/kalibr_export/rate_probe/worker，但没接 coordinator/run_product_capture，
  时间同步语义（IMU 时间戳）未对表。

---

## 6. 已知难点与坑清单（完整版）

### 未解决的技术难点
| # | 难点 | 现状 | 修偏方向 |
|---|---|---|---|
| 1 | ego.color 85px 背码 IPPE 角点系统偏置 | 绝对值偏 ~6mm 的已定位根因，未修 | 换大 tag / ego 拉近 / 逐角点中值 + IPPE 双模式对照 |
| 2 | D435i factory IR +1% 标度误差 | 硬件级，软件免疫（k1k2k3 不受影响） | 换产品相机即消失；用 IR 深度时记着 ×1.01 |
| 3 | Ego color pitch 1–2° 会话噪声 | mount 链最弱一环 | 多会话中值；2UQ2 切换后自然消失 |
| 4 | VINS 静态漂移 2° | 未定位 | 见 task_plan；COVINS-G 绕开 |
| 5 | 双 UMI 缺硬件 | BLOCKED | 等硬件 |

### 运维坑（已趟过，别重复交学费）
- venv 同步要 sudo（被拒）→ 一律仓库根目录 + PYTHONPATH=. + `-B` 跑
- USB autosuspend 修复重开机失效
- sysfs 序列号 ≠ SDK 序列号
- OpenCV 36H11 字典极性与官方相反（反色解码；2UQ2 验收脚本里已内置）
- tag 必须白纸基底 ≥10mm 白边
- D405 YUYV uint16 视图坑；手动曝光 30000µs/gain 96
- 重复性验证必须同几何连采
- 所有"闭合"先统一口径（norm/轴向/平面距；left-IR 光心 vs color 光心）
- **工作树 179 未提交文件**；**任何工厂标定值不得覆盖**（activation 一律 NOT_ACTIVATED）

---

## 7. 接下来的任务（按优先级）

### P0 · 2UQ2 接入 three-device-slam（产品化的关键一跃）
- 入口：`devices/two_uq2/`（已有 5 个模块）+ `acquisition/coordinator.py` +
  `config.py`（把 two_uq2 作为 ego device 注册）。
- 要做：①run_product_capture 支持 two_uq2 ego 身份；②时间同步——2UQ2 IMU
  时间戳语义（主机单调控制传输中点）与 D435i global_time 完全不同，需要新的
  clock_mapping 适配或固定映射验证；③sync_index 已认 `ego.video/ego.xu` 契约
  （2UQ2 本来就是产品 ego），核对即可；④用 09-08 交付参数做验收基线。
- 验收标准：一次 product 会话 acquisition/index/verification 全 PASS，
  索引 trainable 率 ≥99%。

### P1 · 动态空间对齐验证
- 方案：移动 UMI 采 60s，sync index 把 ego.color 上的 tag 观测与 UMI 帧对齐，
  比较动态链 T_ego_umi 与静态共识 46.2mm 的差异。
- 入口：`spatial/pair_tag_alignment.py` + `synchronization/clock_mapping.py`。

### P1 · 工作树提交整理
- 179 个文件分批 commit（先 tests/ + 单 UMI 支持 + clock_mapping + spatial，
  再 scripts）。**需用户明确批准后执行。**

### P2 · 背码角点偏置修偏
- 换 150mm 大 tag（资产已生成 `assets/calibration_tags/umi_left_tag36h11_id0_150mm.png`，
  注意 UMI 背面可能贴不下——需用户确认）或 ego 拉近，重采 dual_ir_submm，
  目标把绝对值收进物理带 46.4–48.4mm。
- 入口：`scripts/dual_ir_color_submm_capture.py --poses 2`。

### P2 · 双 UMI 扩展
- config.py 已支持 right 缺省；等右手硬件到位后跑全量三设备验收
  （`docs/acceptance/d435i-temporary-ego.md` 里有验收口径）。

### P3 · 后端
- COVINS-G 接入（架构文档已就绪）；VINS 漂移排查（先复现静态漂移再查边缘化先验）。

---

## 8. 文档索引（细节都在这些地方）

| 文档 | 内容 |
|---|---|
| `progress.md` | 按日期的完整进展日志（608 行，最权威） |
| `docs/handoff_spatial_alignment_20260905.md` | 空间标定 1328 行完整推导（§49 重采样分析、§62 tilt 隔离） |
| `task_plan.md` | 任务分解与计划（829 行） |
| `findings.md` | 调查结论汇总（667 行） |
| `CODEX_HANDOFF.md` | 与 Codex 阵地的交接（-61 已死、-46.54 候选的出处） |
| `docs/acceptance/d435i-temporary-ego.md` | D435i 单机/barrier/共享 owner HIL 验收口径 + 单机录制命令 |
| `docs/acceptance/opensource-spatial-calibration-route-20260906.md` | 开源空间标定路线 |
| `docs/architecture/ego-umi-covins-integration.md` | COVINS-G 集成架构 |
| `~/桌面/2UQ2_标定交付_20260908/README_交付说明.md` | 2UQ2 标定交付（产品 ego 参数） |

## 9. 关键产物路径速查

```
artifacts/product_sessions/three_device_20260904T114327.548871Z_0868eae8  ← 901/901 索引
artifacts/spatial_bench/ego_color_tag_20260904T232857                     ← 刚性共空间 0.25mm
artifacts/spatial_bench/mount_calibration_umic_consensus_20260905.json    ← mount 共识
artifacts/spatial_bench/mount_z_decision_run1/report.json                 ← 最终 z 决策
artifacts/spatial_bench/dual_ir_submm_20260907T223400/ + submm_final_2pose/ ← 亚mm重复性
~/桌面/2UQ2_标定交付_20260908.tar.xz                                      ← 产品 ego 标定交付
~/桌面/2UQ2_摄像头标定/                                                    ← 2UQ2 复标工具包
```

## 10. 交接检查清单（新人第一周）

1. [ ] 跑通 `pytest tests/` 全绿（确认环境）
2. [ ] 读 `docs/acceptance/d435i-temporary-ego.md` + `progress.md` 前 100 行
3. [ ] 做 §3.2 的 autosuspend 修复（重开机后）
4. [ ] 用 `pair_color_tag_capture.py` 采一小段，跑 `mount_calibration.py` 复现共识值
   （预期 t≈[0.0196, −0.0044, −0.0613]，差 >1cm 说明环境/设备有问题）
5. [ ] 跟用户确认 P0（2UQ2 接入）开工许可
