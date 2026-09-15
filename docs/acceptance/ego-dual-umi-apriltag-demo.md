# Ego 世界系 + 双 UMI AprilTag DEMO（仿真）

日期：2026-08-28

## 结论

- `PASS/simulation`：左右 UMI 的临时 AprilTag 观测、安装外参、独立本地里程计原点、窗口式稳健锚定、遮挡桥接和30 Hz Ego世界轨迹已在确定性仿真中闭环。
- `NOT_RUN/HIL`：本报告没有使用真实相机、IMU、VIO、SLAM或真实安装外参，不能作为空间精度或三设备实机通过证据。
- Ego SLAM 世界 `W` 是唯一全局世界；Tag只为左右 UMI 提供 `T_W_O` 约束，不定义或重置 `W`。

## 坐标与时间契约

所有变换采用 `T_A_B`：把 B 中的坐标转换到 A。

```text
T_W_G = T_W_CE @ T_CE_A @ T_A_G
T_W_O = T_W_G  @ inverse(T_O_G)
T_W_G(t) = T_W_O @ T_O_G(t)
```

- `C_E`：Ego用于检测Tag的相机光学坐标系。
- `A`：左或右Tag坐标系。
- `G`：夹爪目标坐标系。
- `O`：对应UMI独立的VIO/里程计坐标系。
- `T_A_G`是Tag安装外参，必须实测或标定；Tag ID不能代替安装外参。
- 检测完成时间只记录处理延迟。几何计算使用图像采集时间，并先应用有符号的设备时间偏移，再插值Ego与UMI轨迹。

## 当前临时Tag契约

当前已经生成并打印验证的资产保持原契约：

| UMI | family | ID | 有效检测边长 |
|---|---|---:|---:|
| left | `tag36h11` | 0 | 40 mm |
| right | `tag36h11` | 1 | 40 mm |

有效边长是黑色Tag方块/检测角点之间的距离，不包含外侧白色静区。官方AprilTag 3建议新设计优先评估 `tagStandard41h12`，但不能在未重新打印、重新配置和重新验收前静默改变现有ID/family。

## 质量门

仿真接口已经执行：

- family和ID必须与对应左右安装标定一致；
- 默认hamming必须为0；
- 重投影误差不得超过1.5 px；
- Tag深度必须为正且位于配置范围；
- 至少3个同步候选形成SE(3)共识；
- 3 cm / 3 deg之外的锚点候选作为共识离群点拒绝；
- 重新出现后的锚点突跳不得覆盖遮挡前的有效锚点；
- 左右锚点、拒绝计数、安装标定和遮挡状态完全独立。

这些是第一版软件门限；真实毫米/角度精度门必须根据安装标定和外部真值测量冻结。

## 30秒样例

生成命令：

```bash
cd /home/robot/three-device-slam
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. \
python3 -m three_device_slam.spatial.apriltag_demo \
  --output artifacts/spatial_demo/ego_dual_umi_apriltag_20260828_v2 \
  --duration 30 \
  --fps 30
```

输出：

```text
manifest.json
calibration/tag_mounts.json
derived/tag_observations.jsonl
derived/trajectories_ego_world.jsonl
quality/alignment_report.json
exports/demo_overlay.mp4
```

实测仿真证据：

| 项目 | left | right |
|---|---:|---:|
| Tag ID | 0 | 1 |
| 初始候选/内点 | 5 / 4 | 5 / 4 |
| 注入并拒绝的PnP翻转候选 | 1 | 1 |
| 重识别候选/内点 | 5 / 4 | 5 / 4 |
| 重识别时注入并拒绝的PnP翻转候选 | 1 | 1 |
| 锚点平移误差 | `4.97e-16 m` | `6.28e-16 m` |
| 锚点旋转误差 | `3.18e-15 deg` | `6.41e-15 deg` |
| 遮挡期间VIO桥接帧 | 60 | 90 |
| 遮挡后重新看到Tag | 是 | 是 |

修正后的 `v2` 保存了4条真实错误PnP矩阵（左右初始/重识别各1条），而不是只给正常位姿贴拒绝标签；每条都能从同时间的Ego/夹爪真值重算并证明平移误差大于0.1 m、旋转误差大于90 deg。遮挡后实际重新计算5候选/4内点锚点，并经过连续性仲裁后接受。旧 `v1` 保留为复审发现内部证据不一致的历史产物，不得用于验收。

视频使用OpenCV重新打开并逐帧解码：900帧、30 Hz、1280×720。主机没有安装 `ffprobe/ffmpeg`，所以没有声称通过该工具检查容器；逐帧OpenCV解码和文件SHA-256均通过。自动化测试另在隔离子进程中生成并完整解码6帧短视频，避免把 `cv2` 泄漏进采集worker导入隔离测试。

## 图像级检测/PnP回放

第二阶段已经把“直接注入 `T_camera_tag`”替换成真实像素处理链：官方固定版本的Tag位图经过已知相机模型投影到1280×720灰度图，OpenCV参考检测器从图像解码ID和四角，`SOLVEPNP_IPPE_SQUARE`再根据40 mm黑色方形求出米制 `T_camera_tag`。该结果经过正深度、重投影误差和平面双解门，才进入上面的左右独立锚定逻辑。

```bash
cd /home/robot/three-device-slam
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. \
python3 -m three_device_slam.spatial.apriltag_image_demo \
  --output artifacts/spatial_demo/ego_dual_umi_apriltag_image_20260828_v3 \
  --duration 6 \
  --fps 10
```

本次确定性图像回放证据为 `PASS/simulation_image_replay`、`NOT_RUN/HIL`：

| 项目 | left | right |
|---|---:|---:|
| Tag ID | 0 | 1 |
| 初始IPPE候选/SE(3)内点 | 10 / 10 | 10 / 10 |
| 重识别候选/内点 | 10 / 9 | 10 / 8 |
| 图像遮挡帧 | 9 | 9 |
| 初始锚点平移误差 | 27.48 mm | 19.68 mm |
| 初始锚点旋转误差 | 0.737 deg | 0.735 deg |

共处理60帧并输出120条左右预期Tag审计行，未出现错误ID；视频完整解码为60帧、10 Hz、1280×720。6个清单文件的大小和SHA-256均复核通过。像素量化造成的厘米级锚点误差被如实保留，没有用真值替换检测结果。

`v3`在重识别连续性门接受后，按各自接受时间切换到新锚点（left 3.9 s、right 4.8 s）；轨迹同时保存 `T_odom_device`，自动化测试逐帧验证切换前后分别由初始/重识别 `T_world_odom` 传播。检测审计行还保存全部IPPE候选、正深度标志和 `selected_index`，公开模型强制观测位姿/误差与选中候选完全一致。

旧 `v1` 在报告中记录了重识别通过，但导出轨迹一直沿用初始锚点，因此不得用于验收。`v2`修正了轨迹切换，但生成器尚未强制绑定选中候选与观测，也未在证据行保留完整IPPE决策，已由 `v3`取代。

当前后端名称固定为 `opencv_aruco_reference`，只用于确定性软件回放。主机尚未安装官方 `AprilRobotics/apriltag` C后端，所以正式运行后端仍是 `NOT_RUN/AprilRobotics_backend_not_installed`；不能把这次OpenCV回放写成官方检测器或实机精度通过。

## 进入真实回放/HIL前仍缺

1. Ego相机内参、畸变、相机光学帧到Ego SLAM轨迹帧的外参。
2. 左右真实 `T_A_G` 安装外参及其标定ID、输入证据和哈希。
3. Ego与左右UMI采集时间偏移、漂移和残差。
4. Ego SLAM `T_W_CE(t)` 和两路UMI `T_O_G(t)` 的真实可回放轨迹。
5. 官方AprilTag 3生产检测后端，以及实测40 mm Tag像素尺寸、倾角和遮挡范围；当前OpenCV参考回放只覆盖软件接口和数学方向。
6. 外部真值或已知刚体，用于冻结真实位置、角度、重定位和闭环阈值。

在这些条件不完整时，真实空间层必须保持 `BLOCKED`，仿真轨迹不得进入训练数据。

## 软件验证

- 空间专项：`35 passed in 3.24s`。
- 全仓最终回归：`763 passed in 353.58s`。
- `py_compile`、`git diff --check`：通过。
- 独立代码复审：Critical 0、Important 0；图像候选/观测强绑定与重识别轨迹切换两项Important已补测并关闭。
