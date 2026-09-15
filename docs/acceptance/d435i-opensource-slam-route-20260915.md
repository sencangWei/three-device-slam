# D435i Ego 开源实时与后处理 SLAM 路线（2026-09-15）

## 纠正当前实现的身份

刚才运行的 Ego 实时轨迹不是已经验收过的 Ego SLAM。它来自本机 9 月 6 日的
D435i 内置 IMU 回放探针：采集 D435i 双红外和内置 IMU，再送入 UMI 所在工程使用
的 VINS-Fusion ROS 2 fork。虽然输入配置换成了 D435i 工厂几何，代码也标注为
`PROVISIONAL`，但时间偏移、IMU 噪声和静态姿态重复性没有完成正式验收。

因此保留这部分代码只作为数据接口和诊断证据，不把它定义为 Ego 正式程序。

## 官方项目核查

| 项目 | D435i 实时输入 | ROS 2 | 闭环/地图 | 离线后处理 | 本项目判断 |
|---|---|---|---|---|---|
| ORB-SLAM3 | 官方双目惯性 D435i 示例 | 上游示例直接用 librealsense，不是 ROS 2 节点 | 有闭环、Atlas 保存/加载 | 可用同一引擎回放并保存轨迹/Atlas | 选为 Ego 主引擎 |
| VINS-Fusion | 官方有 D435i 双目 IMU 配置 | 上游为 ROS Kinetic/Melodic | 有 loop_fusion 和 pose graph | 可回放 | 本机当前临时路线已接近它，不能解决“未经标定/未验收”的根问题 |
| OpenVINS | 上游有 ROS 2 双目订阅和 D455 示例 | 有 | 主体是 VIO；图优化闭环仍在路线图 | 可输出状态，非完整 Atlas 后端 | 可作对照，不作为主方案 |
| RTAB-Map | 可接 stereo、IMU 或外部 odom | 有 | 有数据库、图优化、重处理 | 强 | 可作独立地图后端，但当前会增加第二套估计器和坐标管理 |

官方来源：

- ORB-SLAM3 D435i 示例：<https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc>
- ORB-SLAM3 示例配置：<https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Examples/Stereo-Inertial/RealSense_D435i.yaml>
- VINS-Fusion D435i 配置：<https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/config/realsense_d435i/realsense_stereo_imu_config.yaml>
- OpenVINS：<https://github.com/rpng/open_vins>
- RTAB-Map ROS 2：<https://github.com/introlab/rtabmap_ros/tree/ros2>
- UMI 建图与批量定位：<https://github.com/real-stanford/universal_manipulation_interface/tree/main/scripts_slam_pipeline>

## 哪些可以直接复用

可以复用 ORB-SLAM3 的双目惯性估计器、闭环、Atlas 保存/加载、轨迹导出，以及 UMI
“先建 Atlas、再加载 Atlas 批量定位”的后处理结构。

不能原样运行官方 `stereo_inertial_realsense_D435i`：

1. 它选取 `devices[0]`，没有绑定本机 D435i 序列号。
2. 它自己创建 librealsense pipeline；本项目已经证明 D435i 与 D405 分进程拥有
   RSUSB 生命周期会不稳定。
3. 它固定 640x480@30，而当前采集契约是 1280x720@30。
4. 配套 YAML 是上游作者那台相机的示例值，不能当作序列号
   `327122078613` 的标定结果。

## 采用的实现边界

1. 保持现有单 owner 采集 D435i 和 D405。
2. 从该 owner 输出 D435i 的成对整流 IR 图像和内置 IMU，送入固定 commit 的
   ORB-SLAM3 `TrackStereo` 接口。
3. 实时路径输出 Ego 本地连续位姿；离线路径读取同一数据契约，做完整闭环和
   Atlas/轨迹落盘。
4. 开始时让 D435i 看到一次固定 AprilGrid，求出并冻结 `T_world_orb`；之后靠
   D435i 双目+IMU 连续运动，AprilGrid 不需要持续可见。
5. 先以已录制 D435i 数据验证确定性和轨迹，再做 D435i 单机实时 HIL，最后才与
   device3 同时运行并检查频率、延迟、丢帧和世界坐标一致性。

## 本机落地验证

已在本机 Ubuntu 22.04、OpenCV 4.5.4 上从官方 commit
`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` 原样编译通过 ORB-SLAM3 核心库和
`stereo_inertial_euroc` 离线双目惯性程序，动态库解析正常。证明官方引擎和离线
后处理可以在本机落地。

官方直连 RealSense 示例当前没有生成，因为主机没有安装可被 CMake 发现的
`realsense2` 开发包；这不是所选路线的阻塞项，因为正式实时适配器本来就不能再
开一条相机 pipeline。构建证据见
`artifacts/spatial_bench/d435i_opensource_route_probe_20260915/report.json`。

## 已完成的项目适配与当前验收边界

已增加 D435i 原始采集到 ORB-SLAM3 EuRoC 双目惯性输入的确定性导出器，并用现存
30 秒静态台架数据完成真实回放。输入为 900 对 1280×720 双红外、6013 条按陀螺仪
时刻插值的 IMU；相机与 IMU 时间严格递增，IMU 覆盖全部相机帧。

官方当前 commit 在 `Rectified` 双目设置日志中存在空指针：内部只保存共享的左相机
模型，却打印未创建的第二模型。本项目补丁仅让日志读取共享模型，不改特征、跟踪、
闭环或优化。修补后的二进制、依赖、词典及逐文件哈希已固定在
`artifacts/runtime/orbslam3-4452a3c4`。

静态回放完整摄取全部数据并到达 `Shutdown`，但因设备没有运动，898 帧报告惯性
激励不足，Atlas 为空；因此该结果只标为
`DIAGNOSTIC_STATIC_INGEST_COMPLETE_NO_MAP`。它证明接口接通，不能作为轨迹通过。
报告位于
`artifacts/spatial_bench/d435i_orbslam3_static_replay_20260915_run2/replay_report.json`。

已经重新保留一次 105 秒动态原始数据。设备3、跨相机时间和运动信号通过；Ego 在
约 87.03 秒发生记录器队列溢出，丢失 114 条记录。四路序列缺口的合计也正好是
114，因此这是写盘反压证据，不能表述为已证实的 USB 掉线。Ego 写队列已从 128
增至 512 条。

中断前连续 85 秒的官方双目惯性回放产生 20 次 bad-IMU 地图重置，最后轨迹只剩
38 帧、1.23 秒，已严格判为 `REPLAY_FAILED_INERTIAL_INITIALIZATION`。相同 2550
对图像的纯双目对照则单地图连续覆盖 84.97 秒，说明图像、双目几何和离线读取链路
可以工作。此次整体平移多数只有 0.02--0.06 m/s，无法稳定满足官方初始化阶段最近
两个关键帧合计至少 0.02 m 的平移保护。中文引导已改为起始阶段以
0.15--0.30 m/s 连续整体平移。

详细诊断分别见
`artifacts/spatial_bench/d435i_orbslam3_dynamic_prefix85_replay_20260915_run1/diagnosis_report.json`
和
`artifacts/spatial_bench/d435i_orbslam3_dynamic_prefix85_stereo_diagnostic_20260915_run2/diagnostic_report.json`。
当时为下一次正式 HIL 设定了两项复检目标：Ego 写队列零丢失，以及双目惯性轨迹
在扣除初始化前缀后连续覆盖到输入末尾。后续正式门槛落为整体帧数和时长至少 85%，
以容纳有意保留的起始静止段和 ORB 惯性初始化时间。

## 105 秒正式复检结果

正式复检会话的 Ego、设备3、跨相机时间和运动信号均为 PASS。Ego 的双红外各
3150 帧、陀螺仪 21035 条、加速度计 21197 条，所有流均零序列缺口，写队列零丢失。

上游初始化保护原值要求最近两个关键帧平移超过 50 mm 才累计初始化时间，却会在
低于 20 mm 时立即重置。本次实测范围为 10.9--49.2 mm，因此原值无法推进。补丁
`patches/orbslam3-d435i-configurable-init.patch` 将两个值改为配置项并保留上游默认
值；D435i 配置采用累计 20 mm、重置 5 mm。

哈希固定的新运行时
`artifacts/runtime/orbslam3-4452a3c4-d435i-init` 在完整数据上只建立一张地图，
完成 VIBA1/VIBA2，零 bad-IMU 重置。初始化耗时 11.645 秒，随后 2801 个位姿连续
覆盖 93.336 秒直到最后一帧；回到起点平移误差 2.799 mm、姿态误差 1.536 度，
最后静止段最大位置漂移 0.829 mm。独立验收的 15 项检查全部通过，状态为
`PROVISIONAL_LOCAL_SLAM_PASS`。它证明本地 D435i 双目惯性路线可用；世界变换、
D435i 专属 IMU 噪声/时间参数和实时共享 owner 接口仍需分别验收。

## 实时 TrackStereo 适配器

新增的原生适配器不打开 RealSense。现有单 owner 进程将严格配对的 D435i 双红外和
按陀螺仪时刻插值的 IMU 通过有界二进制通道送入同一固定版本 ORB-SLAM3
`TrackStereo`，再从独立返回通道取得 `T_orb_camera`。运行时及源文件均有哈希清单。

使用正式 105 秒数据按真实 1× 时间验证时，3150 帧全部送入并全部返回，写队列最大
为 1，零 bad-IMU 重置并完成 VIBA1/VIBA2。处理耗时 P95 为 25.824 ms，端到端 P95
为 27.821 ms。得到的 2801 个有效位姿与正式离线程序具有相同的 11.645 秒初始化
边界和 93.336 秒覆盖，回点与静止指标继续通过。该证据完成实时数据接口的软件
等价验证；相机在线 HIL 与世界坐标锚定仍是后续独立门槛。

## 在线 HIL 与世界坐标接入

放大界面的 60 秒 D435i 在线运行完整接收 1800 对双红外、12020 条陀螺仪和
12115 条加速度计数据，所有相机序列零缺口。跳过前 10 秒静止调整后，1500 帧全部
送入 ORB 并全部返回，1366 帧跟踪正常，VIBA1/VIBA2 完成且没有 bad-IMU 重置；
处理耗时 P95 24.087 ms，端到端 P95 26.672 ms。采集后的上游线程退出没有正常
返回，因此该次报告保留为 FAIL，不能掩盖。

无需重采数据。相同 v3 运行时随后以真实 1× 速度完整回放现有 105 秒数据，
3150/3150 帧返回、2801 帧跟踪正常、VIBA1/VIBA2 完成，进程返回码 0，证明数据与
安全退出补丁均可复用。两设备世界程序现已删除临时 Ego ROS VINS 依赖：单一
RealSense owner 把 D435i 双目/内置 IMU 送入 ORB，把 D405/STM32 送入设备3签发
VINS。一次 AprilGrid 观测分别冻结 `T_world_orb` 与 `T_world_right_odom`，随后
两台设备可移动且不要求持续看到板。世界坐标实机运行是剩余门槛，不再重复本地
105 秒采集。
