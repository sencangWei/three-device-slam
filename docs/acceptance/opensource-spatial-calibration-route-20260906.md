# 开源对照后的空间标定路线（研究建议，尚未接入）

后续状态：用户批准后已新增独立采集/求解软件，见
[公共板操作与验收](common-board-calibration-workflow.md)。仅软件测试完成，
尚未进行这条新路径的真实配对采集或参数启用。以下保留研究时点的记录。

日期：2026-09-06。针对Ego(D435i临时代用)+左UMI(D405)，后续扩展双UMI。
本轮仅调研和方案记录；不启用标定、不改出厂K/D、不修改正式coordinator，
不再采集，不改外仓/opt、不提交。已有5–6mm外Tag双链差仍未解决。

## 结论及与已失败试验的区别

建议把**安装外参标定**的主要参考从两个独立散码改成已知几何的整块公共
AprilGrid，固定出厂成像模型，在双方共同观测下联合求相机关系，再求UMI背码
相对UMI光学坐标的刚性安装变换。两个散码保留为独立诊断对象，不把平均两链
或优化强制两链一致当验收。

这不是声称换优化器就能消除系统误差：

- 本仓库`mount_bundle_diagnostic.py`早已联合拟合X/M/外码布局，留出背码仍约5px。
- 固定旧散码布局的试验反而约7.168px，不能重复当新方法。
- 后来的80mm外码-only联合拟合留出背码仍3.65060px，详见handoff§30。
- 新增信息应是**独立已知的密集刚性板几何、共同静态观测和多种板姿态**，
  而不是给同样稀疏数据加更多待估参数。
- 刚采四组单相机图用于成像诊断，不能仅靠同名序号配成双机外参数据：
  摄像头/板分别移动，缺少跨组不变的板位姿或相机刚性关系证据。

## 查到的实际开源代码和限制

### AprilRobotics / apriltag_ros

[AprilTag README](https://github.com/AprilRobotics/apriltag#pose-estimation)
提供检测/单Tag位姿，不是完整三设备世界融合系统；作者明确了tagsize与坐标定义。
本项目已经做过官方库对照且仍有偏差（handoff§30），不能再宣称只换官方库即可解决。

[apriltag_ros common_functions.cpp](https://github.com/AprilRobotics/apriltag_ros/blob/master/apriltag_ros/src/common_functions.cpp)
`detectTags()`先把bundle内各Tag角点按固定成员变换放到bundle坐标系，
收集全部2D/3D对应点，再一次调用`getRelativeTransform()`求bundle姿态；
不是先各算一个相机姿态再平均。其`getRelativeTransform()`调用solvePnP，
输入按rectified图像解释、畸变置零；不可把我们原始inverseBrown图像原样接入。

### multical：主要参考的离线外参求解

[README](https://github.com/oliver-batchelor/multical#can-multical-calibrate-intrinsic-parameters-separately)
明确支持先给已有内参，再用`--fix_intrinsic`仅标外参；也说明通过
`--fix_intrinsic --fix_camera_poses`在另一批图像验证固定参数。
这是上游用法，不是当前仓库可直接执行的已安装命令。

[参数定义](https://github.com/oliver-batchelor/multical/blob/master/multical/config/arguments.py)
`OptimizerOpts`中存在`fix_intrinsic/fix_camera_poses/fix_board_poses/fix_motion`，
`adjust_board`默认false；损失支持linear/soft_l1/huber/arctan。
本方案不放开板几何去吸收偏差。

[Calibration.bundle_adjust](https://github.com/oliver-batchelor/multical/blob/master/multical/optimization/calibration.py)
优化观测与投影点之差，参数组可独立启停；有离群处理，不能直接拿删后残差
代替我们的全部样本/拒绝率/独立留出验收。

[输入匹配代码](https://github.com/oliver-batchelor/multical/blob/master/multical/config/runtime.py)
按相机文件夹对应图像构造数据。文件名匹配本身不证明同时观测同一个物理板姿态。

**适配约束：**上游相机模型选项与本机SDK inverse_brown_conrady并不直接等价。
实现应保留当前SDK几何投影/反投影，或者先生成数学上等价的固定虚拟无畸变
观测并验证往返；禁止直接把五个inverseBrown系数改名为OpenCV radtan。

### TagSLAM：后续动态Ego＋双UMI的状态模型参考

[Concepts](https://berndpfrommer.github.io/tagslam_web/concepts/)
将相机/Tag附着于body，body可以动态，body内安装变换保持固定。
我们的Ego、左UMI、右UMI应是三个独立运动body；**不能建成一个永远固定的三相机rig**。

[Input files](https://berndpfrommer.github.io/tagslam_web/input_files/)
支持board几何、body关联、像素因子与里程计；模型主要是pinhole+radtan/equidistant。
[Caveats](https://berndpfrommer.github.io/tagslam_web/caveats/)
要求唯一Tag身份；不同border-bit混用受检测器限制，近似同步会损失高频更新。
我们当前外id1与背id1冲突、板2-bit与背码1-bit混用，均不能不经适配直接导入。
安装标定需分别检测、按明确角色/边框验证，若接TagSLAM则重新映射到唯一逻辑身份，
并拒绝不确定关联；不偷偷当成同一个tag。

[Better localization](https://berndpfrommer.github.io/tagslam_web/better_localization/)
强调多个可见Tag、充分分散而非线性布局；并重视快门和同步。
这不是承诺本系统达到任何毫米级指标。

### 平面姿态歧义及VR项目的适用边界

[IPPE作者说明](https://github.com/tobycollins/IPPE#resolving-the-flip-ambiguity)
明确近仿射投影时仅靠单Tag重投影误差无法可靠择支；解决方向包括非共面标记，
或覆盖足够大范围的已知共面多标记统一PnP。我们已做过原数据IPPE模式检查，
因此不能把此一般性问题直接宣布为当前5.63mm根因。

[April-Tag-VR-FullBody-Tracker](https://github.com/ju1ce/April-Tag-VR-FullBody-Tracker)
是VR人体追踪应用。应参考其刚性tracker设计，而不能把能在SteamVR显示的demo
当成三设备标定精度证据；不建议用它替换UMI/Ego的VIO系统。

已进一步核对[其多相机教程](https://github.com/ju1ce/April-Tag-VR-FullBody-Tracker/wiki/Using-multiple-cameras)：
分别运行多个ATT实例并对齐SteamVR playspace，说明仍可能需手动调C值，并用
depth smoothing减轻深度抖动。[空间标定教程](https://github.com/ju1ce/April-Tag-VR-FullBody-Tracker/wiki/Calibrating-playspace)
使用控制器/手动输入进行位置旋转调整。这不是可照搬的自动毫米级双相机安装外参标定。

## 建议实施的数据与数学模型

沿用本仓库约定：`T_A_B`将B坐标变到A。
`E/U`均为实际color光学帧，`B`为公共AprilGrid，`M`为UMI背码。

### 阶段A：两相机临时固定，只移动公共板

以同一个Ego/UMI相对摆位为一组，在两相机都看见板的条件下，缓慢改变板位置/倾角，
每个位置停稳再取静态窗口。软件必须给双画面、板角点/可见性提示、当前动作和计数。
保存原图、SDK时间/主机时间、配对差、板身份/布局、出厂快照及hash。
冻结训练与完整姿态窗口留出，不随机拆相邻帧冒充独立验证。

变量：组内唯一`X=T_E_U`，每个板姿态`B_j=T_U_Bj`；K/D、板角点坐标均固定。
联合最小化：

```
u_Ujk ≈ π_U(B_j p_Bk)
u_Ejk ≈ π_E(X B_j p_Bk)
```

两机和板在采样窗口内都静止可降低时间差影响，但不是已标定时间偏移。
动态运动窗口另行同步，不能把近邻时间配对当硬同步。

### 阶段B：求真正需要长期保存的背码安装外参

同一组内相机相对位置不变，Ego观测背码，则：

```
M = T_U_mount = inverse(X) T_E_mount
```

背码不参加X的标定，避免它参与拟合又验证自己。下一独立相机摆位重求X，
**固定训练M**预测背码角点，检验换视角一致性，而不是再拟合一个M来缩小误差。
最后通过已审计工厂color→IR/body变换对接D405SLAM，不能重复转换。

### 阶段C：回到Ego世界的动态链路

```
T_W_U(t) = T_W_E(t) T_E_mount(t) inverse(T_U_mount)
```

`W`始终是EgoSLAM世界，不是板世界。板只用于安装标定，后续可移走；
临时标出的X不是运动中恒定的Ego↔UMI外参。左右UMI分别有自己的M和VIO世界对齐。
若SLAM输出body/IMU位姿，先通过已有camera-body外参明确转到上述光学帧。

## 验收/失败分流（建议门槛，采集前需冻结）

- 必須保留全量样本及每项拒绝原因；反投影模型/尺寸/身份错立即阻止。
- 固定X在独立板姿态上做跨相机留出角点预测，不能两机各自由拟合板位姿当验证。
- 再固定M在不同Ego摆位做背码预测，按姿态组报告RMS/p95，不只报总体平均。
- 建议初始像素门槛为独立留出p95<=1px；安装重复性沿用完整3D<=5mm/旋转<=3deg，
  不再只看z。像素门槛是本项目提议，非开源项目保证；重复性仍不是绝对计量精度。
- 若板预测通过而散码/背码仍异常，可在固定X下隔离该目标的观测误差，而不用靠
  两条自由单Tag链自洽来证明M。若板跨相机预测也失败，则先检查对应/共同静态条件/
  成像模型/板几何，禁止放松K/D和板尺寸来强制通过。
- 单平面背码若仍有视角相关误差，可另行评估刚性非共面多Tag小支架，但要独立标定
  支架几何并验证；不是简单再打印一张就自动解决。

本轮没有实施上述新采集/求解路径，也没有最终标定通过结果。
下一步先完成共视采集、固定模型求解、留出验收的软件测试，再安排一次有明确
停止条件的操作；不让用户继续盲目挪动原两个箱子。

## 调研记录

- 公开官方网页/raw源文件成功读取；GitHub匿名tree API限流403，未继续重试。
- AnySearch本地CLI与声明v2.1.0原文件hash不同，依skill要求未执行，未读取/写入密钥。
- 两个猜测的VR wiki路径无内容，已改查其主页实际链接/对应raw wiki；不引用空页面。
- 以上链接为查询当时master/公开文档；若采用第三方代码，实施前须锁定commit及许可，
  本轮未安装依赖、未复制第三方实现到运行环境。
