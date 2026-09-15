# 公共板外参＋背码安装标定：操作与验收

2026-09-06。新独立诊断路径；不修改正式 coordinator，不启用任何标定。
出厂 SDK K/D 固定。D435i 临时 Ego + D405 左 UMI；不是原 2UQ2。
代码基于本仓库既有 SDK 几何工具自行实现，未安装/复制第三方求解器。
开源依据与差异见 [调研路线](opensource-spatial-calibration-route-20260906.md)。

## 本次软件能做什么

仅分析当前静态各Tag时，不要求完成16步，也不需要再次按R：有已保存同框
静态窗口即可运行下面离线命令。它验证原图和静态门槛，输出逐码、整板及分组
坐标链诊断，**不输出标定通过，不激活参数**。新输出目录不得已存在。

```bash
cd /home/robot/three-device-slam
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. /opt/three-device-slam/venv/bin/python \
  scripts/common_board_static_analysis.py \
  --session artifacts/spatial_bench/common_board_joint_20260906_run1 \
  --output artifacts/spatial_bench/common_board_joint_20260906_run1/static_analysis_repeat
```

- 同时双画面预览：Ego 蓝框、UMI 绿框；终端每 5 秒有 LIVE 提示。
- 每一步只在按 `R` 后取一个约 1.2 秒的静态窗口。合格才进入下一步；
  不合格保留原图，输出原因，停在当前步骤等待再次按 R。
- v2预览先显示 WAIT（观察时间不足）、HOLD（未稳/覆盖不足）或 READY R，
  并显示首帧共同跟踪码数量、最大漂移。终端有中文原因和本步动作。
  预检未通过仍可按R保留诊断原图，但不会因此放宽窗口验收。
- `Q`、ESC、关窗口或中断都会停止；未完成数据不会进入标定验收。
- 原生彩色流 1280×720@30，保存用于标定的灰度 PNG；不是 30Hz SLAM 原始录像。
- 独立设备时钟原样保存，不拿两个 hardware_clock 数值当共同时间。
  主机到达差只用于静态窗口配对筛查，**没有完成动态同步、延迟或 td 标定**。

## 摆放与开始

### 当前推荐：公共板和背码同框

`--simultaneous --board-detector umi-aprilgrid`：Ego同时看公共板和UMI背码，
UMI看同一公共板。无需移走公共板，也无需UMI看自己的背码。首次R前可以调
两机摆位；首次R后两机固定，仅移动公共板且不遮挡背码。

```bash
cd /home/robot/three-device-slam
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. \
/opt/three-device-slam/venv/bin/python -u scripts/common_board_capture.py \
  --output artifacts/spatial_bench/common_board_joint_20260906_run1 \
  --preview-hz 8 --board-detector umi-aprilgrid --simultaneous
```

此路径保留原16步训练/留出划分；第13–15步仍需板和背码同时可见，
第16步为最后同框复核，没有移板阶段。蓝色Ego板框、绿色UMI板框、紫色背码框。
公共板ID1固定在两机的求解输入中排除，灰色仅显示；其余35个板码照常使用，
不是按误差大小删码。保存完整原图，旧录制/旧验收不改判。

目标身份规则`board35_plus_separate_mount1_v1`随setup冻结：
使用ID1之外至少12码/3行3列的全部角点（无RANSAC）拟合仅用于身份的单应矩阵；
p95≤5px、max≤12px。预测整板黑码范围并向外扩大一个间隔10.56mm，背码四边形
必须位于此范围之外、无交叠且角点距离>3px。**在Ego图里把背码与板并排放，
不要把背码叠到板面上。** 唯一的一位边框ID1才可作为背码，边长≥60px；
二位检测器的外部ID1必须与该背码是同一图像四边形，否则拒绝身份不明。
仍需操作者确认那个独立ID1确实贴在UMI上；视觉不能辨认两个完全相同的印刷码的物理归属。

每个保留窗口同时验证两机板的静态覆盖，以及背码连续可见/漂移≤0.75px，
不拿预览上一帧的READY接受当前失效图。身份用单应矩阵的门槛不是标定精度；
原训练/留出≤1px等门槛不变。求解另输出每个同帧板关系
`X_j=T_E_Bj inverse(T_U_Bj)`；固定三次背码窗口求得的M（或独立参考M），
以`X_j M`预测当帧背码，每窗p95≤1px，不用该窗背码重新拟合M来通过。
逐窗IPPE安装候选仅诊断，所有分支列出，不按卡尺距离择优。任一检查失败为REVIEW。
板身份/观测模式必须在续采和离线重放一致，禁止将旧分时会话换旗标升级为同帧数据。

### 旧模式：分时摆位预检（仅保留兼容）

新增检测/摆位修复（2026-09-06）：`--board-detector umi-aprilgrid` 使用UMI既有
aprilgrid0.5.0检测路径，当前1280×720保持原图分辨率，保留同ID多处冲突；
不缓存上一帧码来掩盖漏检。预览区分visible/valid/dup，黄色仅标冲突。
旧会话缺省仍按原OpenCV重放；续采不能换检测器，禁止改旧setup来重判。

先做三步摆位检查（输出目录必须新建）：

```bash
cd /home/robot/three-device-slam
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. \
/opt/three-device-slam/venv/bin/python -u scripts/common_board_capture.py \
  --output artifacts/spatial_bench/common_board_placement_umi_detector_20260906_run2 \
  --preview-hz 8 --board-detector umi-aprilgrid --placement-check-only
```

1. 公共板先移走，调整摆位让Ego完整看到背码；停稳按R，从此两相机不动。
2. 只放入公共板让两机共视；可遮住背码以免与板ID1冲突。停稳按R。
3. 相机仍不动，移走板露出背码；再次按R检查背码像素闭环≤0.75px。

若必须转动Ego才能换目标看，当前摆位不可用，Q退出重新摆。三步有300秒
上限，只有可见性/静态代理检查，不是标定；求解器拒收此数据。不会自动开启16步。
几何上可以分时看板/背码，但两相机的相对关系必须保持；像素闭环不能独立证明
两机完全静止。正式采集仍用下方16步，另加`--board-detector umi-aprilgrid`，
使用全新目录，最后公共板复核不能省略。

先关闭已有预览/采集，不让两程序争用相机。
Ego 和 UMI 放稳，一组内保持相对位置不变；UMI 背码也不能重新贴动。
两机共同看见已有平整刚性 6×6 板（黑码 35.2mm、间隔 10.56mm，旧式两位边框/黑接点）。
不换用外部 80mm 散码作为板。背码为一位边框 ID1、黑边 40mm。

```bash
cd /home/robot/three-device-slam
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. \
/opt/three-device-slam/venv/bin/python -u scripts/common_board_capture.py \
  --output artifacts/spatial_bench/common_board_train_20260906_run2 \
  --preview-hz 8
```

输出目录必须不存在；run1三次失败原图已保留，新版从run2开始，禁止覆盖旧数据。
启动后只预览。先看清两个画面，再按 R。

## 16 步时间轴

1–12 步：两机不动，只移动公共板，覆盖中/左/右/上/下、不同倾角和远近。
每次把板停稳再按 R。完整窗口 3/6/9/12 预先留出，不能混回训练。
13–15 步：两机仍不动，移走公共板和旧散码，Ego 只看 UMI 背码，分别按 R。
16 步：把公共板放回双画面再按 R，作为背码阶段后的相机关系复核，不用于训练。

每个窗口至少 3 对、跨度至少 1 秒且不超过 2 秒，主机到达差 ≤75ms，
v2不再要求每帧ID集合完全一致：每帧与窗口首帧均须至少12个共同码、
跨至少3行3列。对所有重复出现的码，检查其相对自身第一次出现的角点漂移
≤0.75px，包含首帧没检测到、后来才出现的码；不根据漂移大小删点。
用于求解的中间原始图对，仅使用两机共见且各自至少出现两次的ID，仍需12码/3行3列。
因此偶发漏码不会被当成移动，但缺乏可比较角点或真实漂移超限仍拒绝。
背码阶段仍须ID1连续可见，不适用多码容错。
板须至少 12 个共同 Tag、跨至少 3 行和 3 列、边长至少 15px。
背码须单独可见、边长至少 60px；离线另验倾角 ≤45°、IPPE 歧义及重投影。
像素 px 指原生 1280×720 图像像素，不是缩小后预览上的像素。

程序末尾打印带实际路径的求解命令。采完不等于参数合格：

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. \
/opt/three-device-slam/venv/bin/python -u scripts/common_board_calibration.py \
  --session artifacts/spatial_bench/common_board_train_20260906_run2 \
  --output artifacts/spatial_bench/common_board_train_20260906_run2/solve
```

## 求解与失败分流

`T_A_B` 把 B 坐标变为 A。固定板几何和出厂投影模型，只拟合组内相机关系
`X=T_Ecolor_Ucolor` 和 8 个训练板姿态 `B_j=T_Ucolor_Bj`。
使用 SDK 反投影的 float64 归一化射线、按 fx/fy 缩放残差；不会对 SDK
float32 投影做不可靠的极小步长数值微分。最终像素评估仍用原 SDK 模型。
不删离群角点、不估计板形状/尺寸、不优化 K/D。

每个留出窗口：只用 UMI 角点求板姿态，再用固定 X 预测 Ego 角点；
另做反向预测。不是两机分别自由拟合板姿态再把拟合误差当跨机验证。
每方向、每窗口（含最后复核）p95 必须 ≤1px，训练各窗口也须 ≤1px。
训练板法向跨度 ≥15°、板原点位置跨度 ≥80mm；单窗口独立估计的相机关系
相对联合 X 必须完整 3D ≤5mm、旋转 ≤3°，否则 REVIEW。
四个留出板姿态相对每个训练姿态及其他留出姿态，还须平移至少 20mm 或法向变化至少 7°。
这个差异检查只依据 UMI 单相机板姿态，不能用 Ego 留出误差挑图；
最后的相机关系复核窗口豁免姿态差异要求，但仍须通过跨机预测。

背码不参与上述 X 求解。用 `M=inverse(X) T_Ecolor_mount` 得候选。
同一摆位仅能输出 `CANDIDATE_NEEDS_INDEPENDENT_SETUP`，不叫最终 PASS。
平面位姿候选差异大而重投影近似时拒绝，不靠任取一个支路通过。

第一组候选通过后：只改变 Ego 摆位（相对变化至少 50mm 或 10°），UMI 与背码不变，
重新固定两机，按同样 16 步采集新组。求解第二组时加：

```text
--reference artifacts/spatial_bench/common_board_train_20260906_run2/solve/report.json
```

第二组 X 用第二组板训练求解，**固定第一组 M** 预测第二组背码。
每个背码窗口 p95 ≤1px，M 重复性完整 3D ≤5mm / 旋转 ≤3°；
同时两组板验收都必须通过，才是 `PASS_INDEPENDENT_MOUNT`。
第二组另算 M 只用于报告重复性，不替换固定参考 M 来降低验证误差。
以上是项目诊断门槛，不是开源工具的精度保证，也不是绝对计量验收。

板跨机预测失败：先查共同静态条件、对应/成像、板几何；不要先改出厂内参。
板通过而固定 M 的背码预测失败：集中查背码观测、平面姿态/安装稳定性，
不再通过自由调整两个外部散码布局把问题掩盖。

## 数据契约与后续边界

`setup.json`：设备序列号、出厂完整畸变模型、目标、冻结步骤/门槛、代码 hash。
当前schema为`ego.common_board_static_capture.v3`。v2按原检测规则只读重放，
不把旧拒绝改判通过；v1录制不被静默改判或拿来完成新组。
`attempts.jsonl`：所有尝试，slot/split、每对 SDK 时间域/时间戳/帧号/主机到达时间，
原图路径及 SHA256，接受/拒绝原因。每个接受窗口只取中间原始图对求解，
其余原图用于静态检查并全部保留，不冒充独立重复实验。
每次尝试保存window_quality（每机首帧重叠数量、最大漂移、重复ID、选用ID）；
离线必须重算一致，不能中途另换一组角点。
`capture_report.json`：完成、失败/清理状态及 setup/index hash。
求解器重新验原图 hash、计数/时钟单调、静态门槛，禁止悄悄重选窗口。
`solve/report.json`：全量尝试审计、训练/留出逐窗口指标、变换和参考来源绑定。

本软件不会自动使用结果运行 SLAM，也不修改相机出厂存储。
临时 X 在 Ego/UMI 移动后失效，不能当永久双相机外参。
长期需要的是 UMI 的 M；对接 VIO 时需再使用已审计的 color↔IR/body 变换一次。
动态运行仍是三个独立运动体，世界保持 Ego SLAM 世界：
`T_W_U(t)=T_W_E(t) T_E_mount(t) inverse(M)`。
原来散码双链 5.630870mm 差异是否消除，必须等新真实配对数据验证，当前不宣称修复。

## run1失败的只读回放证据

`artifacts/spatial_bench/common_board_visibility_replay_20260906_run1/report.json`：
3次×6对原图及setup/index哈希验证；原判定均visibility_changed。
Ego/UMI每次最大漂移为0.412/0.438、0.448/0.446、0.387/0.344px，
与首帧重叠码最少为28/19、31/17、31/20；新版静态检查三次均无失败项。
这是开发回放，不是新的完整标定或独立硬件验收。原始FAIL文件未改写。

## v3：背码被两种边框检测器重复识别与续采边界

run2第14步出现“没有板却提示移走”：原图中同一个ID1被两位板检测器和
一位背码检测器解出，二者图像四边形重叠。新版仅在一位检测结果唯一且为ID1、
两位检测同为ID1、quad IoU≥0.65且中心差≤背码最短边的0.1时，归为同一个目标。
不忽略其他位置的ID1，也不忽略其他板码；出现多个背码候选仍拒绝。
新规则未改变角点坐标、40mm尺寸或任何精度门槛。

续采只适用于完成前12个板步骤后、相机和背码**确实没有动过**的中断：
`--resume-from 原目录 --confirm-unchanged --output 新目录`。
必须先退出旧预览使capture_report写完，新程序先只读校验全部旧图/质量记录/hash，
然后按已完成步数进入新目录。旧版本按旧规则验证，旧失败不重新选成通过；
输出绑定来源三份索引/报告hash，求解时重验来源，工厂参数必须完全一致。
新目录与原目录须一起保存。最后板复核不可跳过，重新启动的SDK计数分段验证，
不把两个采集段伪装成连续硬同步录像。

**若拍公共板后移动了Ego或UMI，不能使用该续采选项拼接后来的背码。**
先确认移动对象和时刻；相机关系X改变时，要在新的固定摆位重建共同板关系，
并保持该关系直到背码采完。只移走公共板则不破坏X。
用户已报告run2切换阶段时“动了”，当前等待澄清，不授权续采/求安装参数。
