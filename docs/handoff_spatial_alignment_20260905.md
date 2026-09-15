# 三设备空间对齐交接文档（→ Codex）

日期：2026-09-05。本文档把 D435i-Ego + D405-UMI 临时硬件的安装标定调查交接给 Codex 继续。
会话仓库：`/home/robot/three-device-slam`（main 分支，工作树有大量未提交改动，**不要提交**）。
运行一律：`cd /home/robot/three-device-slam && PYTHONPATH=. /opt/three-device-slam/venv/bin/python ...`。

## 1. 当前状态一句话

安装标定共识 `T_umi_ir_left_from_mount_tag` = t[0.0196, −0.0044, −0.0613]m、rpy[−179.5°, 1.7°, 179.2°]
（`artifacts/spatial_bench/mount_calibration_umic_consensus_20260905.json`）——
六个会话自洽，但用户物理卡尺（玻璃→背tag 52.1mm − 光心 datum 3.7mm = 48.4mm）与算法 z=61.3mm
差 14.6mm，**悖论未结**。双公共 tag 交叉验证工具已建好（见 §6），等一次双共视采集。

## 2. 硬件拓扑

- Ego（头）：D435i，**SDK 序列号 327122078613**；RGB 1280×720@30（锐，fx≈906.9，畸变≈0）；双目 IR 是糊的别用。
- UMI（手）：D405，**SDK 序列号 260322279785** + 外部 IMU（CP2102N，`stm32_combined_v1` 400Hz）。
- **USB sysfs 序列号与 SDK/NVM 序列号不一致**（D435i USB 显示 349643060321，D405 USB 260423073555）——
  设备归属只信 `pyrealsense2.context().query_devices()`。
- UMI 的 D405 装在用户自制壳体内，**镜片 recessed 在壳体里 ~7mm**（"镜头在壳子里面没出来"），卡尺够不到镜片。
- 背 tag：40mm id1，贴在 UMI 壳体背面（白纸基底），正对 Ego。外 tag：40mm id0 + 40mm id1（两个）。

## 3. 标定链路与工具

- 采集：`scripts/pair_color_tag_capture.py --session <dir> --ego-serial 327122078613 --umi-serial 260322279785 --duration 20`
  （单进程双 color；存 Y8 + calibration.json；D405 手动 exposure=30000µs gain=96）。
- 分析（单 tag）：`python -m three_device_slam.spatial.mount_calibration --session <dir> --mount-id 1 --external-id 0`
  → 报告写 `<session>/spatial/mount_calibration_report.json`。
- 分析（双 tag 交叉验证，本次新建）：`python -m three_device_slam.spatial.two_tag_crosscheck --session <dir> --mount-id 1 --ext0-id 0 --ext1-id 1`
  → `<session>/spatial/two_tag_crosscheck_report.json`，含 per-chain T_ego_umi、两条链一致性
  （rotation_deg / translation_m / Δz）、consensus、verdict（pass 阈值：≤3° 且 ≤5mm）。
- 实况摆位预览：`scripts/live_placement_preview.py`（DISPLAY=:0，按 q 退出）。
- 检测器合同：`T_camera_tag` 角点 TL/TR/BR/BL、右手系 z 朝相机；OpenCV DICT_APRILTAG_36H11 极性与官方相反
  （资产 PNG 白纹黑底可直接用）；黑 tag 必须垫白基底露 ≥10mm 白边。
- 测试：`PYTHONPATH=.:tests ... -m pytest tests/ -q`，当前 **872 PASS**（含新 `tests/test_two_tag_crosscheck.py`）。

## 4. z 悖论：已排除项与证据（别再重走）

用户物理：玻璃→背tag 52.1mm（复量仍 52.1），减 datum 3.7mm → **48.4mm**。
算法：z = −61.3mm（六会话 59~63 稳定，两种 UMI 摆位）。已排除：

1. **标度**：tag 尺寸卡尺 40.00mm ✓；Ego 距离卷尺验证（算法 29.9 vs 卷尺 28.7cm，含镜片内缩歧义 ±4%）；
   UMI 距离卷尺验证（两外 tag 三角 27.3 vs 27cm，1%）✓。
2. **旋转**：手工扰动实验——链路任一环 ±3° 旋转，z 只动 3~8mm，但 x 漂 ~12mm；而 140512 的 **x=8.0mm
   与用户独立物理验证 8mm 完全一致**，旋转被钉死。z 误差是**径向**（向量方向一致、纯长度差 14.6mm）。
3. **IPPE 模式翻转**：对 140512 逐帧取双候选全 4 组合算链，被接受组合是唯一横向合理者
   （其余 z=+175/+305mm 或 |t|=198mm），无隐藏正确模式。
4. **算法代码审查**：detect→IPPE→select→compose 全链路重读，角点约定/畸变/内参分派/组合方向
   全部正确，合成图回环恢复到 5mm。
5. ⚠️ **教训（前一轮会话的"破解"是错的）**：曾提出"壳体内缩 7.2mm 与 52.1+7.2+3.7=63.0 闭环"——
   **符号错误，用户已证伪**。光心在镜片后方（更靠近背 tag），只能**减**：OC→tag = 玻璃→tag − 3.7；
   壳体内缩只会让物理值更小、缺口更大。不要再用加法闭环。
6. x/y 跨会话变化 28/33mm（重贴过 tag），z 纹丝不动——算法测的是背板平面深度，被外壳约束。

剩余嫌疑（按优先级）：①多环亚度系统旋转偏差同向复合（单环 1-2° 看不出的那种，x=8 的约束掩盖了方向）；
②某个物理基准面仍错位（y 向横偏从未被物理验证；mount↔ext 距离只在 mv_left 布局量过一次 42cm，
  ego 三角预测 42.2cm——若那次量对了则 ego 侧 3D 全对）；③tag 平面与"被卡尺量的面"不是同一面。

## 5. 建议的下一步实验序列（按判别力排序）

1. **双公共 tag 采集 + crosscheck**（工具就绪，缺一次摆位）：
   两个外 tag 都摆到 UMI 侧面 Ego 也可见的位置（参照 140512 外 tag 位），间距 ≥20cm，
   对每个相机 ≥60px、入射 <45°（tag 面朝两相机中间方向斜 ~30°）。采集 20s 后跑 §3 双 tag 命令。
   - verdict=pass（两链 T_ego_umi ≤3°、Δz ≤5mm）→ 共识可信，悖论归因为物理测量，重查 §4-②③。
   - verdict=review → 两链分歧本身就是旋转偏差测量器，看哪条链的 stability 差。
2. **卷尺闭环 ego 侧**：新采集布局下量 mount↔ext0 距离，对比 ego 三角预测（全 ego 链含旋转的端到端验证）。
3. **补量 y 向横偏**：背 tag 中心相对 UMI 相机轴线的垂直偏移（算法说 +4.3mm，未被验证）。
4. 若全部指向物理测量：让用户把 D405 拆出壳体或在 tag 处贴标记重新卡尺（排除"量的不是 tag 平面"）。

## 6. 本次新建的工具（已测试）

- `three_device_slam/spatial/two_tag_crosscheck.py`：双链交叉验证 + CLI，schema
  `ego.two_device.mount_calibration.two_tag_crosscheck.v1`。同 id 碰撞（mount id1 vs 外 tag id1）
  在 Ego 侧按距离聚类：mount 参考用 `_nearer_cluster`（取近簇），外 tag 用 `_drop_mount_cluster`
  （丢掉 mount 距离 ±10cm 内的观测）。**坑**：4+4 双聚类时 3D 中位数落在两簇之间，MAD 门全放行——
  同 id 系列必须先聚类再取参考，不能先 `_reject_translation_outliers`。
- `tests/test_two_tag_crosscheck.py`：合成刚性 rig 渲染（两个外 tag 的 UMI 真值由同一 T_ego_umi 导出，
  独立选旋转会造出 64° 假分歧——刚性约束必须建进真值）。

## 7. 运维与约束

- 必须用仓库根 + `PYTHONPATH=.` + venv python；`run_three_device_slam.sh` 的 `-I` 隔离会让 /opt venv 旧
  site-packages 抢先（无单 UMI 支持）。
- /opt venv site-packages 归 root，同步需 sudo（此前被权限策略拒绝，未经用户明确批准不要再试）。
- **sudo 密码曾口头告知，绝不允许写进仓库、文档、日志**。
- USB autosuspend 修复（重启后失效）：`echo -1 > /sys/module/usbcore/parameters/autosuspend`，
  usb power/control 全 on，cpupower performance。
- 工作树很脏（标定调查的全部改动），未经用户明确要求**不要 git commit/push**。
- 已知残留 weakness：ego_color_from_umi_color pitch 有 ~1-2° 会话噪声（共识 caveat 里有记录）。

## 8. 关键工件路径

- 共识：`artifacts/spatial_bench/mount_calibration_umic_consensus_20260905.json`
- 干净参考会话：`artifacts/spatial_bench/mount_calib_20260905T140512`（z=−63.0, ego_ext rms 2.17°）、
  `mount_calib_mv_left_144304`（z=−63.2）
- 双外 tag 会话（UMI 侧双 tag，Ego 看不到外 tag，**不能**做双链）：`mount_calib_two_tag2_154217`
- tag 资产：`assets/calibration_tags/`（id0/id1 40mm PNG，150mm id0 已打印但被用户否决贴 UMI）

## 9. Codex 接手复核（2026-09-05，未提交）

- SDK 枚举确认 D435i `327122078613` 和 D405 `260322279785` 均在线。运行使用仓库根、`PYTHONPATH=.`、`/opt/three-device-slam/venv/bin/python`。
- 数值口径：共识 `|z|=61.342mm` 对物理 `48.4mm` 的差为 `12.942mm`；`14.6mm` 是参考会话 `63.0mm` 对 `48.4mm` 的差。二者均未解释；§4 的错误加法闭环继续作废。
- 实际 crosscheck 判据为两链相对旋转差 ≤3° 且 **mount z 差** ≤5mm；完整相机间平移差仅报告，未参与 verdict。§3 的“≤5mm”应按此理解。
- 复现并修正 §6 尚残留的实现顺序：10 个近处 mount 观测 + 30 个远处同 ID 外 Tag 观测，旧代码先 MAD 会删掉全部 mount，然后把远簇当 mount。现在先分物理簇，再在各簇内剔除离群点。
- 同时修正距离剔除错误作用于不同 ID 的情况：只有 external ID 与 mount ID 相同时才使用 mount 距离过滤，id0 可以与 mount 等距。报告新增各链 `ego_ext*_outliers_dropped`，保留过滤计数证据。
- 验证：两项新回归先复现失败，修正后 `test_two_tag_crosscheck.py`、`test_mount_calibration.py`、`test_pair_tag_alignment.py` 共 14 项通过。未改六会话共识或既有原始数据。
- 新的双公共 Tag 正式采集尚未完成，当前等待物理摆位。保持 mount 比同 ID 外 Tag 更近，距离差应明确超过 100mm；双相机共视、≥60px、<45°、外 Tag 间隔≥20cm的条件仍需检查。
- pass 表示所测两链一致，可按 §5 转向补量物理基准；它本身不能排除两链共享的系统偏差，不能单独证明卡尺测量错误。review 表示需结合每链 stability 定位，不能仅凭 verdict 指认某一旋转环节。

## 10. 双公共 Tag 首次实测（2026-09-05 16:49）

- 用户确认摆位后已采集 `artifacts/spatial_bench/two_tag_crosscheck_20260905T164932`：20秒，每相机600正式帧，全部正式帧CRC通过，SDK身份与交接一致。
- `two_tag_crosscheck` 已跑完：`review`；两链旋转差0.963°、相机平移差6.523mm、mount z差5.960mm；ext0/1 z分别−51.742/−57.703mm，当前未验收平均−54.722mm。
- 独立图像检查：双外Tag已共视，但在Ego中最短边仅约38.3/34.1px（要求≥60px），id0角度约44.5°且部分超过45°。本次摆位 `FAIL`。
- 四条观测中Ego→外id0的旋转RMS最大（9.8611°），Ego→外id1为0.7728°；UMI两条为2.1201/1.4360°。优先改善Ego看右侧大纸箱id0的角度和像素尺寸，两个外Tag均需变大，再复采。
- 完整结论、原始帧、示例PNG、报告、源代码/工件哈希已保存至新会话。旧共识未覆盖，物理悖论未结；下一步需要用户调整实物摆位。

## 11. 新Tag与当前预览配置（2026-09-05 17:33）

- 打印40mm `tag36h11 id2`，保留10mm白边，CUPS job9完成；资产与打印记录在 `assets/calibration_tags/external_id2_20260905/`。
- 用户实际把新id2贴在**右侧大纸箱**，替代原外部id0；左侧小纸箱仍是id1，夹爪背码仍是id1。因此当前外部ID是 **1/2**，不要沿用0/1或0/2。
- 旧预览只允许0/1，导致id2显示未知码。新增 `--external-ids` 参数，已重启：`DISPLAY=:0 PYTHONPATH=. /opt/three-device-slam/venv/bin/python -u scripts/live_placement_preview.py --external-ids 1 2`。
- 短采 `artifacts/spatial_bench/tag2_visibility_20260905T173249` 已确认两相机均识别新id2。Ego外Tag约79/65px，UMI约129/105px，角度均在45°以内。预估两个外Tag中心距约18.3cm，仍需达到计划≥20cm。
- 下一次crosscheck命令用 `--mount-id 1 --ext0-id 1 --ext1-id 2`；同ID分簇仍用于左侧外id1与背面id1。绿色预览只表示当帧观测几何合格，不包含跨相机外Tag间距或正式静态标定验收。

## 12. 合格摆位双Tag实测：旧61.3mm共识受到新证据挑战

- 已完成 `two_tag_crosscheck_20260905T173731` 及 `_repeat` 两个20秒会话，分别每相机600/599帧，正式帧CRC全部通过。外码间距约22.6cm，抽查的像素/角度/共视条件全部通过。
- 两次crosscheck均pass：旋转差1.328°/1.295°，mount z差3.911/3.494mm。平均安装z分别−46.294/−46.789mm；同摆位两次三维平移差0.606mm，旋转差0.0491°。
- 合并候选平移 `[14.335,-10.826,-46.541]mm`，RPY `[-176.934,-0.230,179.357]deg`；保存于 `artifacts/spatial_bench/mount_calibration_two_tag_candidate_20260905T173731.json`，未启用，旧共识保留。
- **§5原先“pass即可归因物理测量”的推断不能直接沿用**：本次pass对应新z约−46.54mm，与旧−61.342mm差14.8mm；与卡尺48.4mm仅差约1.86mm。不能拿新pass给旧参数背书，也不能据此指认卡尺有误。仍需排除共享系统误差并核对物理基准。
- 先保留摆位补量：两个外码中心距（预测22.6cm）；背码到左/右外码中心的空间直线距离（预测28.24/29.44cm）；背码中心相对左IR光心的y轴投影（预测−10.83mm）；新id2黑方框是否40mm。
- 详见首会话 `RESULT.md`。预览已在采集前关闭，采集全部结束。源码仅工作树改动，无commit/push，无发布或标定替换。

## 13. 用户更新物理基准与第三轮验证（2026-09-05 17:46）

- 用户最新提供：**壳子外到Tag53.3mm**，还需扣除壳体前沿→玻璃及玻璃→光心的距离。这不是旧52.1mm“玻璃到Tag”的同一基准，禁止直接混算。沿相同前后轴线，正确关系是减去这两段。
- 新会话 `artifacts/spatial_bench/two_tag_crosscheck_20260905T174646`：20秒，每相机599帧，CRC通过；32帧/相机几何抽查全部通过，外码中心距约24.5cm（布局相较上次有变化）。
- 本轮双链旋转差1.962°、z差5.179mm，verdict=**review**；平均z=−46.376mm。两条链为−43.787/−48.965mm，不将0.179mm的门槛超量舍入为pass。
- 三轮平均z=−46.294/−46.789/−46.376mm，最大差0.495mm，支持46–47mm量级的重复性。两链系统分歧与绝对精度仍待核对，旧61.342mm不能继续直接采纳。
- 53.3−46.376=6.924mm，为待独立测量的两段扣除量应满足的合计值，不是实际测量结果。当前这两段均未给出精确值；历史3.7mm datum与约7mm内缩估计需同基准复核，不能用于凑闭环。
- 本轮报告和用户物理基准记录分别在会话 `RESULT.md`、`physical_measurement_context.json`；未改旧共识或上一轮候选。

## 14. Ego调整后验证（2026-09-05 18:58）

- `two_tag_crosscheck_20260905T185830`，20秒，Ego600/UMI601正式帧，CRC全通过；几何抽查30/31帧通过。
- 双链pass：旋转差2.069度、mount z差1.621mm。平均t=[15.350,-10.413,-46.680]mm；完整mount三维平移差8.074mm，不能写成5mm三维精度通过。
- 相对17:46：安装变化3.399mm/0.206度；Ego实际移位约35.8–38.1mm、转角4.40–4.57度，未达到预先要求50mm或5度。相对17:37虽角度差8度以上，但外码布局改变，不能代替受控实验。
- UMI侧外码中位位置相对17:46变化0/1.506mm，相对17:37变化15.0/34.5mm；保留此固定布局诊断及所有review结果。
- 结论是部分检查通过、换视角验收未完成；详见新会话RESULT.md、spatial/changed_view_report.json和evidence_manifest.json。未替换旧/候选外参。
- 下一步保持UMI/外码不动，Ego沿刚才方向再移约2–3cm并保持绿色几何，复测相对17:46的独立视角条件。预览需要用外码1/2。

## 15. 独立视角一致性已完成（2026-09-05 19:50）

- 用户进一步调整Ego后完成 `two_tag_crosscheck_20260905T195005_retry`，599/599正式帧、CRC全通过、每相机32帧几何抽查通过。初次启动与预览退出短暂重叠导致枚举power-state错误，确认进程退出后重试成功；失败记录保存在无_retry目录。以后必须确认预览进程已消失再启动采集。
- 本轮crosscheck=pass：两链旋转1.636度、mount z差1.573mm。平均安装t=[15.204,-12.424,-46.460]mm、RPY=[-177.331,-0.166,179.282]deg。
- 相对18:58通过参考，Ego转角差12.739/12.644度，安装均值变化2.028mm/0.620度；逐链安装差4.270mm/0.864度和2.433mm/0.592度。达到已冻结独立视角及5mm/3度一致性条件。两参考均通过双码与抽样几何，UMI所见外码中位位置差0.225/1.426mm，支持布局基本不变。
- 相对17:46，Ego位移67.484/59.575mm及转角8.836/8.944度足够；均值安装差4.858mm/0.434度，但逐链6.390/6.355mm须保留，17:46的review不升级。
- **完成的是换视角重复性验证，不是绝对精度验收。** 本轮两链完整mount平移差6.057mm，仍超过5mm；z-only gate不代表全三维pass。Ego抽样尺寸60.2px/入射44.93度接近边界，也应在报告中保留。
- 后续不再为了重复性反复要求摆位；利用已有数据做多观测联合残差诊断，并核对独立物理基准。旧61.342mm与已有候选均未替换，结果见RESULT.md、spatial/changed_view_report.json和evidence_manifest.json。相机采集与预览均已停止。

## 16. 离线联合角点与独立留出（2026-09-05）

- 用户批准离线联合分析；新增 `three_device_slam/spatial/mount_bundle_diagnostic.py`，未改正式采集/对齐代码。训练17:46+18:58、主要留出19:50，固定内参/畸变/40mm尺度/工厂colorIR；M共有，X与两外码A按会话独立。
- 主要测试仅用外码拟合X/A，不使用测试背码seed或角点。固定距离带使外码取样也不依赖背码是否检出；这些带仅适用当前台架（Ego背码<300mm，外同ID>350mm）。
- 结果在 `artifacts/spatial_bench/joint_mount_diagnostic_20260905_v2`：留出背码RMSE11.204→5.011px、p9513.327→7.165px，约55.27%改善；训练M左IR平移[13.300,-10.055,-49.593]mm，未启用。
- 联合深度约49.6mm不同于单链平均46.5mm；这证明估计依赖拟合模型，不能宣布49.6是绝对正确，更不可用53.3-3.7凑闭环。训练残差0.42–1.05px而留出5px，仍有系统残差。
- 辅助留一：留17:46为16.934→5.890px、留18:58为12.837→7.144px；每次训练深度49.59–50.88mm。原双链x差4.5–8.6mm持续同号，z差变号，不能仅补z。
- 新6项测试加既有空间测试共20通过。独立审查无未解决Critical/Important；修复取样关联、离线时间戳跨重启兼容、非收敛标记及外码-only伪mount序列化。v1保留为审查前，v2当前。
- 待办：独立检查新右码黑边40mm、平整度/安装稳定与物理光心基准；已有观测不能唯一定位是哪个相机/Tag环节的偏差。完整绝对三维5mm与训练级标定仍未验收。不动旧共识或候选，无发布，无提交。

## 17. 用户确认尺寸与固定布局对照（2026-09-05）

- 用户回答“是，平整”，确认右侧新id2黑方框40×40mm、纸面平整。作为用户确认记录，勿再重复询问同一项；不是代理新测量，光心基准仍独立待证。
- 新离线mount_fixed_layout_diagnostic.py共享训练17:46/18:58的外码位姿，验证19:50仅用Ego外码求X。此19:50为已看过的复用验证集，不是新最终测试。
- 背码RMSE从上版5.011px变成共享布局7.168px；交叉对照fixedX-only5.395、sharedM-only7.187px。无改善，不采用；候选z=-49.605mm不启用。
- 验证UMI外码残差0.716/0.510px、Ego0.555/0.672px，不能因外码拟合好就宣称背码外参准确。该结果不支持加固定外码约束便能解决剩余偏差，但不能唯一归因硬件或证明布局移动。
- 工件 `artifacts/spatial_bench/fixed_layout_diagnostic_20260905_v1`，含2×2报告、用户确认与证据哈希。独立审查无重要项；无采集/预览、无标定替换、无提交。下一步绝对精度验证需要新独立几何证据，不继续对同一验证集调参报喜。

## 18. 左外码褶皱与替换打印（2026-09-05）

- 用户补充**左侧外Tag有褶皱**；此前“是，平整”仅确认右侧id2，不能推及左侧id1。已有残差可能受非平面标签影响，尚未通过对照证明因果。
- 用户明确要求打印替换左外码，保持tag36h11 id1、黑边40×40mm、白边各10mm，不改右id2或夹爪背id1。
- 使用既有矢量生成器新增--id/--output选项，默认id2行为保持。资产在 `assets/calibration_tags/external_id1_replacement_20260905T205121`；PNG与A4PDF均数字解码id1，300dpi渲染检测角点边长约39.878mm（栅格检测量化，不是纸张实测）。
- 已提交Brother_DCP_B7530DN_series_USB任务10，一份A4、高质量、单面、print-scaling=none；最终打印状态见同目录print_record.json。
- 下一步用户将新左码贴平到硬质平板，保留白边、不拉伸黑图案；右侧和夹爪背码不动。新采集是替换非平面目标后的新条件，保留旧数据，不将前后差异直接解释为单一算法改进。

## 19. 平整左码新采集（2026-09-05 20:55）

- `flat_left_tag_20260905T205500`，599/599正式帧CRC全通过，32抽样帧/相机几何通过；Ego最小60.3px/最大44.72度仍接近门槛。
- 原crosscheck=pass（1.583度、z差1.263mm），平均t=[16.611,-12.608,-47.796]mm。完整mount链差6.867mm，相比19:50的6.057mm无改善。
- 冻结joint-v2安装、仅新外码求相机关系，背码预测RMSE5.011→10.620px；冻结旧46.541mm候选11.204→13.279px。未用新背码重拟合M，不能宣称新数据验证通过。
- 左外码在UMI的中位位置估计变化69.44mm，右码3.38mm；Ego左码边长约69.4→60.9px。替换伴随观测几何变化，不能当作只改变褶皱的严格因果对照。
- 保留新平整码和所有旧数据；后续优先提高Ego外码观测像素/角度余量并记录摆位，不继续单靠调参掩盖残差。RESULT.md、frozen_mount_comparison.json和复现脚本均在会话内。采集/预览关闭，无参数启用、无提交。

## 20. 用户确认换箱子后独立重测（2026-09-05 21:01）

- 用户说明换箱子且码位置变了，要求重测；新 `new_box_layout_20260905T210100` 每相机599正式帧CRC通过。完全从本轮重算外码/相机关系，不使用旧箱子位姿或M，区别于上一轮冻结M预测验证。
- 原crosscheck=review：0.899度、z差6.159mm；完整mount三维差7.048mm；未接受平均t=[13.620,-12.964,-49.689]mm。
- 每相机32帧几何抽查中，Ego右外id2一帧入射45.49度且边60.4px，越过<45度门槛；Ego最小边60.3px。UMI90.8px/32.93度通过，外码间距约22.6cm。
- Ego右id2旋转RMS1.6142度大于左id1的0.4688度。下一步预览引导只调整右码朝向、左箱与夹爪不动，争取两个相机都绿且留角度余量；这只是优先改进方向，不宣称唯一根因。
- RESULT.md与spatial/placement_diagnostic.json保存细节。所有旧参数不变，无提交/发布；预览可用--external-ids1 2重新打开。

## 21. 右码角度改善但横向分歧增大（2026-09-05 21:29）

- `right_tag_adjusted_20260905T212910`，20秒、600/600正式帧CRC通过；关闭预览并等exit0后才采集，结束后无相机进程。
- 抽查30帧/相机：Ego边最小60.0px、角最大42.1度；UMI94px/31.12度，角度已改善。但外码中心距19.58–19.82cm，低于预设20cm，摆位FAIL。
- crosscheck=review：相对旋转3.134度超3度；z差仅0.067mm，不能只看z宣布成功。完整mount差15.858mm，XYZ差约[15.857,-0.203,-0.066]mm，主要横向。
- 未验收平均t=[10.790,-11.224,-49.127]mm，旧/新标定均未启用。Ego右码rotationRMS虽降至1.0934度，平均姿态分歧却更大，说明当前小幅调整不保证解决系统误差。
- 停止反复要求用户小幅摆40mm外码；建议更大公共外码/刚性多码板以提高观测分辨率及独立几何约束，夹爪背码保持40mm。新尺寸打印与算法分别配置必须先明确，现有crosscheck只有一个tag-size参数，不能直接把外码换大后仍用40mm求解。
- 不将间距差几毫米认定为15.9mm误差根因；仍需独立诊断。原始数据/RESULT/几何报告/哈希全部保留，无发布或提交。

## 22. 已打印80mm公共外码（2026-09-05 21:33）

- 用户批准打印更大公共外码：左id1、右id2，tag36h11黑方框80×80mm、白边各10mm，夹爪背id1仍40mm。资产 `assets/calibration_tags/external_80mm_pair_20260905T213217/{left_id1,right_id2}`。
- 生成器新增--size-mm40/80、默认40；PNG白边物理尺寸和PDF矢量尺寸分别核验。两PDF300dpi解码id1/id2，检测边约79.925mm（栅格量化），名义80mm，文件哈希一致。
- Brother任务11两个单页PDF各一份，A4、单面、高质量、print-scaling=none；21:33:44确认completed且printeridle。print_record.json在父目录。
- 新左id1/右id2贴平到硬板、保留白边；旧小外码移除或完全遮挡。夹爪背码不动；两外码中心建议约25cm留余量。
- **下一次预览/求解前先实现混合尺寸**：外码80mm、背码40mm。现有two_tag_crosscheck/live_placement_preview/verify_geometry/mount_bundle_diagnostic仍统一40mm，不可直接使用；不能只将全局tag-size改80，否则背码也被放大。同id1的背码与左外码须正确区分物理实例后再应用尺寸，原近远簇阈值可能被错误尺度影响。
- 本轮只完成打印模板/资产，没有声称混合尺寸算法已完成，无采集/标定启用/提交。

## 23. 打印任务11实际只出一张（用户纠正）

- 用户报告只拿到一张，推翻“两张已打印完成”的实际出纸结论。CUPScompleted只证明后台状态，不等于人工确认两张纸。
- /var/log/cups/error_log记录Job11过滤警告：file is damaged、xref not found、Attempting to reconstruct cross-reference table。两个源PDF各自pdfinfo正常且先前分别栅格解码成功，多文件打印链路异常可疑，尚未确定漏掉哪张或唯一根因。
- 已询问用户手上是ID1还是ID2，待答复后只用单文件新任务补打缺失那张，不再把两PDF放同一个lp任务。尚未提交任何补打，避免重复。print_record.json已标记physical_sheets_received=1与received_tag_id=null。
- 用户随后确认拿到ID2。已核对ID1源PDF哈希一致，单文件补打左ID1的80mm外码一份，Brother任务12，A4单面/原尺寸/高质量。实际收到补打纸张仍需用户确认，不把后台completed等同于物理收纸。
- 21:46:34核实任务12后台completed、打印机idle，日志无新的Job12 PDF警告；等待用户确认左ID1实际收纸/贴好。下一步仍必须先实现混合尺寸80mm外码与40mm背码，再开预览求解。

## 24. 混合80/40mm路径已实现，当前待解除遮挡（2026-09-05 21:56）

- 用户补打后说“好了”；实时图像已看到箱子上的ID1和ID2，纸张实测尺寸仍待确认。
- 新独立模块 `three_device_slam/spatial/mixed_size_crosscheck.py`：外ID1/2=80mm、背ID1=40mm。通过唯一ID2的双相机姿态预测外ID1在Ego的像素中心，仅做离散身份选择（误差<=80px、候选差>=40px）；然后保留两外码各自独立的PnP姿态。禁止旧错误尺寸距离排序，禁止用身份预测强制两链一致。
- 所有原始帧检查CRC；正式帧检查非递增时间、无效帧及host-mapped/global_time时钟域；配对<=20ms、一对一。默认每10帧抽查，关联覆盖>=90%、>=8对；所有已关联样本保留参与两链计算，几何失败不被静默丢弃。总体pass同时要求所有抽样关联对几何通过（>=60px、<45度、外码中心>=20cm）与原旋转<=3度/z差<=5mm门槛。完整三维差单列，不等于绝对5mm精度认证。
- 用scipy旋转均值避免旧trace四元数在精确180度时奇异。原40mm分析程序不改，不能用于新80mm数据。新CLI：`PYTHONPATH=. /opt/three-device-slam/venv/bin/python -m three_device_slam.spatial.mixed_size_crosscheck --session <新会话>`，输出spatial/mixed80_40，拒绝覆盖已有输出。
- 新预览 `scripts/live_mixed_placement_preview.py --snapshot-dir <新目录>`，外80/背40分别标注。Ego开启自动曝光以纠正暗图，D435投射器关闭，UMI曝光30000us/gain96。完整try/finally释放所有已启动pipeline；实时预览未硬件同步，不能拿预览当正式验收。
- 测试：新11项+旧相关测试共29 passed；独立审查提出时钟域检查问题，修复并补缺失/不支持域回归后复审无Critical/Important。未修改正式标定、未提交。
- 最新预览 `artifacts/spatial_bench/mixed80_preview_20260905T215620/`，运行中的工具会话77701（须现查进程）。Ego识别外ID1/2，但右下背码被画面右缘裁切；UMI的两个外码下缘被夹爪遮挡，未识别。已请用户将背码移入Ego完整视野、箱子垫高解除夹爪遮挡。**本轮没有新采集或标定通过结果。**
- 下一步：用户调整后先读取latest.json/预览；满足身份与几何再关闭预览，等待明确exit0/释放相机，然后用pair_color_tag_capture采20秒，调用新mixed_size_crosscheck分析。必要时补充纸张80mm实测。不得并行占用相机、不得启用新标定或宣称绝对计量闭环。

## 25. 首轮混合80/40mm采集完成，深度分歧仍超门槛（21:59）

- `mixed80_crosscheck_20260905T215930`：600/600正式帧，CRC含预热655/646全通过；Ego30.0056Hz、UMI30.0107Hz，时间单调/时钟域检查通过。
- 本次stride1逐帧分析：598/600对成功关联（2对因配对时差/重复配对门槛拒绝），所有598对几何通过，最小113.159px、最大44.266度、外码中心距至少211.576mm；接受配对时差最大16.661ms。
- 两链rotation0.608442度通过，但mount z差6.493663mm超5mm；完整三维差6.540477mm，主要z。ID1链t=[15.403,-9.972,-54.451]mm，ID2=[14.655,-9.746,-47.957]mm。未接受平均[15.029,-9.859,-51.204]mm，无启用。
- 分四段z差6.598/6.684/6.416/6.412mm，当前20秒内持续存在，不是孤立跳帧；不可据此认定唯一根因，也不是40mm→80mm单变量对照（摆位也变化）。
- 预览和采集均已关闭，无相机进程。RESULT.md保存完整指标、限制、命令，SHA256SUMS保存原始帧/索引/标定快照/源代码/报告哈希。
- 已请用户保持当前摆放，分别实测左右新外码黑方框的宽和高是否80mm，不含白边。纸张实测未完成前不擅改tag-size补偿。壳体53.3mm仍不能直接与光学坐标z比较；绝对外参未验收、未提交/发布。

## 26. 固定图像算法排查：角点前端影响已复现，另有存储布局缺陷

- 用户确认新外码是80mm，要求继续算法调查。本轮不采集、不修改正式检测器/标定。`mixed80_crosscheck_20260905T215930/spatial/fixed_image_ablation_all_v1/`保留报告/全部角点位姿/验证/RESULT，脚本在其父spatial目录。
- 冻结原598关联帧对、身份、尺寸、K/D；baseline角点和位姿逐值精确复现原始audit。深度差对照：原整数角点IPPE6.494mm；cornerSubPix5x5为6.218mm；OpenCV CORNER_REFINE_APRILTAG前端4.523mm（完整3D4.846mm、旋转0.292度）；原角点iterativePnP6.349mm。
- AprilTag前端方案四段z差4.530/4.526/4.536/4.498mm，所有新几何、正深度、残差/IPPE候选间隙检查通过。平均t=[14.736,-8.894,-49.683]mm仅诊断候选，**未启用**。这不是独立留出数据，不能声称绝对标定已完成。
- 单角色更换表明Ego外码角点敏感性明显；只换背码几乎不改两链差。不择优混搭角色来凑小误差。该OpenCV选项会改变四边形/角点前端，不等于已接入官方独立AprilRobotics库，也不能归因全部只因整数取整。
- 不含背码/工厂colorIR的外码相对变换已存在7.503mm平移差（精修后4.927mm）。说明偏差上游已存在，不能全部归咎壳体到光心的物理基准；标量外码中心距接近不代表完整SE3一致。
- **实证修正早期“算法全正确”的过强结论**：采集把SDK平铺rotation标成rowmajor，mount_calibration按rowmajor读取；本机SDK计算和官方rs2_transform_point_to_point均确认原数组实际columnmajor。示例点[.015,-.01,-.05]m误差0.242mm，转置匹配到约3e-6mm。正式修复尚未做，须区分SDK原数组与真正rowmajor的旧测试/其他生产者，不能一概转置。
- 该存储错误**不是6.5mm链差主因**：两链同乘刚体变换，完整三维差不变；不乘/转置colorIR均为6.540477mm，z仅差约0.003mm。畸变对照全4784个UMI外码角点SDK/OpenCV去畸变射线差至多0.004032等效px，无大幅符号错误证据，但不能据此证明物理内参准确。
- 独立只读审查确认598样本baseline完全匹配、11960个位姿全部有限且角点正深度，无影响诊断结论的Critical/Important问题。不要拿60帧探索的7.172→4.463mm直接与全量6.494比较。
- 下一步可实现有开关的AprilTag前端改进与单独的来源可辨识SDK布局修复/回归，再独立新视角复核。当前仅完成诊断，不放宽阈值，不覆盖原REVIEW报告，不启用49.683mm新M，不提交/发布。

## 27. 用户批准角点前端接入，软件与原始数据重放已验证

- 用户“接入”后完成正式代码路径改动：共享AprilTagDetectorConfig新增corner_refinement=`none|apriltag`且默认none，保留旧调用行为；mixed_size_crosscheck的detect_pair/compute/CLI及live_mixed_placement_preview默认apriltag，并有`--corner-refinement none`回退对照开关。无隐式降级，原尺寸/身份/IPPE/几何门槛不变。
- 报告记录backend/OpenCV版本/精修方法/求解器；新结果写spatial/mixed80_40_apriltag，旧none仍mixed80_40，均拒绝覆盖。预览终端、叠字、latest.json记录方法。使用文档docs/acceptance/mixed80-corner-refinement.md。
- 83项相关测试通过（聚焦48项）；独立审查无Critical/Important。完整600对新入口重放598关联、全部几何及原始CRC/时间检查通过，rotation0.291883度/z4.522605mm/full3D4.845943mm，与先前精修诊断的chains/agreement完全一致。
- none模式完整600对重放的逐帧audit、chains、agreement与原版完全一致，旧REVIEW报告未改。接入前3个源文件副本保存在spatial/frontend_integration_source_before，供旧哈希溯源；当前源码变化不应改写旧哈希。
- 新报告spatial/mixed80_40_apriltag/integration_acceptance.json保存测试/审查/源码报告哈希与验收范围。代码接入通过不等于绝对外参验收；仍NOT_ACTIVATED。SDK旋转存储问题是单独已知问题，本次未改；正式三设备coordinator也未变。
- 本轮仅重放，无硬件占用；预览CLI参数验证通过但未开实时窗口。下一步用新前端的新视角采集独立验证，不要求用户重复测80mm、不直接启用49.683mm候选。无提交/发布/外仓写入。

## 28. 新视角预览与多余ID1区分

- 用户要求打开预览并“区分”多余码，随后表示位置换好。初始屏幕副本推断只适用于此前截图；最新画面第三个ID1实际是左箱后方一张物理小码，已明确纠正。
- associate允许Ego>=2个ID1，仍由唯一ID2双相机锚点预测外ID1中心，原80px误差/40px候选差不变。其他ID1按40mm求姿态，仅一个候选位于估计UMI原点150mm宽半径内才能作为背码；此先验是本体邻近范围，不是49/54mm待标定结果，不择优挑最近者，不拟合外码链。
- 两个以上邻近候选拒绝；任何候选40mm姿态拒绝/无法唯一匹配也必须拒绝（独立审查发现并修复“真实背码求姿态失败后误选近处备用码”的漏洞，两个回归用例覆盖）。多EgoID2或多UMI公共ID仍拒绝，未实现一般多目标/屏幕语义识别。
- preview_resolved_roles只用于显示：已唯一识别的外码可独立按几何标绿/黄，未分配候选黄，完整身份成功后多余码灰。部分标签不能进入标定compute。顶部区分身份失败与角度/大小/间距失败。
- 88项相关测试通过，独立复审无Critical/Important；598旧有效帧对身份/位姿未变，背码到UMI原点约49.47–50.80mm。实机新图：背码约50mm，后方备用码约460mm，被正确排除。
- 当前预览运行toolsession93971，目录`artifacts/spatial_bench/mixed80_identity_preview_20260905T224800`。五角色已正确区分；Ego外码角约49.7/48.4度超<45，用户已获指导只降低Ego、箱子/夹爪不动。尚未采新视角数据；保持预览，调整后再查latest.json，关闭并等待ownershiprelease后才采集。无新M启用/提交/外仓写入。

## 29. 独立新视角已采集：一致性通过，完整三维偏差仍未解决

- 用户再次“好了”后，最新预览五角色均接受，最小约106px/最大43.8度；关闭93971，确认exit0和ownershiprelease后采20秒。新会话`artifacts/spatial_bench/mixed80_newview_20260905T225115`，599/599正式帧，含预热654/645个CRC全通过；29.98777/30.00857Hz，无无效正式帧/时间倒退/不支持时钟域。预览与采集均已停止。
- 冻结APRILTAG前端、尺寸和门槛，stride1为598/599关联成功，全部关联几何通过：最小105.797px、最大43.8907度、外码中心距至少207.597mm、配对时差最大16.648ms（host-mapped，不是硬同步）。
- 新精修两链旋转1.335762度、z差2.237095mm，原<=3度/<=5mm-z门槛pass；完整三维差5.630932mm，delta=[5.034426,1.165055,2.237095]mm。四段z2.254/2.237/2.246/2.222mm，完整3D约5.61–5.64mm持续存在。**不能说完整3D达到5mm。**
- 新数据旧none前端对照同样pass：1.099033度/z1.340215mm/full3D5.504673mm，略小于精修。本轮明确否定“角点改进已普遍解决偏差”的强结论；保持预先选定方法，不按会话挑更好结果，不重新拟合尺寸/内参。
- 新未启用共识t=[13.511,-8.775,-45.416]mm。与21:59精修[14.736,-8.894,-49.683]mm比较，位置变化4.440954mm/旋转0.320571度，通过本次5mm/3度重复性门槛；这是两次估计的比较，不是冻结M的预测/绝对计量验收。
- 两链估计Ego/UMI视角变化75.79/75.72mm、10.05/10.71度；UMI外码1/2观测也变化1.972/40.171mm、6.034/11.148度，不能当作严格只移动Ego且外码布局不变的对照，也不能仅凭姿态变化断言具体哪件物体移动。
- `compare_views.py`、`view_comparison.json`、`RESULT.md`保存可复现分析、原始数据/源代码/旧参考报告SHA256。原始数据与旧报告未覆盖。当前无相机进程，无新M启用/提交/外仓写入，绝对外参、IMU/SLAM/机械臂训练仍未验收。
- 不要求用户继续小幅挪位置。下一步软件工作应先单独修正已实证SDK旋转存储布局问题（必须按来源兼容真rowmajor数据并加回归），再用已保存帧检查外码链重投影/姿态系统误差；该小布局缺陷不是5.63mm完整链差的解释。正式外参启用仍需完整3D/绝对精度证据。

## 30. SDK存储缺陷已修复；上游持续成像/模型偏差待独立标定观测

- 用户要求“不要停直到排查出问题所在并修复”。已完成可独立复现的SDK布局bug修复，**不把它说成剩余5.63mm链差的根因或修复**。新增`devices/realsense_extrinsics.py`，三个本仓库SDK写入入口统一输出真正rowmajor并带嵌套v2/schema/source/layout/convention；平移方向不反转。真实旧快照仅按完整文件SHA256两条白名单转置；未知SDK旧数据和冲突/反向convention拒绝，无来源标记的真正旧rowmajor保持原语义。不要按serial/工厂schema/文件名一概转置，也不要靠删来源字段绕过拒绝。
- `mount_calibration._color_to_ir_transform`接入新reader。mixed报告增加`color_to_ir_read`，新版目录`spatial/mixed80_40_apriltag_layout_v2`/`mixed80_40_layout_v2`拒绝覆盖，保留旧报告。新实现限制/兼容策略见`docs/acceptance/realsense-extrinsics-layout.md`。旧统一40mm工具依然不能用于80/40mm现场。
- 回归先3failed/1passed复现错误，再修复。独立审查发现旧无layout反向convention可误入fallback，修复并补3回归后复审无Critical/Important。最终201项相关测试通过（234.93秒；非全仓测试），包含20个布局测试。新22:51完整CLI重放exit0，599采样/598关联，原始CRC/几何仍通过，逐帧audit与旧报告完全一致。
- `artifacts/spatial_bench/rotation_layout_fix_20260905/frozen_replay.json`对两组各598冻结相机姿态重算：旧结果精确复现，修复后安装位置变化0.211–0.244mm/姿态0.536352度；SDK三测试点最大误差2.094e-8m。完整链差仍4.845943/5.630932mm，刚体共同左乘保持范数。新22:51未启用均值[13.718,-8.787,-45.352]mm，仅记录，不覆盖任何正式M。
- 官方AprilRobotics源码固定b7c0ebe9aa20f82ec7a828579004f9e706bfecd9，clone/build仅在上述artifacts目录，未install/未替换正式检测器。数字ID1/2×四转角验证固定角点次序。每会话固定60帧对：官方full3D5.006/5.670mm，OpenCV4.791/5.626mm；减0.5px像素原点、先角点中位数后PnP均未消除偏差。整幅图案ECC只改善新视角、恶化旧视角6.471/4.579mm，未采用。各诊断是探索性复用数据，不是新留出验收。
- 不使用背码及colorIR的逐帧外码转投影，持续bias坐标RMS1.52–2.56px，传播后的时间波动RMS0.15–0.17px（独立审查复算）；支持持续上游不一致，不是只由静态随机角点抖动或壳体测量造成。
- 外码-only联合拟合两相机关系（80mm固定，目标函数不含背码/安装距离，每会话布局自由）得到跨会话M变化2.25152mm/0.22768度，但冻结21:59M预测22:51背码仍3.65060px欧氏RMSE、p954.72662px。不拿优化收敛或共用X强制一致当外参验收。`external_fit_report.json`留证。
- 独立于链闭合的稀疏内参探索拟合：Ego每会话仅3个独立tag姿态，自由K焦距两会话差异很大且跨视角预测一般变坏；D405仅2姿态，4内参+12姿态参数=16像素坐标残差，无冗余，OpenCV拒绝（**计数本身不证明雅可比欠秩**）。说明目前证据不足可靠分离内参与打印码形状/平整度，不能宣称厂家内参已证明错，也不能用拟合K/尺寸把两链凑齐。
- 当前真正缺的独立观测：平整硬底AprilGrid、多方向/多图像位置的实际color流。已异步问用户是否还有以前的平整AprilGrid，等待物理配合再采，不继续让用户微调两个箱子。现存config名35mm但实际`tagSize=0.0352`、6×6、间隙比例0.3；必须先核对实物，不能仅依文件名用35mm。已有camchain文件属于2UQ2，未找到匹配当前D435i/D405的color标定，勿混用。
- 所有本轮结果/脚本/失败对照/最终哈希在该artifact目录与`RESULT.md`/`acceptance.json`，source_before保存/精确重建并验证旧源码字节。frozen_replay内helperhash为审查前版本，对应归档副本；最终hash另存，不改旧证据。无硬件采集/相机占用/新M启用/提交/发布/外仓或opt写入。SDK修复完成，剩余链差根因和最终精度仍未结。

## 31. 2026-09-06 独立彩色内参采集入口与当前预览

- 用户确认有平整标定板；尚待核对是否6×6、黑边35.2mm、白间隙10.56mm。配置值不等于实物验证。
- 新入口`scripts/color_aprilgrid_capture.py`，双机1280×720@30，预览5Hz、录制独立PNG最高2Hz。**按R才开始90秒**，Q/ESC保留中止数据；保存逐帧SDK时钟域/帧号/主机到达时间/图片SHA256及质量提示，非同步SLAM采集。好坏样本均保留，后续离线筛选、留出验证，未启用任何内外参。
- 实际数字板自检暴露旧aprilgrid库缩图漏检与角点黑框尺寸问题；此新入口使用OpenCV36h11/CORNER_REFINE_APRILTAG，四方向标准图36/36、60px边长验证。中心点网格H仅为预览提示，不证明板子物理尺寸或标定质量。旧2UQ2代码未修改。
- 50项相关测试通过；独立审查提出的close失败可能跳过相机释放已修复并回归，复审无Critical/Important。
- **当前占用相机的是toolsession62577**，输出`artifacts/spatial_bench/color_aprilgrid_20260906T000333`。已亲看新鲜preview.jpg，双机正常；画面仍是箱子ID1/2和背码ID1，没有AprilGrid，未录制、saved=0。请用户遮旧码、放板，确认尺寸；按R前不要自动采集。另起采集必须先停此预览并等待两机释放。无提交/外仓或opt写入。

## 32. 2026-09-06 出厂内参原则恢复、模型传递修复

（后续最新状态见§33；本节的无采集状态是当时状态。）

- **以本节为当前状态**：用户强调UMI以出厂内参/畸变为准，已停止独立内参重标定，不求新K/D。全部标定预览已关闭，最后36999退出0、两机释放，started=false/样本0；前述62577、31173也是已退出状态。没有相机占用或标定参数启用。
- 实物旧板是2-bit黑边+黑角部连接图案；早先1-bit数字板测试与aprilgrid库2-bit不兼容，不能据此断言旧库角点错。新采集预览最终用markerBorderBits2+SUBPIX才实测Ego36/36、UMI34/36；只影响标定板预览，正式空间对齐外码仍1-bit、80/40mm。实物尺寸在只读D405正式资产明确35.2mm/10.56mm，用户沿用同板，无需重复量。
- 新`scripts/audit_factory_camera_usage.py`冻结旧/新两会话各598帧5角色全部角点，对比安装SDK2.58.2纯几何反投影和旧OpenCV算法，不拟合K/D。2398正式color帧的索引尺寸/灰度载荷大小与各自1280×720出厂内参匹配，未发现IR/color内参混用；两份出厂calibration.json哈希与旧证据相同。
- 确认实际软件缺陷：`pair_tag_alignment`读取了畸变系数，却丢掉`distortion_model`，把inverseBrown当普通OpenCVBrown。修复`CameraCalibration`保留/验证模型，inverseBrown使用SDK反投影得到归一化点进入IPPE，SDK正投影计算候选像素误差和混合ID关联；模型未知时拒绝，不静默降级。历史无model测试保留显式opencv默认；出厂数值不变。两个live placement preview也补传str(intr.model)。
- 修复范围不是重新标定，也不改变tag尺寸、检测角点、选支门限、关联门限。新CLI报告目录`mixed80_40_apriltag_factory_model_v3`（none为`mixed80_40_factory_model_v3`），旧unsuffixed/layout_v2结果不覆盖。报告带camera_models/backend。
- 定量影响：真实UMI角点最大约0.0041px；全视场627点最大0.00794px；单tag平移变化<0.0018mm。两链完整3D差4.845943→4.845308mm、5.630932→5.630870mm。**这是真实代码缺陷，但不是剩余5–6mm根因。** 不许称空间对齐已经修好。
- 新9项factory模型测试先7项全红再修复，最终94项相关测试通过1.51s；独立审查指出两个live构造漏传已修复并补表达式隔离回归，复审无Critical/Important。完整新视角CLI599帧exit0、598关联、全部原始CRC和几何检查通过；独立verifier逐帧确认598相同身份/角点、常规选支结果与冻结SDK诊断完全一致。现有pass是z/旋转门限，不是full3D<5mm或绝对精度验收。
- 留证`artifacts/spatial_bench/factory_usage_audit_20260906/`：原始report、postfix/report、source_before、verify.py、verification.json（25哈希）；文档`docs/acceptance/factory-camera-model.md`。旧诊断脚本仍按历史OpenCV K/D模型解释，不应把重新运行它们当新的模型一致性验收。未改D405外仓、ego_vio_humble、任何opt发布目录；未提交，未启用K/D/M。

## 33. 2026-09-06 四组固定出厂板诊断完成，定位旧板角点窗口问题

- 当前无相机占用。最后UMI倒转采集tool68813正常exit0释放；用户已完成四组30s，无需立即再摆板。Ego正常/倒转、UMI正常/倒转各50原始PNG，共200；预览初筛分别46/39/34/27。逐图SHA、setup绑定、尺寸、帧号/SDK时间/主机到达时间单调性全部通过。保存频率约1.667Hz是主动抽样，不是SLAM掉帧。每组operator_review.json保存检查细节。
- 有效四组目录：`board_factory_ego_normal_20260906T002443`、`board_factory_ego_rot180_run2_20260906T0030`、`board_factory_umi_normal_20260906T003607`、`board_factory_umi_rot180_20260906T0040`，均在artifacts/spatial_bench。早先`board_factory_ego_rot180_20260906T0028`虽然标签写rotated180，实际仍正向，明确排除倒转对照；原始元数据不改。
- 新`scripts/fixed_factory_board_diagnostic.py`实现固定出厂K/D+SDK模型、仅训练棋盘奇偶tag求6DOF板位姿，另一组预测并交换。训练>=6tag且>=3行/列；无全板位姿或预览good筛选泄漏；保存每角点像素/板面等效残差、身份、位姿、跳过原因。实际旧板角点顺序经独立额外正向控制图36/36核验，合成图四角约定测试通过。
- 原板诊断角点留出RMS约2.36/2.45/2.32/2.34px，训练误差同样约2.3px；发现轮廓角点向内偏。OpenCV4.10默认relativeCornerRefinmentWinSize0.3使标称5px实际窗口随码元缩至约1px，旧板黑色连接方块附近的初始角点没有充分收敛。仅诊断增加`--refine5`显式5px cornerSubPix，固定原解码ID/门槛/分组，未改正式空间tag检测器或采集器。
- 独立已知几何50px码/15px黑连接方块合成图复现角点RMS2.91548→0.000043px。负对照若换成5px孤立黑点反而变差，因此不能泛化到普通1-bit外码。新测试先失败再实现；最终33项板诊断/比较/采集/出厂模型测试通过0.19s，独立审查无Critical/Important。
- 排除用于探索的Ego-normal第0图后，199图全部保留计数（含训练覆盖不足者），286个可匹配有效fold全部改善。留出RMS(px)：Ego正常2.3635→0.3475，Ego倒转2.4527→0.4195，UMI正常2.3193→0.2694，UMI倒转2.3368→0.2516；p95分别0.6169/0.7693/0.4550/0.4297。图像、源hash、有效ID、训练/留出分组、跳过状态逐项一致，不是筛掉差样本。相邻图相关，不应声称199次独立测量。
- 证据：`fixed_factory_board_prediction_20260906_run1/report.json`原结果；`fixed_factory_board_prediction_20260906_refine5/report.json`精修结果、`comparison_199.json`明确排除开发帧。`scripts/compare_fixed_factory_board_reports.py`可重现比较。baseline_v2用当前源码重放旧基线，旧报告不覆盖。详情docs/acceptance/fixed-factory-board-diagnostic.md。
- **已修复的是旧板诊断角点定位，不是原外部1-bit双链5–6mm差。** K/D全程固定，无新安装M启用、无相机flash、无外仓或发布目录写入、无提交。板姿态覆盖有限、局部夹爪遮挡，残差小也不能证明出厂K/D绝对正确或印刷计量合格。
- 下一步应以修正后的板诊断约束剩余假设，核对原两外码所在图像区域/姿态是否被这些板数据覆盖，再判断是否需要更有针对性的独立证据。不要重复§26已经做过的普通cornerSubPix5外码对照并当作新修复；该旧外码对照仅把z6.494降至6.218mm，官方AprilTag/ECC等也已测试见§30。不要直接把本节refine5套到正式1-bit外码；不得把板角点改进等同安装/绝对精度验收。

## 34. 2026-09-06 开源路线已获批准，新公共板工作流软件完成

- 用户先要求“去看看开源的”，随后“好”批准实现。调研见
  docs/acceptance/opensource-spatial-calibration-route-20260906.md，实际操作和
  冻结门槛见 docs/acceptance/common-board-calibration-workflow.md。
- 新 scripts/common_board_capture.py 双预览8Hz目标刷新（不是保证实测帧率），
  两原生color流1280×720@30；Ego蓝框/UMI绿框，R才采每个静态窗口，
  Q/ESC关闭保留证据。16步=8train+4holdout板、3背码、最后板复核。
  组内两机固定，只移动板；3+对/1–2秒、首帧角点漂移<=0.75px、
  到达差<=75ms只是静态代理，不是曝光硬同步或td标定。
- 新 scripts/common_board_calibration.py 验证全部原图hash/时钟/窗口门槛，
  固定已审计SDK成像模型/旧板尺寸，联合训练X和板位姿；背码不进X。
  归一化SDK射线float64优化避免SDKfloat32数值微分问题，最终原SDK像素评分。
  留出整姿态做双向单侧板姿态→对侧预测，各p95<=1px；全3D门槛5mm/3deg。
  背码阶段后板复核可以检测相机关系改变，不用于重新拟合X。
- Review抓到只隔开slot仍会接受重复训练姿态的假独立验证；已红绿测试修正：
  各留出UMI单目板姿态相对训练/此前留出至少20mm或法向7deg差异。
  不用Ego留出残差选图，所有留出都报告。最后复核仅豁免新颖性。
- 第一组最多CANDIDATE_NEEDS_INDEPENDENT_SETUP；第二独立Ego摆位重新采集，
  --reference 指第一组report，冻结第一组M预测新背码；新组M只报告重复性。
  PASS_INDEPENDENT_MOUNT仍不自动启用，也非绝对计量或正式三SLAM验收。
- 75项聚焦/回归测试通过，含22新工作流测试：真值、有限噪声、SDKinverseBrown、
  留出不影响X、相机移动/重复姿态拒绝、固定M独立组CLI、模拟硬件完整16步、
  未按R不录像、故障释放、原始hash篡改拒绝。复审无剩余Critical/Important。
  /opt仅使用Python/SDK几何，未启相机；没有外仓写入/提交/参数激活。
- **原散码full3D5.630870mm仍未解决。下一步需要真实新配对采集，旧四组单目
  文件不可按序号冒充共视数据。** 当前无owned采集进程。先用上述操作文档第一
  条命令开预览，等用户摆好再逐步采，不要默默启动或连续要求盲挪箱子。

## 35. 2026-09-06 首轮共视采集漏码误拒绝已修复（尚未完成标定）

- 用户运行common_board_train_20260906_run1，3次都卡第1步visibility_changed，
  Ctrl+C停止且cleanup=[]。36PNG及setup/indexhash核验；每码相对自身首次
  出现的最大漂移E0.412/0.448/0.387px、UMI0.438/0.446/0.344px。
  不是超0.75px运动，实际是每帧码数变化（UMI22–31）被严格集合相等拒绝。
- 新schema=v2：每帧相对首帧至少12共同码/3行3列；检查所有重见码相对自身
  首次观测的漂移，晚出现码也检查，不按残差筛点。中间图对仅用双方都重见
  的共同ID并要求12/3x3；window_quality保存且离线严格复算一致。
  背码仍须ID1持续可见。0.75px、1px留出、5mm/3deg精度门槛未提高。
- Preview增加滚动静态预检WAIT/HOLD/READY、首帧重叠数、实际漂移及中文解释。
  R仍是手动采，未ready也可保留诊断但不放宽判定；Q/异常释放保持。
- scripts/audit_common_board_visibility.py与
  artifacts/spatial_bench/common_board_visibility_replay_20260906_run1/report.json
  保留3次旧失败与新静态回放均通过的对比，严格DIAGNOSTIC_ONLY；不改原v1FAIL。
  84项回归测试通过，审查无Critical/Important；本轮未启动相机。
- 下一步：用户按操作文档以common_board_train_20260906_run2启动（输出应新建），
  两机固定，只移动板，等READY R后按R逐步完成16步。原5.630870mm空间链差仍未解决，
  不能把静态门槛修复称为标定成功。main未提交，外仓/opt仍只读。

## 36. 2026-09-06 run4 完整采集已求解，REVIEW（未启用）

- run2 背码误认公共板已用 v3 的同ID+同四边形排重修复；94回归测试通过。
  用户随后明确两相机都移动过，所以没有续采run2。run3仅开预览；公共板
  ID1与背码ID1同时可见会触发重复ID，操作上先遮背码，背码阶段再揭开。
- 新数据 `artifacts/spatial_bench/common_board_train_20260906_run4` 完成16步，
  24次尝试、144对/288原图，cleanup空，离线全部hash/静态判定重放通过。
  已运行 common_board_calibration.py，结果 `run4/solve/report.json`，exit2表示
  REVIEW而非程序崩溃。不要重复写同一solve目录，不要启用候选安装外参。
- 公共板法向跨度18.594deg、位置跨度293.951mm；训练残差、相机一致性、
  双向留出、新颖性未通过。step3/6/9/12/16的Ego->UMI p95分别
  2.765/2.381/2.377/3.944/4.391px（门槛1px）；反向1.421/.892/.987/.977/.915px。
  step6对训练step7仅0.430mm/0.167deg，不能更改split或删掉它来通过。
  末步与此前已有同方向偏差，单凭末步失败不能认定用户碰动相机。
- 背码三窗口自身p95 .402–.425px，但X未通过，M仍不可信；不要把候选
  z=-47.369mm当成正确值或卡尺验证。原散码5.630870mm问题仍未闭合。
- 同13对图独立单目PnP（固定SDK K/D）：Ego RMS .317–.492px、UMI .157–.283px；
  每窗口中位码边Ego约38–42px/UMI55–69px。抽查attempt016中间原图无明显
  拖影，UMI裁掉板上部。低单目重投影残差不证明内参或空间外参准确。
- 本轮无采集进程启动、未改算法/门槛/K/D/板尺寸、未提交、未激活。下一步
  用现有数据排查姿态相关跨机偏差，区分姿态不确定度与系统偏差；不盲目重采，
  不以留出目标残差选择分支/修正模型后宣称独立验证通过。
- 补查全部13窗口×2相机的两个IPPE初值经LM后：或收敛同一位姿，或另一解
  源图RMS明显更差（Ego2.768–8.463px、UMI33.482–47.620px），未发现近等价
  双解选错证据。未使用对侧留出残差选分支。报告SHA256为
  `191f51f5d52f0742b66c5c7948eddb5d5e53c615bdcdd1cf9d4d5f1453d9916f`。

## 37. run4 进一步只读排查；不要继续盲目重采

- 相同目标函数换dense数值雅可比，6次求值收敛，cost128.2180461，X仅差
  .000656mm，留出结果不变；没有原求解器未收敛的证据。未修改算法。
- 每窗口全6帧共同ID子集重复PnP：8窗口有>=8共同码，X组内最大两两波动
  .690–2.705mm/.073–.453deg；其余5窗口不足8码跳过。仅诊断，不改变正式
  中间帧选码、12码静态门槛，也不能与不同ID的正式结果作严格同分布比较。
- 背景LK窄ROI[20,20,180,300]相对首步，末步位移中位E[.166,.102]px、
  UMI[-.275,-.013]px，但p95为5.342/3.172px，有误匹配且视场狭窄；不足以
  证明相机没动，更不能据此把责任归为用户移动。
- 固定K/D，仅train8同名角点拟合essential几何（不使用板布局/尺寸，线性
  loss，无RANSAC删点），R变.793deg、平移方向变.340deg；留出Sampson p95
  .808/.320/.629/.256/.233px。它只是一维极线条件、没有米制尺度，**不能把
  它冒充原两维重投影的PASS**。step3相比原.690px还变差，未隔离唯一根因。
- 用原未验收X三角化检查板：拟合平面RMS .319–.527mm，各窗口码边中位
  35.058–35.292mm，模型35.2mm。没有明显尺寸数量级错误；但用了未验收X，
  不是独立量具，不能证明板绝对平整/尺寸正确，也不能据此拟合板形后放行。
- 以上五项shell只读诊断均exit0；没有新的标定PASS，没有更换K/D/写硬件、
  提交或启相机。原报告SHA不变。下一步建议取得用户同意后做**离线诊断副本
  的成像参数对照**，原出厂K/D及D405 SLAM仍保持不变。应分别检验假设、
  固定训练/验证分割并报告退化风险；不把拟合改善直接认定厂家内参错误。

## 38. 用户授权离线内参对照已完成：不应替换出厂K

- 用户回复“比较”，允许诊断副本，不允许正式替换。新增独立脚本
  `scripts/common_board_intrinsics_comparison.py` 和对应测试；原capture/solver
  及设备配置未改。固定D和畸变枚举、板布局/尺寸、train8/holdout5及全部选中点。
  预先冻结5组：factory、Ego焦距同比例、UMI焦距同比例、Ego K4、UMI K4；
  焦距限定出厂0.9–1.1，主点偏移±40px，没有碰边界，没有拟合背码M。
- 两份报告在 `artifacts/spatial_bench/common_board_intrinsics_compare_20260906_run1/`
  和同名run2的`report.json`；run2使用2倍SDK K数值导数步长验证稳定性。
  run2各组最差双向留出p95：4.391 / 5.860 / 4.126 / 6.470 / 4.848px。
  **全部未过1px，保留出厂K。** Ego训练拟合明显改善但留出step3变差，不能用
  训练误差下降宣称内参更准。畸变D未尝试，不能宣称所有成像参数假设已排除。
- SDK deprojection是float32，显式中心差分log焦距1e-4、主点.02px；run2加倍。
  两次最大留出p95差.003392px，K元素差.019101px。优化器ftol/xtol停止且
  optimality仍约2–26，不说已证明一阶最优；结论对这次步长变化稳定。
- 目标函数是候选SDK归一化射线乘候选fx/fy，不是精确像素SSE；最终评分用
  实际SDK投影。出厂模型保留，copy出的候选仅写诊断JSON，不导出可部署标定。
  初始测试先确认缺模块红灯；后53回归通过，含已知焦距误差恢复、留出不能
  影响X/K、非零畸变SDK inverseBrown K4与主点恢复及双步长比较。只读审查
  无Critical/Important，补齐了审查指出的测试/目标函数说明/源码绑定。
- run2源文件hash/当前源码hash逐个验证，原run4报告SHA仍为§36值。
  本轮比较结束，但原空间链误差根因仍未唯一定位，禁止自动启用/声称标定完成。

## 39. 继续排查：畸变候选交叉验证与 Ego RGB/IR 独立参照（2026-09-06）

- 用户要求继续直到查明；允许离线K/D诊断，不允许自动替换出厂值。最新确认
  公共板是厂家硬质标定板，并明确允许10秒RGB＋左右IR补采。补采已完成，
  不是尚待授权。相机已停，无设备占用，投射器保持原用户要求关闭。
- D-only报告 `common_board_distortion_compare_20260906_run1/report.json`：
  Ego k1=.056214958使全部10项paired holdout改善，最差4.391→1.782px；
  UMI k1仍4.347px；EgoD5最差1.392px但k3触界，不采用。
  老Ego单机板片段冻结k1后normal p95 .617→.672变差、rot180 .769→.671改善。
  原单比特80mm外tag的冻结候选检查：原摆位full3D4.845→9.119mm变差，
  新摆位5.631→3.581mm改善。保留全部归档角点/身份、pose gate拒绝数0。
- KD报告 `common_board_coupled_compare_20260906_run1/report.json`：
  Ego focal+k1最差1.935px；EgoK4+D5最差1.030px但k3触界。冻结focal+k1
  外tag检查原摆位4.845→8.572mm变差、新摆位5.631→3.482mm改善。均未采用。
- `common_board_edge_compare_20260906_run1/report.json`：独立边缘交点定位
  同样本/ID，factory最差4.814px（原corner5为4.391）；冻结k1后2.043px
  （原1.782）。不支持仅靠换cornerSubPix修复；也不证明彻底排除角点误差。

### 新10秒三路数据与对照

- `artifacts/spatial_bench/ego_stereo_color_probe_20260906_run1`：20组三路、
  60PNG，IR-L/IR-R/RGB均1280×720@30；每秒落盘约2组。保存出厂内外参、
  各路SDK时间/域/帧号、PNG/hash、setup/index绑定。IR左右时间差全部0；
  不声称RGB曝光同步。IR-R←IR-L baseline50.04744mm，列主序/方向已复核。
- 新脚本 `ego_stereo_color_probe.py`、`ego_stereo_color_validate.py`。
  验证使用IR双目三角化→RGB投影，不使用板尺寸、平面模型或PnP。
  基线 `solve/report.json` 因IR漏检0有效。诊断IR阈值C11/max53，3倍放大
  仅解码，按像素中心缩回原坐标再native refine5；不能改正式检测器。
  `solve_ir_wide_3x/report.json` 10有效/20组，样本5用于检测器开发，单列排除
  后9组：IR p95 .063–.096px，RGB p952.044–2.179px。RGB漂移≤.475px，
  IR≤.128px。重复时间样本相关，不是9个独立几何姿态。
  同ID边缘交点对照 `solve_ir_wide_3x_edge` RGB仍约2.28–2.67px。
- `analyze_ego_stereo_color_probe.py` 冻结旧paired k1、不重拟K/D：保持厂家
  RGB←IR外参时RGB误差变为4.314–5.403px，反而更差。局部SE3仅用首个
  非开发样本2、偶数checker标签拟合，奇数标签留出；不能部署该局部解。
- **诊断数值缺陷已修复**：原 `frozen_candidate_analysis/report.json` 的
  local fit使用默认微小差分穿过SDK float32，结论不可靠！固定外参评分
  不受影响。保留原报告，不引用其localfit数值。显式中央差分1e-5/2e-5的
  `frozen_candidate_analysis_v2` / `frozen_candidate_analysis_step2` 才有效：
  最差奇数标签p95 factory1.091/1.090px，冻结k1 .779/.783px；局部SE3补偿
  能反转排名，说明单一平面姿态的K/D/外参混淆仍未解除，不是厂家外参错误
  或k1正确的证明。优化终止也不证明精确驻点。已加非零SDK畸变SE3恢复测试。
- IR重投影小只能约束极线误差，不能约束水平视差系统误差。三角化标签边长
  中位数约35.56–35.62mm、平面RMS约.72–1.12mm，也不能当厂家板物理计量。
  不得把当前结果归咎卡尺、板材或单一镜头畸变；原5.631mm问题未唯一定位。
- 当前56项相关回归通过3.39秒；原run4 solve报告SHA仍为§36值。main、无提交，
  外部D405/Ego仓库和发布目录未写，正式标定未启用。

### 下一步（需要用户移动板并授权下一次有界补采）

两台相机不动，只改变板在图中位置/倾角/距离，再补三路数据以检查冻结模型
是否跨姿态成立。建议板先下移约10cm并倾斜约15°，另一个姿态靠近约10cm；
看预览确认共视，不盲目重复16步。当前许可覆盖的10秒已经采完，不能把它
当无限制自动重采许可。未得到额外授权不得开下一次相机。最终决定必须依据
多姿态独立验证，不再单凭一次训练误差或单平面补偿的低误差替换K/D/M。

## 40. 第二姿态已采，冻结单姿态修正失败；双姿态候选待第三姿态验证

- 用户回复“好了”后已授权并完成第二个10秒三路采集：
  `artifacts/spatial_bench/ego_stereo_color_probe_20260906_run2`，20组三路、
  60PNG、清理无错误、投射器关闭。相机现已停止，不能沿用本次许可继续重采。
- 背码露出使IR解码ID1与板ID1重复，第一次验证安全退出且未生成输出。
  新增显式 `--exclude-board-id 1`，所有流/所有帧统一排除身份歧义；不按
  残差挑点。run1同样排除重放，所有原有效测量完全不变，ID1为奇数parity
  本来也没进旧SE3偶数训练。正式检测器未改。开发样本改为显式参数：
  run1 `--detector-development-sample 5`；run2无开发样本，不误排新数据5号。
- `run2/solve_ir_wide_3x_noid1/report.json`：11/20有效；同样阈值/3倍解码/
  native refine5。原factoryRGBp95为2.427–2.577px，静态漂移无明显问题。
  raw/setup/index/派生sample与metadata及点数、时间域/帧号均复核。
- `scripts/crosscheck_ego_stereo_color_poses.py` 不含求解器、不重新拟合，
  直接冻结run1的 `frozen_candidate_analysis_v2` 预测run2：
  `run2/frozen_pose_crosscheck/report.json` 最差factory-local2.942px、
  k1+local1.757px，均FAIL1px。用旧2倍差分候选重放为2.897/1.762px。
  只读审查独立逐项复现。不要把旧姿态上低误差当通用修复。
- 然后明确新增**探索性**双姿态外参拟合，非独立姿态验收：
  `scripts/fit_ego_stereo_color_two_poses.py`，输出
  `run2/two_pose_exploratory_fit/report.json`。两组各首个有效frame的偶数
  checker标签训练SE3（run1 frame2、run2 frame0），K/D冻结旧paired k1，
  不重新拟合K/D，不用奇数标签训练。两组奇数标签最差p95：
  factory[1.612,.765]px；k1[.664,.256]px。2倍差分为factory[1.616,.758]、
  k1[.667,.254]。报告状态EXPLORATORY_REQUIRES_THIRD_POSE，无激活。
- 这说明畸变/外参联合候选值得独立验证，不证明厂家RGB内参唯一有误，也
  不证明修复RGB↔IR就会消除原外tag两链5.631mm差异。IR视差/定位偏差仍
  是参照不确定性；最终必须重做原两链验证，不能偷换验收目标。
- NEXT：需要用户摆第三姿态并许可10秒。建议相机不动，板向左约10cm，
  相对现在反方向倾斜约15°，确保三路可见。**先冻结双姿态报告中k1候选
  和两个差分版本，第三姿态不得参与重新拟合，再评分全部有效角点。**
  不要又先拟合第三姿态再声称它独立通过。main不提交，外部仓库/发布只读。
- 第三姿态主候选预先固定 `label=frozen_paired_board_k1,jac_step=1e-5`；
  2e-5仅数值敏感性对照，不按第三姿态分数选优。全新姿态全部有效标签
  评分，每帧p95≤1px仍为诊断像素门槛，不是整套安装标定通过。至少3个
  有效静态帧、每帧≥8共同tag、漂移≤.75px，缺失/歧义仍显式报告。
  k1候选的SE3补偿含约+8.86mm Z平移，可能吸收IR视差系统误差，不是测得
  真实RGB-IR外参变化。末项只读复算一致；相关61回归通过3.37秒，原run4
  报告SHA未变。增加拒绝两次相同来源的保护，本次来源确实不同。

## 41. 第三姿态冻结验证失败，已排查静止性和原生IR角点视差（2026-09-06）

- 用户“好了”授权的第三个10秒已采完：`ego_stereo_color_probe_20260906_run3`，
  20组/60PNG，emitter关闭，清理无错，相机已停。`solve_ir_wide_3x_noid1`
  沿用冻结检测器/身份排除策略；16有效、4共同tag不足，无开发样本。
- 新 `scripts/validate_ego_stereo_color_frozen_fit.py` 硬绑定训练候选SHA
  `84cd4e3b4e70aa822b2000d48c4931adc475b1b5572fa03650edc243daac7e5d`，
  主候选仍k1/1e-5，来源不同于训练，全部有效角点评分，无重新拟合。
  `run3/frozen_two_pose_validation_with_residuals/report.json` 是详细报告：
  **FAIL**，主16帧p95范围1.03258–1.27574px，全部超过1；2倍步长最差1.28693。
  factory局部SE3最差2.76161px。16帧static/IR timestamp/support均通过，
  不是用户没拿稳或某一尾帧失常。只读审查逐项复算完全一致。
- 保存每角点残差，不删除失败数据。第三姿态残差x/y RMS约.603/.416px，
  相同tag时间噪声通常.07–.12px，呈区域性稳定偏差。沿固定IR左射线微扰深度
  后，垂直于RGB深度投影方向的残差仍RMS.402/p95.711px，不能单靠改深度
  消除全部误差；也不能据此排除IR射线/外参/角点误差、唯一归因RGB。
- 新 `scripts/native_ir_disparity_check.py`，`run3/native_ir_disparity_check`：
  每姿态首个既有有效非开发帧，原生IR水平NCC，11/19像素patch、±2px范围，
  不拟K/D/T、不删除低分点。第三姿态|视差修正|p95仅.074/.099px，最小相关
  .9982/.9969，零搜索边界命中。将其作为诊断替代视差后RGBp95仍1.168/1.158px
  （该帧原1.195px），不支持明显角点视差错位可独自修复。它是posthoc检查，
  不能更改冻结验证FAIL；patch未建透视形变/辐射模型，不是真值。
- 原三姿态深度只有约.798/.768/.778m。审查员同名44角点长距离比例检查
  未见明显整体尺度随姿态漂移，但不能排除恒定视差偏差。不能只看平面残差，
  恒定视差偏差也可能保留平面。下一步需要显著近远距离变化或独立计量，
  而不是不断重复同距离横移。需重新获得有界采集许可，当前相机已停。
- 不重拟第三姿态后声称验证通过，不激活任何K/D/T/M；原5.631mm两链问题
  尚未唯一定位。出厂/正式配置和外部D405仓库未写，main不提交。

## 42. 靠近尝试run4：像素门槛通过，但静止门槛失败

- 用户回复“好了”授权的近距离10秒已完成：
  `artifacts/spatial_bench/ego_stereo_color_probe_20260906_run4`，20组/60PNG，
  清理无错，emitter关闭，相机已停止。沿用现有验证程序，无本轮代码修改。
- `solve_ir_wide_3x_noid1/report.json`：20组均≥8共同tag且可三角测量。
  红外估计中位光轴深度.63357m，仍未达到请求的.45–.50m；板倾斜且每帧
  检出ID不同，不能将逐帧深度中位数范围.622–.655m直接解释为轴向运动。
  原图能看到手扶板。不同于前三组，本组确实更近，但距离跨度仍较有限。
- `frozen_two_pose_validation/report.json`：主候选k1/1e-5保持SHA冻结，
  全20帧p95为.598379–.934444px，均满足1px；2e-5最差.948278px。
  原始factory投影p95为2.33669–2.55237px。
- **总体仍FAIL**：最大漂移RGB1.978646px、IR-L1.346182px、IR-R1.418548px，
  超过预设.75px。static失败frame=[2,4,7,8,9,10,11,12,13,14,15,16,17,18,19]。
  不删除这15帧、保留5帧就改判通过，不事后放宽门槛。像素改善和本组未通过
  静态验收须同时说明，不能用本组覆盖第三姿态的像素FAIL或原两链5.631mm。
- NEXT：需将板靠在稳固支撑上，而非手扶，进一步靠近Ego约10–15cm并确保
  全板可见，再取得新10秒采集许可。不要声称本次已到45–50cm或已经完成标定。
  无参数激活、无提交，外部仓库/发布只读。

## 43. run5近距离角点缺陷已复现；旧公共板误差基本不变

- 本轮用户“好了”授权10秒，`ego_stereo_color_probe_20260906_run5`保存20组/
  60PNG，emitter关闭，清理无错，相机已停。IR中位深度约0.420496m。
  frames.jsonl SHA为 `440cd93dee5fc753edff3b15313dac64822b42dadae9a8794eceb46878a8b972`。
- 找到真实软件缺陷：近距离legacy两位边框板的初始quad偶尔内缩5–7px，
  固定半窗口5px够不到真实外角，cornerSubPix会停在seed或错误局部位置。
  sample0/2 tag23及sample15 tag15/35可重现，扩大窗口8/10/12的结果相近。
  新 `refine_legacy_corners_scaled` 用 `max(5,ceil(.15*median_edge))`，
  只依几何尺度选择，不按残差挑窗口。旧函数/default/正式采集保持不变；
  仅离线验证器显式 `--scale-aware-corners` 启用，报告记录策略。
- `run5/solve_scale_aware_noid1`及`paired_corner_fix_comparison_v2`保留同20帧、
  同全部共同ID/策略，未重拟K/D/T。tag15/35/23相对各自时间中位角点的最大
  跳变分别9.742/8.076/11.412→.528/.364/.336px。报告的首帧参考RGB最大
  漂移9.839→1.103px，仍超过.75，不能说静态通过。
- 修复后factory RGB p95约2.332–2.498px；旧冻结k1+SE3仍4.041–4.992px，
  比factory差，不能用于通用标定。原固定5验证及第三姿态FAIL完整保留。
  新策略输入旧冻结验证器实际测试拒绝，未创建输出。三处policy guards
  防止posthoc角点改动冒充原冻结验收；配对脚本校验策略方向/其他策略/
  帧数/身份/开发帧，始终POSTHOC_DIAGNOSTIC_ONLY。
- 为检验与最初问题的关系，新增 `scripts/common_board_corner_scale_diagnostic.py`：
  先完整回放原run4的raw哈希/旧静态验收；按原中间图、selected IDs、13窗及
  train8/holdout5划分，重新decode原seed且fixed5必须精确复现原pixels，
  再换尺度精修。出厂K/D、板尺寸/布局不变，只重估X与训练板姿态。
  `common_board_corner_scale_20260906_run1/report.json`：最差heldout p95
  **4.391011907→4.389196551px，双方REVIEW**。近距离角点缺陷不能解释
  最早公共板的主要失败，更不能宣称原5.630870mm两链差已修复。
- 下一步不应盲目重复摆板/16步。已有跨深度图像可用于离线模型可辨识性
  分析，但这些姿态已经看过结果，重新拟合它们只能叫开发，不能独立验收。
  任何后续硬件重启需新的有界采集许可。出厂配置/正式coordinator未改，
  原数据报告不覆盖，外部D405仓库及/opt只读，main不提交。
- 最终验证：80项相关回归通过（3.49秒）；独立代码复审无Critical/Important。
  `git diff --check`无输出；原run4 solve SHA仍为191f51f5…，冻结候选SHA仍为
  84cd4e3b…；无探针/公共板采集进程。修复范围仅显式启用的离线legacy板精修。

## 44. 公共板光心到Tag距离审计：公式正确，单图欧氏几何也有失配

- 用户询问“多个固定间距Tag公共板求距离”并授权检查，未授权新采集。
  新证据 `artifacts/spatial_bench/common_board_distance_audit_20260906_run1/RESULT.md`，
  只写该诊断目录与工作文档，不改正式代码/KD/配置。原完整raw/static门槛重放。
- 程序本来已用6×6固定布局，每窗共同12–24码/48–96角点；35.2mm黑边、
  10.56mm间隙、45.76mm中心距，配置与代码一致，不等于独立实物计量。
  X=T_E_B inv(T_U_B)、M=inv(X)T_E_mount方向正确，换板坐标系不影响X。
  原M直线距离50.125445mm，与轴向z=-47.369158mm不同，不得混为一谈。
- 固定同一Ego背码观测，13个整板姿态各自独立X推算M，距离43.187–54.302mm，
  位置最大两两18.130mm。是推算不一致，不是实际安装变化或新的准确距离。
- 同图同标签奇偶训练/留出，对比固定KD刚体PnP与8DOF一般H（无筛点）：
  Ego25/26组H更好，最差留出p95 1.3245→.6044px；UMI24/26，.7469→.3981px。
  Ego归一化H轴夹角88.824–90.852°，UMI89.986–90.079°；这支持当前板坐标/
  成像模型组合的单图欧氏约束存在失配，不能单凭跨步相机移动解释这部分。
  Ego侧更突出，但角点、板几何、模型与其他成像误差尚未唯一分离。
- 5组既有EgoIR三角点相对>100mm名义板中心距也偏+.7–1.0%。run5不同角点
  策略、各组不同可见ID均注明；不据此改板尺寸/IR基线或拟合深度补偿。
  IR和配置板尺寸都不是已证明的绝对真值。原5.630870mm链差未解决。
- 合成12姿态X/M恢复、错误焦距负对照及H/PnP留出故障无泄漏检查通过，
  46项相关回归通过；独立只读审查无Critical/Important。原求解SHA不变。
  NEXT（规格来源更正见§45）：以UMI已有板规格继续核对相机实际图像通道和
  模型，不把缺厂家规格当阻塞；不要盲目再采16步，不按现有已看过的留出
  分数选参数并称独立验收。相机保持停止。

## 45. 用户提醒UMI标定目录：已有板规格，不应重复索要

- 用户明确指向UMI标定文件夹。只读核对来源：
  `/home/robot/umi_docker_device2_d405_formal_20260828/calibration_assets/aprilgrid_6x6_35mm.yaml`，
  原注释“对应实体打印板：tag边长3.52cm，间距1.056cm”。6×6、tagSize=.0352、
  tagSpacing=.3，即中心距45.76mm；不是文件名暗示的35.0mm。
- `/home/robot/releases/umi_device2_d405_product_1.0.2-20260901/calibration_assets/aprilgrid_6x6_35mm.yaml`
  内容相同，两文件SHA256均
  `e0cb280349fa0ca8792462f7864e818f138094890965f6cf4d6dc4413c0eb4bc`。
  当前config及run4 setup.target数值逐项匹配，原计算已用正确的既有规格。
- §32已经记过用户沿用同板、无需重复量。上一轮再次要求用户提供厂家规格
  是遗漏已有证据；已纠正plan/findings/RESULT，不以此阻塞继续排查，也不
  把既有规格变成随意拟合参数。绝对计量不确定性不等于“规格文件不存在”。
- 同时只读找到UMI calibration_sessions中d405_stereo/report.yaml：attempt007，
  policy=USE_INTEL_FACTORY_RECTIFIED_INTRINSICS，IR1280×720@30，fx/fy649.206665、
  zeroD，独立IR极线p95=.333353px。此PASS是IR校正/采集/极线范围，不是当前
  color光心对应的内参（color fx656.2879/fy654.4534、非零inverseBrown）。
  不得将IR camchain直接套到color图像上，也不能当Ego RGB已验收。
- 本轮没有相机启动、参数变更、正式代码修改或外仓写入；仅本仓库文档
  纠正，main未提交。后续从已有标定记录继续检查模型/图像对应关系。

## 46. 用户投诉漏码：同图证实检测路径差异，修复并开启摆位预检

- 只读核对UMI原采集`collect_calib_data.py`、`device_manifest.yaml`和已安装包：
  原为aprilgrid0.5.0/Detector(t36h11)，逐帧重新检测，并不是把旧码缓存不动。
  这里此前用OpenCV ArUco两位边框检测，不能把结果跳变都归于用户移动。
- 原run4的13板窗口×6帧×2相机=156原图哈希校验后比较：
  OpenCV Ego平均27.026个，相邻ID对称差平均8.4；UMI20.949个/3.738。
  新native adapter Ego36个/0，UMI23.423个/.10769；0异常、0重复ID帧。
  原aprilgrid默认最大1000px会缩图，UMI有1次边界cornerSubPix异常；
  新路径large_image_threshold=2000，当前1280×720不缩图，未触发异常。
  证据`artifacts/spatial_bench/board_detector_ab_20260906_run1/`。
  `compare_initial.py`SHA10724af4...保存原A/B源代码，`compare.py`另含adapter模式。
- 新`scripts/common_board_detector.py`通过family代理保留原始decodeQuad结果，
  不沿用上游按最大面积合并同ID的行为（会误吞公共板ID1与背码ID1）。
  canonical角点重排[1,0,3,2]，实际生成码四旋转测试通过。孤立合成码可能
  解出嵌套同ID：保留冲突并拒绝，不选更像预期的角点。真实156板图无此冲突。
- capture新增`--board-detector umi-aprilgrid`，显式opt-in；缺省和旧会话仍opencv。
  setup记录backend，续采先于开相机拒绝换backend，离线父子再次检查；solver
  报告包括backend和adapter代码hash。旧报告/原图不改，K/D不改。
  display独立保留当帧raw并区分visible/valid/dup；重复ID黄色框，不再看成整板0。
- 新`--placement-check-only`先做mount->board->mount，300s上限；第一次R之后
  两台相机不能动，只进出公共板/遮背码。最终mount像素相对初始≤.75px；
  保存6PNG+hash+质量/确认信息，完成状态NOT_CALIBRATION，load_session拒收。
  只证明条件性可见性/静态代理，不证明两相机绝对没动；正式末步板闭环仍必要。
  若必须转Ego才能看到背码，不得沿用板阶段X。此时Q重新摆位，不靠代码绕过。
- 只读review发现旧preview_samples每.2s更新，R可能用旧READY接受当前失效图；
  已新增current_preview_quality把当前显示图纳入同一窗口，负测覆盖当前丢板/
  duplicate；review复核关闭。53项聚焦测试、90项相关回归通过，review允许有界实时摆位预览。
- 原同支持几何对照`solve_same_support.py`先完整load_session验原记录，随后
  在slot8/left发现原selected IDs17、29未被新检测器找到，因此拒绝删点并中止。
  没有新求解残差，不能把检测稳定说成已修复4.391px/5.631mm。原solve/report
  SHA191f51f5d52f0742b66c5c7948eddb5d5e53c615bdcdd1cf9d4d5f1453d9916f不变。
- 最新实时：exec session84894，`common_board_placement_umi_detector_20260906_run1`，
  D435i327122078613/D405260322279785，SDK序列号实查一致，emitter0，1280×720@30。
  最后处于placement1/3提示移走公共板；用户须点预览R，当前尚非标定采集。
  旧preview session8548已Ctrl-C退出，0attempts、cleanup=[]，不要以旧progress误判仍运行。
  接手先查实时进程/报告，避免重复占相机；300秒自动停，Q/ESC可提前停。

## 47. 用户要求立即落实同框：已接入simultaneous采集/求解并开预览

- §46分时流程不是数学必需，用户明确要公共板/背码一起在视野里。现新增
  `--simultaneous --board-detector umi-aprilgrid`，不是只说方案。新文件
  `scripts/common_board_joint.py`；capture/calibration按模式调用，旧会话不改。
- 冻结身份策略`board35_plus_separate_mount1_v1`：两机均事先排除公共板ID1，
  保留其原图、灰色显示；其他35码求板。不按残差、安装距离筛选ID。
  至少12码/3行3列的所有非1角点拟合单应矩阵（无RANSAC/删点），身份用p95≤5px、
  max≤12px。预测完整板黑码外轮廓并扩大间隔10.56mm；背码1必须在轮廓外，
  quad无交叠、每角点相距>3px、边长≥60px。一位边框唯一外部ID1才可作mount，
  二位检测器外部ID1都须与这个同物理quad匹配，额外ID1和非1重号拒绝。
  该单应矩阵只区分目标，不用于替代刚体标定或改K/D；身份门槛≠精度门槛。
  同码物理归属仍由操作者确认：独立ID1确实贴在UMI上，其他散码遮住。
- 每一步同时检测Ego板+背码/UMI板；window_quality分别检查板和mount全窗口，
  任一缺失/运动/歧义拒绝。板可正常显示，即使背码尚未找到。Ego板蓝色、
  UMI板绿色、确认背码紫色。所有16步保留原split，但13–15步改同框静态复测，
  16最后同框复核，不再移板。两相机在首次R后仍固定，不能误解为动态手眼采集。
- setup保存observation_mode、完整identity_policy、新plan和代码hash。续采
  开机前检查模式，离线再次验父子模式；policy/plan不匹配拒绝。原始PNG重检，
  mount角点和板角点来自同一个中间Ego原图，audit保存same_frame_mount_corners。
  主求解仍固定factoryK/D/layout，用既有训练/留出求X和三次mount窗口求M。
  新same_frame_check每个板窗口求X_j=T_E_B inv(T_U_B)，固定M预测当帧背码，
  每窗p95≤1px，任一失败降为REVIEW。独立验证使用旧组referenceM，不就地改M。
  各帧IPPE M分支全部列为诊断，不按“哪个接近卡尺”选择。
- 103项相关回归通过9.91s；独立只读review11项内存/真解码/负向检查通过，
  无Critical/Important阻碍有界预览。合成真实legacy黑接点6x6+同ID1背码成功；
  初版孤立码合成图触发nestedduplicates，改正fixture为实际黑接点布局，未放宽门槛。
  `board_detector_ab_20260906_run1/joint_replay/report.json`实际旧16窗口反例：
 13板阶段Ego35/UMI19–24，mount0正确拒绝；3背码阶段board0正确拒绝，
  不能误认板ID1作背码，也不能拿旧分时资料当新同框标定。
- 最新execsession26845，`artifacts/spatial_bench/common_board_joint_20260906_run1`，
  两台SDK序列号与§46一致，1280×720@30，投射器0，预览目标8Hz，30min有界。
  首次输出STEP1/16 PREVIEW ONLY，尚无R采集；提醒用户板+背码一起放，
  紫框出现且两机板稳定READY再R。接手必须先查此进程，不能重复占用相机。
  旧session84894已300秒超时、cleanup=[]、0attempts，别再用旧运行状态。
- 工作仅本仓库main未提交，其他仓库/发布目录只读，原报告SHA191f51...不变。
  尚未获得新同框实际合格窗口，原空间残差/5.631mm问题不可宣称已解决。
  随后实机预览证实同框成功：Ego35/UMI35/Mount1，多次READY，最近漂移.53px；
  期间.83/1.36px正常HOLD，不能当全时稳定验收。PID3312286仍在STEP1预览等R，
  尚未保存正式窗口。用户无需再移出公共板，板背码共存的软件障碍已现场解除。

## 48. 用户纠正只做静态分析：已有两段直接算，不再要求R/16步

- 用户问“为什么要按r？不是直接根据静态各tag分析吗”。承认流程扩张，
  不把16步标定强加给静态诊断。原预览exec26845/PID3312286正常Ctrl-C停止；
  读取发现用户期间已按R保存两段，各6对，completed_steps=2/attempts=2，
  cleanup=[]。因此直接用已有资料，未实现/启动多余自动采样，未重采。
- 新scripts/common_board_static_analysis.py读取allow_partial同框会话，原图/
  hash/静态验收全量重放后分析；保留35个选用ID、所有正深度精修IPPE分支，
  按本相机像素RMS排序，不按预期安装距离挑分支。固定出厂K/D/板尺寸。
  逐码4点独立链、35码整板链、棋盘奇偶组互相留出均输出，不激活任何参数。
- 两段Mxyz（UMIcolor光心→背码中心，mm）：[24.195,-13.159,-49.355]、
  [24.274,-13.739,-49.727]；欧氏长度56.519/57.016，三维位置差.693704mm。
  仅近同姿态重复性，不是绝对正确。Ego全板p95.635/.647，UMI.356/.358px。
- 逐小码独立链范围49.576–137.814/43.581–142.366mm，相对全板位置差
  中位26/27mm,p9590/94mm，旋转差中位~3.7°/p95~13°。四点小码姿态敏感且
  部分分支歧义，不能拿每个距离当真实光心距离。也不因此宣布原80mm散码
 5.631mm链差已全部归因。奇偶组M仍差2.573/3.420mm，组内留出像素<.737，
  说明全板更稳但模型/观测系统偏差尚未清除；仍不是标定PASS。
- evidence:common_board_joint_20260906_run1/static_analysis/report.json及RESULT.md。
  105项相关测试通过9.95s，只读review独立精确复现两窗口，无Critical/Important。
  相机已停止，无额外硬件动作/外仓写入/提交。后续继续静态诊断，不自动切回16步。

## 49. 用户要求 36 码多测：全部 12 对原图、816 次固定分组对照完成

- 新 `scripts/common_board_resampling_analysis.py`，3 项新测试；先全量复核原
  35 码策略和原图哈希，再按固定几何身份门槛离线恢复板内 ID1。唯一板内候选、
  全部非1角点无RANSAC单应预测规范角点最大差≤3px，不按距离择近。
  实测最大 .355px；12帧两机每帧36码，两段恢复后的静态检查无失败。
  此为派生诊断，不改变live35码策略或原采集验收。
- 全部12帧而非仅两张中间帧；432条逐码链、68固定分组×12=816次拟合。
  groups：全板35/36，奇偶/上下/左右两半，6行/列，逐一去掉行/列/单码。
  无按结果筛帧、删码、调K/D或尺寸；全分支/留出残差/身份/代码hash保存。
- 全板Mnorm56.007–56.876mm，平均xyz[23.928,-13.015,-49.346]mm，
  最大帧间位置差1.381mm；恢复ID1相对35码位置变化中位.508mm。
  奇偶组差.697–2.202mm（中位1.542）；上下组11.644–15.623mm（13.619）；
  左右组2.760–4.203mm（3.517）。上下差向量平均[-3.320,13.135,1.197]mm，
  旋转1.570–2.099°。比较背码三维位置，不是距离长度差或物理误差真值。
- 上下组交叉预测另一半角点，Ego留出p95跨帧中位1.987/1.419px，
  UMI .421/.842px。优先查区域残差与姿态敏感性，但半板支撑几何更窄，
  不能据此唯一认定厂家K/D错误，也不能将重复计算称独立精度验收。
- 108项相关测试通过9.90s；独立只读复核精确复现12帧/432链/816拟合，
  原始hash一致，无Critical/Important。源数据和旧报告保持不变，相机停止，
  主分支未提交，外仓只读，NOT_ACTIVATED。证据
  `common_board_joint_20260906_run1/all36_resampling/{report.json,RESULT.md}`。
  原5.631mm双外码分歧仍未完全归因，不得宣称修复/标定PASS。

## 50. 用户要求实际修复：已做成像模型隔离及候选标定，通用回归未通过

- 全部12对原图/36码新增 `common_board_pose_ablation.py`：native/fixed5/
  scaled/edge/实际像素优化上下组中位差均约13mm；换优化器/精修窗口不是主修复。
  反事实仅Ego理想像素降至.866mm，仅UMI理想像素仍14.085mm，两侧理想.000316mm。
  定位主导不一致在Ego观测/模型，不是唯一证明厂家内参错误，不能部署理想像素。
- 新 `common_board_camera_recalibration.py`，只用旧run4冻结8train单相机拟合
  Ego K4/K4+k1/K4+k1+k2；5holdout及新12对不进入参数拟合，所有结果保留。
  K4+k1: fx904.096670/fy907.165813/cx657.488321/cy378.211710/k1.0508938741，
  显式OpenCV forward Brown，与旧SDK inverseBrown参数不可直接混同；工厂值未改。
  新背码也用每组候选重算。上下中位13.6185→2.5664mm、最大15.623→3.376mm；
  左右最大仍5.450mm，旧pairedholdout最差4.391→1.522px，均非正式标定PASS。
- 新 `validate_ego_camera_candidate_external.py`，CRC审计、设备/KD身份核对，
  两历史场景各598对固定归档角点，不重新选身份，所有posegate拒绝均0：
  21:59双链4.845→9.826mm变差；22:51为5.631→4.835mm改善。因此拒绝通用
  激活，不能用公共板变好宣称已修复。候选全板51.238–52.117mm亦非真值。
- 独立审查发现pose_ablation派生ref帧/背码绑定缺口，已修：完整有序对照
  原accepted rows，从当前raw重新求mount并与ref核对；负向测试覆盖删/重/序/
  改source/伪mount。该工具缺陷修复不等于原几何主问题修复。v1证据保留，
  v2重新执行数值同；camera_recalibration_v2只修正分支记录说明，数值精确同v1。
- 114相关测试通过10.07s，独立复现3候选/oldjoint/全部外部回归，无Critical/
  Important。证据 `common_board_joint_20260906_run1/POSE_REPAIR_RESULT.md`，
  `pose_ablation_v2/report.json`、`camera_recalibration_v2/report.json`、
  `camera_recalibration/external_regression/report.json`（绑定v1原哈希，勿覆盖）。
- 已异步问用户当前相机/板/背码是否保持静止并允许10秒Ego RGB+IR-L+IR-R
  同框核对，尚未收到确认；没启动相机，不要求R/16步。若确认，先查设备占用，
  使用已有ego_stereo_color_probe.py --record-now有界10秒新输出目录，emitter0，
  结束后用统一新检测器离线核对当前同框身份及静态、IR三角点到RGB残差。
  **用户未确认前不要启动硬件。** 旧5组IR/RGB不等同当前同框独立证据。
  主问题OPEN，所有工厂/正式SLAM未修改，外仓只读，main未提交。

## 51. 10秒RGB+双IR已采完：跨深度候选新图改善，仍不能通用启用

- 用户确认“可以”授权本次10秒。`ego_stereo_color_probe.py --record-now`采集
  `ego_stereo_color_probe_20260906_run6`，20组三路/60PNG，emitter0、1280×720@30，
  存储2Hz。报告duration11.467包含清理；采样跨度约9.8秒。cleanup=[]，相机停止，
  该次授权已执行完，后续硬件动作需新授权。setup SHA bb8703a9512c53b777ba9054b2fe0a5a44cad93aa09b1be37e0594988c5ac49f，
  frames SHA25ca7365a2b1b1e9fcafc9511aee584188529c4e9003e6c89d6e1040632a87dd。
- `ego_joint_stereo_diagnostic.py` native检测全部20组三流36板码+独立背码ID1，
  唯一板外候选/原始alias约束，板内ID1按既有几何门槛恢复。RGB最大漂移
  板.571/背码.280px，IR-L.132/.105，IR-R.140/.181，全低于.75px。
  IR背码短边约54.6/54.3px低于60：保留诊断但尺寸FAIL不放宽。
  初版该门槛阻塞整个板分析，且缺测Inf不能JSON化；修为诊断保留小背码并
  明示尺寸FAIL、缺测null、static与geometry分离。v3报告FAIL且static_pass=True。
- IR三角化不使用打印尺寸：板深度中位.768m、背码.390m。factoryRGB p95
  跨帧中位板2.568/背码2.608px，IR内部板.080/背码.394px。首5帧只用板拟合
  IR→RGB T，后15最差factory板.686但背码27.220px；旧K4k1+新T板.428/
  背码1.933px。说明平面拟合会补偿模型失配，不能据板误差小启用外参。
- `ego_multidepth_camera_diagnostic.py`用旧run1/2/4/5共3548个板IR三维点拟合
  全局单一K4+k1+T，保留缺测/历史static失败/角点策略差异。run3/6和所有mount
  不进拟合。候选OpenCVBrown: fx896.950122/fy897.488484/cx661.579296/
  cy378.971288/k1.0472527549。新run6板最差2.597→.518px、背码2.708→.918px，
  但oldrun3仍1.277px。候选只存在JSON，没有写factory/正式SLAM。
- 独立审查指出跨会话IR几何身份guard不足，已补schema/serial/未激活未改厂参、
  3路全部内参、2个外参signature精确一致、旧derived.setup==真实setup、完整
  帧序列/源metadata核对，负向测试补齐。实际6会话本来一致，v3与v2数值精确同。
- 更新旧外码验证器接收显式multidepth候选，只替换EgoRGB模型，不应用诊断T。
  两历史场景各598对/pose拒绝0：旧21:59为4.845→7.891mm退化，新22:51
  为5.631→2.799mm改善。因此通用启用仍拒绝，不能将新图像素过门槛称全链路修复。
- 证据run6/RESULT.md；joint_stereo_diagnostic_v3/report.json，
  multidepth_camera_diagnostic_v3/report.json，外部回归绑定v2保留在
  multidepth_camera_diagnostic_v2/external_regression/report.json。117相关测试通过；
  只读审查独立复现原图、三角化、拟合与评分，Important已修，无新增重要问题。
- 一项未交付的内存探索：新IR与旧UMI第一帧静态桥接；Ego板跨会话最大漂移
  .539px，IR纯PnP上下最大约4.94mm。IR三维板点→UMI像素、背码中心三角化
  首对约1.98mm上下差，但全板投影p95约1.22px。尚未全帧组合/跨会话背码
  锚点验收，严禁作为正式标定结果。后续优先验证**同帧Ego双IR三维参照→UMI**
  直接路线，避免继续用RGB模型承担空间基准；新的同步采集要先获授权。
  当前相机全停，main不提交，外仓/发布只读。原5.631mm问题仍OPEN。

## §52 — 新授权10秒双设备同时采样，直接IR3D→UMI链路（2026-09-06）

- 最新用户“可以”授权一次10秒，无R。`ego_ir_umi_capture.py`新建，双IR和UMI
  color独立SDK callback，1280×720@30；Ego327122078613，UMI260322279785。
  `artifacts/spatial_bench/ego_ir_umi_20260906_run5`完成20triples60PNG/10.000457s。
  主机到达差2.44–30.76ms只做静态代理，不是曝光同步。cleanup=[]，两机已停。
- run1–4均0samples，失败未删除；定位D405 gain GET在紧接SET之后PIPE，开流前
  同样失败。最终仅gain遥测失败标warning/null保留，不能称增益已回读确认。
  曝光30000/auto0验证，Ego emitter0强制验证。`capture_source.py`精确匹配setup
  codehash67f322785ca06d96f182bbeaba0490c9f2bfd5f45c8528e00216108fe446c9c7。
- setup SHA e2e41580a5a6276a386e57023bab112c134c7a13cac628a4909c6d0c8e137215；
  frames SHA44d557c619acb3a0a595b12fcef89858a4918bc0992f0b6b8be7abd8b7b39bad。
- 新`ego_ir_umi_diagnostic.py`：双IR实际3D角点→固定UMI factory像素拟合
  T_Ucolor_Eir，背码仅三角化后变换中心，不参与拟合。7固定组，无KD拟合，
  nominal板仅身份/组内初值，不进实际尺度目标。v1初值曾看整板，审查发现
  holdout泄漏，v2改组自身初值+污染留出像素负测。140组独立复现。
- `direct_diagnostic_v2/report.json`20帧全保留；36tag全部三路/60原图验证。
  上下中心差median2.294945/max4.038448mm；左右1.780147/2.607060；交错
  .524529/1.230181。全板最差p951.229800px；空间holdout最差2.144279px。
  背码最短边IR54.24/53.67px低于60，明确FAIL。静态maxIR-L.291/IR-R.131/
  UMI.372px全pass。20/20帧FAIL，不能激活。
- 全板背码中心UMI color平均[12.6101,-6.8135,-50.2996]mm，norm范围52.017–
  52.648mm，最大两两变化1.070896mm。只代表本次参考模型结果/重复性，不是
  卡尺壳体距离或绝对真值；新旧布局未锁定，不将此xyz与旧23mm x硬比。
- `ego_ir_umi_bundle_diagnostic.py`事后控制：前10均值训练、后10同场景留出，
  优化单T+选中板点3D对三路raw normalizedrays，固定KD，无背码/holdout泄漏。
  7组收敛。上下2.320→1.657mm、左右1.757→.561改善，但全板heldout1.167→
  1.540px、空间holdout1.724→2.025px变坏，拒绝启用，负向报告joint_rays_v1。
- 125相关测试pass9.97s，包含双设备启动清理/emitter不可豁免；直接诊断只读
  审查无剩余Important。完整报告run5/RESULT.md。原run4solve hash未变。
- 下一阶段：需要固定相机、变公共板姿态的独立视角对照和更大IR背码成像，
  分离残余IR参照几何/UMI模型/角点偏差。不要继续同画面调期望距离或称修好。
  本轮授权已用完，不得自动追加录像；所有候选NOT_ACTIVATED，main勿提交。

## §53 — 用户“改变了”：新板姿态固定旧T验收（2026-09-06）

- 自动10秒EgoIR+UMI run6完成20triples60PNG/10.000431s/cleanup[]，两机已停。
  setup SHA8579ac699240ea17f35c545a2ff78490c3803234f3de2d68be884e1a2a978712；
  frames SHA856f0596a054cdb39744e8c0223214c9aa393c79d494b8d392a48617e4911e18。
- 处理新图前封存run5/joint_rays_v1的all36/direct_mean T，frozen_reference.json。
  `ego_ir_umi_frozen_check.py`双会话原图hash/设备内外参一致性验证，再重解码，
  新IR三角化点经旧T投影到UMI；新图不拟合X/KD，板PnP只作新颖性检查。
- 实际UMI板的上/右侧出画，24–25/36码。完整all36诊断0fits/20FAIL保留。
  冻结检查对真实共视24–25码评分，缺失持续FAIL、无残差删点或补点。
- 板原点tag0中心位移约91.4mm/法向11.1deg，确实新姿态；但背码在EgoIR
  旧→新角点最大偏移L1.698–1.738px/R2.479–2.536px，超.75固定门槛。
  窗口内板漂移L.164/R.180/UMI.623px均过，不能与跨会话固定混淆。
- 固定旧T预测新图p95median11.979734/max12.302493px，20FAIL。不能单独归因
  算法/内参，也不能仅凭像素漂移确定哪台相机物理动了。需问用户除板外是否
  也碰动UMI/Ego；不要在不清楚固定关系时混合拟合或自动追加录像。
- 主证据run6/frozen_check_v2/report.json及RESULT.md；v1保留，v2仅准确命名
  origin_translation_mm并增加冻结/执行门槛一致guard，评分不变。只读审查
  60原图/20冻结评分/mount漂移独立精确复现；两个Minor已修。正式参数未变。

## §54 — 用户确认两机未碰动：不能用像素漂移断言物理移动

- 用户回答“没有”，明确两相机未挪/转/碰；按此记录，不继续归因操作。
  全部新工作离线，零硬件访问、零额外采样、零标定启用。
- 新`ego_ir_umi_image_change.py`重验run5+6共120原图hash，同设备/内外参，
  做run5首帧→run6全部20帧的自然纹理双向LK与模板NCC，另做旧0→10对照。
  UMI瓶身38–49LK点位移median[11.487,-2.327]px；模板[11.35,-2.13]、
  NCC.904–.913；旧窗口模板[.010,-.015]px。IR背码内部整块图案L[.994,1.193]
  /R[.856,1.196]px，NCC>.996。现象不只是AprilTag角点抖动，原图也有改变。
- UMI夹爪左仅3–10点/右0，不能据此证明夹爪稳定或排除全图读出变化。
  ROI事后选择、瓶子也可能动，不能由这些证据唯一确定物理相机或操作者动作。
- 新射线与旧T的极线p95=9.823–10.018个去畸变像素，固定射线/T下不能靠
  改IR每点深度消除。误差涉及观测射线/模型/相对投影关系，具体原因未确定。
- 修正frozencheck错误标签为mount_projection_changed_since_reference；
  新frozen_check_v3_projection_label数值与v2精确一致（最大12.302493px），
  不放宽门槛。旧报告不覆盖，用户确认与新解释见run6/OPERATOR_CONFIRMATION.md。
- 136相关测试通过9.92s；只读审查20帧LK/NCC/极线/旧对照精确复现，无重要问题。
- 下一建议：保持一次连续开流采板姿态A/B，排除两次stop/start与板姿态混杂。
  重启只是待排除变量，不是已证明根因；本轮未获新连续采样授权，相机全停。

## §55 — 连续开流 A/B 已实现，本轮等待完整板超时（2026-09-06）

- 用户“可以”授权一次连续开流对照。新增 `--continuous-ab`，A/B 各10秒，
  两设备各启动一次；三路全36码稳定1.5秒自动采A，提示仅移动公共板，再采B。
  不按R，最长300秒从首次启动请求计；原10秒单次模式保留。Ego投射器关闭。
- `ego_ir_umi_continuous_20260906_run1` 实际退出码2，capture_timeout，WAIT_A，
  A=0/B=0，frames.jsonl为空，cleanup_errors=[]。终端最后Ego IR各36、UMI32。
  `preview_screen.png`为桌面预览窗口截图，UMI板顶部/右侧裁切；不是保存的原始
  相机帧或标定输入。无A/B数据，不能运行求解或归因重新开流。进程已退出。
- 新控制器 `ego_ir_umi_continuous.py`；新离线 `ego_ir_umi_continuous_check.py`
  仅用A拟合固定出厂模型变换，B只评分，保留小背码及缺码失败；逐阶段板和背码
  四角静态门槛0.75px，拒绝用均值掩盖A阶段背码变化。投影变化不等于物理移动。
- 下一步需要用户准备好调整板入UMI完整视野，再开新目录的有界连续采样。
  本次未自动延时或重开，未证明误差根因，全部 NOT_ACTIVATED，main勿提交。
- 当前相关回归141项通过（10.04秒），背码静态性修复经独立只读复查通过；
  git diff --check无问题，原run4/solve报告哈希不变。软件测试不是实机标定通过。

## §56 — 连续run2完成，离线分解误差（2026-09-06）

- A20/B20共120PNG，连续开流，DONE，cleanup[]，相机已停止。
  frames SHA7a742710826041bbe03152a89743737acb826d1f39e6f9a7eb6e8e75a2ece17d。
  frozen_check_v1 FAIL：A p95median1.262844px，B2.916412/max3.155293px。
- 用户要求“分析”：仅离线，新analysis_v1报告与脚本。复验全部原始哈希、
  40帧评分。B31–35静态超门槛；其他15帧仍2.913818px，不可全部归因该5帧。
  A/B极线p95median.344867/2.456160px，固定rays/T时IRdepth-only不能消除。
- 各阶段前10帧诊断拟合T相差5.106mm/.362554deg，A/B交叉预测均恶化；
  共享T折中仍~1.7px。Brefit/共享fit不是独立验收，更不能激活。
  背码中心差[1.985,-1.635,-.541]mm，norm2.63mm；距离长度52.231/53.484mm。
  不把T平移差5.106当背码长度差或卡尺距离。固定原T下背码中心变化~.47mm。
- 原纹理mountNCC垂直~.53px相关>.991；UMI夹爪纹理近零。IR背景NCC方向
  不一致/LK不足10点，不能推相机运动或责怪用户。IR亮度有变化但未锁定/逐帧
  记录IR曝光增益，不能据此宣称AE根因。setup只确认Ego emitterOFF。
- 结论：跨姿态相对投影关系不自洽，具体KD/成像/微小几何原因未唯一定位。
  下一建议锁IR曝光后A-B-A返回对照区分区域/时间效应，本轮未开新采集。
  22测试通过；独立复核原图/评分/T差一致，无Important。main勿提交、原参不动。

## §57 — 已接入锁曝光 A-B-C 返回流程，run2 等待用户移板

- `--continuous-aba`新增C=返回A，紫色原始板轮廓；原AB/10秒单次模式保留。
  A/B/C各10秒，总300秒，板全36静态.75px/1.5秒；返回各路median<=5px、
  max<=10px，离线返回位姿<=10mm/2deg。只用A拟合，B/C均不重拟合验收。
- 新ego_ir_lock：AE关闭后锁定曝光/gain，选项读回、每帧metadata证据，退出
  各项best-effort恢复原值/AE；任何失败保留cleanup错误，投射器始终OFF。
- run1锁idle8500us/gain16验证成功但画面过暗IR0/0，助手中止0帧。恢复gain
  即时GET报PIPE，AE仍恢复。不可称clean通过。截图及RESULT保留。
- 修启动：先AE预热至少6秒，实际双IR帧曝光/gain相同且稳定1.5秒严格2%，
  三路36且fresh，才冻结LIVE值。30秒预热deadline无条件执行；锁定写完再
  排空2秒，A/B/C内不再调参。GET RuntimeError最多0.5秒重试，仍失败不豁免。
- 当前 `ego_ir_umi_locked_aba_20260906_run2`，工具会话6108运行中。
  预热6.588秒，实际锁31979us/gain23/AE0；A已20组三路/60PNG，WAIT_B。
  已提示用户仅平移板3–5厘米，然后B完按紫色轮廓返回A。先poll6108勿重启。
- 149相关回归通过；之后严格2%/gain精确匹配改动focused15项通过。独立
  实机前审查无重要问题。未完成ABA/离线验收，不要提前宣称问题修好。

## §58 — locked ABA run2 已完成，返回A误差恢复（2026-09-06）

- 工具会话6108退出0；A/B/C各20，共180PNG。原始hash与所有帧曝光31979us/
  gain23/AE0核验，最终读回匹配、原设置恢复，cleanup[]，相机已停止。
- frozen_check_v1仍FAIL：A逐帧p95均值1.439385px，B中位3.030370/max3.418402，
  C返回A中位1.467168/max1.693761px。均值/中位不要混写。
- 三段静态全过，最大板漂移A.461/B.515/C.576px；跨阶段背码投影最大
  B.290/C.237px过.75。C返回门槛过，末帧原点差~3.5mm、法向~.7deg，非完美同位。
- 仅失败项是固定预测>1px、两IR背码边长<60px。不能再把本轮失败归于那5帧
  抖动/AE变化/重新开流。B差而C恢复更支持视角/区域相关偏差，但具体KD/
  角点/成像根因仍未唯一确定，不能直接换出厂参数。下一步仅用这批三姿态
  离线分解，不自动追加采集。正式参数NOT_ACTIVATED，main勿提交。

## §59 — 刚性板几何离线对照：当前批改善，但未通用修复（2026-09-06）

- 本轮仅离线；相机保持停止。新脚本/JSON/RESULT位于
  `ego_ir_umi_locked_aba_20260906_run2/model_diagnostic_v1`，原报告不覆盖。
  验证当前180PNG及旧AB120PNG哈希、索引、相机模型和冻结报告绑定。
- `analyze.py/report.json`首先做A/B前10训练的敏感性对照：不能把B当留出。
  焦距/UMI畸变单变量不解决；IR焦距触及边界，不得替换出厂标定。
- 更公平`fair_a_only`全部只用A前10拟合共享T；B/C每帧刚性板姿态仅由IR
  求出，不看UMI。逐帧p95中位数：direct A1.432/B3.042/C1.467px；固定
  声明板尺寸rigid A.353/B1.010/C.348px，B最大1.089仍超1px，IR最差.364px。
  A-only联合12DOF求解几乎相同（B1.010458/max1.089411），未进一步解决。
- rigid增加名义尺寸/刚性先验，不是同假设换求解器。A-only IR估计尺度
  1.010739的rigid对照B2.076px，说明名义尺度先验也影响结果。沿用§45
  已有35.2mm/.3板规格，不再要求用户重复提供规格，不靠卡尺期望调参。
  IR逐点轴标度约大1.1%、affineRMS.49–.80mm，不唯一证明哪台相机/板有错。
  mountz从-50.65到-46.95不是绝对真值验证，背码小于60px失败仍保留。
- `prior_ab_replication`在旧连续AB会话各自仅用其A前10训练，B direct2.930
  ->rigid2.806px，强改善没有复现。全部40帧含静态失败保留，不能称通用修复
  或把两批差异全部归于锁曝光；不是跨会话冻结T的独立验收。
- `view_pose_control`单IR左/右已知板预测locked B1.755/2.191px，均未过。
  诊断独立PnP推得IR-R/L旋转A→B差.0305deg，而UMI/IR左右分别.1605/.1365。
  不能唯一认定UMI内参错误或排除IR共同成像/角点偏差，更不是相机物理移动证明。
- 当前A→B板原点移约203mm，但法向只变1.37–1.47deg；C近返回。下一有用
  实机证据是固定两机、改变板倾角的训练/独立留出，不要再重复平行平移。
  本轮未开新采样。原公共板求解本已用几何，不能说旧算法全都没用布局。
- 35项测试通过，含真值恢复/尺度负对照/UMI污染不影响IR板姿态/A训练预测B。
  主对照与联合求解经只读独立精确复现，无Critical/Important。仍NOT_FIXED /
  NOT_ACTIVATED；main不提交，外仓只读，run4原报告SHA191f51f5…3916f不变。

## §60 — 倾角对照已接入，run2预览等待摆位（2026-09-06）

- 用户“好了”后新增`--continuous-aba --tilt-contrast`，同流A/B/C各10秒，
  全36/静态.75px/曝光锁定/退出恢复/300秒总限保留；B须各视图板法向相对A
  >=12deg，纯平移不触发。C仍回紫色A轮廓；原AB/ABA模式默认行为保留。
- 离线checker保留原精度/背码/静态门槛，额外逐帧检查B倾角；不得把只采完
  或小背码保留等同验收。预定只用A拟合，B新倾角评分，C返回，不用B/C调KD。
- run1工具11511退出2/0样本/cleanup[]；截图证明UMI上排裁切，IR36/36而
  UMI30。30秒预热期限误包含人工摆位。修复仅tilt模式：先摆位，总300秒；
  首次all36fresh后启动30秒收敛deadline且永不重置，失帧也检查。原模式不变。
- 37项相关测试通过；合成三视图恢复15deg误差<.00003deg；独立只读复核
  无重要问题。run1证据/截图保留，未改出厂、未激活、外仓只读、main勿提交。
- 当前ACTIVE `ego_ir_umi_tilt_aba_20260906_run2`，工具会话24822。两机已开，
  最后WAIT_A/摆位预览：IR36/36，UMI30，尚未曝光锁定/采样。已提示用户只把
  板在UMI画面往下移露出上排，相机不动。不按R，A完再转板15–20deg，B完回A。
  下轮先poll24822确认是否仍运行/超时，不要盲目重启。总限300秒。

## §61 — 倾角run3已停：A/B各20，C缺失；刚性约束未改善新倾角（2026-09-06）

- run2工具24822已超时0样本、cleanup[]。后续用户好了授权run3；工具35156
  现也已退出2，A20/B20/C0，WAIT_C超时，未完成ABA。不要再poll这些旧会话。
  本轮没有开新采集，不得重启补C并冒称连续同流。两机保持停止。
- run3退出记录gain恢复GET PIPE错误；最终IR锁定读回AE0/31979us/gain26
  匹配采集metadata。之后只读query读到原AE1/8500us/gain16/emitter0，现值
  已恢复，但原cleanup失败保留，不可改写原报告称clean结束。
- 新run3/partial_audit.py仅为该不完整源做显式诊断，原source/phase_rows
  验收入口没放宽、仍拒绝。120PNG及索引/metadata校验，40组三路36码，
  A/B存储跨度9.529/9.496秒；板漂移最大.517/.523px，背码漂移.103/.121px，
  跨阶段背码变化最大.108px。背码边长54.63–54.88px仍失败60px门槛。
- 新B实际法向变化23.724–24.279deg，三路均满足12deg，不再是仅平移对照。
  只用A前10求T，B不拟合T/KD/尺度，逐帧UMI p95中位：direct A1.577/B1.957；
  rigid声明板尺寸 A.516/B2.454（更坏）；rigid_A_IRscale B2.847px。全部B
  未过1px。刚性板约束不是完整修复；不唯一证明哪台内参/角点/模型出错。
- C缺失无法检验返回重复性。本次A/B仍可继续离线诊断，无须因缺C丢弃或
  默认重采；不可称完整标定或安装外参通过。用户当前不用再摆板。
- 独立只读重解120PNG/40组检测/120评分精确一致；43测试通过。主树main
  未提交，出厂参数和外仓未改，NOT_ACTIVATED。证据run3/RESULT.md、
  partial_audit_v1.json、verification_junit.xml；run4原solve SHA不变。

## §62 — 离线排查：角点/深度对照无通用修复，单姿态解跨姿态偏移（2026-09-06）

- 用户本轮“排查”：只离线，无硬件/出厂/运行时修改、无激活、main勿提交。
  新证据目录 `ego_ir_umi_tilt_aba_20260906_run3/isolation_v1`，相机仍停止。
- `corner_controls`精确重放run3+lockedrun2全部300PNG/100组三路36码。
  native/scaled/edge分别只改UMI、只改双IR、一起改，direct/rigid两路径，
  共40case/2000评分；各自仅A前10训练T。倾角B依然约1.92px以上，单纯换
  这些角点精修不能完整解决。旧姿态有median略<1但max>1，不得择优称PASS。
- `plane_controls`每阶段前10均值，归一化单应H与刚性板对照：18码拟合/
  18码棋盘留出，IR p95 .067–.095px，UMI .172–.253px。H列长比IR约
  .9975–.9986、UMI .9965–.9975。额外平面自由度吸收偏差，不能唯一归因KD；
  统一板尺寸缩放不影响该指标，板各向异性/相机纵横焦距错误均可产生此类差。
- `depth_controls` IR-only双H平滑/平面视差原左射线/平滑左射线，不强加板
  物理尺寸，仍只A拟合T。倾角B 1.696/2.440/2.473px；旧平移B 2.601/
  3.429/3.408px。深度平滑也不是通用修复；不排除系统视差偏差。
- `pose_support_v2`重要区别：这是**A/B参与训练的解释性对照**，不是B独立
  留出。各阶段前10均值，固定factoryKD/baseline、IR-only刚性板3D：
  倾角 A-only A.516/B2.454px；B-only A1.439/B.426px，两个相机间T相差
  4.449953mm/.342741deg（不是背码距离差，不证明相机移动）。AB联合A.726/
  B.519，最大逐帧p95 .776/.593px。旧lockedABA AB联合中位.503/.506/.521。
  其余A/B帧只重复已拟合姿态，旧C近返回，均非独立新角度验收。不能称已修复。
- 单姿态小训练残差不足保证本数据跨姿态准确；不等于单平面数学上不可解。
  理想合成A-only仍恢复真值并预测B。局部固定3D Jacobian条件数（1deg/1mm
  参数尺度，除sqrtN）89.69->57.52，只做局部说明，不是完整协方差/根因证明。
  候选方向是多倾角全板联合共享T，再冻结后独立姿态验证，不凭拟合换出厂KD。
- 独立审查发现v1 B-only初值继承A，修正为B自身PnP初始化；v1报告保留但
  superseded，最终引用v2。新增缺失/NaN A投毒测试，像素评分不变。31相关
  测试通过（新文件20+原partial6+geometry5），见verification_junit_v2.xml。
  独立重算v2两会话完整结果精确一致，无剩余Critical/Important。
- 原run3仍INCOMPLETE/A20B20C0/cleanup错误/小背码失败，旧loader仍拒绝。
  run4原solve、run3原capture/partial报告SHA均不变，完整值在本轮RESULT.md。
  不要重启旧采集会话，不自动补C，当前没有正在运行的硬件或求解进程。

## §63 — 纠正重复多姿态建议；同数据方法对照与比例补偿反例（2026-09-06）

- 用户指出“昨天已经做过多姿态联合且没用”。核对run4确实8train/4holdout+
  返回复核，助手已撤回以“缺多姿态”为根因的重采建议。用户继续仅做离线。
  新证据run3/matched_methods_v1/RESULT.md，不打开相机、不改出厂/正式代码。
- compare.py将旧joint数学用于新IR左/UMI同角点，比较两阶段IR-only刚性3D
  与joint三视图。所有候选相同单目正反向/立体评分；双视图少用IR右，不能
  将其差单归优化器。相同信息集的两阶段/joint3view，倾角A-only B立体评分
  2.453595/2.452627px，仍失败。同AB T的A立体评分.726、单目评分1.911px，
  此前低分混有评分来源/训练验证变化，不能拿来横比旧RGB留出或宣称修复。
- 原run4全部288PNG和audit重放，8train densejoint复现旧报告留出p95最大差
  .000287px、T差.000655mm。最差4.390725px仍失败；改两阶段单目为5.596307。
  old1/2/8固定前N原训练slot对照全保留，summary train为原训练池，实际拟合
  看used_for_training/training_slots。原REVIEW/novelty失败不能删。
- source_geometry：IR-only mono/stereo boardpoints倾角A中位差.508078mm，
  固定AB T投影两组预测差p95=1.523336px。是预测间差而非真值误差，不能
  唯一归因物理内参/板/角点，也不证明基线移动。两源姿态未用UMI，AB T用了。
- 额外单变量：倾角A前10双IR H列normratio平均.9976495278，未用B/UMI/旧RGB
  拟合因子；只在诊断副本板X乘该数，其余不变。倾角A-only B立体p95中位
  2.454->.797/max.844。但同因子旧locked各自A-only B1.010->3.089/max3.223，
  旧RGB8train原holdout中位2.765->3.269/worst4.391->3.746仍失败。
  是等效横纵比例敏感性，不是实体板尺寸测量；反例否定通用修复，勿挑新批
  .797报PASS。未修改板配置/任何KD，未激活。已有数据已看过，不称盲测。
- 最终aspect_control_v3.json：v1无旧locked复制，v2原RGB求解checks不全；
  v3补绑定旧reportSHA/REVIEW/board_checks（noveltyfalse）。旧版本保留但
  superseded。45相关tests通过，独立只读数值复算一致，审查保留证据要求已修。
- 原run4solve/run3capture/partialSHA不变，相机停止，没有任何活跃采集/
  求解会话。物理根因仍未唯一分离；本轮否定的补偿不进入coordinator或SLAM。

## §64 — 真正上游 multical/OpenCV 同数据对照已跑通，仍复现失败

- 用户问开源实现并批准继续。本次不采集，仅原run4全288PNG/hash/static
  重放。上游multical固定efe9d7d0af85d51d32669e4b77469494972ac25a，
  位于artifacts/spatial_bench/upstream_multical_20260906_run1/vendor/multical。
  tracked源码未改；实际调用Calibration.bundle_adjust/Camera/StaticFrames，
  不是仿写；依赖隔离deps，未装到/opt或外仓。
- 固定原8train/5双向holdout，1288相机-角点全部保留，关闭K/D/板形优化、
  outlier剔除。SDK射线转同K虚拟pinhole像素，与本地归一化射线目标一致；
  评分仍原SDK像素/source-only板姿态。不是上游完整CLI/检测前端复现。
- 本地最差4.391011907px；实际multical4.390738283px。另实际上游
  camera.stereo_calibrate/OpenCV无外参初值4.390746727px，再替换全部
  camera/time初值运行multical为4.390488482px。1px门槛均失败，不再把
  “换一个联合求解器”当未经做过的修复。也未证明所有算法/厂家参数正确。
- 最终36相关tests通过4.14s，verification_junit_v2.xml。v1只读审查完整原图/拟合/评分精确复现无重要项；
  tiny角差cv2.Rodrigues置0的Minor由verify_results.py用scipyRotation明确
  (.000120100deg)，旧报告保留。扩展独立初值审查也完成，全部原图/变换/
  分数精确复现；旧初值与holdout投毒不改结果，已补回归，无剩余重要项。
- 主报告result_v1、独立初值independent_result_v1、RESULT.md及复核脚本在
  该目录。保留原REVIEW/novelty失败，全部NOT_ACTIVATED，原run4solveSHA未变。
  无相机/后台求解、无工厂/正式SLAM/coordinator/外仓修改、main未提交。

## 65. iPhUMI式共享世界路线：左手真实输入适配（2026-09-06）

- 用户同意保留D435i＋D405，先Ego＋左UMI，借鉴独立SLAM＋共享地图，不换手机。
  不再要求重复16步板采集。共享地图后端候选COVINS-G，已读实际源码，固定
  c5b180b443b59d2267a14584fa4b090429038698；ROS1图像＋里程计wrapper，
  不要求前端导出3D地图点。尚未安装/运行后端，不能称对齐完成。
- 新增three_device_slam/spatial/covins_export.py、scripts/verify_covins_input.py、
  tests/test_covins_export.py。实际左UMI源/tmp/umi-v4-065204-loop-probe-slam-run
  和/tmp/umi-v4-065204-slam-export只读；1284原始body位姿与图像成对导出，
  不读取corrected轨迹，body_T_cam0按原配置保留，逐流原始时间戳不重置/补td。
- 最终证据artifacts/spatial_bench/covins_left_input_20260906_run2/RESULT.md：
  1284像素及1284位姿全部回读一致，7源哈希一致，时间差最大232ns数值舍入。
  39tests通过2.67s，审查修正默认pytest临时目录掩盖错误测试的问题，已无重要项。
  状态INPUT_EXPORT_PASS/READBACK_PASS；共享世界严格NOT_RUN，未求两设备变换。
- **重要库存纠正**：旧D435i临时SLAM确实存在，数据在
  /home/robot/umi_ego_vio_data_device2_c48df736/temporary_d435i，012230/092539
  会话有1440rawposes/PASS。但device_binding和active_runtime_calibration/manifest
  明确internal_imu_policy=DISABLED_NOT_COLLECTED_NOT_USED，用UMI那只外置STM32。
  不能把这套外置IMU外参当成新Ego内置IMU标定，不能用同一只UMI IMU驱动独立移动的Ego。
- 下一步具体缺口：D435i自身图像/内置IMU的独立VIO输入与验收；隔离COVINS-G
  实际wrapper/backend回放；再做两设备公共自然场景匹配/独立验证、map epoch/reset
  及Ego世界换基，然后右UMI。先复用录包，不默认开硬件。完整接口及门槛在
  docs/architecture/ego-umi-covins-integration.md。
- 旧背码/板残差未解决，未激活标定；该路线只能解除相机轨迹对背码安装外参的依赖，
  不能解除TCP标定/工厂模型误差。左输入仍有损候选，不是最终训练精度验收。
  本轮无硬件/ROS/后端进程，无外仓/发布目录修改，main无提交。

## 66. Ego自身IMU的真实离线VINS已跑通，精度/共享地图仍未验收

- 后续用户“继续”：新增d435i_ego/vins_export.py、ego_vins_offline_probe.py，
  复用Sep3pairrun2的Ego原始30秒，900双目/6013gyro/6059accel。accel按gyro
  原时间戳夹逼插值，无外推；SDK原始gyro轴，不用UMI STM32旋转。出厂
  body_T_cam0逆变换平移[-5.52,5.10,11.74]mm。内置IMU，非旧外置STM32。
- 仅出厂几何＋上游参考噪声＋td0，未完成动态/时间外参标定，无ZUPT。
  独立全量回读1800原图像素/6013gyro完全一致，accel舍入1.18e-6m/s²。
- 原run1实际启动失败是dlopen本地typesupport库搜索路径，已修独立进程
  LD_LIBRARY_PATH，保留失败。run2/run3真实ROS离线回放各900双目/6013IMU，
  backend900queue0，889rawposes约30.014Hz。工程冒烟PROVISIONAL_REPLAY_PASS，
  不等于精度PASS。运行均domain92/localhost、只读既有VINS编译产物，进程已清理。
- 静态角度run2 2.400557°；同输入同数值配置run3 1.189903°。纯双目对照
  1.586943°。存在未解决的重放变化，不可把run3较小值当修复，也不可单凭
  一次对照断言IMU惩罚。未唯一确定静态退化/算法运行/几何/惯性标定根因。
- 最终artifacts/spatial_bench/ego_internal_vins_probe_20260906_run3/RESULT.md，
  含真实报告、rawCSV、log、启动前runner快照与主solver/typesupport SHA/
  实际LD路径；旧run2/control未补写成事前记录。56tests通过2.65s，审查无
  阻塞，后续可补异常pose验收测试。校准未激活，正式coordinator本轮未改。
- 共享地图COVINS-G仍NOT_RUN；下一步真实wrapper/backend与共视场景验证，
  不能把无关联左轨迹＋静态Ego轨迹当已统一。全新工作只在本仓main，未提交，
  无新硬件采集/外仓修改，不默认再次要求16步公共板采集。

## 67. Ego重复性分叉定位、真实场景门禁、COVINS依赖核查

- 最新用户“继续”，仍复用录包，未开相机、未改工厂标定/外仓/正式SLAM。
  证据：artifacts/spatial_bench/ego_vins_diagnostic_20260906_run1/RESULT.md。
  源码只读复制2.1MB到该目录source，build是隔离诊断版；仅增加日志，
  未改变数值算法。不得覆盖此build，否则旧报告的二进制证据失效。
- 同900双目/6013内置IMU，旧两轨迹前635位姿逐位相同，21.1568秒开始
  分叉，最大轨迹间角差2.812353°。诊断replay_b/c一致1.189903°；
  replay_d真实复现2.400557°。900帧特征输入hash仍完全一样。
- 首个差异是MARG索引611（ts800151.411440513，m122/n82，均LDLT），
  此时求解cost仍相同；下一SOLVE索引636初始cost才分叉。两侧相同8次
  迭代限制，非该处墙钟超时。定位到边缘化先验计算边界，不等于已查明
  唯一具体算子：尚未记录pre-Schur A,b/因子与参数排序，勿宣称已修复。
- OMP_NUM_THREADS=1两次仍1.189903/2.400557°：该假设失败，它不限制
  显式num_threads(4)组装。再加OMP_THREAD_LIMIT=1，serial_assembly_a/b
  的900INPUT/890SOLVE/867MARG和889CSV位姿全同，但静态角变化2.047757°。
  两次一致不证明通用确定性/真实精度，没有把线程设置部署到正式链路。
- 成像独立核查：近处模糊少纹理桌面占大部分。LK仅9有效立体点，PNP
  未过30点门槛，不能报独立姿态结果。31对SIFT唯一候选匹配11/14/16
  (min/median/max)，支持率0。400条跟踪特征不等于400有效3D约束。
- 新d435i_ego/scene_quality.py绑定原始源hash/工厂整流模型，固定30点、
  4格、25%竖向跨度、至少10样本和80%支持。默认离线probe在ROS启动前
  BLOCKED_SCENE_SUPPORT。真实guarded_replay_final退出2，solver_started=false。
  显式--allow-unsupported-scene只允许DIAGNOSTIC_REPLAY_COMPLETE，不升格PASS。
  这只是工程输入门禁，不是已证明模糊解释全部数值问题，更非标定验收。
- 75项测试通过3.22s；JUnit artifacts/test_tmp/ego_repeatability_20260906_run5.xml。
  独立review无Critical/Important，两个Minor(准确solver_started/边界测试)修正。
  verify_diagnostics.py可只读复核轨迹/日志/源码差异/失败门禁。初次诊断
  缺introspection插件、一次测试体误放NameError均保留失败证据且已修。
- COVINS实际源码已在covins_runtime_20260906_run1/source固定
  c5b180b443b59d2267a14584fa4b090429038698。临时只读无网络Noetic容器
  查到缺配套vision/math/catkin依赖，上游Eigen3.3.4EXACT/OpenCV3，不混
  hostHumble库，不运行主机广泛安装脚本。backend仍NOT_RUN，见该目录RESULT。
  离线执行上游关键帧判据(非真实wrapper)得左9帧、静态Ego仅1帧；当前
  两段无关联数据不具备共享地图验证条件，不能降阈值虚构运动基线。
- 下一步：新隔离版本记录pre-Schur A,b、因子/参数布局，查首次分叉；
  并构建匹配ROS1依赖，真实wrapper/backend回放后再验共视自然场景运动。
  不默认重做16步公共板。所有本轮进程已退出，无硬件采集、无提交；
  主仓main/既有脏树保留。共享世界NOT_RUN，标定NOT_ACTIVATED。

## 2026-09-07 Claude 侧收尾：Ego RGB 模型选择 + 两条链重算（z 悖论 ≈ 已解）

- 新脚本：`scripts/ego_rgb_model_selection.py`（LOSO CV，IR 双目三角化 3D 点锚定，
  不走板打印标度）、`scripts/mount_z_decision.py`（联合 12 帧板+背码重算）、
  `scripts/mount_z_original_chain.py`（monkeypatch 换 Ego 内参重放旧单外 tag 链）。
- 结论报告：`artifacts/spatial_bench/ego_rgb_model_selection_run2/report.json`、
  `mount_z_decision_run1/report.json`、`mount_z_original_chain_run1/report.json`。
- 模型决策 **k1k2k3**（fx 892.62, cx 664.98, cy 376.34; D=[0.0787, 0.0179, 0, 0, −0.5453]，
  opencv_brown_conrady；fx 比 factory −14px）。factory 零 D 被决定性拒绝
  （held-out board p95 ~1.0px；run6 板位姿外推 mount 26.5px → 2.7px）。
  k1k2 紧随（mount 3.45px，离轴行为可能更稳，端到端值得再比）。
- 联合 12 帧数据集：factory norm 56.4 / z −49.3 → k1k2k3 **norm 46.2 / z −43.4**
  （UMI color 与 ir_left 帧几乎同值），进入物理带 46.4-48.4，与 Codex 候选 −46.54 同带。
- 旧单外 tag 链（013727/024412/024608）：013727 −59.0→−54.4 有改善，
  024412/024608 −61 不动、k1k2k3 反劣化 −70 → 旧链不 repro。主因 = 弱观测
  （ego 看外 tag 49px@0.75m、旋转 rms 17°）+ **UMI color 反向畸变在 mc/tag
  检测路径被 OpenCV 当正向用**（联合板数据集走 librealsense SDK 投影，模型语义正确）。
- 教训：旧验证全帧内自洽，位姿吸收模型误差后残差全绿但世界系错 15mm。
- 待办：UMI inverse→正向畸变转换后重放两条链；k1k2 vs k1k2k3 端到端对比；
  接受与否由用户定，NOT_ACTIVATED，factory 值未动。

## 2026-09-07 第三轮更正与收官（卡尺裁决）

- **更正 1**：上轮"UMI color 反向畸变被 OpenCV 当正向用"**错误**——
  `apriltag_detector._solve_ippe_candidates` 对 inverse_brown_conrady 有专门分支
  （SDK deproject/project 归一化射线），两条链 UMI 畸变语义本就正确，无需转换。
- **更正 2**："打印标度 +1%"系**ego-IR 标度误差**所致，非板的真实属性。用户卡尺
  裁决 0906 板黑边 = 35.2-35.3mm（名义值精确，即产品原板）。 ego-IR 三角化普遍
  虚高 s=1.01014±0.0001（20 帧点阵相似拟合；板边/间距/背码三处一致）——
  **D435i S/N 327122078613 的 factory IR 立体标定自带 +1% 标度误差**
  （fx×baseline 组合；影响一切 ego IR 度量用途）。 产品 bench（D405#737 自标定
  IR 立体）独立复测同一物理板 = 名义值精确（重投影 0.05px，n=512），见
  `scripts/board_print_scale_crosscheck.py`。
- **数学事实**：标定 3D 点缩放不改变 calibrateCamera 目标函数（平移吸收标度）
  → IR 标度误差对 K/D 拟合免疫，k1k2k3（fx892.62, D=[0.0787,0.0179,0,0,−0.5453]）
  就是真度量最优模型；`scripts/mount_z_scaled_pipeline.py` 缩放前后输出逐位相同
  （该 run 报告仅作此性质之证据）。factory fx906.9 是零畸变占位逼出的妥协值。
- **打印标度修正 run 作废**（`mount_z_measured_geometry_run1/`，当时基于 +1% 假设）。
- **最终答案**：名义几何 + k1k2k3 + UMI factory（Intel 自标定，与精确板自洽）：
  **T_umi_ir_left_from_mount_tag ≈ z −43.4mm / |t| 46.2mm**（12 帧中位，spread 0.8mm）。
  对 Codex 壳体物理链 46.4mm 闭合 **0.2mm**；对用户玻璃链 48.4mm（norm 口径）差 2.2mm
  （纯轴向口径差 5mm，属贴面曲面/测量 datum 口径差）。k1k2 备选：z −42.5 / |t| 45.3，
  旧链离轴行为更稳。旧 −61 链死于弱观测（49px@0.75m），不作数。
- **口径教训**：所有"闭合"先对齐比较口径（norm/轴向/平面距）与坐标帧再比数字。
- 状态：NOT_ACTIVATED，factory 值未动；接受与否（k1k2 vs k1k2k3）待用户定。
  若要再收紧：采一版 ego IR + UMI IR + 双 color 同帧板+背码（会话内双侧 IR 锚定），
  彻底绕开打印与 RGB 模型中间层。

## 2026-09-07 深夜追加：双 IR + 双 color 同帧亚 mm 实验（"压到亚 mm"）

按上节"若要再收紧"的建议采了会话 `artifacts/spatial_bench/dual_ir_submm_20260907T223400/`
（`scripts/dual_ir_color_submm_capture.py` + `scripts/submm_detection.py` +
`scripts/dual_ir_submm_analysis.py`；D405 20kµs/g32 手动、ego 调参集 lexicographic 指标、
背码与板同 id1 用板凸包拒识、T_ir_color 逐帧 PnP + 3mm 平移离群拒绝、umeyama 转置修复、
双设备相似拟合链式拼 T_egoIr_umiIr、RGB 模型换 k1k2k3_scaled 精确常量）。

- **跨位姿重复性（2 板位）：z 差 0.22mm —— 亚 mm 目标在重复性口径达成**。
  s_ego=1.0099 复现 +1% IR 标度误差；会内 T_ir_color 对 factory color→IR 快照差 2.9mm；
  工厂外参路径变体 z −51.8mm。
- **绝对值 z −49.8 / norm 54.7mm，比已验收联合链（z −43.4 / norm 46.2）偏 ~6mm、
  在物理带（46.4-48.4 norm）外**。已定位最可疑偏置源：ego.color 85px 背码 IPPE_SQUARE
  的系统性角点偏置（0.42m 处 1px ≈ 5mm；掠射角曾 >45° 完全解不出，用户重摆后才 85px）。
- **结论：最终交付仍以联合链 norm 46.2mm 为准**；本会话是其独立交叉验证 +
  亚 mm 重复性证据。修偏方向：背码换大 / ego 拉近 / 逐角点中值细化 + IPPE 双模式对照。
- 采集侧修复：sha256 回填少 pose 目录、finally 崩掉丢原始错误（均已修，加 --poses）；
  pose1 中道 USB 掉帧崩（15/60 帧仍可用）。
- 报告：`artifacts/spatial_bench/submm_final_2pose/`。状态：NOT_ACTIVATED。
