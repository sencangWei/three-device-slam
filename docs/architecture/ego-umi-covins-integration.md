# Ego + UMI shared-world integration — 2026-09-06

Status: left frontend **INPUT_EXPORT_PASS**, Ego built-in-IMU offline VINS
**PROVISIONAL_REPLAY_PASS**, shared world **NOT_RUN**.
These milestones are input/runtime checks, not accepted spatial accuracy or
mount/TCP calibration. Isolated offline ROS replay has now run; no hardware,
COVINS backend or installer started.
All new files remain in `three-device-slam`, main, uncommitted.

## Architecture and actual upstream

Retain D435i Ego + D405 left/right UMI. Borrow iPhUMI's independent tracking and
shared-map architecture, not its iPhone-only ARKit implementation. Candidate
backend: [COVINS-G](https://github.com/v4rl-ucy/covins/blob/c5b180b443b59d2267a14584fa4b090429038698/docs/run_COVINS-G.md).
Its generic wrapper consumes images and odometry, extracts features and estimates
inter-agent constraints using multiple views. It does not require VIO map-point
export. VINS-Fusion/stereo frontends are supported by upstream examples.

Pinned source: `c5b180b443b59d2267a14584fa4b090429038698` (verified remote master).
Inspected actual `covins_frontend/src/frontend_wrapper.cpp`, `frontend_node.cpp`,
`config/EuRoC.yaml`, `launch/vins_euroc_agent.launch`, and build definition.
Upstream is ROS1/catkin, not a ready ROS2 Humble plugin. Its map-reset limitation
requires lifecycle handling before use. No COVINS runtime has been built/tested.

The intent is Ego/left/right local VIO -> collaborative backend -> all poses in
one map -> one common change of basis defining Ego world. Server-assigned client
IDs must be explicitly bound to logical roles; launch names alone are insufficient.
Reset must invalidate the affected map epoch and its world transform. Never align
the first three local origins independently and call that a common world.

## What exists locally

| Component | Evidence | Scope |
|---|---|---|
| Current D435i Ego acquisition | `artifacts/d435i_ego/hil_720p_200hz_preview_20260828_run1/calibration.json` | Factory stereo/IMU geometry and capture; not a VIO acceptance |
| Left Docker2 VINS | `/tmp/umi-v4-065204-loop-probe-slam-run` | Real raw body poses; 1284 poses, 1295 source images |
| Historical D435i VINS | `/home/robot/umi_ego_vio_data_device2_c48df736/temporary_d435i` | Exists, but uses external STM32, **internal IMU disabled** |
| Current coordinator | `three_device_slam/acquisition/coordinator.py` | Acquisition owner, not collaborative SLAM owner |

Important correction to the initial inventory: D435i SLAM is not globally absent.
The historical device binding and active runtime manifest both explicitly state
`DISABLED_NOT_COLLECTED_NOT_USED` for its internal IMU and bind STM32
`c48df736b505f011adda8d1272aab386`. The old temporary Docker2 tag is present. The
20260828_012230/092539 run reports PASS and 1440 raw poses. This is NOT the new
independent Ego/internal-IMU input, and cannot share an independently moving UMI's
IMU. Do not copy that external-IMU calibration into the new Ego frontend.

## Implemented adapter contract

`three_device_slam/spatial/covins_export.py` supports the audited RK3576 export
plus `test_vins_auto_loop` output layout only. It does not yet accept the historical
D435i external-IMU layout or standalone D435i binary capture.

- Reads `vio_raw.csv` only; never feeds loop-corrected poses back as raw odometry.
- Local VINS publishes `T_local_world_body`. `body_T_cam0` maps camera to body.
  The derived audit camera pose is `T_local_world_body @ body_T_cam0`.
- Wrapper gets `odom_in_imu_frame: 1` and the actual `Tbc`, not identity or inverse.
- Preserves native image dimensions, pixels, rectified intrinsics and per-stream
  timestamps. Pair tolerance is 1000 ns for numeric conversion only; no interpolation,
  independent timestamp zeroing, IMU `td` reapplication or rate fabrication.
- Checks device serial, source manifest/DB3/calibration hashes, source PASS,
  counts, monotonic timestamps, unique image pairing and 0.05 m raw-step gate.
- Map epoch and logical role are explicit. Silent resets cannot be certified from
  CSV alone; this is an input sanity check, not full lifecycle acceptance.
- Output: ROS1 `frontend.bag`, wrapper `frontend.yaml`, camera-pose/pixel audit
  `pairs.jsonl`, provenance/limitations `report.json`. A new artifact directory is
  mandatory. Partial failures have no success report and are retained.
- Covariance/velocity unavailable in CSV: ROS zero placeholders, explicitly unknown.
  Pinned wrapper consumes only pose. Do not reuse these as measured uncertainty.

Reproduce the left input (choose a NEW output directory):

```bash
cd /home/robot/three-device-slam
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 PYTHONPATH=. \
/opt/three-device-slam/venv/bin/python -u -m three_device_slam.spatial.covins_export \
  --run /tmp/umi-v4-065204-loop-probe-slam-run \
  --session /tmp/umi-v4-065204-slam-export \
  --output artifacts/spatial_bench/covins_left_input_replay_new \
  --agent left --map-id rk3576_065204_epoch0 --serial 260322279785
```

Read-only verification: `scripts/verify_covins_input.py <output-directory>` with
the same Python/environment. It decodes every bag image/pose against original
DB3/CSV, checks all source/output hashes, and verifies camera/body composition.

Required upstream remaps, also recorded in report:
`/camera/image_raw -> /left/cam0/image_raw`, `/cam_odom -> /left/odometry_raw`.
Pass the generated config to wrapper `config_file`; do not use EuRoC calibration.

## Remaining acceptance gates (not completed)

### Ego built-in-IMU milestone

`devices/d435i_ego/vins_export.py` exports the existing Sep3 pair recording:
900 stereo pairs and 6013 original gyro samples in 30s; acceleration is bracketed
at gyro timestamps (no extrapolation) and rotated into the factory gyro frame.
Factory `body_T_cam0 = inverse(T_ir_left_gyro)`, no UMI STM32 axis transform.
All 1800 image payloads and 6013 gyro values independently read back exactly.
Noise is only the [official VINS D435i reference](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/config/realsense_d435i/realsense_stereo_imu_config.yaml),
not measured for this device; `td=0` and factory geometry remain unvalidated.

`scripts/ego_vins_offline_probe.py` replays only in ROS domain92/localhost, owns
and cleans its child, uses original timestamps, and records raw body odometry.
Run3 saved a runner snapshot plus executable/primary solver/type-support hashes
and effective loader path before launch. This is not a full OS dependency lock.
The runner requires the existing audited local VINS build, not a portable release.

Evidence: `artifacts/spatial_bench/ego_internal_vins_probe_20260906_run3/RESULT.md`.
900 pairs/6013 IMU reached replay; backend processed900, queue0;889 poses~30.014Hz.
Two IMU-enabled replays of the same input/config gave static angular excursions
2.401° and1.190°; visual-only diagnostic gave1.587°. Thus the latter is not proof
of a quantified IMU penalty, and the newer smaller value is NOT an accuracy fix.
Static excitation, numerical/runtime repeatability, camera geometry and inertial
calibration remain to be separated. No claim of a unique cause.
56 tests pass. Five-centimetre/five-degree gates are only gross runtime smoke
checks, never the spatial/robot-training acceptance. COVINS execution remains pending.

### End-to-end gates

Latest diagnostic evidence (2026-09-06):
`artifacts/spatial_bench/ego_vins_diagnostic_20260906_run1/RESULT.md`.
The actual instrumented VINS reproduces the two trajectories with all900 feature
inputs equal. First differing Schur-prior record611 precedes solve636; identical
iteration stops exclude a timeout at that boundary. OMP_NUM_THREADS=1 alone did
not fix it. Unique numerical cause and accuracy remain unresolved.
Adding OMP_THREAD_LIMIT=1 produced two byte-identical889-pose replays, but still
2.048deg static excursion. This is diagnostic-only, not proof of general
determinism or accuracy and not activated in the formal pipeline.

The static recording also fails the new hash-bound stereo scene preflight:
31 sampled pairs,11–16 unique candidate matches each,0% sufficient support.
The runner now refuses beforeROS by default; explicit diagnostic override cannot
emit a replay PASS. This is not proof all drift is caused by texture alone.
75 related tests pass; no calibrated shared-map acceptance has been issued.

Actual COVINS-G source is now in `artifacts/spatial_bench/covins_runtime_20260906_run1/source`,
pinned to c5b180b443b59d2267a14584fa4b090429038698. The existing Noetic base image
lacks its matched vision/math dependencies; backend build/execution remain pending.
See adjacent RESULT.md. Do not run its broad installer against the host or mix
Humble Eigen3.4/OpenCV4 libraries with its Eigen3.3.4EXACT/OpenCV3 requirements.
Offline evaluation of the upstream keyframe predicate selects only1staticEgoKF;
this is not a real wrapper/backend run and cannot validate multiview map alignment.

1. Ego-specific stereo/internal-IMU frontend, correct frame/time/noise/calibration
   inputs, verified independent trajectory. Input export and actual static runtime
   now work (details below); dynamic accuracy/calibration and replay repeatability
   remain unvalidated. Do not demand another 16-step board sequence by default.
2. Isolated pinned COVINS-G runtime with read-only source mounts and all outputs
   here; test the generated input through the actual wrapper/backend. Check ROS1
   compatibility, feature extraction and keyframe initialization, not just bag IO.
3. Both devices observing overlapping **static natural scene structure**, with
   geometric verification, spatial support, nondegenerate motion, cross-agent loop
   evidence and held-out evaluation. Unrelated per-agent pose CSVs are not enough.
   Purely planar/small/textureless scenes and moving hands remain risks.
4. Map revisions/resets, one Ego-world basis, clock offset/drift, image-pose matching
   and stale/lost states must be explicit before coordinator integration. Matching
   within one device does not establish cross-device time synchronization.
5. Then add right UMI using the same interface, not a separate alignment convention.

Backend map alignment can remove the dependency on an unaccepted back-tag mount
for **camera trajectory alignment**. It does not remove factory-camera bias, drift,
or camera-to-gripper/TCP calibration. Prior board holdout failures remain open.
Current left source is a `lossy_stereo_slam_candidate`, not final training-quality
or absolute-accuracy evidence. Do not claim millimetre accuracy from a closed loop.
