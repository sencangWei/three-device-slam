# Ego-world UMI spatial alignment simulation

## Current: device3 static mount calibration PASS; physical-size + dynamic world HIL next (2026-09-14)

- run22 C′ acquisition/coordinator/index/seal/pair gates pass: 450 raw pairs, 449 strict
  common-time rows, timing p95 16.551ms (<20ms), and static motion PASS.
- Coverage-selected fixed C′ scales Ego4x/right2x yield 150 accepted windows, 33 stable
  IDs and 150 mount samples. Whole-camera bidirectional frozen prediction passes at
  .533px p95; the completely excluded C′ mount prediction passes at .231px p95. C′ is
  novel from A′/B′ by 165--243mm and 21--37deg. Every frozen C′ gate passes.
- Device3 right/ID1 camera mount candidate is packaged with both transform directions and
  active VINS `body_T_cam0` composition. `T_right_ir_left_from_mount_tag` has translation
  [13.118,-7.911,-48.543]mm, norm 50.902mm. Status is
  `PASS_STATIC_MOUNT_CALIBRATION_DYNAMIC_WORLD_HIL_PENDING`; no runtime activation.
- World reconciliation confirms device3 matches final right/ID1 contract. The print job
  used actual-size/no-scaling but physical 40mm black-square X/Y measurement remains
  pending. Device2 historical mount evidence used ID1 and cannot serve final left/ID0
  without ID0 mount verification when that hardware returns.
- Next: operator measures the device3 black square horizontally and vertically. If scale
  closes, run the 105s single-pair dynamic Ego-world + device3 VIO/tag HIL. Current free
  disk is about 25GB after removing 15.373GiB accepted static raw payloads under the
  standing cleanup authorization; more space is needed before a full 105s raw capture.

## Superseded: run20/run21 A′/B′ model PASS; independent C′ required (2026-09-14)

- run21 B′ native capture, coordinator, seal/index and pair verification pass: 450 raw
  pairs, 449 strict common-time rows, timing p95 6.458ms and static gyro peaks
  1.100/.767deg/s.
- Coverage-selected B′ scales are Ego4x/right2x. The compact A′/B′ extraction retains
  150/137 stride3 common-board windows, 33/28 IDs and 150 mount samples per pose.
- B′ is novel versus A′ by 78.649/78.752mm and 15.484/15.533deg in Ego/right. The
  mounted Tag remains stable to .041px, .028mm/.067deg, proving board-only movement.
- Shared-rigid checker holdouts pass at .218/.283px p95; folds differ only
  .143mm/.022deg; full-ID model p95 is .256px. The frozen candidate status is
  `PASS_MODEL_CANDIDATE_NEEDS_INDEPENDENT_CPRIME`, activation remains disabled.
- Next operator step: keep the complete Ego/device3/Tag assembly untouched. Move only
  AprilGrid to a third pose distinct from both A′ and B′, preferably 5--10cm in another
  direction with an opposite or different-axis 10--15deg tilt, then collect C′.

## Superseded: run20 A′ baseline PASS; operator must move only AprilGrid for B′ (2026-09-14)

- User confirmed the assembly was fixed. run20 A′ native capture, coordinator, 450-row
  index/seal, and pair verification all pass; timing p95 is 3.932ms and static gyro peaks
  are 1.082/.470deg/s.
- Coverage-selected detector scales are Ego3x/right2x, chosen only from decoded tag count
  before any residual calculation. All 450 frames pass with 26 common tags median;
  board PnP p95 is .119/.202px and the mount edge is about 97px.
- The static-window heldout mount prediction passes at .289px p95. Temporal halves differ
  only .037mm/.011deg. Per-frame planar PnP remains a diagnostic at 1.261px p95 and does
  not override the already established static-window estimator.
- A′ is `PASS_BASELINE_CANDIDATE_NOT_ACTIVATED`. Next operator action: do not touch Ego,
  device3, mount Tag, bracket, base, or cables; move only AprilGrid by 5--10cm and tilt it
  10--15deg while retaining broad visibility in both cameras, then capture B′.

## Superseded: run19 C proves rigid-pair movement; lock rig and restart A′/B′/C′ (2026-09-14)

- run19 C native acquisition, indexing, sealing, and pair verification pass: 450 pairs;
  cross-camera timing p50/p95/max is 1.181/2.490/2.540ms and both cameras are static.
- C has 150 usable common-board windows, 31 IDs with at least 30 samples, 150 mount
  samples, unchanged factory geometry, and sufficient novelty versus A/B. Its first
  fixed4x failure was a right-camera scale blind spot; a coverage-selected fixed2x replay
  restores detections but still fails geometry at 74.679px board p95 and 36.573px mount
  p95. Mixed Ego4x/right2x remains 74.195px, excluding detector scale as the cause.
- Independent C board and mounted-tag estimates agree on the current inter-camera pose to
  2.609mm/.326deg, while each differs from frozen A/B by about 11--12mm/6.5deg. The
  physical `devices+mount rigid whole session` contract was broken before C.
- Do not activate either transform and do not attempt to hand-restore the old A/B pose.
  Required operator action: rigidly lock Ego, device3, and the device3 mount tag in their
  current arrangement. Then restart A′/B′/C′, changing only the AprilGrid between poses.
- The superseded A/B/C raw `.bin` payloads were removed under the user's standing cleanup
  authorization after preserving extracted observations, reports, hashes, metadata and
  the failure diagnosis. 15.444GiB was reclaimed; free space is now about 25GB.

## Superseded: shared-rigid model fixed; independent pose C required (2026-09-14)

- User reports the board looks flat and requested continuation. Fixed stride3 A/B median
  observations retain148/150 paired windows,31/26 decoded common IDs and150 mount samples
  per pose with no residual filtering.
- Replaced the biased composition of two independent planar PnP poses with one joint model:
  shared `T_ego_ir_left_from_right_ir_left` plus one right-camera board pose per A/B;
  factory K/D and nominal35.2mm/10.56mm board geometry stay fixed, mount excluded from X.
- Checker parity cross-validation passes both directions: heldout p95.228/.256px; the two
  shared-X folds differ.316mm/.039deg. Full-ID A/B fit p95 is.232px. This supports planar
  PnP composition bias rather than visible board bow as the current cause.
- Frozen development candidate and exact replay manifest are ready. The C validator was
  exercised on B: board/mount scores.284/.271px pass, but novelty correctly rejects B as
  not new. Runtime activation remains disabled.
- Next operator step: move only AprilGrid to a third pose distinct from both A and B;
  keep the now-stable rig untouched. One C capture provides the final untouched whole-pose
  test before device-2 world-anchor reconciliation.
- Verification: model assertions and Python compile pass;21 relevant existing tests pass.
  One unrelated pre-existing CLI path test fails because it writes outside repository
  artifacts while production intentionally rejects such output. Free disk about15GB.

## Current: controlled run17/run18 A-B isolates pose bias; stop recapture (2026-09-14)

- run18 control-B acquisition/index/seal/pair verifier pass:450 pairs, timing p956.138ms,
  Ego/right gyro peaks1.020/.510deg/s,410 exact-common-ID accepted frames.
- Board-only change is valid: A->B center62.7mm and normal16.93/17.00deg in Ego/right.
  External mount is rigid:42um/.048deg pose change and[.034,.063]px center shift.
- Frozen A candidate passes A at.313px static-window p95 but fails independent B at
  2.994px. A B-only diagnostic candidate would shift1.109mm/.167deg and still cannot be
  used as acceptance. Factory K/D/extrinsics and stream profiles are identical.
- Controlled evidence isolates a planar-target/model pose bias rather than operator motion,
  timestamp failure, camera parameter drift, or insufficient board coverage. Runtime/world
  activation remains disabled. Retain A/B raw for offline flatness/print-geometry and
  multi-pose model diagnosis; do not request more capture until a testable fix exists.
- Current free disk is about9.4GB after retaining both accepted diagnostic sessions.

## Current: run17 controlled baseline A passed; operator must move board for B (2026-09-14)

- Rejected run16 raw was archived and5.534GB removed before capture; compact reports and
  identity montage remain. Free space returned to20GB before run17 and is about15GB now.
- run17 control-A acquisition/index/seal/pair verification pass:450 pairs, timing p95
  9.883ms, Ego/right gyro peaks1.020/.414deg/s,411 exact-common-ID accepted frames.
- run16->run17 confirms the newly fixed setup is stable: external-tag center shifts only
  .16px, edge .08px, and independently fitted candidates differ.165mm/.118deg.
- Per-frame planar PnP prediction is retained as a jitter diagnostic at1.785px p95. The
  correct bench-static window estimator uses train-frame board observations to estimate
  one constant inter-camera transform and scores only heldout mount corners; it reaches
  .313px p95 and temporal halves differ.038mm/.006deg. Baseline A candidate passes but
  remains NOT_ACTIVATED.
- Next requires operator action: keep Ego, device3, external tag, bases, bracket and cables
  untouched; move only AprilGrid5-10cm and tilt10-15deg, then capture control B. Freeze the
  run17 candidate and never fit B mount pixels; require B window prediction p95<=1px.

## Current: run16 protocol violation, recapture required (2026-09-14)

- run16 novel-pose acquisition, canonical seal/index, and pair verifier pass:450 pairs,
  cross-camera p9516.052ms, Ego/right gyro peak1.020/.733deg/s.
- Board novelty is sufficient:59/74mm translation and16.62/16.71deg normal change in
  Ego/right;410 frames meet the exact12-common-ID and3x3 coverage gate.
- The frozen run15 candidate fails run16 prediction at5.071px p95. Factory camera
  intrinsics/extrinsics and active stream profiles are byte-equivalent in substance.
- Identity montage proves the one-bit detector still selects the external mount tag, not
  the board ID1. Its Ego center shifts[-40.8,-85.8]px, edge grows83.87->96.48px, and
  observed Ego-from-mount pose changes54.1mm/1.85deg. The fixed-camera protocol was not
  preserved, regardless of which physical item shifted.
- Preserve run16 as failed diagnostic for now; do not activate or merge its fit. Next
  recapture only after Ego, device3 D405, external tag, bases and cables are rigidly fixed,
  with only the AprilGrid moved. Free disk is about15GB.

## Current: device3 mount candidate passed, independent pose validation next (2026-09-14)

- run15 is the accepted 15 s Ego D435i + device3/right D405 static session. Acquisition,
  canonical index/seal, and the pair verifier pass. Ego/right formal gyro peaks are
  1.005/0.577 deg/s; all450 Ego frames are present; cross-camera delta is12.175ms median,
  14.700ms p95,16.142ms max against20ms.
- The physical framing is adequate. A fixed4x detection scan has151 examined pairs,
  exact-common-ID median20/p95 23.5/max25,128/151 pairs at least12 common IDs, and
  140/151 with both board lattices valid.
- Fixed2x detection was an algorithmic blind spot for the soft D435i image. Fixed4x
  recovers the decoded IDs. Source-pixel fixed5 cornerSubPix is also required for this
  legacy two-bit junction board; it changes no decoded IDs and leaves factory K/D fixed.
- The final run15 mount replay accepts412 frames (206 train/206 heldout). Heldout mounted-
  tag corner prediction is0.687px p95; the two temporal halves differ0.020mm/.007deg;
  Ego/right board PnP p95 is.085/.226px. Every frozen candidate gate passes.
- Candidate `T_right_ir_left_from_mount_tag` remains NOT_ACTIVATED. Odd/even frames are
  independent images but the same physical board pose, so the next required evidence is
  a genuinely changed board/rig pose followed by the same heldout prediction and then
  the device-2 anchor/reconciliation plus independent HIL route.
- Rejected run14 was archived and5.478GB raw removed after run15 passed. Current free
  space is about20GB. Exact run15 replay script/report and hashes are retained under
  `artifacts/shared_world_device3_recovery_20260914/`.

## Completed: run9 endpoint-visible validation and bounded next step (2026-09-14)

- Captured and sealed a fresh 105 s Ego + device3/right rigid-rig session with
  Chinese phase guidance. Acquisition and both VINS replays pass.
- Recovered small Ego-grid detections with explicit 2x analysis scaling and source-
  pixel corner restoration; all seven protocol phases now have Tag-ID constraints.
- A whole-run fixed transform fails at 38.194 mm / 3.714 deg phase-anchor span;
  stride-3 independently confirms 36.991 mm / 3.656 deg. No activation.
- Coarse phase anchors are not robust: full-rate yaw holdout is 29.301 mm, but
  stride-3 is 34.157 mm, and phase-local epipolar P95 is 3.558-3.900 px.
- Short 1-2 s pose-only windows reduce heldout P95 to 12.394-13.811 mm and
  1.003-1.117 deg across full/stride-3. Run9 cannot supply the missing mounted-tag
  extrinsic: only seven numeric board+mount overlaps and zero pass the existing 60 px
  mount-edge gate. Next is a dedicated 10-20 s static preview-gated board+mount capture,
  then replay the device-2 anchor/reconciliation path and perform independent HIL.

## Completed: run8 Tag-ID world constraints and fixed-transform test (2026-09-14)

- Implemented reusable decoded-ID AprilGrid pose and `T_egoWorld_rightWorld(t)`
  constraint primitives in `three_device_slam/spatial/tag_world_constraints.py`.
- Added offline CLI, deterministic phase-local train/holdout split, explicit
  diagnostic policies, per-phase checks, provenance, JSONL and residual plot.
- Run8 full30Hz:325 constraints, PnP P95 Ego/right1.700/1.597px. Fixed transform
  rejected:phase anchors43.683mm/2.530deg, translation-phase heldout P9539.208mm,
  ID-corner epipolar P9514.815px, initial/final coveragezero. Independent10Hz
  sampling:96constraints, span46.897mm/2.766deg, same rejection.
- No transform or calibration activated. Next model is tag factors or time-varying
  world correction, then an independent endpoint-visible capture.

## Current: Ego + left UMI shared-world feasibility/integration (2026-09-06)

- Stronger OMP_THREAD_LIMIT1 control nowtwoexactreplays (889poses), but2.048deg
  staticexcursion remains. Diagnostic-only; no productionthread/calibrationchange.
  Evidence and next pre-Schur comparison recorded in handoff section67.

- Latest: instrumented the actual VINS copy and reproduced both trajectories.
  First differing prior at MARG611 precedes SOLVE636; all900input hashes equal,
  same iteration stops, no timeout at divergence. OMP_NUM_THREADS=1 did not fix.
  Unique numeric cause still open; next compare pre-Schur A,b/factor layout.
- Implemented scene preflight in offline runner: current31pairs have11–16unique
  candidate stereo matches and0% support. Default blocks beforeROS; diagnostic
  override cannot claimPASS.75tests pass; reviewMinor fixes included.
- Actual pinned COVINS source acquired, ROS1 dependencies inspected/missing;
  backend NOT_RUN. CurrentstaticEgo gives only1upstream-predicate keyframe.
  Needmatchedruntime andgenuineoverlap/motion, not more16stepboard recapture.

- Current continuation: locate first repeatability divergence before changes.
  Run2/run3 first635poses bit-identical; first difference index635 at21.1568s,
  then grows2.812deg betweenruns. Initialization/data conversion not the first
  divergence. Inspect solver timing, marginalization, frontend RNG/threading;
  choose one controlled hypothesis, preserve factoryconfig/externalrepos.

- Completed Ego built-in-IMU export and actual isolated VINS offline replay:
 900stereopairs/6013IMU,889poses~30Hz,56tests,fullpixel/gyroreadback. Run3 has
 prelaunch runner/library provenance. No hardware/external writes/commits.
 Accuracy still open: identical-input IMU runs2/3 angular excursions2.40/1.19deg;
 visual-only control1.59deg, not a causal IMU penalty or a fix. SharedworldNOT_RUN.
 See ego_internal_vins_probe_20260906_run3/RESULT.md; next actual backend runtime
 and genuine common-scene geometry, not another repetitive board capture.

- User "继续": implement Ego-specific offline input/replay using existing D435i
  stereo + built-in gyro/accel recordings. Preserve source hashes, factory frame
  geometry, native units/timing. Noise/td not measured: exploratory replay only,
  never calibrated/accuracy PASS. No live sensor or formal D405 runtime change.
  Inspect first, then export bounded input and use isolated VINS process if safe.

- User approved retaining D435i/D405 and borrowing iPhUMI architecture, not ARKit
  code or phone replacement. First inspect actual SLAM inputs/outputs and available
  recorded overlap, then implement the narrow offline map-alignment verification
  path supported by evidence. Do not present simulation as working shared maps.
- Inventory complete for repo paths: real left VINS raw body poses + IR DB3 exist;
  Ego has capture/factory stereo but no corresponding validated trajectory found.
  No hardware start, release changes or commits.
- Completed: offline COVINS-G image/odometry adapter on real left recording.
  Pin upstream wrapper convention; preserve raw local odometry (not corrected),
  camera-body extrinsics, exact source-image pairing and separate map identities.
  COVINS backend is not installed/run; export alone is NOT shared-world PASS.
- Final input run2: 1284 pairs, max stamp delta232ns; all pixels/body poses and
  seven source hashes independently read back. 39 tests pass, review issues fixed.
- Inventory correction: historical D435i temporary VINS/1440-pose PASS DOES exist,
  but explicitly disables built-in IMU and uses UMI's STM32. It is not the required
  independently-moving Ego path. Next: Ego-specific internal-IMU VIO and isolated
  COVINS runtime, then genuine two-device overlap/heldout validation. No recapture
  requested this turn; no calibration activation. See ego-umi-covins-integration.md.
- Pending: choose implementable cross-map constraints; test direction/scale/time/
  reset handling and recorded behavior. If genuine missing hardware data/calibration
  prevents end-to-end verification, report exact missing prerequisite, not fake PASS.
- Prior upstream optimizer comparison completed; mount discrepancy remains open.

## Current: upstream solver comparison (2026-09-06)

- User approved continuing the upstream-reference comparison. Offline only;
  preserve factory parameters, raw data, failed gates, and all external repositories.
- Completed: pin and inspect actual multical source, adapt frozen factory rays
  and identical board corners; execute upstream optimizer without fitting K/D/board
  or residual-based pruning. First baseline: original run4 train8/holdout5.
- Completed: 36 tests (4.14s); full original run4 replay; upstream BA 4.390738px vs own
  4.391012px. Actual upstream stereoCalibrate independent initialization yields
  4.390747px, then multical 4.390488px. No pruning/KD/board change; original REVIEW.
- Independent review v1 clear except tiny Rodrigues angle resolution; added stable
  scipy Rotation verification without overwriting old evidence. Extended independent
  initializer review also clear with full raw/fit replay and old-seed poison test.
  No calibration activation; physical root cause open.
- Setup note: initial plan patch used a nonexistent context line and made no change;
  corrected using the real title. Earlier repeated multi-pose proposals are superseded.

## Active implementation: common-board route (2026-09-06, user approved)

- CURRENT user "继续" after correcting duplicate multi-pose proposal: offline
  matched-data method comparison, no new capture. Yesterday run4 ALREADY jointly
  fitted8train and failedwhole-pose holdouts; missing multipose is NOT established
  cause. Compare old two-view joint math, IR-only rigid3D fit, three-view joint,
  on identical new IR/UMI pixels and A-only versus AB splits. Use identical scoring
  routes across candidates; AB B is training. Replay original RGB run4 for solver
  parity, never invent absent simultaneous RGB/IR/UMI observations or compare
  unrelated camera poses as causal sensor evidence. Preserve all failures/KD.
  Evidence new tilt_run3/matched_methods_v1. No runtime integration/activation.
  COMPLETE matched comparisons: old8train reproducedwithin.000287px; same
  stereo score A-only B two-stage2.453595/joint3view2.452627px, nofix. SameAB T
  A stereo.726 vsmono1.911px shows scoringroute changes cannotbe conflated.
  Extra IR-only sourcepose .508mm pointdifference predicts1.523px difference.
  Frozen IR-A-derived X/Y factor .9976495 improves tiltB2.454->.797 but worsens
  oldlockedB1.010->3.089; oldRGBmedian2.765->3.269. NOT a universalboardfix.
  aspect_control_v3 keepsoldREVIEW/novelty;v1/v2 superseded.45tests pass; no
  hardware or activation. No proof uniquephysicalrootcause; do not demand
  duplicate multipose recapture based on these failed development hypotheses.

- Current user "排查": OFFLINE only, no hardware/activation or factory writes.
  Freeze tilt_run3 and locked_ABA_run2 raw observations and A-first10 training.
  Compare native/scaled/edge corners one sensor group at a time against exact
  fixed5 replay, both direct and rigid geometry; then inspect per-view residual
  patterns and model consistency. No pruning, no fit-to-B acceptance, retain
  missingC/cleanup failures. New evidence goes in tilt_run3/isolation_v1.
  DONE offline corner/plane/depth isolation and explanatory A/B pose support.
  Corner swaps and IR-only depth smoothing do not fix tilted B. Independent
  A/B-only rigid T differ4.450mm/.343deg; fixed-KD AB fit A/B median.726/.519px,
  but B participates in fit, NOT new-angle acceptance.31tests pass; sourcefailures
  unchanged, nohardware/activation. See isolation_v1/RESULT.md and pose_support_v2
  (v1 superseded for B seed leakage). Unique physical cause remains unresolved;
  next candidate direction multipose sharedT, freeze before independentview test.

- User "好了" authorizes the proposed new tilt contrast. Add opt-in tilt gate
  to existing locked ABA workflow: A first, B normal change >=12deg in all3
  views, C return; retain all36/static/lock/300s/cleanup and default modes.
  Verify tests and read-only review before one bounded camera run. No robot
  motion, no factory edits, no activation; emitter OFF, main uncommitted.
  Run1 exited2/0samples/cleanup[]: UMI30, screenshot top row cropped. Split
  tilt-only framing wait from30s exposure-convergence timer; start latter once
  all36fresh, neverreset, retain total300s and oldmodes.37tests pass.
  Run2 toolsession24822 closed exit2/capture_timeout,0samples,cleanup[],IRrestored.
  Run3 toolsession35156 CLOSED exit2: A20/B20/C0, timeout WAIT_C; restore gain
  GET PIPE error retained. Later readonly query confirms original AE1/8500/16
  andemitter0 nowrestored. No liveprocess. This turn partial-data offline audit,
  no restart: check120PNGs/metadata/static/tilt, A-only fit direct/rigid controls,
  preserve incomplete/cleanup flags and missingreturn, never formalacceptance.
  Audit COMPLETE:120raw hashes,40all36,staticmax.524px,tilt23.72–24.28deg,
  crossphasemount.108px. A-only directB1.957px vsrigid2.454px(worse); nofix,
  noactivation.43tests andexactreadonlyreview. ReuseA/B forfurtheroffline diagnosis;
  do not re-capture by default or invent C. See run3/RESULT.md.

- NEW offline-only lockedABA geometry diagnosis: completed A20/B20/C20,
  exposure31979/gain23 stable, no active camera. Frozen medians1.430/3.030/1.467.
  Reproduce mean/per-frame residual and phase-specificpose; compare shared
  fixed-factory direct triangulation with rigid-board/stereo constraints and
  one-sensor focal/model sensitivity using trainA/B first10, untouchedC scoring.
  All controls diagnostic NOT_ACTIVATED; no global accuracy claims from C nearA.
  COMPLETE offline controls: fair A-only direct B3.042px -> rigid known-size
  B1.010/max1.089px, still FAIL; joint fit same. Prior AB replication remains
  B2.806px vs direct2.930, so NOT_FIXED. Individual IR routes do not fix it.
  Main sensitivity evidence and35tests in lockedrun2/model_diagnostic_v1/RESULT.md.
  Physical rootcause not unique. A/B normal difference only~1.4deg despite203mm
  translation; next useful live evidence is varied tilt, not another parallel
  translation. No new capture in this offline turn; no factory/runtime changes.

- User "继续": implement opt-in locked-IR continuous A-B-C(returnA) diagnostic.
  Read-only device query confirms Ego AE1/exposure8500us/gain16/emitter0.
  Preserve original modes, add automaticreturn gate/ghostoutline, per-frameIR
  actualexposure/gain evidence and checked option lock, A-only offline scoring.
  Test/review before one bounded live attempt; no factory calibration changes.
  Run1 idlelock8500/gain16 darkIR0 ->agentstop0data, restoregainGETPIPE retained.
  Revised converged LIVE-metadata warmup>=6s/max30s, actual locked31979/gain23.
  COMPLETED run2 toolsession6108 exited0: A20/B20/C20,180PNG, cleanup[].
  Do not poll/restart stale6108 or follow the historical WAIT_B operator note.
  Frozen check FAIL; see handoff§58–59 and offline controls above. No activation.

- 2026-09-06 current request: offline analysis of completed continuous_run2.
  A20/B20,120PNGs; frozen check FAIL B medianp95=2.9164px, max3.1553px.
  Investigate static-vs-changing frames, independent mount texture, epipolar vs
  depth residual, per-view geometry and diagnostic-only refits; no hardware,
  no production changes, no B fitting used as independent acceptance.
  COMPLETE bounded analysis: source120 hashes and40 frozen scores reproduced;
  static15B median2.913818px; epi2.456160px excludes depth-only correction at
  fixedrays/T. Diagnostic fits differ5.106mm/.362554deg, center2.63mm not5.106.
  RawNCC mount~.53px, backgroundinconclusive, EgoIR exposure notlogged/locked.
  Physicalrootcause unresolved; recommend lockedIR A-B-A return control, not
  executed/authorized this turn. See continuous_run2/RESULT.md. No activation.

- NEW user authorizes continuous-stream A/B. Implement opt-in automatic all36
  static gate, A10s then wait onlyboardchange, B10s, max300s, noR/no stream restart.
  Keep emitterOFF, samefactory, allrawfailures, stage/time evidence; analyze frozen
  A-only T on B, never fit B for acceptance. No outsidewrites/activation/commits.
  IMPLEMENTED; authorized run ego_ir_umi_continuous_20260906_run1 stopped at
  300s timeout in WAIT_A, A=0/B=0, cleanup_errors=[]; no capture process remains.
  Ego IR 36/36 each, UMI ended32/36. Preview screenshot shows top/right board
  clipping in UMI. Needs physical board reframing before another authorized run;
  no A/B result or restart-cause conclusion, no automatic retry/activation.

- User confirms NO camera touched/moved. Do not equate mount pixel shift with
  physical movement. Offline run5→run6 natural-background and mount texture
  tracking, optical-ray checks; no newcapture/activation. Preliminary UMIbottle
  ~11.5px shift vsfewgrippertracks~0, IRbackgroundmostlysubpixel; verifyallframes.
  ExploratoryemptyROI percentile error logged; new diagnostic must retain null
  for insufficient tracks instead of crash or claimingstable.
  DONE offline20frames: UMIbottleLK11.49/-2.33px andNCC11.35/-2.13, oldcontrol
  .01px; IRmountwholepatternNCC>.996 shifted~1/1.2px independentofTagdetector.
  Grippertracksinsufficient; no physicalcause/operatorblame conclusion. Epipolar
  p95~9.9pxrulesoutIRdepth-onlyfixunderfixedrays/T. Renamefailuretoprojection_changed.
  Next proposed continuous-stream two-board-pose check; nohardwareauthorizedyet.

- NEW user "改变了": independent board-pose check, assuming cameras/mount fixed.
  One bounded10s run6 capture; freeze run5 direct_mean all36 transform BEFORE
  evaluating new images; no new-view fitting for acceptance. Keep original gates,
  source/calibration identity, static checks, board novelty and mount stability.
  COMPLETE run6 20triples60PNGs/10.000431s/cleanup[]. Board changed91mm/11deg,
  but UMIboardcropped24–25/36 and IRmountdrift1.7/2.5px exceeds.75. FrozenoldT
  p95median11.980/max12.302px FAIL; cannot isolatealgorithmfromsetupchange.
  Wait operator clarification whether Ego/UMI also touched before anothercapture.

- Current user NEW approval: one automatic10s Ego dualIR + UMI color capture.
  Implement isolated callback-based capture and direct IR3D-to-UMI2D diagnostic;
  preserve raw/metadata/factory values, emitterOFF, no calibration activation.
  Fixed spatial groups and heldout pixels, static/timing/size failures retained.
  COMPLETE capture run5:20triples60PNGs/10.000457s/cleanup[]; runs1-4 startup0samples
  retained. gainGETPIPE optional warning only; mandatoryemitter0. Direct20frames
  horizontalmedian2.295/max4.038mm,vertical1.780/max2.607,checker.525/max1.230;
  holdout2.144px andsmallmount54px FAIL. Group-onlyseedleakfix+negative test.
  Posthoc3raybundle reducesgroupdifferences butheldoutprojectionworse; reject.
 125tests pass; allcamerasstopped, NOT_ACTIVATED, 10sauthorizationconsumed.
  Evidence ego_ir_umi_20260906_run5/RESULT.md. Mainaccuracyissue remainsOPEN.

- NEW user confirmed10s automaticEgoRGB+dualIR capture, noR required. Started
  ego_stereo_color_probe_20260906_run6 with --record-now, emitter0, samefactory
  profiles1280x720@30, rawsample2Hz. Analyze newnativeboarddetector(allrawduplicates),
  sameframeboard/mountidentity, static.75px andIRtimestamp.2ms, triangulation→RGB
  factoryandfrozenK4k1 candidates WITHOUT refit. Preserveallframes/failures.
  DONE run6 capture20triples60PNG cleanup[]; all36/allstreams nativeidentity and
  staticmax RGB.571px/IR.140px pass, mountIR~54px below60 preservedFAIL.
  JointfactoryIR→RGB boardmedian2.568px/mount2.608px; boardonlyTfit heldoutmount
  27.220pxmax demonstratesplanarcompensation; oldK4k1+T gives1.933pxstillFAIL.
  Addedmultidepthcandidate trainold1/2/4/5IR3D no print dimensions,holdout3/6/allmount;
  newrun6 boardmax.518/mount.918px,oldrun3max1.277FAIL. Externalold4.845→7.891
  worse/new5.631→2.799better, soNOT_ACTIVATED. Geometricalidentityguardfixed/review.
  CamerasSTOPPED;user10sauthorizationconsumed. Further simultaneousIR+UMIhardware
  needsnewapproval, notautomaticretry. Evidence run6/RESULT.md. MainissueOPEN.
- ACTIVE repair request: fixed-image factorial diagnosis before any calibration
  activation. Hypothesis1: downstream cornerSubPix introduces regional bias;
  compare native/raw vs fixed5 vs existing edge refinement on exact12pairs/all36.
  Hypothesis2: normalized-ray LM vs pixel-model objective changes pose; keepKD,
  board/mountgeometry/sourcepixelsfixed. Assess halfboarddisagreement AND heldout
  pixels; no parameter tuning to desired46mm, no automatic acceptance basedonmean.
  Preserve alloldreports, nohardware, onlymainrepochanges. Then implement only
  reproduceddefect withregression andindependentreview; unresolvedcause staysopen.
  DONE ablation all12: native/scaled/edge/pixelobjective retain~13mm horizontalgap;
  idealEgo .866mm vs idealUMI14.085mm localizesdominantEgoobserv/modelinconsistency.
  Implemented per-camera old8train K4+k1 candidate, new12 horizontalmedian2.566mm,
  max3.376 butverticalmax5.450mm;oldpairedholdout4.391→1.522px stillFAIL.
  External regression old4.845→9.826mm worse/new5.631→4.835mm better: REJECT
  generalactivation.114tests/review,referencebindingfixverified; noformalKDchange.
  Await optional10s RGB+IR-L+IR-R currentstatic scene confirmation; noR/16steps.
  Evidence common_board_joint_20260906_run1/POSE_REPAIR_RESULT.md. MainissueOPEN.
- NEW user requests more measurements with36boardtags. Offlinefirst: auditall12
  rawpairs ratherthanonlytwo middlepairs; reconstructboardID1 ONLY uniqueinside
  nonID1boardfootprint and predictedcanonicalquad<=3px. Newderiveddiagnosticpolicy,
  neverchangeold35-IDcapture/acceptance. Freeze groups beforeoutcome: whole35/36,
  parity halves/top-bottom/left-right,6rows/6cols,leaveeachID/row/colout. Fullframes,
  noresidualpruning/activation; temporalrepeatabilityvs spatialgroupsensitivity.
  DONE: all12pairs/all36tags,432individualchains,68frozengroups*12=816fits.
  Whole norm56.007–56.876mm/maxpositionrepeat1.381mm; checker.697–2.202mm,
  horizontal11.644–15.623mm,vertical2.760–4.203mm. ID1restorewholechange.508mm
  median. Horizontaldelta mainlyy13.135mm; narrower-support/modelbiasconfounded,
  notabsoluteerror/uniqueKDcause.108tests/independentreviewclear. Evidence
  common_board_joint_20260906_run1/all36_resampling/{report.json,RESULT.md}.
  Sourceunchanged,camerasSTOPPED,NOT_ACTIVATED; nextregionalresidualvsgeometry
  sensitivitydiagnosis, no manual16step demand or calibrationPASS.
- LATEST user wants static per-tag analysis, NOT manual16step calibration. Stop
  oldpreview normally; readbackrevealed2acceptedjointwindows already saved byR.
  Use these existing rawwindows immediately, no newautocaptureimplementationneeded
  to answercurrenttask. Verifyhash/staticreplay; comparewholeboardX/Mvsindividual
  tagchains and heldoutspatialtag prediction withfixedfactoryKD. Preservealltags/
  branches, separate repeatability fromabsoluteaccuracy; nocalibrationPASS.
  DONE: existing2windows hash/staticreplay verified, all35individualtagchains plus
  wholeboard and parityfolds compared. WholeMnorm56.519/57.016mm,Δ3D.693704mm;
  paritygroupsstill2.573/3.420mm apart. 105tests/reviewclear. No newcapture/autostatic
  implementationneededforcurrentdata; camerasSTOPPED, no manualRrequiredforanalysis.
  NEXT use staticdiagnostics forrootcause; do notconflate near-identicalpose repeat
  withabsoluteaccuracy or demandfull16step workflow absentoperatorintent.
- ACTIVE supersedes sequential placement: implement explicit simultaneous mode.
  Ego must see public board AND mount in every retained pair; UMI sees board.
  Fixed identity policy excludes board ID1 in BOTH cameras upfront (35 other IDs),
  preserves raw detections, distinguishes mount ID1 outside fitted board footprint;
  ambiguous/overlapping/extra ID1 remains blocked. No size/KD tuning or old replay
  reclassification. Freeze policy in new setup, no cross-mode continuation.
  Keep original16window split/geometry solver but collect board+mount every step,
  check both static gates, show distinct board/mount colors, no remove-board prompts.
  DONE: real synthetic decoding, duplicate/occlusion/stale/movement negatives,
  capture/loadroundtrip, fixedMpredictiontransform;103tests and read-onlyreviewclear.
  LIVE execsession26845 common_board_joint_20260906_run1, firststagepreview;
  operator places board+mounttogether beforefirstR. No newmetricPASS. Do notrestart
  sequentialplacement oraskuserremoveboard. Ifsameframequalityfails, retainreason.
- LATEST user asked to fix unstable detection and visibility, continue implementation.
  Completed quantitative same-image comparison and optional UMIaprilgrid0.5.0
  native backend with raw duplicate preservation; 53tests, reviewed. Fresh display
  exposes conflicts instead of erasing whole board visually. Backend binds sessions,
  old OpenCV replay unchanged, continuation cannot switch. Placement-only3step mode
  checks mount->board->mount BEFORE16steps, current displayed frame included in
  static gate. No old-X reuse after camera motion; no calibration status inflation.
  Live session84894 in common_board_placement_umi_detector_20260906_run1,300sbound.
  NEXT: operator placement actions; observe real detector behavior. Original spatial
  residual unresolved. Old-support solve contrast blocked on missing IDs17/29 at
  slot8/left; no residual-driven point deletion allowed. Do not rewrite old setup.

- SOURCE CORRECTION (user reminder): UMI formal calibration_assets already records
  the same physical board:6x6, black35.2mm, gap10.56mm, pitch45.76mm. Source
  /home/robot/umi_docker_device2_d405_formal_20260828/calibration_assets/aprilgrid_6x6_35mm.yaml
  agrees exactly with current config and captured target. Handoff§32 already said
  same board/no repeated measurement. Do NOT block continuation asking user to resend
  vendor specs; latest request corrects that mistaken blocker. Preserve measured
  residual failures; use established board geometry, inspect camera/image pipeline.

- Active user request: inspect common-board optical-center-to-tag geometry.
  Offline only. Audit target/frame conventions and original raw bindings; compare
  per-window whole-board PnP, disjoint-tag prediction, and inferred fixed mount.
  Keep factory K/D/layout and all original frames/IDs; no activation or fix by fitting
  already-inspected holdouts. Produce distance evidence, separate Euclidean norm
  from z-axis component, and state any unidentifiable physical cause explicitly.
  COMPLETED offline audit: formulas/gauge/synthetic tests correct, 12–24 tags/window.
  Per-window inferred mount norm43.187–54.302mm, max3D difference18.130mm (not real
  motion measurement); pooled candidate norm50.125mm versus z=-47.369mm.
  Same-tag heldout H beats rigid PnP25/26 Ego and24/26 UMI; not just training DOF gain.
  More prominent Ego model/geometry mismatch, but unique physical cause not identified.
  IR metric vs declared board also+.7–1.0%; do not treat either as absolute truth.
  Next camera-model/image-pipeline checks use existing UMI board assets (see source
  correction above), not another blind16-step capture or heldout-driven tuning.
  Full evidence common_board_distance_audit_20260906_run1.

- CURRENT run5 completed 20 triples at IR median depth 0.4205 m, cameras stopped.
  Confirmed near-image fixed5 corner-search defect; opt-in scale-aware refinement
  and same-source/no-pruning comparison added. RGB drift 9.839 -> 1.103 px,
  still above 0.75; old frozen k1+SE3 remains 4.041–4.992 px after repair.
  Candidate NOT generalizable / NOT_ACTIVATED. Original common-board run4 replay
  (fixed factory, same windows/IDs/split, X/board poses refit) gives worst heldout
  4.391012 -> 4.389197 px: near-corner defect does NOT explain original failure.
  Evidence common_board_corner_scale_20260906_run1 and probe_run5/paired_corner_fix_comparison_v2.
  Original 5.631 mm full-chain issue unresolved; no new blind capture, no factory
  overwrite, no commit. Next evidence must discriminate systematic camera-model /
  IR reference errors across depth, not fit already-inspected views then claim validation.

- CURRENT run4 closerattempt completed20triples underexplicit"好了"authorization.
  All20valid, frozenprimarypixelp95 .59838–.93444px satisfies1px, butoverallFAIL
  because15framesdrift>.75px; maxRGB1.97865/IRL1.34618/IRR1.41855px.
  IRmedianopticaldepth .63357m, notrequested.45–.50m; image shows handholdingboard.
  No retuning/pruning/activation. Needstablephysicalsupport andcloserplacement
  beforeanynewboundedcapture; currentpermissionused,camerasstopped. Keep prior
  thirdpose1.276pxFAIL visible; thismovingrun doesnotcloseoriginalmountissue.

- LATEST THIRDpose completed asrun3 (20triples cleanstop). Frozenk1/1e-5 from
  two_pose_exploratory_fit SHA84cd4e3b... testedwithoutrefit: FAIL16/16validframes,
  p951.03258–1.27574px (sensitivityworst1.28693), static/IRtimestampgatesallpass.
  DoNOTrefitthirdposeandcallvalidationpassed; candidateNOTACTIVATED.
  Independentreviewerexactreplay. Existingdata nativeIRNCCcheck firstvalidframe
  ofeachpose,2patchsizes: no grosscornerdisparity mismatch; thirdposep95 still
  1.158–1.168px vs1.195baseline. Thisisposthocdiagnosticonly, formalFAILunchanged.
  Residualperpendicular todepthdirection remainsRMS.402/p95.711px; notdepth-only.
  Threeposesdepths~.798/.768/.778m: insufficientdepthdiversity toseparate
  constantdisparity/systematiccamera-model errors. NEXT discussdepth-separated
  comparison or independentmetrology ratherthananotherlateralpose; no new
  captureauthority, hardwarestopped. Original5.631mmchainrootcauseNOTclosed.

- CURRENT after user's next"好了": secondpose10s completed as
  ego_stereo_color_probe_20260906_run2,20triples cleanstop. BackmountID1uncovered
  duplicatesboardID1inIR; uniformlyexcludeID1allframes/allstreams, run1replay
  identicalonallvalidmeasurements; ID1notinold evenparitySE3training.
  Fixedrun1SE3crosspredictionrun2 FAIL: factory-local2.942px, k1+local1.757px;
  2xderivativestep2.897/1.762px.11validframes/20, nostillnessissue/developmentframe.
  ExploratoryTfit usingfirstvalidframe/evenIDs ofBOTHposes withfrozenoldk1 gives
  oddheldoutworst[.664,.256]px (2xstep[.667,.254]); factory[1.612,.765].
  NOTindependentposevalidation. Candidate frozeninrun2/two_pose_exploratory_fit.
  NEXT requires THIRDposepermission: camerasfixed, boardmoveleft~10cm+opposite
  tilt15deg; RGB+IR10s, preview first. Scorefrozenboth-posecandidate unchanged,
  no refit beforethirdposeevaluation. Original5.631mmmountproblemnotclosed,
  diagnosticRGB-IRcompensationnotautomaticallyamountextrinsicfix.

- LATEST: user confirmed manufacturer rigid board and explicitly allowed10s
  Ego RGB+IR-L+IR-R capture. Completed20triples, emitterOFF, cleanstop.
  Evidence ego_stereo_color_probe_20260906_run1. Baseline detector cannot decode
  enough IR; diagnostic C11/window53/3x decode +native refine yields10frames,
  sample5 development excluded =>9diagnostic frames. Fixedfactory RGBp95
  2.044–2.179px vsIR .063–.096px; neither reference proves absolute metric accuracy.
  Frozenpaired k1 +unchangedRGB-IR extrinsics worsens4.314–5.403px.
  However localSE3 compensation with stablecentralderivatives gives heldout
  worstfactory1.091px / k1 .779px: singleplane K/D/extrinsic confounding unresolved.
  Originallocalfit report is INVALID for localfit conclusions (SDK finite
  difference quantization); use frozen_candidate_analysis_v2/step2 only.
  NEXT requires operator repositioning board with cameras fixed and fresh
  boundedcapture permission; vary image location/normal/distance, retain triple
  streams to test frozenmulti-pose model. Do not repeat16steps or activate k1/M.
  Manufacturerboard confirmation weakens paper-wrinkle hypothesis but does not
  independently certify actual dimensions/planarity. No unique rootcause yet.

- NEW LEAD: Ego-only k1=.05621496 improves EVERY paired heldout (worst4.391->
  1.782px), UMI k1 does not. EgoD5 worst1.392px but k3 hits bound: do not adopt.
  Freeze simple k1 from paired train8; next test earlier mono normal/180 clips
  without tuning it, and independent edge localization on current pairs. This
  targets Ego zero-D model as contributor, not yet unique lens-vs-corner proof.

- Continuing per explicit user persistence request: K-only comparison is not a
  stopping condition. Next frozen diagnostic is D-only with factory K unchanged:
  baseline, Ego/UMI k1, Ego/UMI D5; same train8/holdout5/selected corners, fixed
  distortion enum and board geometry. k1delta +/- .2; D5delta bounds
  +/-(.3,.5,.02,.02,.5). Known SDK radial error recovery test passed,4tests total.
  In parallel inspect corner extraction/view-dependent error, not new capture.
  No deployment or claims that model-fitting proves unique physical cause.

- COMPLETED authorized K-only comparison, still NOT_ACTIVATED: five frozen cases
  all fail1px heldout. Worstp95 factory4.391 / Ego focal5.860 / UMI focal4.126 /
  EgoK4 6.470 / UMIK4 4.848px. Doubling SDK K finite-difference step changes any
  heldoutp95 <.0034px, K entries<.0192px.53tests pass including SDK K4 recovery.
  No basis to replace factory K; distortion not varied, no unique root cause.
  Evidence common_board_intrinsics_compare_20260906_run2/report.json; main
  run4 solve/report.json unchanged. Do not label this as calibration completed.

- User now explicitly authorizes comparison of offline diagnostic intrinsics.
  Frozen experiment: factory baseline, Ego-only focal scale, UMI-only focal scale,
  Ego-only K4, UMI-only K4; fixed distortion/layout/size, same train8/holdout5
  windows and all selected corners. Bounds focal .9–1.1 x factory; principal
  point +/-40px. Never auto-select/deploy a winner. Synthetic recovery/leakage
  checks before real solve; preserve original result and factory config. Report
  bounds/conditioning and duplicate holdout caveat even if residuals improve.

- Latest diagnostic-only follow-up: dense optimizer reproduces original X within
  .000656mm (same objective), so no convergence fix indicated. Same-ID temporal
  subset repeats .690–2.705mm across8usable windows;5windows skipped for <8
  all-frame common IDs (diagnostic selection, not acceptance/gate change).
  Geometry-only essential fit changes R .793deg; no metric scale recovered and
  Sampson error MUST NOT be compared as equivalent to full reprojection acceptance.
  Triangulation with unaccepted X does not establish independent board accuracy.
  No unique root cause. Proposed NEXT: seek permission for offline diagnostic
  intrinsics comparison, preserving factory/product values and all old reports.
  Do not silently fit/replace K/D or ask another blind capture.

- CURRENT: fresh run4 completed all16steps,24attempts/144pairs; offline hash/gate
  replay succeeded and solve/report.json is REVIEW (exit2), NOT_ACTIVATED.
  Cross-view heldout p95 Ego->UMI2.377–4.391px vs1px gate; reverse0.892–1.421px.
  Training residual / camera consistency / heldout novelty also fail. Step6 vs7
  differs only0.430mm/0.167deg. No split/gate/factory changes or recapture yet.
  Read-only monocular replay: Ego RMS0.317–0.492px, UMI0.157–0.283px;
  discrepancy appears in cross-camera consistency, root cause still unproven.
  Run2 must NOT resume: user clarified both cameras moved. Run3 was preview;
  board-stage duplicate ID1 handled operationally by covering mount until needed.
  Next: diagnose pose-dependent cross-view error on run4 without fitting K/D,
  board geometry, or choosing solutions using heldout target residuals.

- Run2 follow-up software fixed/reviewed: user reached step14,13steps accepted. Raw attempt24/25
  shows same physical ID1 decoded by both border2 and border1 detectors at
  overlapping quads; unconditional `if raw` falsely demands board removal.
  Plan exact-ID+geometric duplicate suppression only, preserve other board alarms;
  tests + immutable replay done; added hash-bound continuation from stopped partial session
  with fixed-device/operator confirmation and final board closure. No pose/gate
  relaxation or old-record relabel. User stopped then reported movement between
  stages; asked which object. Review clear,94regressiontests pass. PAUSE physical
  continuation/installation solve until clarified; do NOT assume unchanged cameras.

- Follow-up fixed: run1 all3 first-step failures were ID flicker with per-tag
  drift<0.45px. v2 gate preserves0.75px, requires each frame's>=12/3x3 overlap
  with first frame and checks all repeated IDs vs first observation. Selected
  middle-pair IDs must be repeated in each camera; full quality bound on reload.
  All3 immutable v1 windows pass revised static replay (diagnostic only);84
  focused/regression tests pass; independent review clear. No live camera start.
  OriginalFAIL retained. Next operator command uses new run2, READY thenR.

- Software implemented: paired preview-first static windows; fixed factory model joint X;
  independent mount M and whole-window held-out prediction. No camera auto-start.
- Freeze before acquisition: 8 training + 4 held-out board windows, 3 mount windows,
  then 1 board closure window (held out; detects camera disturbance during mount stage);
  each window >=3 pairs spanning >=1s; <=0.75px drift; host-arrival pair gap <=75ms
  (static proxy only, NOT synchronized exposure). Board >=12 tags over >=3 rows/columns,
  >=15px edges. Mount one-bit ID1/40mm, >=60px, board absent in mount stage.
- Acceptance: every held-out direction/window p95<=1px; training board normals span
  >=15deg and centers span >=80mm; per-window camera-X disagreement <=5mm/3deg;
  independent setup fixed-M mount p95<=1px and full3D repeatability <=5mm/3deg.
- Reviewer catch fixed before hardware: every main holdout pose must differ from
  every training pose by >=20mm center OR >=7deg normal using UMI-only pose estimates;
  final board closure exempt from novelty, not from cross-prediction.
- Preserve every attempted raw frame/rejection/hash; no K/D, size or layout fitting;
  no runtime activation. Software/simulation completion is not hardware acceptance.
- Software verification: 75 focused/regression tests pass; independent review
  clear after fixing duplicated-pose holdout acceptance. No HIL acceptance yet.
- Remaining: operator-led new paired capture, solve and independent setup validation.
  Existing four monocular clips
  cannot supply the missing common-pose observations.

## Goal

Current continuation 2026-09-14: Device3 right static mount calibration A′/B′/C′ passes. The 40.0 mm Tag scale is closed by the hash-bound vector PDF, printer job 17 with no scaling, and the operator's confirmation that the installed tag is that same asset; no caliper measurement is claimed. Reuse run9's accepted 105 s motion/synchronization and two VIO trajectories; its failed fixed-world transform must not be activated. First perform the retained-data offline regression, then run only a bounded current-candidate end-to-end anchor HIL to close the missing integration evidence. Do not repeat the full 105 s protocol. Device2 left-ID0 reconciliation remains deferred until that hardware is present.

The bounded HIL entry is ready as `--mode device3_anchor_hil --duration 45 --start-delay 20 --preview-hz 5`. The setup UI gives 15 s for adjustment and 5 s for settling, then Chinese cues keep Ego/AprilGrid fixed while moving only device3. Await the operator's physical readiness before opening the cameras.

Read the existing D405 coordinate-transform implementation and relevant open-source Ego/UMI alignment systems, then add a deterministic, evidence-producing simulation that validates independent left and right UMI trajectories transformed into the Ego-defined world frame without touching any D405 or Ego release tree.

## Constraints

2026-09-06 user requested open-source solution: research completed, proposal in docs/acceptance/opensource-spatial-calibration-route-20260906.md. Recommended next implementation is common known-geometry AprilGrid paired-static fixed-factory extrinsics X, then independent mount M and heldout validation. This is proposed, NOT implemented/accepted; four prior monocular clips lack cross-camera common-pose constraints. No more blind scattered-tag repetitions. Runtime keeps Ego + two UMI as independent moving bodies, not a fixed rig.

- Work only on `/home/robot/three-device-slam` branch `main`.
- Treat `/home/robot/ego_vio_humble`, D405 repositories, and release directories as read-only.
- Do not claim HIL or real spatial accuracy from simulation.
- Preserve the product topology: one Ego with independent left and right UMI chains behind one launcher.

## Phases

1. **completed** — Inspect repository and read-only D405 coordinate/time conventions.
2. **completed** — Research primary open-source Ego/UMI alignment implementations and record reusable patterns.
3. **completed** — Write failing tests for SE(3), time interpolation, world anchoring, and UMI-to-Ego-world alignment.
4. **completed** — Implement the smallest deterministic simulation and evidence report.
5. **completed** — Verify focused/full tests, inspect isolation, and obtain independent code review.

## Success criteria

- Frame convention is explicit: `T_A_B` maps coordinates from B to A.
- Simulated `T_W_U` is reconstructed from `T_W_E`, `T_E_U`, and arbitrary-origin UMI VIO increments.
- A nonzero time offset demonstrably causes error and its correction recovers the trajectory.
- First healthy post-recording Ego pose defines the gravity-aligned world origin.
- Left/right UMI chains both run by default and keep independent calibration, time, anchor, and health state.
- Output verdict is `PASS/simulation`, never HIL PASS.
- Existing tests remain green and external read-only trees remain unchanged.

## AprilTag Ego-world dual-UMI alignment and DEMO export (2026-08-28)

### Goal

Extend the already-completed deterministic spatial simulation with a production-shaped AprilTag observation/alignment layer for one Ego plus independent left/right UMI chains, while keeping the Ego SLAM world as the only global frame. Produce replayable machine-readable DEMO evidence before any HIL claim.

### Constraints

- Work only in `/home/robot/three-device-slam` on `main`.
- Keep `/home/robot/ego_vio_humble`, every D405 repository, and release directory read-only.
- Use `T_A_B` to mean a transform that maps coordinates from frame B into frame A.
- AprilTag IDs identify left/right rigid mounts; they do not define the world frame.
- Preserve image measurement time separately from detector completion time.
- Simulation/replay output is `PASS/simulation` or `PASS/replay`, never HIL PASS.

### Phases

1. **completed** — Inventory the existing spatial simulation, coordinator extension points, dependencies, tests, and current worktree without modifying external trees.
2. **completed** — Freeze the AprilTag observation, mount-extrinsic, clock, alignment, trajectory, and DEMO manifest schemas with explicit frame directions.
3. **completed** — Add red tests for left/right ID isolation, transform composition, temporal interpolation, planar-pose rejection, occlusion, and relocalization.
4. **completed** — Implement the smallest deterministic AprilTag alignment/replay path and world-trajectory DEMO exporter.
5. **completed** — Generate a synthetic/replay DEMO session and overlay/3D evidence where supported; report unavailable hardware stages as BLOCKED.
6. **completed** — Run focused/full verification, inspect isolation and repository state, and independently review the changed code before completion.

### Success criteria

- Both left and right chains use independent tag IDs, mount extrinsics, local odometry frames, anchors, and health state.
- `T_W_G = T_W_CE * T_CE_A * T_A_G` is verified against known truth and inverse-direction mistakes fail tests.
- A UMI local trajectory is aligned with `T_W_O = T_W_G * inverse(T_O_G)` using a robust window, not one unqualified frame.
- Tag processing latency cannot change the measurement timestamp; configurable acquisition offsets are applied before SE(3) interpolation.
- Invalid depth, excessive reprojection error, unknown IDs, and discontinuous planar-pose flips are rejected with auditable reasons.
- Tag occlusion preserves the last accepted world-to-odometry anchor; reacquisition is quality-gated and does not silently reset the world.
- DEMO artifacts bind source/calibration/config hashes and expose pose source, quality, timing residual, and rejection counters.

## Errors encountered

| Error | Attempt | Resolution |
|---|---:|---|
| ROS Humble `launch_testing` collection removed the repository package from imports (`ModuleNotFoundError: three_device_slam`) | 1 | Run repository tests with explicit `PYTHONPATH=.` so collection reaches the intended test module. |
| AnySearch could not reach its API for the Kalibr YAML-format extraction while adjacent primary-source requests succeeded | 1 | Record the transient failure; use local Kalibr output/help and retry the official page before freezing transform-direction conversion. |
| A combined planning-file patch used a context line after its section had moved and was rejected | 1 | Re-read the file tails and apply the same documentation updates with exact current context; no partial patch was written. |
| `v4l2-ctl` is unavailable on the HIL host | 1 | Use installed ffmpeg/GStreamer, direct V4L2 ioctl, sysfs, and the vendor XU bridge; avoid an unnecessary system package mutation. |
| AnySearch batch query rejected a shared `--max_results`; retry then returned only auto-generated credentials, not results | 1–2 | Did not persist or reuse the credentials; switch to public web/code search without storing secrets. |
| User ran the illustrative Kalibr command with literal `SESSION`; `ego_stereo.bag` did not exist | 1 | Confirmed no ROS1 bag exists in the repository. Treat the prior command as solver-only, require a real capture/export stage and concrete session path before invoking Kalibr. |
| A combined vendor-evidence planning patch included a stale example error-row context and was rejected atomically | 1 | Reapplied the findings/progress update without the unrelated stale hunk; no partial write occurred. |
| Selector-1 XU polling requested at 200 Hz reaches only about 100 distinct reads/s, including with a 210 Hz video mode and two concurrent readers | 1–3 | Do not repeat the same ioctl design; test direct USB control transfer, then search for a documented streaming/FIFO endpoint. |
| The first 200 Hz recovery-plan patch applied hunks in reverse file order and was rejected atomically | 1 | Re-read the plan and apply the error row before the later subplan section. |
| Direct read-only libusb control transfer could not open `1bcf:0b15` as the current user | 1 | Keep the working V4L2 XU bridge, but do not expand the backup group. Later X5 evidence proved the PC bridge was halving query rate by issuing `GET_LEN` before every `GET_CUR`. |
| A second planning patch again placed a later-file phase hunk before an earlier-file error-table hunk and was rejected atomically | 1 | Apply planning updates in ascending file order. |
| Misread “标定命令” as the offline Kalibr solver instead of the operator-facing calibration-data capture command | 1 | Build and HIL-verify a real 60/200 capture entry point before presenting any command. |
| `ffprobe` and `ffmpeg` were unavailable while validating the generated DEMO MP4 | 1 | Do not install packages or repeat the unavailable command. Reopen with the installed OpenCV backend, decode all 900 frames, and verify reported FPS/resolution plus manifest SHA-256. |
| First DEMO `v1` labelled regenerated clean poses as rejected PnP outliers and only inferred reacquisition from visibility | 1 | Preserve `v1` as superseded evidence; add red tests, export the actual rejected matrices, run a post-occlusion 5-candidate/4-inlier anchor solve plus continuity reconciliation, and generate hash-bound `v2`. |
| A module-level/default video regression loaded `cv2` into the shared pytest interpreter and failed the existing worker import-isolation test | 1–2 | First moving the explicit import into the test was insufficient because the renderer still imported `cv2`. Reproduced with the exact two-test order, then moved video generation/decode into a child interpreter. Minimal order passed 6 tests and final full regression passed 757. |
| First calibration-capture HIL serialized JPEG and IMU writes under one lock: 55.36 Hz video / 106.73 Hz IMU | 1 | Move all storage to one bounded writer queue so camera and XU producers never perform disk I/O. |
| Queued writer restored 59.49 Hz video but IMU remained 163.31 Hz | 2 | Differential run proved Python's full JPEG structural scan held the GIL; use constant-time SOI/EOI checks during acquisition and defer deep validation offline. |
| Operator-facing 2UQ2 capture recorded data but opened no image/AprilGrid preview, so the operator could not judge whether both lenses saw a usable target | 1 | Add D405-style latest-only stereo preview and AprilGrid feedback; keep preview off the lossless storage and XU hot paths and rerun 60/200 HIL. |
| First preview HIL opened and processed both halves but main-thread `imshow/waitKey` reduced IMU to 181.14 Hz | 1 | Differential runs isolated the GUI event pump: decode-only held 201.84 Hz, while GUI with tag detection disabled still reached only 186.17 Hz. Move HighGUI pumping to an independent preview thread and let the XU loop poll only an abort flag. |
| Threaded preview passed on an empty scene but fell to 179.68 Hz when real tag detections appeared; process+Queue variants fell to 174.91/160.93 Hz | 2–3 | Tag detection held the thread GIL; multiprocessing Queue still pickled large JPEGs in the producer. Replace both with a spawned preview process fed by a non-blocking two-slot shared-memory handoff. Final 10 s run restored 59.468/202.491 Hz with zero inferred drops. |
| After repeated capture cycles, both the exact preview snapshot and independent no-preview/rate-probe paths degraded to 168.86–176.61 Hz IMU; ordinary `usbreset` lacked permission and noninteractive sudo required a password | 1–2 | Classify current HIL as BLOCKED by device XU/USB state, not preview-specific. Require the operator to unplug/replug only the 2UQ2, then rerun the bounded rate/preview gate before any 120 s formal capture. |
| Physical replug restored the independent 200 Hz path, but the preview child spawned 24 OpenBLAS workers and intermittently reduced XU polling to 84–88 Hz | 1–3 | Runtime thread evidence and an `OPENBLAS_NUM_THREADS=1` differential HIL isolated the cause. Limit OpenBLAS only inside the preview child before NumPy/OpenCV imports; the ordinary command then passed transport at 59.456 Hz video / 202.251 Hz IMU with zero inferred drops. |
| First X5 write-path preflight reported 28.78 Hz retained video and 12 native sequence gaps | 1 | Index evidence localized one initial 235.856 ms gap (driver sequence 3→17) to sleeping through warm-up with V4L2 buffers filling. Change warm-up to actively query IMU and drain video, then require the operator to rerun the same 5 s gate before formal capture. |
| X5 changed IP during a 787 MB `scp`; the board reboot cleared the source dataset and updated tool from `/tmp` | 1 | Preserve the 531 MB local partial artifact, CRC-validate its 60.40 s/1813-frame prefix, and supersede X5 acquisition with the user-selected PC path. Never treat `/tmp` as durable evidence storage. |
| First PC stereo operator run stopped before creating its output with `preview_process_ready_timeout` | 1 | Preserve the absence of an output directory as evidence of clean pre-capture failure. Direct Qt/AprilGrid startup completed in 0.147 s and the exact preview child handshake completed in 0.226 s on immediate differential retest, so do not guess at a code fix; require a bounded full-stack preview preflight before retrying 90 s. |
| First target-visible PC preflight passed AprilGrid observability but failed IMU at 174.659 Hz / 14.71% inferred drops | 1 | Do not repeat the unchanged 0.75 s warm-up. The same run recovered to 201.788 Hz / 0.39% drops after second 10; test a single 10 s active-warm-up hypothesis before modifying source. |
| Temporary 10 s IMU warm-up test first hit the reader's 5 s ready timeout, then still failed at 182.230 Hz after extending only the test timeout | 1–2 | Warm-up length is not the root fix. Preserve `run2` as an empty pre-measurement manifest and `run3` as complete FAIL evidence; question the combined preview/IMU architecture and re-check the independent transport state. |
| Independent rate probe was first invoked with an unsupported `--output` flag | 1 | Parser exited before touching hardware; reran without that flag and retained the terminal evidence in planning files. |
| Kalibr container capability probe passed `bash` through the image's forced `rosrun kalibr` entrypoint and left a temporary container running | 1 | Identified the exact container by pinned image, stopped only that probe container, and repeated inspection with `--entrypoint /bin/bash`; no calibration data or external repository was modified. |
| Stereo solver completed optimization but failed only while creating its PDF graph because container Python lacks `cairo` | 1 | Preserve the valid camchain/results text and replace the unavailable PDF-only check with independent numerical/rectification evidence; do not repeat the identical command. |
| First camera–IMU solve allowed both hardware-synchronized cameras to estimate independent time offsets, yielding 8 px reprojection residuals and an impossible 11.8 ms inter-camera offset difference | 1 | Reject the result. Solve only the reference left camera `T_C0_I,td`, then derive `T_C1_I` through the accepted fixed stereo transform as required by the frozen frame model. |

## 2UQ2 Ego calibration and SLAM bring-up advisory

### Goal

Define an evidence-backed bench procedure for calibrating the 2UQ2 integrated stereo cameras and built-in XU IMU, then producing and accepting an Ego stereo-inertial SLAM trajectory. This phase is design/research only and must not modify the current acquisition implementation or any external D405/release tree.

### Phases

1. **completed** — Inventory the local 2UQ2 capture/XU contract and available calibration/SLAM tooling.
2. **completed** — Verify primary-source calibration requirements and choose the narrowest practical toolchain.
3. **completed** — Specify tomorrow's capture motions, artifacts, frame/time conventions, rejection gates, and rerun rules.
4. **completed** — Specify the first Ego SLAM replay and bench acceptance criteria without claiming HIL before hardware evidence exists.

## Live 2UQ2 Ego calibration at 60 Hz camera / 200 Hz IMU

### Goal

Read the connected 2UQ2 identity and any retrievable factory intrinsics, configure and measure native 60 Hz full-resolution stereo plus 200 Hz IMU without falsifying unsupported rates, then calibrate the stereo and camera–IMU extrinsics using D405-compatible explicit frame conventions. Write all new artifacts only in this repository.

### Safety and scope

- Bench/no-motion until an operator-guided target or handheld motion stage is explicitly started.
- `/home/robot/ego_vio_humble`, all D405 repositories, vendor SDK trees, and release directories remain read-only.
- Do not overwrite factory/device calibration or claim calibration PASS without a dataset, residuals, and validation evidence.
- Do not relabel one frame-linked XU sample as 200 Hz; separately prove internal ODR and host-exported timestamped sample rate.

### Phases

1. **completed** — Record host/revision/dirty state and enumerate the exact connected camera, UVC formats/controls, USB topology, and available XU/calibration interfaces.
2. **in_progress** — Attempt non-mutating factory intrinsic retrieval and preserve raw evidence; otherwise mark factory retrieval unavailable.
3. **superseded** — Earlier PC evidence expanded the backup group as a second temporal sample. X5 source/HIL evidence invalidated that interpretation: only bytes 3–14 are the main IMU sample; bytes 15–26 remain backup/diagnostic data.
4. **in_progress** — Provide and HIL-verify one concrete operator command that records immutable 60 Hz SBS plus 200 Hz IMU calibration data while showing a non-blocking left/right AprilGrid preview with live detection feedback; then capture or stage stereo target data and compute `K0,D0,K1,D1,T_C1_C0` with reprojection/rectification evidence.
5. **pending** — Capture or stage synchronized camera–IMU motion data and compute/validate `T_C0_I` and signed time shift, deriving the D405-style `T_I_C0/T_I_C1` convention.
6. **pending** — Freeze calibration artifacts and HIL report with PASS/FAIL/BLOCKED gates; run repository verification.

### 200 Hz IMU recovery subplan

1. **completed** — Benchmark the original PC bridge and record its approximately 100 calls/s behavior.
2. **completed** — Inspect the X5 implementation and HIL evidence: its hot loop issues one fixed-length 27-byte `GET_CUR`, observes 194–202 main-payload updates/s, and never expands the backup group.
3. **completed** — Identify the PC bottleneck: `ylx_read_imu27()` issues `GET_LEN` plus `GET_CUR` for every sample, doubling the approximately 4.8 ms USB control-transfer cost.
4. **in_progress** — Replace the obsolete PC temporal-pair path with one-time length validation plus one `GET_CUR` per main sample, publish only main bytes 3–14, then pass both 10 s and 90 s 60/200 HIL gates.

### X5 board calibration execution

1. **completed** — Inventory X5 ROS/Kalibr/capture capabilities and verify the current 2UQ2 identity, ownership, storage, and main-IMU timing path without changing deployed source or services.
2. **pending** — Select a board-native capture/export path that records stereo images and only the main IMU group with explicit host-monotonic timestamp provenance.
3. **superseded** — The bounded X5 preflight passed, but the 90 s run exposed sustained XU polling stalls and `/tmp` evidence was lost after a board reboot during transfer. User selected PC-direct acquisition instead.
4. **completed** — Salvaged a CRC-valid 60.40 s stereo prefix locally for diagnostic use only; it failed the frozen 60% AprilGrid visibility gate and will not replace a fresh PC dataset.

### Provisional acceptance gates

- Exact device and lens-order identity recorded; image resolution exactly 3840x1080 before split.
- Camera requested/measured at native 60 Hz ±5%, no timestamp regression, drop ratio ≤1% over the accepted window.
- IMU requested/measured 200 Hz ±5% with monotonic acquisition timestamps and drop ratio ≤1%; otherwise phase 3 is `BLOCKED`, never synthetically upsampled.
- Stereo mean reprojection error <0.5 px; rectified vertical correspondence residual p95 <1.0 px.
- Camera–IMU solution converges with physically valid SE(3), repeatable rotation, plausible lever arm, and independently replayed estimator motion; exact residual gates are frozen before collection based on the selected tool output.
- Default operator capture opens a left-half/right-half preview, outlines detected AprilGrid tags, and shows per-half tag counts. At least 60% of sampled preview frames must contain ≥4 tags in each half and each half must average ≥6 tags; this is an observability gate, not a substitute for Kalibr residual/held-out validation.
- Preview acceptance filters to unique expected IDs 0–35 and requires one consistent 6×6-grid planar homography before a frame counts as a valid grid observation; arbitrary t36h11 tags do not satisfy the gate.
- Transport acceptance records acquisition-interval inferred drop ratios (≤1%), maximum video jitter (≤8.334 ms), and maximum IMU jitter (≤2.5 ms), in addition to average rate and monotonicity.
- User-authorized temporary calibration stages do not weaken or rewrite the strict transport checks: `stereo` ignores IMU for its stage verdict, while `cam-imu-provisional` requires 170–210 Hz, inferred IMU drops ≤15%, maximum IMU acquisition interval ≤30 ms, zero XU errors/regressions, and all strict video checks. The latter must report `PROVISIONAL_PASS`, never final product PASS.
- The first PC camera-IMU run was intentionally stopped at 70.83 s because `--no-preview` and a silent capture loop provided no operator feedback. Preserve its hash-valid streams as diagnostic-only. Before run2, require immediate startup messages, 5 s progress lines, a low-rate live preview, and clean Ctrl-C finalization into an operator-aborted FAIL report.

### Offline Ego calibration solve (authorized 2026-08-28)

1. **completed** — Verified the operator-created deterministic 5 Hz stereo export, hashes, pair count, image contract, and disk.
2. **completed** — Created and inspected the pinned-container ROS1 stereo bag: 444 frames/topic, 888 messages, 89.916 s, indexed.
3. **completed / FAIL acceptance** — Solved `K0,D0,K1,D1,T_C1_C0`, but independent held-out rectification failed: p95 vertical residual 13.014 px versus 1.0 px. Do not release these stereo parameters.
4. **completed** — Exported run2 at 1,099 image pairs plus all 22,393 SI IMU rows and created an indexed 24,591-message ROS1 bag. Noise priors and their non-manufacturer provenance are explicit.
5. **completed / provisional numeric only** — Reference-camera solve produced plausible `T_C0_I` and -2.387 ms shift with 0.595 px mean reprojection; derived right-camera transforms are frozen, but cannot pass the full chain while stereo acceptance is FAIL.
6. **completed / overall FAIL** — Frozen solver outputs, logs, hashes, frame conventions, PDF, composed matrices, and `ego_calibration_provisional_acceptance.json`. A 30 s recording is diagnostic-only pending a better slow stereo calibration capture.
7. **completed / internal-training PASS with provisional transport** — Re-solved stereo from run2, passed a disjoint 511-frame rectification gate at 0.927 px p95, re-solved cam0-to-IMU at 0.584 px mean reprojection and -2.129 ms shift, and froze `ego_calibration_training_v1.yaml`. Final release remains blocked by strict IMU transport and independent camera-IMU repeatability.

## Image-level AprilTag detector/PnP replay (2026-08-28)

### Goal

Replace ideal AprilTag pose injection with a deterministic image-to-pose replay path: rendered `tag36h11` pixels are detected, solved with square IPPE PnP, quality-gated, and then passed into the existing Ego-world dual-UMI alignment layer.

### Constraints

- Keep all writes inside `/home/robot/three-device-slam`; external Ego/D405/release trees remain read-only.
- Preserve image acquisition time independently from detector completion time.
- Use the printed tag's 40 mm black-square size and left/right IDs 0/1 as explicit calibration inputs.
- The locally available OpenCV AprilTag dictionary is a deterministic reference/replay backend only. It must not be labelled as the official AprilRobotics production backend.
- Image replay may report simulation/replay acceptance only, never HIL or training-data readiness.

### Phases

1. **completed** — Inventory raw D435i/D405 stream contracts, available intrinsics, tag assets, and detector/PnP dependencies.
2. **completed** — Freeze camera-calibration, detection, IPPE-candidate, timestamp, and rejection schemas.
3. **completed** — Add red tests using projectively rendered official tag bitmaps with known camera/tag poses.
4. **completed** — Implement the narrow OpenCV reference detector/PnP path and feed accepted observations into the existing independent left/right alignment chains.
5. **completed** — Generate hash-bound image-level replay evidence and visibly distinguish detected, rejected, occluded, and reacquired states.
6. **completed** — Run focused/full verification and independent review before claiming completion.

### Success criteria

- Pixels, not injected poses, determine tag ID, corners, and `T_camera_tag`.
- Square-size scale, corner order, pose direction, positive depth, reprojection error, and planar IPPE ambiguity all have regression coverage.
- Unknown IDs and invalid/ambiguous candidates are rejected with auditable reasons.
- Detector latency cannot change the measurement timestamp.
- Both ID 0 and ID 1 independently initialize/reacquire their world anchors through the existing robust-window logic.
- The generated manifest states the detector backend and evidence class precisely and binds camera/tag/config/source hashes.

## D405 docker2 / simultaneous SLAM integration audit (2026-08-28)

### Goal

Determine whether the afternoon D405 `docker2` implementation is the left-UMI capture+SLAM chain, whether the owned three-device coordinator launches it together with Ego and right UMI, and whether their trajectories are actually fused into the Ego world frame rather than merely captured concurrently.

### Constraints

- This phase is read-only diagnosis unless the user explicitly asks for implementation after the gap is known.
- Keep `/home/robot/ego_vio_humble`, D405 repositories, Docker images, deployed services, and release trees unchanged.
- Distinguish four states: process launch, sensor capture, local SLAM trajectory, and Ego-world spatial fusion.

### Phases

1. **completed** — Located the afternoon device-2 D405 Docker tree/image, entry points, device binding, candidate calibration/runtime state, global ROS topics, fixed viewer port, and local odometry evidence.
2. **completed** — Traced the owned coordinator: it starts three synchronized raw-capture workers (or one shared rsusb owner), not the D405 Docker VINS/loop processes.
3. **completed** — Compared contracts. No owned live consumer connects Docker `/odometry_rect` to the AprilTag alignment layer; phase-one workers explicitly reject `--publish-vins`; Docker role/world alignment remains unassigned.
4. **completed** — Classified the present system: three-device simultaneous capture is implemented; one D405 candidate SLAM is proven independently; simultaneous namespaced Ego+dual-UMI SLAM and Ego-world fused trajectories are not yet implemented or HIL-verified.

### Success criteria

- Every claim is tied to a concrete file, command, topic, frame, or test.
- “Three processes start” is never described as “three trajectories fused.”
- Left/right device identity and Docker resource ownership cannot silently collide.
- Timestamp and `T_A_B` frame directions are explicit at the integration boundary.

## Mount calibration changed-view validation (2026-09-05)

1. **completed** — User confirmed Ego-only repositioning; preview closed, 20 seconds captured in two_tag_crosscheck_20260905T185830, external IDs 1/2, mount ID 1.
2. **completed** — All formal CRC valid; sampled geometry passes; crosscheck pass (2.069deg, z difference1.621mm).
3. **incomplete: physical repositioning required** — Mount change vs17:46 is3.399mm/0.206deg, within5mm/3deg, but view change35.8–38.1mm and4.40–4.57deg is below frozen50mm-or5deg condition. 17:37 has sufficient angle difference but changed external layout; not controlled Ego-only evidence. Full within-session mount chain separation8.074mm remains unresolved.
4. **completed** — Raw data, geometry, comparison, result and source/data hashes saved. No activation, commit, or release write.

Recovery note: initial plan patch expected a nonexistent heading and failed without changing files; append using verified context instead.
One diagnostic orchestration call had a surplus brace and did not execute; corrected call succeeded without source changes.

2026-09-05 19:50 follow-up completed: operator confirmed additional Ego movement. First startup failed at device enumeration while preview was still exiting (failed to set power state). Confirmed both processes exited before retrying in two_tag_crosscheck_20260905T195005_retry. Failure directory preserved; no hardware reset or gate changes.

Changed-view consistency phase now **completed**: versus accepted18:58 reference, both chains exceed12deg view change; mount consensus2.028mm/0.620deg, individual chains4.270mm/0.864deg and2.433mm/0.592deg, within frozen5mm/3deg. Both sessions pass original crosscheck and sampled geometry. Currentz=-46.460mm. Remaining **unresolved**: full within-session mount chain disagreement6.057mm and independent absolute physical accuracy. Stop repeated placement requests; retain all raw sessions and review cases. No calibration activation or deployment.

## Offline joint-corner diagnosis (operator approved)

1. **completed** — Extracted sampled raw corners with independent fixed identity bands and geometry counts; per-axis chain differences persist mainlyx (4.5–8.6mm), not fixedz.17:46 review retained.
2. **completed** — Joint fit implemented in mount_bundle_diagnostic.py, fixed intrinsics/40mm/colorIR. Primary17:46+18:58 training producesz=-49.593mm, NOT_ACTIVATED. All fits converge.
3. **completed** — Frozen primary holdout19:50 external-only nuisance fit predicts mount corners:11.204→5.011px vs prior46.541mm candidate. Auxiliary leave-one-out and training-only tag ablations reported. Absolute3D accuracy remains unresolved; no final acceptance.
4. **completed** —6 new regressions plus existing tests:20passed. Independent review important findings fixed; source/data evidence and handoff saved under joint_mount_diagnostic_20260905_v2. External repositories unchanged.

## Fixed-layout offline contrast

- Operator confirmed right-tag black square40x40mm and flat paper. Record as operator measurement, not newly instrumented metrology.
- **completed** — Fit shared external poses across17:46/18:58; register19:50 Ego using external corners and frozen trainingA. ValidationUMI external residual0.716/0.510px.
- **completed** — 2x2 score table: original5.011px, fixedX only5.395px, sharedM only7.187px, sharedM+fixedX7.168px. No improvement: reject as fix, retain negative result. New recovery/isolation tests and independent review pass.19:50 remains reused validation NOTfreshfinaltest; no hardware or activation.
- Initial test collection found Python3.10 rejects starred index expressions in np.r_; replaced with np.concatenate(list). No data were produced by the failed invocation.

## Flat-left-tag replacement measurement

- **completed** — User confirmed replacement;20s session flat_left_tag_20260905T205500 after confirmed previewexit.599/599formal frames.
- **completed** — CRC allvalid,32 sampled geometry frames/camera pass, originalcrosscheck1.583deg/1.263mm-z pass. Fullmount3D6.867mm vs old6.057mm: no improvement.
- **completed** — Frozenjoint-v2 new RMSE10.620px vs prior5.011; frozen46.541mm baseline13.279px vs11.204. No new M fitting. Left externalPnP median location changes69.44mm and imageedge shrinks69.4→60.9px; not wrinkle-only A/B. Save raw/report/hash, no activation or absolute accuracy claim.

## New-box layout recapture (operator requested)

- **completed** — Operator changed box and tagpositions. New20s capture599/599 frames, all poses re-estimated within current session. No previous external layout used.
- **completed: review** — CRCvalid; sampled Ego rightid2 angle45.49deg violates<45deg, geometryFAIL. Crosscheck0.899deg/6.159mm-z review; fullmount7.048mm. Mean49.689mmz unaccepted. Need physicalrightTag angle adjustment before next acceptance attempt, not altered solver thresholds.

## Right-tag adjustment recapture

- **completed** — Operator confirmed adjustment;previewPID3064843exit0;20s right_tag_adjusted_20260905T212910 captured600/600formal frames.
- **completed: review** — CRCvalid, angleimproved42.1degmax but spacing19.58–19.82cm<20cm. Crosscheck3.134degfailsrotation despitez0.067mmpass;fullmount15.858mm mainlyx. No acceptance/activation. Stop repeatedsmallplacementadjustments; consider larger external rigid target with explicit size configuration/approval as next independent observation.

## 80mm public tag printing

- **completed** — Operator approved. Leftid1/rightid2 black80mm+10mm white border generated, decoded, dimension-checked and printed CUPSjob11 completed21:33:28. Existingmountid1 stays40mm. No other print job submitted.
- **physical printing incomplete (operator correction)** — User received onlyone sheet despite job11completed. Both sourcePDFs validate; CUPSfilter warned damaged/xrefnotfound for multi-documentjob. AskreceivedID, then singlyreprint onlymissingID. No supplementaljob sent yet.
- **next required before any preview/solve** — Separate external80mm from mount40mm in detector/identity/crosscheck paths; do not simply set one globaltag-size80. SameID collision requires correct physical-instance classification. Current mixed-size algorithms NOTimplemented/verified.
- Handoff append initially used stale plan context; patch failed without edits and was reapplied with verified context.

## Mixed80mm external /40mm mount implementation and bench check

- **completed** — Operator confirmed placement after missingleftID1 reprint. Added isolated mixed-size detector/crosscheck and preview; olduniform40mm pipeline remains unchanged.
- Use common unique externalID2 to predict externalID1 imagecenter, discretely associate Ego's twoID1quads (not infer from incorrectly scaled distance). Then retain independently solved80mm external1/2 poses and40mm mount pose; association is not a pose-fitting constraint.
- **completed** — Synthetic swapped-range/identity ambiguity/scale/frame composition tests plus existing regressions:29passed. Independent review clock-domain finding fixed; re-review clear. Geometry>=60px/<45deg/spacing>=20cm, originalcrosscheck<=3deg/<=5mm-z plusfull3Dreported. Mixedpreview labels sizes and identity failures.
- **completed measurement / REVIEW** — mixed80_crosscheck_20260905T215930 captured600/600frames, CRC allpassed; stride1 yields598/600associatedpairs, allgeometrypass. Twochainsrotation0.608deg butz6.494mm >5mm, full3D6.540mm; meanz-51.204mm NOTaccepted. Quarterwisez6.41–6.68mm persistent. No activation/commit; camera processes stopped.
- **completed operator check** — User confirms80mm. Preserve confirmation; no size fitting or further placement requested.

## Frozen-image algorithm investigation

- **completed diagnostic** — No hardware or formal calibration changes. Fixed598samples baselineexactlyreproduced; subpixz6.218mm, APRILTAGfrontend4.523mm/full3D4.846mm, iterative6.349mm vsbaseline6.494mm. Fourquarterrefinedz4.50–4.54mm; allgeometry/positive-depth/errorgatespass. One-role sensitivity strongestEgoexternalcorners; notunique-rootcause/absoluteaccuracyproof.
- SDK inverse-Brown deprojection vs current OpenCV normalized rays measured0.004px max on first paired UMI corners, no evidence for a large sign reversal. Integer corners/no defaultrefinement identified as testable implementation choice, not yet proven cause.
- **completed** — Independentreview noCritical/Important; fullbaseline/sample/poseidentityanddepthchecks verified. report/corners/verificationSHA256/RESULT preserved in fixed_image_ablation_all_v1. ConfirmedSDKcolumnmajor mislabeledrowmajor; separate0.24mmtestpointdefect, NOTcause of6.5mmnormgap; noformalfixyet.
- **next proposed implementation** — Opt-in APRILTAGfrontend with regressiontests, provenance-aware SDKrotation handling; freshview verification before anymountactivation. Diagnostic task does not authorize silently replacingformalcalibration.

## Integrate approved corner frontend

- **completed** — Validated none/apriltag detectoroption; shareddefaultnone, mixedpreview/crosscheckdefaultapriltag, explicitoldfrontend switch; methodmetadata and segregatednonoverwritingreports. Allpose/identity/geometrygates unchanged, nofallback.
- **completed** — 83relatedtests passed, focused48passed; independentreviewnoCritical/Important. IntegratedrefinedCLI600pair replaypass598associated,z4.523/full3D4.846mm exactdiagnosticparity; none600pairaudit/chains/agreement exactoldparity. OldREVIEWreport unchanged; priorsources archivedbeforeediting. Previewhelp verified, nohardwareopeningthisturn.
- **next validation** — Freshindependentviewwithrefinedpreview/capture, beforeanymountactivation. Usageandlimits in docs/acceptance/mixed80-corner-refinement.md.
- SDKrotationlayout is a separate knownissue and remains unchanged thisfrontend-only integration; noformalMactivation or externalrepositorywrites/commits.

## Extra-tag identity and per-role preview

- **in_progress** — User requested automaticallydistinguishing extraID1; thenconfirmednewposition. Latestimage shows physicalspareID1behindleftbox, notscreen as previouslyassumed. Allow>=2EgoID1quads, locateexternal1throughuniqueID2crosscamera anchor, then accept exactlyone40mmmountcandidate withinbroad0.15mofestimatedUMIorigin. NoexactMprior/no nearest-candidatefallback; ambiguityfailsclosed.
- **completed** — Partialpreviewlabels aredisplayonly; extrasgraywhencompleteassociation.88relatedtests pass; reviewerfound rejectedtrue-mountcouldpromotenearspare, fixedbyrejectinganyunresolved40mmcandidate, added2regressions and re-reviewclear.598archivedacceptedposepairs preserveidentityandposes; broadmountprior49.47–50.80mm within150mm.
- **waiting_geometry** — Livepreview now93971, directorymixed80_identity_preview_20260905T224800. ExtraID1~460mmfromUMIorigin excluded, trueback~50mm accepted; fivecorrectroles. Egotwoexternals~49.7/48.4deg exceed45; askedlowerEgo, keepboxes/UMIfixed. Nocaptureorcalibrationactivation. Stoppreview/waitrelease beforecapture.

## Independent new-view check after frontend integration

- **completed** — Freshpreview geometry accepted; preview93971 exited0/released before20s capture mixed80_newview_20260905T225115.599/599formal frames, CRC/clock checks pass; all598associated pairs geometry pass.
- **completed, consistency only** — Refined rotation1.336deg/z2.237mm pass; full3D5.631mm unresolved. Across-view mean mount change4.441mm/0.321deg passes5mm/3deg repeatability, notabsoluteaccuracy. New legacy control is1.099deg/z1.340mm/full3D5.505mm: refinement is not universally better. UMI external2 pose changes40.171mm/11.148deg, so not controlled Ego-only movement. Reports/raw/source hashes and RESULT preserved, cameras stopped; noMactivation orcommits.

## SDK rotation storage correction (operator: continue)

- **completed implementation** — Reproduced3failed/1passed regressions beforefix. Sharedserializer now emits actualrowmajor with explicitv2/source;3writersfixed, legacyreaderexact2auditedhashes only; unknownSDK failsclosed, truelegacyrowmajor unchanged.
- **completed verification** —201targeted regressions passed in234.93s, including20layouttests; reviewlegacyreverseconvention finding fixed, re-reviewclear. Frozen598+598poses reproduceoldagreement exactly; correctedM shifts0.21–0.244mm/0.536deg, full3D4.846/5.631mm invariant. FullCLI599pairs replay finishedexit0,598associated/allgeometrypass, new_layout_v2directory; frozenauditandcorrectedagreement verified separately.
- **completed scope** — Reports/raw/sourcebefore copies preserved, prior source bytes reconstructed where needed and exactoldhashverified. Independentcode/diagnosticreviews noCritical/Important. No live acquisition, calibration activation, external repository mutation or commit. SDK defectfixed, remainingchainerror NOTfixed.

## Remaining chain residual investigation (operator: persist until cause/fix)

- **completed diagnostic** — OfficialAprilRobotics pinned b7c0ebe9aa20f82ec7a828579004f9e706bfecd9 built locally underartifacts only (noinstall).8digitalID/rotation fixtures verifyfixednativecornerorder.60fixedsamples/session: nativefull3D5.006/5.670mm vsOpenCV4.791/5.626mm; halfpixel-origin correction4.825/5.610mm, no meaningfulremoval. Noidentity/posefailures. Preserve negativeevidence; do notadoptper-sessionbestmethod.
- **completed offline checks** — External-only transfer persistentbias1.52–2.56px vs propagatedtemporalRMS0.15–0.17px. Full-patternmaskedECC improvesnewview butworsensold6.471/4.579mm, reject. External-onlyjointfit reducesmountrepeatabilityto2.252mm/0.228deg butfrozenoldM predictsnewmount3.651pxRMSE; no acceptance. Exploratoryintrinsicsfitting independentlyofchain shows unstableEgoK andD4052poses/16params=16residuals no redundancy (not byitselfproofofJacobianrankdeficiency). No K/sizecandidateactivation.
- **awaiting_operator_physical_evidence** — User asked persistuntilrootcause/fix. Current data cannot reliably separatecameramodel/printedtargetgeometry. Asked asynchronouslywhether previousflatAprilGridavailable; need independentmultiorientationcolorframes beforefurtherphysicalcalibration. Existingcamchainfilesare2UQ2, no matchingD435i/D405colorcamchainfound; cannotreusewrongcamera. Do not continueboxmicro-adjustments or claimallalgorithmsprovenright.
- Diagnostic errors retained: initialexternalfit adapter usedCameraCalibration whereK/Ddictneeded (fixedbeforeoutput); firstdenseECC backgroundmaskincorrect fortrimmedmountmargin (0/60accepted, retainednegativeartifact; separateinside-tag-maskvariant); D405free-Kcvcalibrate rejectsno-redundancydata (recorded, notsilentlybypassed).

## Independent AprilGrid color calibration acquisition

- **stopped by scope correction** — User reiterates factory intrinsics authoritative; no free-K/D solve authorized. Preview36999 stopped and exit0/release confirmed, zero images recorded. Proceed fixed factory-data usage audit: stream/resolution, SDK distortion rays, tag geometry, extrinsic direction; no new fitted calibration. Earlier active-preview states below are historical.

## Factory calibration usage audit 2026-09-06

## Fixed-factory board holdout diagnostic (operator approved)

- **Ego rotated180 attempt actually normal; repeat orientation step** —50imagesintegrityPASS/38previewgoodbutall38decodedlatticeorientationsnormal, not180.92327closed. Preserveasnormaladditionaldatawithoperator_reviewsidecar, nooriginalmetadatamutation. Need physicallyupside-downboardconfirmedbeforeR; do notadvanceUMIorcomparethisas180.

- **Ego normal captured/integrityPASS/coverageLIMITED** —50images/46previewgood, allhashes/shapes/timestamps verified; samefactoryK/D.41687closed. Firstclipretained; noheldoutaccuracyresultyet. NextEgo rotated180 preview92327 active at board_factory_ego_rot180_20260906T0028; boardrotation physicallypending, userRarms30s. Needrepeatnormalviewpoints, hideboardcopyonlaptop; laterUMIclips andpossiblynormalcoverage补拍.

- **preview_ready/waiting_operator_R** —23tests pass, reviewclear; firstEgo normalpreview41687 active at board_factory_ego_normal_20260906T002443,35/36tagsfresh, zero recorded. Eachclip30s manuallyarmed, samefactoryK/D preserved. Heldoutsolver implementation/validationpending; don'tclaimdiagnosisfrompreview. Stop/waitcamera releasebeforeothercapture.

- **in_progress** — Four separately armed30s clips: Ego normal/board rotated180, UMI normal/rotated180. Samephysicaloldboard6x6/35.2mm/10.56mm, noK/Dfit, noMactivation. Reuse preview-firstsampler with single-device/orientationlabels and timedposeguidance; freeze factorysnapshot and retainallsamples. StartEgo previewonly aftertests/review. Actualboardpose fitting/heldoutcode anddataanalysis followcapture; no successclaims fromduration/countalone.

- **completed audit and confirmed-defect fix** — Factoryvalues unchanged, modelenum nowpreserved and inverseBrown SDK geometry used byIPPE/reprojection/livepreview.94tests pass; independentreviewclear after2liveconstructorsfixed. FullCLI599/598associations/cornerssame, rawCRC/geometrypass, productionexactSDKdiagnosticagreement,25hashesverified. Remainingfull3D5.630870mm notresolved; noabsoluteacceptance oractivation. No furtherfactoryK/Dfitrequested/authorized; calibrationpreviewstoppedzeroimages.

- **in_progress** — Frozen598+598corner datasets, unchanged factoryJSON/K/D. Check actualcolor1280x720metadata and SDK projection/deprojection against OpenCV over fullsampledfield, not onlyfirstframe. Existing loader drops distortion_model; quantify impact before fixing and don't attribute5–6mm gap without evidence. Record newdiagnostic separately, preserve oldreports.

- **latest, supersedes earlier preview state** — Realboard two-bitborder/blackcornerjunction mismatch fixed in newpreview only (OpenCVmarkerBorderBits2+SUBPIX),50testspass; cameras from62577 and31173 stopped/released. Active toolsession36999 at color_aprilgrid_legacy_board_20260906T0012; freshlive36/36Ego,34/36UMI acceptedpreview, no recordingyet. UserinstructedR90s with45s variedposes/camera. Sameoldphysicalboard dimensions foundexplicitly35.2mm/10.56mminformalD405asset; priorlibrarycornerbuginferenceinvalidbecausefixtureborderincompatible. NoformalK/Mactivation.

- **in_progress** — Operator has rigidboard. Asked actual6x6/blacksize/gap verification (config35.2mm/10.56mm isnotmeasurement). Prepare separate preview-first, R-to-record90s dual-color PNG sampler at2Hz, explicitquality/progress/clockmetadata, noauto-solving orcalibrationactivation. This is independentcamera-calibration sampling, not30HzrawSLAM acceptance.
- **completed implementation/preview** — Added scripts/color_aprilgrid_capture.py;50 related tests pass, independent review clear after file-close exception cleanup fix. Real synthetic grid exposed old aprilgrid default resize missing33/36 and incorrect black-corner geometry; new entry uses OpenCV APRILTAG refinement and preview-only center-lattice check. Four rotation fixtures recognize36/36 with correct60px black edges. No old2UQ2 path modified.
- **waiting for physical board** — Preview active, toolsession62577, artifacts/spatial_bench/color_aprilgrid_20260906T000333. Both cameras opened/fresh preview verified; current scene still oldbox1/2+mount1, noAprilGrid, zero saved images. R starts90s, Q/ESC preservespartial/releases. Physical board dimensions unverified. Preserve allsampledimages includingqualityfailures; finalintrinsicacceptance requires variedposes/independentvalidation, notsamplecountalone. Stop ownedpreview and awaitrelease before anotherhardwarecapture.

## 2026-09-14 device3 short anchor HIL closure

- **completed / PASS** — Run24 recorded the planned 20 s Chinese setup countdown and
  45 s formal window. Pair transport, coordinator, 1350-row sync index, seal and offline
  verifier pass. Ego remained static and device3 supplied the required translation and
  rotation excitation.
- **completed / PASS** — Both offline VINS replays pass. The Ego export boundary bug was
  fixed by retaining the valid pre-window acceleration sample only as a bracketing
  interpolation support point; no extrapolation or warmup IMU publication occurs.
- **completed / PASS** — Fixed-AprilGrid HIL gives 79/79 initial and 17/17 final stationary
  consensus inliers. ID1 is absent for 2.400 s and reacquires. Initial-to-final device3
  world-anchor update is 7.910 mm/1.045 deg; the independent direct-board path is
  8.247 mm/.813 deg. Static mount-vs-board p95 is 3.229 mm/1.115 deg.
- Device3 right ID1 mount is accepted in
  `device3_right_world_mount_accepted_20260914.json`. Three-device group installation
  remains deferred until device2 has a verified physical left ID0 mount. Runtime must
  preserve the accepted anchor during fast rotation/occlusion and update only from a
  low-angular-rate multi-frame consensus.
- **completed cleanup** — After the accepted package hashes verified, removed five raw
  image payloads and two reproducible VINS input DB3 files. Released 16,338,059,264 bytes;
  all compact evidence, source indexes, IMU, VIO outputs and acceptance records remain.
