# Progress

- Final stronger OpenMP cap controls serial_assembly_a/b completedexit0, all900
  pairs6013IMU/889poses. INPUT900/SOLVE890/MARG867/rawCSV exactbetweenruns;
  angle2.047757deg remains. Notaccuracyfix; notdeployed. Bothownedchildrenexited.
  Final scene implementation hash matches guarded_replay_final; noROSstartedthere.

- Latest diagnostic runs b/c/d and serial_eigen_a/b complete; reproduced divergent
  trajectories with identical900featureinputhashes. FirstMARG611,nextSOLVE636;
  same8iteration stop, no timeout. OMP_NUM_THREADS1 didnotfix; no rootfix claimed.
  Source-copy loggingonly, externalreposreadonly. verify_diagnostics.py passes.
- Added hash-bound stereo scene gate and default pre-ROS block; all31samplesfail
  support. Explicitoverride preserveddiagnostic-only status inreal replays.
  Finalguarded_replay_final exits2 solver_startedfalse.75relatedtests pass3.22s.
  Independentreview noCritical/Important; twoMinor fixed/tested. Intermediate
  misplacedtestbody NameError corrected; failedrun2JUnit preserved.
- COVINS pinnedsource + read-only no-network temporaryNoetic container audit
  complete, missingmatcheddependencies. Installer neverrun, backendNOT_RUN.
  RuntimeprereqRESULT saved. ScenePNPfailed9<30support, not accuracy result.
  Detailed trace/limitations in ego_vins_diagnostic_20260906_run1/RESULT.md.

- Repeatability continuation: isolated2.1MBVINS source copy into
  artifacts/spatial_bench/ego_vins_diagnostic_20260906_run1; addedfeaturehash,
  Ceres costs/iterationstopreason andSchurprior spectra logging only, builtclean
  Humble/Ceres2/OpenCV4.5.4. Initialreplay_a failed missingintrospectionplugin;
  built exactlocalplugin, addedpreflightfilehashguard with9-test red/greenproof.
  replay_b ongoing. No external source modifications. Official COVINS pinned
  source cloned into artifacts/covins_runtime...; installer NOTexecuted because
  dependencies require matchedEigen3.3.4/OpenCV3/ROS1, not mixedhostHumble.
  Official README path lowercase; two guesseduppercaseURL404 andGitHubAPIfailed;
  gitclone pinnedHEAD succeeded. AnySearchoriginalCLIversionunverifiable, unused.

- Final run3 complete exit0: ownedROSchild exited,900backend frames/queue0,
 889poses30.014Hz,staticrotation1.189903deg vsrun2 2.400557deg (not fixed).
 Visual-only control1.586943deg also complete. Fullprelaunch runner/library/
 loader provenance now saved without overwriting oldruns. Added9runner tests,
 all56pass2.65s; finalincremental review no blocking findings. Saved independent
 verify_readback.py ranPASS1800exactimages6013exactgyro; source/outputhashesmatch.
 No calibrated sharedworld or robottraining accuracy claim. RESULT/architecture/
 handoff updated; no hardware, externalrepo changes or commits.

- Ego run2 completed:900backendframes queue0,889poses~30.014Hz, all900pairs6013IMU
  published. Provisional engineering gate passes butstaticorientation drifts2.4deg;
  after20s remainingrotation1.524deg, so not justinitialtransient. Original first/last
  LK118points median.249px/p95.753px; stereoimagesare distinct (medianhorizontal
  disparity63.68px). Next same-input offline stereo-only control explicitly labelled
  VISION_ONLY_CONTROL, not replacing requested internal-IMU production path.

- First Ego replay run1 FAIL after2images/15IMUs,0poses: exactlog says missing
  dlopen libvins_fusion_ros2__rosidl_typesupport_fastrtps_cpp.so. Library exists
  beside audited binary; ldd binary alone misses dynamic plugin lookup. This is
  wrapper environment failure, not camera/IMU loss or calibration evidence.
  Set child LD_LIBRARY_PATH to that exact binary build dir and hash plugin;
  preserve run1 failure, rerun unchanged sensors/config thresholds to run2.

- Ego internal-IMU continuation: exported Sep3run2 formal30s to
  artifacts/spatial_bench/ego_internal_vins_input_20260906_run1:900stereopairs,
  6013gyro/6059accel. All1800images pixel-exact and6013nativegyro verified.
  Factoryinverse givesbody_T_cam0 t[-.00552,.00510,.01174]m; gyroaccelaxesidentity.
  Reference noise only/td0 fixed/no ZUPT; NOT calibrated.8tests pass; reviewer
  confirmed SDK direction and fork/fullrate/CDR compatibility. Fixed parent ROS
  log path, deadline within waiting loop, competingpublisher check before replay.
  First bounded replay gate frozen: allsensorpublished,>=95%imageposecoverage,
  finite/unitquaternions,strictstamps,imagepairdelta<=1us,staticexcursion<=5cm/5deg.
  Domain92/localhost, ownedchildonly, nohardware; logs/output onlyrepo. Proceeding
  first replay to ego_internal_vins_probe_20260906_run1, max85s + boundedcleanup.

- 2026-09-06 user approves iPhUMI-style retained-hardware route: implemented
  covins_export.py and independent verify_covins_input.py, plus tests/docs.
  Real left source1284poses/1295images -> ROS1bag1284pairs; all decoded image pixels
  and rawbodyposes verified; pairingdelta max232ns numeric rounding. COVINS upstream
  pinned c5b180b443b59d2267a14584fa4b090429038698; no runtime installed or executed.
  Final39tests pass2.67s (junit artifacts/test_tmp/covins_export_20260906_run4.xml).
  Review fixed default pytest-root false positives, fault-specific errors, remaps;
  no outstanding important issues. Initial test invocation lacked parent tempdir,
  repaired environment without production change; preserved run1 then finalrun2.
  Historical D435i VINS found but its internal IMU is DISABLED, externalSTM32only;
  new Ego VIO still needs independent sensor/config contract. No hardware start,
  no outside repo writes/calibration activation/commit. Next gates documented in
  docs/architecture/ego-umi-covins-integration.md. Sharedworld remains NOT_RUN.

- Tilt run3 CLOSED exit2: A20/B20/C0, timeout WAIT_C; gain restore GET PIPE
  retained. Readonlylater AE1/8500/16/emitter0 originalvalues, no newcapture.
  partial_audit checked120PNG hashes and40triples; all36, staticmax.524px,
  tilt~24deg, mountcrossphase.108px. Onlysmallmount54.6–54.9px fails60px.
  A-only Bprediction direct1.957px, rigiddeclared2.454px, IR_Ascale2.847px:
  rigid prior is NOTfullfix. MissingC/cleanupfail neverwaived.43testspass;
  independent redecoding/120scores exact. All evidenceinrun3/RESULT.md.

- User-ready run3 toolsession35156 opened with unchanged tilt capture/gates.
  Run2 exited2 timeout0samples/cleanup[]/IRrestored. Current IR36/35–36, UMI35,
  WAIT_A; screenshot upperedge stillcropped, asked boarddown until whiteborder.
  Same300s bound, emitterOFF, nofactoryedits; poll35156 nextturn before restart.

- ACTIVE tilt run2 toolsession24822 after reviewer approved timeout separation.
  Bothpipelines1280x720@30 started, emitterOFFconfirmed. WAIT_A: IR36/36, UMI30,
  top row remains cropped. Preview open; prompted boarddown in UMI, noR.
  Max300s; poll same process nextturn, do not start another capture blindly.

- Tilt run1 stopped0samples, UMI30 because screenshot top row cropped; IR36/36,
  exposure31979/gain21/AE1. OriginalIR restored, cleanup[], no activecamera.
  Tilt-only framing wait now separated from30s convergence, outer300 unchanged;
  timer starts oncefirstall36fresh, noreset.37tests pass, oldmodes unaffected.
  Run2 planned, user prompted board DOWN in UMI view; no gate relaxation.

- 2026-09-06 user ready for tilt contrast: opt-in --tilt-contrast requires locked
  ABA; B>=12deg normals in eachview, no pure-translation trigger; Creturn same.
  Gate/CLI/offline-policy tests36pass; synthetic3view15deg recovered within.00003deg.
  Desktop/device no-motion preflight confirmed D435i/D405 identities, emitter0,
 185GiB available. No stream started pending pre-bench read-only review.
  Newoutputplanned ego_ir_umi_tilt_aba_20260906_run1, preflight.md records gates.

- 2026-09-06 lockedABA offline controls completed, no active hardware:
  A-only fair direct vs rigid known-board Bmedian3.042->1.010px/max1.089,
  C1.467->.348px; A-only joint same.35targetedtests passed (JUnit retained).
  Read-only review main180case/frame scores and no-UMI leakage reproduced.
  Prior AB replication B2.930->2.806px: not a general fix. Single IR routes
  locked B1.755/2.191px also fail. No activation, gate relaxation or factory edits.
  Reports/scripts: locked_aba_run2/model_diagnostic_v1; handoff§59.
  Unique physical root not established; next varied-tilt evidence suggested,
  current ABA mainly203mm translation/~1.4deg normal change. No new capture.

- Warmup follow-up safeguards: fresh frame age<=250ms/arrivalgap<=75ms before
  warmup convergence; unconditional prelock30s deadline even missingframes;
  drain2s clock taken AFTER lock/readbacks/setupwrite. New livewarm unit tests
  include finite supported metadata/convergence, returnC, persistent/transient
  GET faults.149 regressions passed9.90s before final3line timing guard;
  focused15 tests afterguard pass. Run1 failed0data retained, noparameteractivation.

- LockedABA run1 startup observation: actual8500us/gain16 lock and frame metadata
  verified but IR preview severely dark, gate0/0/UMI36. Agent interrupted zero
  samples, retainedpreview screenshot and metadata. Idle settings are not the
  converged runtime AE values. Change startup to auto warmup>=6s and live-frame
  exposure/gain stability>=1.5s +all36, then lock once before recording;30s
  warmup deadline inside300s. No mid-A/B/C auto exposure or gain adaptation.

- 2026-09-06 usercontinue: added opt-in continuous-aba (C=returnA) plus EgoIR
  original-option snapshot/manual lock, start/final readback, required actual
  exposure/gain frame metadata, optional explicit AE metadata, and best-effort
  original-setting restore on cleanup. Returnghost purple, perview median5/max10
  px gate and original static.75px/1.5s; allphases10s/max300s, noR/no restart.
  Read-only query Ego AE1/8500us/gain16; no stream started yet. Reviewer found
  restore exposure failure skippingAE; fixed tryeach+aggregate+negative test.
  Oldrun2 checker compatibility40 scores exactlysame. Focused13tests passed.

- 2026-09-06 continuous_run2 analysis complete:40frozen scores and120raw hashes
  reproduced; stable15B still2.914px, epipolar2.456px. Independent read-only
  review reproduced main analysis, no Important/Critical;22regressions pass.
  Added only artifact diagnostic scripts/reports and docs, no hardware/activation.
  Unresolved rootcause explicitly separated from observed crossview geometry
  inconsistency; recommended future lockedIR A-B-A, not autoexecuted. RESULT.md.

- Continuous_run2 completed A20/B20/120PNGs, cleanup[], frozen score FAIL.
  Offline analysis first run stopped on left-wall ROI x15 <24px NCC search
  margin; no report written. Corrected fixed ROI x32 before rerun, no raw edits.
  B samples31–35 static failure; other15 B frames still ~3px residual.

- 2026-09-06 operatorconfirmsno cameras moved/touched. Offline120rawhashverified,
  added20frameLK/NCC/epipolar diagnostic witholdwindowcontrol. Rawpatternchanges
  confirmed independentlyoftagcorners, physicalcauseunresolved. Correctedfailure
  labeltoprojection_changed, no gatewaiver. Alloldreports preserved. No livecalls.

- 2026-09-06 user"改变了": run6 automatic10s20triples60PNG cleanup[]; sourcehashes
  verified. Newfrozenchecker pinsrun5direct_meanall36T andcameraidentity, nofitX.
  CropUMI24–25/36, mountcrosssessiondrift1.7/2.5px, fixedpredictionmax12.302px.
  RetainFAIL andaskwhethercamerasalsotouched; noadditionalhardware/activation.

- 2026-09-06 userconfirmed10s; existingprobe --record-now run6 captures20triples,
  duration11.467includescleanup, emitter0, allstoppedcleanup[]. Addedjointstereo
  diagnostic raw/identity/static/triang/frozenK+boardonlyT checks; initialsmallmount
  60gate wronglyblockedboarddiag andmissingInfserialization, now explicitfailedsmall
  mountretention/nullmissingmetrics/separatestaticandsize, v3reportcomplete.
  MultidepthK4k1+T fromold4depthsets improvesrun6 heldoutboard.518/mount.918px butold
  run3/externaloldviewstillfail. Addedcomplete crosssessiongeomidentityguard afterreview,
  v3numericallysamev2. Allreportshashespreserved, noactivation/mainuncommitted.

- 2026-09-06 latest repair request: implemented pose_ablation, camera_recalibration,
  frozenexternalregression (offlineonly). All12rawpairs/all36 identity/sourcechecked.
  Reviewfoundreferenceframe/mountbindinghole, fixedwith2negative tests andv2replay.
  New6tests inclknownK recovery/heldoutpoison/candidatemount;114relatedpassed10.07s.
  DominantEgoside modelinconsistency localized; K4+k1 candidate usefulonnewboardbut
  externaloldview worsens9.826mm, so NOT_ACTIVATED. Originalsource/oldreport unchanged.
  Requested operator10s currentRGB+dualIR confirmation asynchronously; nohardware
  started/noRrequired, mainuncommitted/externalreposreadonly. SeePOSE_REPAIR_RESULT.md.

- CURRENT: user rejected unnecessaryR/16step forstaticanalysis. Ownedpreview26845
  stoppednormally; reportshowed2acceptedwindows already saved, so usedthemwithout
  newcapture/autotriggercode. cleanup=[],12pairs/24PNGs. Newstatic_analysis.py exact
  source/staticreplay, all35tags/branch alternatives and parityfolds. WholeMnorm
 56.519/57.016mm,z-49.355/-49.727,Δxyz.693704mm; per-smalltagchainssprawled,
  paritygroupΔM2.573/3.420mmremains. STATIC_DIAGNOSTIC only, notmetricPASS.
 105tests passed9.95s, independentreadonlyreview exactreproductionclear.
  CamerasSTOPPED; do notrequireuserR justtoanalyzesavedstaticframes.
  Evidence common_board_joint_20260906_run1/static_analysis/{report.json,RESULT.md}.

- ACTIVE latest user explicitly asked implementation, not another proposed plan.
  Added --simultaneous board+mount everyframe mode, fixed boardID1 exclusion upfront,
  identity checks, dualstaticwindow gates, magenta mount display, no removeboard
  instructions, loader mode/policybinding, same-frame frozenM validation.103related
  tests passed9.91s; independent review11checks/noCriticalImportant. Previouslive STOPPED
  execsession26845, common_board_joint_20260906_run1,30minbound,preview8Hztarget.
  Ego327122078613/UMI260322279785 color1280x720@30,emitter0. FirstRnotyetpressed at
  logpoint; user to placebothboard+mount inEgo, board inUMI. No calibrationPASS.
  LIVE verified both targets coexist: Ego35/UMI35/Mount1, multiple READY windows,
  latest maxdrift.53px <=.75,hostarrival.3ms(notexposuresync). Intervening.83/1.36px
  windows correctlyHOLD; not claimingcontinuousstaticPASS. PID3312286 nowSTOPPED.
  Old300s placement preview timedout cleanly,0attempts/no images; ownerreleased.

- PREVIOUS (STOPPED): bounded300s placement-only preview, exec session84894,
  artifacts/spatial_bench/common_board_placement_umi_detector_20260906_run1.
  Ego327122078613/UMI260322279785 color1280x720@30, preview target8Hz, emitter0.
  STEP1/3 mount: user must remove board, Ego sees mount, R locks placement;
  thereafter board->mount with both cameras fixed. No calibration collection/PASS.
  New umi-aprilgrid backend, fresh raw conflict display. Check process before owning
  cameras; Q/ESC/300s stop. Prior session8548 has STOPPED,0attempts,interrupt,clean.

- Same156 raw board images: opencv Ego mean27.026/IDchurn8.4, UMI20.949/3.738;
  native umi-aprilgrid adapter Ego36/0, UMI23.423/.10769,0errors/duplicate frames.
  Upstream default1000px downscale had1cv2border exception; adapter threshold2000
  keeps1280x720 native. Preserves upstream raw duplicate IDs before deduplication.
  53 focused tests and90 full related regressions passed; independent read-only
  review cleared boundedpreview.
  Reviewer caught staleREADY/currentframe mismatch; fixed and negative-tested.
  Same-support old-board solver contrast STOPPED atslot8left IDs17,29 absent:
  no pruning, no new residual claim. Original report SHA191f51... unchanged.
  New backend fixes demonstrated detection instability, not original5.631mm issue.

- User pointed to UMI calibration folder; read-only inspection found formal target
  YAML and identical released1.0.2 asset, compared numeric target to current config
  and raw capture successfully. Corrected prior redundant specs request and plan
  blocker (handoff§32 already had this fact). D405 stereo report refers rectifiedIR,
  no use of its649.2067px/zeroD as color656.2879px/nonzeroD calibration. No hardware,
  parameter or external-repository changes.

- User requested public-board geometry check: added only diagnostic scripts/evidence
  under artifacts/spatial_bench/common_board_distance_audit_20260906_run1. Original
  fullraw/static audit reproduced; fixed target/frame chain correct. 13window mount
  distance inconsistency and disjoint-tag projective-vs-rigid holdout quantified;
  five prior IR capture hash bindings and metric board length crosscheck recorded.
  46 related/new diagnostic tests passed. Independent review noCritical/Important.
  No production code/KD changes, no capture or activation, no commit. See RESULT.md.

- Final run5 verification:80 related tests passed in3.49s, independent review
  clear after policy-pair guard fix. Original common-board solve and frozen-fit
  hashes unchanged, git diff --check clean, no live capture processes. All work
  remains uncommitted on main; only opt-in offline board localizer changed.

- 2026-09-06 run5: authorized10s captured20 RGB/IR triples and clean stop.
  Near fixed5 corner defect reproduced with synthetic frontal/perspective tests;
  opt-in scale-aware offline path added, frozen default unchanged. Same-image
  paired comparison now binds exact corner/non-corner policies, support and frames.
  Actual new-policy input rejected by frozen validator before output creation.
  Same original common-board data replay completed: worst 4.391012 -> 4.389197 px,
  both REVIEW, fixed K/D/layout/IDs/split; original reports preserved. Near repair
  does not close original alignment failure; old k1+SE3 invalid at near depth.

- Authorizednearattempt run4:20triples/60PNGs, cleancleanup/emitteroff. Existing
  validator unchanged:20measured, primaryp95 .598379–.934444px, sensitivityworst
  .948278px. PixelboundmetbutoverallFAIL due15staticgatefailures,maxdrift
  RGB1.97865px. Depthmedian.63357m (perframevarieswithobservedtagIDs/tiltedplane;
  doNOTinterpretmedianrangeasboardaxialmotion). No subsetpromotion/refit.
  Setup/frozenmodel/originalrun4commonsolverreporthashesunchanged. No codeedits,
  nextrequiresphysicalboardstabilization andfreshcaptureapproval.

- Thirdposeauthorizedcapture run3 complete20triples;16valid withfrozendecoder,
  noID1/developmentframes. Newvalidator pinscandidateSHA andprimarybeforecapture,
  scoresallvalidcornerswithoutoptimization:FAIL1.03258–1.27574px,allstaticandIR
  timestamppass. FrozenT/K/Dunchanged. Independentreadonlyreviewerexactreplay.
  NativeIRphotometricpatchdiagnostic firstvalidframeeachpose, sizes11/19,
  zero searchboundhits, mincorrelation>.9969; thirdpose shiftabs95 .074/.099px,
  correctedRGBp951.168/1.158px stillabove1. Sourcecornerestimationnotuniqueroot.
  Frozenvalidation66relatedtests passed3.38s before3newNCCtests; noactivation.

- Finalverification61relatedtests passed3.37s; originalrun4SHAunchanged and
  diffcheckclean. Readonlyreviewer independentlymatchedfrozenpredictionand
  exploratorytwo-posefit; addedrejectduplicatesources. Thirdposeprimaryfrozen
  to k1/1e-5 case;2e-5sensitivityonly, p95<=1px/allvalidframes, >=3frameswith
  >=8commontags/drift<=.75px. k1SE3Zdelta+8.86mm mayabsorbIRbias, notmetrology.

- Secondposeauthorizedcapture completed20triples under run2, cleanstop/emitteroff.
  Initialvalidator stopped onduplicateID1 (backmountuncovered), beforeoutputcreation.
  Addedexplicitidentity exclusion andper-sessiondevelopmentlabel; run1andrun2
  replayuniformlyexcludeID1. Newcrosspose checker validatesrawhashes andsame
  factoryprofiles, freezespriorSE3; factory-local2.942px/k1+local1.757px FAIL1px.
  Priorreviewer independentlyreproducedallresults, confirmedrun1noid1unchanged
  andnotrainleakage. Addedrequestedderivedsample/metadata/pointcountguards.
  Firstcrosschecker invocationmissinglegacyedge_corners field fixedbydocumented
  opt-infalselegacydefault; nooutputwascreatedbyfailedinvocation.
  Newtwo-poseexploratorySE3fit (fixedK/D; firstframe/evenIDs eachposeonly) yields
  k1oddheldout[.664,.256]px and2xstep[.667,.254], versusfactory[1.612,.765].
  EXPLORATORY_REQUIRES_THIRD_POSE, notcalibrationPASS. Newindependentposepending
  operatorpermission/repositioning; camerasstopped, allfactory/runtimefilesintact.

- Latest hardware supplement authorized by user: manufacturer rigidboard,
  EgoRGB+IRleft/right10s,20triples/60PNGs, emitteroff and cleanupempty.
  Newprobe/validator metricIRtriangulation doesnotuse boarddimensions/planarPnP.
  Baseline0valid; IRdiagnostic threshold+3xdecode nativecornerrefine produces10
  validframes, excludesdevelopmentframe5 from frozenanalysis.9remaining RGBp95
  2.044–2.179px; IR .063–.096px. CornerdriftRGBmax.475px, IRmax.128px.
  SameIDs edge-localization variant doesnotremoveRGBbias (2.28–2.67px incldev).
  Independentreview caught SDKfloat32 defaultjacobian issue in NEW localSE3
  diagnostic; explicitcentral1e-5/2e-5 fixes it and reverses localfitranking.
  Corrected localheldoutworstfactory1.091/1.090px, k1 .779/.783px. Original
  frozen_candidate_analysis remainspreserved but itslocalfitnumbers superseded
  by _v2 and _step2. Fixed-extrinsic comparison unaffected: k1worse4.314–5.403px.
  No productioncalibration/deviceflash/release changes or adoption. Singlepose
  cannotseparateintrinsics/extrinsics/IRsystematics; nextneedschangedboardpose.

- Coupled K/D finished: Ego focal+k1 worstpairedholdout1.935px, UMI4.121px;
  EgoK4+D5 worst1.030px butk3boundhit; UMI4.984px. FrozenEgo focal+k1 tested
  originalexternalsetup4.845→8.572mm (worse), changedview5.631→3.482mm.
  Thus no coupledmodel adopted. These were offline diagnosticcopies only.

- Independent edge diagnostic completed: worstholdout corner5/factory4.391,
  edge/factory4.814, corner5/frozenk1 1.782, edge/frozenk1 2.043px. Not a
  cornerSubPix-only repair. Reviewer noCritical/Important; added single-session
  guard and comparison-source/K/k1 binding checks. Coupled K/D diagnostic now
  testing focal+k1 and K4+D5 per camera with frozen same bounds/splits; synthetic
  coupled SDK recovery included. Candidate fitting still not deployment.

- Continuing: D5/k1 diagnostic, prior mono clips, and separateexternal-tag check
  completed, none justify deployment. Ego k1 improves allpairedholdouts but original
  externalsetup full3D4.845->9.119mm worsens; changedview5.631->3.581 improves.
  Raw CRCs/calibration hashes verified; fixedcorners & identities acrossarms,
  posegate rejections0. Prior mono supportguard added after reviewer minorfinding.
  Independent gradient-edge corner intersection diagnostic now running onrun4
  with factoryK/D and frozenEgo k1, same13middlepairs/IDs, no pruning. Synthetic
  blurredsquare localization test passes. No device/runtime changes or acceptance.

- Comparison finished: run2 derivative-step2x replay all5cases still fail; maximum
  p95 change .003392px vsrun1, candidate Kentry .019101px.53finalregressiontests
  passed5.39s; sourcehashes/currentcodebindings verified and originalrun4reportSHA
  unchanged; git diff --check clean. Newdiagnostic only, no production/device
  writes or activation. Reviewer minorcoverage/objective/provenance addressed.

- User authorized offline K comparison. New standalone diagnostic script and
  tests, no production changes. Initial absent-module red confirmed; synthetic
  focal recovery/holdout-isolation tests passed. First5cases finished: maxholdout
  p95 factory4.391px/Ego-focal5.859/UMI-focal4.124/Ego-K4 6.472/UMI-K4 4.848;
  none satisfy1px. Independent reviewer noCritical/Important; minor documentation
  and provenance fixed, inverseBrown SDK K4 recovery test added/passed (3tests).
  Candidate termination optimality2.45–25.51 noted, not declared stationaryoptima.
  A2x K-derivative-step replay is running in comparison_run2 to test sensitivity.

- Run4 bounded diagnosis continued after user frustration; no code/device changes.
  Five readonly calculations exited0: temporal same-ID subset X; background LK;
  train8 essential-only fit; same-objective dense least squares; triangulated
  board geometry. Findings detailed in docs/handoff_spatial_alignment_20260905.md
  section37. Dense optimizer agrees within .000656mm; no fix justified yet.
  No camera started, no K/D fit, no activation. Proposed offline intrinsics
  diagnostic needs explicit agreement with prior factory-authoritative policy.

- Run4 follow-on: checked both IPPE/LM seeds on all26selected camera/board
  observations; same-pose convergence or clearly worse source-only alternative,
  no near-tied ambiguity found. Both readonly diagnostic commands exited0.
  git diff --check clean, main/HEAD unchanged. Overall calibration remains REVIEW;
  next diagnosis is systematic/model-vs-pose uncertainty, not branch flipping.

- 2026-09-06 run4 complete: verified all288 PNGs/24attempts and source bindings;
  solver finished exit2 REVIEW at common_board_train_20260906_run4/solve/report.json.
  Every mount-window p95<0.425px but board fails, so mount is NOT accepted.
  Ego->UMI heldout p95 2.377–4.391px; reverse0.892–1.421px; step6 duplicates7.
  Independent per-view fits use unchanged SDK K/D: Ego RMS0.317–0.492px,
  UMI0.157–0.283px across13boardwindows. Median tag edges perwindow Ego38–42px,
  UMI55–69px. Inspected attempt016 middle raw pair: no obvious motion smear,
  UMI upper board cropped, tabletop bright. This is not proof of K/D error,
  board warp, or camera movement. No camera start/code change/activation/commit.
  Run2 cannot resume because user confirmed moving both cameras; new run4 is
  the complete fresh setup. Continuing offline diagnosis, not requesting recapture.

- Current follow-up: run2 stopped atstep14. Same physicalID1 decoded atboth
  border sizes caused false remove-board warning; v3 uses exactID+quad-overlap
  duplicate exclusion, otherboarddetections still block. Actual attempt24/25
  12frames re-detected withoutfalsealarm, raw remainsunchanged. Added hash-bound
  readonly-parent continuation after12boardsteps requiring --confirm-unchanged;
  legacyv2 decisions preserved, not relabeled.94regressiontests pass; review clear.
  USER reports movement between board/mount stages; clarification asked. Do NOT
  resumehardware/solveM until movement object resolved. No camera process remains.

- 2026-09-06 visibility follow-up fixed: raw run1 checked all36PNGhashes and
  setup/index binding;3firststepattempts failed exactIDsets though maxdrift<.45px.
  New capture schema v2 uses firstframe overlap>=12/3x3 + all reobserved tag
  drift<=.75px unchanged, including late-appearing IDs. Middlepair solver selects
  only independently reobserved common IDs, quality persisted/rechecked. Added
  actual readiness/anchorcounts/drift + Chinese retry/actions, no auto recording.
  Development replay report common_board_visibility_replay_20260906_run1 shows
  all3 static checks pass; originalv1FAIL preserved, no calibration activation.
  84regressiontests pass incl9newfaultcases; readonlyreview noCritical/Important.
  No camera started, no external writes or commits. Next user retrynew run2.

- 2026-09-06 approved common-board route implemented (software only): new
  scripts/common_board_capture.py and scripts/common_board_calibration.py;
  16 guided steps, raw/hashes/rejections preserved, fixed SDK K/D/board joint X,
  whole-window bidirectional holdout, mount independent of X, fixed-reference M
  independent setup validation, final board closure. 75 focused/regression tests
  pass (22 new workflow tests); reviewer clear after red/green correction for
  duplicate-pose holdout. No camera started; no parameter activation/commits or
  external writes. Original 5.630870mm discrepancy still unresolved pending NEW
  paired-static hardware data. Operator command and gates:
  docs/acceptance/common-board-calibration-workflow.md. Next: arrange fixed Ego/UMI
  with common board, run preview, R each step. Do not reuse prior monocular clips.

- Open-source re-review per user completed: source-level apriltag_ros all-cornerbundlePnP, multical fixedintrinsics BA and separatevalidation, TagSLAMbody/time/identity restrictions, IPPE ambiguitylimits. Compared preexistingfailedBA so nofalse"justaddBA"claim. Wrote proposed dense-commonboard/pair-static/XthenM/heldout route, world remainsEgo andcameraXonlytemporary. No newcapture/runtimecodechange. AnySearchCLIhashmismatch preventedexecution; publicofficialweb/raw worked; GitHubtreeAPI403 not retried; emptyguessedwikiurls not used.

- Final verification: baseline_v2 tool3396 exit0; all session data/results exactly equal preserved run1. Current code hashes for baseline_v2/refine5 and bound source/index/report hashes verified; comparison199/286folds exact.33tests passed and git diff --check clean. No owned livecamera process remains. Diagnostic correction completed, original1-bit spatial disagreement not claimed fixed; latest continuation in handoff§33/task_plan.

- Completed paired board-diagnostic confirmation: refine5 replay9918 exit0; all4 clips RMS0.346/0.420/0.269/0.252px including development frame. Explicit comparison_199 excludes Ego-normal sample0:199raw images accounted,286 usable folds ALL improve, identical sourcehash/ID/train-held splits/statuses; RMS0.3475/0.4195/0.2694/0.2516px.33tests pass0.19s including comparison tamper/development-exclusion tests. Independent reviewer confirmed noCritical/Important and correct no-global-one-bit inference. Updated handoff§33 and acceptance doc. Baseline_v2 reproducibility replay3396 ongoing; no camera process. Original spatial full3D5.630870mm remains unresolved, NOT_ACTIVATED.

- Fixed-factory heldout solver implemented in scripts/fixed_factory_board_diagnostic.py,6 initial tests pass after expected absent-module red; independent review noCritical/Important. Fourclip baseline completed (92/78/65/53 OK folds),original report retained. Found narrow oldboard corner refinement issue; added geometry-specific synthetic redtest (missinghelper) then explicit5px diagnostic-only candidate,30combined tests pass. Refined4clip replay tool9918 running at fixed_factory_board_prediction_20260906_refine5; cameras allclosed. Firstdevelopmentframe excluded from confirmation comparisons. One failed unrelated read: aprilgrid package found in user site not /opt venvsite; source read-only. No K/D/Mactivation or commits.

- 2026-09-06 final UMI180 clip `board_factory_umi_rot180_20260906T0040` completed normally; tool68813 exit0/released, no camera preview remains.50 PNG/hash/setup/shape/monotonic checks PASS; factory snapshot exactly same as UMI normal;27 preview-good all physically rotated,23 rejects retained. Saved endpoints29.39165s, intentional1.66714Hz. Sample20 shows some lower-corner gripper occlusion; coverage one center cell only. Four intended clips now have200raw images /146 preview-good (Ego46+39, UMI34+27); extra falsely-labeled Ego180 excluded. No metric acceptance; implementing fixed-factory held-out board prediction next.

- Live continuation 00:40: UMI-only rotated180 preview tool68813 active at `artifacts/spatial_bench/board_factory_umi_rot180_20260906T0040`; starts preview-only, R arms30s. Need actual board180 orientation/full visibility check before recording; no concurrent camera owner. Previous tool14541 released.

- 2026-09-06 00:39: UMI normal `board_factory_umi_normal_20260906T003607` completed normally; tool14541 exit0 and camera released. All50 PNG/hash/setup/shape/timestamp checks PASS;34 preview-good frames independently re-detected all normal direction,16 rejected retained. Endpoint29.41223s, saved1.66597Hz/max0.61221s intentional subsampling. Visual20 normal/full grid; coverage limited2center cells. Saved operator_review.json; held-out metric analysis pending, no K/D/M activation. Next UMI rotated180 preview, R manually arms30s after physical orientation/visibility check.

- 2026-09-06 00:36: Ego rotated180 repeat `board_factory_ego_rot180_run2_20260906T0030` completed; tool60179 exited0 and camera released. All50 PNG/hash/setup/shape/timestamp checks pass, factory K/D identical;39 preview-good images all have physically rotated lattice direction (0 normal). Raw rejects retained; saved29.40254s/1.66652Hz intentional subsampling. Visual sample20 confirms upside-down board. Added operator_review.json; no metric accuracy acceptance. Opened UMI-only normal preview tool14541 at `board_factory_umi_normal_20260906T003607`, R manually arms30s; verify board restored normal/full visibility first. This supersedes prior live60179 entry.

- Livecontinuation: newEgo180run2previewtool60179active at artifacts/spatial_bench/board_factory_ego_rot180_run2_20260906T0030, previewonly/Rmanuallyarms30s. UseraskednotpressRuntilboardphysicallyinvertedandvisuallyconfirmed. OnlyEgoopen; stop/waitreleasebeforeanyothercapture.

- 2026-09-06 00:30 requestedEgo180clip92327exit0/released,50images/hashbindings/shapes/timestampsallvalid,samefactoryK/D;38previewgood. BUT actualboardremainednormal: all38gooddecodedlattices columnsright/rowsup, visually0/20/49sameorientationasfirstclip. Capturelabelrotated180isoperatorrequestonly, notactualproof. Savedoperator_review.json FAILrequested_orientation, rawlabel/datauntouched; retainassupplementalnormaldata, EXCLUDEfrom180comparison. Askeduserturnwholeboardupside-down(ID0upperright/ID35lowerleft), doNOTpressRbeforevisualconfirmation. NoUMIpreviewyet.

- 2026-09-06 00:28 Ego normal clipcomplete:41687exit0/released.50PNGs allhash/shape/setupbindings valid; framecounter/SDKtime/arrival strictlyincrease;29.3926s between savedends,1.667Hzsaved/0.6011smaxinterval (intentionalsamplingnotSLAMdroprate). Factoryintrinsics exactlyequalpriorEgo colorJSON.46previewgood;4zero-tagframes24/37/38/39, allretained. Inspected0/20/40/49: mainlysameearlyview, centersmiddle/right, limitedcoverage; laptopboardcopyvisiblelate, advisedhide(notprovenfailurecause). Savedoperator_review.json. OpenednextEgo rotated180 previewtool92327 at board_factory_ego_rot180_20260906T0028, R30s onlyafterboardturned; noautostart/KDfit. More normalcoverage maybeneeded; noanalysisacceptanceyet.

- 2026-09-06 00:25 fixed-factory board diagnostic approved: extended existing sampler withsingle-role/orientation, timed8poseguidance, per-frame/setuphash binding, explicitNEVER FIT K/D andqualityreview-before-next.23tests pass0.15s, review noCritical/Important. FirstEgo normal preview nowtoolsession41687 at artifacts/spatial_bench/board_factory_ego_normal_20260906T002443; inspectedfreshpreview35/36tags/min41.19px, recordingfalse/count0. OnlyEgo open, R starts30s. LaterEgo rotated180,UMI normal/rotated180 separateclips afterreview. NoK/Dfit orMactivation; heldoutanalysis stilltoimplement/verify beforediagnosis. See docs/acceptance/fixed-factory-board-diagnostic.md. Minor: coverage_cells stillcontains emptyunopenedleft key in single-ego preview; actualcounts/setup.devices/streams onlyego, noUMIopened.

- 2026-09-06 factory-modelfix complete: FullnewviewCLI96249 exit0,599attempts/598associated/geometrypass, allCRCpass. verification.json confirms598exactcorner/IDassociations and SDKdiagnostic/productionagreement exact,25hashes. Remainingfull3D5.630870mm unresolved; noK/D/Mactivation.94relatedtestspass, review/refollowupclear. Previewsallstopped, nofreeintrinsiccapture. Currentdocs docs/acceptance/factory-camera-model.md; handoff§32. No commits/externalreposwrites.

- 2026-09-06 factory-authoritative audit: User declined freeK/D recalibration; preview36999 exit0/released, startedfalse/counts0. CapturedfactoryJSON exactunchanged, color1280x720metadata all2398formalframes match across2sessions. Frozen598+598corners SDKrays vs priorOpenCV maxactualUMI0.0041px, posechange<0.0018mm, fullchain4.845943→4.845308mm/5.630932→5.630870mm; notremaining5mmrootcause. Foundloader dropsdistortion_model;9newregressions plus85relatedtests total94pass afterpreservingmodel, SDKpuregeometryinverseBrownPnP/reprojection, unknownmodelfailclosed, live2constructorfix. Reviewer initialImportantliveomissionfixed/re-reviewpending. Full599frameCLIreplay96249 ongoing into_factory_model_v3; oldreports notoverwritten. Auditartifact factory_usage_audit_20260906 andpostfix, source_before archived; noK/D/Mactivation. Testdirectoryversionexpectedv2failedonce (91pass/1fail), expectationupdated tov3; patchcontext mismatchfailedatomicallythenfixed.

- 2026-09-06 physical board preview: Actual old Kalibr board uses2-bit black borders plus black corner-junction pattern, not earlier1-bit synthetic fixtures. Reproduced4 failing synthetic2-bit tests; markerBorderBits2 fixes decoding but APRILTAG quad segmentation still misses most actual tags. Same archived preview contour/SUBPIX30tags vs APRILTAG8; changed only this calibration preview to2-bit+SUBPIX.50 tests pass; restarted owners sequentially and verified fresh live Ego36/36, UMI34/36, green lattice/size checks. Tool session36999 active at artifacts/spatial_bench/color_aprilgrid_legacy_board_20260906T0012, recording=false/saved0. User instructed R then45s each camera variedposes. Oldcamera sessions62577/31173 exited0/released. Prior aprilgrid synthetic73px observation was incompatibleborder1fixture versuslibraryborder2, NOT proof librarywrong forrealboard. D405 formal asset explicitly maps physical6x6 to35.2mm/10.56mm; reuse sameboard peruser, no repeatmeasurement demanded. No calibrationactivation.

- 2026-09-06 00:03: Operator has flat board. Added preview-first dual RealSense color PNG sampler, R starts90s, Q/ESC preservespartial;50 related tests pass and independent review clear after close-failure camera-release fix. Actual detector smokecheck found old aprilgrid downscale misses33/36 and black-corner size mismatch; new entry uses tested OpenCV36h11 APRILTAG refinement, four rotations36/36. Preview toolsession62577 active at artifacts/spatial_bench/color_aprilgrid_20260906T000333; both fresh streams verified, scene still box tags and mount, no grid/recording. Actual board dimensions pending; no K/M activation. Stop ownedpreview before anothercamera process.

- 2026-08-27: Started read-only D405/open-source research and deterministic spatial-alignment simulation on `main` at `73b1d4f74b2d53914696cc6e160462b2313e6120`.
- 2026-08-27: Inspected D405 product coordinate and timing contracts read-only; detected and preserved pre-existing dirty state in the external release tree.
- 2026-08-27: Reviewed HoMMI/MoF, Stanford UMI, and iPhUMI read-only; extracted explicit overlap/resampling, named-frame composition, repeated offset measurement, and independent-device topology patterns.
- 2026-08-27: Locked the simulation contract for D405 camera/body conversion, Ego gravity-aligned origin, arbitrary UMI VIO anchoring, delayed timestamp correction, and independent future UMI chains.
- 2026-08-27: Added contract-first spatial simulation tests. Initial pytest collection exposed a ROS `launch_testing` import-path issue; no production implementation exists yet.
- 2026-08-27: Confirmed the intended red test (`three_device_slam.spatial` absent), then implemented strict SE(3), timestamp-aware alignment, and deterministic simulation modules. Focused suite: 6 passed.
- 2026-08-27: Dual virtual-UMI run returned `PASS/simulation`; a 60 ms uncorrected delay caused 11.54 mm / 19.46 mm translation RMSE, while explicit correction recovered numerical closure.
- 2026-08-27: Documented the complete two-UMI-plus-Ego design, data model, timeline, fault injection matrix, staged acceptance gates, and one-click three-device boundary.
- 2026-08-27: Corrected the product topology after user clarification: simulation and CLI now default to Ego + left UMI + right UMI; single-side execution remains diagnostic-only.
- 2026-08-27: Re-read HoMMI v2, iPhUMI iOS collaboration code, offline aligner, and camera-to-TCP conversion after user correction. Corrected the durable design: the open-source three-iPhone system shares an ARKit world during capture; our top tags are a D405/2UQ2 substitute anchor, not HoMMI's original world-sharing algorithm.
- 2026-08-27: Generated official AprilRobotics tag36h11 ID 0 (left) and ID 1 (right) assets with a 40.0 mm black square, decoded both from the rendered A4 PDF, and printed one unscaled high-quality A4 copy as CUPS job `Brother_DCP_B7530DN_series_USB-5`. Physical paper measurement remains pending.
- 2026-08-27: Added direct off-grid acquisition-time coverage for linear translation and SO(3) rotation interpolation.
- 2026-08-27: Independent read-only review found no blocker and two medium pre-HIL risks. Added mandatory `max_interpolation_gap_ns`, explicit calibrated `forward_axis_in_ego`, signed negative-delay coverage, rotation round-trip evidence, and left-chain isolation checks. Focused suite: 10 passed.
- 2026-08-27: Independent re-review confirmed the fixes and reported zero remaining blocking or medium-risk findings; deferred real-calibration/reset/outlier items remain explicitly HIL-scoped.
- 2026-08-27: Final repository verification passed: `git diff --check`, Python compile check, deterministic dual-UMI `PASS/simulation`, and 640 pytest tests in 337.34 s.
- 2026-08-27: Corrected a device-ownership error after user feedback: `/home/robot/ego_vio_humble` is D405/external-IMU material, while the actual 2UQ2 Ego adapter is `three_device_slam/devices/two_uq2`. Confirmed the owned Ego contract requests side-by-side 3840x1080 MJPEG at 60 Hz and reads a frame-linked 27-byte XU main/backup IMU packet.
- 2026-08-27: Started a design-only 2UQ2 calibration/SLAM bring-up advisory. Confirmed pinned Kalibr camera and camera-IMU wrappers are installed, reviewed official Kalibr/OpenVINS procedures, and identified the XU sample-rate/frame-phase measurement as tomorrow's first hardware gate. No implementation or external repository was modified.
- 2026-08-27: Finished the executable bench-procedure design: raw-contract gate, rigid Aprilgrid stereo calibration, main/backup IMU screening and long stationary noise characterization, smooth six-axis camera-IMU calibration, explicit Kalibr-to-estimator frame/time conversion, and first offline Ego SLAM replay acceptance. Repository source code remains unchanged.
- 2026-08-27: User connected the 2UQ2 and authorized live calibration work at 30 Hz camera / 200 Hz IMU. Started a bench/no-motion hardware phase; external Ego/D405/vendor/release trees remain read-only and all artifacts are scoped to this repository on `main`.
- 2026-08-27: Enumerated the connected 2UQ2 (`1bcf:0b15`) separately from the D435i, recorded its USB 2.0 topology and advertised V4L2 modes, and confirmed the read-only D405 reference uses two independent Kalibr AprilGrid stereo–IMU attempts. No external file was changed.
- 2026-08-27: Probed XU controls read-only and measured the current data path. Full-resolution 30 Hz negotiation failed; full-resolution 60 Hz succeeded. Unit-3 selector-1 IMU packets advance only with video at roughly frame rate, so the current interface does not expose a true 200 Hz stream. Captured one occluded diagnostic SBS frame inside this repository.
- 2026-08-27: AnySearch did not return the requested GUID/selector results and emitted auto-generated credentials; they were not saved or reused. Continuing with public official/code search.
- 2026-08-27: Added an owned copy of the measured D405 AprilGrid geometry (`6x6`, black tag 35.2 mm, spacing ratio 0.3) for the 2UQ2 calibration command. No D405/external file was modified.
- 2026-08-27: Diagnosed the user's Kalibr traceback: `SESSION` was a literal placeholder and no `ego_stereo.bag`/camchain/IMU YAML exists yet. Only the diagnostic SBS JPEG and target YAML exist; the rosdep warning was not causal.
- 2026-08-27: Corrected an overstatement after user challenge: live tests disprove 200 Hz output only for the supplied selector-1 current-frame API, not the physical IMU's internal ODR. Re-searched local archives and public product material; no selector-2/3 or buffered-export definition was found.
- 2026-08-27: Inspected the user-supplied vendor RAR safely. The official manual resolves the distinction: accelerometer 800 Hz and gyro 400 Hz internally, but the documented 27-byte selector-1 host packet exports one main plus one backup sample per video frame. Extracted only PDF/SDK ZIP evidence into this repository; did not run the bundled Windows executables.
- 2026-08-28: Corrected the selector-1 interpretation by measuring the 24-byte sensor payload independently from its frame sequence. Payloads do refresh multiple times within one video sequence, but the blocking XU control path is limited to about 100 reads/s. Neither a 210 Hz video trigger nor two concurrent readers reached 200 Hz. No acquisition source code or external repository was modified.
- 2026-08-28: Corrected the physical data model after comparing the manual, its packet illustration, the added SDK demo, and 300 live packets. The manual says two same-type six-axis groups, not two physical IMUs; the new 200 Hz candidate is one IMU with two temporal samples in each approximately 100 Hz XU packet. Group ordering and 5 ms timing remain to be proven before calibration acceptance.
- 2026-08-28: Added a minimal single-IMU temporal-pair parser and a 30/200 transport-rate probe in the owned repository. Live 10 s HIL passed at 29.574 Hz retained full-resolution SBS and 201.222 Hz IMU with zero read failures/regressions. Preserved the JSON evidence; formal acquisition integration and dynamic timing validation remain open.
- 2026-08-28: User selected native camera 60 Hz. Updated the owned rate contract and reran HIL: 59.472 Hz full-resolution SBS plus 202.505 Hz single-IMU output, with 0 XU failures and 0 timestamp regressions over 10.005 s. Preserved separate 60/200 evidence; the earlier 30/200 artifact remains historical.
- 2026-08-28: Corrected the requested deliverable from offline solving to operator-facing calibration capture. Added an immutable 60 Hz SBS + 200 Hz IMU recorder with bounded asynchronous storage, raw XU evidence, SI metadata, CRCs, and SHA-256 manifests. Two diagnostic HIL failures isolated write-lock and JPEG-scan GIL contention; the final 5.001 s capture passed at 59.590/202.766 Hz with zero read/timestamp failures.
- 2026-08-28: User correctly rejected the recorder as incomplete because it opened no calibration image. Root-cause comparison with the read-only D405 collector confirmed that the missing component is an independent latest-only stereo/AprilGrid preview, not a camera command-line option. Started a targeted preview addition; external D405 and release trees remain read-only.
- 2026-08-28: Added default left/right SBS preview with t36h11 corner overlays, per-half counts, repeated dual-view target observability checks, and q/ESC abort semantics. First HIL exposed main-thread HighGUI interference (181.14 Hz IMU); controlled no-preview/no-detect/no-GUI runs isolated it and the GUI pump was moved off the XU loop. Final 10 s unmodified HIL preserved 59.479 Hz video and 202.328 Hz IMU with zero transport regressions; visual quality intentionally failed because no AprilGrid was present.
- 2026-08-28: Code review exposed late-abort, lifecycle-cleanup, target-identity/geometry, and drop/jitter evidence gaps; all received regression coverage and targeted fixes. Real-tag HIL then proved thread isolation insufficient and Queue IPC still disruptive. Replaced preview transport with a non-blocking two-slot shared-memory handoff to a spawned one-thread OpenCV process. Current 10 s HIL restored 59.468 Hz video / 202.491 Hz IMU, zero inferred drops, and bounded jitter while retaining the live window and geometric AprilGrid gate.
- 2026-08-28: A subsequent exact-snapshot run, no-preview baseline, and independent rate probe all showed the device XU path degraded to 168.86–176.61 Hz while video stayed ~59.5 Hz. Exact-device USB reset was attempted only after confirming no process owned `/dev/video0/1`, but host permissions/password blocked the reset. Current state is BLOCKED pending a physical 2UQ2 unplug/replug; do not start the formal 120 s run until the bounded gate is rerun.
- 2026-08-28: Follow-up review confirmed the four original Important issues resolved and identified three process-lifecycle edge cases; added unexpected-child detection and safe unstarted-process/shared-memory cleanup. Focused capture/rate tests: 113 passed; `git diff --check` clean.
- 2026-08-28: Operator physically replugged only the 2UQ2. Re-enumeration as Bus 001 Device 036 restored the independent 60/200 probe and no-preview recorder, but two preview runs failed at 176.94 and 168.03 Hz IMU. Runtime evidence isolated 24 OpenBLAS workers in the preview child. Limiting OpenBLAS to one thread before child imports restored the ordinary preview command to 59.456 Hz video / 202.251 Hz IMU with zero inferred drops and 1.866 ms maximum IMU jitter. AprilGrid observability remains pending because the target was absent during this transport-only gate.
- 2026-08-28: Formal `run3` captured strong stereo AprilGrid observability (97.73% simultaneous, about 33 tags per half) but was aborted at 45.45 s and failed transport at 181.297 Hz IMU with 10.996% inferred drops. Preserved the report and manifest unchanged. A 1 Hz preview differential recovered 198.509 Hz average but failed strict drop/jitter gates and contained no visible target; do not reuse `run3` or start another 120 s run until a reduced-cadence real-grid HIL gate passes.
- 2026-08-28: Connected read-only to the X5 and confirmed the same USB 2UQ2. X5 source and HIL evidence showed one fixed-length `GET_CUR` yields 194–202 main-payload updates/s; the PC bridge's per-sample `GET_LEN + GET_CUR` caused its approximately 100 calls/s ceiling. Corrected the data model: bytes 15–26 are backup data, not a second temporal sample.
- 2026-08-28: User selected X5 as the calibration acquisition host. Started board-capability and ownership inventory; deployed source/services remain unchanged, and final calibration evidence remains scoped to this repository.
- 2026-08-28: X5 capability inventory found no ROS/rosbag/Kalibr/OpenCV installation, so the frozen architecture is X5 raw acquisition plus PC-side bag conversion and Kalibr solving. A board-native 10 s preflight passed at 202.35 Hz unique main IMU, 59.79 Hz native SBS, and 30.19 Hz retained SBS with zero XU errors/duplicates.
- 2026-08-28: Copied one immutable X5 SBS preflight frame into `artifacts/ego_calibration/x5_preflight`. Visual inspection found both eyes severely blurred/occluded, so formal calibration capture is BLOCKED pending operator lens/scene correction and a repeated still-image gate.
- 2026-08-28: Repeated X5 still-image gate after operator correction; both stereo halves are sharp and the complete 6x6 AprilGrid is simultaneously visible. First 5 s write-path preflight preserved 195.22 Hz main IMU but failed video at 28.78 Hz/12 gaps. The index localized the failure to an undrained warm-up buffer (235.856 ms, sequence 3→17); capture warm-up now actively drains video and the operator requested D405-style one-command-at-a-time execution.
- 2026-08-28: Operator reran the corrected 5 s X5 write-path gate. It passed at 200.91 Hz main IMU, 30.05 Hz retained stereo, and 59.57 Hz native video with zero XU errors, duplicate main payloads, native sequence gaps, or operator abort; IMU p99/max interval was 6.53/7.09 ms. Authorized progression to the stereo calibration dataset.
- 2026-08-28: The 90 s X5 stereo run captured gap-free 59.463 Hz native / 29.997 Hz retained video, but the combined old gate reported FAIL because main-IMU polling averaged 181.288 Hz with repeated 20–35 ms XU stalls. Preserved the original board report; classified the camera transport as usable for stereo evaluation and the IMU stream as unacceptable for camera–IMU calibration.
- 2026-08-28: Added stage-aware X5 acceptance (`preflight`, `stereo`, `cam-imu`) and operator-facing terminal summaries. Stereo capture now treats an IMU-rate miss as WARN rather than a false overall failure, while camera–IMU still requires sustained 190–220 Hz and zero XU errors. Focused verification: 5 tests passed; source compile and diff check passed.
- 2026-08-28: X5 rebooted while the 787 MB dataset was being copied and cleared `/tmp`. Non-destructive local salvage verified 1813 CRC-valid JPEG frames over 60.40 s at 29.9997 Hz; a 605-frame AprilGrid scan measured 42.64% simultaneous visibility, so the salvage remains diagnostic and fails the frozen observability gate.
- 2026-08-28: User selected PC-direct acquisition. Added red native-bridge and Python contract tests, reproduced the per-read `GET_LEN` defect, then moved length validation to bridge open and changed rate/capture paths to publish only the main IMU group. Focused software verification is 116 passed; PC HIL remains pending the rebuilt repository-local bridge.
- 2026-08-28: Built the corrected x86_64 bridge inside the owned repository (SHA-256 `a00b0d2056b90b19e030bb77c5ead20e3f4326d97a6670d75ac011597f403824`). Main-only PC HIL failed at 179.993 Hz with Python video callbacks; pure GStreamer improved average scheduling but repeated 20–25 ms `GET_CUR` stalls remained after active warm-up. Classified software semantics as fixed but 200 Hz transport as FAIL pending a different direct USB topology.
- 2026-08-28: Implemented and tested an isolated spawned IMU process plus 32-sample batched IPC (119 focused tests passed). Formal 10 s storage HIL reached 200.477 Hz main IMU and 59.493 Hz video with zero XU/video errors, but remained FAIL at 1.1828% inferred IMU drops from twelve paired 13–15 ms control-transfer holes. Generic libusb cannot issue the XU request while `uvcvideo` owns the interface; next gate is the motherboard's separate rear Thunderbolt controller.
- 2026-08-28: No USB-C-to-USB-C data cable was available, so ran the existing USB-C-to-USB-A topology for 90 s instead. Video passed at 59.466 Hz with zero inferred drops; main IMU averaged 199.499 Hz with zero XU failures/regressions but failed the strict drop gate at 1.3624% (248 inferred misses). Preserve the dataset as stereo-usable and camera-IMU provisional only; do not mark final transport acceptance PASS.
- 2026-08-28: The first PC stereo operator run stopped cleanly before creating data with `preview_process_ready_timeout`. No leaked process/shared memory or device owner remained. Direct Qt/AprilGrid startup and the exact spawned preview handshake immediately passed in 0.147/0.226 s, so no speculative code change was made; a 10 s full-stack preview preflight is required before the 90 s retry.
- 2026-08-28: The required 10 s full-stack preview preflight passed all transport/processing checks at 59.497 Hz video and 200.691 Hz IMU with zero inferred drops. Its overall report correctly failed because no AprilGrid tag was visible in either half. Hold the board in both views and pass a target-visible preflight before authorizing the formal 90 s stereo run.
- 2026-08-28: Target-visible preflight passed observability at 92.41% simultaneous detections and about 32 tags/half, but failed transport at 174.659 Hz IMU / 14.71% inferred drops while video remained gap-free. Per-second evidence showed recovery to 201.788 Hz / 0.39% drops after second 10 with the target preview still active; testing a 10 s active warm-up as the single causal hypothesis before editing source.
- 2026-08-28: Rejected the 10 s warm-up hypothesis: after correcting a test-only 5 s ready-timeout conflict, the measured run still failed at 182.230 Hz / 10.36% inferred drops with no AprilGrid visible. An independent lean probe then produced 192.602 Hz but a 24.914 ms XU-call maximum and about 3.7% sample-count deficit. Current state is a recurring device/USB XU degradation, not a target-detector-only fault. Stop coupled-preview fix attempts; split stereo preview acceptance from no-preview camera-IMU acquisition and require physical replug/fresh transport gate before the latter.
- 2026-08-28: User authorized lowered limits for provisional calibration while manufacturer-side 128-sample FIFO support is pursued. Froze stage-specific semantics before implementation: strict evidence remains unchanged; stereo ignores IMU; provisional camera-IMU uses 170–210 Hz, ≤15% inferred drops, ≤30 ms max gap, zero XU/regression, and strict video, with an explicit `PROVISIONAL_PASS` result.
- 2026-08-28: Implemented stage-specific acceptance and maximum IMU interval reporting in the owned PC capture path. Red test first failed on the absent API; focused verification then passed 129 tests. Real `run1` replay remained strict `FAIL` while evaluating as stereo `PASS` and camera-IMU `PROVISIONAL_PASS` at a 15.297 ms max gap. Added operator-facing NEXT/STOP output and corrected the strict-PASS message during local review.
- 2026-08-28: Formal stereo run1 passed stage acceptance and all manifest hashes were independently verified. Target observability was 91.07% simultaneous across 448 preview samples. Began the derived ROS1/Kalibr export phase; confirmed the pinned container has ROS1 bag/message/OpenCV support, cleaned one mis-invoked probe container, and found no `kalibr_bagcreater` command on PATH.
- 2026-08-28: Located and fully read the pinned package's internal `kalibr_bagcreater`; selected its static cam-folder contract instead of implementing ROS1 serialization. Froze a 5 Hz deterministic export (444 synchronized pairs, estimated 1.84 GB bag), left-half cam0/right-half cam1 mapping, identical acquisition timestamps, CRC/shape validation, and the measured 35.2 mm AprilGrid target.
- 2026-08-28: The first 120 s PC camera-IMU command was intentionally stopped at 70.83 s because `--no-preview` and a silent loop gave no operator feedback. Preserved it as diagnostic-only: 59.482 Hz video, 196.054 Hz main IMU, zero timestamp regressions, 19.024 ms maximum IMU gap, and valid manifest hashes, but no formal acceptance due to abort and short duration.
- 2026-08-28: Added immediate setup/preview/recording/finalization messages, 5 s progress counters, and clean Ctrl-C conversion to an operator-aborted report with final IMU draining. Regression coverage fixes the progress and KeyboardInterrupt contracts. The next formal attempt must use a new run2 directory and 1 Hz live preview.
- 2026-08-28: Operator completed `ego_pc_cam_imu_calib_20260828_run2`: 120.000 s, 59.466 Hz video, 186.608 Hz main IMU, 8.353% inferred IMU drops, 16.486 ms maximum gap, zero XU failures/regressions, and 95% simultaneous valid-grid preview. Stage result is `PROVISIONAL_PASS`; strict transport remains FAIL.
- 2026-08-28: Operator authorized autonomous offline parameter solving and already completed the deterministic stereo export: 444 synchronized pairs over 89.916 s, measured 4.927 Hz, derived-index SHA-256 `d9b039ef6c71f6327320149f4961d2d3b0c68d51553167931507a4c1e7fc4e04`, status PASS. Started pinned-container bag creation and calibration solve; all outputs remain repository-local.
- 2026-08-28: Created and verified the 1.8 GB stereo ROS1 bag (444 images/topic). Kalibr solved intrinsics and a 74.886 mm baseline; PDF-only graph generation failed because the pinned container lacks `cairo`, while camchain/results were already safely written.
- 2026-08-28: Exported provisional run2 to 1,099 image pairs plus 22,393 original SI IMU samples and created a verified 4.3 GB ROS1 bag. The first two-camera IMU solve was rejected at ~8 px reprojection and inconsistent per-camera shifts. The corrected cam0-only solve reached 0.595 px mean reprojection, 43.36 mm lever arm, and -2.387 ms shift; cam1 transforms were composed from fixed stereo geometry.
- 2026-08-28: Overall calibration acceptance is FAIL, not provisional PASS, because independent held-out stereo rectification measured 13.014 px vertical p95 versus the frozen 1.0 px gate. Frozen all parameters and evidence for diagnosis; any immediate 30 s dataset is diagnostic-only. A new slow, sharp stereo target capture is required for release calibration.
- 2026-08-28: Recovered an internal-training calibration from the higher-quality run2 capture without relaxing gates. New stereo held-out validation passed at 0.927 px p95 over 511 excluded frames; the new cam0-IMU solve passed residual/plausibility checks at 0.584 px reprojection and -2.129 ms time shift. Frozen `ego_calibration_training_v1.yaml` plus acceptance/hashes as `PASS_WITH_PROVISIONAL_TRANSPORT`; fresh verification passed 131 tests, matrix/hash checks, `py_compile`, `git diff --check`, and a valid 12-page report PDF.
- 2026-08-28: Reviewed the official AprilTag 3 and April-Tag-VR-FullBody-Tracker source for the requested Ego + dual-UMI AprilTag world-alignment design. Selected official AprilTag as detector, rejected OpenVR playspace logic as the product architecture, froze Ego-world transform equations, and opened a repository-local implementation phase without modifying external D405/Ego/release trees.
- 2026-08-28: Added red AprilTag spatial-alignment tests for mount composition, quality rejection, robust-window planar-pose outlier rejection, signed timing correction, detector-latency isolation, independent left/right anchors, and relocalization continuity. Expected red result: collection fails only because `three_device_slam.spatial.apriltag_alignment` is not yet implemented.
- 2026-08-28: Implemented the detector-independent AprilTag mount/observation/quality/robust-anchor layer and immutable relocalization reconciliation. Focused tests passed: 9 AprilTag alignment tests, then 12 combined alignment/DEMO tests.
- 2026-08-28: Generated `artifacts/spatial_demo/ego_dual_umi_apriltag_20260828_v1`: 30 s at 30 Hz, 900 common-world rows, 550 visible Tag observations, independent 60/90-frame left/right VIO occlusion bridges, one rejected planar-pose outlier per chain, and a 6.4 MB MP4. All five manifest hashes and trajectory/quaternion contracts passed. `ffprobe/ffmpeg` were absent; installed OpenCV independently decoded all 900 frames at 1280x720/30 Hz.
- 2026-08-28: Independent code review rejected DEMO `v1`: rejected rows regenerated clean poses, reacquisition was visibility-only, one-shot pose iterators were consumed, duplicate timestamps could fake consensus, and BLOCKED consensus omitted outlier reasons. Added five reproducing red tests, fixed each root cause, and reached 17 focused passes.
- 2026-08-28: Generated corrected `artifacts/spatial_demo/ego_dual_umi_apriltag_20260828_v2`: 900 trajectory rows, 564 Tag rows, four preserved rejected PnP matrices, real post-occlusion 5-candidate/4-inlier solves for both sides, accepted continuity reconciliation, five valid manifest hashes, and complete 900-frame OpenCV decode at 30 Hz/1280x720. `v1` is superseded and must not be accepted.
- 2026-08-28: Closed the independent re-review at Critical 0 / Important 0, then added negative-delay and default MP4/hash/decode regressions. The first shared-interpreter video test polluted the pre-existing `cv2` import-isolation gate; a minimal 5-pass/1-fail reproduction proved the boundary. Running video generation/decode in a child interpreter restored the minimal order to 6 passes. Final fresh evidence: 29 spatial tests passed, all 757 repository tests passed in 294.06 s, and no implementation change followed that full run.
- 2026-08-28: Advanced the AprilTag chain from ideal pose injection to deterministic image replay. Added strict camera/detector/IPPE schemas, positive-depth/reprojection/planar-ambiguity gates, 40 mm scale and timestamp regressions, and an OpenCV reference backend that does not claim to be the unavailable official AprilRobotics production backend.
- 2026-08-28: Generated final reviewed `artifacts/spatial_demo/ego_dual_umi_apriltag_image_20260828_v3`: 60 rendered 1280x720 frames, 120 left/right audit rows, no false IDs, independent initial and post-occlusion anchors, explicit 3.9/4.8 s relocalization switches, 19.68-27.48 mm / 0.735-0.737 deg simulated image-derived anchor error, full IPPE candidate/selection evidence, six verified file hashes, and a fully decoded 60-frame MP4. Verdict remains `PASS/simulation_image_replay`, HIL `NOT_RUN`; image v1/v2 are superseded by trajectory-switch and candidate-binding review fixes respectively.
- 2026-08-28: Independent final re-review closed at Critical 0 / Important 0. Fresh final verification after all implementation fixes: 35 spatial tests passed in 3.24 s, all 763 repository tests passed in 353.58 s, v3 six evidence hashes/three generator-source hashes matched, all 60 trajectory rows obeyed their pre/post-reacquisition anchors, the 60-frame MP4 fully decoded, `py_compile` passed, and `git diff --check` remained clean.
- 2026-08-28: Completed a read-only audit of the afternoon D405 device-2 Docker candidate against the owned three-device coordinator. Confirmed a healthy independent local VINS/loop run, but no installed formal runtime, no left-role/world-alignment binding, and unsafe global ROS/Rerun collisions for a second unchanged live Docker chain. Confirmed the owned coordinator currently performs synchronized three-device raw capture only and does not launch or consume Docker SLAM; live Ego-world dual-UMI fusion remains unimplemented and unverified.
- 2026-09-05 18:58: Captured user-confirmed Ego repositioning, 600/601 formal frames, all CRC valid. Two-tag gate passes at2.069deg/1.621mm-z, consensusz=-46.680mm; full mount chain separation8.074mm remains. Compared against17:46, mount change3.399mm/0.206deg but viewpoint change only35.8–38.1mm/4.40–4.57deg: insufficient for frozen50mm-or5deg gate. Saved reproducible evidence in artifacts/spatial_bench/two_tag_crosscheck_20260905T185830. Old61.342mm and provisional46.541mm calibration files unchanged.
- 2026-09-05 19:50: Additional operator repositioning produced599/599 formal frames after one documented preview-release startup collision. CRC all valid;32 sampled geometry frames per camera pass. Current crosscheck1.636deg/1.573mm-z passes; z=-46.460mm. Compared with18:58, view rotation12.739/12.644deg and mount change2.028mm/0.620deg establish PASS_CHANGED_VIEW_CONSISTENCY_ONLY. Complete mount-chain disagreement6.057mm and physical metrology remain unresolved. Full evidence saved under two_tag_crosscheck_20260905T195005_retry; no source edits, activation or commit.
- 2026-09-05 offline follow-up: Added diagnostic-only joint raw-corner fit, independent heldout external registration and leakage/nonconvergence tests. Primary train17:46+18:58/test19:50 improves mount pixelRMSE11.204→5.011px; provisional jointz=-49.593mm.20tests pass; independent review findings addressed. Results/evidence in joint_mount_diagnostic_20260905_v2; absolute accuracy remains unverified, no hardware or calibration activation.
- 2026-09-05 fixed-layout follow-up: Recorded user confirmation of righttag40mm and flatness; added shared-external-pose offline contrast.8new+previous diagnostic tests pass, independent review no Important issues. Reused-validation error5.011→7.168px, no improvement; reject constraint as proposed fix and preserve report in fixed_layout_diagnostic_20260905_v1. No hardware or calibration changes.
- 2026-09-05 20:55: After user replaced wrinkled leftid1 with printedflatid1, captured599/599formal frames, CRC and sampledgeometrypass. Crosscheck1.583deg/1.263mm-z passes butfullmount6.867mm not improved. FrozenjointM scores10.620px vs old5.011. Left-tag estimated position also changed69.44mm andEgo footprint shrank, so not a pureflatnessA/B. Evidence and reproducible frozen score script saved flat_left_tag_20260905T205500; no calibration activation.
- 2026-09-05 21:01/21:29: Newboxcapture firstreview6.159mm-z andrightangle45.49deg. Afterrightadjustment600/600CRCvalid, rightangleimproved42.1deg butspacing<20cm andcrosscheckrotation3.134deg fails;z0.067mmagrees whilefullmount15.858mm mainlyx. Currentresultnotaccepted; stop micro-placementloop and propose stronger external geometry rather than changinggates. No activation, preview/capturestopped.
- 2026-09-05 21:56: Added isolated mixed80 external/40mount detector, discrete duplicate-ID association, static crosscheck and live preview. 29 related tests pass; reviewer clock-provenance issue fixed and re-reviewed. Preview now live in mixed80_preview_20260905T215620; Ego external ID1/2 visible, mount cropped right; UMI both external tags occluded by gripper. Waiting operator adjustment, no fresh capture/acceptance or activation. See handoff §24.
- 2026-09-05 21:59: Operator adjusted; captured mixed80_crosscheck_20260905T215930,600/600formal frames and allCRCpass. Full-frame mixedanalysis598/600associated/allgeometrypass, rotation0.608deg/z6.494mm/full3D6.540mm => REVIEW. Fourchunks z6.41–6.68mm persistent. No activecamera processes or calibrationactivation. Preserve RESULT/SHA256SUMS; operator asked to measure both printedblack-square widths/heights (nominal80mm), keep currentlayout. Handoff §25.
- 2026-09-05 algorithm investigation after operator80mm confirmation: froze598samples and exactbaselineposes. APRILTAGcornerfrontend z6.494→4.523mm, full3D6.540→4.846mm; solver-only6.349mm andsubpix6.218mm controls. Fourtimechunksrefined4.50–4.54mm. Positive-depth/geometry/reprojectionchecks andindependentreview passed, noabsoluteaccuracyclaim. SeparateSDKrotationcolumnmajor/rowmajor defect verified butcommonrigidtransformnorminvariance excludes it as6.5mmrootcause. Diagnostic artifacts+hashes saved fixed_image_ablation_all_v1; formalcode/calibration untouched. Handoff §26.
- 2026-09-05 approvedfrontend integration: sharedconfignone/apriltag(defaultnone), mixedpreview/crosscheckdefaultapriltag, explicitlegacy switch andseparatereports.83relatedtests pass, independentreviewclear. Integrated600pairrefinedreplay exactlymatchesdiagnostic4.523mm-z/4.846mm-full; legacy600pairaudit/chains exactoldparity. Evidence in mixed80_40_apriltag/integration_acceptance.json, docusage added, sourcebeforecopies preserved. NoMactivation/livehardware/coordinatorchange orSDKlayoutfix; freshviewpending. Handoff §27.
- 2026-09-05 22:40: User requested preview fornewview. Opened refineddualpreview, toolsession47113, artifacts/spatial_bench/mixed80_newview_preview_20260905T224028. Egosnapshot shows backmount belowimageboundary and laptopdisplay showingadditionalTagcopies, producingambiguousIDs. Askuser keepphysicalbacktag fullyvisible andturnscreenoutofcamera/don'tshowtagscreen. Noformaldatarecording. Previewremainsopen; stopandawaitownershiprelease beforefuturecapture.
- 2026-09-05 extraID1followup: UpdatedpictureactuallyshowsphysicalspareID1behindleftbox, correctingearlierscreen-onlyinference. Added>=2ID1association plusbroad150mmUMIoriginmountproximity, exactlyonecandidate/noexactMfit; unresolvedcandidatefailsclosed afterreviewfix. Per-rolecolors nowwork independently;88tests pass, reviewerresolved. Live93971 at mixed80_identity_preview_20260905T224800 correctlyignoresfarspare~460mm, recognizesback~50mm. Egotwoexternalangles49.7/48.4deg stillfail; asklowerEgo, nofreshcaptureyet. Existingpreviewstops eachconfirmedexit0beforerestart.
- 2026-09-05 22:51 new-view: Preview93971 stopped/released; captured mixed80_newview_20260905T225115,599/599 formal frames, all CRC/clock checks pass. Frozen refined fullframe598/599 associations/allgeometry pass; chain1.336deg/z2.237mm pass, full3D5.631mm remains. Reference refined mount change4.441mm/0.321deg passes repeatability only; UMI external2 pose also changed40.171mm, not pure Ego-only controlled motion. Legacy control is slightly better1.099deg/z1.340mm/full3D5.505mm; no universal frontend improvement claim. RESULT/comparison/handoff saved;17 evidence hashes verified, independent translation calculation agrees, git diff --check clean. Cameras stopped, no activation/commit or external writes.
- 2026-09-05 SDKfix and residualdiagnosis: Fixed3local serializers and provenance-aware mountreader,20layouttests and201targetedregressions passed in234.93s. Code/diagnostic independentreviews clear afterlegacyconventionguardfix. Frozen598+598poses exactly reproduceoldagreement; correctedM shifts0.21–0.244mm/0.536deg, full3D4.846/5.631mm unchanged. NewviewfullCLI599pairs/598associated passesexistinggates, exactoldaudit preserved in_layout_v2. BuiltpinnedofficialAprilTag underrepoartifacts only; native/half-pixel/median-corner/densepattern controls notconsistentlybetter. Propagatedcross-tagbias1.52–2.56px versusnoise0.15–0.17px; external-onlybundle's frozenoldM predictsnewmount3.651px. SparseKfits unstable/noresidualredundancy, notproofphysicalKwrong.50finalevidencehashes verified, raw/oldreports preserved, nohardware/activation/commits/externalwrites. AwaitoperatorflatAprilGrid availability forindependentmultiviewobservations; cannotclaimremainingrootcausefixed. Fullhandoff§30 androtation_layout_fix_20260905/RESULT.md.
# 2026-09-06: continuous A/B implementation; live attempt incomplete

- Added opt-in continuous A/B automatic static gate and A-only frozen offline
  checker. Single pipeline start per device, bounded300s, noR, no calibration
  activation. Review identified missing mount static gate; fixed per-phase board
  and mount drift checks with negative regression and A-quality rejection.
- Authorized continuous_run1 timed out WAIT_A, zero A/B samples, cleanup[].
  Ego36/36 both; UMI ended32/36, preview screenshot top/right board crop.
  Cameras released, no automatic retry. Need board reframed for a new attempt.
- Final related regression:141 passed in10.04s; independent review cleared the
  mount-static fix (5 tests). git diff --check clean; original run4 solve SHA
  191f51f5d52f0742b66c5c7948eddb5d5e53c615bdcdd1cf9d4d5f1453d9916f unchanged.

# 2026-09-06: automatic simultaneous Ego IR + UMI diagnostic

- One authorized10s completed at ego_ir_umi_20260906_run5:20triples60PNG, cleanup[].
  Startup failures1-4 zeroimages retained; identified immediategainGETPIPE, optional
  metadata warning only, no emitter waiver. Capture source hash reproduced exactly.
- Added isolatedcapture, directfixedfactory3D→2D groups and posthoc3raybundle.
  Directmedianhorizontal2.295mm,holdout2.144pxFAIL; bundle lowersgroupdifference
  butworsensheldout. No activation, no furtherhardware, no externalwrites/commits.
- Independentreview v2 all60raw/140groupfits exactly reproduced, seedleak fixed.
 125targeted tests passed9.97s, includingnewcapturecleanup/safety guards.

# 2026-09-06: completed requested all36 multi-group static analysis

- Added offline common_board_resampling_analysis.py and3regressiontests. All12source
  pairs audited underoriginalpolicy first; uniquegeometricboardID1restoredonlyderived.
  Completed816frozengroupfits/432single-tagchains, sourcehashesandstaticgatesverified.
- Whole36norm56.007–56.876mm; regionalcontrasts checker.697–2.202, horizontal
  11.644–15.623, vertical2.760–4.203mm. Regionalinconsistencyremains, notabsoluteerror
  oruniquephysicalrootcause. Report/RESULTsaved all36_resampling;handoff§49updated.
-108relatedtests passed9.90s; independentreviewexactlyreproducedall12framesandfits,
  noCritical/Important. No livecamera ornewmanualcapture; factorycalibrationunchanged,
  NOT_ACTIVATED, worktreemainuncommitted, externalrepositoriesreadonly.
# 2026-09-06: offline corner, plane, depth and pose-support isolation

- Current user "排查": no hardware or factory writes. Replayed both sessions'
  300PNG/100triples, all36/failurelabels;40cornercases and2000scores complete.
- New artifacts tilt_run3/isolation_v1: corner_controls, plane_controls,
  depth_controls, pose_support_v2 reports/scripts and RESULT. Tested corner/depth
  changes do notfix B. A/B separateT differ4.450mm/.343deg; AB sharedfixedKDfit
  achievesmedian.726/.519px ontilt A/B but is explanatorytraining, NOTacceptance.
- Reviewer foundB-only initializer inheritedA; fixed own-phase seed, keptv1 as
  superseded, addedmissing/poisonedA negative.31tests pass; baseline/source hashes
  unchanged. Noactivation, uniquephysicalcauseunresolved. Seehandoff§62.

# 2026-09-06: matched-data method comparison after duplicate-proposal correction

## Follow-up: actual upstream multical baseline

- Pinned upstream efe9d7d0af85d51d32669e4b77469494972ac25a in
  artifacts/spatial_bench/upstream_multical_20260906_run1/vendor/multical.
  Dependencies isolated in the same diagnostic directory; no /opt installs.
- Adapter calls real unchanged Calibration.bundle_adjust, camera model and
  StaticFrames. Frozen SDK rays -> virtual pinhole; fixed numeric board, same
  selected corners/train slots, linear loss/no outlier deletion. Upstream camera
  gauge retained, only relative camera transform compared. Not a detector/CLI test.
- Initial import failed missing cached_property; installed isolated dependencies.
  Initial negative test polluted a shared synthetic object array including train;
  fixed fixture to replace holdout array. All 5 focused tests pass (0.69 s).
- Original run4 complete image/hash/static replay and real optimizer comparison
  complete. Own4.391011907px, upstream4.390738283px; actual independent upstream
  stereoCalibrate4.390746727px, then multical4.390488482px. 1288/1288 inputs kept.
  Final36 tests pass4.14s (verification_junit_v2.xml); verification script passes hashes/supports and emits stable
  small-angle scipy Rotation values. No hardware or calibration activation.
- V1 independent reviewer reloaded all288PNGs and reran actual multical in memory,
  exact numeric reproduction. No Critical/Important; reported Rodrigues tiny-angle
  zero documented with separate stable verifier. Independent-init extension review
  complete: raw/fit/score exact replay, old-seed+holdout poison unchanged, no remaining
  Critical/Important. Poison check added as regression. Original run4 report hash
  unchanged. RESULT records scope and limitations; no active capture/solver.

- No hardware. OldRGB original16-step/288PNG replay and old8train optimizer
  parity max.000287px. New twoIRsessions same scoring: two-stage vsjoint3view
  tilt A-only B2.453595/2.452627px; nofix. SameAB T source-score A.726/1.911.
- Source-only mono/stereo boardpoints .508mm -> fixedT predictions1.523px apart,
  nottrutherror. Additional frozenA-IR X/Y factor.9976495 improvesnewtiltB.797
  butoldlockedB3.089 andRGBmedian3.269; no transfer, no deployment.
- Reports/scripts/tests in tilt_run3/matched_methods_v1; aspectv3 finalpreserves
  oldRGBsolverREVIEW/novelty (reviewrequestfixed).45tests pass, originalreports
  hashes unchanged. Noactivation/commits/newcamera. Detailedlimitations inRESULT.
# 2026-09-14: completed requested steps 1 and 2

- Added `tag_world_constraints.py`, public exports, CLI, five focused unit tests,
  per-phase holdout policy and residual visualization.
- 12 related tests pass. Real run8 constraint chain and all nine recorded provenance
  hashes were independently recomputed; 96/96 matrices satisfy
  `T_EW_RW = T_EW_grid * inv(T_RW_grid)`.
- Full30Hz/10Hz results consistently reject the fixed transform (325/96 constraints).
  No runtime config changed and no sensor/ROS/Kalibr process remains.

# 2026-09-14: run9 capture, VINS, Tag-ID crosscheck and short-window diagnosis

- Fresh105s Chinese-guided run9 acquisition passed and was sealed; Ego/right VINS
  passed with3139/3117 poses. No capture or SLAM process remains.
- Added explicit1x/2x detector analysis scaling plus source-coordinate restoration;
 20 focused tests pass. Full2x run produces1664 constraints with endpoint coverage.
- Fixed transform rejected in full and stride-3 analyses. Full coarse phase-anchor
  pose gate passes marginally, but stride-3 yaw translation P9534.157mm fails and
  phase-local epipolar P95 remains3.558-3.900px. No world transform activated.
- Preserved reproducible short-window pose diagnostics.1-2s windows reduce heldout
  P95 to12.394-13.811mm/1.003-1.117deg, supporting the bounded next step of replaying
  device3's mounted-tag observations through the existing device-2 anchor path.
- Existing mount inventory remains incomplete. A run9 IR overlap scan found7 numeric
  board+mount candidates but0 pass the formal60px mount-edge gate (max59.419px).
  Saved the blocked diagnostic; the next hardware action is only a10-20s static
  preview-gated board+mount capture, not another105s motion run.

# 2026-09-14: operator-requested run11 static mount recapture

- run10 was explicitly rejected; archived12 compact reports and removed3.889GB raw.
  First run11 launch was storage-blocked before hardware open, then the retry completed.
- run11 capture PASS, canonical single_umi coordinator/seal/index and verifier PASS.
  The first generated topology string was noncanonical; archived its derived failure,
  rebuilt from unchanged raw data, and obtained clean verification.
- Strict common-ID v2 keeps54/450 frames. Mount size/fit and half-candidate repeatability
  pass, but heldout mount-corner prediction P9510.345px fails the1px gate. No activation.
  Saved v1 method control, v2 decision report, preflight counts and framing montage.

# 2026-09-14: staged dual-camera preview and10s adjustment countdown

- Replaced the rsusb_pair display source selection so it shows Ego left IR and each active
  UMI left IR side by side instead of the two Ego stereo eyes. The display remains active
  before and during formal capture and keeps the existing Chinese cue/countdown overlay.
- Changed the entry default start delay from0.5s to10s. The first8s permit whole-rig/board
  adjustment without changing relative camera mounting; the last2s require settling.
  Added source-selection, cue-boundary and rendered dual-payload regression coverage;
  17 rsusb_pair plus19 adjacent D435i/coordinator tests pass, and the offline1280x360
  render was visually inspected. A wider coordinator suite was interrupted after25 tests
  when an unrelated heartbeat `fsync` waited on the full filesystem journal. No live
  camera process was started; next capture awaits the operator's explicit start instruction
  and storage cleanup.

# 2026-09-14: cleanup plus two bounded dual-preview recaptures

- Preserved compact reports then removed rejected run11 raw, pip/npm caches and user Trash;
  run12/run13 raw were also removed only after their rejection reports and montages were
  written. Free space rose from6.9GB to25GB. APT cache500MB remains because sudo needs an
  interactive password; project runtime dependencies, run8/run9 and accepted sessions stay.
- run12 transport PASS but physical static FAIL: right gyro P957.821/max18.676deg/s with
  sustained movement through4.77s, plus only3-7 sampled common tags. This exposed the old
  bench motion status bug. Added a <=3deg/s static gate that now controls overall status.
- run13 with dual preview and10s-adjust+5s-settle passes acquisition/static timing:
  450 Ego pairs, right451 frames, pair max12.932ms, gyro peaks1.063/.720deg/s. A151-pair
  board preflight rejects framing:0 pairs >=12 common tags, Ego median4/max9, right
  median30/max35, common median3/max8. Saved run13 framing montage/report; NOT_ACTIVATED.
## 2026-09-14 run15 mount capture

- Captured, indexed, sealed, and pair-verified run15: PASS.
- Confirmed dual-camera framing with fixed4x AprilGrid detection.
- Replayed exact-common-ID mount calibration with fixed5 source-pixel corner refinement:
  412 accepted frames, heldout p95 .686px, temporal-half delta .020mm/.007deg, all
  candidate gates PASS; activation intentionally remains disabled pending novel-pose HIL.
- Preserved the replay script/report with SHA256 manifest. Archived and removed rejected
  run14 raw payload, reclaiming5.478GB.
- Final verification:18 focused RSUSB pair tests and116 D435i/coordinator tests pass;
  pair verifier returns no reasons, `git diff --check` passes, and no capture/VINS process
  remains active.
## 2026-09-14 run16 novel-pose capture

- Captured/indexed/sealed/pair-verified run16: acquisition PASS.
- Frozen run15 validation: board novelty PASS,410 accepted frames, prediction FAIL at
  5.071px p95. Confirmed correct external-tag identity and54.1mm/1.85deg relative pose
  change versus run15. Saved reports and comparison montage; activation remains disabled.
## 2026-09-14 run17 control A

- Archived/deleted invalid run16 raw and reclaimed5.534GB.
- Captured, sealed and pair-verified run17 control A: PASS acquisition.
- Established a stable baseline candidate:411 frames, static-window heldout.313px p95,
  temporal half delta.038mm/.006deg. Saved the per-frame1.785px jitter diagnostic and
  explicit decision report. Await board-only movement for control B; no activation.
## 2026-09-14 run18 control B

- Captured/indexed/sealed/pair-verified run18: acquisition PASS.
- Controlled A/B: board novelty62.7mm/~17deg, external mount stability42um/.048deg,
  frozen-A B prediction2.994px p95 FAIL. Saved frozen validation, diagnostic pose-bias and
  consolidated decision reports. No process remains; no activation; stop recapture.
## 2026-09-14 A/B shared-rigid model fix

- Extracted fixed stride3 median-ID observations from retained A/B raw.
- Implemented/replayed shared-rigid two-pose fit, checker parity swap, all-ID final
  candidate, hashed package manifest, and independent C validator.
- Both checker holdouts pass below.256px; final fit.232px; validator selftest behaves as
  specified. No activation. Archived/deleted superseded run15 raw, reclaiming5.479GB.

## 2026-09-14 run19 C capture and rigid-change diagnosis

- Captured/indexed/sealed/pair-verified run19 C; native acquisition PASS with 450 pairs
  and 2.490ms cross-camera timing p95.
- Recovered C board detections with coverage-selected fixed2x and checked mixed
  Ego4x/right2x. Both reject the frozen A/B relation by about 74px; frozen mount
  prediction independently rejects it at 36.573px.
- Quantified the physical change: C board and mount estimates agree within
  2.609mm/.326deg, while each is about 11--12mm/6.5deg away from frozen A/B. Saved the
  non-activation diagnosis; no runtime/world transform was changed.
- Removed 27 superseded raw `.bin` files (15.444GiB) under the previously authorized
  aggressive cleanup policy after preserving reports, extracted observations, session
  metadata, hashes, and a cleanup manifest. Free disk is about 25GB.
- Waiting only for operator to rigidly lock the present Ego/device3/tag assembly. The
  next automated work is fresh A′/B′/C′ capture with board-only pose changes.

## 2026-09-14 run20 A′ baseline

- Captured and verified 450 synchronized pairs with Chinese dual preview and 15s setup
  delay. Transport, calibration identity, static motion and pair gates all PASS.
- Frozen coverage-selected Ego3x/right2x detector scales; 450/450 frames meet board and
  mount coverage. Static-window heldout prediction .289px p95 and half stability
  .037mm/.011deg pass. Saved `run20_Aprime_decision.json`; activation remains disabled.
- Awaiting board-only movement for B′. The rigid camera/tag assembly must remain untouched.

## 2026-09-14 run21 B′ and A′/B′ freeze

- Captured and verified B′: 450 raw pairs, 449 common-time rows, timing p95 6.458ms and
  static motion PASS.
- Extracted A′/B′ observations at coverage-selected fixed scales. Board novelty is
  78.7mm/15.5deg; mount stability is .041px and .028mm/.067deg.
- Both checker parity joint fits and the full-ID shared-rigid candidate pass below .284px
  p95. Saved `run20_run21_Aprime_Bprime_decision.json`; no activation. Await C′ board-only
  movement and independent capture.

## 2026-09-14 run22 C′ and static mount calibration closure

- Captured/indexed/sealed/verified C′ with 450 raw pairs, 449 common-time rows and
  16.551ms timing p95. All static and identity gates pass.
- Independent frozen validation passes: .533px whole-camera board p95, .231px mount p95,
  150 windows, 33 IDs, sufficient novelty and unchanged factory geometry.
- Generated `device3_right_world_mount_candidate_20260914.json`, package manifest and the
  device3 spatial candidate record with explicit camera/body/tag directions. No active
  runtime or release was changed.
- Reclaimed 15.373GiB by removing packaged static raw payloads. Pending physical black
  square X/Y measurement, additional disk space, and 105s dynamic world-anchor HIL.

## 2026-09-14 print-scale record and dynamic-HIL preparation

- Rechecked the original device3 print records: tag36h11 ID1, 40.0 mm effective black
  square, 50.0 mm with quiet zone, PDF SHA256 `79eb0a...c58788`, and completed printer
  job 17 with A4/actual-size (`print-scaling=none`). Operator confirmed the mounted tag
  is this same printed asset. Generated a print-verified R2 candidate and package; kept
  runtime activation disabled and recorded that no independent caliper reading exists.
- Preserved compact evidence and deleted only the superseded run8/run9 raw image `.bin`
  files. Released 39,206,707,200 bytes; free root filesystem space increased to about
  62 GB, sufficient for the planned 105 s dynamic capture.
- A run23 dynamic attempt was stopped at about 67 s after live feedback remained
  `不足` through the required translation, yaw and early pitch phases. It is explicitly
  marked `FAIL_OPERATOR_ABORTED` and cannot be used for acceptance. Both cameras released;
  five incomplete image payloads were removed after preserving calibration, timestamps,
  IMU data and abort/cleanup reports, returning free space to about 62 GB.
- Re-audited the necessity of a full dynamic recapture. The prior run9 decision explicitly
  said not to repeat 105 s after obtaining a valid mount calibration. Its capture, sync,
  3139/3117-pose VIO replays and 0.43 m/70 deg excitation remain valid evidence; only its
  failed fixed world transform is rejected. Superseded the stale 105 s next-step plan with
  retained-data offline regression plus a short current-candidate end-to-end anchor HIL.
- Added the dedicated `device3_anchor_hil` capture contract instead of reusing the long
  rigid-pair motion mode. It provides dual-camera preview, a 20 s Chinese preparation
  countdown (15 s adjustment plus 5 s settling), and a 45 s Chinese-guided formal window.
  Ego must remain static while only the whole device3 assembly moves; setup motion peaks
  are reset at formal start. Capture/export/replay focused tests pass 49/49. The command is
  staged in `device3_anchor_short_capture_plan_20260914.json`; no camera process is active.

## 2026-09-14 run24 device3 anchor HIL PASS

- Captured 1350 synchronized Ego/device3 stereo pairs over 45 s after the requested
  15 s adjustment and 5 s settling countdown. Pair timing p95/max is 13.307/13.785 ms;
  Ego peak gyro is 1.972 deg/s and device3 peak is 55.111 deg/s. Coordinator, seal,
  pair sync index and session verifier all pass.
- Corrected the D435i VINS exporter to use one valid warmup acceleration sample as the
  first formal gyro's interpolation bracket. It still prohibits extrapolation and only
  publishes formal gyro timestamps. Added regressions; focused exporter/replay tests pass.
- Ego VINS passes with 1339 poses and 2.315 mm/3.262 deg static excursion. Device3 VINS
  passes with 1328 poses, .266 m/30.349 deg excursion and 9.093 mm maximum pose step.
- Added and ran `analyze_device3_anchor_hil.py`. From 450 sampled frames it accepts 422
  ID1 detections and 409 joint tag/board/VIO solutions. The 2.400 s occlusion reacquires;
  endpoint anchor updates and the independent direct-board path both pass 30 mm/3 deg.
- Wrote the accepted device3-only calibration and hash-bound package. The three-device
  group remains deferred solely because device2 left ID0 mount evidence is unavailable.
- Verified 157 focused acquisition/sync/export/VINS/anchor tests. Hash-checked then removed
  seven large reproducible payloads, releasing 16.338 GB and leaving about 61 GB free.
  Cleanup evidence is in `run24_post_acceptance_payload_cleanup_manifest.json`.

## 2026-09-15 device3 world-live trajectory response repair

- Diagnosed the unresponsive orange trajectory as a source-VINS ZUPT hold, not a world
  transform or Rerun subscription failure. The old run kept publishing odometry while
  repeatedly anchoring the position and zeroing velocity after a false stationary latch.
- Broadened the ZUPT release evidence to include body rotation and strong incoherent
  optical flow while retaining the verified static vibration thresholds. Built an
  isolated library (`cc3c8d2c...813c`) and mounted it read-only over the unchanged formal
  device3 image with a matching hash manifest.
- Reduced Rerun logging rates/history without changing `ViewCoordinates.RDF` or any XYZ
  transform. Made the launcher source ROS itself and wait for the solver-ready marker,
  removing two terminal-environment/startup races.
- C++ tests pass 31/31 and focused Python tests pass 18/18. The 180 s live run
  `device3_world_live_pair_20260915_190638` completed with both anchors, 29.79 Hz D405,
  397.30 Hz device3 IMU, no pose-integrity failure and clean process exit.
- Live evidence contains repeated stationary -> motion -> stationary transitions. The
  new gyro-only escape specifically released ZUPT at 11:09:27 (`gyro_mean=0.0206973`,
  low visual/acceleration evidence), and it reactivated at 11:09:49. The device3 world
  trajectory has 2132 rows, a 0.695 m 3D span and 18.6 mm maximum step.

## 2026-09-15 device3 world-live axis and lifecycle repair

- Root cause of the apparent 180 s process exit: the diagnostic launch explicitly set `DEVICE3_WORLD_DURATION=180`; `run_status.json` recorded a clean `STOPPED_AFTER_WORLD_RUN`, ORB return code 0, and no failure.
- Added explicit `runtime_control`, `stop_reason`, runtime exception reporting, remaining duration, and terminal/GUI mode text. Default remains duration 0 (operator stop). Relative output paths are now canonicalized for Docker bind mounts.
- Corrected the Rerun AprilGrid world declaration from camera `RDF` to the frozen legacy-grid `LDB`: +X increasing grid column, +Y increasing grid row, +Z printed-board normal toward the camera-facing side. Added the metric 6x6 board plane, outline, labeled ID0/ID5/ID30/ID35 reference points, and semantic axis labels.
- Rerun reported a software rasterizer and consumed about 5.5 CPU cores. Future launches use 2 Rerun compute threads and isolate the nice=10 viewer to the last four CPUs on hosts with at least eight CPUs. Applied CPU 20-23 affinity to the active 24-core run.
- Verification: Python compile and shell syntax checks pass; `test_tag_world_constraints.py`, `test_device3_world_runtime.py`, and `test_device3_world_live.py` pass (24 tests). A 3 s HIL smoke run recorded `stop_reason=configured_duration_elapsed`, `failure=null`, both cameras and device3 IMU online, ORB return code 0, and no residual process/container.
- Active indefinite run: `artifacts/spatial_bench/device3_world_live_pair_20260915_191817`; double anchor accepted and `WORLD_RUNNING`.

## 2026-09-15 Ego ORB long-run abort diagnosis and v5 containment

- Recovered the actual failure from `device3_world_live_pair_20260915_191817`: after
  335.94 s, the ORB child aborted with Sophus `SO3::exp` receiving a roughly 1e19
  gyro-bias correction. The Python `BrokenPipeError` was downstream of child SIGABRT;
  both cameras, device3 VINS, and the STM32 stream were still healthy.
- Built hash-pinned runtime `orbslam3-4452a3c4-d435i-live-v5`. It rejects non-finite
  or nonphysical frame/keyframe biases, guards rotation/velocity/position bias
  corrections before Sophus, retains the last sane state, and preserves v4 for rollback.
  A native injection using the recorded failure-scale vector exits 0 without abort.
- A 420.016 s HIL soak exceeded the old failure point and ended only at its configured
  duration: 12,171/12,171 Ego poses returned, 10,334 tracking-OK poses, zero backpressure
  drops, ORB exit 0, D435i 29.996 Hz, D405 29.963 Hz, and STM32 398.847 Hz.
- That soak exposed the preceding trigger: the UI froze the Ego world anchor after visual
  TRACKING_OK, before inertial BA2, while ORB repeatedly reset low-motion initializations.
  The final v5 adds a 2 s sustained-low-motion grace, exports BA2 readiness in the native
  pose protocol, blocks both world-anchor creation and the stop-moving instruction until
  BA2 is complete, and gives a Chinese continue-moving prompt meanwhile.
- Final recorded-data verification: 3,150/3,150 poses, 2,801 tracking-OK, VIBA1/VIBA2
  complete, no bad-IMU reset, no transport failure, clean child exit, processing p95
  25.685 ms. Focused Python tests pass 37/37 and runtime/settings hashes validate.
- A fresh HIL run of the final BA2-gated UI remains the last acceptance step; the 420 s
  soak validated the numeric containment but preceded the BA2 protocol/guidance addition.

## 2026-09-15 Ego ORB v5 final review and shutdown repair

- Corrected the BA2 readiness contract after code review. `InitializeIMU` now reports
  success explicitly; the live-ready atomic is set only after the full BA2 optimization
  and map update return successfully, and every full/active-map reset clears it. BA1/BA2
  map flags are likewise committed only after success.
- World output now fails closed if BA2 readiness disappears after the Ego world anchor
  has frozen. This prevents a new ORB map from being transformed through a stale anchor.
  The readiness getter also requires the current map's own BA2 flag, covering direct
  `CreateMapInAtlas()` paths which do not request a LocalMapping reset.
  The packed pose/runtime contract is versioned as v2 and rejects v1 runtimes before
  launch instead of decoding an 84-byte packet as the new 92-byte layout.
- Fixed two further numerical/state issues: the intermediate 5--20 mm motion band now
  breaks the sustained-low-motion timer, and frame/keyframe bias is committed only if
  the matching preintegration accepts the same correction. Threshold configuration also
  rejects NaN/Inf.
- Reproduced the remaining nondeterministic shutdown SIGSEGV under GDB. The stack ended
  in `Atlas::PreSave -> Map::PreSave -> MapPoint::PreSave`, triggered by the offline
  dataset's `System.SaveAtlasToFile: "map_atlas"`. The live wrapper now writes an
  ephemeral settings copy with Atlas serialization disabled; live trajectories remain
  exported after the worker threads finish.
- Final full-rate replay passes every check: 3,150/3,150 frames and poses, 2,801
  tracking-OK poses, 2,207 BA2-ready poses, VIBA1/VIBA2 complete, no bad-IMU reset,
  no transport failure, and child return code 0. Processing/E2E p95 are 26.070/27.616 ms.
  A separate 300-frame shutdown replay also exits 0. Native huge-bias injection passes,
  focused Python tests pass 42/42, and strict validation ties the required runtime files
  to commit `4452a3c4`, patch `94f07be9...b951`, adapter source, protocol v2, and 92-byte
  pose packets.
