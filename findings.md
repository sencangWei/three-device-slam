# Findings

- Stronger control OMP_THREAD_LIMIT=1 + OMP_NUM_THREADS=1 completedtwice:
  serial_assembly_a/b have identical900INPUT/890SOLVE/867MARG and889rawposes;
  excursionstill2.0477574865deg. Twoidenticalruns NOTgeneraldeterminism/accuracy
  proof; no formalruntime change. Computation-order sensitivity remains candidate.

- Follow-up negative control: OMP_NUM_THREADS=1 runs serial_eigen_a/b still
  give1.189903/2.400557deg with identical900input hashes. This hypothesis failed;
  it does not disable explicit4threadassembly. No threading fix activated.
  Earlier b/c-only "not yet reproduced" text below is superseded by replay_d.
- All31sampled stereo pairs fail frozen scene support (11/14/16 unique matches
  min/median/max); new default runner refuses beforeROS, explicit override only
  diagnostic. This is an engineering observability warning, not unique physical
  cause or calibration proof. Real guarded_replay_final confirms no child start.
- COVINS actual source acquired/pinned; existingNoetic image lacks necessary
  vision/math/catkin dependencies. UpstreamEigen3.3.4EXACT/OpenCV3 requireisolated
  matchedbuild; no installer run. StaticEgo has1KF vsleft9 underactualwrapper
  predicate evaluatedoffline, NOTrealbackend execution or sharedworld evidence.

- IMPORTANT update: explicitdiagnostic replay_d reproduced old2.400557deg with
  instrumentedbinary. All900featurehashessame asb/c. Firstdifference isMARGindex611,
  atts800151.411440513,m122n82,LDLTpathboth,priormaxeig~1.19e16. SOLVEcoststhatstep
  same, nextSOLVEindex636initialcost4181.656711 vs4181.657332. Both8iterations,
  no timebudget stop. Old symptom nowlocatedatSchurprior numericalcalculation,
  not merelyscene support. InvestigateEigenparallelGEMM independently; no rootfix
  claim. Newcontrolledhypothesis: OMP_NUM_THREADS1 disablesEigen internalGEMM
  threading (doesNOTdisableexplicit4-thread factorassembly), input/KDunchanged.

- Diagnostic b/c actual900INPUT hashes,890Ceres numericalcost/stop records,
 864marginalizationhash/spectrum records EXACTLYidentical; both1.189903deg.
 889solvesstop8iterationlimit,1functiontolerance; no timebudgetstop. Oldrun2
  raredivergence notyetreproduced/instrumented, notdeclaredfixed.
- Crucial imagequality evidence: firststereo predominantly blurred/textureless
 near desk; stablebackground onlytopstrip. Independent firststereo LK(FB.5px,
 |vertical|<=2px,disp>2px,depth.1–10m) gives9validlandmarks, belowfrozen30support
 gate (PNP scriptFAIL). SeparateSIFT2000features produces152/126features,
 mutualratio.75 gives15matches,14epipolarpositive matches; all y2.6–173.8 of720.
 So400trackedfeaturesinVINS isNOT400verified3Danchors. Currentstaticdataset is
 insufficientindependentprecisionevidence. Doesn'tuniquelyprovehardware/calib
 orallVINSerrorscausedbythis. Rawframepair savedstereo_first.png forinspection.

- Repeatability trace: run2/run3 first635 rawposes exactly identical, firstdiff
  at635 (21.156812499s afterfirstpose), >1e-9 at636; maximumcrossrunangle2.812353deg.
  Thus don't blame initialIMUaxis/initialization mismatch for this divergence.
  Ceres uses32/40ms wallbudget; marginalization OpenMP NUM_THREADS, frontend
  runs different ROSworker thread IDs. Need instrument/control before rootclaim.

- Final Ego continuation: same input/config IMU run3 again produced889poses,
 but rotation1.189903deg versusrun2 2.400557deg; translation.318mm. Visual-only
 control1.586943deg. Repeatability remains unresolved; do NOT call lower run3
 value a fix, or infer quantitative IMU penalty from a single control. Source
 timestamp/format geometry checks passed, not proof factorycalibration correct.
 All1800images/6013gyro exact in saved verify_readback.py; accelroundoff1.18e-6.
 56tests pass, finalreview no blockers, onlyfuture abnormalposegate testcoverage.

- Actual Ego internal-IMU VINS probe run2 now completed (run1 plugin lookup failure
  preserved):900pairs/6013IMU published, PERF-ROS900/900, backendprocessed900queue0,
  889outputposes, matchingtimestamps/unitquats. Statictranslationexcursion.104mm
  BUT rotationexcursion2.400557deg; engineering5cm/5deg gate only, NOT calibrated
  spatialaccuracy. Factorygeometry+reference noise+td0, noZUPT. Needinspect angular
  convergence before using as sharedmap input, not call this millimetre accuracy.

- Ego internal-IMU continuation: standalone raw recorder has distinct gyro(rad/s)
  and accel(m/s2) streams, each float32xyz with acquisition/arrival ns and CRC32.
  Need combine accel at gyro timestamps by bracketed interpolation, not concatenate
  asynchronous values or relabel one sensor as two. Upstream VINS D435i example
  uses acc_n=.1,gyr_n=.01,acc_w=.001,gyr_w=.0001, g9.805; these are reference
  defaults only, not our measured noise. Its extrinsics are NOT our SDK raw-axis
  calibration and must NOT be copied. Factory gyro->IR0 inverse is body_T_cam0.

- Shared-world followup: historical D435i temp VINS exists in Docker2/data,
  20260828_012230/092539 PASS with1440rawposes. Binding and calibration explicitly
  disable internal IMU and share UMI STM32c48df736; cannot reuse as independent
  Ego internal-IMU calibration. Earlier "no Ego trajectory found" only described
  inspected repo paths, NOT global absence. Current left COVINS input run2 gives
  1284 exact image/body-pose pairs, max numeric clock delta232ns, source hashes
  preserved. It does not compute a shared-world transform or run the backend.

- 2026-09-06 shared-world inventory: local Docker2 VINS raw body poses and matching
  IR DB3 exist at /tmp/umi-v4-065204-loop-probe-slam-run and its slam-export sibling;
  1284 raw poses / 1295 stereo pairs. D435i acquisition/factory stereo calibration
  exists, but no corresponding Ego trajectory found in the inspected repo paths.
  Coordinator currently owns acquisition, not collaborative SLAM. COVINS-G upstream
  c5b180b443b59d2267a14584fa4b090429038698 supplies a ROS1 image+odometry wrapper
  and VINS examples; no map points required. It is NOT already deployed on Humble.
  First implementation is an offline raw-body-odometry + exact-image export for
  that wrapper, preserving body_T_cam0, source stamps, agent/map IDs and provenance.
  Export success must remain distinct from backend execution/shared-world accuracy.

- Matched-methods correction after user's "昨天已经做过": oldrun4 ALREADY
  joint8train, replicatedsameholdouts maxdiff.000287px. New same-stereo-scoring
  A-only tiltB two-stage2.453595 vsjoint3view2.452627px: no optimizerfix.
  AB T fixed, A stereo score.726 vsmono1.911; old/new comparisons mixedsource
  geometry plustraining/validation. No causal RGBvsIR claim fromunpairedsessions.
  IR-only mono/stereo boardpoints Amedian.508mm -> fixedT predictiondiff1.523px.
  A-only IRhomography-derived boardX/Y factor.9976495 improves tiltB2.454->.797
  butsamefactor oldlockedB1.010->3.089; oldRGBmedian2.765->3.269 stillFAIL.
  Rejectasglobalcompensation, notphysicalboardmeasurement orKDproof.45tests,
  review/rawhashes retained;aspect_control_v3 final, nohardware/activation.
  Evidence tilt_run3/matched_methods_v1/RESULT.md;no duplicate recapture request.

- 2026-09-06 user "排查", offline isolation_v1: 300raw/100triples replayed,
  40corner cases/2000scores; native/scaled/edge one-sensor changes stillfail tiltedB.
  Per-view projective plane held-tag p95 IR.067-.095px/UMI.172-.253px; metric
  columnratio deviation mixes camera/board/corners, not uniqueKD proof.
  IR-only projective/planardisparity smoothing alsofails B (1.696/2.440/2.473px).
  Independently initialized A-only/B-only rigid transforms differ4.450mm/.343deg;
  fixed-KD AB jointfit A/B median.726/.519 max.776/.593px. This usesB fortraining,
  NOT unseenview acceptance or true mountdistance proof. Local scaledJacobian
  condition89.69->57.52 onlyexplainsconditionedgeometry, notphysicalrootcause.
  v2 isolatesB initializer;31tests, exactreadonlyreview; originalfailureskept.
  No camera/opening/activation/commits; evidence tilt_run3/isolation_v1/RESULT.md.

- Tilt run3 new~24deg A/B evidence: all120images/40triples36tags, boardstatic
  A.517/B.523px; crossphasemount.108px, IRlock31979/gain26 verified. A-only
  directBmedian1.9573px vsrigiddeclared2.4536px(worse), IR_Ascale2.8475px.
  This tilted-view counterexample rejects rigidboardprior ascompletefix; does
  notuniquelyassignKD/corner/IRmodel. CapturedA20/B20/C0 timeout, restoregain
  GETerrorretained evenlatercurrentsettingsnormal. Noactivation, no newcapture.
  Standardacceptanceloader unchanged/rejects source.43tests/exactreadonlyreview.

- 2026-09-06 lockedABA geometry ablation: all36/A-first10-only rigid board known
  dimensions predicts Bmedianp95 1.010482/max1.089379px vs direct3.041887;
  returning C .347917 vs1.467115. Additional rigidity/metric prior, not an
  equivalent-assumption solver comparison; factoryKD unchanged. Jointfit same.
  Prior unlockedAB method replication B2.930450->2.805549: improvement not
  general. Do not call fixed, blame only AE, or overwrite official calibration.
  IRraw shape about1.1% larger than declared board; no unique attribution to
  board/baseline/KD/corners. Single IR left/right routes still1.755/2.191px.
  Declared-size IR route mountz-46.95 vsdirect-50.65 is NOT groundtruth agreement.
  Current A/B only~1.4deg normal change with203mm translation; new tilted-board
  observations are more informative than repeating parallel translation.
  Evidence locked_aba_run2/model_diagnostic_v1/RESULT.md;35tests, no activation.

- UserexplicitlyconfirmsNOcamera touched/moved. New20frame raw-texture check:
  UMIbottle38–49LKtracks median[11.49,-2.33]px; NCC~[11.35,-2.13],score>.904.
  Old0→10control~.01px. IRmountwholepatternNCC>.996 shifts~[1,1.2]px. Therefore
  phenomenonnotonlyAprilTagcornerjitter, butno uniquephysicalcause inferred.
  Gripper3–10/0tracks insufficient; do notclaimrigidpartstability asproven.
  OldT newrays epipolarp95~9.9undistortedpx meansdepth-onlyfixcannotresolveit.
  Measurementfailure renamedmount_projection_changed, thresholds/numbersunchanged.
  Evidence run6/OPERATOR_CONFIRMATION.md,image_change_v1. Nohardware/activation.

- Changed-board run6: 20triples60PNG, onlyUMI24–25/36 dueupper/rightcrop.
  Boardnovelty91mm/11deg, butEgoIRbackmountold→newdrift1.7/2.5px>.75 gate;
  freshwindowstaticpassed. Oldfrozenrun5all36/direct_mean T predictsnewvisible
  UMIcornersp95median11.980/max12.302px. No newXfit, noactivation. Notclean
  pureboardmotionexperiment; cannotattributeall12px toalgorithm orprovewhich
  cameramoved. Userclarificationpending; evidence ego_ir_umi_20260906_run6/RESULT.md.

- 2026-09-06 simultaneous EgoIR+UMI run5:20triples,36tagsall60PNGs,staticpass,
  directIR3D→UMI2D horizontalmedian2.295/max4.038mm,vertical1.780,checker.525;
  wholeboardcenterrepeat1.071mm; holdoutp95max2.144FAIL; mountIR54pxfails60.
  Per-groupinitialization leakage fixedv2, independently reproduced140fits.
  OptionalD405gainGET immediatelyafterSETfailsUSBPIPE evenbeforestream; SETnoerror,
  currentactualgainunverified, warningretained. Exposure/emitterchecksnotrelaxed.
  Errors-in-variables3raybundle first10train/last10test reduceshorizontal2.320→1.657
  andvertical1.757→.561 butwholeboardheldout1.167→1.540px worse: rejectactivation.
  OldRGBvsnewIR notsameexposureA/B. Sourceandnegativeevidence run5/RESULT.md.

- 2026-09-06 run6 authorized10s complete20triples/60PNG; nativeall36+mount eachstream.
  StaticmaxRGB.571px/IR.140px pass;IRmount54px fails60gate retaineddiagnostic.
  IRtriang→factoryRGB boardp95median2.568/mount2.608px; first5boardonlyTfit gives
  heldoutboardmax.686 butmount27.220px: planarcompensationnotvalidextrinsics.
  NewmultidepthIR3D candidate(train1/2/4/5,exclude3/6/allmount) new6maxboard.518/mount
  .918px butold3max1.277 andexternaloldchain7.891mm worse, soNOT_ACTIVATED.
  IRreferencegeometrynotabsolutetruth; no formalworld/SLAM acceptance. run6/RESULT.md.

- 2026-09-06 regional repair investigation: all12 factorial controls isolate dominant
  Egoobserv/model inconsistency (idealEgo horizontal .866mm vsidealUMI14.085mm).
  native/scaled/edge/pixelobjective all~13mm, notoptimizer/window mainfault.
  New singlecamera old8train OpenCVK4+k1 candidate improvesnewhorizontal13.619→2.566mm
  andoldpairedholdout4.391→1.522px butverticalmax5.450mm. External oldview4.845→9.826mm
  regressionrejectsactivation;newview5.631→4.835mm aloneinsufficient. Maincause not
  uniquelyphysical/fixed. No factory changes; details currentjoint/POSE_REPAIR_RESULT.md.

Research findings and design decisions for Ego-world UMI spatial alignment.

- Current2savedjointstaticwindows already sufficientforrequestedper-tagdiagnosis;
  requiringR/16steps was unnecessaryworkflow expansion. Stoppedpreview cleanlyand
  processedexisting12pairs withfullsource/staticreplay. Both35boardtags,allkept.
  WholeboardMnorm56.519/57.016mm,z-49.355/-49.727mm;3Ddifference.693704mm onlyrepeat.
  Individual35.2mm4cornerchain medianΔwhole26/27mm,p95~90/94mm,rotationmedian~3.7°,
  p95~13°; lowfiterrorambiguousdistinctbranches9/15outof70perwindowcamera-tagposes.
  Allbranchesretained,nophysicaldistance-based selection. Wholeboardparitygroups
  still2.573/3.420mm apart althoughheldoutpixels<.737px; originalsystematicissue
  notclosed. Reportcommon_board_joint_20260906_run1/static_analysis/RESULT.md.
- SAME-FRAME fix: no mathematical need to separate publicboard/mount visibility.
  New explicit mode excludesboardID1 beforemeasurement in bothcameras, keeps35other
  IDs and rawimages. Mount uniqueoutsideexpandedboardoutline, samephysicalrawalias
  matches only; missingboard, missingmount, overlap, extraID1 or otherduplicate block.
  Realdecoder synthetic legacyfilledjunction board+1border mount separates correctly.
  Initial isolated-marker synthetic failed on nestedduplicates as expected; corrected
  fixture to actuallegacyfilledjunctionlayout, no detectorgaterelaxation. Actualold
  board-onlyraw detects35/19..24board but no mount instead of mistaking boardID1.
  H identity thresholds distinctfromcalibrationresidualgates; noKD orphysicalsizefit.

- Corrected missing-board-spec assumption: UMI formal calibration asset explicitly
  documents physical blacktag3.52cm and gap1.056cm; released1.0.2 copy identical
  SHAe0cb280349fa0ca8792462f7864e818f138094890965f6cf4d6dc4413c0eb4bc. Current YAML and
  run4 captured target numerically match. Handoff§32 already confirmed same board.
  No need to ask user to send/remeasure specs again. Existing D405 stereoPASS is
  factory rectified IR1280x720, not present color intrinsic calibration; don't mix.

- Public-board distance audit confirms fixed 12–24 shared tags and correct transform
  direction; per-window whole-board inferred mount norm43.187–54.302mm, pairwise
  position18.130mm, not actual mechanical movement. Original pooled norm50.125mm
  is not z=-47.369mm. Same-view disjoint-tag H versus rigid prediction improves
  Ego25/26 (worst1.3245->.6044px), UMI24/26 (.7469->.3981px). Thus single-image
  metric-model mismatch exists under current board/factory assumptions; cross-step
  motion alone cannot explain it. No unique K/D versus corner versus board cause.
  Existing fixed-IR triangulated long grid spacings also differ+.7–1.0% from nominal
  board; neither reference is absolute metrology. Independent reference needed.

- Run5 close-depth (0.4205 m) revealed a real legacy-board localizer defect:
  coarse seeds 5–7 px inward cannot reach the outer corner in a fixed5 half-window.
  Opt-in half-window max(5,ceil(.15*median edge)) reduces tag15/35/23 temporal
  jumps from 9.742/8.076/11.412 to .528/.364/.336 px (relative tag temporal median).
  Same20-frame report RGB first-reference max drift 9.839 -> 1.103 px, still FAIL
  .75. Original frozen candidate after repair p95 4.041–4.992 px vs factory
  2.332–2.498 px: old compensating model cannot be deployed as general calibration.
  Original paired run4 same-ID raw replay: fixed5 worst 4.391012 px; scaled
  4.389197 px, both REVIEW. This bug is not the main cause of that old failure.
  Do not conflate software corner fix with resolving original two-chain 5.631 mm.

- Closerattempt IRdepthmedian.63357m (ego_stereo_color_probe_20260906_run4),
  comparedprior~.77–.80m. Frozenk1+SE3all20p95 .59838–.93444px, factoryoriginal
  2.33669–2.55237px. ButstaticdriftmaxRGB1.97865px, IRL1.34618/IRR1.41855px,
  15/20staticfail; image showsboardhandheld. OverallFAILmustremain, cannot
  retainonly5staticpassframesorrelaxgateafterseeingresults. Pixelimprovement
  isdiagnosticevidenceonly, notuniqueerrorcauseorvalidationoforiginalmount.

- Thirdposefrozenprediction FAIL, notmotion/noiseframe: all16p951.033–1.276px;
  primarypercoordinateRMS x.603/y.416px vsfirst2posesx.165/.120,y.266/.101.
  Same-tagtemporalpercomponentRMS generally.07–.12px, coherentregionalbias
  muchlarger. Depthdirectiondecomposition orthogonalRMS.402/p95.711px;
  along-raydepthchangealonecannotremoveallresiduals, butIRray/extrinsic/localizer
  remainpossiblecontributors. NativestereopatchNCC(firstvalidframeofeachpose,
  11/19pxpatches,±2pxhorizontal) findsno largecornerdisparitymismatch: thirdpose
  |delta disparity|p95 .074/.099px, minimumcorrelation .9982/.9969, noedgehits;
  diagnosticcorrectedRGBp95 remains1.168/1.158 vs1.195px. Donotdeploypatchshifts.
  ReviewerIR-only44sharedcornerlongdistance ratiosnear1 acrossposes, butmedian
  depths~.798/.768/.778m aretoo similar toexcludeconstantdisparitybias. No basis
  todeclarefactoryRGBuniquelywrongororiginal5.631mmmountproblemfixed.

- Secondindependentboardpose (ego_stereo_color_probe_20260906_run2):20triples,
  11validusingfrozenIRdecoder/noID1policy, no detector development samples.
  FactoryRGBp952.427–2.577px. Frozenrun1factorySE3p952.776–2.942px;
  frozenrun1k1+SE3p951.588–1.757px, bothfail1px. Reviewerexactindependentreplay.
  Thisrejectsblindone-poseextrinsiccompensation, notproofagainstallk1models.
  Exploratorytwo-poseSE3onlyfit wfixedk1 improvesoddtagheldoutbelow1pxinboth:
  [.664,.256] withstep1e-5 / [.667,.254] with2e-5. Thirdposemusttestunchanged
  candidatebeforeacceptance; bothcurrentposeswerealreadyusedforfit. IRbias
  remainsconfounded. No claimRGB-IRcorrectionalonefixestwo-external-chain5.631mm.

- Independent3stream capture now exists: ego_stereo_color_probe_20260906_run1.
  User confirmedmanufacturerhardboard.20framesets/60PNG verified, IRstampsidentical,
  baseline50.04744mm, extrinsicdirection/columnmajorserialization reviewed.
  On9nondevelopmentvalidframes, nativeIRtriangulation→factoryRGBp95 2.044–2.179px,
  whileIRepipolarprojectionp95.063–.096px. This bypasses printedsize/boardplane,
  but lowepipolarerror doesNOT bound horizontaldisparity/depth bias. FactoryRGB
  meanresidual approximately(+1.16,-.90)px. IRtriangulatededge medians35.56–35.62mm
  andplanerms.72–1.12mm are NOT independentmanufacturing metrology.
  FrozenpairedboardEgo k1 with unchangedRGB-IRfactoryT worsensRGB4.314–5.403px;
  singlepose refittingT canreversecomparison, demonstrating confounding rather
  than validatedlensrepair. Correct centraldifference localfit oddtagheldout
  worstfactory1.091px / k1 .779px (2xstep1.090/.783); noactivation.
  IMPORTANT: original frozen_candidate_analysis localfit wasincorrect due to
  SDKfloat32 defaultfinite-difference noise. Keepit asnegativeprovenance, usev2
  andstep2. FixedT projections unchanged. AddedSDKSE3recoveryregression.
  Remaining rootcause cannotbeuniquelyidentified fromsingleboardpose: need
  additionalimageposition/normal/distance whilecamerasfixed, not blindrepetition.

- D-only evidence: Ego k1=.056214958 reduces ALL10 paired holdout p95s, worst
  4.391->1.782px; UMI k1 worst4.347px. EgoD5 worst1.392px but k3 hits-.5bound,
  not deployable. PriorEgo clips fixed-k1: normal RMS.34747->.32931 but p95
  .61687->.67168 (worse); rotated RMS.41950->.35666, p95.76933->.67122.
  Reviewer verified matching supports (6252/4856corners), no unique lens proof.
  Independent ONE-BIT external tags reuse unchanged archivedcorners/allpairs:
  initial chain full3D4.845308->9.118517mm WORSE; changedview5.630870->3.581477mm
  improves. Zero posegate rejections in both arms/both datasets. Thus thisk1 is
  NOT a validated general repair, despite matching most paired-board errors.
  New external replay initially failed missing config args then wrong selection
  field; corrected to family/allowedIDs + selected_index; no output existed on
  failed attempts, raw untouched. Current output ego_frozen_radial_external_tags_20260906_run1.

- Authorized K-only comparison completed. Final report:
  artifacts/spatial_bench/common_board_intrinsics_compare_20260906_run2/report.json.
  Same train8/holdout5/corners, fixed D/model/board; factory baseline worstp95
  4.391px; Ego focal5.860; UMI focal4.126; EgoK4 6.470; UMIK4 4.848. No bounds
  hit, no casepasses1px. Trainingcost falls128.22->62.14/58.98 for Ego focal/K4
  while unseenpose step3 worsens: not validated calibration, no factory-error proof.
  Doubling K derivative steps: maxheldoutp95 difference .003392px, maxKentry
  .019101px. SDKfloat32 optimum stopping remains caveat but does not change
  comparison conclusion. Distortion coefficients never fit, so don't claim all
  intrinsic/distortion hypotheses excluded. Source and currentcode hashes verified;
  original solverreport SHA unchanged.53tests passed, independentreview clear.

- Run4 further diagnostics (not acceptance): dense least_squares with identical
  train8 objective converges in6evaluations/cost128.2180461, X differs .000656mm
  from original; heldout unchanged. Not a sparse solver convergence problem.
  Intersection-of-all-six-frames ID subset:8windows usable (>=8tags), pairwiseX
  scatter .690–2.705mm/.073–.453deg;5windows too fewIDs and skipped, all original
  records retained. These subsets differ from productionmiddle-pair selectedIDs.
  Background LK ROI[20,20,180,300]: closure median displacement E[.166,.102]px,
  UMI[-.275,-.013]px, but p95 outliers5.342/3.172px and narrow repetitive-texture
  ROI mean this CANNOT verify fixed camera poses or reject tiny physical motion.
  Geometry-only essential fit (fixedK/D, train8 only, allcorners/noRANSAC) changes
  R .793deg/t-direction .340deg, heldout Sampsonp95 .808/.320/.629/.256/.233px;
  no metric scale. This weaker1D condition does not replace 2D cross-reprojection
  gate or prove board/K/D error; firstheldout worsens vsoriginal .690px.
  Triangulation using originalUNACCEPTED X: planeRMS .319–.527mm, perwindow
  median tag edges35.058–35.292mm against35.2mm model. Not independent metrology,
  does not prove board flatness/size or root cause. Further identical capture is
  not justified; proposed diagnostic-only intrinsics comparison, noactivation.

- Run4 IPPE diagnostic: checked both refined planar seeds for all13boardwindows
  in both cameras. They either converge to the same pose or the alternative has
  much worse source-only RMS (Ego2.768–8.463px, UMI33.482–47.620px).
  No near-equal alternate branch found in these26selected observations; no
  target-heldout residual used for branch selection. This narrows one hypothesis,
  not a general proof of correct poses or calibration. Solver report SHA256:
  191f51f5d52f0742b66c5c7948eddb5d5e53c615bdcdd1cf9d4d5f1453d9916f.

- Current run4 dense paired result is REVIEW, not a mounting solution: report at
  artifacts/spatial_bench/common_board_train_20260906_run4/solve/report.json.
  Board normal span18.594deg / center span293.951mm pass diversity, but perwindow
  independently inferred X differs3.147–8.945mm across training (5mm gate).
  Heldout Ego->UMI p95=[2.765,2.381,2.377,3.944,4.391]px forsteps3,6,9,12,16;
  reverse=[1.421,.892,.987,.977,.915]px. Closure error alone cannot prove movement:
  similar directional errors already precede mounting stage. Step6 vs7 is only
  .430mm/.167deg, failing independent-pose novelty; do not delete or relabel it.
  Mono-only fits on same13selected pairs: Ego RMS .317–.492px/p95 .538–.955px;
  UMI RMS .157–.283px/p95 .268–.566px. Ego tag edges~38–42px vsUMI55–69px.
  Thus each view can fit its own planar pose substantially better than one commonX
  predicts across views. This does NOT isolate factory K/D, target nonplanarity,
  pose uncertainty, or physical camera motion. Raw attempt016 pair visually has
  no obvious motion smear, with upper board cropped inUMI; no absolute accuracy
  claim from appearance. Mount local p95 .402–.425px does not validate its M.

- Active mounting false-board bug: run2 reachedstep14 withslots0..12accepted.
  Attempt24/25 board2 andmount1 bothdecodedID1 atsame physicalquad (outer~95px vs
  inner~80px); unconditional board-detection existence triggered removal alarm.
  New v3 identity filter checks exactID1 plusIoU>=.65 andcenter<=.1edge, only when
  unique1-bitmount present; doesnotignore ID1 elsewhere orotherboardtags.
  User has now stopped but reports moving between board andmount stages; awaiting
  which object moved. NO continuation/calibration acceptance until clarified.
  If either camera relativepose changed, oldX cannot beused forlatermount.

- Follow-up real run1 first-step failures:3 attempts ×6 pairs, all setup/index/PNG
  hashes verified. Ego counts32–36 and UMI22–31 fluctuate; max per-tag drift
  (first observation to any subsequent observation) E0.412/0.448/0.387px,
  U0.438/0.446/0.344px, all below unchanged0.75px gate. First-frame overlap
  minima E28/31/31, U19/17/20, each spans>=5rows/cols. Complete-window ID
  intersections U10/5/8 are unnecessarily restrictive. Viewing UMI attempt1
  frame2 shows full visible legacygrid with image noise, no large occlusion.
  Root cause confirmed: exact ID-set equality conflated detector flicker with
  motion; no evidence to blame user movement. Fix plan:version2 anchor overlap
  >=12tags/3rows/3cols eachframe; check ALL reobserved tags against their first
  observation, never prune by drift; solve only reobserved middle-pair commonIDs.
  Keep v1 raw/report unchanged and compare offline; no camera startup.

- 2026-09-06 implementation follow-through: source-only UMI board-pose novelty
  must be checked in addition to timestamp/slot holdout. Review reproduced a false
  board PASS when four holdouts repeated training pose0. Fixed: >=20mm center OR
  >=7deg normal difference versus every train/earlier-holdout pose, final closure
  exempt only from novelty. Holdout faults never alter X; noisy synthetic truth,
  SDK inverseBrown, camera-motion, independent fixed-M prediction and fake-hardware
  capture/cleanup/load verified. No actual metric/HIL acceptance. Software route
  uses 8 train +4 heldout board windows,3 mount windows,1 heldout closure; allcode
  and output here on main, factorygeometry immutable, activation NOT_ACTIVATED.

- 2026-09-06 user asks open-source solution to original5–6mm discrepancy: reviewed official AprilTag/apriltag_ros bundle code, multical fix_intrinsic/Calibration.bundle_adjust, TagSLAM bodies/caveats and IPPEauthor. Proposed independent known-geometry commonAprilGrid + paired static observations + fixed-factory jointX, then independentmountM estimation/heldout, NOT merely anotheroptimizer. Existing scattered-tagBA already failed5px/7.168px and80mmexternal-only3.6506px holdout; retain failures. Fourrecentmonocularclips are diagnostic only, cannot pair matchingfilenames into extrinsics. TagSLAM requires independentmovingbody perEgo/UMI, notconstantthreecamerarig; duplicateIDs/mixedborder andSDKinverseBrown needadapters. Proposal only, notimplemented. See docs/acceptance/opensource-spatial-calibration-route-20260906.md.

- 2026-09-06 fixed-factory board analysis: all4clips captured,200raw images/146preview-good; original ArUco reference corners yield heldout RMS2.362/2.453/2.319/2.337px and training RMS~2.3px, characteristic inward-per-corner bias in both cameras. Physical corner-order convention independently checked on excluded normalcontrol `board_factory_ego_rot180_20260906T0028/ego/000000.png`, SHA2560d59256fa9515fa7dc3ad3644336182a7e1a6724197f4cb73a2327efe6487d7d:36/36tags match fixed(+,-),(-,-),(-,+),(+,+). This is not metric target-size fitting.

- New narrow hypothesis: OpenCV4.10 ArUco relativeCornerRefinmentWinSize0.3 clamps nominal5px subpixel window to~1px on these two-bit markers; contour corners stay inset at filled blackjunctions. Explicit cv2.cornerSubPix(5,5) on development Ego-normal sample0 changes heldout2.28239→0.27874px with factoryK/D unchanged. Synthetic known50px tags/gap15 with filled blackjunctions reproduces2.91548px error before and0.000043px after. Negativecontrol with tiny5px isolated cornerdots instead of actual filledjunctions worsens2.915→3.519px: NEVER generalize this refinement to separate one-bit spatialtags. Added diagnostic-only refine5 option, same decodedIDs/gates; remaining199samples confirmation pending.30tests pass; no production detector or calibration activation changed.

- RequestedEgo180clipboard_factory_ego_rot180_20260906T0028 actuallystillnormal:38/38preview-gooddecodedlatticecolumnright/rowup, ID0below30; metadataorientationisoperatorlabelonly. All50imagesintegrityPASS, validassupplementarynormalposesbutnot180control. No inferenceaboutboard-attachedvsimage-attachedbiascanuseitasa180sample. Explicitsidecar preservescorrectionwithoutrewritingrawmetadata.

- FirstfixedfactoryEgo normal30scliphas50validPNGs and46previewgood butlimitedcentercoverage(middle/right). Frames0/20nearidenticalview, later40/49differentview; don'tequate50imageswith50independentposes. Laptopdisplaysboardcopyinlaterframes, potentialduplicate-source(notprovenreasonfor4zero-tagframes). FactoryK/DexactpriorEgo colorbaseline, noimageintegrity/timestampregression failures. Heldoutpredictionstillpending.

- 2026-09-06 operatorapproved fixed-factory board diagnostic is distinct from rejectednewintrinsiccalibration. Four30s clips (Ego normal/board180,UMI normal/board180), independentcameraimages, unchangedfactoryK/D. FirstEgo normalpreviewfresh35/36/min41.19px; no captureyet. Intendedcheckerparityheldouttags must not influenceposefit/branch/layoutselection. Board180label isoperatorprovided; validateactualdecodedlayoutoffline. Timing/previewgood isnotaccuracyacceptance.

- Factorymodelaudit2026-09-06: Same2sealed color1280x720sessions, complete598+598acceptedcorners. Loader preservedK/Dnumbers but discardedSDK inverse_brown_conrady enum andimplicitlyusedOpenCVstandardBrown. NativeSDKraymaxactual0.0041px, fullfield627samplemax0.00794px; individualtagtranslationchange<0.0018mm. Twochainfullnorm4.845943→4.845308mm and5.630932→5.630870mm, so confirmedmetadata/modelbug isnot5–6mmrootcause. Newmodel-aware inverse path callsinstalledSDKpuregeometry (nohardwareconnection), keepsfactoryarrays unchanged, normalizedIPPE plusSDKpixelreprojection; unknownmodelsreject. D435iDzero, noIR/colorKmix found. Userrequiresfactoryauthoritative, do notfitnewK/D or resumeAprilGridcalibrationwithoutnewchoice.

- Correction after realboard inspection2026-09-06: Previous synthetic1-bit fixture was incompatible with oldaprilgrid library's2-bit decoder;73px versus60px is not proof thelibrary mislocatesrealboardcorners. Realphysicalboard oldlibrary recognizes36/36. NewOpenCV defaultborder1 fails0; markerBorderBits2 plus contour/SUBPIX detects live36Ego/34UMI, whereas APRILTAG quadsegmentation misses many at blackcornerjunctions. Keep this calibrationboard frontend distinct from1-bit externalalignmenttags. FormalD405 calibration_assets/aprilgrid_6x6_35mm.yaml explicitly records physical35.2mm/10.56mm and oldmanual saysreuseexistingboard; user sayssameUMISLAMboard, no needrequestmeasurementagain.

- 2026-09-06: New color AprilGrid preview real-image regression is necessary beyond mocked lifecycle tests. Installed aprilgrid detector on1280x720 rendered6x6/60px board detects3/36 with default1000px resize; disabling resize restores36/36 but reported minimumedge73px instead of60px. Isolated new capture entry uses OpenCV36h11 APRILTAG corners, recognizes36/36 at allfour rotations and58..61px blackedges. Preview lattice uses tagcenters only, not a metric calibration acceptance. Existing2UQ2 capture untouched. Physical board6x6/35.2mm/10.56mm remains unverified.

## Read-only D405 product baseline

- `/home/robot/ego_vio_humble` resolves to `/home/robot/releases/ego_vio_humble/product_v1_20260824`, branch `release/humble-stm32-product-v1-20260824`, HEAD `a7a143df9a138ada481e6c234b03803ab0cae837`.
- The external tree already contains user modifications and an untracked firmware header. It must remain untouched; no status cleanup, build, or checkout is permitted.
- `config/product_live_stm32/vins_config.yaml` freezes `estimate_extrinsic: 0`, `estimate_td: 0`, `td: -0.009312`, and calibrated `body_T_cam0` / `body_T_cam1` 4x4 matrices.
- D405 IMU samples are rotated into the VINS body/level convention with `R_level_from_imu`; calibration candidates are rejected unless orthonormal with determinant +1.
- Runtime pose quaternions are `xyzw`. The local VIO stub explicitly documents the pose quaternion as body-to-world, so the spatial simulator will use `T_world_body` and reject ambiguous transforms.
- Camera/IMU timing uses image acquisition time plus signed `td`; ROS/DDS delivery ordering and callback arrival are not the fusion time.
- D405 output odometry is an arbitrary local VIO world. It provides relative motion but does not by itself establish the Ego world transform.

## Immediate mathematical boundary

- `T_A_B` maps coordinates from frame B into frame A.
- Direct Ego observation yields `T_W_U(t) = T_W_E(t) @ T_E_U(t)`.
- Arbitrary-origin UMI VIO is anchored by `T_W_V = T_W_E(t0) @ T_E_U(t0) @ inverse(T_V_U(t0))`, then propagated as `T_W_U(t) = T_W_V @ T_V_U(t)`.
- Left and right UMI must use independent anchors/fusion states while sharing the same Ego trajectory.

## Open-source alignment evidence

- Corrected topology after re-reading the HoMMI paper and iPhUMI iOS source: HoMMI uses three iPhones simultaneously (left gripper, right gripper, head). ARKit multi-device collaboration establishes one shared AR world during capture; the head does not observe tags to spatially register the two hands.
- The iPhUMI app enables `isCollaborationEnabled`, exchanges `ARSession.CollaborationData` over MultipeerConnectivity, elects one host, shares a named identity world anchor, and calls `setWorldOrigin(relativeTransform:)` on non-host devices. Each device then records `frame.camera.transform`, which Apple defines as the camera pose in world coordinates.
- iPhUMI offline alignment intersects the left/right/head time ranges and resamples their already-common-world poses at 60 Hz. Fixed per-device calibration then composes `W_T_I @ I_T_TCP`; it is not an offline map merge.
- The AR tags used by iPhUMI gripper processing estimate finger/gripper width. They are not the mechanism that creates the three-phone shared world.
- Our 2UQ2/D405 hardware has no ARKit collaborative map. Temporary top tags are therefore an explicit substitute/bootstrap: Ego observations anchor each independent UMI VIO into Ego world. This is functionally analogous to a shared-world bridge but must not be described as the original HoMMI algorithm.
- HoMMI/MoF (`gsanpark/mof_hommi`) also intersects valid time ranges, samples poses at video acquisition times, preserves invalid/lost samples, and applies explicit fixed device/TCP transforms.
- HoMMI's QR time-sync stage estimates an explicit device offset from repeated observations and reports dispersion. We will preserve actual timestamps and skew rather than replace one device's timestamps with another device's merely because they are close.
- Stanford UMI calibration composes `T_slam_tag = T_slam_cam @ T_cam_tag`, associates observations by timestamp, filters implausible detections, and selects a robust representative from repeated transform candidates.
- iPhUMI documents multiple independently recorded phones sharing a world frame plus an optional third phone. This is architecture evidence for independent per-device alignment chains, not evidence that timestamps or origins may be shared implicitly.

## Fixed simulation decisions

- D405-style `body_T_cam` is `T_body_camera`; therefore `T_world_body = T_world_camera @ inverse(T_body_camera)`.
- Ego world initialization selects the first healthy Ego sample at or after recording start, makes its translation the origin, maps measured gravity to world `-Z`, and uses the projected Ego forward axis to fix zero yaw.
- Each UMI owns an independent `T_world_vio` anchor. Future left/right operation shares Ego world and orchestration only.
- A UMI observation stamped `d` later than acquisition is evaluated at `stamp - d`; callback/arrival time is not accepted as fusion time.
- Evidence can only report `PASS/simulation`; physical latency, calibration accuracy, and ROS 2 HIL remain unverified.
- Independent review found no blocking mathematical issue. It identified two pre-HIL risks now fixed: interpolation requires an explicit maximum pose gap, and Ego physical forward is an explicit frame-contract vector rather than a hard-coded body +X assumption.
- Review also requested stronger evidence now added: off-grid SO(3) interpolation, signed negative delay, camera/body rotation round-trip, and identical left-chain output whether right simulation is present or absent.

## Primary references

- https://github.com/gsanpark/mof_hommi
- https://raw.githubusercontent.com/gsanpark/mof_hommi/refs/heads/main/hommi/demonstration_processing/process_stages/align.py
- https://raw.githubusercontent.com/gsanpark/mof_hommi/refs/heads/main/hommi/demonstration_processing/process_stages/timesync.py
- https://github.com/real-stanford/universal_manipulation_interface/blob/d095ba9590df789df5189eea5ee7e431689038a6/scripts/calibrate_slam_tag.py
- https://github.com/real-stanford/iPhUMI
- https://github.com/real-stanford/iPhUMI/blob/main/ios_app/iPhUMI/DataCollection/ViewController.swift
- https://github.com/real-stanford/iPhUMI/blob/main/python_package/iphumi/demonstration_processing/process_stages/align.py
- https://github.com/real-stanford/iPhUMI/blob/main/python_package/iphumi/demonstration_processing/utils/gripper_util.py
- https://arxiv.org/html/2603.03243v2

## 2UQ2 Ego calibration research (2026-08-27)

- Correct device ownership: `/home/robot/ego_vio_humble` is the read-only D405/external-IMU product baseline and is not the 2UQ2 Ego implementation.
- The owned 2UQ2 adapter requests one side-by-side MJPEG frame at `3840x1080@60/1`; the two image halves must be decoded/split without changing the frame acquisition timestamp. The formal health gate requires at least 59 Hz.
- On every 2UQ2 video callback, the adapter reads one 27-byte XU packet: a 24-bit video sequence followed by two 12-byte six-axis groups. Existing code calls them `main` and `backup`, but the vendor manual only says “two groups of the same type”; the physical/temporal meaning and phase relative to exposure must be measured rather than inferred from those names.
- Kalibr camera and camera-IMU calibration commands are already installed under `/home/robot/.local/bin`; no equivalent Ego dataset exporter or stereo splitter currently exists in this repository.
- The shortest candidate toolchain is Kalibr for stereo intrinsics/extrinsics plus camera-IMU spatial/temporal calibration, followed by an independent 2UQ2 stereo-inertial estimator in this repository. Selection and exact data contract remain under primary-source review.
- The installed Kalibr commands are wrappers around the pinned container image `ego-vio-kalibr:1f602274-minimal`. Camera calibration supports a two-topic chain, an Aprilgrid, explicit camera models, approximate synchronization tolerance, and feature subsampling. Camera-IMU calibration has temporal calibration enabled by default with a default ±30 ms time-offset search range.
- Kalibr's official requirements are stricter than “record and run”: correct low-jitter same-clock timestamps, prior IMU intrinsic/noise characterization, low motion blur, and smooth motion exciting all IMU axes. Official examples cite 20 Hz camera / 200 Hz IMU; OpenVINS guidance recommends roughly 200–500 Hz IMU and at least one translation plus two orientation degrees of excitation.
- The current 2UQ2 adapter exposes one XU sample per 60 Hz video callback. Before selecting Kalibr or declaring stereo-inertial SLAM ready, HIL must determine whether the XU interface can deliver a higher-rate buffered stream or only one frame-linked 60 Hz sample. A nominal 60 Hz stream is not silently relabeled as 200 Hz.
- Kalibr and OpenVINS both require inspection of the IMU inter-sample-time plot; burst delivery or gaps invalidate camera-IMU calibration even when average rate looks correct. Camera calibration should target <0.5 px reprojection error as a provisional gate, with <0.2–0.5 px described as good by OpenVINS.
- Official IMU noise guidance calls for Allan-deviation estimation from a long stationary record (Kalibr: approximately 15–24 h; OpenVINS: upwards of 20 h), producing gyro/accelerometer noise density and bias random walk in SI units. A short static capture can check bias/axes tomorrow but cannot replace the full noise characterization.
- Kalibr requires a ROS1 bag. Its official file-based path is suitable here: timestamp-named `cam0/` and `cam1/` images plus `imu0.csv` with nanosecond timestamps, angular velocity in rad/s, and acceleration in m/s^2, converted using `kalibr_bagcreater`. This avoids rewriting raw capture or treating arrival time as acquisition time.
- The 40 mm left/right UMI identity tags are not a camera calibration target. Kalibr recommends a rigid, flat, remeasured Aprilgrid with a white border; all unrelated AprilTags should be hidden during calibration.
- For camera-only stereo calibration, Kalibr recommends moving the target through different image regions/orientations while the camera system is fixed and subsampling to roughly 4–10 Hz. The left/right topic ordering defines `cam0` and `cam1` and must match the physical lens-order contract.
- OpenVINS has a native ROS 2 path and consumes Kalibr camera-IMU and IMU-chain YAML. Upstream VINS-Fusion supports stereo+IMU plus online spatial/temporal refinement, but its official upstream is ROS1; any ROS2 use here must be an independently owned integration in this repository rather than a runtime dependency on the read-only D405 release.
- AnySearch transiently failed to extract Kalibr's YAML-formats page on the first attempt; the other primary pages and local installed-command help were available. Exact transform direction must be checked from the generated YAML and official format definition before conversion into an estimator config.
- The retry of Kalibr's official YAML-format page succeeded. `T_cam_imu` maps IMU coordinates into camera coordinates (`T_C_I`), `T_cn_cnm1` maps the previous camera into the next (`T_C1_C0`), and `timeshift_cam_imu` uses `t_imu = t_cam + shift`. An estimator requiring `body_T_cam = T_I_C` must use `inverse(T_cam_imu)`; this conversion needs a unit test before runtime use.
- Only the two calibration executables and validator have host wrappers; `kalibr_bagcreater` is not currently installed as a host command. The future derived-data exporter must either write the supported static-folder/CSV input and invoke a pinned bag creator in the container, or produce a verified ROS1 bag directly.
- The current immutable raw layout is sufficient for deterministic export: `ego.video.jsonl` and `ego.xu.jsonl` carry offset, size, acquisition timestamp, validity and the shared vendor sequence, while the corresponding `.bin` streams preserve the MJPEG and 27-byte XU payloads. Export must split the decoded side-by-side image into two 1920x1080 images with the same timestamp and convert `g -> m/s^2` and `deg/s -> rad/s`.
- One physical IMU frame is sufficient for the Ego chain: calibrate `T_C0_I` together with stereo `T_C1_C0`, then derive `T_C1_I = T_C1_C0 @ T_C0_I`. The two XU groups must not be modeled as separate IMUs unless hardware evidence actually proves two physical sensors.

## Connected 2UQ2 HIL discovery (2026-08-27)

- Host is Ubuntu 22.04.5 / ROS 2 Humble, repository `main` at `73b1d4f74b2d53914696cc6e160462b2313e6120` with only the previously recorded untracked project artifacts.
- The connected Ego is unambiguously `/dev/video0` and `/dev/video1`: USB VID:PID `1bcf:0b15`, vendor `YLX-260701-W`, model `2UQ2`, reported serial string `01.00.00`, UVC driver, USB path `pci-0000:80:14.0-usb-0:10.3:1.0`.
- A separate Intel RealSense D435i serial `349643060321` occupies `/dev/video2`–`/dev/video7`; it must not be selected for Ego calibration.
- The 2UQ2 is currently enumerated on a USB 2.0 480 Mbit/s branch, while the D435i is on 5 Gbit/s USB. This is sufficient for compressed MJPEG only if the advertised mode is present and measured; it is a bandwidth risk to record in acceptance evidence.
- `v4l2-ctl` is not installed (`command not found`). Continue with existing ffmpeg/GStreamer/ioctl/OpenCV tools first; do not install system packages merely for enumeration.
- GStreamer/V4L2 reports the 2UQ2 capture node as `/dev/video0`; `/dev/video1` is the companion metadata-only node and is not advertised as an independent camera source.
- The device advertises side-by-side `3840x1080` MJPEG at exactly 60 Hz, not 30 Hz. Other MJPEG modes are 60/120/210/570 Hz; raw YUY2 at 3840x1080 is only 1 Hz on the current USB 2.0 link. Requesting downstream 30 Hz with `videorate` would only discard frames and is not evidence that the camera sensor is configured to 30 Hz.
- The user's required calibration is the integrated stereo–IMU calibration analogous to D405, not a camera-only checkerboard pass. A static board is insufficient for `T_C0_I`; the correct dataset must combine a rigid calibrated target with synchronized six-degree-of-freedom camera motion and a true high-rate IMU stream.
- The read-only D405 baseline confirms the intended method: a rigid 6×6 AprilGrid (35 mm tag geometry), Kalibr stereo/camera–IMU joint calibration, explicit `T_cam_imu`/`timeshift_cam_imu` conversion, and dynamic AprilGrid validation. D405 runtime freezes the consensus result rather than continuously estimating it.
- The authoritative read-only D405 calibration kit is `/home/robot/ego_vio_calib_kit`, branch `release/calibration-product-v4-20260824`, HEAD `4b1689ab994d96de229c319d833f7c99073e806a`; it already has substantial user modifications/untracked evidence and will not be touched.
- D405 stage 5 runs two independent AprilGrid + stereo + raw-IMU attempts, compares them, and gates rotation repeatability ≤0.5°, translation repeatability ≤5 mm, time-offset difference ≤3 ms, per-camera mean reprojection ≤0.5 px, gyro residual mean ≤0.02 rad/s, and acceleration residual mean ≤0.25 m/s². The 2UQ2 procedure should preserve this method but use measured 30/200 Hz contracts and 2UQ2-specific intrinsics/noise rather than copy D405 numbers.
- The 2UQ2 USB descriptors expose XU unit 3 (GUID `d5c2f42c-1808-4d9f-be56-753e271c9244`) with three controls and unit 4 with 25 controls. Unit 3 selector 1 is a 27-byte sequence-plus-two-six-axis-groups packet; only the added Linux demo—not the manual—names the groups main/backup. No supplied selector is documented as factory calibration data.
- The vendor IMU demo calls `GET_CUR` repeatedly and labels the 24-bit counter a frame sequence, but its shipped `FRAME_INTERVAL_MS` is 1000 ms and it provides no device timestamp. This code is not evidence of 200 Hz delivery; sequence behavior must be measured while streaming.
- Live no-video test: 400 XU reads paced at 200 Hz over 1.993 s completed at 200.218 calls/s, but all packets had sequence 0. The built-in IMU/XU state does not advance while the UVC video stream is stopped.
- Initial live streaming test: with `3840x1080 MJPEG@60` active, 600 attempted XU polls took 4.585 s; 224 unique video sequences were observed, with sequence deltas mostly 1 and some 2. This first test measured only the frame-linked sequence and was insufficient to determine the 24-byte sensor-payload refresh rate; the corrected payload-level result is recorded below.
- The corrected payload-level test shows that faster polling does return new current sensor values between video-sequence increments, but the published blocking XU path still tops out at about 100 reads/s. This does **not** limit the physical IMU's internal ODR; a streaming/FIFO interface or manufacturer support is required for a true 200 Hz host dataset.
- The installed bridge is `/opt/three-device-slam/lib/libtwo_uq2_xu.so`; its source is owned by this repository and calls only unit 3 selector 1. The local source still hard-codes the camera to 60 Hz and associates one XU read with each image callback.
- Read-only XU enumeration found unit 3 selector 2 is a 1-byte control with current/default 1 and range 0–6; selector 3 is a 1-byte control with current/default 1 and range 1–4. Their meanings are absent from the supplied SDK, so they must not be written by guesswork. Unit 4 exposes generic Sunplus controls, including opaque 64/1024-byte buffers, but no local documentation identifies any as stereo intrinsics/extrinsics.
- A direct GStreamer negotiation request for `3840x1080 MJPEG@30` fails immediately with `not-negotiated`; the same node successfully captured `3840x1080 MJPEG@60`. This confirms the current firmware does not expose native 30 Hz at full side-by-side resolution.
- First live full-resolution frame was preserved at `artifacts/ego_calibration/20260827_discovery/ego_sbs_3840x1080.jpg`, SHA-256 `e471e977e7713ba238c425bd39e23f4a41f63ef951ba4ec8bba21cb9c6a9ed71`. Visual inspection confirms a valid SBS image but both lenses are almost completely occluded by a very close dark object; no calibration target or scene features are usable.
- Public web/code searches found no authoritative mapping for 2UQ2 unit-3 selectors 2/3 and no public factory-intrinsics reader. The unit-4 GUID appears in unrelated generic Cypress FX3 extension-unit examples, so its opaque buffers cannot be treated as 2UQ2 calibration data without a manufacturer register map.
- Re-check after user challenge: the official/store product material confirms an integrated six-axis gyro and hardware frame synchronization but does not publish a host-visible IMU sample-rate/interface table. The supplied SDK archive remains limited to selector-1 current-frame data. The correct unresolved question is host export/buffering at 200 Hz, not whether the sensor silicon can internally run at 200 Hz.

## Ego training-calibration recovery (2026-08-28)

- Re-solving stereo from the higher-coverage `ego_pc_cam_imu_calib_20260828_run2` capture produced a 74.213 mm baseline. Kalibr used 95 of 532 processed 5 Hz views after final outlier filtering; the missing Cairo camera-report dependency did not prevent it from writing the camchain and text results.
- Independent validation used the interleaved odd 10 Hz export frames excluded by the 5 Hz solve. A fixed per-camera AprilGrid homography-RANSAC rule removed geometrically inconsistent tag decodes before evaluating rectified correspondences. Across 511 frames and 14,501 accepted common tags, vertical error was 0.287 px median and 0.927 px p95, passing the frozen <=1.0 px gate.
- Re-solving cam0-to-IMU with the accepted cam0 intrinsics converged to 0.584 px mean reprojection, 0.00565 rad/s mean gyro residual, 0.0487 m/s^2 mean acceleration residual, a 43.55 mm lever arm, and `t_imu = t_cam - 2.129 ms`. Relative to the previous numeric solution, the transform changed by only 0.319 degrees and 0.633 mm.
- `ego_calibration_training_v1.yaml` is calibration PASS and internally usable for training metadata. Transport remains explicitly provisional: the source capture measured 186.608 Hz and 8.353% inferred IMU drops, IMU random-walk values remain conservative priors, and no second independent camera-IMU motion capture has yet established repeatability. It is not a final release/safety-control calibration.

## Vendor RAR/manual evidence (2026-08-27)

- User supplied `/home/robot/下载/c57d3dfd20f5d6926363ac8cafc05c15_729238008267210.rar`, RAR5, 2,361,196 bytes, SHA-256 `35e7156c62a4eb182535894745c51e051816f82990bdd89d49114388d4273df4`.
- The archive is unencrypted and contains a 3-page vendor PDF, AMCAP, UVCXU-V1.9, and `YLX_XU_API_2026721.zip`. Only the PDF and SDK ZIP were extracted into this repository; neither Windows executable was run.
- The vendor manual explicitly states the physical accelerometer runs at 800 Hz and the gyroscope at 400 Hz, with ±8 g and ±2000 dps ranges. The earlier shorthand “no 200 Hz IMU” was wrong for internal sensor ODR.
- The same manual explicitly states that the host-facing XU payload is 27 bytes, selector 1, with a 3-byte sequence bound to the video frame and two 12-byte sensor groups; it is sent continuously “according to frame rate.” If a video frame is lost, that frame's sequence/data is lost too. It also says video must be opened before IMU data can be obtained.
- The manual says only that the final 24 bytes are “two groups of the same type,” 12 bytes per group; it does not state that two physical IMUs are installed. Its illustrated packet even shows identical accelerometer bytes across the two groups with slightly different gyroscope bytes, which is consistent with two temporal samples from one IMU. The added SDK demo's main/backup labels are therefore not sufficient hardware evidence.
- The manual prescribes MJPG 3840×1080 at 60 FPS as the default/full stereo mode. It documents external trigger up to the camera frame rate but no buffered/FIFO API for exporting the intermediate 400/800 Hz samples.
- Preserved vendor evidence: `artifacts/ego_calibration/vendor_docs/2UQ2_IMU_manual_2607.pdf` SHA-256 `fcd396b94806ac0789e2a6e4771d8481d357fc58860380814a04a150ad7bc32f`; SDK ZIP SHA-256 `1892f72c534c2efc140e4159f6975b19c6071abdf85093c95d1a2a1258c0c67e`.
- Every file in the RAR's SDK ZIP is byte-for-byte identical to the already installed read-only `/home/robot/vendor/2uq2/YLX_XU_API_2026721`; the archive contains no additional buffered-IMU implementation or register map.
- Live test of the repository's current per-frame capture path produced 421 structurally valid 3840×1080 frames but only 49.466 Hz by acquisition timestamps. It reported 89 sequence gaps and 2 duplicates (`gap_ratio=0.2114`) with no JPEG/XU/timestamp failures. The blocking `GET_CUR` in the video callback plus current USB path cannot pass the existing ≥59 Hz/≤0.1% sequence-gap gate.
- `/dev/video1` is V4L2 metadata capture only (`UVCH`, UVC Payload Header Metadata), not a second image or buffered IMU endpoint. The captured MJPEG APP markers contain AVI/JPEG encoding metadata only; no EXIF/focal/stereo calibration payload is present.
- Standard UVC descriptors report zero objective/ocular focal lengths and no factory-calibration interface. Factory intrinsics therefore remain unavailable through every documented/read-only local path examined; physical AprilGrid calibration or a manufacturer calibration file is required.

## D405 Docker2 / three-device runtime integration audit (2026-08-28)

- The afternoon tree `/home/robot/umi_docker_device2_d405_formal_20260828` and image `umi-ego-vio:device2-c48df736-d405-product1-base-20260828` implement a complete independent D405 + external STM32 acquisition, VINS-Fusion, loop-closure, health, trajectory CSV, and same-source raw-recording chain. The latest preserved candidate health report is `SLAM_HEALTHY`/`product_usable=true` with 1,110 raw and corrected samples over 79.34 s.
- Its immutable identity is `UMI_DEVICE_02_C48DF736`, D405 serial `260322279785`, STM32 `c48df736...`. It is not intrinsically a `left` device: the manifest declares `runtime_namespace: umi_device_02`, but `alignment_group_id: UNASSIGNED_PENDING_MULTI_DEVICE_ALIGNMENT` and `alignment_status: NOT_CALIBRATED`. A separate owned product configuration must explicitly bind that physical identity to the left role.
- Candidate SLAM was exercised, but the formal runtime is not installed: `/home/robot/umi_ego_vio_data_device2_c48df736/active_runtime_calibration` contains no files. Therefore current evidence supports the candidate chain only, not the formal `realtime` command.
- The declared runtime namespace is metadata, not active isolation. The live product scripts use absolute global topics `/cam0/image_raw`, `/cam1/image_raw`, `/imu0`, `/odometry`, `/odometry_rect`, `/imu_propagate`, `/pose_graph_path`, `/slam/health`, and a fixed managed Rerun port 9876. The launcher uses host networking and does not set `ROS_DOMAIN_ID` or apply a ROS namespace. Its own preflight rejects any second matching VINS/loop/viewer process. Two unchanged UMI Docker live chains therefore cannot run safely together on the host.
- The owned `three_device_slam` coordinator does not invoke either Docker launcher or `run_vins_realtime.sh`. It starts synchronized Python capture workers for left/right D405 and Ego, or one shared `rsusb_pair` owner for D435i Ego plus both D405 units. The D405 phase-one worker explicitly rejects `--publish-vins`.
- The owned AprilTag spatial implementation is currently a deterministic reference/replay layer. No coordinator node subscribes to Docker odometry or Ego SLAM, and no live node publishes left/right trajectories transformed into the Ego world. Thus current capability is simultaneous three-device capture, plus separately proven single-UMI local SLAM—not simultaneous three-SLAM Ego-world fusion.
- Minimum safe integration requires: explicit physical-role binding; one unique ROS namespace/topic/frame set and viewer endpoint per local estimator; a single common acquisition-time contract; an Ego SLAM pose input; left/right odometry adapters; live AprilTag observations plus independent tag-to-gripper mount calibrations; a quality-gated fusion/relocalization node; and one supervisor barrier with health/cleanup/recording evidence. Merely starting three containers does not satisfy fusion.

## Selector-1 payload-rate correction (2026-08-28)

- Re-testing compared the 24-byte IMU payload itself rather than treating the first three video-sequence bytes as an IMU sample counter. This corrects the earlier inference that selector 1 only refreshes one sensor sample per video frame.
- With video stopped, 600 `GET_CUR` calls paced at 200 Hz achieved 199.996 calls/s, but both six-axis groups remained constant. The video stream must be active for sensor data to refresh.
- With `3840x1080 MJPEG@60` active, all 400 first-group and all 400 second-group payloads were distinct. There were 165 payload changes while the video sequence was unchanged, proving that the current-value registers refresh between video frames.
- The same run achieved only 100.044 reads/s because each selector-1 control transfer took 9.779 ms median (11.148 ms p95). Thus the sequence is frame-linked, but the sensor values are not limited to one refresh per sequence.
- Switching the trigger stream to the advertised `1280x480 MJPEG@210` mode did not remove the control-transfer limit: 800 requested reads achieved 100.881 packets/s; both groups changed every packet; call latency was 9.564 ms median and 11.005 ms p95.
- Two independent file descriptors and two concurrent XU readers also remained serialized: 800 combined completions took 7.951 s, or 100.620 reads/s aggregate, while individual call latency rose to 19.672 ms median. Parallel polling cannot create a 200 Hz selector-1 stream.
- Group-correlation check over 300 static packets found the groups were never byte-identical, while their per-axis means matched closely. This supports—but does not alone prove—the user's physical-one-IMU statement and a two-temporal-samples-per-packet interpretation.
- Superseding X5 evidence invalidates the temporal-pair hypothesis. The PC bridge called `GET_LEN` and `GET_CUR` for every sample, while X5 uses one fixed-length `GET_CUR`; X5 observes 194–202 changes/s in bytes 3–14 alone. Bytes 15–26 are vendor-SDK backup data and must not be synthesized into a second timestamped sample.
- Current X5 no-motion poll preflight on the attached 2UQ2 measured 202.35 main-payload changes/s, 59.79 native video frames/s, and 30.19 deterministically retained frames/s. XU errors and duplicate main payloads were zero; IMU interval p99 was 6.47 ms.
- The current X5 preflight SBS image is not calibration-usable: both halves are severely defocused or occluded and contain only large dark blurred regions. Optical visibility must be corrected before any stereo or camera–IMU dataset collection.
- Direct AMCAP-equivalent OpenCV/V4L2 configuration accepted calls to set MJPG, 3840×1080, and 30 fps, but the device immediately read back 60.0 fps and delivered 240 frames at 60.170 Hz. The manual's Video Capture Pin paragraph says resolution can be changed for the current preview; it does not document a native 3840×1080@30 interval. The current firmware therefore requires deterministic every-second-frame retention for a 30 Hz full-resolution output.
- A dedicated-thread 10.009 s transport probe passed the requested final rate contract: 59.048 Hz hardware SBS, 29.574 Hz retained SBS output, 100.611 XU packets/s, and 201.222 single-IMU samples/s after expanding two wire-order groups per packet. It observed 2014 samples, zero XU failures, zero timestamp regressions, 5.000 ms median sample interval, and zero identical group pairs.
- This is a transport-rate PASS, not yet a camera–IMU timing-calibration PASS. The 5 ms intra-packet spacing and group order still require dynamic validation; current timestamps use the host-monotonic midpoint of each blocking control transfer, with the earlier group placed 5 ms before the later group.
- After the user selected native 60 Hz camera output, a second 10.005 s HIL run passed at 59.472 Hz full-resolution SBS and 202.505 Hz single-IMU output (101.252 two-sample packets/s). It captured 595 video frames and 2026 IMU samples with zero XU failures, zero timestamp regressions, 5.000 ms median interval, and 5.274 ms p95. This 60/200 contract supersedes 30/200 as the intended final configuration.
- The first real calibration writer failed despite the rate probe passing: serializing large JPEG writes and IMU writes under one Python lock reduced the 3.017 s run to 55.356 Hz video and 106.735 Hz IMU. A bounded single-writer queue removed disk I/O from both producers and restored video, but Python's per-byte JPEG structural scan still held the GIL and limited IMU to 163.310 Hz.
- A controlled run changed only the hot-path JPEG check from a full Python segment scan to constant-time SOI/EOI boundaries; the same immutable writer then passed at 59.567 Hz video and 202.687 Hz IMU. Full JPEG structural validation is therefore an offline task, not an acquisition callback task.
- Final 5.001 s HIL of the operator-facing calibration capture passed: 298 raw 3840×1080 MJPEG frames at 59.590 Hz, 507 raw XU packets at 101.383 Hz, and 1014 raw single-IMU samples at 202.766 Hz, with zero JPEG-boundary failures, XU failures, or timestamp regressions. Each stream has an append-only index, CRC per record, and SHA-256 manifest.
- The first 2UQ2 operator command was not operator-complete: unlike the read-only D405 collector, it opened no live image, did not split the SBS frame for visual confirmation, and gave no AprilGrid detection feedback. Transport PASS alone cannot establish calibration-target observability or dataset fitness.
- The read-only D405 reference keeps recording and quality/preview paths separate, uses a latest-only preview, overlays detected AprilGrid corners and per-stage feedback, and permits `q` to stop. The 2UQ2 implementation should preserve that separation because earlier Python work on every full-rate JPEG measurably starved the 200 Hz XU path.
- Preview HIL root-cause isolation: default first attempt processed 36 preview frames but reduced IMU to 181.135 Hz; no-preview restored 205.060 Hz; keeping decode/resize while removing the main-thread GUI pump held 201.835 Hz; moving the GUI pump to its own thread held 202.241 Hz without tag detection and 202.445 Hz with the real detector. The blocking HighGUI call in the XU loop—not JPEG decode or AprilGrid detection—caused the rate regression.
- Final unmodified preview HIL ran for 10.004 s with 595 video frames (59.479 Hz), 1012 XU packets (101.164 Hz), and 2024 single-IMU samples (202.328 Hz), zero timestamp/JPEG/XU failures, 49 processed preview frames, and zero preview processing errors. Transport passed; target observability correctly failed because no AprilGrid was in either half.
- Independent code review found four important gaps: late q/ESC could miss the final verdict, initialization failures could poison a new output directory, any t36h11 tags could satisfy the target gate, and average rates did not prove the planned ≤1% drop requirement. Added explicit regression tests and fixed all four before handing off the operator command.
- Real-grid load invalidated the first thread design: one run saw geometrically valid grid frames in only the right half and IMU fell to 179.675 Hz. A spawned detector process still failed when fed through `multiprocessing.Queue` because producer-side JPEG pickling produced paired 19 ms XU gaps; OpenCV thread limiting alone did not help. A two-slot shared-memory handoff removed producer serialization.
- Final shared-memory preview HIL ran 10.005 s with 595 camera frames (59.468 Hz), 1013 XU packets (101.245 Hz), and 2026 IMU samples (202.491 Hz). It recorded zero inferred video/IMU drops, 3.463 ms max video jitter, 1.957 ms max IMU jitter, zero timestamp/JPEG/XU/preview errors, and 52/52 preview frames processed without queue replacement. Target observability failed only because the AprilGrid was not visible in both halves.
- That shared-memory PASS was not stable across later device cycles: the exact current preview run fell to 172.324 Hz and a no-preview capture to 176.610 Hz; the independent rate probe also fell to 168.859 Hz with 27.934 ms p95 XU call latency. This isolates the current condition to the 2UQ2 XU/USB state rather than preview load. `usbreset 001/035` precisely selected the 2UQ2 (D435i is on Bus 002), but ordinary access was denied and `sudo -n` required a password, so no reset occurred.
- Follow-up review confirmed the original late-abort, geometry, lifecycle, and drop/jitter issues were fixed, then found child-death, process-start cleanup, and shared-memory resource-tracker edge cases. The current code detects unexpected child exit, never joins an unstarted process, and leaves shared-memory registration owned by the parent; focused verification is 113 passed.
- Physical 2UQ2 unplug/replug changed the device from Bus 001 Device 035 to Device 036 and restored the independent probe to 59.453 Hz video / 200.643 Hz IMU. A full no-preview recorder also passed at 59.456/202.650 Hz with zero inferred drops.
- Repeated preview failures were not caused by USB bandwidth or shared-memory copying: runtime inspection showed the spawned preview child creating 24 native workers on a 24-logical-CPU host. Importing its `cv2`, NumPy, and `aprilgrid` stack reproduced exactly 24 Linux threads even after `cv2.setNumThreads(1)`; OpenCV is linked to OpenBLAS.
- A single-variable HIL with `OPENBLAS_NUM_THREADS=1` restored preview transport to 59.493 Hz video / 202.375 Hz IMU with zero inferred drops. The permanent fix scopes that setting to the preview child before imports. A fresh ordinary-command HIL then passed transport at 59.456 Hz video / 202.251 Hz IMU, zero inferred drops/regressions/failures, 3.639 ms maximum video jitter, and 1.866 ms maximum IMU jitter. Overall report status remained FAIL only because no AprilGrid was present in either half; formal target observability remains pending operator collection.
- The first real moving-grid formal attempt (`ego_calib_run3_20260828`) was operator-aborted after 45.450 s and is not calibration-acceptable. Target observability itself was excellent: 215/220 sampled frames (97.73%) contained a geometrically consistent grid in both halves, averaging 33.26/33.20 tags. Transport failed at 59.493 Hz video / 181.297 Hz IMU, 10.996% inferred IMU drops, and 19.304 ms maximum IMU jitter.
- `run3` wrote only 19.0 MB/s versus 18.5 MB/s in passing empty-scene runs, and offline detector timing was 29.7 ms/frame with the grid versus 25.8 ms/frame without it. JPEG volume and per-tag decode alone do not explain the full XU loss. XU packet windows ranged from 81.2 to 96.2 Hz with many approximately 20 ms intervals, while the host exposes 24 independent logical cores rather than paired hyperthreads.
- A 15 s differential run at `--preview-hz 1` recovered average transport to 59.526 Hz video / 198.509 Hz IMU, and its final 10 s sustained approximately 101 XU packets/s, but strict acceptance still failed due to 1.716% inferred drops and 9.901 ms maximum IMU jitter. No target was visible in that run, so a real-grid reduced-cadence gate remains required before retrying formal capture. The evidence currently favors separating preflight target-quality inspection from the lossless acquisition hot path unless that gate passes.
- The first 90 s X5 stereo dataset preserved 59.463 Hz native SBS, 29.997 Hz retained SBS, and zero native sequence gaps, but its host-polled main IMU averaged 181.288 Hz. Its median XU interval remained 4.782 ms while 478 intervals exceeded 15 ms and 477 exceeded 20 ms (p99 23.865 ms, max 35.453 ms). The prior 5 s preflight had no interval above 7.5 ms. Because the X5 interface obtains each current sample through a synchronous `UVCIOC_CTRL_QUERY(GET_CUR)` with no timestamped FIFO, a short successful polling-rate sample does not establish sustained 200 Hz delivery; missed polling opportunities cannot be reconstructed.
- X5 JPEG write volume does not explain the IMU loss: the passing 5 s preflight wrote 47.3 MB (about 9.46 MB/s) and the 90 s run wrote 812.1 MB (about 9.02 MB/s); per-second IMU count versus average JPEG size had only 0.105 correlation. The remaining architecture risk is synchronous XU polling coupled to video dequeue/scheduling. A dedicated high-priority XU reader must pass a 90 s HIL gate before camera–IMU calibration data is accepted.
- The X5 reboot during transfer cleared both the `/tmp` dataset and deployed helper. The PC retained 547,422,208 bytes of video payload plus the full index. Sequential validation accepted exactly 1813 contiguous frames/60.400567 s at 29.999718 Hz; every accepted payload matched its stored CRC and JPEG boundaries. Frame 1813 was partial and excluded without modifying the source artifact. A deterministic 10 Hz AprilGrid scan found 258/605 (42.64%) simultaneous-valid stereo samples, below the frozen 60% observability gate despite useful position/scale diversity.
- PC source audit reconfirmed the already-measured bottleneck: `native/2uq2_xu_bridge/bridge.c::ylx_read_imu27()` issued `xu_get_len` followed by `xu_get_cur` for every sample. The Python probe and calibration writer additionally retained the superseded interpretation that the two 12-byte groups were two temporal samples. New contract tests require one length query at open, one `GET_CUR` per hot-loop read, and exactly one published main sample per 27-byte packet.
- The corrected PC bridge eliminated per-sample `GET_LEN`, but the first real 10 s main-only HIL still failed at 179.993 Hz while video held 59.498 Hz. Replacing the Python video callback with a pure GStreamer fakesink improved one run to 191.146 Hz but exposed a 253.3 ms ioctl stall. After one second of active XU warm-up, the formal window still averaged 187.017 Hz with p99/max intervals 23.833/24.819 ms and 7.79% inferred missing samples. The delay is inside synchronous `GET_CUR`, not between Python loop iterations.
- PC USB topology places the 2UQ2 at 480 Mbit/s behind `Bus 01 Port 6 -> Genesys Hub Port 1`, sharing that USB2 hub with a CP210x device. Bus 03 exposes a separate currently unused 480 Mbit/s root path. The next single-variable HIL is a physical replug to a different motherboard port and topology confirmation before any additional software change.
- A second USB-A port moved 2UQ2 to `Bus 01 Port 10 -> internal hub Port 3` but remained on the same chipset USB2 controller; the actual callback probe improved only to 185.847 Hz. Independent-process polling with the real Python video callback reached 200.166 Hz, proving process isolation is effective. The first per-sample IPC implementation regressed to 189.386 Hz; batching 32 timestamped samples restored formal capture to 200.477 Hz with zero XU errors and gap-free 59.493 Hz video.
- The batched formal run still failed the frozen inferred-drop gate narrowly: 24 inferred misses / 2029 expected (1.1828%). Exact intervals show 1993/2004 normal intervals and twelve paired 13.3–14.8 ms USB scheduling holes; no interval exceeded 15 ms. Direct generic libusb was permission-enabled and could open the device, but the UVC `GET_CUR` returned `LIBUSB_ERROR_IO` while `uvcvideo` owned the VideoControl interface. A true RSUSB-like backend would require detaching the kernel driver and replacing the entire UVC video path, not just the IMU query.
- Official MSI specifications for the MPG Z890 EDGE TI WIFI confirm two rear Thunderbolt 4 Type-C ports. Local PCI/sysfs topology maps the unused Bus 03/04 pair to Intel controller `0000:00:0d.0`; those ports are the next candidate for a physically independent USB controller, unlike the tested USB-A ports on Bus 01/02 controller `0000:80:14.0`.
- The operator has no USB-C-to-USB-C data cable, so the independent Thunderbolt-controller test is presently unavailable. A fresh 90.000 s no-preview capture on the existing USB-C-to-USB-A path preserved 5352/59.466 Hz gap-free video frames and 17955/199.499 Hz main-IMU samples with zero XU failures or timestamp regressions. It still failed only the frozen IMU-drop gate: 248 inferred misses / 18203 expected (1.3624% > 1%). This is adequate to proceed with stereo-only calibration and a provisional camera-IMU solve, but not to accept the final camera-IMU timing/extrinsic result without an independent-controller rerun or a revised evidence-backed gate.
- The first PC stereo operator command was blocked before acquisition by `preview_process_ready_timeout`; the output directory was never created and no capture/preview child or owned shared-memory segment remained. The X display remained reachable, `/dev/shm` had 16 GiB available, direct OpenCV 4.10 Qt5 + AprilGrid initialization took 0.147 s, and the exact spawned `StereoAprilGridPreview.start()` handshake then succeeded in 0.226 s. The failure is currently transient and non-reproducible; evidence does not justify increasing the timeout or changing code without a full-stack preflight result.
- The subsequent 10.000 s full-stack preview preflight started cleanly and passed transport: 595 video frames / 59.497 Hz, 2007 main-IMU samples / 200.691 Hz, zero inferred drops, regressions, XU failures, bad JPEGs, or preview-processing errors; 53/54 submitted preview frames were processed. Overall status remained FAIL solely because the current scene contained zero expected AprilGrid tags in both halves. The next operator gate is a short target-visible preflight, not the 90 s dataset.
- The 15.001 s target-visible preflight passed every AprilGrid gate (73/79 simultaneous valid = 92.41%, 32.35/32.76 average tags) and video transport (59.464 Hz, zero inferred drops), but failed main IMU at 174.659 Hz with 452/3072 = 14.71% inferred misses. Interval analysis localized 226 intervals above 7.5 ms to the first approximately 9 s; after second 10 the same still-active 5 Hz target preview delivered 201.788 Hz with 0.39% inferred drops. The prior 90 s no-preview run similarly met the 1% drop gate only after excluding its first 8 s. Current `IsolatedImuReader` actively warms for only 0.75 s, so the next one-variable test is a 10 s active warm-up without changing preview rate, target, storage, or transport.
- The 10 s warm-up hypothesis was rejected. Its first temporary run was blocked by the reader's independent 5 s ready timeout and left only an empty manifest; after extending that test-only wait, the 15 s measured window still failed at 182.230 Hz / 10.36% inferred drops. No expected tag was visible during that second test, so AprilGrid detection load is not necessary for the degradation. A following independent no-preview/no-storage rate probe reached only 192.602 Hz with 1927 samples in 10.005 s, 24.914 ms maximum XU call time, and 15.200 ms maximum sample interval. The probe's legacy average-rate-only check said PASS, but approximately 3.7% count deficit violates the formal ≤1% drop contract. This reproduces the earlier device/XU state degradation after repeated stream cycles and rules out a preview-only code fix.
- The workflow architecture must separate concerns: stereo calibration may run target preview and gate only image/AprilGrid evidence because IMU is not required for `K,D,T_C1_C0`; camera-IMU calibration must first prove target visibility, then record on the lean no-preview path and retain the strict 200 Hz/drop gate. On the current shared USB-A topology, even the lean path can degrade and therefore still requires physical replug plus a fresh bounded gate; a dedicated Type-C controller remains the final hardware remedy when a cable is available.
- The user explicitly authorized a provisional calibration path before manufacturer FIFO support. Freeze it separately from final acceptance: stereo requires strict video plus AprilGrid observability and does not consume IMU; provisional camera-IMU allows 170–210 Hz, ≤15% inferred missing samples, ≤30 ms maximum acquisition gap, zero XU failures/timestamp regressions, and strict video integrity. Raw/strict checks remain visible and unchanged, and the provisional result is labeled `PROVISIONAL_PASS` so it cannot be mistaken for release acceptance.
- Stage acceptance is now implemented without rewriting strict checks. The recorder also reports the true maximum IMU acquisition interval, distinct from period-multiple jitter. Offline replay of the real target-visible `run1` produces the required three-way evidence: recorded strict transport remains `FAIL`, stereo becomes `PASS`, and camera-IMU becomes `PROVISIONAL_PASS` with a measured 15.297 ms maximum gap. The CLI exposes `--stage {strict,stereo,cam-imu-provisional}` and prints an explicit advance/stop instruction.
- Formal PC stereo run1 is immutable and internally consistent: all six payload/index SHA-256 values match its manifest; 5351 video frames ran at 59.455 Hz with zero inferred drops, and 408/448 preview samples (91.07%) contained a valid grid in both halves with about 32 tags per side. The pinned Kalibr container has ROS1 `rosbag`, `sensor_msgs`, `cv_bridge`, OpenCV, and NumPy, but no `kalibr_bagcreater` executable on PATH; inspect the package before deciding whether to invoke its internal script or write a directly verified ROS1 bag.
- The pinned package does include `kalibr_bagcreater` behind `rosrun kalibr`. Its exact input contract is `camN/<timestamp_ns>.png` plus optional `imu*.csv`; it writes grayscale `sensor_msgs/Image` topics `/camN/image_raw` at the filename timestamps. Deterministic 5 Hz selection of stereo run1 yields 444 synchronized pairs over 89.916 s and an estimated 1.84 GB uncompressed mono bag; 455 GB is free. The owned exporter will validate source CRC/JPEG/shape, split left half to cam0 and right half to cam1 at the identical host-monotonic acquisition timestamp, and preserve a derived index before invoking the pinned creator.
- `ego_pc_cam_imu_calib_20260828_run1` was intentionally stopped by the operator because `--no-preview` produced neither an image window nor terminal progress; it was not a device crash. Append-only cleanup preserved all three streams and their manifest. Offline reconstruction measures 4214 video frames over 70.828188 s at 59.482 Hz and 13888 main-IMU samples over 70.832666 s at 196.054 Hz, with zero timestamp regressions and a 19.024 ms maximum IMU gap. Despite provisionally acceptable transport, this dataset is diagnostic-only because it covers only 70.83 of 120 s and was operator-aborted.
- The causal software gap was operator observability plus incomplete interrupt handling: the capture loop printed no progress, and `KeyboardInterrupt` bypassed normal report creation because it derives from `BaseException`, not `Exception`. The owned capture path now prints setup milestones, explicit preview state, recording start, 5 s elapsed/remaining counts, finalization, and report location. Ctrl-C inside acquisition requests the IMU child to stop without prematurely closing its drain queue and finalizes an operator-aborted FAIL report.
- The operator-created stereo export is internally consistent: `cam0`, `cam1`, and the derived index each contain exactly 444 entries; left/right timestamp filenames are identical; and the independently computed index SHA-256 matches `d9b039ef6c71f6327320149f4961d2d3b0c68d51553167931507a4c1e7fc4e04` in the export report.
- The pinned Kalibr image exposes `kalibr_bagcreater` only through the catkin environment and accepts exactly `--folder` plus `--output-bag`. The stereo export already matches its required `cam0/<timestamp_ns>.png`, `cam1/<timestamp_ns>.png` layout.
- Stereo optimization converged and wrote a usable camchain/results file before report plotting failed on the container's missing Python `cairo` module. Final intrinsics are cam0 `[664.89256,665.33588,945.13747,589.63846]`, cam1 `[664.33011,665.00405,954.06458,518.16668]`; baseline magnitude is 74.8863 mm. Reprojection component standard deviations are cam0 `[0.3836,0.3464]` px and cam1 `[0.2572,0.2188]` px.
- The first two-camera IMU solve is rejected: optimized reprojection means rose to 8.11/7.74 px and it assigned -6.455 ms to cam0 but +5.333 ms to synchronized cam1. This violates the identical camera timestamp contract. The corrected model estimates one `T_C0_I,td` from cam0 only and composes cam1 through fixed `T_C1_C0`.
- The corrected cam0-only IMU solve converged to `T_C0_I` translation `[0.032441,-0.004906,-0.028353]` m, lever arm 43.36 mm, and `timeshift_cam_imu=-0.0023869114` s under `t_imu=t_cam+shift`. Residual means are 0.5953 px, 0.00556 rad/s, and 0.04753 m/s². These are provisional because transport is strict FAIL and random-walk values are priors.
- Full-chain acceptance remains FAIL. Independent rectification over 97 sampled target-visible frames measured a 13.014 px vertical p95 (median 0.482 px) and exposed a heavy late-sequence tail. Kalibr itself kept only 109/431 stereo views and removed 6,341 corners. This indicates dataset inconsistency/outlier contamination; do not use the frozen stereo parameters as release calibration despite the low in-solver residual on retained views.
# AprilTag open-source review for Ego + dual UMI (2026-08-28)

- Reviewed `AprilRobotics/apriltag` master `b7c0ebe9aa20f82ec7a828579004f9e706bfecd9`. It is the appropriate detector dependency: AprilTag 3, Linux support, small-tag improvements, pose estimation, recommended `tagStandard41h12`, and BSD-2-Clause licensing.
- Official `tagsize` is the distance between detection corners at the black/white border, not the printed paper's outer edge. A wrong value causes proportional translation/depth error.
- Official optical convention is camera x right, y down, z forward; the tag is centered on its plane. The repository adapter must normalize pose direction explicitly and test it rather than rely on ambiguous prose.
- Reviewed `ju1ce/April-Tag-VR-FullBody-Tracker` master `c91270b23211599173a764694655aa6ec70d5a75`. Reusable ideas are corner-order normalization, multi-tag rigid boards, repeated mount calibration, predicted ROIs, acquisition-time latency handling, and preview overlays.
- Its active path binds to `videoStreams[0]` and `cameras[0]`; its `UpdateMulticam` refines a manual OpenVR playspace transform against a driver pose. It is not a synchronized three-camera world-alignment implementation and should not be imported as the product framework.
- Its recommended smoothing introduces material latency and its README warns current master builds are partly broken. Reuse concepts only, not the OpenVR/manual-scale application layer.
- Frozen product equation: `T_W_Gi = T_W_CE * T_CE_Ai * T_Ai_Gi`. If UMI local odometry provides `T_Oi_Gi`, solve `T_W_Oi = T_W_Gi * inverse(T_Oi_Gi)` over a quality-gated synchronized observation window, then transform the full local trajectory with `T_W_Gi(t) = T_W_Oi * T_Oi_Gi(t)`.
- Ego SLAM remains the only world definition. AprilTags initialize/relocalize/correct left/right UMI anchors; tag IDs only select the corresponding mount calibration.
- The current host has OpenCV 4.10 with `DICT_APRILTAG_36h11` and `SOLVEPNP_IPPE_SQUARE`, but no installed official `libapriltag`. The OpenCV implementation is therefore explicitly a reference/replay backend; production AprilRobotics integration remains pending.
- The most recent D435i temporary-Ego HIL session stores paired 1280x720 Y8 infrared frames at 30 Hz with factory `fx=fy=644.0672`, `cx=642.2068`, `cy=367.0062`, zero reported coefficients, acquisition timestamps, CRCs, and a PASS stream report. This is sufficient to freeze the raw replay contract without inventing intrinsics.
- The official tag PNG contains a 50 mm 10x10 quiet-zone grid while PnP must use the inner 40 mm 8x8 black square. Supplying 50 mm to PnP would scale translation/depth by 1.25; regression coverage now fixes this contract.
- OpenCV's square IPPE returns two planar candidates. The replay gate discards non-positive-depth candidates and rejects a near-tied second solution only when its SE(3) pose is materially distinct; accepted observations retain acquisition and detector-completion time separately.
- 2026-09-05: Current mount evidence is approximately46–47mm depth, not the historical61.3mm. 18:58 changed-view capture passes the z-only two-chain gate but has8.074mm full mount translation disagreement. Relative17:46, UMI external-tag median positions change0/1.506mm; relative17:37 they change15.0/34.5mm, so only the latest reference supports an approximately fixed external layout. Cross-view repeatability is not absolute physical accuracy; shell53.3mm still needs independently measured inward offsets.
- 2026-09-05 19:50: Controlled changed-view repetition now passes against18:58 (about12.7deg new viewpoint,2.03mm/0.62deg mount change); depth46.46mm. Two-chain full3D mount disagreement remains6.06mm despitez-only gate pass. All599 frames per camera have accepted relevant chains; geometric size/angle check sampled32frames and is close to limits (60.2px/44.93deg). Do not infer absolute5mm accuracy or training readiness.
- Offline joint-corner diagnostic: raw chain delta primarily persistent+x (8.609/7.886/4.473mm), whilez changes sign. Joint fitting fixed intrinsics/scale shifts mount depth to49.593mm and improves independent heldout pixel prediction11.204→5.011px. This is a model-dependent candidate, not absolute metrology. All three leaveout folds improve but retain5–7px residual, so neither simple averaging nor optimizer convergence closes calibration. Full v2 evidence and trained/external-only nuisance poses saved; no formal calibration replacement.
- Operator confirmed rightid2 black40x40mm and flat. Fixed external layout across training sessions worsens reused-validation mountRMSE5.011→7.168px; preserve negative evidence and do not adopt. Fixed-X-only5.395px also fails to improve. Existing data cannot uniquely identify remaining systematics; need new independent geometry for final metrology rather than repeated validation-set tuning.
- User later identifiedLEFTid1 wrinkles (rightflatness nevercoveredleft). Replacement printedandmounted,20:55capture fullchain6.867mm andfrozenjointprediction10.620px do not improve. Replacement also altersleftPnPposition69.44mm andEgoedge69.4→60.9px; cannotinfer wrinkle causality from this confoundedcomparison. Keepnewflat target, improveobservationgeometry for furtherindependenttests, no M refit or activation.
- First mixed80mmexternal/40mmmount realcapture21:59: all598associated framepairs passgeometry with min113px vs previous~60px. Rotation0.608deg, mountdifference[0.748,-0.226,-6.494]mm: remaining discrepancy predominantlydepth; repeatedfourchunks6.41–6.68mm. This isolates persistent within-session disagreement, not its cause; physicalprinted dimensions stillunmeasured. Mean51.204mmdepth notactivated and not an absolute match to53.3mmshell datum. Results in mixed80_crosscheck_20260905T215930.
- User subsequently confirmed80mm. Fixed598image ablation identifies frontend sensitivity: OpenCV APRILTAGrefinement z4.523mm/full3D4.846mm vs baseline6.494/6.540; subpix5x5/iterative controls largely retaingap. External-onlyrelativepose alreadydisagrees, so not solelybackmount/shelloffset. ConfirmedSDKflatrotation misinterpretedrowmajor; smallseparatedefect cannotexplainnormgap. Full report/verification in fixed_image_ablation_all_v1. Do notactivate49.683mmmean or claimexclusivecause/absolutemetrology; heldoutview andformal regression-backed changespending.
- Operatorapproved frontend nowintegrated: mixed80/40 pathdefaultsapriltag with nonelegacy switch; shareddefaultnone preservesolduniformpaths. Integratednewentry fullreplayreproducesdiagnosticexactly, oldnone fullauditparity proven. Changesare frontendonly, SDKlayoutknownissue pending; noinstallationMactivation orfullthree-devicecoordinatorchange.83relatedtestspass, freshviewvalidation stillneeded.
- Independent22:51 new-view preserves frozen frontend/sizes but not demonstrated fixed external layout: refined chain z2.237mm/rotation1.336deg pass, full3D5.631mm exceeds5mm. Mean mount repeatability4.441mm/0.321deg passes declared bounds but is not frozen-model prediction or absolute accuracy. Same new frames legacy none z1.340mm/full3D5.505mm is slightly better; old-image frontend benefit is not universal. Four quarters show persistent5.61–5.64mm full3D disagreement. Data evidence and all17 hashes verified under mixed80_newview_20260905T225115; no M activation, next confirmed-defect work is provenance-aware SDK layout correction rather than further user micro-adjustments.
- SDKarraylayoutbug nowfixed andverified201tests/50hashes; affectsabsoluteM0.21–0.244mm/0.536deg but cannotchangefullchainnorm. OfficialAprilTag sameimagecomparison5.006/5.670mm doesnotresolvepersistentbias. Cross-tagpixelbias1.52–2.56px exceedspropagatedtemporalRMS0.15–0.17px; upstreamimaging/modeldisagreement remains, notmerelyshellmeasurement. Densepatternregistration helpsoneviewandworsensother, noadoption. External-onlyjointfit improvesMrepeatability2.25mm butfrozenoldMnewview3.65pxRMSE remains. Sparse per-camera intrinsicfits are unstable (D40516params/16residuals no redundancy, not count-based proof of rank deficiency); cannotreliablyseparatecameraandprinted-targetgeometrybias. Nextindependentmeasurement needsflatAprilGridmultiviewcolorimages; old2UQ2camchains arenottheseRealSensecameras. No size/Kfitactivation, noformalM.
# 2026-09-06: detector parity and placement preflight

- UMI original calibration uses aprilgrid0.5.0 Detector(t36h11), not the OpenCV
  ArUco path previously used here. Same156 accepted raw board images (hash-checked)
  show Ego mean27.026/8.4adjacent-IDtoggles vs native36/0, UMI20.949/3.738 vs23.423/.108.
  Adapter replay: zero errors and duplicate frames. New adapter keeps raw ID conflicts
  (upstream normally removes duplicate IDs by largest area), native1280x720, no
  temporal hiding. Original defaultdownscale had1cv2error; threshold2000avoidsdownscale.
- Evidence artifacts/spatial_bench/board_detector_ab_20260906_run1/{result,adapter_result}/report.json.
  compare_initial.py preserves exact original diagnostic code SHA10724af4...;
  compare.py adds adapter mode without modifying either stored report.
- Same-support geometry contrast halted at slot8left: original selected17/29 not
  found by new detector. This is not permission to delete them or claim a new PASS.
  Detection stability is demonstrated, metric root cause5.631mm remains unresolved.
- Simultaneous board+mount observation is not mathematically necessary if both
  cameras/mount remain fixed between observations. Simultaneous FOV feasibility
  (without rotating camera between stages) IS necessary for this static bridge.
  Added early mount->board->mount placement preflight, preserves6PNGs/quality/hashes
  on3confirmations, no16stepcalibration or solve eligibility. Pixel closure0.75px
  remains a proxy, not proof against common motion. Live layout requires operator.
# 2026-09-06: continuous A/B current evidence

- run2 hash-verified120PNGs/40triples; reproduced frozen report exactly. B static
  subset15/20 medianp95=2.913818px vs all20=2.916412; transient board drift not
  sufficient cause. A/B epipolarp95 medians .344867/2.456160px: IR depth-only
  correction under unchanged rays/T cannot remove B discrepancy.
- First10 each phase diagnostic-only fits differ5.10644mm/.362554deg; own-view
  errors~1.26/1.34px, cross-view~2.94/2.74px. SharedT compromise~1.70/1.71px.
  No B refit accepted/activated. Separate per-view board fits~.15pxIR/.39pxUMI
  do not prove factory intrinsics accurate (pose can absorb model error).
- UMI bottle occluded in B, so old bottle ROI is NOT reusable. Current gripper
  ROIs show small texture shifts; mount raw NCC shifts~.53px vertical whereas
  maxcornerchange~1.34px. Need background control before physical interpretation.
- analysis_v1/report.json retainsallframes/failures, idealraycontrol andROI
  reliability. Ideal per-view rays stillB~2.9px; not only random corner noise.

# 2026-09-06: all36 static regional diagnostic

## Actual upstream reference executed (later update)

- Real multical efe9d7d0af85d51d32669e4b77469494972ac25a, unchanged optimizer API;
  source inspected and tracked hash clean, deps isolated under repo artifacts.
  Original run4 all288PNG replay, frozen8train/5holdout,1288 observations kept.
- Own worst4.391011907px; actual multical4.390738283; actual upstream OpenCV
  stereoCalibrate independent init4.390746727; then multical4.390488482.
  Factory rays -> virtual pinhole, same linear objective and raw SDK scoring.
- This is optimizer-level comparison, not upstream detector/CLI. It does not
  exclude shared corner/model/layout errors or prove factory intrinsics wrong.
  SixDOF upstream gauge normalized by relative pose; tiny rotation differences
  require scipy Rotation, old cv2 difference zeros them. Detailed RESULT under
  artifacts/spatial_bench/upstream_multical_20260906_run1; no activation.

- All12 existing paired frames retain36boardtags; geometrically unique restoredboardID1
  error max.355px, both restoredstaticwindows pass. Original35capturepolicyunchanged.
- 68frozengroups/frame=816fits plus432individualchains. WholeMnorm56.007–56.876mm,
  maxpositionrepeat1.381mm. Checkerhalves.697–2.202mm vs horizontal11.644–15.623mm
  andvertical2.760–4.203mm; restoreID1wholeeffectmedian.508mm. HorizontalmeanXYZdelta
  [-3.320,13.135,1.197]mm, mainlyy, not13.6mmdistance-lengtherror. Egoheldoutp95
  medians1.987/1.419px vsUMI.421/.842px. Regionalpose/model/cornerbiasandnarrower
  supportconfounded, cannotuniquelyblamefactoryKD. Notindependentaccuracyverification.
- Evidence common_board_joint_20260906_run1/all36_resampling/RESULT.md;108tests,
  readonlyreviewexactreproduction; noactivation/hardware/externalwrites/commit.
# 2026-09-14: Tag-ID constraints prove run8 fixed transform is not stable

- Formalized the legacy 6x6 board association as decoded tag ID -> frozen metric
  corners. Full30Hz run8 yields325 synchronized cross-world constraints with no
  duplicate-ID frames and acceptable PnP P95 (Ego1.700px, device31.597px).
- Phase-local train/holdout prevents good phases hiding a bad one. Overall translation
  P95 is32.291mm, while translation-phase is39.208mm P95. Phase anchors span
  43.683mm/2.530deg. A separate10Hz selection yields96constraints and
  46.897mm/2.766deg; central candidates differ only1.354mm/.087deg.
- Both static endpoints have zero joint board observations; full-frame known-ID
  epipolar P95 is14.815px. Reject the fixed transform. Preserve Tag-ID constraints for a pose-graph
  factor or time-varying correction; no activation.

# 2026-09-14: run9 closes endpoint coverage but still rejects one fixed world transform

- Fresh105s run9 passes capture and both independent VINS replays. Two-times image
  analysis restores missed small Ego-grid detections to source-pixel coordinates and
  yields1664 constraints across every planned phase, including both static endpoints.
- Endpoint coverage was not the cause of run8's instability. Run9 phase anchors span
  38.194mm/3.714deg; stride-3 repeats36.991mm/3.656deg. Whole-run known-ID epipolar
  P95 is9.091/9.983px. A fixed `T_egoWorld_rightWorld` remains rejected.
- Coarse phase anchors are sampling-sensitive: full yaw heldout translation P95 is
  29.301mm, while stride-3 is34.157mm. Phase-local epipolar P95 stays3.558-3.900px.
- Deterministic0.5/1/2/5s pose-only window replay supports the existing device-2
  correction architecture:1-2s heldout results are12.394-13.811mm and1.003-1.117deg
  across full/stride-3. This is diagnostic support, not mounted-tag runtime or HIL PASS.
- The mounted device3 ID1 is visible in run9, but board+mount numeric overlap is only
  seven frames and the mount edge max59.419px; zero frames satisfy the existing60px
  gate. The prior mount artifact is still measurement pending. Do not derive or activate
  `tag_from_device3` from run9. Next capture a preview-gated10-20s static same-frame
  board+mount window, then replay `estimate_tag_anchor_window`/`reconcile_anchor`.
- Do not recalibrate Ego factory intrinsics or fit factory K/D from these results.

# 2026-09-14: run11 fixes mounted-tag visibility but board-chain prediction still fails

- Operator-requested15s recapture passes raw acquisition/seal/offline checks and static
  motion. Mounted ID1 is unique in all formal Ego frames, ~68.97px, with.229px P95 own
  reprojection. This resolves run9's <60px/zero-overlap limitation.
- Exact common-ID support leaves54/450 accepted frames over14.6s. Per-camera board PnP
  is low (.752/1.625px P95) and first/second-half mount candidates differ1.486mm/.081deg,
  but the training candidate predicts heldout mount corners at10.345px P95 vs1px.
- The stable half means cannot override pixel failure. Montage examples show Ego only
  9/12/12 board detections and8-11 common IDs while right has27-31. Move only the board
  closer/more frontal to Ego and require stable live counts before recapture; keep rig/tag.

# 2026-09-14: next recapture now has a true two-camera adjustment preview

- The prior shared RSUSB UI reused the D435i worker preview, so it showed Ego left/right
  eyes and no D405 image. The capture loop did acquire both devices, but the operator
  could not verify their common framing from that window.
- The entry now keeps the latest D405 left-IR payload for display and renders one tile per
  physical camera: Ego left IR plus device2/device3 left IR. The same window remains live
  during initialization, the10s pre-record countdown, and formal recording. Preview work
  is bounded to the configured display rate and does not change the append-only writers.
- The default start delay is10s. A synthetic image test proves the left/right tiles carry
  distinct source payloads; an offline rendered frame confirms the Chinese titles and
  countdown fit in1280x360. The first8s explicitly allow adjustment and the last2s require
  all movement to stop. The17 focused and19 adjacent integration tests pass. No camera was
  opened for this software-only change.

# 2026-09-14: run12/run13 recaptures isolate motion gate and physical framing failures

- run12 raw transport passed, but device3 gyro stayed above3deg/s through many intervals
  from.77-4.77s after formal start (P957.821, max18.676deg/s); Ego peak was1.039deg/s.
  It therefore violates a rigid static mount observation. Its later frames still had only
  3-7 common board IDs. Compact evidence/montage were preserved and raw removed.
- `bench_no_motion` had incorrectly returned motion PASS unconditionally. The shared owner
  now requires every formal-window peak <=3deg/s and includes this result in overall status;
  shared_world_motion retains its >=3deg/s motion-observability requirement.
- run13 adds5s settling after the requested10s adjustment. It passes transport, timing and
  static motion (Ego/right1.063/.720deg/s peaks), proving the timing/settling path works.
  However, none of151 stride3 pairs reaches12 common IDs: Ego median4/max9 vsright
  median30/max35, common median3/max8. Images show a sharp but physically too-small and
  upper-left Ego board, not blur or a D405-side failure. Move the board closer/centered and
  more face-on to Ego while retaining full D405 view; no solver should run before this gate.
## 2026-09-14 run15 accepted static mount candidate

- Session `three_device_mount_static_20260914T192047_device3_right_run15` passes native
  acquisition, indexing/sealing, and `verify_pair_session`. Static peaks are1.005deg/s
  Ego and.577deg/s right; paired timing p50/p95/max is12.175/14.700/16.142ms.
- The fixed2x board detector failure was scale-specific. The same source frames at4x
  yield sufficient decoded common IDs without sharpening or ID substitution.
- Applying the existing legacy-board fixed5px `cornerSubPix` after restoring4x detections
  to source coordinates reduces board PnP p95 from.826/2.358px to.085/.226px for
  Ego/right. Decoded IDs, selected frames, factory K/D, and gates are unchanged.
- Final common-ID mount validation accepts412 frames and passes heldout prediction at
  .686px p95. The candidate transform is evidence only and is not activated because this
  capture contains one physical pose; a novel-pose heldout remains necessary.
- The earlier pixel-optimized result without fixed5 refinement remained4.290px p95,
  excluding simple SE(3) averaging bias as the explanation.
## 2026-09-14 run16 novel-pose failure

- Native acquisition and pair verification pass, and the board pose is genuinely novel.
- Frozen run15 transform predicts the external mount corners at5.071px p95, failing1px.
- The detector did not confuse board ID1 with the external mount ID1. Direct image/pose
  comparison shows the true external tag moved40.8px left and85.8px up in Ego, with
  scale83.87->96.48px and Ego-from-mount change54.1mm/1.85deg.
- Camera factory geometry and stream profiles are unchanged. The evidence therefore
  requires a fixed-rig recapture; no transform was activated.
## 2026-09-14 run17 controlled baseline A

- Native/session/pair gates pass with411 usable common-ID frames and9.883ms timing p95.
- run16->run17 external-tag center stability is.16px; the rig is now repeatable.
- Independent per-frame planar PnP has1.785px p95 jitter. A single static-window transform
  estimated from train board frames predicts heldout mount corners at.313px p95, with no
  heldout mount pixels fitted. This is the appropriate A-window candidate metric; the
  per-frame number remains reported.
- Candidate is provisional and NOT_ACTIVATED pending a board-only novel-pose B holdout.
## 2026-09-14 controlled run17 A / run18 B decision

- B acquisition and board novelty pass; the external mount remains fixed to42um/.048deg.
- Frozen A predicts B mount corners at2.994px p95 versus1px. This is a clean independent
  failure with410 frames and low board PnP p95 (.153/.200px Ego/right).
- Factory geometry is identical. A B-specific diagnostic fit differs1.109mm/.167deg from
  A, quantifying pose-dependent planar-chain bias; fitting B cannot count as validation.
- Stop recapture. Preserve A/B raw and diagnose board flatness/printed geometry and a
  multi-pose model before any further HIL or activation.
## 2026-09-14 shared-rigid multi-pose correction

- Compact A/B observations show sufficient ID support without result-based deletion.
- Joint shared-X optimization with fixed factory rays and fixed nominal board geometry
  passes checker-ID heldouts at.228/.256px p95; parity-fold X differs.316mm/.039deg.
- Full-ID model p95 is.232px. The prior2.994px B failure is therefore caused by composing
  separately ill-conditioned planar PnP poses, not evidence that the visibly flat board
  must be bent.
- Frozen model package and C-only validator are ready. Validator selftest passes geometry
  (.284px board/.271px mount) and correctly fails novelty when B is reused.

## 2026-09-14 run19 independent C detects changed rigid relation

- Capture transport is clean: 450 paired frames, timing p50/p95/max
  1.181/2.490/2.540ms, static gyro peaks 1.044/.418deg/s, and unchanged factory geometry.
- The board is fully visible. Right-camera fixed4x detection is a sampling blind spot;
  fixed2x produces 150 accepted windows and 31 stable common IDs. A mixed Ego4x/right2x
  replay gives the same failed geometry, so scale selection is not the root cause.
- Frozen A/B prediction fails by 74.679px board p95 and 36.573px mount p95. C-derived
  inter-camera transforms from the common board and the independently observed mounted
  tag agree within 2.609mm/.326deg, but differ from A/B by 11.052mm/6.469deg and
  12.429mm/6.610deg respectively. Two independent measurements therefore identify a
  changed Ego/device3 physical relation rather than timestamp, intrinsics, corner scale,
  board visibility, or C motion.
- The executable capture contract explicitly says `devices+mount rigid whole session`.
  Reuse of A/B is invalid once that relation changes. Lock the current assembly and
  recollect A′/B′/C′ while moving only the AprilGrid; keep activation disabled.
- Diagnosis: `run19_rig_change_diagnosis.json`. Raw-cleanup manifest:
  `run17_run19_superseded_raw_cleanup_manifest.json`; 15.444GiB reclaimed while compact
  evidence and all session metadata remain.

## 2026-09-14 run20 A′ controlled baseline

- Native capture, single-UMI coordinator, 450-row sync index/seal and pair verifier PASS;
  timing p95 3.932ms and static gyro peaks 1.082/.470deg/s.
- Detection scales Ego3x/right2x were frozen from decoded coverage counts. All 450 frames
  pass with common-tag median 26 and board PnP p95 .119/.202px.
- Static-window heldout mount prediction is .289px p95 and temporal halves differ
  .037mm/.011deg, so A′ is a valid baseline. Per-frame planar PnP is retained only as the
  known depth-jitter diagnostic at 1.261px p95. No transform is activated.

## 2026-09-14 run21 B′ and frozen A′/B′ model

- B′ acquisition/index/seal/pair gates pass. It is novel from A′ by about
  78.7mm/15.5deg in both camera frames, while the independently observed mount Tag moves
  only .041px and .028mm/.067deg.
- A′/B′ shared-rigid checker parity holdouts pass at .218/.283px p95 and their shared-X
  transforms differ .143mm/.022deg. The full-ID fit p95 is .256px.
- The frozen candidate needs one whole-pose C′ whose mount pixels are excluded from all
  fitting. Runtime/world activation remains disabled.

## 2026-09-14 run22 C′ PASS and device3 world-mount candidate

- C′ transport/static gates pass. Its fixed Ego4x/right2x detections provide 150 common
  windows, 33 stable IDs and 150 independent mount observations.
- Frozen A′/B′ whole-camera prediction passes at .533px p95 and the mount prediction at
  .231px p95. C′ novelty is 165--243mm and 21--37deg; all independent gates pass.
- Packaged both `T_right_ir_left_from_mount_tag` and its inverse, plus VINS body-frame
  compositions. Camera-from-tag translation is [13.118,-7.911,-48.543]mm with 50.902mm
  norm and valid SO(3). Device3 is correctly bound to temporary/final right ID1.
- Static calibration is complete but runtime remains disabled pending physical 40mm tag
  measurement and dynamic world-anchor HIL. Device2's historical calibration names ID1;
  final left ID0 needs separate physical verification when device2 is available.
- Removed 15.373GiB A′/B′/C′ raw `.bin` payloads only after hash-bound compact evidence
  was packaged. Reports, extracted observations, session metadata and frame reviews remain.

## 2026-09-14 device3 Tag print-scale closure

- The installed device3 right Tag is operator-confirmed as the same project-generated
  tag36h11 ID1 asset with a 40.0 mm effective black square and 50.0 mm quiet-zone square.
- Printer job 17 is hash-bound to the vector PDF and used A4 with `print-scaling=none`.
  The 300 dpi verification raster measures 39.878 mm because of pixel quantization; it
  does not replace the vector/print-contract dimension of 40.0 mm.
- The metric-scale gate is accepted from the exact print pipeline plus installed-asset
  identity confirmation. No independent caliper measurement is claimed. The candidate
  remains NOT_ACTIVATED pending the 105 s single-pair dynamic world-anchor HIL.
- Removed only ten large raw image payloads from superseded world runs 8 and 9 after
  writing a cleanup manifest; 39,206,707,200 bytes were released. Timing JSONL, sync,
  calibration, IMU, acceptance reports and seals remain.
- The subsequent run23 attempt is not evidence against the calibration: the rig was not
  moved during the required excitation windows. It was operator-aborted at about 67 s,
  marked unusable for acceptance, and its incomplete large image payloads were removed.
- The full 105 s protocol does not need to be repeated. Run9 already passes acquisition,
  synchronization and both dynamic VIO replays with about 0.43 m and 70 deg excursion.
  Its one fixed world transform failed and must not be used, while A′/B′/C′ later supplied
  the missing mounted-tag calibration. Reuse run9 for motion/VIO evidence and close only
  the current mount/runtime integration with a bounded short HIL.

## 2026-09-14 run24 anchor HIL diagnosis and acceptance

- The apparent 14 deg ID1 orientation excursion around 30--33 s is not VIO or rigid-mount
  failure. The independent right-camera AprilGrid path stays below .855 deg p95 during
  the rotation phase, while the ID1-vs-board path alone jumps. This is a single planar
  tag PnP branch/transient under motion and must not drive one-frame anchor updates.
- Low-rate consensus windows isolate the product behavior: initial 79/79 inliers, final
  17/17 inliers, 2.400 s occlusion bridged, and final tag path 7.910 mm/1.045 deg from
  initial. Direct board/VIO endpoint is 8.247 mm/.813 deg; static mount-vs-board p95 is
  3.229 mm/1.115 deg. All frozen HIL checks pass.
- A naive end-to-end chain using the provisional static D435i VINS reports
  14.551 mm/3.081 deg because Ego itself drifts 3.262 deg while physically fixed. The
  AprilGrid-defined validation world removes that known Ego-only drift from the device3
  verdict; no Ego recalibration or threshold relaxation was performed.
- Accept the device3 right ID1 mount. Runtime anchor updates require device3 gyro at or
  below 3 deg/s and 30 mm/3 deg multi-frame consensus; retain the prior anchor through
  fast rotation and tag loss. Device2 left ID0 remains the only group-formation blocker.
- Run24 raw image payloads and derived replay DB3 files were removed only after accepted
  calibration/package hashes passed. Compact constraints, plots, timestamps, IMU and VIO
  outputs remain; the cleanup manifest records the seven verified hashes and 16.338 GB.
