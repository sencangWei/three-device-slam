# Factory camera model audit — 2026-09-06

Factory K/D remain authoritative. No intrinsic/distortion fitting, camera flash write,
installation-transform activation or external repository modification is performed.
AprilGrid preview was stopped with zero recorded samples; both cameras released.

## Confirmed defect and correction

`pair_tag_alignment` preserved numeric coefficients but dropped `distortion_model`.
RealSense inverse Brown was therefore implicitly interpreted as OpenCV Brown.
The model is now retained in immutable `CameraCalibration`; unknown models and
inconsistent `none`/nonzero-coefficient data are rejected. Historical model-less
fixtures keep an explicit `opencv_brown_conrady` default.

For `distortion.inverse_brown_conrady`, pure geometry calls in the already-installed
pyrealsense2 SDK supply normalized rays for IPPE, and pixel projection for candidate
reprojection error and mixed-tag association. No camera is opened by these calls.
The SDK is required for this model; no silent numerical fallback is allowed.
Existing OpenCV-model callers retain their original solver path. Both live placement
previews now pass `str(intr.model)` as well. Calibration values are not altered.

The model-specific formulas are distinct in the [official librealsense source](https://github.com/realsenseai/librealsense/blob/master/src/rs.cpp).
The actual offline comparison used installed SDK2.58.2, not a newly installed build.

## Evidence and limits

`artifacts/spatial_bench/factory_usage_audit_20260906/report.json` contains the
pre-fix audit. `postfix/report.json` repeats it with explicit historical OpenCV
baseline reconstruction. `source_before/` preserves the exact original scripts
referenced by the original report; do not rewrite historical hashes.

- Two sessions, each598 accepted frame pairs; all5roles/all4corners, no resampling.
- All2398 formal color frames' index dimensions/encoding/payload sizes agree with
  factory1280×720 color intrinsics. Full CLI additionally checks raw CRC.
- Factory snapshot hashes remain identical to the prior archived values.
- D435i color has allzeroD. D405 color's largest observed SDK-versus-OpenCV ray
  displacement is about0.0041px;627 full-field testpixels have max0.00794px.
- Individual pose translation changes remain below0.0018mm.
- Fullchain difference changes4.845943→4.845308mm and5.630932→5.630870mm.

The frozen SDK-ray audit deliberately matches each candidate to the original pose
branch to isolate the model change. It is not independent branch acceptance,
fresh hardware verification, or absolute accuracy validation. The model omission
is real but does NOT explain the persistent5–6mm chain discrepancy.

94 focused/related tests pass, including9 factory-model tests. Independent read-only
review found live preview omissions; both fixed, re-review has no Critical/Important.
The full CLI uses separate`_factory_model_v3` output directories and unchanged
candidate/association/geometry thresholds. No K/D/M is activated by it.

Full new-view CLI replay finished with exit0:599attempted/598associated, all geometry
checks and raw CRC pass. A separate verifier confirms the same598identities and
exactly identical corners as the original report, and exact agreement between
normal production candidate selection and the frozen SDK-ray diagnostic.
`verification.json` records25 hashes and the remaining5.630870mm full3D difference.
Existing verdictpass checks z/rotation, NOT an absolute full3D5mm guarantee.

The earlier2-bit Kalibr board preview decoder fix is separate: external alignment
tags still use1-bit borders, external1/2 black80mm and mount1 black40mm. Do not
transfer Kalibr board spacing, border width, or camera color/IR parameters between
these different paths.
