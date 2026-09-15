# Mixed80/40 AprilTag frontend integration

Scope: static EgoD435i + oneUMID405 external-tag crosscheck, not the full three-device SLAM coordinator. ExternalID1/2 black80mm, backmountID1 black40mm. Installation calibration remains NOT_ACTIVATED.

## Selection

- Shared `AprilTagDetectorConfig.corner_refinement` accepts `none` or `apriltag`, default `none` for existing callers.
- Mixed-size `detect_pair`, `compute`, crosscheck CLI and live preview default to `apriltag` after operator-approved integration.
- `--corner-refinement none` explicitly selects legacy integer-corner behavior. No automatic fallback or gate relaxation.
- This is OpenCV's CORNER_REFINE_APRILTAG frontend, not a new standalone AprilRobotics library dependency. Metric square-IPPE, candidate acceptance, duplicate-ID association and all geometry gates are unchanged.
- Reports include detector backend, OpenCV version, corner method and solver. Preview prints the method and stores it inlatest.json.

## Commands

Run from repository root with the existing/optvenv. Substitute a NEW previewdirectory or actual rawsession path for the angle-bracket placeholders.

```bash
cd /home/robot/three-device-slam
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. /opt/three-device-slam/venv/bin/python \
  scripts/live_mixed_placement_preview.py \
  --snapshot-dir <new-preview-directory> --corner-refinement apriltag
```

Close withq/ESC and wait for camera ownershiprelease before a capture. Do not run preview and capture together.

```bash
cd /home/robot/three-device-slam
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. /opt/three-device-slam/venv/bin/python -u \
  -m three_device_slam.spatial.mixed_size_crosscheck \
  --session <raw-session-directory> --stride 1 --corner-refinement apriltag
```

Current output after factory distortion-model propagation is`<session>/spatial/mixed80_40_apriltag_factory_model_v3/`; integer-corner`none` output is`mixed80_40_factory_model_v3/`. Original unsuffixed and`_layout_v2` reports are preserved. Existing directories cause explicit refusal, never overwrite. For read-only repeat computation, use`compute(Path(session),1,corner_refinement='none')` or`'apriltag'` without the writingCLI. Note:`none` selects old corners, not the old discarded-distortion-model bug. New results include model/ray backend, calibration-file hash and layout-read policy; see [SDK layout compatibility](realsense-extrinsics-layout.md) and [factory model audit](factory-camera-model.md).

## Verification 2026-09-05

- 83 related image/spatial/demo/simulation/mount tests passed; focused new/shared subset48passed. New coverage includes accepted-option validation, legacydefault, mixedpropagation,40/80scale, candidatebinding, unknownIDs, emptyimages, reprojectionreject and separate immutableoutputdirectories.
- Independent read-only code review found noCritical/Important issues; focused reviewer tests26passed.
- Integrated refinedCLI replay on`mixed80_crosscheck_20260905T215930`:600attempted/598associated, no geometryfailures, allrawCRC and timestampgatespass; rotation0.291883deg,z4.522605mm,full3D4.845943mm, verdictpass. Exactchains/agreement parity with prior frozen-imageAPRILTAGdiagnostic.
- Integrated`none` read-only600pairreplay exactly reproduces archivedaudit,chains,agreement and598association count. OriginalREVIEW report hash unchanged.
- Preview CLI option/help verified without camera activation this integration turn; live display/newviewcapture are pending, not claimed tested here.
- Preintegration source copies are retained in session`spatial/frontend_integration_source_before/` for historicalhashverification. Current source hashes necessarily differ from earlier diagnostics' sourcehashes; do not rewrite old evidence.

## Limits / next

The replay demonstrates software integration and within-session two-chain consistency, not independent calibration accuracy. New-view real data is still needed before activating an installation transform. The separately identifiedSDKextrinsic column-major/row-major provenance issue remains unchanged in this frontend-only change. No externalD405/ego_vio repository edits, no commits, no publishedcalibration, no automaticSLAMcoordinatorbehavior change.

Later status: independent22:51 capture and SDK layout correction are now complete; the preceding verification section is historical evidence, not current source parity. Corrected new-view full3D chain difference remains5.63093mm. Official detector/image-only comparisons did not consistently remove it. No installation calibration is active; independent flat-grid multiview color-camera observations are needed to separate remaining camera/printed-target model bias. Current evidence is in `artifacts/spatial_bench/rotation_layout_fix_20260905/acceptance.json` and handoff§30.

2026-09-06 scope correction: operator reiterates factory K/D remain authoritative. AprilGrid intrinsic recalibration was stopped without recorded samples. Do not fit or activate replacement K/D. Fixed-factory model-usage audit and its separate small software correction are documented in factory-camera-model.md; the historical request for free-intrinsic calibration above is not an active instruction.
