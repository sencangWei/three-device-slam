# Fixed-factory board prediction diagnostic

Purpose: isolate the unresolved cross-tag pixel bias without estimating replacement
camera intrinsics or distortion. No camera flash, new K/D, mount activation or SLAM
parameter changes. The output consists of independent grayscale images, not a
synchronized SLAM bag.

## Operator procedure

Use the same rigid6x6 board already used for UMI: blacktag35.2mm, gap10.56mm,
two-bit black border. These are the existing documented board dimensions, not a
fresh metrology certificate. Keep the print flat on its backing and avoid glare.

Capture four separate30s clips, with quality review between clips:

1. Ego, normal board orientation.
2. Ego, board rotated180degrees **within its plane**, printed face still visible.
3. UMI, normal board orientation.
4. UMI, board rotated180degrees within its plane.

For each clip, the assistant opens the selected camera preview first. Click the
preview and pressR only when ready. Follow the eight timed cues: center, left,
right, upper, lower, tiltleft, tiltright, near/far. Move gently, then hold each pose
still for about2seconds. Keep the whole board in view when possible. Moving the
camera with a fixed board is allowed; moving the board with a fixed camera is also
allowed. Do not loosen the UMI camera or mountTag attachment.

Q/ESC exits and preserves partial evidence. After30seconds the camera closes and
the terminal names the next step, but no next recording starts automatically.
Orientation metadata is an operator label, not an automatic measurement.

## Command for the first clip

```bash
cd /home/robot/three-device-slam
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. /opt/three-device-slam/venv/bin/python -u \
  scripts/color_aprilgrid_capture.py \
  --output artifacts/spatial_bench/board_factory_ego_normal_NEW \
  --role ego --board-orientation normal --duration 30 --preview-hz 5
```

Use a new output path per clip. For the other clips choose `--role left` for UMI
and/or `--board-orientation rotated180`. Do not start another process while the
current preview owns the camera. This file's command is a template; use the live
session path recorded in progress.md rather than overwriting previous artifacts.

## Analysis contract

- Validate PNG/index hashes, frame dimensions/counter/timestamps and the bound
  factory snapshot. No use of display-resized preview JPEG for metric analysis.
- Decode native-resolution board corners; validate printed board ID/orientation
  convention before constructing object points. Do not reuse the1-bit external
  tag interpretation for this2-bit board.
- Fix factory K/D and SDK distortion model. Fit only6DOF board pose per image.
- Freeze checkerboard-parity tag groups before analysis. Estimate pose from one
  group and predict the other; then swap groups. A held-out corner/tag must not
  influence its own pose fit, branch selection, or target-layout selection.
- Save per-tag pixel residual vectors, image coordinates, board coordinates,
  train/holdout identities, pose, and quality exclusions. Keep rejected samples.
- Compare residual direction/magnitude across camera, image position, board180
  rotation and viewpoint. These comparisons localize hypotheses; no single
  threshold or result uniquely proves lens, print, or algorithm causality.
- Preview visibility and elapsed capture duration are not calibration acceptance.
  Insufficient pose coverage or too few clear held-out observations require more
  evidence, not a relaxed accuracy threshold or fitted replacement K/D.

## Implemented diagnostic and 2026-09-06 result

`scripts/fixed_factory_board_diagnostic.py` reads the four frozen sessions named
in `SESSIONS`. It verifies PNG/index/setup bindings and only fits board pose using
training-parity normalized factory SDK rays. IPPE seeds are LM-refined on training
points, ranked on training pixel residual, never on held-out tags. At least six
training tags spanning three rows and columns are required. The per-tag finite /
15px gate is frozen before optional refinement. Preview-quality and an all-board
pose are NOT used to filter the metric comparison.

Legacy corner convention is fixed: tag center `(column,row)*45.76mm`, canonical
corner offsets `(+,-),(-,-),(-,+),(+,+)*17.6mm`. Verified independently on all36
tags of the excluded normal control image
`board_factory_ego_rot180_20260906T0028/ego/000000.png` (SHA256
`0d59256fa9515fa7dc3ad3644336182a7e1a6724197f4cb73a2327efe6487d7d`).
Do not use that falsely labeled clip as a rotated180 control.

The original OpenCV ArUco subpixel option uses a relative-module window limit
(`relativeCornerRefinmentWinSize=0.3`, OpenCV4.10). For these two-bit tags the actual
window is much smaller than the nominal5px. At filled black junctions the initial
contour corners stay inward. `--refine5` explicitly calls `cornerSubPix` with a
5px half-window, preserving original IDs and train/heldout membership. This is a
diagnostic correction for THIS legacy board, not a global spatial-tag change.

Known-geometry regression:50px tags with15px gaps filled by black junction squares
have corner RMS2.91548px before and0.000043px after the correction. A negative
control with tiny isolated5px dots instead of the actual filled junctions worsens
under5px refinement; consequently this operation must not be generalized to
arbitrary boards or the separate one-bit spatial tags.

Paired confirmation excludes development Ego-normal image0; all other199 raw
images remain accounted for, including insufficient-coverage frames. Across286
matched usable folds, every fold improves. These are correlated frames, not286
independent physical trials.

| Clip | Baseline held-out RMS(px) | Explicit5px RMS(px) | Explicit5px p95(px) |
|---|---:|---:|---:|
| Ego normal (49 confirmation images) | 2.3635 | 0.3475 | 0.6169 |
| Ego rotated180 | 2.4527 | 0.4195 | 0.7693 |
| UMI normal | 2.3193 | 0.2694 | 0.4550 |
| UMI rotated180 | 2.3368 | 0.2516 | 0.4297 |

Evidence: `artifacts/spatial_bench/fixed_factory_board_prediction_20260906_refine5/`
contains all per-corner predictions and `comparison_199.json`. The baseline report
is preserved in `fixed_factory_board_prediction_20260906_run1/`; later baseline_v2
reproduces its numerical results with current source hashes. The comparison script
checks matching source hashes, frame identities, valid IDs, fold status and splits.

This fixes a demonstrated corner-localization defect in the board diagnostic.
It does NOT prove factory intrinsics exact, printed metric dimensions exact,
or original one-bit external-tag chain agreement resolved. No K/D, mount transform,
runtime spatial detector or external repository was modified by this correction.
