# UMI calibration tag print assets

Run from the repository root:

```bash
python3 scripts/generate_umi_apriltags.py
```

The A4 PDF is the authoritative print asset. Print with `Actual size` / 100%
scaling. The measured black tag square is 40.0 mm; the full 10x10 grid,
including one white quiet-zone cell on each edge, is 50.0 mm.

- left UMI: AprilTag `tag36h11`, ID `0`
- right UMI: AprilTag `tag36h11`, ID `1`

The human-readable labels are outside the tag quiet zone. Measure both the
40 mm black square and the 100 mm ruler after printing. Record the measured
tag size in calibration metadata rather than assuming the printer is exact.

## Mounting rules (learned from bench bring-up, 2026-09-04)

- **Light-colored, flat rigid backing is mandatory.** The detector finds the
  tag by its outer square contour; a black tag border mounted on a black
  surface gives zero outer-edge contrast and no detection candidates at all
  (observed: 20 rejected quads elsewhere in the scene, none on the tag).
  Mount on white paper/card with >= 10 mm of visible white margin around
  the black square.
- **Polarity note for the OpenCV reference backend:** the printed pattern
  (white data cells on black square, as in these assets) is what
  `cv2.aruco.DICT_APRILTAG_36H11` expects; do not invert the artwork.
- A worn/dirty print may fail to decode even when sharp; reprint rather
  than tuning detector thresholds.

Source patterns are pinned to
`AprilRobotics/apriltag-imgs@f3fd9a7add5bfd82a886fc65240fdb8e3c9ac5a1`.
