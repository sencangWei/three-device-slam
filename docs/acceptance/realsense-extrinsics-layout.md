# RealSense rotation layout correction

The SDK flat9 rotation is column-major, source→target. Its original serializers copied it into a `rotation_row_major` field, causing readers to use the transposed rotation **without** the corresponding inverse translation. This is an array-layout defect, not a valid transform inversion.

## New snapshots

The three local producers (pair color capture, RSUSB D405 group snapshot, D435i worker) use `serialize_sdk_extrinsics`. New nested extrinsics carry:

- `schema: ego.realsense.extrinsics.v2`
- `source: librealsense`, `source_rotation_layout: column_major`
- `rotation_layout: row_major`, with correctly reordered `rotation_row_major`
- original meter translation and source→target convention.

No SDK/hardware import is needed for serialization. SDK float32 roundoff is projected to SO(3) at read time only, after rejecting nonfinite, malformed, reflected, or materially nonorthogonal rotations.

## Old sealed data

`load_color_to_ir_snapshot` accepts an exact SHA256 allowlist of **complete raw calibration files** independently traced to the faulty color-capture serializer. Both21:59/22:51 sessions share the audited D435i and D405 files. An exact match permits read-time transpose and records the hash/policy; no file is rewritten.

This is deliberately not inferred from a serial, schema, filename, or small rotation. Reformatting a known file changes its hash and requires re-audit. Unknown SDK-marked old snapshots fail with `unknown legacy SDK rotation layout`; do not “fix” this by deleting provenance or adding arbitrary hashes. Audit producer and SDK point semantics first. Unmarked old `rotation_row_major` inputs retain their declared semantics; explicit unknown/reverse conventions are rejected.

Mixed crosscheck reports include `color_to_ir_read`, and new CLI output directories are `spatial/mixed80_40_apriltag_layout_v2` or `spatial/mixed80_40_layout_v2`. They refuse overwrite and do not replace the earlier reports. Old uniform40mm tools must not be used on mixed80/40mm sessions.

## Verification scope

Tests cover nonsymmetric rotations, all three producer paths, inverse direction, true legacy row-major compatibility, audited SDK snapshots, malformed arrays/values, reflections, conflicting/unknown metadata, and changed legacy bytes. New report outputs remain unactivated.

Two archived598-pair datasets were recomposed with frozen camera poses: old agreement reproduced, corrected SDK point mapping agrees within2.1e-8m for tested points. Mount estimates change0.21–0.244mm and0.536deg. Complete mount-chain differences remain4.84594/5.63093mm because applying one common rigid transform preserves that norm. **This bug is fixed but is not the cause of the remaining chain disagreement.**

Evidence: `artifacts/spatial_bench/rotation_layout_fix_20260905/`; production/runtime calibration not activated, no external repository or `/opt` modifications, no hardware capture or robot motion in this correction.
