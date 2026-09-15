# RK3576 UMI MJPEG preview 15 Hz HIL — 2026-09-04

## Verdict

PASS. The RK3576 RSUSB collector's debug MJPEG preview branch was changed from 10 Hz to 15 Hz without changing the 30 Hz RGB/dual-IR recording branches, the existing 15 Hz H.264/RTP branch, or STM32 capture.

## Evidence

- Deployed collector SHA-256: `2e4e5189b4629b1b7d800fd10debdb2d85fa84c0d7904dbf701b5add97fa9f16`
- Rollback copy SHA-256: `6513352a37df0e55e1ba989c52b616cf38fd70e24bb20e4a80574c8ab57953c6`
- Software regression: 49/49 tests passed.
- HIL session: `rk3576-rsusb-20260904T105727Z-bb0e45cd`
- Strict session validation: PASS with no warnings.
- Preview: 900 frames published over 60 seconds, 15.032 Hz observed, 825 delivered to the client, zero skipped frames, one client connection, no reader failure, and 4.668 ms maximum write time. The client was attached after capture startup and received 24,865,267 bytes during its bounded window.
- Recording: 1,800 RGB frames and 1,800 stereo-IR pairs; all three camera streams measured 30.0004 Hz with zero sequence gaps. RGB and IR queues peaked at one frame and had zero overflow.
- STM32: 24,000 packets at 399.998 Hz with zero CRC errors and zero sequence gaps.
- Thermal: 42.538 C at the end of the HIL window, below the 85 C stop threshold.
- No GStreamer error/failure entry; captured kernel USB evidence contained enumeration only.

The test artifact remains on the board pending the planned cleanup before the separately authorized seven-hour soak.
