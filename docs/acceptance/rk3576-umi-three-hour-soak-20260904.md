# RK3576 UMI three-hour internal-storage soak — 2026-09-04

## Verdict

PASS for the requested three-hour storage, capture, and debug-preview soak using three App-compatible one-hour segments. All three sessions sealed normally and passed the board's strict manifest, checksum, timing, continuity, and queue validation with no warnings. Full long-file video decode was not part of this storage soak; the same deployed H.265 pipelines were decode-validated in the preceding bounded HIL runs.

## Configuration

- RK3576 collector: deployed RSUSB D405 + STM32 candidate
- RGB: 1280x720 at 30 Hz, H.265 Annex B
- Stereo IR: two 1280x720 Y8 streams at 30 Hz, separate H.265 Annex B files
- STM32: lossless raw packets at nominal 400 Hz
- Preview: 640x360 MJPEG at 10 Hz with an external client attached
- Storage: RK3576 internal overlay filesystem
- Segmentation: three consecutive 3,600-second sessions

## Results

| Session | RGB / stereo pairs | STM32 packets | Bytes | Strict validation |
|---|---:|---:|---:|---|
| `T072105Z-915543e7` | 108,000 / 108,000 | 1,440,080 | 6,295,228,871 | PASS, no warnings |
| `T082126Z-a2f00d02` | 108,000 / 108,000 | 1,440,081 | 6,293,656,968 | PASS, no warnings |
| `T092147Z-9945a414` | 108,000 / 108,000 | 1,440,081 | 6,294,227,381 | PASS, no warnings |

Aggregate formal capture span was 10,800.569 seconds. The artifacts contain 324,000 RGB frames, 324,000 synchronized stereo-IR pairs, and 4,320,242 STM32 packets. Total file bytes are 18,883,113,220 (18.88 GB / 17.59 GiB), or 1.748 MB/s averaged over the formal capture windows.

All camera streams measured 29.9983–29.9985 Hz. The largest observed camera arrival interval was 72.103 ms, below the 250 ms acceptance ceiling. Every stream recorded zero payload-size errors, sequence gaps, sequence regressions, and timestamp regressions. RGB queue depth peaked at one frame and stereo-IR at two frames; every recording queue reported zero overflow. RGB PTS regressions were zero.

STM32 measured 399.9997–400.0019 Hz. CRC errors, discarded bytes, sequence gaps, sequence regressions, invalid encoder flags, and invalid IMU flags were all zero.

The preview publisher produced exactly 108,000 preview frames at 10 Hz. The external client received 105,865 frames reported by the board and at least 2,994,167,297 bytes reported by the host. The 2.0% undelivered portion is dominated by the client being attached after the first segment began; only 32 preview frames were marked skipped across all segments. Expected connection refusals occurred only while one segment was sealing and the next preview server was starting. `reader_failure` was null in all three sessions, and recording queues were unaffected.

The highest manifest thermal sample was 51.769 C. No GStreamer log contained an error/failure entry. Captured kernel USB records contained only D405 enumeration messages, with no disconnect, reset, protocol, or transfer error. After the soak, the filesystem retained 29,659,361,280 bytes (27.62 GiB) free.

## Seven-hour readiness

At the measured 1.748 MB/s, seven hours require approximately 44.06 GB. Deleting only these validated three-hour soak artifacts restores about 48.54 GB free, leaving an estimated 4.48 GB after seven hours. This remains above the deployed collector's 2.762 GiB low-storage stop threshold, but the margin is intentionally modest; the built-in guard must remain enabled. The seven-hour run will start only after the operator supplies its start time.
