# Z2 edge collector v1

## Locked acquisition profile

The customized Z2 target records the same sensor evidence needed by the current UMI
pipeline while using RGB only as a bandwidth-efficient visual stream:

- D405 RGB: 1280 x 720 at 30 Hz, hardware H.265 Annex-B. This stream is lossy and is
  used for phone preview and downstream training images.
- D405 infrared left/right: 1280 x 720 at 30 Hz, native Y8 bytes without image or
  video compression. These streams are the lossless SLAM observations.
- STM32: the exact 63-byte `combined_v1` wire packet at nominally 400 Hz. Parsing
  metadata is indexed, but the original packet is always retained.

RGB H.265 is never described as lossless. Every RGB access unit and every IR
frameset carries the D405 frame identity, D405 timestamp and the board's monotonic
timestamp. Every STM32 packet carries its board arrival timestamp.

## Data path

```text
D405 USB 3.x ─┬─ RGB YUYV ─ hardware H.265 ─┬─ bounded preview queue ─ Wi-Fi app
              │                              └─ chunked RGB storage
              └─ left/right Y8 ───────────────── chunked lossless storage

STM32 CP2102N 921600 baud ─ raw 63-byte packets ─ chunked lossless storage
```

The network is not in the lossless recording path. It carries a rate-limited RGB
preview and performs resumable transfer after recording. A phone disconnect or
slow Wi-Fi must not stall or drop IR/STM32 storage.

## Session layout and crash behavior

`Z2EdgeSessionWriter` creates one new directory containing:

```text
.recording
session_config.json
infrared_framesets.jsonl
rgb_access_units.jsonl
stm32_packets.jsonl
data/*.bin
manifest.json                 # only after a successful seal
```

Payload files rotate at a configured chunk boundary. Index rows contain relative
file, offset, byte length and CRC-32 references. On seal, all payload/index files
are flushed and fsynced, and `manifest.json` freezes sizes and SHA-256 digests.
Only after the manifest is durable is `.recording` removed. Therefore power loss
cannot silently turn a partial recording into a valid session.

The PC validator rejects unsealed sessions, missing or extra files, truncation,
checksum corruption, unreferenced bytes, timestamp regressions, malformed H.265
access units, non-Y8 frame lengths, and malformed STM32 packets.

Run validation with:

```bash
python -m three_device_slam.edge_z2.validate /path/to/session
```

## Hardware integration gates

The portable format and validation core are implemented without assuming an
undocumented vendor ABI. The final Z2 capture executable remains blocked until the
custom board package exposes and verifies all of the following:

1. A USB 3.x/5 Gbit/s host path that can enumerate the D405 composite device and
   open RGB plus both infrared streaming interfaces simultaneously.
2. CP2102N serial access at 921600 baud while all camera streams are active.
3. Sustained storage throughput of at least 63.62 MB/s for dual Y8 plus STM32,
   including filesystem and chunk-rotation overhead.
4. A hardware H.265 encoder API accepting 1280 x 720 RGB frames without blocking
   the lossless capture path, plus an API for a bounded low-rate preview stream.
5. A documented build/deploy mechanism, service supervision, logs and a recoverable
   firmware update path.

The vendor statement that functionality and structure can be customized supports
requesting this package or a board revision. It does not by itself supply these
interfaces, their headers, or a safe flashing procedure.
