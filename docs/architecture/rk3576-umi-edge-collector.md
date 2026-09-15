# RK3576 UMI edge collector

## Product boundary

RK3576 owns the D405 and STM32/IMU/magnetic-encoder USB devices. It records,
seals, previews, and transfers data; the PC performs SLAM and data cleaning.
The Android App is not modified in this repository. Its later UMI integration
should control this board-side owner and display only the RGB preview, just as
the shared App controls EGO.

## Default recording profile (RSUSB v4)

- RGB recording: D405 `YUYV 1280x720@30` input to RK MPP H.265 Annex-B at
  6 Mbit/s.
- RGB preview: RK MPP H.264/RTP, `960x540@15`, 2 Mbit/s, GOP 30. This matches
  EGO's preview encoder shape and does not stream either IR eye to the App.
- Left/right IR: separate `Y8 1280x720@30` inputs to separate RK MPP H.265
  Annex-B streams, CBR 3.456 Mbit/s per eye, GOP 30. This is explicitly lossy.
- STM32: byte-exact 63-byte `stm32_combined_v1` packets at 921600 baud,
  including KT-EX9 IMU, magnetic encoder response, MCU timestamps, sequence,
  validity flags, and CRC.
- Indexes retain each camera sequence, device global timestamp, host monotonic
  observation, and each raw STM32 packet's host monotonic observation.

Schema v3 remains available as the lossless rollback:

```text
--ir-encoding y8_split_zstd
```

Each completed session is published only after strict continuity, rate,
payload, queue, CRC, encoder, storage, and SHA-256 checks. Failed capture stays
as an explicitly unsealed hidden `.partial` directory.

## Board commands

Installed candidate:

```text
/home/pi/rk3576-umi-collector-candidate-20260902
```

The following now uses v4 by default:

```bash
cd /home/pi/rk3576-umi-collector-candidate-20260902
PYTHONPATH="$PWD:/home/pi/rsusb-port/runtime" \
python3 -m three_device_slam.edge_rk3576.rsusb_capture \
  --output-root /home/pi/rk3576-umi-sessions \
  --duration 30 \
  --d405-sdk-serial 260322279785 \
  --d405-usb-serial 260423073555
```

For an EGO-shaped App preview ingress, add:

```text
--preview-rtp-host MANAGER_IP --preview-rtp-port 5004
```

`--until-signal` changes duration into a hard maximum and seals on the first
SIGINT/SIGTERM after aligning RGB and both IR streams. A second signal forces
an unsealed abort.

## PC validation and SLAM conversion

After copying a compact v4 sealed directory, validate every encoded frame:

```bash
python3 -m three_device_slam.edge_rk3576.validate SESSION \
  --decode-rgb --decode-ir
```

For a lossless split-eye v3 session, use `--decode-rgb`; the normal validator
already streams both Zstd files to EOF and verifies their exact decompressed
byte counts without creating raw files on disk.

Create an immutable derived input for the existing docker2 pipeline:

```bash
source /opt/ros/humble/setup.bash
python3 -m three_device_slam.edge_rk3576.export_slam_session \
  SESSION DERIVED_SESSION
```

The exporter accepts both lossless split-eye v3/Zstd and compact split-eye
v4/H.265 sessions. The export contains two `sensor_msgs/Image mono8` streams
and their timestamp metadata in `umi_ir.db3`, `d405_frames.csv`, and the
existing packed `<dI7f>` IMU sidecar. `source_provenance.json` binds all
derived files to the original manifest. The board source is never rewritten.

For this exact docker2 assembly, use the signed calibration in:

```text
/home/robot/releases/umi_device2_d405_product_1.0.2-20260901/formal_runtime_calibration
```

The formal 30 Hz VINS executable is the `loop_ws` build (SHA-256
`5059b188...37973`), not the separate 15 Hz `vins_ws` stability build.

RK3576 sessions are already camera-warmed before their sealed frame zero.  Run
the Docker2 postprocessor with `--skip-s 0`; its generic 1.5-second startup
skip moves the SLAM origin into the user's motion and makes a physical
start/end closure look falsely offset.

## Measured evidence

The 30-second moving HIL session
`rk3576-rsusb-20260903T192219Z-d38ff7e0` contains 900 synchronized RGB/stereo
pairs and 12,000 STM32 packets at 400.029 Hz. It has no sequence, timestamp,
payload, CRC, or queue errors. Its complete artifact is 52,073,555 bytes
(about 1.74 MB/s); the two IR streams are 25,833,867 bytes total. At roughly
30 GiB usable capacity after reserve, this rate projects to about 5.1 hours.

Host validation decoded RGB and both IR eyes at exactly 900 frames each. The
signed docker2 30 Hz SLAM run produced 843 raw and corrected poses from 855
expected post-skip samples (98.60% coverage), 59 final pose-graph keyframes,
minimum 174 tracked features, zero input/backlog drops, and a maximum raw step
of 8.58 mm. The run verdict is PASS.

## Remaining App integration

The collector already provides the data and H.264/RTP preview plane. A future
board Manager adapter still needs to expose authenticated start/stop/status,
preview signaling, catalog entries, immutable Range transfer, receipts, and
verified deletion under the shared EGO/UMI/third-device App contract. Do not
make the App or another process open the D405/STM32 devices directly.
