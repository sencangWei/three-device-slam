# RK3576 UMI collector application release

This installs the collector application and its RSUSB Python runtime. It is NOT
a bootable Rockchip firmware image and does not complete the shared App Manager,
WebRTC, catalog, or resumable-transfer integration.

## Supported target

- RK3576, aarch64, Ubuntu 24.04, Python 3.12, Rockchip kernel and GStreamer MPP.
- Tested runtime: librealsense 2.58.2, `FORCE_RSUSB_BACKEND=ON`, Release build.
- One D405 on USB 3.x (8086:0b5b) and one CP2102N-backed STM32 using the existing
  63-byte combined-v1 IMU/encoder packet protocol. Same model does not waive the
  packet-format requirement. Optional Zstd rollback also needs `zstd` installed.
- Native runtime dependencies: libusb-1.0, libudev, libstdc++, libgcc, libc, libm.
  MPP GStreamer plugins come from the board's compatible vendor OS image; they
  are not replaced by this application package. `check-host.sh` checks them.

## Install on another matching board

Copy the version directory, then run on that board:

```sh
bash install.sh /home/pi/umi-collector
/home/pi/umi-collector/current/bin/umi-record \
  --output-root /home/pi/rk3576-umi-sessions \
  --d405-sdk-serial 260322279785 --d405-usb-serial 260423073555 \
  --duration 60 --until-signal --preview-listen 0.0.0.0 --preview-port 8765
```

Version 0.1.1 restores REQUIRED `--d405-sdk-serial` and `--d405-usb-serial`.
The example IDs identify the original device assembly; replace BOTH IDs when
deploying to a different assembly and associate its own SLAM calibration.
The generic launcher does not embed an ID. The collector selects the requested
camera explicitly, verifies SuperSpeed and profiles, and records identities in
the session. If the selected device is missing it fails rather than substituting
another camera. STM32 still uses the existing CP2102N discovery; supply
`--stm32-port /dev/serial/by-id/<paired-board>` to select a particular serial board.
Version 0.1.0 auto-discovery remains an archived release and is not the current
selection. Neither version packages passwords, network settings or old calibration.

RGB and stereo IR record at 1280x720/30 Hz with H.265; debug MJPEG preview is
640x360/15 Hz. STM32 data remains raw. Ctrl+C seals the session. The maximum
session duration is 3600 seconds. Restarting separate sessions introduces a
gap; it is NOT seamless in-process segmentation.

If a new board lacks device access, an administrator can install the included
`99-umi-devices.rules` into `/etc/udev/rules.d/`, reload udev, add the recording
user to `video` and `dialout`, then reconnect devices and log in again. Installation
does not modify permissions, services, network, or start recording automatically.

Validate after recording:

```sh
/home/pi/umi-collector/current/bin/umi-validate /absolute/path/to/sealed-session
```

This checks hashes, packet CRCs, timestamps, indices, profiles and counters.
Full video decode (`--decode-rgb --decode-ir`) additionally requires OpenCV and
an H.265 decoder on the validating host. Per-device camera intrinsics come from
the attached device; camera/IMU mounting extrinsics must be calibrated per new
assembly for SLAM, not copied from the old device's Docker2 calibration.

## Rollback and replication

`releases/<version>` preserves each installed version and `current` selects it.
To roll back, stop any recording normally, create a temporary symlink to the
previous verified version, and atomically rename it over `current`.
The old `/home/pi/rk3576-umi-collector-candidate-20260902` tree is also retained.
No recording files are included in the release or deleted by the installer.

The local Git repository stores the exact application, native runtime, license,
tests, dependency notes, checksums and installer. Clone/bundle/archive it for
another board with the same base system. A full flashable OS image requires
separate image preparation; do not flash this application archive as firmware.dat.
