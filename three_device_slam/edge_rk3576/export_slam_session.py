"""Export a sealed RK3576 UMI split-eye session for frozen PC SLAM replay.

The board recording remains immutable.  This command decodes either the v3
lossless Zstd streams or the v4 compact HEVC streams into a derived ROS 2 sqlite
bag and converts validated STM32 packets to the existing 40-byte KT-EX9 replay
sidecar.
"""

from __future__ import annotations

import argparse
import csv
import gc
import hashlib
import json
import os
import shutil
import struct
import subprocess
import uuid
from pathlib import Path
from typing import Any, Iterator

from three_device_slam.devices.d405_umi.imu.stream_protocol import parse_combined

from .capture import HEIGHT, STM32_PACKET_BYTES, WIDTH
from .validate import (
    RSUSB_SPLIT_H265_SCHEMA,
    RSUSB_SPLIT_ZSTD_SCHEMA,
    validate_rk3576_session,
)


LEFT_DATA = "/device_0/sensor_0/Infrared_1/image/data"
LEFT_META = "/device_0/sensor_0/Infrared_1/image/metadata"
RIGHT_DATA = "/device_0/sensor_0/Infrared_2/image/data"
RIGHT_META = "/device_0/sensor_0/Infrared_2/image/metadata"
IMU_RECORD = struct.Struct("<dI7f")


class SlamExportError(RuntimeError):
    """The source cannot be represented by the frozen SLAM replay contract."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _jsonl(path: Path) -> Iterator[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise SlamExportError(
                    f"{path.name}:{line_number} is invalid JSON: {exc}"
                ) from exc
            if not isinstance(row, dict):
                raise SlamExportError(f"{path.name}:{line_number} is not an object")
            yield row


def _stamp(seconds: float):
    from builtin_interfaces.msg import Time

    nanoseconds = round(seconds * 1_000_000_000)
    return Time(
        sec=nanoseconds // 1_000_000_000,
        nanosec=nanoseconds % 1_000_000_000,
    )


def _topic_metadata(name: str, message_type: str):
    import rosbag2_py

    return rosbag2_py.TopicMetadata(
        name=name,
        type=message_type,
        serialization_format="cdr",
    )


class _H265IrReader:
    def __init__(self, path: Path):
        try:
            import cv2
        except ImportError as exc:
            raise SlamExportError(f"OpenCV with FFmpeg is required: {exc}") from exc
        self._cv2 = cv2
        self._capture = cv2.VideoCapture(str(path))
        if not self._capture.isOpened():
            raise SlamExportError(f"cannot decode {path.name}")

    def read_frame(self, label: str, frame_index: int) -> bytes:
        ok, frame = self._capture.read()
        if not ok:
            raise SlamExportError(f"{label} ended before indexed frame {frame_index}")
        if frame.shape[:2] != (HEIGHT, WIDTH):
            raise SlamExportError(f"{label} frame {frame_index} shape is {frame.shape}")
        if frame.ndim == 3:
            frame = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2GRAY)
        payload = frame.tobytes()
        if len(payload) != WIDTH * HEIGHT:
            raise SlamExportError(f"{label} frame {frame_index} is not mono8")
        return payload

    def assert_exhausted(self, label: str) -> None:
        ok, _frame = self._capture.read()
        self._capture.release()
        if ok:
            raise SlamExportError(f"{label} contains unindexed trailing frames")

    def close(self) -> None:
        self._capture.release()


class _ZstdIrReader:
    def __init__(self, path: Path):
        executable = shutil.which("zstd")
        if executable is None:
            raise SlamExportError("zstd is required to decode lossless IR")
        self._path = path
        self._process = subprocess.Popen(
            [executable, "--decompress", "--quiet", "--stdout", str(path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if self._process.stdout is None or self._process.stderr is None:
            self.close()
            raise SlamExportError(f"cannot open zstd decoder for {path.name}")

    def read_frame(self, label: str, frame_index: int) -> bytes:
        assert self._process.stdout is not None
        remaining = WIDTH * HEIGHT
        chunks = []
        while remaining:
            chunk = self._process.stdout.read(remaining)
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        payload = b"".join(chunks)
        if len(payload) != WIDTH * HEIGHT:
            raise SlamExportError(
                f"{label} ended before indexed frame {frame_index}: "
                f"{len(payload)} of {WIDTH * HEIGHT} bytes"
            )
        return payload

    def assert_exhausted(self, label: str) -> None:
        assert self._process.stdout is not None
        assert self._process.stderr is not None
        trailing = self._process.stdout.read(1)
        if trailing:
            self.close()
            raise SlamExportError(f"{label} contains unindexed trailing bytes")
        stderr = self._process.stderr.read().decode("utf-8", errors="replace")
        returncode = self._process.wait()
        if returncode != 0:
            raise SlamExportError(
                f"{label} zstd decoder failed: {stderr.strip() or returncode}"
            )

    def close(self) -> None:
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()


def _open_ir_reader(path: Path, encoding: str):
    if encoding == "y8_split_h265":
        return _H265IrReader(path)
    if encoding == "y8_split_zstd":
        return _ZstdIrReader(path)
    raise SlamExportError(f"unsupported split-eye IR encoding: {encoding}")


def _open_ir_video(path: Path):
    """Compatibility wrapper retained for callers of the original v4 helper."""
    try:
        import cv2
    except ImportError as exc:
        raise SlamExportError(f"OpenCV with FFmpeg is required: {exc}") from exc
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise SlamExportError(f"cannot decode {path.name}")
    return capture


def _mono8_frame(capture, label: str, frame_index: int) -> bytes:
    import cv2

    ok, frame = capture.read()
    if not ok:
        raise SlamExportError(f"{label} ended before indexed frame {frame_index}")
    if frame.shape[:2] != (HEIGHT, WIDTH):
        raise SlamExportError(f"{label} frame {frame_index} shape is {frame.shape}")
    if frame.ndim == 3:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    payload = frame.tobytes()
    if len(payload) != WIDTH * HEIGHT:
        raise SlamExportError(f"{label} frame {frame_index} is not mono8")
    return payload


def _assert_video_exhausted(capture, label: str) -> None:
    ok, _frame = capture.read()
    capture.release()
    if ok:
        raise SlamExportError(f"{label} contains unindexed trailing frames")


def _write_image_pair(
    writer,
    *,
    row: dict[str, Any],
    left_payload: bytes,
    right_payload: bytes,
) -> None:
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    device_ms = float(row["left_source_timestamp_ms"])
    right_device_ms = float(row["right_source_timestamp_ms"])
    if abs(device_ms - right_device_ms) > 0.1:
        raise SlamExportError(
            f"stereo timestamp skew exceeds 0.1 ms at frame {row['record_index']}"
        )
    seconds = device_ms / 1000.0
    bag_timestamp_ns = round(seconds * 1_000_000_000)
    sequence = int(row["sequence"])
    for topic, meta_topic, frame_id, payload, timestamp_ms in (
        (LEFT_DATA, LEFT_META, "cam0", left_payload, device_ms),
        (RIGHT_DATA, RIGHT_META, "cam1", right_payload, right_device_ms),
    ):
        message = Image()
        message.header.stamp = _stamp(timestamp_ms / 1000.0)
        message.header.frame_id = frame_id
        message.height = HEIGHT
        message.width = WIDTH
        message.encoding = "mono8"
        message.is_bigendian = False
        message.step = WIDTH
        message.data = payload
        metadata = String()
        metadata.data = f"frame_number={sequence} timestamp={timestamp_ms:.6f}"
        writer.write(topic, serialize_message(message), bag_timestamp_ns)
        writer.write(meta_topic, serialize_message(metadata), bag_timestamp_ns)


def _write_frame_clock_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = [
        "set_index",
        "infrared_left_frame_number",
        "infrared_left_device_ms",
        "infrared_left_mono",
        "infrared_left_domain",
        "infrared_right_frame_number",
        "infrared_right_device_ms",
        "infrared_right_mono",
        "infrared_right_domain",
    ]
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, row in enumerate(rows):
            host_monotonic_s = int(row["observed_host_monotonic_ns"]) / 1e9
            writer.writerow(
                {
                    "set_index": index,
                    "infrared_left_frame_number": int(row["left_sequence"]),
                    "infrared_left_device_ms": float(
                        row["left_source_timestamp_ms"]
                    ),
                    "infrared_left_mono": f"{host_monotonic_s:.9f}",
                    "infrared_left_domain": "global_time",
                    "infrared_right_frame_number": int(row["right_sequence"]),
                    "infrared_right_device_ms": float(
                        row["right_source_timestamp_ms"]
                    ),
                    "infrared_right_mono": f"{host_monotonic_s:.9f}",
                    "infrared_right_domain": "global_time",
                }
            )
        stream.flush()
        os.fsync(stream.fileno())


def _write_imu_sidecar(source: Path, destination: Path) -> int:
    rows = list(_jsonl(source / "stm32_packets.jsonl"))
    count = 0
    with (source / "stm32.bin").open("rb") as payload, destination.open("xb") as out:
        for expected, row in enumerate(rows):
            packet = payload.read(STM32_PACKET_BYTES)
            if len(packet) != STM32_PACKET_BYTES:
                raise SlamExportError(f"STM32 payload truncated at packet {expected}")
            parsed = parse_combined(packet)
            if row.get("record_index") != expected or row.get("sequence") != parsed.sequence:
                raise SlamExportError(f"STM32 index mismatch at packet {expected}")
            timestamp_s = int(row["host_read_complete_monotonic_ns"]) / 1e9
            out.write(
                IMU_RECORD.pack(
                    timestamp_s,
                    parsed.counter,
                    parsed.gx,
                    parsed.gy,
                    parsed.gz,
                    parsed.ax,
                    parsed.ay,
                    parsed.az,
                    parsed.temperature_c,
                )
            )
            count += 1
        if payload.read(1):
            raise SlamExportError("STM32 payload contains unindexed trailing bytes")
        out.flush()
        os.fsync(out.fileno())
    return count


def export_slam_session(source: str | Path, output: str | Path) -> Path:
    source = Path(source).resolve()
    output = Path(output).resolve()
    manifest = json.loads((source / "manifest.json").read_text(encoding="utf-8"))
    schema = manifest.get("schema")
    if schema not in {RSUSB_SPLIT_ZSTD_SCHEMA, RSUSB_SPLIT_H265_SCHEMA}:
        raise SlamExportError("SLAM export requires an RK3576 UMI split-eye session")
    ir_encoding = manifest.get("profile", {}).get("infrared", {}).get("encoding")
    report = validate_rk3576_session(
        source,
        decode_rgb=False,
        decode_ir=schema == RSUSB_SPLIT_H265_SCHEMA,
    )
    if output.exists():
        raise SlamExportError(f"output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = output.parent / f".{output.name}.partial-{uuid.uuid4().hex[:8]}"
    stage.mkdir(mode=0o700)
    try:
        try:
            import rosbag2_py
            from rclpy.serialization import serialize_message  # noqa: F401
        except ImportError as exc:
            raise SlamExportError(
                "ROS 2 Humble Python environment is required; source setup.bash"
            ) from exc

        rows = list(_jsonl(source / "ir_frames.jsonl"))
        if ir_encoding == "y8_split_h265":
            left_path = source / "infrared-left-y8.h265"
            right_path = source / "infrared-right-y8.h265"
            ir_conversion = "H.265 decoded through OpenCV FFmpeg to ROS Image mono8"
        elif ir_encoding == "y8_split_zstd":
            left_path = source / "infrared-left-y8.raw.zst"
            right_path = source / "infrared-right-y8.raw.zst"
            ir_conversion = "lossless Zstd streamed to ROS Image mono8"
        else:
            raise SlamExportError(f"unsupported split-eye IR encoding: {ir_encoding}")
        left = None
        right = None
        try:
            left = _open_ir_reader(left_path, ir_encoding)
            right = _open_ir_reader(right_path, ir_encoding)
            bag_directory = stage / "_rosbag"
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(
                    uri=str(bag_directory), storage_id="sqlite3"
                ),
                rosbag2_py.ConverterOptions("", ""),
            )
            for name, message_type in (
                (LEFT_DATA, "sensor_msgs/msg/Image"),
                (LEFT_META, "std_msgs/msg/String"),
                (RIGHT_DATA, "sensor_msgs/msg/Image"),
                (RIGHT_META, "std_msgs/msg/String"),
            ):
                writer.create_topic(_topic_metadata(name, message_type))
            for index, row in enumerate(rows):
                if row.get("record_index") != index:
                    raise SlamExportError(
                        f"IR record index mismatch at frame {index}"
                    )
                _write_image_pair(
                    writer,
                    row=row,
                    left_payload=left.read_frame("left IR", index),
                    right_payload=right.read_frame("right IR", index),
                )
            left.assert_exhausted("left IR")
            right.assert_exhausted("right IR")
        finally:
            if left is not None:
                left.close()
            if right is not None:
                right.close()
        del writer
        gc.collect()

        bags = list(bag_directory.glob("*.db3"))
        if len(bags) != 1:
            raise SlamExportError(f"ROS bag writer produced {len(bags)} DB3 files")
        bag_path = stage / "umi_ir.db3"
        os.replace(bags[0], bag_path)
        metadata_path = bag_directory / "metadata.yaml"
        if metadata_path.is_file():
            shutil.copy2(metadata_path, stage / "rosbag_metadata.yaml")
        shutil.rmtree(bag_directory)

        _write_frame_clock_csv(stage / "d405_frames.csv", rows)
        (stage / "external_imu").mkdir()
        imu_count = _write_imu_sidecar(
            source, stage / "external_imu" / "imu.bin"
        )
        files = {}
        for path in sorted(item for item in stage.rglob("*") if item.is_file()):
            relative = path.relative_to(stage).as_posix()
            files[relative] = {
                "size": path.stat().st_size,
                "sha256": _sha256(path),
            }
        provenance = {
            "schema": "three-device-slam.rk3576-umi-slam-export.v1",
            "source_session": str(source),
            "source_session_id": report["session_id"],
            "source_manifest_sha256": _sha256(source / "manifest.json"),
            "source_schema": manifest["schema"],
            "conversion": {
                "ir": ir_conversion,
                "imu": "validated stm32_combined_v1 to packed <dI7f>",
                "clock": "camera global_time mapped to board host monotonic arrival",
            },
            "counts": {
                "stereo_pairs": len(rows),
                "image_messages": len(rows) * 2,
                "metadata_messages": len(rows) * 2,
                "imu_records": imu_count,
            },
            "files": files,
        }
        (stage / "source_provenance.json").write_text(
            json.dumps(provenance, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.replace(stage, output)
        return output
    except BaseException:
        shutil.rmtree(stage, ignore_errors=True)
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args(argv)
    try:
        result = export_slam_session(args.source, args.output)
    except (SlamExportError, ValueError, OSError) as exc:
        print(json.dumps({"status": "FAIL", "reason": str(exc)}, ensure_ascii=False))
        return 1
    print(json.dumps({"status": "PASS", "output": str(result)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
