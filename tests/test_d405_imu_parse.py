"""KT-EX9-2 IMU 帧解析单元测试(不依赖硬件)。"""
import struct
import sys
import types

import numpy as np
import pytest


from three_device_slam.devices.d405_umi.imu.imu_reader import (
    FRAME_SIZE, HEADER0, HEADER1, EXPECTED_LEN,
    ImuReader, verify_checksum, parse_frame, find_frames,
)


COMBINED_SIZE = 63


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def make_frame(gx=1.0, gy=2.0, gz=3.0, ax=0.0, ay=0.0, az=1.0, temp=25.0, counter=1):
    buf = bytearray(FRAME_SIZE)
    buf[0] = HEADER0
    buf[1] = HEADER1
    buf[2] = EXPECTED_LEN
    buf[3] = 0x01
    struct.pack_into("<7f", buf, 4, gx, gy, gz, ax, ay, az, temp)
    struct.pack_into("<I", buf, 32, counter)
    # 校验和: 字节[0..35] 累加低 8 位(含帧头)
    buf[36] = sum(buf[0:36]) & 0xFF
    return bytes(buf)


def make_combined(sequence=1, counter=1, imu_us=1000, flags=0x03):
    packet = bytearray(COMBINED_SIZE)
    packet[:4] = b"\xa5\x5a\x01\x3f"
    struct.pack_into(
        "<HIIIIH", packet, 4, flags, sequence, imu_us,
        (imu_us + 40) & 0xFFFFFFFF, counter, 0x1234,
    )
    packet[24:61] = make_frame(counter=counter)
    struct.pack_into("<H", packet, 61, crc16_ccitt_false(packet[:61]))
    return bytes(packet)


def test_checksum_valid():
    assert verify_checksum(make_frame()) is True


def test_checksum_bad():
    f = bytearray(make_frame())
    f[10] ^= 0xFF     # 破坏一个数据字节
    assert verify_checksum(bytes(f)) is False


def test_parse_values():
    f = make_frame(gx=10.5, gy=-20.0, gz=0.0, ax=0.1, ay=-0.2, az=9.8, temp=42.0, counter=123)
    # parse_frame 用 monotonic 时间戳，值不测
    s = parse_frame(f)
    assert s is not None
    assert abs(s.gx - 10.5) < 1e-4
    assert abs(s.gy - (-20.0)) < 1e-4
    assert abs(s.az - 9.8) < 1e-3     # float32 精度
    assert abs(s.temp - 42.0) < 1e-3
    assert s.counter == 123


def test_find_frames_in_stream():
    f1 = make_frame(counter=1)
    f2 = make_frame(counter=2)
    junk = b"\x00\x01\x02"
    stream = junk + f1 + junk + f2
    frames, tail, stats = find_frames(stream)
    assert len(frames) == 2
    assert stats["bad_checksum"] == 0


def test_reject_garbage_header():
    # header 不对 → parse_frame 拒绝
    bad_header = bytes([0x00] * FRAME_SIZE)
    assert parse_frame(bad_header) is None
    # 校验和明确不对 → verify 拒绝
    bad_chk = bytearray(make_frame())
    bad_chk[36] ^= 0xFF
    assert verify_checksum(bytes(bad_chk)) is False


def test_reader_does_not_publish_warmup_frames():
    received = []
    reader = ImuReader("unused", on_sample=received.append, warmup_frames=2)

    reader._handle(make_frame(counter=1))
    reader._handle(make_frame(counter=2))
    reader._handle(make_frame(counter=3))

    assert [sample.counter for sample in received] == [3]


def test_reader_formal_stats_exclude_warmup_transport_faults():
    reader = ImuReader("unused", warmup_frames=2)
    reader.frames_bad = 1
    reader.resyncs = 46

    reader._handle(make_frame(counter=10))
    reader._handle(make_frame(counter=12))
    reader._handle(make_frame(counter=13))

    formal = reader.stats_since_warmup()
    assert formal["frames_ok"] == 1
    assert formal["frames_bad"] == 0
    assert formal["resyncs"] == 0
    assert formal["dropped_frames"] == 0
    assert reader.warmup_stats()["dropped_frames"] == 1


def test_reader_formal_stats_retain_faults_after_warmup():
    reader = ImuReader("unused", warmup_frames=1)
    reader._handle(make_frame(counter=20))

    reader.frames_bad += 1
    reader.resyncs += 37
    reader._handle(make_frame(counter=22))

    formal = reader.stats_since_warmup()
    assert formal["frames_ok"] == 1
    assert formal["frames_bad"] == 1
    assert formal["resyncs"] == 37
    assert formal["dropped_frames"] == 1


def test_reader_decodes_stm32_combined_packet_and_uses_mcu_spacing():
    received = []
    reader = ImuReader("unused", on_sample=received.append, warmup_frames=0)

    reader._consume_data(
        make_combined(sequence=10, counter=20, imu_us=100000)
        + make_combined(sequence=11, counter=21, imu_us=102500)
    )

    assert [sample.counter for sample in received] == [20, 21]
    assert abs((received[1].ts - received[0].ts) - 0.0025) < 1e-9
    assert received[0].protocol == "stm32_combined_v1"
    assert received[0].sequence == 10
    assert received[0].encoder_response == 0x1234
    assert received[0].encoder_sensor_gap_us == 40
    assert abs((received[0].encoder_ts - received[0].ts) - 0.000040) < 1e-9
    assert reader.stats_since_warmup()["protocol"] == "stm32_combined_v1"
    assert reader.stats_since_warmup()["sequence_gaps"] == 0


def test_bad_stm32_crc_cannot_leak_embedded_legacy_frame():
    received = []
    reader = ImuReader("unused", on_sample=received.append, warmup_frames=0)
    damaged = bytearray(make_combined())
    damaged[-1] ^= 1

    reader._consume_data(bytes(damaged))

    assert received == []
    assert reader.stats_since_warmup()["frames_bad"] == 1
    assert reader.stats_since_warmup()["resyncs"] == COMBINED_SIZE


def test_valid_raw_packet_is_exposed_before_parsed_sample_callback():
    events = []
    packet = make_combined(sequence=10, counter=20)
    reader = ImuReader(
        "unused",
        on_raw_packet=lambda raw: events.append(("raw", raw)),
        on_sample=lambda sample: events.append(("sample", sample.counter)),
        warmup_frames=0,
    )

    reader._consume_data(packet)

    assert events == [("raw", packet), ("sample", 20)]


def test_crc_failed_wire_candidate_is_preserved_without_parsed_sample():
    raw_packets = []
    samples = []
    damaged = bytearray(make_combined(sequence=10, counter=20))
    damaged[-1] ^= 1
    reader = ImuReader(
        "unused",
        on_raw_packet=raw_packets.append,
        on_sample=samples.append,
        warmup_frames=0,
    )

    reader._consume_data(bytes(damaged))

    assert raw_packets == [bytes(damaged)]
    assert samples == []


def test_encoder_and_imu_stay_aligned_across_mcu_timer_wrap():
    received = []
    reader = ImuReader("unused", on_sample=received.append, warmup_frames=0)
    reader._consume_data(
        make_combined(sequence=1, counter=1, imu_us=(1 << 32) - 20)
        + make_combined(sequence=2, counter=2, imu_us=2480)
    )
    assert [sample.encoder_sensor_gap_us for sample in received] == [40, 40]
    assert (received[1].ts - received[0].ts) == pytest.approx(0.0025)
    assert (received[1].encoder_ts - received[0].encoder_ts) == pytest.approx(0.0025)


def test_stm32_transport_flags_and_sequence_gap_remain_visible():
    reader = ImuReader("unused", warmup_frames=0)
    reader._consume_data(make_combined(sequence=5, counter=30, imu_us=1000))
    reader._consume_data(
        make_combined(sequence=7, counter=31, imu_us=3500, flags=0x01 | (1 << 5))
    )

    formal = reader.stats_since_warmup()
    assert formal["sequence_gaps"] == 1
    assert formal["queue_overflow_flags"] == 1
    assert formal["invalid_imu_flags"] == 0


def test_reader_uses_continuous_host_clock_across_device_counter_resets():
    received = []
    reader = ImuReader("unused", on_sample=received.append, warmup_frames=0)
    counters = list(range(100, 130)) + list(range(1, 31))

    for counter in counters:
        reader._handle(make_frame(counter=counter))

    assert reader._clock_counter == len(counters)
    assert reader.counter_resets == 1
    assert np.all(np.diff([sample.ts for sample in received]) > 0.0)


def test_reader_preserves_small_forward_packet_gap_in_virtual_counter():
    reader = ImuReader("unused", warmup_frames=0)

    reader._handle(make_frame(counter=10))
    reader._handle(make_frame(counter=12))

    assert reader._clock_counter == 3


def test_reader_counts_repeated_one_as_one_reset_and_stalls():
    reader = ImuReader("unused", warmup_frames=0)

    for counter in (398, 399, 1, 1, 1, 2):
        reader._handle(make_frame(counter=counter))

    assert reader.counter_resets == 1
    assert reader.counter_stalls == 2
    assert reader.dropped_frames == 0
    assert reader._clock_counter == 6


def test_reader_reconnect_keeps_virtual_counter_and_reanchors_clock(monkeypatch):
    opened = []

    class FakeSerial:
        def __init__(self, port, baud, timeout):
            self.port = port
            self.baud = baud
            self.timeout = timeout
            self.closed = False
            opened.append(self)

        def reset_input_buffer(self):
            pass

        def close(self):
            self.closed = True

    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=FakeSerial))
    reader = ImuReader("/dev/serial/by-id/test", warmup_frames=0)
    assert reader._open_port() is True
    reader._clock_counter = 1234
    reader._ts_fitter.feed(reader._clock_counter, 10.0)

    reader._disconnect_serial("test")
    assert opened[0].closed is True
    assert reader._ser is None
    assert reader._try_reconnect() is True

    assert len(opened) == 2
    assert reader.serial_reconnects == 1
    assert reader._clock_counter == 1234
    assert reader._ts_fitter._last_output_ts is None


def test_start_rebuild_keeps_raw_candidate_callback(monkeypatch):
    raw_packets = []
    reader = ImuReader("unused", on_raw_packet=raw_packets.append, warmup_frames=0)
    monkeypatch.setattr(reader, "_open_port", lambda: True)

    class IdleThread:
        def __init__(self, **_kwargs):
            pass

        def start(self):
            pass

    monkeypatch.setattr("three_device_slam.devices.d405_umi.imu.imu_reader.threading.Thread", IdleThread)

    assert reader.start()
    packet = make_combined(sequence=3, counter=4)
    reader._decoder.feed(packet)

    assert raw_packets == [packet]


def test_reconnect_rebuild_keeps_raw_candidate_callback(monkeypatch):
    raw_packets = []
    reader = ImuReader("unused", on_raw_packet=raw_packets.append, warmup_frames=0)
    monkeypatch.setattr(reader, "_open_port", lambda: True)

    assert reader._try_reconnect()
    damaged = bytearray(make_combined(sequence=5, counter=6))
    damaged[-1] ^= 1
    reader._decoder.feed(bytes(damaged))

    assert raw_packets == [bytes(damaged)]


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"  {name}: OK")
    print("test_imu_parse: ALL PASSED")
