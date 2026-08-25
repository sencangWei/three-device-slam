"""KT-EX9 37-byte and STM32 v1 63-byte stream decoding primitives."""

from __future__ import annotations

import struct
from dataclasses import dataclass


RAW_HEADER = b"\xeb\x90\x22"
COMBINED_HEADER = b"\xa5\x5a"
RAW_SIZE = 37
COMBINED_SIZE = 63
UINT32 = 1 << 32


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = (
                ((crc << 1) ^ 0x1021) & 0xFFFF
                if crc & 0x8000
                else (crc << 1) & 0xFFFF
            )
    return crc


def raw_imu_checksum_valid(frame: bytes) -> bool:
    return len(frame) == RAW_SIZE and (sum(frame[:36]) & 0xFF) == frame[36]


@dataclass(frozen=True)
class ImuPacket:
    protocol: str
    counter: int
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float
    temperature_c: float
    raw_packet: bytes
    sequence: int | None = None
    flags: int = 0
    imu_first_byte_rx_us: int | None = None
    encoder_read_us: int | None = None
    encoder_response: int | None = None

    @property
    def imu_valid(self) -> bool:
        return self.protocol == "kt_ex9_37" or bool(self.flags & 0x01)


def parse_raw_imu(frame: bytes) -> ImuPacket:
    if len(frame) != RAW_SIZE or frame[:3] != RAW_HEADER:
        raise ValueError("invalid KT-EX9 frame header/length")
    if not raw_imu_checksum_valid(frame):
        raise ValueError("invalid KT-EX9 checksum")
    gx, gy, gz, ax, ay, az, temperature = struct.unpack_from("<7f", frame, 4)
    counter = struct.unpack_from("<I", frame, 32)[0]
    return ImuPacket(
        protocol="kt_ex9_37",
        counter=counter,
        gx=gx,
        gy=gy,
        gz=gz,
        ax=ax,
        ay=ay,
        az=az,
        temperature_c=temperature,
        raw_packet=frame,
    )


def parse_combined(packet: bytes) -> ImuPacket:
    if len(packet) != COMBINED_SIZE or packet[:2] != COMBINED_HEADER:
        raise ValueError("invalid combined packet header/length")
    if packet[2] != 1 or packet[3] != COMBINED_SIZE:
        raise ValueError("unsupported combined packet version/length")
    expected_crc = struct.unpack_from("<H", packet, 61)[0]
    if crc16_ccitt_false(packet[:61]) != expected_crc:
        raise ValueError("invalid combined packet CRC")
    flags = struct.unpack_from("<H", packet, 4)[0]
    sequence, imu_us, encoder_us, outer_counter = struct.unpack_from(
        "<IIII", packet, 6
    )
    encoder_response = struct.unpack_from("<H", packet, 22)[0]
    embedded = parse_raw_imu(packet[24:61])
    if embedded.counter != outer_counter:
        raise ValueError("combined/embedded IMU counter mismatch")
    return ImuPacket(
        protocol="stm32_combined_v1",
        counter=embedded.counter,
        gx=embedded.gx,
        gy=embedded.gy,
        gz=embedded.gz,
        ax=embedded.ax,
        ay=embedded.ay,
        az=embedded.az,
        temperature_c=embedded.temperature_c,
        raw_packet=packet,
        sequence=sequence,
        flags=flags,
        imu_first_byte_rx_us=imu_us,
        encoder_read_us=encoder_us,
        encoder_response=encoder_response,
    )


class StreamDecoder:
    """Resynchronizing decoder for either supported serial protocol."""

    def __init__(self, protocol: str = "auto", *, on_raw_candidate=None) -> None:
        if protocol not in {"auto", "kt_ex9_37", "stm32_combined_v1"}:
            raise ValueError(f"unsupported protocol: {protocol}")
        self.protocol = protocol
        self.on_raw_candidate = on_raw_candidate
        self.buffer = bytearray()
        self.crc_or_checksum_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[ImuPacket]:
        self.buffer.extend(data)
        packets = []
        while True:
            choices = []
            if self.protocol in {"auto", "stm32_combined_v1"}:
                choices.append(
                    (self.buffer.find(COMBINED_HEADER), COMBINED_SIZE, parse_combined)
                )
            if self.protocol in {"auto", "kt_ex9_37"}:
                choices.append((self.buffer.find(RAW_HEADER), RAW_SIZE, parse_raw_imu))
            found = [choice for choice in choices if choice[0] >= 0]
            if not found:
                keep = 2 if self.protocol != "stm32_combined_v1" else 1
                if len(self.buffer) > keep:
                    removed = len(self.buffer) - keep
                    del self.buffer[:removed]
                    self.discarded_bytes += removed
                break
            index, size, parser = min(found, key=lambda item: item[0])
            if index:
                del self.buffer[:index]
                self.discarded_bytes += index
            if len(self.buffer) < size:
                break
            candidate = bytes(self.buffer[:size])
            if self.on_raw_candidate is not None:
                self.on_raw_candidate(candidate)
            try:
                packets.append(parser(candidate))
                del self.buffer[:size]
            except ValueError:
                self.crc_or_checksum_errors += 1
                # Never expose the embedded legacy frame when a syntactically
                # valid STM32 envelope fails CRC or internal consistency.
                discard = (
                    size
                    if parser is parse_combined
                    and candidate[:4]
                    == COMBINED_HEADER + bytes((1, COMBINED_SIZE))
                    else 1
                )
                del self.buffer[:discard]
                self.discarded_bytes += discard
        return packets


class TimerUnwrapper:
    def __init__(self) -> None:
        self.previous: int | None = None
        self.epoch = 0

    def extend(self, value: int) -> int:
        value &= 0xFFFFFFFF
        if (
            self.previous is not None
            and value < self.previous
            and self.previous - value > UINT32 // 2
        ):
            self.epoch += UINT32
        self.previous = value
        return self.epoch + value
