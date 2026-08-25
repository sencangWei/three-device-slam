from dataclasses import dataclass, field
from typing import Any, Mapping, Tuple


@dataclass(frozen=True)
class SensorRecord:
    stream_id: str
    sequence: int
    acquisition_ns: int
    arrival_ns: int
    clock_domain: str
    warmup: bool
    valid: bool
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        if not self.stream_id:
            raise ValueError("stream_id must be non-empty")
        if min(self.sequence, self.acquisition_ns, self.arrival_ns) < 0:
            raise ValueError("sequence and timestamps must be non-negative")
        if self.clock_domain == "host_monotonic" and self.arrival_ns < self.acquisition_ns:
            raise ValueError("arrival_ns precedes acquisition_ns")


@dataclass(frozen=True)
class FrameStamp:
    device_id: str
    sequence: int
    acquisition_ns: int
    payload_ref: str


@dataclass(frozen=True)
class DeviceHeartbeat:
    device_id: str
    at_ns: int
    healthy: bool
    reasons: Tuple[str, ...] = ()


@dataclass(frozen=True)
class Triplet:
    sample_ns: int
    ego: FrameStamp
    left: FrameStamp
    right: FrameStamp
    span_ns: int
    trainable: bool
    reason: str
