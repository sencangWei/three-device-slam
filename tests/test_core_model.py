import pytest

import three_device_slam.core as multidevice
from three_device_slam.core.model import FrameStamp, SensorRecord


def test_host_monotonic_arrival_cannot_precede_acquisition():
    with pytest.raises(ValueError, match="arrival_ns"):
        SensorRecord(
            "ego.video", 7, 2000, 1999, "host_monotonic", True, True, {}
        )


def test_frame_stamp_preserves_source_time_and_reference():
    stamp = FrameStamp("left", 42, 1_234_000_000, "left/frames/42")
    assert stamp.acquisition_ns == 1_234_000_000
    assert stamp.payload_ref == "left/frames/42"


def test_public_package_exports_session_writer():
    assert multidevice.AppendOnlySessionWriter.__name__ == "AppendOnlySessionWriter"
