from three_device_slam.acquisition.rsusb_pair import (
    capture_cue,
    counter_delta,
    cross_timing_report,
    formal_window_samples,
    nearest_timestamp_deltas_ns,
    motion_signal_label,
    motion_signal_status,
    parse_args,
    pre_capture_cue,
    preview_sources,
    required_capture_free_bytes,
    _show_pair_preview,
    umi_specs,
)
import pytest
from three_device_slam.devices.d435i_ego.capture import StreamSample


def test_pair_cli_requires_explicit_identities_calibration_and_protocol(tmp_path):
    args = parse_args(
        [
            "--session",
            str(tmp_path / "session"),
            "--ego-serial",
            "ego-serial",
            "--ego-calibration-id",
            "ego-cal",
            "--umi-serial",
            "umi-serial",
            "--umi-port",
            "/dev/serial/by-id/umi",
            "--umi-calibration-id",
            "umi-cal",
            "--umi-protocol",
            "stm32_combined_v1",
            "--duration",
            "3",
        ]
    )

    assert args.umi_device_id == "left"
    assert args.preview_hz == 5
    assert args.start_delay == 10
    assert args.mode == "bench_no_motion"
    assert args.umi_calibration_id == "umi-cal"
    assert args.umi_protocol == "stm32_combined_v1"


def test_preview_sources_show_ego_and_each_umi_left_ir():
    ego_image = (b"ego", 1280, 720, 101)
    right_image = (b"right", 1280, 720, 202)

    assert preview_sources(
        {"latest_images": {"ir_left": ego_image, "ir_right": (b"unused", 1, 1, 0)}},
        [
            {
                "spec": {"device_id": "right"},
                "state": {
                    "latest_images": {
                        "ir_left": right_image,
                        "ir_right": (b"unused", 1, 1, 0),
                    }
                },
            }
        ],
    ) == [
        ("Ego 左IR", ego_image),
        ("设备3 左IR", right_image),
    ]


def test_pair_preview_renders_both_camera_images_side_by_side():
    import cv2
    import numpy as np

    class PreviewCv2:
        def __init__(self):
            self.image = None

        def __getattr__(self, name):
            return getattr(cv2, name)

        def imshow(self, _window, image):
            self.image = image.copy()

        @staticmethod
        def waitKey(_delay):
            return 0

    fake = PreviewCv2()
    shape = (720, 1280)
    ego_image = (np.full(shape, 32, np.uint8).tobytes(), 1280, 720, 101)
    right_image = (np.full(shape, 224, np.uint8).tobytes(), 1280, 720, 202)

    stopped = _show_pair_preview(
        fake,
        np,
        {"latest_images": {"ir_left": ego_image}},
        [
            {
                "spec": {"device_id": "right"},
                "state": {"latest_images": {"ir_left": right_image}},
            }
        ],
        "准备完成｜10.0秒后开始｜保持静止",
    )

    assert stopped is False
    assert fake.image.shape == (360, 1280, 3)
    assert fake.image[180, 100].tolist() == [32, 32, 32]
    assert fake.image[180, 740].tolist() == [224, 224, 224]


def test_pre_capture_cue_reserves_adjustment_then_settle_time():
    assert "调整" in pre_capture_cue(10.0)
    assert "相对安装" in pre_capture_cue(2.1)
    assert pre_capture_cue(2.0) == "停止调整，正式采集即将开始，请完全保持静止"
    assert "调整" in pre_capture_cue(5.1, 15.0)
    assert pre_capture_cue(5.0, 15.0) == "停止调整，正式采集即将开始，请完全保持静止"
    assert "可调整Ego和设备3的位置" in pre_capture_cue(
        10.0, 15.0, "device3_anchor_hil"
    )
    assert "相机、IMU和ID1" in pre_capture_cue(
        10.0, 15.0, "device3_anchor_hil"
    )
    assert "可调整Ego和设备3的位置" in pre_capture_cue(
        5.1, 20.0, "device3_anchor_hil"
    )
    assert pre_capture_cue(5.0, 20.0, "device3_anchor_hil") == (
        "停止调整，正式采集即将开始，请完全保持静止"
    )


def test_shared_world_mode_has_unambiguous_motion_cues(tmp_path):
    args = parse_args(
        [
            "--session", str(tmp_path / "session"),
            "--ego-serial", "ego-serial",
            "--ego-calibration-id", "ego-cal",
            "--umi-serial", "umi-serial",
            "--umi-port", "/dev/serial/by-id/umi",
            "--umi-calibration-id", "umi-cal",
            "--umi-protocol", "stm32_combined_v1",
            "--duration", "60",
            "--mode", "shared_world_motion",
            "--start-delay", "10",
        ]
    )

    assert args.mode == "shared_world_motion"
    assert args.start_delay == 10
    assert "锁紧在同一刚性支架" in capture_cue(0, 60, args.mode)
    assert capture_cue(10, 60, args.mode).startswith("缓慢整体平移刚性支架")
    assert "禁止分别移动" in capture_cue(10, 60, args.mode)
    assert capture_cue(30, 60, args.mode).startswith("缓慢整体转动刚性支架")
    assert capture_cue(50, 60, args.mode).startswith("整体回到起点附近")


def test_shared_world_motion_signal_names_each_device():
    assert motion_signal_label(
        "shared_world_motion", 9.9, 1.0, {"right": 20.0}
    ) == ""
    assert motion_signal_label(
        "shared_world_motion", 12.0, 0.01, {"right": 20.0}
    ) == "当前转速：Ego=不足 设备3=已检测"
    assert motion_signal_label(
        "shared_world_motion", 12.0, 0.1, {"right": 4.0}
    ) == "当前转速：Ego=已检测 设备3=已检测"
    assert motion_signal_label(
        "shared_world_motion", 12.0, 0.8, {"right": 30.0}
    ) == "当前转速：Ego=过快，请减速 设备3=过快，请减速"


def test_device3_anchor_hil_has_short_chinese_operator_contract(tmp_path):
    args = parse_args(
        [
            "--session", str(tmp_path / "session"),
            "--ego-serial", "ego-serial",
            "--ego-calibration-id", "ego-cal",
            "--umi-device-id", "right",
            "--umi-serial", "umi-serial",
            "--umi-port", "/dev/serial/by-id/umi",
            "--umi-calibration-id", "umi-cal",
            "--umi-protocol", "stm32_combined_v1",
            "--duration", "45",
            "--mode", "device3_anchor_hil",
            "--start-delay", "15",
        ]
    )

    assert args.start_delay == 15
    assert "完全静止" in capture_cue(0, 45, args.mode)
    assert "移动设备3" in capture_cue(15, 45, args.mode)
    assert "转动设备3" in capture_cue(25, 45, args.mode)
    assert "短暂离开视野" in capture_cue(34, 45, args.mode)
    assert "回到起点" in capture_cue(40, 45, args.mode)


def test_device3_anchor_hil_motion_feedback_keeps_ego_fixed():
    assert motion_signal_label(
        "device3_anchor_hil", 15.0, 0.01, {"right": 5.0}
    ) == "当前状态：Ego=静止正常 设备3=已检测"
    assert "发生移动" in motion_signal_label(
        "device3_anchor_hil", 15.0, 0.1, {"right": 5.0}
    )
    assert "静止正常" in motion_signal_label(
        "device3_anchor_hil", 40.0, 0.01, {"right": 1.0}
    )


def test_device3_anchor_hil_motion_gate_requires_static_ego_and_moving_umi():
    assert motion_signal_status("device3_anchor_hil", 2.0, {"right": 5.0}) == "PASS"
    assert motion_signal_status("device3_anchor_hil", 4.0, {"right": 5.0}) == "FAIL"
    assert motion_signal_status("device3_anchor_hil", 2.0, {"right": 2.0}) == "FAIL"


def test_motion_gate_enforces_static_and_shared_world_protocols():
    assert motion_signal_status("bench_no_motion", 1.0, {"right": 2.0}) == "PASS"
    assert motion_signal_status("bench_no_motion", 1.0, {"right": 3.1}) == "FAIL"
    assert motion_signal_status("shared_world_motion", 4.0, {"right": 5.0}) == "PASS"
    assert motion_signal_status("shared_world_motion", 2.0, {"right": 5.0}) == "FAIL"


def test_long_shared_world_mode_guides_observable_multi_axis_motion():
    assert "AprilGrid" in capture_cue(0, 105, "shared_world_motion")
    assert "两台设备" in capture_cue(0, 105, "shared_world_motion")
    assert "前后、左右平移" in capture_cue(15, 105, "shared_world_motion")
    assert "偏航" in capture_cue(35, 105, "shared_world_motion")
    assert "俯仰" in capture_cue(55, 105, "shared_world_motion")
    assert "侧倾" in capture_cue(75, 105, "shared_world_motion")
    assert "回到起点" in capture_cue(90, 105, "shared_world_motion")
    assert "两台设备都看见AprilGrid" in capture_cue(
        100, 105, "shared_world_motion"
    )
    assert "完全保持静止" in capture_cue(100, 105, "shared_world_motion")


def test_long_capture_storage_budget_covers_raw_warmup_and_headroom():
    one_umi = required_capture_free_bytes(105, 10, 1)
    two_umis = required_capture_free_bytes(105, 10, 2)

    assert one_umi > 28_000_000_000
    assert two_umis > one_umi


def test_shared_world_mode_rejects_too_short_capture(tmp_path):
    with pytest.raises(SystemExit):
        parse_args(
            [
                "--session", str(tmp_path / "session"),
                "--ego-serial", "ego-serial",
                "--ego-calibration-id", "ego-cal",
                "--umi-serial", "umi-serial",
                "--umi-port", "/dev/serial/by-id/umi",
                "--umi-calibration-id", "umi-cal",
                "--umi-protocol", "stm32_combined_v1",
                "--duration", "20",
                "--mode", "shared_world_motion",
            ]
        )


def test_group_cli_accepts_two_distinct_umis_under_one_owner(tmp_path):
    args = parse_args(
        [
            "--session", str(tmp_path / "session"),
            "--ego-serial", "ego-serial",
            "--ego-calibration-id", "ego-cal",
            "--umi-device-id", "left",
            "--umi-serial", "left-serial",
            "--umi-port", "/dev/left-imu",
            "--umi-calibration-id", "left-cal",
            "--umi-protocol", "stm32_combined_v1",
            "--right-serial", "right-serial",
            "--right-port", "/dev/right-imu",
            "--right-calibration-id", "right-cal",
            "--right-protocol", "kt_ex9_37",
            "--duration", "3",
        ]
    )

    assert umi_specs(args) == [
        {
            "device_id": "left",
            "serial": "left-serial",
            "port": "/dev/left-imu",
            "calibration_id": "left-cal",
            "protocol": "stm32_combined_v1",
        },
        {
            "device_id": "right",
            "serial": "right-serial",
            "port": "/dev/right-imu",
            "calibration_id": "right-cal",
            "protocol": "kt_ex9_37",
        },
    ]


@pytest.mark.parametrize(
    "extra",
    [
        ["--right-serial", "right-only"],
        [
            "--right-serial", "left-serial",
            "--right-port", "/dev/right-imu",
            "--right-calibration-id", "right-cal",
            "--right-protocol", "stm32_combined_v1",
        ],
        [
            "--right-serial", "right-serial",
            "--right-port", "/dev/left-imu",
            "--right-calibration-id", "right-cal",
            "--right-protocol", "stm32_combined_v1",
        ],
    ],
)
def test_group_cli_rejects_partial_or_duplicate_right_umi(tmp_path, extra):
    base = [
        "--session", str(tmp_path / "session"),
        "--ego-serial", "ego-serial",
        "--ego-calibration-id", "ego-cal",
        "--umi-device-id", "left",
        "--umi-serial", "left-serial",
        "--umi-port", "/dev/left-imu",
        "--umi-calibration-id", "left-cal",
        "--umi-protocol", "stm32_combined_v1",
        "--duration", "3",
    ]
    with pytest.raises(SystemExit):
        parse_args(base + extra)


def test_nearest_timestamp_deltas_are_deterministic():
    assert nearest_timestamp_deltas_ns(
        [100, 200, 300, 410], [90, 205, 390]
    ) == [10, 5, 90, 20]
    assert nearest_timestamp_deltas_ns([1, 2], []) == []


def test_formal_window_excludes_shutdown_tail_samples():
    samples = [
        StreamSample(index, timestamp, timestamp + 1, "global_time")
        for index, timestamp in enumerate((99, 100, 150, 199, 200), start=1)
    ]

    assert [
        sample.acquisition_ns
        for sample in formal_window_samples(samples, 100, 200)
    ] == [100, 150, 199]


def test_counter_delta_scopes_transport_errors_to_formal_window():
    assert counter_delta(
        {"resyncs": 25, "frames_bad": 0},
        {"resyncs": 25, "frames_bad": 1},
    ) == {"resyncs": 0, "frames_bad": 1}


def test_cross_timing_uses_candidates_outside_formal_window_at_boundaries():
    reference = [StreamSample(1, 100, 101, "global_time")]
    candidates = [
        StreamSample(1, 90, 91, "global_time"),
        StreamSample(2, 210, 211, "global_time"),
    ]

    report = cross_timing_report(reference, candidates)

    assert report["nearest_delta_ms_max"] == 0.00001
    assert report["status"] == "PASS"
