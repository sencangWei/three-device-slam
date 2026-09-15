from types import SimpleNamespace

from scripts.x5_2uq2_calibration_capture import (
    build_next_action,
    build_stage_acceptance,
    print_stage_summary,
)


def acceptance(stage="preflight", *, imu_hz=200.0, video_gaps=0):
    return build_stage_acceptance(
        stage=stage,
        requested_duration_seconds=90.0,
        duration_seconds=90.0,
        native_video_hz=60.0,
        retained_video_hz=30.0,
        video_sequence_gaps=video_gaps,
        imu_hz=imu_hz,
        xu_errors=0,
        operator_aborted=False,
    )


def test_stereo_stage_warns_for_slow_imu_without_failing_capture():
    report = acceptance("stereo", imu_hz=181.3)

    assert report["status"] == "PASS"
    assert report["components"]["camera"]["status"] == "PASS"
    assert report["components"]["imu"] == {"status": "WARN", "required": False}
    assert report["checks"]["imu_hz"]["status"] == "WARN"


def test_cam_imu_stage_requires_imu_rate():
    report = acceptance("cam-imu", imu_hz=181.3)

    assert report["status"] == "FAIL"
    assert report["components"]["imu"] == {"status": "FAIL", "required": True}


def test_every_stage_requires_gap_free_video():
    report = acceptance("stereo", video_gaps=1)

    assert report["status"] == "FAIL"
    assert report["checks"]["video_sequence_gaps"]["status"] == "FAIL"


def test_preflight_pass_prints_next_stereo_capture_command(capsys):
    args = SimpleNamespace(
        output="/tmp/preflight",
        stage="preflight",
        x5_address="192.168.113.32",
        pc_artifact_root="/home/robot/three-device-slam/artifacts/ego_calibration",
    )
    stage = acceptance("preflight")
    report = {
        "stage_acceptance": stage,
        "next_action": build_next_action(args, stage),
    }

    print_stage_summary(report)

    output = capsys.readouterr().out
    assert "本步结果: PASS" in output
    assert "--stage stereo" in output
    assert "--duration-seconds 90" in output


def test_stereo_pass_prints_pc_copy_command():
    args = SimpleNamespace(
        output="/tmp/ego-x5-stereo-calib-run1",
        stage="stereo",
        x5_address="192.168.113.32",
        pc_artifact_root="/home/robot/three-device-slam/artifacts/ego_calibration",
    )

    action = build_next_action(args, acceptance("stereo", imu_hz=181.3))

    assert action["state"] == "PENDING_OFFLINE_APRILGRID"
    assert action["command"] == (
        "scp -r pi@192.168.113.32:/tmp/ego-x5-stereo-calib-run1 "
        "/home/robot/three-device-slam/artifacts/ego_calibration/"
        "ego-x5-stereo-calib-run1"
    )
