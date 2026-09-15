from three_device_slam.devices.two_uq2.rate_probe import evaluate_rates


def test_60_hz_camera_and_200_hz_imu_rate_contract_passes():
    report = evaluate_rates(
        duration_s=10.0,
        hardware_video_frames=600,
        output_video_frames=600,
        imu_packets=2000,
        imu_samples=2000,
        xu_failures=0,
        timestamp_regressions=0,
    )

    assert report["status"] == "PASS"
    assert report["rates"]["camera_output_hz"] == 60.0
    assert report["rates"]["imu_sample_hz"] == 200.0


def test_rate_contract_rejects_100_hz_imu_samples():
    report = evaluate_rates(
        duration_s=10.0,
        hardware_video_frames=600,
        output_video_frames=600,
        imu_packets=1000,
        imu_samples=1000,
        xu_failures=0,
        timestamp_regressions=0,
    )

    assert report["status"] == "FAIL"
    assert report["checks"]["imu_sample_hz"]["status"] == "FAIL"


def test_rate_contract_rejects_synthetic_second_sample_per_packet():
    report = evaluate_rates(
        duration_s=10.0,
        hardware_video_frames=600,
        output_video_frames=600,
        imu_packets=500,
        imu_samples=2000,
        xu_failures=0,
        timestamp_regressions=0,
    )

    assert report["status"] == "FAIL"
    assert report["checks"]["one_main_sample_per_packet"]["status"] == "FAIL"
