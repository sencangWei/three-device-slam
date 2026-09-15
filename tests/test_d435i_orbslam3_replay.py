from scripts.run_d435i_orbslam3_offline import classify_replay


def test_static_ingest_is_diagnostic_not_acceptance() -> None:
    status, limitations = classify_replay(
        returncode=-6,
        source_mode="bench_no_motion",
        stdout="Shutdown\nThere are 0 maps in the atlas\n",
        atlas_exists=True,
        trajectory_exists=False,
    )
    assert status == "DIAGNOSTIC_STATIC_INGEST_COMPLETE_NO_MAP"
    assert any("not pose or map quality" in item for item in limitations)


def test_dynamic_success_remains_unassessed() -> None:
    status, _ = classify_replay(
        returncode=0,
        source_mode="dynamic",
        stdout="Shutdown\n",
        atlas_exists=True,
        trajectory_exists=True,
        frame_trajectory_rows=990,
        frame_trajectory_duration_s=32.9,
        input_stereo_pairs=1000,
        input_duration_s=33.3,
    )
    assert status == "REPLAY_COMPLETE_UNASSESSED"


def test_short_final_trajectory_after_bad_imu_resets_fails() -> None:
    status, limitations = classify_replay(
        returncode=0,
        source_mode="dynamic",
        stdout="Shutdown\nTRACK: Reset map because local mapper set the bad imu flag\n",
        atlas_exists=True,
        trajectory_exists=True,
        frame_trajectory_rows=38,
        frame_trajectory_duration_s=1.23,
        input_stereo_pairs=2550,
        input_duration_s=85.0,
    )
    assert status == "REPLAY_FAILED_INERTIAL_INITIALIZATION"
    assert any("final short trajectory" in item for item in limitations)


def test_expected_initialization_prefix_can_be_omitted() -> None:
    status, _ = classify_replay(
        returncode=0,
        source_mode="dynamic",
        stdout="Shutdown\n",
        atlas_exists=True,
        trajectory_exists=True,
        frame_trajectory_rows=2801,
        frame_trajectory_duration_s=93.3,
        input_stereo_pairs=3150,
        input_duration_s=104.97,
    )
    assert status == "REPLAY_COMPLETE_UNASSESSED"


def test_missing_shutdown_fails() -> None:
    status, _ = classify_replay(
        returncode=0,
        source_mode="dynamic",
        stdout="",
        atlas_exists=False,
        trajectory_exists=False,
    )
    assert status == "REPLAY_FAILED"
