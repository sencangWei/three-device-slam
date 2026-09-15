import copy

import numpy as np
import pytest

from three_device_slam.devices.d435i_ego.orbslam3_export import (
    orbslam3_settings,
    validate_rectified_factory_model,
)


def calibration():
    camera = {
        "width": 1280,
        "height": 720,
        "fx": 644.0672,
        "fy": 644.0672,
        "ppx": 642.2068,
        "ppy": 367.0062,
        "distortion_model": "distortion.brown_conrady",
        "coefficients": [0, 0, 0, 0, 0],
    }

    def transform(translation):
        return {
            "rotation_row_major": np.eye(3).reshape(-1).tolist(),
            "translation_m": translation,
        }

    return {
        "schema": "ego.d435i.factory_calibration.v1",
        "device": {"serial": "327122078613", "name": "Intel RealSense D435I"},
        "intrinsics": {"ir_left": camera, "ir_right": copy.deepcopy(camera)},
        "extrinsics": {
            "gyro_to_ir_left": transform([0.00552, -0.0051, -0.01174]),
            "accel_to_ir_left": transform([0.00552, -0.0051, -0.01174]),
            "gyro_to_accel": transform([0, 0, 0]),
            "ir_left_to_ir_right": transform([-0.0500474386, 0, 0]),
        },
    }


def test_factory_model_generates_bound_rectified_orb_settings():
    cal = calibration()
    model = validate_rectified_factory_model(cal)
    text = orbslam3_settings(cal)

    assert model["baseline_m"] == pytest.approx(0.0500474386)
    np.testing.assert_allclose(model["T_body_cam0"][:3, 3], [-0.00552, 0.0051, 0.01174])
    assert 'Camera.type: "Rectified"' in text
    assert "Camera.width: 1280" in text
    assert "Camera.height: 720" in text
    assert "Stereo.b: 0.0500474386" in text
    assert 'System.SaveAtlasToFile: "map_atlas"' in text
    assert "IMU.Frequency: 200.0" in text
    assert "IMU.InitTranslationAccumulationThreshold: 0.02" in text
    assert "IMU.InitTranslationResetThreshold: 0.005" in text
    assert "PROVISIONAL" not in text  # comments describe provenance without a fake PASS


def test_rejects_wrong_device_nonrectified_or_nonhorizontal_geometry():
    cal = calibration()
    cal["device"]["serial"] = "wrong"
    with pytest.raises(ValueError, match="bound"):
        validate_rectified_factory_model(cal)

    cal = calibration()
    cal["intrinsics"]["ir_left"]["coefficients"][0] = 0.1
    with pytest.raises(ValueError, match="rectified"):
        validate_rectified_factory_model(cal)

    cal = calibration()
    cal["extrinsics"]["ir_left_to_ir_right"]["translation_m"][1] = 0.001
    with pytest.raises(ValueError, match="horizontal"):
        validate_rectified_factory_model(cal)


def test_atlas_and_noise_contracts_fail_closed():
    cal = calibration()
    with pytest.raises(ValueError, match="both"):
        orbslam3_settings(cal, save_atlas="new", load_atlas="old")
    with pytest.raises(ValueError, match="omit"):
        orbslam3_settings(cal, save_atlas="map.osa")
    with pytest.raises(ValueError, match="positive"):
        orbslam3_settings(cal, noise_gyro=0)
    with pytest.raises(ValueError, match="feature"):
        orbslam3_settings(cal, features=100)
    with pytest.raises(ValueError, match="translation thresholds"):
        orbslam3_settings(
            cal,
            init_translation_accumulation_m=0.005,
            init_translation_reset_m=0.02,
        )
