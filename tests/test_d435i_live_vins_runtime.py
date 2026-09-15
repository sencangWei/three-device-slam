from pathlib import Path

from three_device_slam.devices.d435i_ego.live_vins import prepare_ego_live_config


def test_prepare_ego_live_config_binds_topics_and_output(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    (source / "left.yaml").write_text("left\n")
    (source / "right.yaml").write_text("right\n")
    (source / "vins_config.yaml").write_text(
        '%YAML:1.0\nimu_topic: "/ego_probe/imu0"\n'
        'image0_topic: "/ego_probe/cam0/image_raw"\n'
        'image1_topic: "/ego_probe/cam1/image_raw"\n'
        'output_path: "/old/output"\ntd: 0.\n'
    )
    output = tmp_path / "runtime"

    config = prepare_ego_live_config(source, output)
    text = config.read_text()

    assert 'imu_topic: "/ego_live/imu0"' in text
    assert 'image0_topic: "/ego_live/cam0/image_raw"' in text
    assert 'image1_topic: "/ego_live/cam1/image_raw"' in text
    assert f'output_path: "{output / "solver_output"}"' in text
    assert (output / "left.yaml").read_text() == "left\n"
    assert (output / "right.yaml").read_text() == "right\n"
