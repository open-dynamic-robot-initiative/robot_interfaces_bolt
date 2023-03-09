import textwrap

import pytest
import numpy as np

from robot_interfaces_bolt import bolthumanoid


def test_load_config_file_not_found():
    with pytest.raises(RuntimeError):
        bolthumanoid.Config.from_file("a_file_that_does_not_exist.yml")


def test_load_config_empty_file(tmp_path):
    yml_file = tmp_path / "foo.yml"
    yml_file.touch()

    default_cfg = bolthumanoid.Config()
    cfg = bolthumanoid.Config.from_file(yml_file)

    assert cfg.network_interface == default_cfg.network_interface
    assert cfg.slider_serial_port == default_cfg.slider_serial_port
    assert cfg.max_motor_current_A == default_cfg.max_motor_current_A
    np.testing.assert_array_equal(cfg.home_offset_rad, default_cfg.home_offset_rad)


def test_load_config_full(tmp_path):
    yml_file = tmp_path / "foo.yml"
    yml_file.write_text(
        textwrap.dedent(
            """
            network_interface: eth0
            slider_serial_port: foo
            max_motor_current_A: 5.3
            home_offset_rad: [1, 2, 3, 4, 5, 6, 7, 8, 9]
            """
        )
    )

    cfg = bolthumanoid.Config.from_file(yml_file)
    assert cfg.network_interface == "eth0"
    assert cfg.slider_serial_port == "foo"
    assert cfg.max_motor_current_A == 5.3
    np.testing.assert_array_equal(
        cfg.home_offset_rad,
        np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]),
    )


def test_load_config_partial(tmp_path):
    yml_file = tmp_path / "foo.yml"
    yml_file.write_text(
        textwrap.dedent(
            """
            network_interface: eth0
            home_offset_rad: [1, 2, 3, 4, 5, 6, 7, 8, 9]
            """
        )
    )

    default_cfg = bolthumanoid.Config()
    cfg = bolthumanoid.Config.from_file(yml_file)
    assert cfg.network_interface == "eth0"
    assert cfg.slider_serial_port == default_cfg.slider_serial_port
    assert cfg.max_motor_current_A == default_cfg.max_motor_current_A
    np.testing.assert_array_equal(
        cfg.home_offset_rad,
        np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]),
    )


def test_load_config_strpath(tmp_path):
    yml_file = tmp_path / "foo.yml"
    yml_file.write_text(
        textwrap.dedent(
            """
            network_interface: eth0
            """
        )
    )

    # verify that using string instead of Path for the file name works as well
    cfg = bolthumanoid.Config.from_file(str(yml_file))
    assert cfg.network_interface == "eth0"


def test_load_config_invalid_type(tmp_path):
    yml_file = tmp_path / "foo.yml"
    yml_file.write_text(
        textwrap.dedent(
            """
            max_motor_current_A: "this is a string"
            """
        )
    )

    with pytest.raises(RuntimeError):
        bolthumanoid.Config.from_file(yml_file)
