# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
### Added
- PyBullet driver for BoltHumanoid for testing in simulation.
- New demo `demo_solo12_simulation` illustration how to use the PyBullet driver.
- New option `--sim` for `solo12_show_data` to use the simulation backend. 

### Changed
- Make `create_solo12_backend()` more generic by accepting a driver as argument (so it
  can be used for all driver types).
  The old version is still available but renamed to `create_real_solo12_backend()`
  (`create_real_backend()` in the Python bindings).

## [0.9.0] - 2022-11-11

First beta release.  Contains the `BoltHumanoidDriver` with Python bindings, two
simple demo applications and a test application which shows sensor data.



[Unreleased]: https://github.com/open-dynamic-robot-initiative/robot_interfaces_solo/compare/v0.9.0...HEAD
[0.9.0]: https://github.com/open-dynamic-robot-initiative/robot_interfaces_solo/releases/tag/v0.9.0
