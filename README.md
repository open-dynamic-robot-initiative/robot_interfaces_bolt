robot_interfaces Driver for Solo Robots
=======================================

This package provides the necessary types and driver to operate a Solo12 robot
with [robot_interfaces](https://github.com/open-dynamic-robot-initiative/robot_interfaces).


Installation
------------

### Basic Usage

TODO: setup an Apptainer image similar to trifinger_user/robot


### For Development

Get the **trifinger_base** Apptainer image (it also contains everything needed
for this package):

```bash
$ apptainer pull oras://ghcr.io/open-dynamic-robot-initiative/trifinger_singularity/trifinger_base:latest
```

Install treep:
```bash
$ pip install --upgrade treep
```

Create a workspace directory, clone the treep configuration and the project:
```bash
$ mkdir ~/my_workspace
$ cd ~/my_workspace

$ git clone git@github.com:machines-in-motion/treep_machines_in_motion.git 
$ treep --clone ROBOT_INTERFACES_SOLO
```

Build the workspace using the Apptainer image:
```bash
$ cd ~/my_workspace/workspace
$ apptainer shell -e path/to/trifinger_user.sif
Apptainer> source /setup.bash  # Needed to setup the environment
Apptainer> colcon build

Apptainer> source install/setup.bash  # Needed to be able to use the built packages
```

To run an application with Apptainer:

```bash
$ cd ~/my_workspace/workspace
$ apptainer shell -e path/to/trifinger_user.sif
Apptainer> source install/setup.bash
# now you can run applications (e.g. one of the demos):
Apptainer> ros2 run robot_interfaces_solo demo_solo12_sine path/to/config.yml
```

**NOTE:** When running commands in the following, it is always assumed that this
is done in inside the container with the setup.bash of the workspace sourced.



Configuration
-------------

You need to create a small configuration file for the robot.  The file needs to
be in YAML format and can contain the following values:

- network_interface:  Name of the network interface to which the robot is
  connected (e.g. "eth0").
  This parameter is required.

- slider_serial_port: Name of the serial port to which the hardware slider is
  connected. This can typically be omitted, in which case the port is
  auto-detected.

- max_motor_current_A (default: 8 A): Maximum current that can be applied to the
  motors (in Ampere).

- home_offset_rad (default: [0, 0, ...]): Offset between home position (=encoder
  index) and zero position.  See the section home offset calibration below.

- logger_level (default: "warning"): Controls the verbosity of the output.
  Valid values are {"trace", "debug", "info", "warning", "error", "critical",
  "off"}.



Home Offset Calibration
-----------------------

Before using the robot, the home offsets need to be calibrated.  This can be
done in the following way:

1. Manually move the joints to be roughly at the desired zero position.
2. Run the following command:
   ```bash
   ros2 run solo solo12_hardware_calibration <network-interface>
   ```
   This will find the next encoder indices (this is the 'home position') and
   then start printing the offset.
3. Manually move the joints back to the desired zero position.  This time it has
   to be precise.
4. Stop `solo12_hardware_calibration` (via Ctrl+C).  Copy the last offsets that
   were printed to the terminal and insert them in the configuration file as
   value for `home_offset_rad` (see above).



Run Demo
--------

This package contains two demos (on in C++ and one in Python).  Both can be run
with this command:
```bash
ros2 run robot_interfaces_solo <demo-name> path/to/config.yml
```

- **demo_solo12_hold** [C++]: Initialises the robot and then simply holds the joints
  in place using position commands.  It is implemented in
  `demos/demo_solo12_hold.cpp`.
- **demo_solo12_sine** [Python]: Initialises the robot and then moves all joints
  back and forth using a sine profile.  It is implemented in
  `demos/demo_solo12_sine.py`.


These demos should give an idea on how to use robot_interfaces with the Solo12
driver.


Actions and Observations
------------------------

TODO: Add api doc for the action and observation types.

### Solo12Action

`Solo12Action` has the following fields.  Each is a vector with 12 elements, one
for each joint of the robot.

TODO: build and link docs of solo package for a mapping of index to joint

- `joint_torques`:  Desired torques of the joints.
- `joint_positions`:  Desired positions of the joints (`joint_position_gains`
  needs to be set as well for this to be effective).
- `joint_velocities`:  Desired velocities of the joints (`joint_velocity_gains`
  needs to be set as well for this to be effective).
- `joint_position_gains`:  P-gains of the on-board PD+ controller.
- `joint_velocity_gains`:  D-gains of the on-board PD+ controller.


### Solo12Observation

`Solo12Observation` has the following fields:
The numbers in square brackets denote the length of the vectors.

Measurements from the joints:
- `joint_positions` [12]
- `joint_velocities` [12]
- `joint_torques` [12]
- `joint_target_torques` [12]  (?)
- `joint_encoder_index` [12]  (?)

Positions of the hardware sliders:
- `slider_positions` [4]

Measurements from the IMU:
- `imu_accelerometer` [3]
- `imu_gyroscope` [3]
- `imu_linear_acceleration` [3]
- `imu_attitude` [4]

Information on connection reliability (typically only relevant when using wifi):
- `num_sent_command_packets`:  Total number of command packets sent to the
  robot.
- `num_lost_command_packets`:  Number of command packets that were sent to the
  robot but were lost in transmission.
- `num_sent_sensor_packets`:  Total number of sensor packets sent from the
  robot.
- `num_lost_sensor_packets`:  Number of sensor packets that were sent from the
  robot but were lost in transmission.



Copyright and License
---------------------

Copyright (c) 2022 Max Planck Gesellschaft.  All rights reserved.
License: BSD 3-clause (see [LICENSE](LICENSE)).
