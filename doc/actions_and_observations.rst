************************
Actions and Observations
************************

BoltHumanoidAction
============

"Actions" are used to send commands to the robot.  The members are vectors with
once field per joint (see :cpp:class:`solo::BoltHumanoid` for a mapping of index to
joint).

.. doxygenstruct:: robot_interfaces_bolt::BoltHumanoidAction
   :members: joint_torques,
             joint_positions,
             joint_velocities,
             joint_position_gains,
             joint_velocity_gains,
             Zero


BoltHumanoidObservation
=================

"Observations" contain the sensor data that is provided by the robot.  The
``joint_*`` members follow the same order as in the action (see above).


.. doxygenstruct:: robot_interfaces_bolt::BoltHumanoidObservation
   :members: joint_positions,
             joint_velocities,
             joint_torques,
             joint_target_torques,
             joint_encoder_index,
             slider_positions,
             imu_accelerometer,
             imu_gyroscope,
             imu_linear_acceleration,
             imu_attitude,
             num_sent_command_packets,
             num_lost_command_packets,
             num_sent_sensor_packets,
             num_lost_sensor_packets
