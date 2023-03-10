#!/usr/bin/env python3
"""Basic demo on how to control BoltHumanoid.

This demo uses the on-board PD controller to move the joints with a sine-wave position
profile.
"""
import argparse

import numpy as np

from robot_interfaces_bolt import bolthumanoid


N_JOINTS = 9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "config_file",
        type=str,
        help="YAML file with BoltHumanoid driver configuration.",
    )
    args = parser.parse_args()

    kp = 3.0
    kd = 0.05
    freq = 0.5
    amplitude = np.pi / 4

    # load robot configuration
    config = bolthumanoid.Config.from_file(args.config_file)

    # Storage for all observations, actions, etc.
    robot_data = bolthumanoid.SingleProcessData()

    # The backend takes care of communication with the robot hardware.
    robot_backend = bolthumanoid.create_real_backend(robot_data, config)

    # The frontend is used by the user to get observations and send actions
    robot_frontend = bolthumanoid.Frontend(robot_data)

    # Initializes the robot (e.g. performs homing).
    robot_backend.initialize()

    # start by sending a zero-torque action (this is needed to start the backend loop)
    action = bolthumanoid.Action.Zero()
    t = robot_frontend.append_desired_action(action)

    # get the initial joint positions
    observation = robot_frontend.get_observation(t)
    initial_positions = observation.joint_positions

    while True:
        # compute target positions and velocities for the joints
        t_ms = t / 1000.0
        target_positions = initial_positions + amplitude * np.sin(
            2 * np.pi * freq * t_ms
        )
        target_velocities = [
            2.0 * np.pi * freq * amplitude * np.cos(2 * np.pi * freq * t_ms)
        ] * N_JOINTS

        action = bolthumanoid.Action()
        action.joint_torques = np.array([0.0] * N_JOINTS)
        action.joint_positions = np.array(target_positions)
        action.joint_velocities = np.array(target_velocities)
        action.joint_position_gains = np.array([kp] * N_JOINTS)
        action.joint_velocity_gains = np.array([kd] * N_JOINTS)

        t = robot_frontend.append_desired_action(action)
        robot_frontend.wait_until_timeindex(t)

        # Alternatively to wait_until_timeindex(t) the following could be used, in case
        # the observation is needed in the control loop (both methods will wait until
        # time step t is reached and thus ensure correct timing of the loop):
        #
        # observation = robot_frontend.get_observation(t)


if __name__ == "__main__":
    main()
