#!/usr/bin/env python3
"""Basic demo on how to control BoltHumanoid in simulation."""
from __future__ import annotations

from dataclasses import dataclass
import typing

import numpy as np

from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.bolthumanoidwrapper import BoltHumanoidRobot

from robot_interfaces_bolt import bolthumanoid


@dataclass
class BaseState:
    position: np.ndarray
    orientation: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray

    @classmethod
    def from_state(
        cls, sim_robot_state: typing.Tuple[np.ndarray, np.ndarray]
    ) -> BaseState:
        q, dq = sim_robot_state
        return cls(
            position=q[0:3],
            orientation=q[3:7],
            linear_velocity=dq[0:3],
            angular_velocity=dq[3:6],
        )


def main() -> None:
    kp = 3.0
    kd = 0.05
    freq = 0.5
    amplitude = 0.2

    # Storage for all observations, actions, etc.
    robot_data = bolthumanoid.SingleProcessData()

    # Create a backend with a PyBullet driver.  Here we need to instantiate the driver
    # on our own, so we have a handle to access the simulation.
    sim_driver = bolthumanoid.PyBulletDriver(real_time_mode=True, visualize=True)
    sim_env: BulletEnvWithGround = sim_driver.get_bullet_env()
    robot_backend = bolthumanoid.create_backend(
        robot_data, sim_driver, enable_timing_watchdog=False
    )
    robot_backend.initialize()

    # The front end is used by the user to get observations and send actions
    robot_frontend = bolthumanoid.Frontend(robot_data)

    # start by sending a zero-torque action (this is needed to start the backend loop)
    action = bolthumanoid.Action.Zero()
    t = robot_frontend.append_desired_action(action)

    # get the initial joint positions
    observation = robot_frontend.get_observation(t)
    initial_positions = observation.joint_positions

    sim_robot: BoltHumanoidRobot = sim_env.robots[0]

    while True:
        # compute target positions and velocities for the joints
        t_ms = t / 1000.0
        target_positions = initial_positions + amplitude * np.sin(
            2 * np.pi * freq * t_ms
        )
        target_velocities = [
            2.0 * np.pi * freq * amplitude * np.cos(2 * np.pi * freq * t_ms)
        ] * 12

        action = bolthumanoid.Action()
        action.joint_torques = np.array([0.0] * 12)
        action.joint_positions = np.array(target_positions)
        action.joint_velocities = np.array(target_velocities)
        action.joint_position_gains = np.array([kp] * 12)
        action.joint_velocity_gains = np.array([kd] * 12)

        t = robot_frontend.append_desired_action(action)
        observation = robot_frontend.get_observation(t)

        # The base state is not part of the robot observation and has to be acquired
        # directly from the simulation (for the real robot, this would be done, for
        # example, with a Vicon system).
        base_state = BaseState.from_state(sim_robot.get_state())

        if t % 1000 == 0:
            print(f"Base position: {base_state.position}")
            print(f"Joint positions: {observation.joint_positions}")


if __name__ == "__main__":
    main()
