/**
 * \file
 * \brief Observation for Solo12.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#pragma once

#include <Eigen/Eigen>
#include <cereal/cereal.hpp>
#include <serialization_utils/cereal_eigen.hpp>

#include <robot_interfaces/loggable.hpp>

namespace robot_interfaces_solo
{
// FIXME: implement robot_interfaces::Loggable
struct Solo12Observation
{
    typedef Eigen::Matrix<double, 12, 1> Vector12d;

    // joints data
    Vector12d joint_positions;
    Vector12d joint_velocities;
    Vector12d joint_torques;
    Vector12d joint_target_torques;
    Vector12d joint_encoder_index;

    // additional data
    Eigen::Vector4d slider_positions;
    Eigen::Vector3d imu_accelerometer;
    Eigen::Vector3d imu_gyroscope;
    Eigen::Vector3d imu_linear_acceleration;
    Eigen::Vector4d imu_attitude;

    // robot status
    double num_sent_command_packets;
    double num_lost_command_packets;
    double num_sent_sensor_packets;
    double num_lost_sensor_packets;

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(
            CEREAL_NVP(joint_positions),
            CEREAL_NVP(joint_velocities),
            CEREAL_NVP(joint_torques),
            CEREAL_NVP(joint_target_torques),
            CEREAL_NVP(joint_encoder_index),
            CEREAL_NVP(slider_positions),
            CEREAL_NVP(imu_accelerometer),
            CEREAL_NVP(imu_gyroscope),
            CEREAL_NVP(imu_linear_acceleration),
            CEREAL_NVP(imu_attitude),
            CEREAL_NVP(num_sent_command_packets),
            CEREAL_NVP(num_lost_command_packets),
            CEREAL_NVP(num_sent_sensor_packets),
            CEREAL_NVP(num_lost_sensor_packets)
        );
    }
};
}  // namespace robot_interfaces_solo

