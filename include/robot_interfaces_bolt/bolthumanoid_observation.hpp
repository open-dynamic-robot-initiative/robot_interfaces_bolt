/**
 * \file
 * \brief Observation for BoltHumanoid.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <cereal/cereal.hpp>
#include <serialization_utils/cereal_eigen.hpp>

#include <robot_interfaces/loggable.hpp>

#include "basic_types.hpp"

namespace robot_interfaces_bolt
{
/**
 * @brief Observation of the BoltHumanoid robot.
 *
 * This observation class contains all the sensor data provided by the
 * BoltHumanoid robot. The names of the attributes correspond to the names used
 * in ``bolt::BoltHumanoid`` of the bolt package (for each attribute X, there is
 * a method ``get_X()``). See there for more information.
 */
struct BoltHumanoidObservation : public robot_interfaces::Loggable
{
    //! Measured joint positions.
    Vector9d joint_positions;
    //! Measured joint velocities.
    Vector9d joint_velocities;
    //! Measured joint torques.
    Vector9d joint_torques;
    //! Target joint torques applied by the controller.
    Vector9d joint_target_torques;

    //! Positions of the hardware sliders, if connected (range: 0 to 1)
    Eigen::Vector4d slider_positions;

    //! Measurement of the IMU accelerometer.
    Eigen::Vector3d imu_accelerometer;
    //! Measurement of the IMU gyroscope.
    Eigen::Vector3d imu_gyroscope;
    //! Linear acceleration measured by the IMU.
    Eigen::Vector3d imu_linear_acceleration;
    //! Attitude measured by the IMU.
    Eigen::Vector4d imu_attitude;

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(

            CEREAL_NVP(joint_positions),
            CEREAL_NVP(joint_velocities),
            CEREAL_NVP(joint_torques),
            CEREAL_NVP(joint_target_torques),
            CEREAL_NVP(slider_positions),
            CEREAL_NVP(imu_accelerometer),
            CEREAL_NVP(imu_gyroscope),
            CEREAL_NVP(imu_linear_acceleration),
            CEREAL_NVP(imu_attitude)

        );
    }

    std::vector<std::string> get_name() override
    {
        return {"joint_positions",
                "joint_velocities",
                "joint_torques",
                "joint_target_torques",
                "slider_positions",
                "imu_accelerometer",
                "imu_gyroscope",
                "imu_linear_acceleration",
                "imu_attitude"};
    }

    std::vector<std::vector<double>> get_data() override
    {
        // first map the Eigen vectors to std::vectors

        std::vector<double> _joint_positions;
        _joint_positions.resize(joint_positions.size());
        Vector9d::Map(&_joint_positions[0], joint_positions.size()) =
            joint_positions;

        std::vector<double> _joint_velocities;
        _joint_velocities.resize(joint_velocities.size());
        Vector9d::Map(&_joint_velocities[0], joint_velocities.size()) =
            joint_velocities;

        std::vector<double> _joint_torques;
        _joint_torques.resize(joint_torques.size());
        Vector9d::Map(&_joint_torques[0], joint_torques.size()) = joint_torques;

        std::vector<double> _joint_target_torques;
        _joint_target_torques.resize(joint_target_torques.size());
        Vector9d::Map(&_joint_target_torques[0], joint_target_torques.size()) =
            joint_target_torques;

        std::vector<double> _slider_positions;
        _slider_positions.resize(slider_positions.size());
        Eigen::Vector4d::Map(&_slider_positions[0], slider_positions.size()) =
            slider_positions;

        std::vector<double> _imu_accelerometer;
        _imu_accelerometer.resize(imu_accelerometer.size());
        Eigen::Vector3d::Map(&_imu_accelerometer[0], imu_accelerometer.size()) =
            imu_accelerometer;

        std::vector<double> _imu_gyroscope;
        _imu_gyroscope.resize(imu_gyroscope.size());
        Eigen::Vector3d::Map(&_imu_gyroscope[0], imu_gyroscope.size()) =
            imu_gyroscope;

        std::vector<double> _imu_linear_acceleration;
        _imu_linear_acceleration.resize(imu_linear_acceleration.size());
        Eigen::Vector3d::Map(&_imu_linear_acceleration[0],
                             imu_linear_acceleration.size()) =
            imu_linear_acceleration;

        std::vector<double> _imu_attitude;
        _imu_attitude.resize(imu_attitude.size());
        Eigen::Vector4d::Map(&_imu_attitude[0], imu_attitude.size()) =
            imu_attitude;

        // then return them in a fixed size vector of vectors to avoid
        // copying due to pushing back value of information!
        std::vector<std::vector<double>> result;
        result = {_joint_positions,
                  _joint_velocities,
                  _joint_torques,
                  _joint_target_torques,
                  _slider_positions,
                  _imu_accelerometer,
                  _imu_gyroscope,
                  _imu_linear_acceleration,
                  _imu_attitude};

        return result;
    }
};
}  // namespace robot_interfaces_bolt
