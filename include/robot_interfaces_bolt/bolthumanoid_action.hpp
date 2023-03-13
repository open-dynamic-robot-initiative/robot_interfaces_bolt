/**
 * \file
 * \brief Action for BoltHumanoid.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#pragma once

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
 * @brief Action for the BoltHumanoid robot.
 *
 * BoltHumanoid has an on-board PD+ controller which can be used by setting the
 * P and D control gains per joint to the fields @ref joint_position_gains and
 * @ref joint_velocity_gains. If the gains are zero, @ref joint_positions and
 * @ref joint_velocities are ignored and only the torque commands in @ref
 * joint_torques are executed.
 */
struct BoltHumanoidAction : public robot_interfaces::Loggable
{
    //! Desired joint torques.
    Vector9d joint_torques = Vector9d::Zero();
    //! Desired joint positions.  Only used if @ref joint_position_gains are set
    //! to a non-zero value.
    Vector9d joint_positions = Vector9d::Zero();
    //! Desired joint velocities.  Only used if @ref joint_velocity_gains are
    //! set to a non-zero value.
    Vector9d joint_velocities = Vector9d::Zero();
    //! P-gains of the on-board PD+ controller.
    Vector9d joint_position_gains = Vector9d::Zero();
    //! D-gains of the on-board PD+ controller.
    Vector9d joint_velocity_gains = Vector9d::Zero();

    /**
     * @brief Create a zero-torque action
     *
     * This is equivalent to just using the default constructor but can be used
     * to more explicitly show intend in the code.
     */
    static BoltHumanoidAction Zero()
    {
        return BoltHumanoidAction();
    }

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(

            CEREAL_NVP(joint_torques),
            CEREAL_NVP(joint_positions),
            CEREAL_NVP(joint_velocities),
            CEREAL_NVP(joint_position_gains),
            CEREAL_NVP(joint_velocity_gains)

        );
    }

    std::vector<std::string> get_name() override
    {
        return {"joint_torques",
                "joint_positions",
                "joint_velocities",
                "joint_position_gains",
                "joint_velocity_gains"};
    }

    std::vector<std::vector<double>> get_data() override
    {
        // first map the Eigen vectors to std::vectors
        std::vector<double> torque;
        torque.resize(joint_torques.size());
        Vector9d::Map(&torque[0], joint_torques.size()) = joint_torques;

        std::vector<double> position;
        position.resize(joint_positions.size());
        Vector9d::Map(&position[0], joint_positions.size()) = joint_positions;

        std::vector<double> velocity;
        velocity.resize(joint_velocities.size());
        Vector9d::Map(&velocity[0], joint_velocities.size()) = joint_velocities;

        std::vector<double> position_gains;
        position_gains.resize(joint_position_gains.size());
        Vector9d::Map(&position_gains[0], joint_position_gains.size()) =
            joint_position_gains;

        std::vector<double> velocity_gains;
        velocity_gains.resize(joint_velocity_gains.size());
        Vector9d::Map(&velocity_gains[0], joint_velocity_gains.size()) =
            joint_velocity_gains;

        // then return them in a fixed size vector of vectors to avoid
        // copying due to pushing back value of information!
        std::vector<std::vector<double>> result;
        result = {torque, position, velocity, position_gains, velocity_gains};

        return result;
    }
};
}  // namespace robot_interfaces_bolt
