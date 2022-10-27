/**
 * \file
 * \brief Action for Solo12.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#pragma once

#include <Eigen/Eigen>
#include <cereal/cereal.hpp>
#include <serialization_utils/cereal_eigen.hpp>

#include <robot_interfaces/loggable.hpp>

#include "types.hpp"

namespace robot_interfaces_solo
{
struct Solo12Action : public robot_interfaces::Loggable
{
    Vector12d joint_torques = Vector12d::Zero();
    Vector12d joint_positions = Vector12d::Zero();
    Vector12d joint_velocities = Vector12d::Zero();
    Vector12d joint_position_gains = Vector12d::Zero();
    Vector12d joint_velocity_gains = Vector12d::Zero();

    /**
     * @brief Create a zero-torque action
     *
     * This is equivalent to just using the default constructor but can be used
     * to more explicitly show intend in the code.
     */
    static Solo12Action Zero()
    {
        return Solo12Action();
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
        Vector12d::Map(&torque[0], joint_torques.size()) = joint_torques;

        std::vector<double> position;
        position.resize(joint_positions.size());
        Vector12d::Map(&position[0], joint_positions.size()) = joint_positions;

        std::vector<double> velocity;
        velocity.resize(joint_velocities.size());
        Vector12d::Map(&velocity[0], joint_velocities.size()) =
            joint_velocities;

        std::vector<double> position_gains;
        position_gains.resize(joint_position_gains.size());
        Vector12d::Map(&position_gains[0], joint_position_gains.size()) =
            joint_position_gains;

        std::vector<double> velocity_gains;
        velocity_gains.resize(joint_velocity_gains.size());
        Vector12d::Map(&velocity_gains[0], joint_velocity_gains.size()) =
            joint_velocity_gains;

        // then return them in a fixed size vector of vectors to avoid
        // copying due to pushing back value of information!
        std::vector<std::vector<double>> result;
        result = {torque, position, velocity, position_gains, velocity_gains};

        return result;
    }
};
}  // namespace robot_interfaces_solo
