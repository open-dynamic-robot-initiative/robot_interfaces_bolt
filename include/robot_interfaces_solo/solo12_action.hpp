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
// FIXME: implement robot_interfaces::Loggable
struct Solo12Action
{
    Vector12d joint_torques = Vector12d::Zero();
    Vector12d joint_positions = Vector12d::Zero();
    Vector12d joint_velocities = Vector12d::Zero();
    Vector12d joint_position_gains = Vector12d::Zero();
    Vector12d joint_velocity_gains = Vector12d::Zero();
    double heart_beat;  // TODO: what is this?  currently unused

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(

            CEREAL_NVP(joint_torques),
            CEREAL_NVP(joint_positions),
            CEREAL_NVP(joint_velocities),
            CEREAL_NVP(joint_position_gains),
            CEREAL_NVP(joint_velocity_gains),
            CEREAL_NVP(heart_beat)

        );
    }

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
};
}  // namespace robot_interfaces_solo
