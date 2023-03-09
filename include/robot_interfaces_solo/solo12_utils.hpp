/**
 * \file
 * \brief Utility functions for BoltHumanoid.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#pragma once

#include <limits>

#include "solo12_types.hpp"

namespace robot_interfaces_bolt
{
/**
 * @brief Create robot backend using a BoltHumanoid driver.
 *
 * @param robot_data  Instance of RobotData used for communication.
 * @param robot_driver  Driver instance, connecting to the robot or simulation.
 * @param first_action_timeout  Duration for which the backend waits for the
 *     first action to arrive.  If exceeded, the backend shuts down.
 * @param max_number_of_actions  Number of actions after which the backend
 *     automatically shuts down.
 * @param enable_timing_watchdog  Whether to enable the backend timing watchdog
 *     (triggers an error if real-time constraints of the backend are violated).
 *     This should enabled if using the real robot but may be disabled when
 *     using simulation.
 *
 * @return A RobotBackend instances which uses a BoltHumanoid driver.
 */
BoltHumanoidBackend::Ptr create_solo12_backend(
    BoltHumanoidData::Ptr robot_data,
    BaseBoltHumanoidDriver::Ptr robot_driver,
    const double first_action_timeout = std::numeric_limits<double>::infinity(),
    const uint32_t max_number_of_actions = 0,
    bool enable_timing_watchdog = true);
}  // namespace robot_interfaces_bolt
