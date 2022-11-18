/**
 * \file
 * \brief robot_interfaces types for Solo12.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#pragma once

#include <robot_interfaces/robot_backend.hpp>
#include <robot_interfaces/robot_data.hpp>
#include <robot_interfaces/robot_frontend.hpp>

#include "solo12_action.hpp"
#include "solo12_config.hpp"
#include "solo12_observation.hpp"

namespace robot_interfaces_solo
{
typedef robot_interfaces::RobotBackend<Solo12Action, Solo12Observation>
    Solo12Backend;
typedef robot_interfaces::RobotFrontend<Solo12Action, Solo12Observation>
    Solo12Frontend;
typedef robot_interfaces::RobotData<Solo12Action, Solo12Observation> Solo12Data;
typedef robot_interfaces::SingleProcessRobotData<Solo12Action,
                                                 Solo12Observation>
    Solo12SingleProcessData;
typedef robot_interfaces::MultiProcessRobotData<Solo12Action, Solo12Observation>
    Solo12MultiProcessData;

}  // namespace robot_interfaces_solo
