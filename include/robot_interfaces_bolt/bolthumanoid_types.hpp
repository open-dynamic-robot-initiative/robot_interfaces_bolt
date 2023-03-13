/**
 * \file
 * \brief robot_interfaces types for BoltHumanoid.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#pragma once

#include <memory>

#include <robot_interfaces/robot_backend.hpp>
#include <robot_interfaces/robot_data.hpp>
#include <robot_interfaces/robot_frontend.hpp>

#include "bolthumanoid_action.hpp"
#include "bolthumanoid_config.hpp"
#include "bolthumanoid_observation.hpp"

namespace robot_interfaces_bolt
{
typedef robot_interfaces::RobotBackend<BoltHumanoidAction,
                                       BoltHumanoidObservation>
    BoltHumanoidBackend;
typedef robot_interfaces::RobotFrontend<BoltHumanoidAction,
                                        BoltHumanoidObservation>
    BoltHumanoidFrontend;
typedef robot_interfaces::RobotData<BoltHumanoidAction, BoltHumanoidObservation>
    BoltHumanoidData;
typedef robot_interfaces::SingleProcessRobotData<BoltHumanoidAction,
                                                 BoltHumanoidObservation>
    BoltHumanoidSingleProcessData;
typedef robot_interfaces::MultiProcessRobotData<BoltHumanoidAction,
                                                BoltHumanoidObservation>
    BoltHumanoidMultiProcessData;

//! Base class for BoltHumanoid drivers
class BaseBoltHumanoidDriver
    : public robot_interfaces::RobotDriver<BoltHumanoidAction,
                                           BoltHumanoidObservation>
{
public:
    typedef std::shared_ptr<BaseBoltHumanoidDriver> Ptr;
    typedef std::shared_ptr<const BaseBoltHumanoidDriver> ConstPtr;
};

}  // namespace robot_interfaces_bolt
