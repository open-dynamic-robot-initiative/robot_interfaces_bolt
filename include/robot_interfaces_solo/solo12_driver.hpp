/**
 * \file
 * \brief Driver for using Solo12 in robot_interfaces::RobotBackend.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#pragma once

#include <robot_interfaces/monitored_robot_driver.hpp>
#include <robot_interfaces/robot_backend.hpp>
#include <robot_interfaces/robot_data.hpp>
#include <robot_interfaces/robot_driver.hpp>
#include <solo/solo12.hpp>

#include "solo12_action.hpp"
#include "solo12_observation.hpp"

namespace robot_interfaces_solo
{
typedef robot_interfaces::RobotBackend<Solo12Action, Solo12Observation>
    Solo12Backend;
typedef robot_interfaces::RobotData<Solo12Action, Solo12Observation> Solo12Data;

struct Solo12Config
{
    std::string network_interface = "";
    std::string serial_port = "";
};

class Solo12Driver
    : public robot_interfaces::RobotDriver<Solo12Action, Solo12Observation>
{
public:
    Solo12Driver(const Solo12Config &config) : config_(config)
    {
    }

    // RobotDriver methods
    void initialize() override;
    Action apply_action(const Action &desired_action) override;
    Observation get_latest_observation() override;
    std::string get_error() override;
    void shutdown() override;

private:
    const Solo12Config config_;
    solo::Solo12 solo12_;
    Action applied_action_;
    bool is_initialized_ = false;
};

Solo12Backend::Ptr create_solo12_backend(
    Solo12Data::Ptr robot_data,
    const Solo12Config &driver_config,
    const double first_action_timeout = std::numeric_limits<double>::infinity(),
    const uint32_t max_number_of_actions = 0);
}  // namespace robot_interfaces_solo
