/**
 * \file
 * \brief Driver for using Solo12 in robot_interfaces::RobotBackend.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#pragma once

#include <filesystem>

#include <robot_interfaces/monitored_robot_driver.hpp>
#include <robot_interfaces/robot_backend.hpp>
#include <robot_interfaces/robot_data.hpp>
#include <robot_interfaces/robot_driver.hpp>
#include <robot_interfaces/robot_frontend.hpp>
#include <solo/solo12.hpp>

#include "solo12_action.hpp"
#include "solo12_observation.hpp"
#include "types.hpp"

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

/**
 * @brief Configuration for the Solo12 driver.
 */
struct Solo12Config
{
    /**
     * @brief Name of the network interface to which the robot is connected
     * (e.g. "eth0").
     */
    std::string network_interface = "";

    /**
     * @brief Name of the serial port to which the hardware slider is connected.
     *
     * This can typically be left empty, in which case the port is
     * auto-detected.
     */
    std::string slider_serial_port = "";

    /**
     * @brief Maximum current that can be applied to the motors (in Ampere).
     */
    double max_motor_current_A = 8.0;

    /**
     * @brief Offset between home position (=encoder index) and zero position.
     *
     * Angles (in radian) between the encoder index and the zero position of
     * each joint.
     */
    Vector12d home_offset_rad = Vector12d::Zero();

    /**
     * @brief Load configuration from a YAML file.
     *
     * @param file Path to the file
     *
     * @return Configuration instance.  For parameters not provided in the file
     *         the default values are kept.
     */
    static Solo12Config from_file(const std::filesystem::path &file);
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
