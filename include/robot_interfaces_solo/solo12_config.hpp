/**
 * \file
 * \brief Config for Solo12 drivers.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#pragma once

#include <filesystem>

#include "basic_types.hpp"

namespace robot_interfaces_solo
{
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
     * @brief Logger output level.
     *
     * One of {"trace", "debug", "info", "warning", "error", "critical", "off"}.
     * See documentation of spdlog for details.
     */
    std::string logger_level = "warning";

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

}  // namespace robot_interfaces_solo
