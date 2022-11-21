/**
 * \file
 * \brief Driver for using Solo12 in robot_interfaces::RobotBackend.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#pragma once

#include <filesystem>

#include <pybind11/pybind11.h>
#include <spdlog/spdlog.h>

#include <robot_interfaces/robot_driver.hpp>

#include "solo12_types.hpp"

namespace robot_interfaces_solo
{
namespace py = pybind11;

class PyBulletSolo12Driver : public BaseSolo12Driver
{
private:
    std::shared_ptr<spdlog::logger> log_;

    // FIXME real_time_mode_ and visualize_ are not used
    //! @brief If true, step simulation at 1 kHz, otherwise as fast as possible
    bool real_time_mode_;

    //! @brief If true, pyBullet GUI for visualization is started.
    bool visualize_;

    int command_packet_counter_ = 0;
    int sensor_packet_counter_ = 0;

    Vector12d applied_torques_ = Vector12d::Zero();

    // TODO: user should be able to access the env
    py::object sim_env_;
    py::object sim_robot_;

public:
    inline static const std::string LOGGER_NAME = "PyBulletSolo12Driver";

    PyBulletSolo12Driver(bool real_time_mode,
                         bool visualize,
                         const std::string &logger_level = "debug");

    void initialize() override;
    Solo12Observation get_latest_observation() override;
    Solo12Action apply_action(const Solo12Action &desired_action) override;
    std::string get_error() override;
    void shutdown() override;
};

/**
 * @brief Create robot backend using the PyBullet Solo12 driver (for testing).
 *
 * Arguments are the same as for @ref create_solo12_backend.
 *
 * @see PyBulletSolo12Driver
 * @see create_solo12_backend
 */
Solo12Backend::Ptr create_pybullet_solo12_backend(
    Solo12Data::Ptr robot_data,
    const Solo12Config &driver_config,
    const double first_action_timeout = std::numeric_limits<double>::infinity(),
    const uint32_t max_number_of_actions = 0);

}  // namespace robot_interfaces_solo
