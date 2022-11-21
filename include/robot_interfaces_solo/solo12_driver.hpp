/**
 * \file
 * \brief Driver for using Solo12 in robot_interfaces::RobotBackend.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#pragma once

#include <filesystem>
#include <limits>
#include <memory>
#include <string>

#include <spdlog/spdlog.h>

#include <robot_interfaces/monitored_robot_driver.hpp>
#include <robot_interfaces/robot_frontend.hpp>
#include <solo/solo12.hpp>

#include "solo12_types.hpp"

namespace robot_interfaces_solo
{
//! Driver to use Solo12
class Solo12Driver : public BaseSolo12Driver
{
public:
    inline static const std::string LOGGER_NAME = "Solo12Driver";

    explicit Solo12Driver(const Solo12Config &config);

    // RobotDriver methods
    void initialize() override;
    Action apply_action(const Action &desired_action) override;
    Observation get_latest_observation() override;
    std::string get_error() override;
    void shutdown() override;

private:
    std::shared_ptr<spdlog::logger> log_;
    const Solo12Config config_;
    solo::Solo12 solo12_;
    Action applied_action_;
    bool is_initialized_ = false;
};

//! Fake driver for testing (ignores actions and returns some artificial
//! observations).
class FakeSolo12Driver : public BaseSolo12Driver
{
public:
    inline static const std::string LOGGER_NAME = "FakeSolo12Driver";

    explicit FakeSolo12Driver(const Solo12Config &config);

    // RobotDriver methods
    void initialize() override;
    Action apply_action(const Action &desired_action) override;
    Observation get_latest_observation() override;
    std::string get_error() override;
    void shutdown() override;

private:
    std::shared_ptr<spdlog::logger> log_;
    const Solo12Config config_;
    Action applied_action_;
    bool is_initialized_ = false;
    double t_ = 0.0;
};

/**
 * @brief Create robot backend using the Solo12 driver.
 *
 * @see Solo12Driver
 *
 * @param robot_data  Instance of RobotData used for communication.
 * @param driver_config  Driver configuration.
 * @param first_action_timeout  Duration for which the backend waits for the
 *     first action to arrive.  If exceeded, the backend shuts down.
 * @param max_number_of_actions  Number of actions after which the backend
 *     automatically shuts down.
 *
 * @return A RobotBackend instances which uses a Solo12 driver.
 */
Solo12Backend::Ptr create_solo12_backend(
    Solo12Data::Ptr robot_data,
    const Solo12Config &driver_config,
    const double first_action_timeout = std::numeric_limits<double>::infinity(),
    const uint32_t max_number_of_actions = 0);


/**
 * @brief Create robot backend using the fake Solo12 driver (for testing).
 *
 * Arguments are the same as for @ref create_solo12_backend.
 *
 * @see FakeSolo12Driver
 * @see create_solo12_backend
 */
Solo12Backend::Ptr create_fake_solo12_backend(
    Solo12Data::Ptr robot_data,
    const Solo12Config &driver_config,
    const double first_action_timeout = std::numeric_limits<double>::infinity(),
    const uint32_t max_number_of_actions = 0);

}  // namespace robot_interfaces_solo
