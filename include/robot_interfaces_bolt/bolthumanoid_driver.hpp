/**
 * \file
 * \brief Driver for using BoltHumanoid in robot_interfaces::RobotBackend.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#pragma once

#include <filesystem>
#include <limits>
#include <memory>
#include <string>

#include <spdlog/spdlog.h>

#include <bolt/bolt_humanoid.hpp>
#include <robot_interfaces/robot_frontend.hpp>

#include "bolthumanoid_types.hpp"

namespace robot_interfaces_bolt
{
//! Driver to use BoltHumanoid
class BoltHumanoidDriver : public BaseBoltHumanoidDriver
{
public:
    inline static const std::string LOGGER_NAME = "BoltHumanoidDriver";

    explicit BoltHumanoidDriver(const BoltHumanoidConfig &config);

    // RobotDriver methods
    void initialize() override;
    Action apply_action(const Action &desired_action) override;
    Observation get_latest_observation() override;
    std::string get_error() override;
    void shutdown() override;

private:
    std::shared_ptr<spdlog::logger> log_;
    const BoltHumanoidConfig config_;
    bolt::BoltHumanoid bolthumanoid_;
    Action applied_action_;
    bool is_initialized_ = false;
};

//! Fake driver for testing (ignores actions and returns some artificial
//! observations).
class FakeBoltHumanoidDriver : public BaseBoltHumanoidDriver
{
public:
    inline static const std::string LOGGER_NAME = "FakeBoltHumanoidDriver";

    explicit FakeBoltHumanoidDriver(const BoltHumanoidConfig &config);

    // RobotDriver methods
    void initialize() override;
    Action apply_action(const Action &desired_action) override;
    Observation get_latest_observation() override;
    std::string get_error() override;
    void shutdown() override;

private:
    std::shared_ptr<spdlog::logger> log_;
    const BoltHumanoidConfig config_;
    Action applied_action_;
    bool is_initialized_ = false;
    double t_ = 0.0;
};

/**
 * @brief Create robot backend using the BoltHumanoid driver (real robot).
 *
 * @see BoltHumanoidDriver
 *
 * @param robot_data  Instance of RobotData used for communication.
 * @param driver_config  Driver configuration.
 * @param first_action_timeout  Duration for which the backend waits for the
 *     first action to arrive.  If exceeded, the backend shuts down.
 * @param max_number_of_actions  Number of actions after which the backend
 *     automatically shuts down.
 *
 * @return A RobotBackend instances which uses a BoltHumanoid driver.
 */
BoltHumanoidBackend::Ptr create_real_bolthumanoid_backend(
    BoltHumanoidData::Ptr robot_data,
    const BoltHumanoidConfig &driver_config,
    const double first_action_timeout = std::numeric_limits<double>::infinity(),
    const uint32_t max_number_of_actions = 0);

/**
 * @brief Create robot backend using the fake BoltHumanoid driver (for testing).
 *
 * Arguments are the same as for @ref create_real_bolthumanoid_backend.
 *
 * @see FakeBoltHumanoidDriver
 * @see create_real_bolthumanoid_backend
 */
BoltHumanoidBackend::Ptr create_fake_bolthumanoid_backend(
    BoltHumanoidData::Ptr robot_data,
    const BoltHumanoidConfig &driver_config,
    const double first_action_timeout = std::numeric_limits<double>::infinity(),
    const uint32_t max_number_of_actions = 0);

}  // namespace robot_interfaces_bolt
