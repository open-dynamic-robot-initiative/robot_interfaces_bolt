/**
 * \file
 * \brief Driver for using simulated Solo12 in robot_interfaces::RobotBackend.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#pragma once

#include <pybind11/pybind11.h>
#include <spdlog/spdlog.h>

#include <robot_interfaces/robot_driver.hpp>

#include "solo12_types.hpp"

namespace robot_interfaces_solo
{
namespace py = pybind11;

/**
 * @brief Driver for Solo12 in PyBullet simulation.
 *
 * This driver can be used as a replacement for the "real" Solo12 driver for
 * testing things in simulation.
 *
 * @warning
 *   The fields ``imu_linear_acceleration`` and ``imu_attitude`` of the
 *   observation are not set by this driver, as the corresponding values are not
 *   provided by the simulation.
 */
class PyBulletSolo12Driver : public BaseSolo12Driver
{
private:
    std::shared_ptr<spdlog::logger> log_;

    //! @brief If true, step simulation at 1 kHz, otherwise as fast as possible
    bool real_time_mode_;

    //! @brief Number of received commands (=actions).
    int command_packet_counter_ = 0;
    //! @brief Number of provided observations.
    int sensor_packet_counter_ = 0;

    //! @brief The torque-part of the last desired action.
    Vector12d desired_torques_ = Vector12d::Zero();
    /**
     * @brief Actual torques that were applied based on the latest action.
     *
     * This also includes the resulting torque from the PD controller if the
     * corresponding parts of the action were set.
     */
    Vector12d applied_torques_ = Vector12d::Zero();

    //! Instance of ``bullet_utils.env.BulletEnvWithGround`` used to set up the
    //! simulation.
    py::object sim_env_;
    //! Instance of ``robot_properties_solo.solo12wrapper.Solo12Robot`` for
    //! controlling the simulated robot.
    py::object sim_robot_;

public:
    //! @brief Name of the spdlog logger used.
    inline static const std::string LOGGER_NAME = "PyBulletSolo12Driver";

    /**
     * @param real_time_mode  If true, sleep when stepping the simulation, so it
     *     runs in real time.
     * @param visualize  If true PyBullet's visualisation is enabled
     *     ("GUI" mode), otherwise it is run in "DIRECT" mode without
     *     visualisation.
     * @param use_fixed_base  If true, the robot's base is fixed and cannot
     *     move (i.e. the robot is hanging in the air).  Can be useful for
     *     debugging.
     * @param logger_level  Output level used by the logger.  Has to be a
     *     level supported by spdlog (e.g. "debug", "info", ...).
     */
    PyBulletSolo12Driver(bool real_time_mode=true,
                         bool visualize=true,
                         bool use_fixed_base=false,
                         const std::string &logger_level = "debug");

    void initialize() override;
    Solo12Observation get_latest_observation() override;
    Solo12Action apply_action(const Solo12Action &desired_action) override;
    std::string get_error() override;
    void shutdown() override;

    //! Get the bullet environment instance for direct access to the simulation.
    py::object get_bullet_env();
};

/**
 * @brief Create robot backend using the PyBullet Solo12 driver (for testing).
 *
 * Arguments are the same as for @ref create_real_solo12_backend.
 *
 * This function uses default values for initialising the simulation driver.  If
 * you want more control over the settings (including direct access to the
 * simulation environment), create a driver instance yourself and use @ref
 * create_solo12_backend.
 *
 * @see PyBulletSolo12Driver
 * @see create_solo12_backend
 * @see create_real_solo12_backend
 */
Solo12Backend::Ptr create_pybullet_solo12_backend(
    Solo12Data::Ptr robot_data,
    const Solo12Config &driver_config,
    const double first_action_timeout = std::numeric_limits<double>::infinity(),
    const uint32_t max_number_of_actions = 0);

}  // namespace robot_interfaces_solo
