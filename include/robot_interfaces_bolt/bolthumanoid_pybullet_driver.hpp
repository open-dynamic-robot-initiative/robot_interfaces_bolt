/**
 * \file
 * \brief Driver for using simulated BoltHumanoid in robot_interfaces::RobotBackend.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#pragma once

#include <pybind11/pybind11.h>
#include <spdlog/spdlog.h>

#include <robot_interfaces/robot_driver.hpp>

#include "bolthumanoid_types.hpp"

namespace robot_interfaces_bolt
{
namespace py = pybind11;

/**
 * @brief Driver for BoltHumanoid in PyBullet simulation.
 *
 * This driver can be used as a replacement for the "real" BoltHumanoid driver for
 * testing things in simulation.
 *
 * @warning
 *   The fields ``imu_linear_acceleration`` and ``imu_attitude`` of the
 *   observation are not set by this driver, as the corresponding values are not
 *   provided by the simulation.
 */
class PyBulletBoltHumanoidDriver : public BaseBoltHumanoidDriver
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
    Vector9d desired_torques_ = Vector9d::Zero();
    /**
     * @brief Actual torques that were applied based on the latest action.
     *
     * This also includes the resulting torque from the PD controller if the
     * corresponding parts of the action were set.
     */
    Vector9d applied_torques_ = Vector9d::Zero();

    //! Instance of ``bullet_utils.env.BulletEnvWithGround`` used to set up the
    //! simulation.
    py::object sim_env_;
    //! Instance of ``robot_properties_bolt.bolthumanoidwrapper.BoltHumanoidRobot`` for
    //! controlling the simulated robot.
    py::object sim_robot_;

    //! Get tuple (joint_positions, joint_velocities).
    std::tuple<Vector9d, Vector9d> get_position_and_velocity();

public:
    //! @brief Name of the spdlog logger used.
    inline static const std::string LOGGER_NAME = "PyBulletBoltHumanoidDriver";

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
     *     level supported by spdlog (e.g. "debug", "info", ...).  This is only
     *     considered if a new logger is initialised, i.e. the level is not
     *     changed, if a logger with the name stored in @ref
     *     PyBulletBoltHumanoidDriver::LOGGER_NAME does already exist.
     */
    PyBulletBoltHumanoidDriver(bool real_time_mode = true,
                         bool visualize = true,
                         bool use_fixed_base = false,
                         const std::string &logger_level = "debug");

    void initialize() override;
    BoltHumanoidObservation get_latest_observation() override;
    BoltHumanoidAction apply_action(const BoltHumanoidAction &desired_action) override;
    std::string get_error() override;
    void shutdown() override;

    //! Get the bullet environment instance for direct access to the simulation.
    py::object get_bullet_env();
};

/**
 * @brief Create robot backend using the PyBullet BoltHumanoid driver (for testing).
 *
 * Arguments are the same as for @ref create_real_bolthumanoid_backend.
 *
 * This function uses default values for initialising the simulation driver.  If
 * you want more control over the settings (including direct access to the
 * simulation environment), create a driver instance yourself and use @ref
 * create_bolthumanoid_backend.
 *
 * @see PyBulletBoltHumanoidDriver
 * @see create_bolthumanoid_backend
 * @see create_real_bolthumanoid_backend
 */
BoltHumanoidBackend::Ptr create_pybullet_bolthumanoid_backend(
    BoltHumanoidData::Ptr robot_data,
    const BoltHumanoidConfig &driver_config,
    const double first_action_timeout = std::numeric_limits<double>::infinity(),
    const uint32_t max_number_of_actions = 0);

}  // namespace robot_interfaces_bolt
