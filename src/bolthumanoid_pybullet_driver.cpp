#include <robot_interfaces_bolt/bolthumanoid_pybullet_driver.hpp>

#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/stl_bind.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <robot_interfaces_bolt/bolthumanoid_utils.hpp>

using namespace pybind11::literals;

namespace robot_interfaces_bolt
{
PyBulletBoltHumanoidDriver::PyBulletBoltHumanoidDriver(
    bool real_time_mode,
    bool visualize,
    bool use_fixed_base,
    const std::string &logger_level)
    : real_time_mode_(real_time_mode)
{
    // initialise logger and set level based on config
    log_ = spdlog::get(LOGGER_NAME);
    if (!log_)
    {
        log_ = spdlog::stderr_color_mt(LOGGER_NAME);
        auto log_level = spdlog::level::from_str(logger_level);
        log_->set_level(log_level);
    }

    // initialize Python interpreter if not already done
    if (!Py_IsInitialized())
    {
        log_->debug("Initialize Python interpreter.");
        py::initialize_interpreter();
    }
    else
    {
        log_->debug("Python interpreter is already initialized.");
    }

    // set up simulation environment
    {
        py::gil_scoped_acquire acquire;

        py::module pybullet = py::module::import("pybullet");
        py::module bullet_utils_env = py::module::import("bullet_utils.env");
        py::module bolt_humanoid_wrapper =
            py::module::import("robot_properties_bolt.bolt_humanoid_wrapper");

        py::object RPSBoltHumanoidConfig =
            bolt_humanoid_wrapper.attr("BoltHumanoidConfig");

        py::object pybullet_server =
            visualize ? pybullet.attr("GUI") : pybullet.attr("DIRECT");
        sim_env_ =
            bullet_utils_env.attr("BulletEnvWithGround")(pybullet_server);
        sim_robot_ = bolt_humanoid_wrapper.attr("BoltHumanoidRobot")(
            "use_fixed_base"_a = py::cast(use_fixed_base));
        sim_env_.attr("add_robot")(sim_robot_);

        sim_robot_.attr("reset_to_initial_state")();
    }
}

void PyBulletBoltHumanoidDriver::initialize()
{
}

std::tuple<Vector9d, Vector9d>
PyBulletBoltHumanoidDriver::get_position_and_velocity()
{
    py::gil_scoped_acquire acquire;

    py::tuple py_state = sim_robot_.attr("get_state")();
    Vector9d joint_positions =
        py_state[0][py::slice(-9, std::nullopt, 1)].cast<Vector9d>();
    Vector9d joint_velocities =
        py_state[1][py::slice(-9, std::nullopt, 1)].cast<Vector9d>();

    return {joint_positions, joint_velocities};
}

BoltHumanoidObservation PyBulletBoltHumanoidDriver::get_latest_observation()
{
    BoltHumanoidObservation observation;

    observation.joint_torques = applied_torques_;
    observation.joint_target_torques = desired_torques_;

    // for the stuff below, we need to access Python objects
    py::gil_scoped_acquire gil;

    auto [joint_positions, joint_velocities] = get_position_and_velocity();
    observation.joint_positions = joint_positions;
    observation.joint_velocities = joint_velocities;

    py::function get_slider_position = sim_robot_.attr("get_slider_position");
    observation.slider_positions[0] = get_slider_position("a").cast<double>();
    observation.slider_positions[1] = get_slider_position("b").cast<double>();
    observation.slider_positions[2] = get_slider_position("c").cast<double>();
    observation.slider_positions[3] = get_slider_position("d").cast<double>();

    observation.imu_accelerometer =
        sim_robot_.attr("get_base_imu_linacc")().cast<Eigen::Vector3d>();
    observation.imu_gyroscope =
        sim_robot_.attr("get_base_imu_angvel")().cast<Eigen::Vector3d>();
    // NOTE: Other IMU data is currently not provided by the simulation, so set
    // them to zero
    observation.imu_linear_acceleration.setZero();
    observation.imu_attitude.setZero();

    return observation;
}

BoltHumanoidAction PyBulletBoltHumanoidDriver::apply_action(
    const BoltHumanoidAction &desired_action)
{
    auto start_time = std::chrono::system_clock::now();

    auto [joint_positions, joint_velocities] = get_position_and_velocity();

    // PD+ controller: Iq_ref = Iq_feeforward + Kp*err_pos + Kd*err_vel
    Vector9d position_error = desired_action.joint_positions - joint_positions;
    Vector9d velocity_error =
        desired_action.joint_velocities - joint_velocities;
    desired_torques_ = desired_action.joint_torques;
    applied_torques_ =
        desired_action.joint_torques +
        desired_action.joint_position_gains.cwiseProduct(position_error) +
        desired_action.joint_velocity_gains.cwiseProduct(velocity_error);

    {
        // wrap python calls in a block so GIL is only kept as long as it
        // is really needed
        py::gil_scoped_acquire acquire;

        py::object py_torques = py::cast(applied_torques_);
        sim_robot_.attr("send_joint_command")(py_torques);
        sim_env_.attr("step")(/*sleep=*/real_time_mode_);
    }

    if (real_time_mode_)
    {
        using namespace std::chrono_literals;
        std::this_thread::sleep_until(start_time + 1ms);
    }

    return desired_action;
}

std::string PyBulletBoltHumanoidDriver::get_error()
{
    return "";  // no errors
}

void PyBulletBoltHumanoidDriver::shutdown()
{
}

py::object PyBulletBoltHumanoidDriver::get_bullet_env()
{
    return sim_env_;
}

BoltHumanoidBackend::Ptr create_pybullet_bolthumanoid_backend(
    BoltHumanoidData::Ptr robot_data,
    const BoltHumanoidConfig &driver_config,
    const double first_action_timeout,
    const uint32_t max_number_of_actions)
{
    auto driver = std::make_shared<PyBulletBoltHumanoidDriver>(
        true, true, false, driver_config.logger_level);

    constexpr bool enable_timing_watchdog = false;
    return create_bolthumanoid_backend(robot_data,
                                       driver,
                                       first_action_timeout,
                                       max_number_of_actions,
                                       enable_timing_watchdog);
}
}  // namespace robot_interfaces_bolt
