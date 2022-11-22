#include <robot_interfaces_solo/solo12_pybullet_driver.hpp>

#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/stl_bind.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <robot_interfaces_solo/solo12_utils.hpp>

using namespace pybind11::literals;

namespace robot_interfaces_solo
{
PyBulletSolo12Driver::PyBulletSolo12Driver(bool real_time_mode,
                                           bool visualize,
                                           bool use_fixed_base,
                                           const std::string &logger_level)
    : real_time_mode_(real_time_mode), visualize_(visualize)
{
    // initialise logger and set level based on config
    log_ = spdlog::get(LOGGER_NAME);
    if (!log_)
    {
        log_ = spdlog::stderr_color_mt(LOGGER_NAME);
        // TODO: better way to configure this?
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
        py::module solo12wrapper =
            py::module::import("robot_properties_solo.solo12wrapper");

        py::object RPSSolo12Config = solo12wrapper.attr("Solo12Config");

        py::object pybullet_server =
            visualize ? pybullet.attr("GUI") : pybullet.attr("DIRECT");
        sim_env_ =
            bullet_utils_env.attr("BulletEnvWithGround")(pybullet_server);
        sim_robot_ = solo12wrapper.attr("Solo12Robot")(
            "useFixedBase"_a = py::cast(use_fixed_base));
        sim_env_.attr("add_robot")(sim_robot_);

        sim_robot_.attr("reset_to_initial_state")();
    }
}

void PyBulletSolo12Driver::initialize()
{
}

Solo12Observation PyBulletSolo12Driver::get_latest_observation()
{
    Solo12Observation observation;

    sensor_packet_counter_++;

    // these fields can already be set before acquiring the GIL
    observation.num_sent_command_packets = command_packet_counter_;
    observation.num_lost_command_packets = 0;
    observation.num_sent_sensor_packets = sensor_packet_counter_;
    observation.num_lost_sensor_packets = 0;

    observation.joint_torques = applied_torques_;
    // FIXME this is wrong, target_torques only contains the torque part of the
    // action
    observation.joint_target_torques = applied_torques_;

    // for the stuff below, we need to access Python objects
    py::gil_scoped_acquire gil;

    py::tuple py_state = sim_robot_.attr("get_state")();
    observation.joint_positions =
        py_state[0][py::slice(-12, std::nullopt, 1)].cast<Vector12d>();
    observation.joint_velocities =
        py_state[1][py::slice(-12, std::nullopt, 1)].cast<Vector12d>();

    py::function get_slider_position = sim_robot_.attr("get_slider_position");
    observation.slider_positions[0] = get_slider_position("a").cast<double>();
    observation.slider_positions[1] = get_slider_position("b").cast<double>();
    observation.slider_positions[2] = get_slider_position("c").cast<double>();
    observation.slider_positions[3] = get_slider_position("d").cast<double>();

    observation.imu_accelerometer =
        sim_robot_.attr("get_base_imu_linacc")().cast<Eigen::Vector3d>();
    observation.imu_gyroscope =
        sim_robot_.attr("get_base_imu_angvel")().cast<Eigen::Vector3d>();
    // FIXME: Other IMU data

    return observation;
}

Solo12Action PyBulletSolo12Driver::apply_action(
    const Solo12Action &desired_action)
{
    auto start_time = std::chrono::system_clock::now();

    // TODO: can be done more efficiently (only joint pos and vel are used)
    Solo12Observation observation = get_latest_observation();

    // PD+ controller: Iq_ref = Iq_feeforward + Kp*err_pos + Kd*err_vel
    Vector12d position_error =
        desired_action.joint_positions - observation.joint_positions;
    Vector12d velocity_error =
        desired_action.joint_velocities - observation.joint_velocities;
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

    command_packet_counter_++;

    return desired_action;
}

std::string PyBulletSolo12Driver::get_error()
{
    return "";  // no errors
}

void PyBulletSolo12Driver::shutdown()
{
}

py::object PyBulletSolo12Driver::get_bullet_env()
{
    return sim_env_;
}

Solo12Backend::Ptr create_pybullet_solo12_backend(
    Solo12Data::Ptr robot_data,
    const Solo12Config &driver_config,
    const double first_action_timeout,
    const uint32_t max_number_of_actions)
{
    auto driver = std::make_shared<PyBulletSolo12Driver>(
        true, true, false, driver_config.logger_level);

    constexpr bool enable_timing_watchdog = false;
    return create_solo12_backend(robot_data,
                                 driver,
                                 first_action_timeout,
                                 max_number_of_actions,
                                 enable_timing_watchdog);
}
}  // namespace robot_interfaces_solo
