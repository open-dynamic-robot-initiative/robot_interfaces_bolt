#include <robot_interfaces_bolt/bolthumanoid_driver.hpp>

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <fmt/format.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <yaml-cpp/yaml.h>
#include <boost/range/adaptor/indexed.hpp>

#include <real_time_tools/spinner.hpp>

#include <robot_interfaces_bolt/bolthumanoid_utils.hpp>

namespace robot_interfaces_bolt
{
BoltHumanoidDriver::BoltHumanoidDriver(const BoltHumanoidConfig &config)
    : config_(config)
{
    // initialise logger and set level based on config
    log_ = spdlog::get(LOGGER_NAME);
    if (!log_)
    {
        log_ = spdlog::stderr_color_mt(LOGGER_NAME);
        auto log_level = spdlog::level::from_str(config.logger_level);
        log_->set_level(log_level);
    }
}

void BoltHumanoidDriver::initialize()
{
    log_->debug("Initialize BoltHumanoid");
    bolthumanoid_.initialize(config_.network_interface,
                             config_.slider_serial_port);
    bolthumanoid_.set_max_current(config_.max_motor_current_A);

    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);

    // we have to call acquire_sensors() and send_target_joint_torque() to
    // trigger enabling the motors and updating the state machine to know once
    // it is ready
    log_->debug("Enable motors");
    Vector9d zero_torque = Vector9d::Zero();
    do
    {
        bolthumanoid_.acquire_sensors();
        bolthumanoid_.send_target_joint_torque(zero_torque);
        spinner.spin();
    } while (!bolthumanoid_.is_ready());

    // Homing
    // The homing is also driven by calling send_target_joint_torque() in a
    // loop.  Use do-while-loop because after requesting calibration, we first
    // need to call it once to change the state from "ready" to "calibrate".
    log_->debug("Start homing");
    bolthumanoid_.request_calibration(config_.home_offset_rad);
    do
    {
        bolthumanoid_.acquire_sensors();
        bolthumanoid_.send_target_joint_torque(zero_torque);
        spinner.spin();
    } while (!bolthumanoid_.is_ready());

    is_initialized_ = true;
    log_->debug("Initialization finished");
}

BoltHumanoidDriver::Action BoltHumanoidDriver::apply_action(
    const Action &desired_action)
{
    double start_time_sec = real_time_tools::Timer::get_current_time_sec();

    if (!is_initialized_)
    {
        throw std::runtime_error(
            "Robot needs to be initialized before applying actions.  Run "
            "the `initialize()` method.");
    }

    // TODO: safety checks?
    applied_action_ = desired_action;

    bolthumanoid_.send_target_joint_position_gains(
        applied_action_.joint_position_gains);
    bolthumanoid_.send_target_joint_position(applied_action_.joint_positions);
    bolthumanoid_.send_target_joint_velocity_gains(
        applied_action_.joint_velocity_gains);
    bolthumanoid_.send_target_joint_velocity(applied_action_.joint_velocities);

    // this method does the actual sending, so should be called in the end
    bolthumanoid_.send_target_joint_torque(applied_action_.joint_torques);

    // FIXME: implement better timing
    real_time_tools::Timer::sleep_until_sec(start_time_sec + 0.001);

    return applied_action_;
}

BoltHumanoidDriver::Observation BoltHumanoidDriver::get_latest_observation()
{
    if (!is_initialized_)
    {
        throw std::runtime_error(
            "Robot needs to be initialized before getting observations.  Run "
            "the `initialize()` method.");
    }

    Observation obs;

    bolthumanoid_.acquire_sensors();

    obs.joint_positions = bolthumanoid_.get_joint_positions();
    obs.joint_velocities = bolthumanoid_.get_joint_velocities();
    obs.joint_torques = bolthumanoid_.get_joint_torques();
    obs.joint_target_torques = bolthumanoid_.get_joint_target_torques();
    // FIXME
    // obs.joint_encoder_index = bolthumanoid_.get_joint_encoder_index();

    obs.slider_positions = bolthumanoid_.get_slider_positions();
    obs.imu_accelerometer = bolthumanoid_.get_base_accelerometer();
    obs.imu_gyroscope = bolthumanoid_.get_base_gyroscope();
    obs.imu_linear_acceleration = bolthumanoid_.get_base_linear_acceleration();
    obs.imu_attitude = bolthumanoid_.get_base_attitude();

    // FIXME
    // obs.num_sent_command_packets =
    // bolthumanoid_.get_num_sent_command_packets();
    // obs.num_lost_command_packets =
    // bolthumanoid_.get_num_lost_command_packets(); obs.num_sent_sensor_packets
    // = bolthumanoid_.get_num_sent_sensor_packets();
    // obs.num_lost_sensor_packets =
    // bolthumanoid_.get_num_lost_sensor_packets();

    return obs;
}

std::string BoltHumanoidDriver::get_error()
{
    std::string error_msg = "";

    auto board_errors_mat = bolthumanoid_.get_motor_board_errors();
    // copy Eigen matrix to std::vector
    std::vector<int> board_errors(
        board_errors_mat.data(),
        board_errors_mat.data() + board_errors_mat.size());

    if (bolthumanoid_.has_error())
    {
        for (auto error_code : boost::adaptors::index(board_errors))
        {
            if (error_code.value() != 0)
            {
                if (!error_msg.empty())
                {
                    // add separator if there are multiple errors
                    error_msg += " | ";
                }

                // TODO: translate error code into human-readable message
                error_msg += "Board " + std::to_string(error_code.index()) +
                             ": Error " + std::to_string(error_code.value());
            }
        }

        if (error_msg.empty())
        {
            error_msg += "Unknown Error";
        }
    }

    // TODO
    // Some errors on the master board are not reported but simply result in a
    // motor to be disabled.  So check for this explicitly.

    return error_msg;
}

void BoltHumanoidDriver::shutdown()
{
    // TODO: is there a way to completely disable motors?
    if (is_initialized_)
    {
        apply_action(Action::Zero());
    }
}

FakeBoltHumanoidDriver::FakeBoltHumanoidDriver(const BoltHumanoidConfig &config)
    : config_(config)
{
    // initialise logger and set level based on config
    log_ = spdlog::get(LOGGER_NAME);
    if (!log_)
    {
        log_ = spdlog::stderr_color_mt(LOGGER_NAME);
        auto log_level = spdlog::level::from_str(config.logger_level);
        log_->set_level(log_level);
    }
}

void FakeBoltHumanoidDriver::initialize()
{
    log_->debug("Initialize Fake BoltHumanoid");
    real_time_tools::Timer::sleep_sec(2);
    is_initialized_ = true;
}

FakeBoltHumanoidDriver::Action FakeBoltHumanoidDriver::apply_action(
    const Action &desired_action)
{
    double start_time_sec = real_time_tools::Timer::get_current_time_sec();

    if (!is_initialized_)
    {
        throw std::runtime_error(
            "Robot needs to be initialized before applying actions.  Run "
            "the `initialize()` method.");
    }

    real_time_tools::Timer::sleep_until_sec(start_time_sec + 0.001);

    return desired_action;
}

FakeBoltHumanoidDriver::Observation
FakeBoltHumanoidDriver::get_latest_observation()
{
    if (!is_initialized_)
    {
        throw std::runtime_error(
            "Robot needs to be initialized before getting observations.  Run "
            "the `initialize()` method.");
    }

    double freq = 0.5;
    double amplitude = M_PI;

    Observation obs;

    // fill with some fake data (using sine waves)
    obs.joint_positions.fill(amplitude * std::sin(2.0 * M_PI * freq * t_));
    obs.joint_velocities.fill(2.0 * M_PI * freq * amplitude *
                              std::cos(2.0 * M_PI * freq * t_));
    obs.joint_torques = obs.joint_velocities / 3;

    obs.slider_positions << 0.0, 0.3, 0.5, 1.0;

    obs.imu_accelerometer = Eigen::Vector3d::Random();
    obs.imu_gyroscope = Eigen::Vector3d::Random();
    obs.imu_linear_acceleration = Eigen::Vector3d::Random();
    obs.imu_attitude = Eigen::Vector4d::Random();

    obs.num_sent_command_packets = t_ * 1000;
    obs.num_lost_command_packets = 0;
    obs.num_sent_sensor_packets = t_ * 1000;
    ;
    obs.num_lost_sensor_packets = 0;

    t_ += 0.001;

    return obs;
}

std::string FakeBoltHumanoidDriver::get_error()
{
    return "";
}

void FakeBoltHumanoidDriver::shutdown()
{
}

BoltHumanoidBackend::Ptr create_real_bolthumanoid_backend(
    BoltHumanoidData::Ptr robot_data,
    const BoltHumanoidConfig &driver_config,
    const double first_action_timeout,
    const uint32_t max_number_of_actions)
{
    constexpr bool enable_timing_watchdog = true;
    return create_bolthumanoid_backend(
        robot_data,
        std::make_shared<BoltHumanoidDriver>(driver_config),
        first_action_timeout,
        max_number_of_actions,
        enable_timing_watchdog);
}

BoltHumanoidBackend::Ptr create_fake_bolthumanoid_backend(
    BoltHumanoidData::Ptr robot_data,
    const BoltHumanoidConfig &driver_config,
    const double first_action_timeout,
    const uint32_t max_number_of_actions)
{
    auto driver = std::make_shared<FakeBoltHumanoidDriver>(driver_config);

    constexpr bool enable_timing_watchdog = false;
    return create_bolthumanoid_backend(robot_data,
                                       driver,
                                       first_action_timeout,
                                       max_number_of_actions,
                                       enable_timing_watchdog);
}

}  // namespace robot_interfaces_bolt
