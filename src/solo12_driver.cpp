#include <robot_interfaces_solo/solo12_driver.hpp>

#include <boost/range/adaptor/indexed.hpp>

namespace robot_interfaces_solo
{
void Solo12Driver::initialize()
{
    solo12_.initialize(config_.network_interface, config_.slider_serial_port);
    solo12_.set_max_current(config_.max_motor_current_A);

    // we have to call send_target_joint_torque() to trigger enabling the motors
    // and updating the state machine to know once it is ready
    Vector12d zero_torque = Vector12d::Zero();
    while (!solo12_.is_ready())
    {
        solo12_.send_target_joint_torque(zero_torque);
        real_time_tools::Timer::sleep_ms(100);
    }

    // Homing
    // The homing is also driven by calling send_target_joint_torque() in a
    // loop.  Use do-while-loop because after requesting calibration, we first
    // need to call it once to change the state from "ready" to "calibrate".
    solo12_.request_calibration(config_.home_offset_rad);
    do
    {
        solo12_.send_target_joint_torque(zero_torque);
        real_time_tools::Timer::sleep_ms(1);
    } while (!solo12_.is_ready());

    is_initialized_ = true;
}

Solo12Driver::Action Solo12Driver::apply_action(const Action &desired_action)
{
    double start_time_sec = real_time_tools::Timer::get_current_time_sec();

    if (!is_initialized_)
    {
        throw std::runtime_error(
            "Robot needs to be initialized before applying actions.  Run "
            "the `initialize()` method.");
    }

    // TODO: safety checks
    applied_action_ = desired_action;

    solo12_.send_target_joint_position_gains(
        applied_action_.joint_position_gains);
    solo12_.send_target_joint_position(applied_action_.joint_positions);
    solo12_.send_target_joint_velocity_gains(
        applied_action_.joint_velocity_gains);
    solo12_.send_target_joint_velocity(applied_action_.joint_velocities);

    // this method does the actual sending, so should be called in the end
    solo12_.send_target_joint_torque(applied_action_.joint_torques);

    // FIXME: implement better timing
    real_time_tools::Timer::sleep_until_sec(start_time_sec + 0.001);

    return applied_action_;
}

Solo12Driver::Observation Solo12Driver::get_latest_observation()
{
    if (!is_initialized_)
    {
        throw std::runtime_error(
            "Robot needs to be initialized before getting observations.  Run "
            "the `initialize()` method.");
    }

    Observation obs;

    solo12_.acquire_sensors();

    obs.joint_positions = solo12_.get_joint_positions();
    obs.joint_velocities = solo12_.get_joint_velocities();
    obs.joint_torques = solo12_.get_joint_torques();
    obs.joint_target_torques = solo12_.get_joint_target_torques();
    obs.joint_encoder_index = solo12_.get_joint_encoder_index();

    obs.slider_positions = solo12_.get_slider_positions();
    obs.imu_accelerometer = solo12_.get_imu_accelerometer();
    obs.imu_gyroscope = solo12_.get_imu_gyroscope();
    obs.imu_linear_acceleration = solo12_.get_imu_linear_acceleration();
    obs.imu_attitude = solo12_.get_imu_attitude();

    obs.num_sent_command_packets = solo12_.get_num_sent_command_packets();
    obs.num_lost_command_packets = solo12_.get_num_lost_command_packets();
    obs.num_sent_sensor_packets = solo12_.get_num_sent_sensor_packets();
    obs.num_lost_sensor_packets = solo12_.get_num_lost_sensor_packets();

    return obs;
}

std::string Solo12Driver::get_error()
{
    std::string error_msg = "";

    auto board_errors = solo12_.get_motor_board_errors();

    if (solo12_.has_error())
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

        if (!error_msg.empty())
        {
            error_msg += "Unknown Error";
        }
    }

    // TODO
    // Some errors on the master board are not reported but simply result in a
    // motor to be disabled.  So check for this explicitly.

    return error_msg;
}

void Solo12Driver::shutdown()
{
    // TODO: is there a way to completely disable motors?
    if (is_initialized_)
    {
        apply_action(Action::Zero());
    }
}

Solo12Backend::Ptr create_solo12_backend(Solo12Data::Ptr robot_data,
                                         const Solo12Config &driver_config,
                                         const double first_action_timeout,
                                         const uint32_t max_number_of_actions)
{
    constexpr double MAX_ACTION_DURATION_S = 0.003;
    constexpr double MAX_INTER_ACTION_DURATION_S = 0.005;

    // config.print();

    // wrap the actual robot driver directly in a MonitoredRobotDriver
    auto monitored_driver =
        std::make_shared<robot_interfaces::MonitoredRobotDriver<Solo12Driver>>(
            std::make_shared<Solo12Driver>(driver_config),
            MAX_ACTION_DURATION_S,
            MAX_INTER_ACTION_DURATION_S);

    constexpr bool real_time_mode = true;
    auto backend = std::make_shared<Solo12Backend>(monitored_driver,
                                                   robot_data,
                                                   real_time_mode,
                                                   first_action_timeout,
                                                   max_number_of_actions);
    backend->set_max_action_repetitions(std::numeric_limits<uint32_t>::max());

    return backend;
}
}  // namespace robot_interfaces_solo
