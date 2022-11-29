#include <robot_interfaces_solo/solo12_utils.hpp>

#include <memory>

#include <robot_interfaces/monitored_robot_driver.hpp>

namespace robot_interfaces_solo
{
Solo12Backend::Ptr create_solo12_backend(Solo12Data::Ptr robot_data,
                                         BaseSolo12Driver::Ptr robot_driver,
                                         const double first_action_timeout,
                                         const uint32_t max_number_of_actions,
                                         bool enable_timing_watchdog)
{
    constexpr double MAX_ACTION_DURATION_S = 0.003;
    constexpr double MAX_INTER_ACTION_DURATION_S = 0.005;

    std::shared_ptr<
        robot_interfaces::RobotDriver<Solo12Action, Solo12Observation>>
        wrapped_driver;

    if (enable_timing_watchdog)
    {
        // wrap the actual robot driver directly in a MonitoredRobotDriver
        wrapped_driver = std::make_shared<
            robot_interfaces::MonitoredRobotDriver<BaseSolo12Driver>>(
            robot_driver, MAX_ACTION_DURATION_S, MAX_INTER_ACTION_DURATION_S);
    }
    else
    {
        // no wrapping
        wrapped_driver = robot_driver;
    }

    constexpr bool real_time_mode = true;
    auto backend = std::make_shared<Solo12Backend>(wrapped_driver,
                                                   robot_data,
                                                   real_time_mode,
                                                   first_action_timeout,
                                                   max_number_of_actions);
    backend->set_max_action_repetitions(std::numeric_limits<uint32_t>::max());

    return backend;
}

}  // namespace robot_interfaces_solo
