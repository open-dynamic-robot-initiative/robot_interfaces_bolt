/**
 * \file
 * \brief Demo using Solo12 that holds all joints at its current position.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#include <cli_utils/program_options.hpp>

#include <robot_interfaces_solo/solo12_driver.hpp>

using namespace robot_interfaces_solo;

class Args : public cli_utils::ProgramOptions
{
public:
    std::string network_interface, serial_port;

    std::string help() const override
    {
        return R"(Run pose detection on image files of one camera observation.

Load images of the three cameras from files "camera{60,180,300}.png", run the
pose detection on them and visualize the result.

Usage:  single_observation [options] <data-dir>

)";
    }

    void add_options(boost::program_options::options_description &options,
                     boost::program_options::positional_options_description
                         &positional) override
    {
        namespace po = boost::program_options;
        // clang-format off
        options.add_options()
            ("network-interface",
             po::value<std::string>(&network_interface)->required(),
             "Name of the network interface to which the robot is connected."
             " (e.g. 'eth0').")
            ("serial-port",
             po::value<std::string>(&serial_port)->required(),
             "Serial port to which the hardware slider is connected.")
            ;
        // clang-format on

        positional.add("network-interface", 1);
        positional.add("serial-port", 1);
    }
};

int main(int argc, char *argv[])
{
    Args args;
    if (!args.parse_args(argc, argv))
    {
        return 1;
    }

    Solo12Config config;
    config.network_interface = args.network_interface;
    config.serial_port = args.serial_port;

    Solo12Driver driver(config);
    auto data = std::make_shared<Solo12SingleProcessData>();
    Solo12Backend::Ptr backend = create_solo12_backend(data, config);
    Solo12Frontend frontend(data);

    backend->initialize();

    // start with a zero-torque action
    auto t = frontend.append_desired_action(Solo12Action::Zero());
    Solo12Observation obs = frontend.get_observation(t);

    Solo12Action action;
    action.joint_position_gains.fill(1);
    action.joint_velocity_gains.fill(0.5);
    action.joint_positions = obs.joint_positions;

    while (backend->is_running())
    {
        t = frontend.append_desired_action(action);
        frontend.wait_until_timeindex(t);
    }

    backend->request_shutdown();

    return 0;
}
