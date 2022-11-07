/**
 * \file
 * \brief Demo using Solo12 that holds all joints at their current positions.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#include <cli_utils/program_options.hpp>

#include <robot_interfaces_solo/solo12_driver.hpp>

using namespace robot_interfaces_solo;

class Args : public cli_utils::ProgramOptions
{
public:
    std::string config_file;
    float kp = 3.0, kd = 0.05;

    std::string help() const override
    {
        return R"(Demo using Solo12 that holds all joints at their current positions.

Usage:  demo_solo12_hold <network-interface> [<serial-port>]

)";
    }

    void add_options(boost::program_options::options_description &options,
                     boost::program_options::positional_options_description
                         &positional) override
    {
        namespace po = boost::program_options;
        // clang-format off
        options.add_options()
            ("config-file",
             po::value<std::string>(&config_file)->required(),
             "Path to the driver configuration file.")
            ("kp",
             po::value<float>(&kp),
             "P-gain for the position control.")
            ("kd",
             po::value<float>(&kd),
             "D-gain for the position control.")
            ;
        // clang-format on

        positional.add("config-file", 1);
    }
};

int main(int argc, char *argv[])
{
    Args args;
    if (!args.parse_args(argc, argv))
    {
        return 1;
    }

    Solo12Config config = Solo12Config::from_file(args.config_file);

    auto data = std::make_shared<Solo12SingleProcessData>();
    Solo12Backend::Ptr backend = create_solo12_backend(data, config);
    Solo12Frontend frontend(data);

    backend->initialize();

    // start with a zero-torque action
    auto t = frontend.append_desired_action(Solo12Action::Zero());
    Solo12Observation obs = frontend.get_observation(t);

    Solo12Action action;
    action.joint_position_gains.fill(args.kp);
    action.joint_velocity_gains.fill(args.kd);
    action.joint_positions = obs.joint_positions;

    while (backend->is_running())
    {
        t = frontend.append_desired_action(action);
        frontend.wait_until_timeindex(t);
    }

    backend->request_shutdown();

    return 0;
}
