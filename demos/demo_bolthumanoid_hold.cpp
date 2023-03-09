/**
 * \file
 * \brief Demo using BoltHumanoid that holds all joints at their current positions.
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#include <memory>
#include <string>

#include <cli_utils/program_options.hpp>

#include <robot_interfaces_bolt/bolthumanoid_driver.hpp>

namespace ris = robot_interfaces_bolt;

// Class to get console arguments
class Args : public cli_utils::ProgramOptions
{
public:
    std::string config_file;
    float kp = 3.0, kd = 0.05;

    std::string help() const override
    {
        return R"(Demo using BoltHumanoid that holds all joints at their current positions.

Usage:  demo_bolthumanoid_hold <network-interface> [<serial-port>]

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

    // load the driver configuration from a YAML file
    ris::BoltHumanoidConfig config = ris::BoltHumanoidConfig::from_file(args.config_file);

    // create a robot data instance, robot backend and frontend.
    auto data = std::make_shared<ris::BoltHumanoidSingleProcessData>();
    ris::BoltHumanoidBackend::Ptr backend = create_real_bolthumanoid_backend(data, config);
    ris::BoltHumanoidFrontend frontend(data);

    // initialise the robot (this also runs the homing)
    backend->initialize();

    // start with a zero-torque action (we need to send an initial action first,
    // before we can access the observation)
    auto t = frontend.append_desired_action(ris::BoltHumanoidAction::Zero());
    ris::BoltHumanoidObservation obs = frontend.get_observation(t);

    // construct a simple position control action, using the current position
    // from the observation as target position.
    ris::BoltHumanoidAction action;
    action.joint_position_gains.fill(args.kp);
    action.joint_velocity_gains.fill(args.kd);
    action.joint_positions = obs.joint_positions;

    // simply apply the action in a loop, to hold the joints at the current
    // position
    while (backend->is_running())
    {
        t = frontend.append_desired_action(action);
        frontend.wait_until_timeindex(t);
    }

    backend->request_shutdown();

    return 0;
}
