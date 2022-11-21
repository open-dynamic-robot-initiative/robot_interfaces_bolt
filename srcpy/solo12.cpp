/**
 * \file
 * \brief Python bindings for Solo12
 * \copyright Copyright (c) 2022, Max Planck Gesellschaft.
 */
#include <limits>
#include <memory>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include <robot_interfaces/pybind_helper.hpp>
#include <robot_interfaces_solo/solo12_driver.hpp>
#include <robot_interfaces_solo/solo12_pybullet_driver.hpp>
#include <robot_interfaces_solo/solo12_utils.hpp>

namespace ris = robot_interfaces_solo;

PYBIND11_MODULE(solo12, m)
{
    pybind11::options options;
    // disable automatic function signature generation as this does not look too
    // nice in the Sphinx documentation.
    options.disable_function_signatures();

    // import robot_interfaces to provide bindings for Status
    pybind11::module::import("robot_interfaces");

    robot_interfaces::create_interface_python_bindings<ris::Solo12Action,
                                                       ris::Solo12Observation>(
        m);

    pybind11::class_<ris::Solo12Action>(m, "Action", "Action for Solo12")
        .def(pybind11::init<>())
        .def_readwrite("joint_torques",
                       &ris::Solo12Action::joint_torques,
                       "List of desired torques, one per joint.")
        .def_readwrite("joint_positions",
                       &ris::Solo12Action::joint_positions,
                       "Desired joint positions of the P controller running on "
                       "the card.  For this to have an effect "
                       ":attr:`joint_position_gains` needs to be set as well.")
        .def_readwrite("joint_velocities",
                       &ris::Solo12Action::joint_velocities,
                       "desired joint velocity of the D controller running on "
                       "the card.  For this to have an effect, "
                       ":attr:`joint_velocity_gains` needs to be set as well.")
        .def_readwrite("joint_position_gains",
                       &ris::Solo12Action::joint_position_gains,
                       "P-gains for the on-board controller, one per joint.")
        .def_readwrite("joint_velocity_gains",
                       &ris::Solo12Action::joint_velocity_gains,
                       "D-gains for the on-barod controller, one per joint.")
        .def(pybind11::pickle(
            [](const ris::Solo12Action &a) {  // __getstate__
                // Return a tuple that fully encodes the state of the object
                return pybind11::make_tuple(a.joint_torques,
                                            a.joint_positions,
                                            a.joint_velocities,
                                            a.joint_position_gains,
                                            a.joint_velocity_gains);
            },
            [](pybind11::tuple t) {  // __setstate__
                if (t.size() != 5)
                {
                    throw std::runtime_error("Invalid state!");
                }

                // Create a new C++ instance
                ris::Solo12Action action;
                action.joint_torques = t[0].cast<ris::Vector12d>();
                action.joint_positions = t[1].cast<ris::Vector12d>();
                action.joint_velocities = t[2].cast<ris::Vector12d>();
                action.joint_position_gains = t[3].cast<ris::Vector12d>();
                action.joint_velocity_gains = t[4].cast<ris::Vector12d>();

                return action;
            }))
        .def("Zero", &ris::Solo12Action::Zero, "Create a zero-torque action");

    pybind11::class_<ris::Solo12Observation>(m, "Observation")
        .def(pybind11::init<>())
        .def_readwrite("joint_positions",
                       &ris::Solo12Observation::joint_positions)
        .def_readwrite("joint_velocities",
                       &ris::Solo12Observation::joint_velocities)
        .def_readwrite("joint_torques", &ris::Solo12Observation::joint_torques)
        .def_readwrite("joint_target_torques",
                       &ris::Solo12Observation::joint_target_torques)
        .def_readwrite("joint_encoder_index",
                       &ris::Solo12Observation::joint_encoder_index)
        .def_readwrite("slider_positions",
                       &ris::Solo12Observation::slider_positions)
        .def_readwrite("imu_accelerometer",
                       &ris::Solo12Observation::imu_accelerometer)
        .def_readwrite("imu_gyroscope", &ris::Solo12Observation::imu_gyroscope)
        .def_readwrite("imu_linear_acceleration",
                       &ris::Solo12Observation::imu_linear_acceleration)
        .def_readwrite("imu_attitude", &ris::Solo12Observation::imu_attitude)
        .def_readwrite("num_sent_command_packets",
                       &ris::Solo12Observation::num_sent_command_packets)
        .def_readwrite("num_lost_command_packets",
                       &ris::Solo12Observation::num_lost_command_packets)
        .def_readwrite("num_sent_sensor_packets",
                       &ris::Solo12Observation::num_sent_sensor_packets)
        .def_readwrite("num_lost_sensor_packets",
                       &ris::Solo12Observation::num_lost_sensor_packets);

    pybind11::class_<ris::Solo12Config, std::shared_ptr<ris::Solo12Config>>(
        m, "Config")
        .def(pybind11::init<>())
        .def_readwrite("network_interface",
                       &ris::Solo12Config::network_interface,
                       "Name of the network interface to which the robot is "
                       "connected (e.g. 'eth0')")
        .def_readwrite("slider_serial_port",
                       &ris::Solo12Config::slider_serial_port,
                       "Name of the serial port to which the hardware slider "
                       "is connected. This can typically be left empty, in "
                       "which case the port is auto-detected.")
        .def_readwrite(
            "max_motor_current_A",
            &ris::Solo12Config::max_motor_current_A,
            "Maximum current that can be applied to the motors (in Ampere).")
        .def_readwrite("home_offset_rad",
                       &ris::Solo12Config::home_offset_rad,
                       "Offset between home position (=encoder index) and zero "
                       "position.\n\nAngles (in radian) between the encoder "
                       "index and the zero position of each joint.")
        .def("from_file",
             &ris::Solo12Config::from_file,
             "Load configuration from a YAML file (using default values for "
             "parameters missing in the file).");

    pybind11::class_<ris::BaseSolo12Driver, ris::BaseSolo12Driver::Ptr>
        base_driver(m, "BaseSolo12Driver");

    pybind11::class_<ris::PyBulletSolo12Driver,
                     std::shared_ptr<ris::PyBulletSolo12Driver>,
                     ris::BaseSolo12Driver>(m, "PyBulletDriver")
        .def(pybind11::init<bool, bool, const std::string &>(),
             pybind11::arg("real_time_mode"),
             pybind11::arg("visualize"),
             pybind11::arg("logger_level") = "debug")
        .def("get_bullet_env", &ris::PyBulletSolo12Driver::get_bullet_env);

    m.def("create_backend",
          &ris::create_solo12_backend,
          pybind11::arg("robot_data"),
          pybind11::arg("robot_driver"),
          pybind11::arg("first_action_timeout") =
              std::numeric_limits<double>::infinity(),
          pybind11::arg("max_number_of_actions") = 0,
          pybind11::arg("enable_timing_watchdog") = true);

    m.def("create_real_backend",
          &ris::create_real_solo12_backend,
          pybind11::arg("robot_data"),
          pybind11::arg("driver_config"),
          pybind11::arg("first_action_timeout") =
              std::numeric_limits<double>::infinity(),
          pybind11::arg("max_number_of_actions") = 0);

    m.def("create_fake_backend",
          &ris::create_fake_solo12_backend,
          pybind11::arg("robot_data"),
          pybind11::arg("driver_config"),
          pybind11::arg("first_action_timeout") =
              std::numeric_limits<double>::infinity(),
          pybind11::arg("max_number_of_actions") = 0);

    m.def("create_pybullet_backend",
          &ris::create_pybullet_solo12_backend,
          pybind11::arg("robot_data"),
          pybind11::arg("driver_config"),
          pybind11::arg("first_action_timeout") =
              std::numeric_limits<double>::infinity(),
          pybind11::arg("max_number_of_actions") = 0);
}
