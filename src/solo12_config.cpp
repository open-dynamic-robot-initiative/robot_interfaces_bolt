#include <robot_interfaces_solo/solo12_config.hpp>

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include <yaml_utils/yaml_eigen.hpp>

namespace robot_interfaces_solo
{
template <typename T>
void _set_optional_config_value(const YAML::Node &user_config,
                                const std::string &name,
                                T *var)
{
    try
    {
        if (user_config[name])
        {
            *var = user_config[name].as<T>();
        }
    }
    catch (const YAML::Exception &e)
    {
        throw std::runtime_error(fmt::format(
            "FATAL: Failed to load parameter '{}' from configuration file: {}",
            name,
            e.what()));
    };
}

Solo12Config Solo12Config::from_file(
    const std::filesystem::path &config_file_name)
{
    Solo12Config config;
    YAML::Node user_config;

    try
    {
        user_config = YAML::LoadFile(config_file_name);
    }
    catch (const YAML::Exception &e)
    {
        throw std::runtime_error(
            fmt::format("FATAL: Failed to load configuration from '{}': {}",
                        config_file_name.string(),
                        e.what()));
    }

    _set_optional_config_value(
        user_config, "network_interface", &config.network_interface);
    _set_optional_config_value(
        user_config, "slider_serial_port", &config.slider_serial_port);
    _set_optional_config_value(
        user_config, "max_motor_current_A", &config.max_motor_current_A);
    _set_optional_config_value(
        user_config, "home_offset_rad", &config.home_offset_rad);
    _set_optional_config_value(
        user_config, "logger_level", &config.logger_level);

    return config;
}

}  // namespace robot_interfaces_solo
