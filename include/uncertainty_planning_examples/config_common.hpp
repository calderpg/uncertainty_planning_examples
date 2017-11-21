#include <ros/ros.h>
#include <uncertainty_planning_core/uncertainty_planning_core.hpp>

#ifndef CONFIG_COMMON_HPP
#define CONFIG_COMMON_HPP

namespace config_common
{
    struct TASK_CONFIG_PARAMS
    {
        double environment_resolution;
        double simulation_controller_frequency;
        double sensor_error;
        double proportional_actuator_error;
        double minimum_actuator_error;
        std::string environment_id;

        TASK_CONFIG_PARAMS(const double in_environment_resolution, const double in_simulation_controller_frequency, const double in_sensor_error, const double in_proportional_actuator_error, const double in_minimum_actuator_error, const std::string& in_environment_id) : environment_resolution(in_environment_resolution), simulation_controller_frequency(in_simulation_controller_frequency), sensor_error(in_sensor_error), proportional_actuator_error(in_proportional_actuator_error), minimum_actuator_error(in_minimum_actuator_error), environment_id(in_environment_id) {}

        TASK_CONFIG_PARAMS() : environment_resolution(0.0), simulation_controller_frequency(0.0), sensor_error(0.0), proportional_actuator_error(0.0), minimum_actuator_error(0.0), environment_id("") {}
    };

    inline TASK_CONFIG_PARAMS GetOptions(const TASK_CONFIG_PARAMS& initial_options)
    {
        TASK_CONFIG_PARAMS options = initial_options;
        // Get options via ROS params
        ros::NodeHandle nhp("~");
        options.environment_resolution = nhp.param(std::string("environment_resolution"), options.environment_resolution);
        options.simulation_controller_frequency = nhp.param(std::string("simulation_controller_frequency"), options.simulation_controller_frequency);
        options.sensor_error = nhp.param(std::string("sensor_error"), options.sensor_error);
        options.proportional_actuator_error = nhp.param(std::string("proportional_actuator_error"), options.proportional_actuator_error);
        options.minimum_actuator_error = nhp.param(std::string("minimum_actuator_error"), options.proportional_actuator_error);
        options.environment_id = nhp.param(std::string("environment_id"), options.environment_id);
        return options;
    }

    inline std::ostream& operator<<(std::ostream& strm, const TASK_CONFIG_PARAMS& options)
    {
        strm << "TASK_CONFIG_PARAMS:";
        strm << "\nenvironment_resolution: " << options.environment_resolution;
        strm << "\nsimulation_controller_frequency: " << options.simulation_controller_frequency;
        strm << "\nsensor_error: " << options.sensor_error;
        strm << "\nproportional_actuator_error: " << options.proportional_actuator_error;
        strm << "\nminimum_actuator_error: " << options.minimum_actuator_error;
        strm << "\nenvironment_id: " << options.environment_id;
        return strm;
    }
}

#endif // CONFIG_COMMON_HPP
