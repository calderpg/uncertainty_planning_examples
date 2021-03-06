#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <functional>
#include <random>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <arc_utilities/simple_rrt_planner.hpp>
#include <uncertainty_planning_core/simple_pid_controller.hpp>
#include <uncertainty_planning_core/simple_uncertainty_models.hpp>
#include <uncertainty_planning_core/uncertainty_contact_planning.hpp>
#include <uncertainty_planning_examples/se3_common_config.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

using namespace uncertainty_contact_planning;

void plan(ros::Publisher& display_debug_publisher)
{
    const uncertainty_planning_core::PLANNING_AND_EXECUTION_OPTIONS options = se3_common_config::GetOptions();
    //options.prng_seed = UINT64_C(1490038935709266544);
    const config_common::TASK_CONFIG_PARAMS extra_options = se3_common_config::GetExtraOptions();
    std::cout << PrettyPrint::PrettyPrint(options) << "\n" << PrettyPrint::PrettyPrint(extra_options) << std::endl;
    const std::pair<uncertainty_planning_core::SE3Config, uncertainty_planning_core::SE3Config> start_and_goal = se3_common_config::GetStartAndGoal();
    const uncertainty_planning_core::SE3SamplerPtr sampler = se3_common_config::GetSampler();
    const simple_robot_models::SE3_ROBOT_CONFIG robot_config = se3_common_config::GetDefaultRobotConfig(extra_options);
    const uncertainty_planning_core::SE3Robot robot = se3_common_config::GetRobot(robot_config, 1.0, 1.0);
    const uncertainty_planning_core::SE3SimulatorPtr simulator = se3_common_config::GetSimulator(extra_options, options.debug_level);
    auto planner_result = uncertainty_planning_core::PlanSE3Uncertainty(options, robot, simulator, sampler, start_and_goal.first, start_and_goal.second, display_debug_publisher);
    const auto& policy = planner_result.first;
    const std::map<std::string, double> planner_stats = planner_result.second;
    const double p_goal_reached = planner_stats.at("P(goal reached)");
    if (p_goal_reached >= options.goal_probability_threshold)
    {
        std::cout << "Planner reached goal, saving & loading policy" << std::endl;
        // Save the policy
        assert(uncertainty_planning_core::SaveSE3Policy(policy, options.planned_policy_file));
        const auto loaded_policy = uncertainty_planning_core::LoadSE3Policy(options.planned_policy_file);
        std::vector<uint8_t> policy_buffer;
        policy.SerializeSelf(policy_buffer);
        std::vector<uint8_t> loaded_policy_buffer;
        loaded_policy.SerializeSelf(loaded_policy_buffer);
        assert(policy_buffer.size() == loaded_policy_buffer.size());
        for (size_t idx = 0; idx > policy_buffer.size(); idx++)
        {
            const uint8_t policy_buffer_byte = policy_buffer[idx];
            const uint8_t loaded_policy_buffer_byte = loaded_policy_buffer[idx];
            assert(policy_buffer_byte == loaded_policy_buffer_byte);
        }
        assert(policy.GetRawPreviousIndexMap().size() == loaded_policy.GetRawPreviousIndexMap().size());
    }
    else
    {
        std::cout << "Planner failed to reach goal" << std::endl;
    }
    // Print out the results & save them to the log file
    const std::string log_results = "++++++++++\n" + PrettyPrint::PrettyPrint(options) + "\n" + PrettyPrint::PrettyPrint(extra_options) + "\nRESULTS:\n" + PrettyPrint::PrettyPrint(planner_stats, false, "\n");
    std::cout << "Planner results for " << options.num_particles << " particles:\n" << log_results << std::endl;
    std::ofstream log_file(options.planner_log_file, std::ios_base::out | std::ios_base::app);
    if (!log_file.is_open())
    {
        std::cerr << "\x1b[31;1m Unable to create folder/file to log to: " << options.planner_log_file << "\x1b[37m \n";
        throw std::invalid_argument( "Log filename must be write-openable" );
    }
    log_file << log_results << std::endl;
    log_file.close();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "se3_contact_planning_node");
    ros::NodeHandle nh;
    ROS_INFO("Starting SE(3) Contact Planning Node...");
    ros::Publisher display_debug_publisher = nh.advertise<visualization_msgs::MarkerArray>("uncertainty_planning_debug_display_markers", 1, true);
    plan(display_debug_publisher);
    return 0;
}
