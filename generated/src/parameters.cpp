#include "logger_ros2_node/parameters.hpp"

parameters_t Parameters;

void load_parameters(rclcpp::Node* node)
{
    node->declare_parameter("log_path", rclcpp::PARAMETER_STRING);
    node->declare_parameter("log_prefix", rclcpp::PARAMETER_STRING);
    node->declare_parameter("disabled_log_restart_time", rclcpp::PARAMETER_INTEGER);

    Parameters.log_path = node->get_parameter("log_path");
    Parameters.log_prefix = node->get_parameter("log_prefix");
    Parameters.disabled_log_restart_time = node->get_parameter("disabled_log_restart_time");
}

ParameterizedNode::ParameterizedNode(std::string node_name) : rclcpp::Node(node_name)
{
    load_parameters(this);
}