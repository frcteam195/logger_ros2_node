#pragma once
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <map>

typedef struct parameters
{
    rclcpp::Parameter log_path;
    rclcpp::Parameter log_prefix;
    rclcpp::Parameter disabled_log_restart_time;
} parameters_t;
extern parameters_t Parameters;
void load_parameters(rclcpp::Node* node);

class ParameterizedNode : public rclcpp::Node
{
public:
    ParameterizedNode(std::string node_name);
};
