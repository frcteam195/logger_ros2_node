#include "logger_ros2_node/logger_ros2_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <thread>
#include <string>
#include <mutex>
#include <atomic>
#include <sys/stat.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <array>

#include <ck_ros2_base_msgs_node/msg/robot_status.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/process.hpp>

#include "ck_utilities_ros2_node/node_handle.hpp"

#define NODE_NAME "logger_ros2_node"

class LocalNode : public ParameterizedNode
{
public:
    LocalNode() : ParameterizedNode(NODE_NAME)
    {
        robot_status_subscriber = this->create_subscription<ck_ros2_base_msgs_node::msg::RobotStatus>("/RobotStatus", 1, std::bind(&LocalNode::robot_status_callback, this, std::placeholders::_1));

		std::stringstream tmp_file_name;
		tmp_file_name << Parameters.log_path << "/" << Parameters.log_prefix;
		MCAP_file_name = tmp_file_name.str();
    }

    ~LocalNode()
    {

    }


	void stop_ros_bag()
	{
		RCLCPP_INFO(this->get_logger(), "Stopping the recording...");
		pid_t pid = log_process.id ();
		kill (pid, SIGINT);
		log_process.join();
		system("sudo sync");
		RCLCPP_INFO(this->get_logger(), "Stopped the recording!");
	}

	boost::posix_time::ptime rclcpp_time_to_boost_time(rclcpp::Time input_time)
	{
		#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
			return boost::posix_time::from_time_t(input_time.seconds()) + boost::posix_time::nanoseconds(input_time.nanoseconds());
		#else
			return boost::posix_time::from_time_t(input_time.seconds()) + boost::posix_time::microseconds((rcl_time_point_value_t)(input_time.nanoseconds()/1000.0));
		#endif
	}

	void start_ros_bag()
	{
		if (!log_process.running())
		{
			boost::posix_time::ptime my_posix_time = rclcpp_time_to_boost_time(this->get_clock()->now());
			std::string date_string = boost::posix_time::to_iso_extended_string(my_posix_time);

			std::stringstream log_command;
			log_command << "bag record -s mcap --tcpnodelay -a -x '/MotorControl$|/MotorConfiguration$' --split --duration 5m --max-splits 1 --repeat-latched -O " << MCAP_file_name << date_string << ".mcap --storage-preset-profile fastwrite";

			log_process = boost::process::child(boost::process::search_path("ros2"), log_command.str());

			RCLCPP_INFO(this->get_logger(), "Starting a recording at: %s", log_command.str().c_str());
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to start log! ROS2 bag is already running...");
		}
	}

private:
    rclcpp::Subscription<ck_ros2_base_msgs_node::msg::RobotStatus>::SharedPtr robot_status_subscriber;

	boost::process::child log_process;

	std::string MCAP_file_name;

	void robot_status_callback (const ck_ros2_base_msgs_node::msg::RobotStatus &msg)
	{
		static rclcpp::Time time_in_disabled = rclcpp::Time(0);

		if (msg.robot_state != ck_ros2_base_msgs_node::msg::RobotStatus::DISABLED)
		{
			time_in_disabled = this->get_clock()->now();
		}

		if (time_in_disabled != rclcpp::Time(0) && (this->get_clock()->now() - time_in_disabled) > rclcpp::Duration::from_seconds(Parameters.disabled_log_restart_time))
		{
			stop_ros_bag();
			start_ros_bag();
			time_in_disabled = rclcpp::Time(0);
		}
	}

};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalNode>();
	node->start_ros_bag();
    rclcpp::spin(node);
	node->stop_ros_bag();
    rclcpp::shutdown();
    return 0;
}
