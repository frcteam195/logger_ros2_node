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

#define NODE_NAME "logger_ros2_node"

std::string exec(const char* cmd)
{
	std::array<char, 128> buffer;
	std::string result;
	std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
	if (!pipe)
	{
		throw std::runtime_error("popen() failed!");
	}
	while (fgets(buffer.data(), buffer.size(), pipe.get()))
	{
		result += buffer.data();
	}
	return result;
}

class LocalNode : public rclcpp::Node
{
public:
    LocalNode() : rclcpp::Node(NODE_NAME)
    {
        robot_status_subscriber = this->create_subscription<ck_ros2_base_msgs_node::msg::RobotStatus>("/RobotStatus", 1, std::bind(&LocalNode::robot_status_callback, this, std::placeholders::_1));
    }

    ~LocalNode()
    {

    }


	void stop_ros_bag()
	{
		RCLCPP_INFO(this->get_logger(), "Stopping the recording");
		system("pkill -2 rosbag");
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
		const std::string DEFAULT_BAG_NAME = "/mnt/working/ros_log_";

		boost::posix_time::ptime my_posix_time = rclcpp_time_to_boost_time(this->get_clock()->now());
		std::string date_string = boost::posix_time::to_iso_extended_string(my_posix_time);

		std::stringstream s;
		s << "rosbag record --tcpnodelay -a -x '/MotorControl$|/MotorConfiguration$' --split --duration 5m --max-splits 1 --repeat-latched -O " << DEFAULT_BAG_NAME << date_string << ".bag &";

		system(s.str().c_str());

		RCLCPP_INFO(this->get_logger(), "Starting a recording at: %s", s.str().c_str());
	}

	void sync_fs()
	{
		system("sudo sync");
	}

private:
    rclcpp::Subscription<ck_ros2_base_msgs_node::msg::RobotStatus>::SharedPtr robot_status_subscriber;

	void robot_status_callback (const ck_ros2_base_msgs_node::msg::RobotStatus &msg)
	{
		static rclcpp::Time time_in_disabled = rclcpp::Time(0);

		if (msg.robot_state != ck_ros2_base_msgs_node::msg::RobotStatus::DISABLED)
		{
			time_in_disabled = this->get_clock()->now();
		}

		if (time_in_disabled != rclcpp::Time(0) && (this->get_clock()->now() - time_in_disabled) > rclcpp::Duration::from_seconds(10))
		{
			stop_ros_bag();
			sync_fs();
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
	node->sync_fs();
    rclcpp::shutdown();
    return 0;
}