#pragma once

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/classification.hpp>


#include <iostream>
#include <sstream>
#include <typeinfo>
#include <cassert>
#include <mutex>
#include <math.h>
#include <chrono>
#include <memory>
#include <vector>
#include <map>

#include <serial/serial.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "roboteq_interfaces/msg/channel_values.hpp"
#include "roboteq_interfaces/srv/config.hpp"
#include "roboteq_interfaces/srv/maintenance.hpp"
#include "roboteq_interfaces/srv/command.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

class RoboteqDriver : public rclcpp::Node
{
public:
	explicit RoboteqDriver(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

	~RoboteqDriver(){
		if (ser_.isOpen()){
			ser_.close();
		}
	}

private:
	
	// Serial
	serial::Serial 			ser_;
	std::string 			serial_port_;
	int32_t 				baudrate_;

	// Pub & Sub
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr 		cmd_vel_sub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr 		serial_read_pub_;

	std::vector<rclcpp::Publisher<roboteq_interfaces::msg::ChannelValues>::SharedPtr>  query_pub_;
	rclcpp::TimerBase::SharedPtr 				timer_pub_;

	// services 
	std::shared_ptr<rclcpp::Service<roboteq_interfaces::srv::Maintenance>> maintenance_srv_;
	std::shared_ptr<rclcpp::Service<roboteq_interfaces::srv::Config>> 	   config_srv_;
	std::shared_ptr<rclcpp::Service<roboteq_interfaces::srv::Command>> 	   command_srv_;

	bool 					closed_loop_,
							diff_drive_mode_;

	double 					wheel_circumference_,
							track_width_,
							max_rpm_;

	std::string 			cmd_vel_topic_;

	// queries
	std::map<std::string, std::string>	queries_;

	int frequency_;
	std::mutex 				locker;

	void declare();
	void init();

	void cmdSetup();
	void cmdVelCallback(const geometry_msgs::msg::Twist &);
	void powerCmdCallback(const geometry_msgs::msg::Twist &);

	
	bool configService(roboteq_interfaces::srv::Config_Request::SharedPtr request, roboteq_interfaces::srv::Config_Response::SharedPtr response);
    bool commandService(roboteq_interfaces::srv::Command_Request::SharedPtr request, roboteq_interfaces::srv::Command_Response::SharedPtr response);
	bool maintenanceService(roboteq_interfaces::srv::Maintenance_Request::SharedPtr request, roboteq_interfaces::srv::Maintenance_Response::SharedPtr response);
	void initializeServices();
	void run();

	void queryCallback();

};