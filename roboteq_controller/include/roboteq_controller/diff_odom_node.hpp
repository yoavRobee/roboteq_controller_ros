#pragma once

#include <math.h>
#include <limits>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/int16.hpp"

#include "tf2_ros/transform_broadcaster.h"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "roboteq_interfaces/msg/channel_values.hpp"

class Odometry_calc  : public rclcpp::Node
{

    public:
        explicit Odometry_calc(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        void run();


    private:

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Subscription<roboteq_interfaces::msg::ChannelValues>::SharedPtr wheel_sub_;

        tf2_ros::TransformBroadcaster odom_broadcaster;

        //Encoder related variables
        double encoder_min;
        double encoder_max;

        double encoder_low_wrap;
        double encoder_high_wrap;

        double prev_lencoder;
        double prev_rencoder;

        double lmult;
        double rmult;

        double left;
        double right;

        double rate;

        rclcpp::Duration t_delta;

        rclcpp::Time last_time;


        double enc_left ;

        double enc_right;

        double ticks_meter;

        double base_width;

        double dx;

        double dr;

        double x_final,y_final, theta_final;


        void encoderCb(const roboteq_interfaces::msg::ChannelValues& ticks);

        void update();
};