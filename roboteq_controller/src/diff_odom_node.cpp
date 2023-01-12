#include "roboteq_controller/diff_odom_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


Odometry_calc::Odometry_calc(const rclcpp::NodeOptions &options): 
    Node("Odometry_calc", options),
    odom_broadcaster(this),
    encoder_min(-65536), encoder_max(65536),
	encoder_low_wrap(((encoder_max - encoder_min) * 0.3) + encoder_min),
    encoder_high_wrap(((encoder_max - encoder_min) * 0.7) + encoder_min),
    prev_lencoder(0), prev_rencoder(0),
    lmult(0), rmult(0),
    left(0), right(0), rate(10),
	t_delta(0,floor(1000000000.0 / rate)),
    last_time(this->now()),
    enc_left(10), enc_right(0),	ticks_meter(50),
    base_width(0.3), dx(0),	dr(0),
    x_final(0),y_final(0),
    theta_final(0)
  	
{
    RCLCPP_INFO(this->get_logger(), "Started odometry computing node");

    wheel_sub_ = this->create_subscription<roboteq_interfaces::msg::ChannelValues>(
      "/hall_count",1000, std::bind(&Odometry_calc::encoderCb, this, _1));
  	odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom1", 50);


    timer_ = this->create_wall_timer(
        100ms, std::bind(&Odometry_calc::update, this)); // if rate is 10 then its 100ms cycle

}

void Odometry_calc::update()    {

    auto now = this->now();
	
	//rclcpp::Duration elapsed;
    double elapsed;
	double d_left, d_right, d, th,x,y;

    elapsed = (now - last_time).nanoseconds()*1000000; // time in miliseconds 
    //RCLCPP_INFO_STREAM(this->get_logger(),"elapsed =" << elapsed);

    
    if(enc_left == 0){
        d_left = 0;
        d_right = 0;
    }
    else{
        d_left = (left - enc_left) / ( ticks_meter);
        d_right = (right - enc_right) / ( ticks_meter);
    }
    
    enc_left = left;
    //RCLCPP_INFO_STREAM(this->get_logger(),left);
    enc_right = right;

    d = (d_left + d_right ) / 2.0;

    RCLCPP_INFO_STREAM(this->get_logger(), d_left << " : " << d_right);


    th = ( d_right - d_left ) / base_width;
    
    dx = d /elapsed;

    dr = th / elapsed;


    if ( d != 0){
        x = cos( th ) * d;
        //RCLCPP_INFO_STREAM(this->get_logger(),x);
        y = -sin( th ) * d;
        // calculate the final position of the robot
        x_final = x_final + ( cos( theta_final ) * x - sin( theta_final ) * y );
        y_final = y_final + ( sin( theta_final ) * x + cos( theta_final ) * y );
    }

    if( th != 0)
        theta_final = theta_final + th;

    geometry_msgs::msg::Quaternion odom_quat ;

    odom_quat.x = 0.0;
    odom_quat.y = 0.0;
    odom_quat.z = 0.0;

    odom_quat.z = sin( theta_final / 2 );	
    odom_quat.w = cos( theta_final / 2 );

    //first, we'll publish the transform over tf
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = now;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x_final;
    odom_trans.transform.translation.y = y_final;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x_final;
    odom.pose.pose.position.y = y_final;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = dx;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = dr;

    //publish the message
    odom_pub_->publish(odom);

    last_time = now;
//  RCLCPP_INFO_STREAM(this->get_logger(),"dx =" << x_final);
//	RCLCPP_INFO_STREAM(this->get_logger(),"dy =" << y_final);
}

void Odometry_calc::encoderCb(const roboteq_interfaces::msg::ChannelValues& ticks){
    RCLCPP_INFO_STREAM(this->get_logger(),"Left tick = " << ticks.value[1] << ", Right tick = " << ticks.value[2]);
	double encl = ticks.value[1];
	if((encl < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap))
	    lmult = lmult + 1;
	else if((encl > encoder_high_wrap) && (prev_lencoder < encoder_low_wrap))
		lmult = lmult - 1;
	left = 1.0 * (encl + lmult * (encoder_max - encoder_min ));
	prev_lencoder = encl;

	double encr = ticks.value[2];
	if((encr < encoder_low_wrap) && (prev_rencoder > encoder_high_wrap))
        rmult = rmult + 1;
	if((encr > encoder_high_wrap) && (prev_rencoder < encoder_low_wrap))
		rmult = rmult - 1;
	right = 1.0 * (encr + rmult * (encoder_max - encoder_min ));
	prev_rencoder = encr;
    RCLCPP_INFO_STREAM(this->get_logger(),"Left =  " << left << ", Right  = " << right);
}


int main(int argc, char **argv)

{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Odometry_calc>());
    rclcpp::shutdown();
    return 0;
}