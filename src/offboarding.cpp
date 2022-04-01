// #include "ros/subscriber.h"
// #include "ros/time.h"

#include "ros/init.h"
#include <offboarding.h>
#include <string>
#include <iostream>
// #include "ros/time.h"

// initiate subscribers in constructor
OffBoarding::OffBoarding():nh_("") {

	ROS_INFO("Class instantiatied. Creatind subscribers");
	// short queue. to 1. might cause problems bu the topic type allows it
	remote_sub_ = nh_.subscribe("/remote/beat", 1,
						   &OffBoarding::remote_cb_, this);
	state_sub_ = nh_.subscribe("/mavros/state", 10,
						   &OffBoarding::state_cb_, this);
	twist_sub_ = nh_.subscribe("/cmd_vel", 10,
						   &OffBoarding::remote_cb_, this);
	joy_sub_ = nh_.subscribe("/joy", 10,
						   &OffBoarding::joy_cb_, this);
	
	local_pos_pub = nh_.advertise<mavros_msgs::PositionTarget>
	("mavros/setpoint_raw/local", 1);
	vision_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>
	("mavros/vision_pose/pose", 1);
	
	ROS_INFO("Init Finished");
}

OffBoarding::~OffBoarding() { ros::shutdown(); }

//==============================================================================
// other functions

// simply check is the private attribute representing last time the callback
// triggered from remote computer has happened
bool OffBoarding::is_beat_fresh() {
	if( (ros::Time().now() - last_remote_beat_) < ros::Duration(1.0))
		return true;
	else
		return false;
}
	
// check if last request is at least n seconds old
bool OffBoarding::is_request_old() {
	if( (ros::Time::now() - last_request_ < ros::Duration(5.0)) )
		return false;
	else
		return true;
}

bool OffBoarding::is_offboard() {
	if(current_state_.mode == "OFFBOARD")
		return true;
	else 
		return false;
}	

bool OffBoarding::is_armed() {
	if(current_state_.armed) 
		return true;
	else
		return false;
}

bool OffBoarding::is_autoland() {
	if(current_state_.mode == "AUTO.LAND")
		return true;
	else 
		return false;
}	

// if last twist if old switch to position control.
// the time variable is set in cmd_vel callback
// the twist age is checked against goal stamp
bool OffBoarding::is_twist_old() {
	if( (current_goal_.header.stamp - last_twist_received_ > ros::Duration(1.0)) )
		return false;
	else
		return true;
}

void OffBoarding::set_beat() {
	last_remote_beat_ = ros::Time().now();
}

void OffBoarding::set_request_time() {
	last_request_ = ros::Time().now();
}

// !!!!
// void OffBoarding::updatePose()


//==============================================================================
// callbacks
void OffBoarding::state_cb_(const mavros_msgs::State::ConstPtr& msg) {
	current_state_ = *msg;
}


void OffBoarding::twist_cb_(const geometry_msgs::Twist::ConstPtr& msg) {
	// If beat is fresh publish the received twist, otherwise skip it.
	if( is_beat_fresh() ){
		if(current_goal_.type_mask == POSITION_CONTROL)
		{
			ROS_INFO("Switch to velocity control");
		}
		current_goal_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
		current_goal_.type_mask = velocity_mask_;
		current_goal_.velocity.x = msg->linear.x;
		current_goal_.velocity.y = msg->linear.y;
		current_goal_.velocity.z = velocity_mask_ == VELOCITY2D_CONTROL?0:msg->linear.z;
		current_goal_.position.z = 1.5;
		current_goal_.yaw_rate = msg->angular.z;
		current_goal_.yaw_rate = msg->angular.z;
		last_twist_received_ = ros::Time::now();
	}
}

void OffBoarding::joy_cb_(const sensor_msgs::Joy::ConstPtr& msg) {
	if(msg->buttons[1] == 1) {
		a_prem_ = true;
		ROS_INFO("a is pressed");
	}

	if(msg->buttons[5] == 1)
		// When holding right trigger, accept velocity in Z
		velocity_mask_ = VELOCITY_CONTROL;
	else
		velocity_mask_ = VELOCITY2D_CONTROL;
}

void OffBoarding::remote_cb_(const std_msgs::Empty::ConstPtr& msg) {
	// update time for last beat
	// lastRemoteBeat_ = ros::Time().now();
	// set beat value through method
	set_beat();
}

