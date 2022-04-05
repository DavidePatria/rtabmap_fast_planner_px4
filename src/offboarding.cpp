// #include "ros/subscriber.h"
// #include "ros/time.h"

#include "ros/init.h"
#include <offboarding.h>
#include <ostream>
#include <string>
#include <iostream>
// #include "ros/time.h"

// initiate subscribers in constructor
OffBoarding::OffBoarding():nh_("") {

	ROS_INFO("Class instantiatied. Creating subscribers");
	// short queue. to 1. might cause problems bu the topic type allows it
	remote_sub_ = nh_.subscribe("/remote_beat", 1,
		&OffBoarding::remote_cb_, this);
	state_sub_ = nh_.subscribe("/mavros/state", 10,
		&OffBoarding::state_cb_, this);
	twist_sub_ = nh_.subscribe("/cmd_vel", 10,
		&OffBoarding::twist_cb_, this);
	joy_sub_ = nh_.subscribe("/joy", 10,
		&OffBoarding::joy_cb_, this);
	
	arming_client = nh_.serviceClient<mavros_msgs::CommandBool>
		("mavros/cmd/arming");
	command_client = nh_.serviceClient<mavros_msgs::CommandLong>
		("mavros/cmd/command");
	set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>
		("mavros/set_mode");

	local_pos_pub = nh_.advertise<mavros_msgs::PositionTarget>
		("mavros/setpoint_raw/local", 1);
	vision_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>
		("mavros/vision_pose/pose", 1);

	// initialize here, imitating the orignal programme
	last_twist_received_ = ros::Time().now();

	set_request_time();

	// initialize variables in the constructor
	donotprint_ = false;
	a_prem_ = false;
	
	ROS_INFO("Init Finished");
}

OffBoarding::~OffBoarding() { ros::shutdown(); }

//==============================================================================
// other functions

// simply check is the private attribute representing last time the callback
// triggered from remote computer has happened
bool OffBoarding::is_beat_fresh() {
	bool cond = (ros::Time().now() - last_remote_beat_) < ros::Duration(1.0);
	// ROS_INFO("Beat is fresh as fish");
	return cond;
}
	
// check if last request is at least n seconds old
bool OffBoarding::is_request_old() {
	bool cond = (ros::Time::now() - last_request_) > ros::Duration(5.0);
	// ROS_INFO("request is old = %s", cond ? "true":"false" );
	return cond;
}

// check if the field mode is the custom mode desired (that is "OFFBOARD")
bool OffBoarding::is_offboard() {
	bool cond = current_state_.mode == "OFFBOARD";
	// ROS_INFO("mode is offboard = %s", cond ? "true":"false" );
	return cond;
}	

// field armed is a bool, so it doesn't require checking for string as before
bool OffBoarding::is_armed() {
	// ROS_INFO("Is armed: %s", current_state_.armed ? "true":"false");
	// direct return of state value
	return current_state_.armed;
}

// same type of check as the previous one for offboard
bool OffBoarding::is_autoland() {
	return current_state_.mode == "AUTO.LAND";
}	

bool OffBoarding::is_a_pressed() {
	// ROS_INFO("a is pressed");
	return a_prem_;
}

// if last twist if old switch to position control.
// the time variable is set in cmd_vel callback
// the twist age is checked against goal stamp
bool OffBoarding::is_twist_old() {
	bool condition = (current_goal.header.stamp - last_twist_received_ > ros::Duration(1.0));
	// ROS_INFO("Is twist old: %s", condition ? "true":"false" );
	return condition;
}

// check connetion
bool OffBoarding::is_connected() {
	// ROS_INFO("Is connected: %s", current_state_.connected ? "true":"false");
	// direct return of state value
	return current_state_.connected;
}

// check if left joystick is down right
// this joystick moves the drone by setting the goal)
bool OffBoarding::is_joystick_down() {
	bool cond = current_goal.velocity.z < -0.4 && current_goal.yaw_rate < -0.4;
	return cond;
}

void OffBoarding::set_request_time() {
	// ROS_INFO("setted request time");
	last_request_ = ros::Time().now();
}

void OffBoarding::set_goal_vel_zero() {
	current_goal.velocity.x = 0;
	current_goal.velocity.y = 0;
	current_goal.velocity.z = 0;
	current_goal.yaw_rate = 0;
	current_goal.acceleration_or_force.x = 0;
	current_goal.acceleration_or_force.y = 0;
	current_goal.acceleration_or_force.z = 0;
}

// set the internal current_pose variable to the transform gotten usin an
// external method
void OffBoarding::update_pose(const tf::StampedTransform &transf) {
	current_pose.pose.position.x    = transf.getOrigin().x();
	current_pose.pose.position.y    = transf.getOrigin().y();
	current_pose.pose.position.z    = transf.getOrigin().z();
	current_pose.pose.orientation.x = transf.getRotation().x();
	current_pose.pose.orientation.y = transf.getRotation().y();
	current_pose.pose.orientation.z = transf.getRotation().z();
	current_pose.pose.orientation.w = transf.getRotation().w();
}

// get a position and set the internal goal variable to the supplied pose
void OffBoarding::set_pos_goal(geometry_msgs::PoseStamped &pose) {
	current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	current_goal.type_mask = POSITION_CONTROL;
	current_goal.position.x = pose.pose.position.x;
	current_goal.position.y = pose.pose.position.y;
	current_goal.position.z = 1.5;
}


//==============================================================================
// callbacks
void OffBoarding::state_cb_(const mavros_msgs::State::ConstPtr& msg) {
	current_state_ = *msg;
	// std::cout << "mode is: " << msg->mode << std::endl;
}


void OffBoarding::twist_cb_(const geometry_msgs::Twist::ConstPtr& msg) {
	// If beat is fresh publish the received twist, otherwise skip it.
	if( is_beat_fresh() ){
		if(current_goal.type_mask == POSITION_CONTROL)
		{
			ROS_INFO("Switch to velocity control");
		}
		current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
		current_goal.type_mask = velocity_mask_;
		current_goal.velocity.x = msg->linear.x;
		current_goal.velocity.y = msg->linear.y;
		current_goal.velocity.z = velocity_mask_ == VELOCITY2D_CONTROL?0:msg->linear.z;
		current_goal.position.z = 1.5;
		current_goal.yaw_rate = msg->angular.z;
		current_goal.yaw_rate = msg->angular.z;

		last_twist_received_ = ros::Time::now();
	}
}

void OffBoarding::joy_cb_(const sensor_msgs::Joy::ConstPtr& msg) {
	if(msg->buttons[0] == 1) {
		a_prem_ = true;
		// ROS_INFO("a is pressed");
	}

	if(msg->buttons[1] == 1) {
		a_prem_ = false;
		// ROS_INFO("a is pressed");
	}


	if(msg->buttons[5] == 1)
		// When holding right trigger, accept velocity in Z
		velocity_mask_ = VELOCITY_CONTROL;
	else
		velocity_mask_ = VELOCITY2D_CONTROL;
}

void OffBoarding::remote_cb_(const std_msgs::Empty::ConstPtr& msg) {
	// update last beat time 
	// ROS_INFO("beat updated after %f secs, or whatever", last_remote_beat_.toSec());
	last_remote_beat_ = ros::Time().now();
}

