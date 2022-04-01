// #include "ros/subscriber.h"
// #include "ros/time.h"

#include "offboarding.h"
#include "ros/time.h"

void OffBoarding::state_cb_(const mavros_msgs::State::ConstPtr& msg) {
	current_state_ = *msg;
}

void OffBoarding::twist_cb_(const geometry_msgs::Twist::ConstPtr& msg) {
	// If beat is fresh publish the received twist, otherwise skip it.
	if(is_beat_fresh()){
		if(current_goal_.type_mask == POSITION_CONTROL)
		{
			ROS_INFO("Switch to velocity control");
		}
		current_goal_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
		current_goal_.type_mask = velocity_mask;
		current_goal_.velocity.x = msg->linear.x;
		current_goal_.velocity.y = msg->linear.y;
		current_goal_.velocity.z = velocity_mask == VELOCITY2D_CONTROL?0:msg->linear.z;
		current_goal_.position.z = 1.5;
		current_goal_.yaw_rate = msg->angular.z;
		current_goal_.yaw_rate = msg->angular.z;
		lastTwistReceived_ = ros::Time::now();
	}
}

void OffBoarding::joy_cb_(const sensor_msgs::Joy::ConstPtr& msg) {
	if(msg->buttons[1] == 1) {
		a_prem_ = true;
		ROS_INFO("a is pressed");
	}

	if(msg->buttons[5] == 1)
		// When holding right trigger, accept velocity in Z
		velocity_mask = VELOCITY_CONTROL;
	else
		velocity_mask = VELOCITY2D_CONTROL;
}

void OffBoarding::remote_cb_(const std_msgs::Empty::ConstPtr& msg) {
	// update time for last beat
	// lastRemoteBeat_ = ros::Time().now();
	// set beat value through method
	set_beat();
}

bool OffBoarding::is_beat_fresh() {
	if( (ros::Time().now() - lastRemoteBeat_) < ros::Duration(1.0))
			return true;
	else
		return false;
}
	
void OffBoarding::set_beat() {
	lastRemoteBeat_ = ros::Time().now();
}
