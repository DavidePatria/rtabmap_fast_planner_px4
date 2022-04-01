#ifndef OFFBOARDING_H
#define OFFBOARDING_H

#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>


#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#define VELOCITY2D_CONTROL 0b011111000011
#define VELOCITY_CONTROL 0b011111000111
#define POSITION_CONTROL 0b101111111000

class OffBoarding {
public:
	OffBoarding();
	bool is_beat_fresh();
	void set_beat();

private:

	unsigned short velocity_mask_ = VELOCITY2D_CONTROL;

	// state tracking variables
	bool was_flying_;
	ros::Time letItDoItsThing_;
	mavros_msgs::PositionTarget current_goal_;
	mavros_msgs::State current_state_;
	ros::Time lastTwistReceived_;
	ros::Time lastRemoteBeat_;
	bool donotprint_ = false;
	bool a_prem_;


	ros::NodeHandle nh_;

	ros::Subscriber remote_pub_;
	ros::Subscriber state_pub_;
	ros::Subscriber twist_sub_;
	ros::Subscriber joy_sub_;

	void state_cb_(const mavros_msgs::State::ConstPtr& msg);
	void twist_cb_(const geometry_msgs::Twist::ConstPtr& msg);
	void joy_cb_(const sensor_msgs::Joy::ConstPtr& msg);
	void remote_cb_(const std_msgs::Empty::ConstPtr& msg);
};

#endif
