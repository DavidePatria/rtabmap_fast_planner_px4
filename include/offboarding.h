#ifndef OFFBOARDING_H
#define OFFBOARDING_H

#include "ros/publisher.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
// for mavros services
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#define VELOCITY2D_CONTROL 0b011111000011
#define VELOCITY_CONTROL 0b011111000111
#define POSITION_CONTROL 0b101111111000

#include "takeoff_custom_srv_server.h"

class OffBoarding {
public:
	OffBoarding();
	~OffBoarding();

	// getter methods
	bool is_beat_fresh();
	bool is_request_old();
	bool is_offboard();
	bool is_armed();
	bool is_autoland();
	bool is_a_pressed();
	bool is_twist_old();
	bool is_connected();
	bool is_joystick_down();
	bool is_want_to_autoland();

	// various setter methods
	void set_request_time();
	void set_goal_vel_zero();
	void update_pose(const tf::StampedTransform &transf);
	void set_pos_goal(geometry_msgs::PoseStamped &pose);

	// set modes methods
	bool set_offboard();
	bool set_autoland();
	bool set_arm();
	bool set_disarm();


	void toggle_up_down(bool request);
	void go_autoland();

	ros::Publisher local_pos_pub;
	ros::Publisher vision_pos_pub;

	ros::ServiceClient arming_client;
	ros::ServiceClient command_client;
	ros::ServiceClient set_mode_client;

	mavros_msgs::PositionTarget current_goal;
	geometry_msgs::PoseStamped current_pose;

private:

	unsigned short velocity_mask_ = VELOCITY2D_CONTROL;

	// private messages for mavros services
	mavros_msgs::SetMode offb_set_mode_;
	mavros_msgs::SetMode autol_set_mode_;
	mavros_msgs::CommandBool arm_cmd_;
	mavros_msgs::CommandLong disarm_cmd_;

	// class where the custom takeoff service is defined
	TeichingOfServis takeoff_srv_;

	// state tracking variables
	bool was_flying_;
	bool donotprint_;
	bool a_prem_;
	bool want_to_autoland_;

	mavros_msgs::State current_state_;

	// ros::Time let_it_do_its_thing_;
	ros::Time last_request_;
	ros::Time last_twist_received_;
	ros::Time last_remote_beat_;

	ros::NodeHandle nh_;

	// callback from subscribers
	ros::Subscriber remote_sub_;
	ros::Subscriber state_sub_;
	ros::Subscriber twist_sub_;
	ros::Subscriber joy_sub_;

	void state_cb_(const mavros_msgs::State::ConstPtr& msg);
	void twist_cb_(const geometry_msgs::Twist::ConstPtr& msg);
	void joy_cb_(const sensor_msgs::Joy::ConstPtr& msg);
	void remote_cb_(const std_msgs::Empty::ConstPtr& msg);

	// pointer to member function to be passed to TeichingOfServis
	// which will be instantiated here
};

#endif
