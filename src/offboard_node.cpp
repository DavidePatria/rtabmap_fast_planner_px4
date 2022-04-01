/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include "ros/subscriber.h"
#include "ros/time.h"
#include "std_msgs/Empty.h"

#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf/transform_listener.h>

#include <std_msgs/Time.h>


/*
uint16 IGNORE_PX=1
uint16 IGNORE_PY=2
uint16 IGNORE_PZ=4
uint16 IGNORE_VX=8
uint16 IGNORE_VY=16
uint16 IGNORE_VZ=32
uint16 IGNORE_AFX=64
uint16 IGNORE_AFY=128
uint16 IGNORE_AFZ=256
uint16 FORCE=512
uint16 IGNORE_YAW=1024
uint16 IGNORE_YAW_RATE=2048
 */
#define VELOCITY2D_CONTROL 0b011111000011
#define VELOCITY_CONTROL 0b011111000111
#define POSITION_CONTROL 0b101111111000
unsigned short velocity_mask = VELOCITY2D_CONTROL;

//==============================================================================
// GLOBAL VARIBALES

// Variable to check is the drone has flew at least once.
// Since the drone begins in AUTO.LAND this is used to let it switch to OFFBOARD
// for the first time the programme is launched, because later the drone being
// in AUTO.LAND can solely mean that a manual activation has been done
bool wasFlying = false;
ros::Time letItDoItsThing;

mavros_msgs::PositionTarget current_goal;
ros::Time lastTwistReceived;
// keep track of beat from remote computer
ros::Time lastRemoteBeat;

bool donotprint = false;

//botton pressed? used to change state of the drone
bool a_prem;

//used for reorganisation  to multi-file format
mavros_msgs::State current_state;
//==============================================================================
// CALLABACK FUNCTIONS


void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

void twist_cb(const geometry_msgs::Twist::ConstPtr& msg){
	// If beat is fresh publish the received twist, otherwise skip it.
	if( (ros::Time().now() - lastRemoteBeat).toSec() < 1.0){
		if(current_goal.type_mask == POSITION_CONTROL)
		{
			ROS_INFO("Switch to velocity control");
		}
		current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
		current_goal.type_mask = velocity_mask;
		current_goal.velocity.x = msg->linear.x;
		current_goal.velocity.y = msg->linear.y;
		current_goal.velocity.z = velocity_mask == VELOCITY2D_CONTROL?0:msg->linear.z;
		current_goal.position.z = 1.5;
		current_goal.yaw_rate = msg->angular.z;
		current_goal.yaw_rate = msg->angular.z;
		lastTwistReceived = ros::Time::now();
	}
}

void joy_cb(const sensor_msgs::Joy::ConstPtr& msg){
	if(msg->buttons[1] == 1) {
		a_prem = true;
		ROS_INFO("a is pressed");
	}

	if(msg->buttons[5] == 1)
		// When holding right trigger, accept velocity in Z
		velocity_mask = VELOCITY_CONTROL;
	else
		velocity_mask = VELOCITY2D_CONTROL;
}

void remote_cb(const std_msgs::Empty::ConstPtr& msg) {
	// update time for last beat
	lastRemoteBeat = ros::Time().now();
}

//==============================================================================
// ANTI-CLUTTER FUNCTIONS
//

void updatePose(geometry_msgs::PoseStamped &pose, const tf::StampedTransform &vision) {
	pose.pose.position.x    = vision.getOrigin().x();
	pose.pose.position.y    = vision.getOrigin().y();
	pose.pose.position.z    = vision.getOrigin().z();
	pose.pose.orientation.x = vision.getRotation().x();
	pose.pose.orientation.y = vision.getRotation().y();
	pose.pose.orientation.z = vision.getRotation().z();
	pose.pose.orientation.w = vision.getRotation().w();
}

void setPosGoal(mavros_msgs::PositionTarget &goal, geometry_msgs::PoseStamped &pose ) {
	goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	goal.type_mask = POSITION_CONTROL;
	goal.position.x = pose.pose.position.x;
	goal.position.y = pose.pose.position.y;
	goal.position.z = 1.5;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "offboard_node");
	ros::NodeHandle nh;

	// ros::ServiceClient client = nh.serviceClient<rtabmap_drone_example::MakeTakeoff>

	ros::Subscriber remote_pub = nh.subscribe<std_msgs::Empty>
	("/remote_beat", 1, remote_cb);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
	("mavros/state", 10, state_cb);
	ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::Twist>
	("/cmd_vel", 1, twist_cb);
	ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>
	("/joy", 1, joy_cb);

	ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
	("mavros/setpoint_raw/local", 1);
	ros::Publisher vision_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
	("mavros/vision_pose/pose", 1);

	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
	("mavros/cmd/arming");
	ros::ServiceClient command_client = nh.serviceClient<mavros_msgs::CommandLong>
	("mavros/cmd/command");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
	("mavros/set_mode");


	//the setpoint publishing rate MUST be faster than 2Hz
	// this is 50, way faster
	ros::Rate rate(50.0);
	// set to the starting time before trying to change modes
	letItDoItsThing = ros::Time().now();

	// wait for FCU connection
	while(ros::ok() && !current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}

	// at first the drone will be activated but it won't move until the condition
	// on the remote computer is satisfied
	//


	// define a new message for setting au
	// let's try removing the takeoff after start and see if it creates problems
	mavros_msgs::SetMode offb_set_mode;
	mavros_msgs::SetMode autol_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	// autol_set_mode.request.custom_mode = "AUTO.LAND";
	//
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	mavros_msgs::CommandLong disarm_cmd;
	disarm_cmd.request.broadcast = false;
	disarm_cmd.request.command = 400;
	//disarm_cmd.request.param2 = 21196; // Kill no check landed

	ros::Time last_request = ros::Time::now();
	lastTwistReceived = ros::Time::now();

	tf::TransformListener listener;

	// ROS_INFO("Setting offboard mode... (5 seconds)");
	ros::spinOnce();

	if(!listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(5)))
	{
		ROS_ERROR("Cannot get current position between /map and /base_link");
		return -1;
	}


	// try{
	// 	tf::StampedTransform visionPoseTf;
	//
	// 	listener.lookupTransform("/map", "/base_link", ros::Time(0), visionPoseTf);
	//
	// 	//update currentPose
	// 	current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	// 	current_goal.type_mask = POSITION_CONTROL;
	// 	current_goal.position.x = visionPoseTf.getOrigin().x();
	// 	current_goal.position.y = visionPoseTf.getOrigin().y();
	// 	current_goal.position.z = 1.5;
	// 	current_goal.yaw = tf::getYaw(visionPoseTf.getRotation());
	// 	current_goal.velocity.x = 0;
	// 	current_goal.velocity.y = 0;
	// 	current_goal.velocity.z = 0;
	// 	current_goal.yaw_rate = 0;
	// 	current_goal.acceleration_or_force.x = 0;
	// 	current_goal.acceleration_or_force.y = 0;
	// 	current_goal.acceleration_or_force.z = 0;
	// 	ROS_INFO("Initial position=(%f,%f,%f) yaw=%f",
	// 			current_goal.position.x,
	// 			current_goal.position.y,
	// 			visionPoseTf.getOrigin().z(),
	// 			current_goal.yaw);
	// }
	// catch (tf::TransformException & ex){
	// 	ROS_ERROR("%s",ex.what());
	// 	return -1;
	// }

	//send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i){
		local_pos_pub.publish(current_goal);
		ros::spinOnce();
		rate.sleep();
	}

	geometry_msgs::PoseStamped current_pose;
	current_pose.header.frame_id = "map";

	ROS_INFO("entering while");

	while(ros::ok()){

		if(a_prem) {
			if( set_mode_client.call(offb_set_mode) &&
					offb_set_mode.response.mode_sent){
				ROS_INFO("Offboard enabled");
				ROS_INFO("Vehicle arming... (5 seconds)");
			}
		}

		// this try catch to send the position has to be done every cycle
		// regardless of the remote machine state otherwise px4 doesn't get
		// vision data and stops, so it is kept outside the main if as in the
		// original programme
		try{
			tf::StampedTransform visionPoseTf;
			listener.lookupTransform("/map", "/base_link", ros::Time(0), visionPoseTf);

			//update currentPose
			updatePose(current_pose, visionPoseTf);
		}
		catch (tf::TransformException & ex){
			ROS_ERROR("%s",ex.what());
		}

		// if(current_state.mode == "AUTO.LAND" && wasFlying == true)
		if(current_state.mode == "AUTO.LAND" && !a_prem ) {
			tf::StampedTransform visionPoseTf;
			ROS_INFO("Drone is in autolanding mode, skipping");
			updatePose(current_pose, visionPoseTf);
		}
		else {
			if( ros::Time().now() - lastRemoteBeat < ros::Duration(1) )	{
				donotprint = false;
				if( current_state.mode != "OFFBOARD" &&
						(ros::Time::now() - last_request > ros::Duration(5.0))){
					if( set_mode_client.call(offb_set_mode) &&
							offb_set_mode.response.mode_sent){
						ROS_INFO("Offboard enabled");
						ROS_INFO("Vehicle arming... (5 seconds)");
					}
					last_request = ros::Time::now();
				} else {
					if( !current_state.armed &&
							!(current_goal.velocity.z < -0.4 && current_goal.yaw_rate < -0.4) && // left joystick down-right
							(ros::Time::now() - last_request > ros::Duration(5.0))){
						if( arming_client.call(arm_cmd) &&
								arm_cmd.response.success){
							ROS_INFO("Vehicle armed");
							ROS_INFO("Take off at 1.5 meter... to position=(%f,%f,%f) yaw=%f",
									current_goal.position.x,
									current_goal.position.y,
									current_goal.position.z,
									current_goal.yaw);
						}
						last_request = ros::Time::now();
						//attempt to set the variable, might not be the right spot
						wasFlying = true;
					}
					else if(current_goal.velocity.z < -0.4 && current_goal.yaw_rate < -0.4 && // left joystick down-right
							(ros::Time::now() - last_request > ros::Duration(5.0))){
						if( command_client.call(disarm_cmd) &&
								disarm_cmd.response.success){
							ROS_INFO("Vehicle disarmed");
							ros::shutdown();
						}
						else
						{
							ROS_INFO("Disarming failed! Still in flight?");
						}
						last_request = ros::Time::now();
					}
				}

				current_goal.header.stamp = ros::Time::now();

				if(current_goal.header.stamp.toSec() - lastTwistReceived.toSec() > 1 and current_goal.type_mask != POSITION_CONTROL)
				{
					//switch to position mode with last position if twist is not received for more than 1 sec

					setPosGoal( current_goal, current_pose);

					tfScalar yaw, pitch, roll;
					tf::Matrix3x3 mat(tf::Quaternion(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w));
					mat.getEulerYPR(yaw, pitch, roll);
					current_goal.yaw = yaw;
					ROS_INFO("Switch to position control (x=%f, y=%f, z=%f, yaw=%f)",
							current_goal.position.x, current_goal.position.y, current_goal.position.z, current_goal.yaw);
				}

				current_pose.header.stamp = current_goal.header.stamp;
				local_pos_pub.publish(current_goal);

				// Vision pose should be published at a steady
				// frame rate so that EKF from px4 stays stable
				vision_pos_pub.publish(current_pose);
			}
			// supposedly this else is if((ros::Time()::now() - lastRemoteBeat).toSec()>1.0)
			// so if the message is too old
			else {

				// if the programme starts without a beat from the remote machine
				// it won't take off at first, since is goes directly into this
				// else that doesn't arm/OFFBOARD the drone
				if(!donotprint){
					ROS_INFO("Remote beat topic not received");
					// print once per case
					donotprint = true;
				}


				current_goal.header.stamp = ros::Time::now();

				if(current_goal.header.stamp.toSec() - lastTwistReceived.toSec() > 1 and current_goal.type_mask != POSITION_CONTROL)
				{
					//switch to position mode with last position if twist is not received for more than 1 sec

					current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
					current_goal.type_mask = POSITION_CONTROL;
					current_goal.position.x = current_pose.pose.position.x;
					current_goal.position.y = current_pose.pose.position.y;
					current_goal.position.z = 1.5;
					current_goal.yaw = current_pose.pose.orientation.z;

					tfScalar yaw, pitch, roll;
					tf::Matrix3x3 mat(tf::Quaternion(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w));
					mat.getEulerYPR(yaw, pitch, roll);
					current_goal.yaw = yaw;
					ROS_INFO("Switch to position control (x=%f, y=%f, z=%f, yaw=%f)",
							current_goal.position.x, current_goal.position.y, current_goal.position.z, current_goal.yaw);
				}

				current_pose.header.stamp = current_goal.header.stamp;
				local_pos_pub.publish(current_goal);

				// Vision pose should be published at a steady
				// frame rate so that EKF from px4 stays stable
				vision_pos_pub.publish(current_pose);
			}
		}

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

