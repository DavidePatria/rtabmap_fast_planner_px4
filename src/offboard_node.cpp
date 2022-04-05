/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include "offboarding.h"

#include <ros/ros.h>

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

#define VELOCITY2D_CONTROL 0b011111000011
#define VELOCITY_CONTROL 0b011111000111
#define POSITION_CONTROL 0b101111111000
unsigned short velocity_mask = VELOCITY2D_CONTROL;

bool donotprint = false;

int main(int argc, char **argv)
{
	ROS_INFO("about to start, get ready!");
	ros::init(argc, argv, "offboard_node");
	// ros::NodeHandle nh;

	// ros::ServiceClient client = nh.serviceClient<rtabmap_drone_example::MakeTakeoff>

	// ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
	// ("mavros/cmd/arming");
	// ros::ServiceClient command_client = nh.serviceClient<mavros_msgs::CommandLong>
	// ("mavros/cmd/command");
	// ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
	// ("mavros/set_mode");
	
	OffBoarding offb;
	//the setpoint publishing rate MUST be faster than 2Hz
	// this is 50, way faster
	ros::Rate rate(50.0);
	// set to the starting time before trying to change modes
	// letItDoItsThing = ros::Time().now();

	// wait for FCU connection
	while(ros::ok() && !offb.is_connected()){
		ROS_INFO("not connected yet!");
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

	// ros::Time last_request = ros::Time::now();
	// lastTwistReceived = ros::Time::now();

	tf::TransformListener listener;

	// ROS_INFO("Setting offboard mode... (5 seconds)");
	ros::spinOnce();

	if(!listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(5)))
	{
		ROS_ERROR("Cannot get current position between /map and /base_link");
		return -1;
	}


	// ==================o===========================================================
	// for now manually set the goal to arm the drone.
	// setting it to the right values is required for it to work properly
	try{
		tf::StampedTransform visionPoseTf;

		listener.lookupTransform("/map", "/base_link", ros::Time(0), visionPoseTf);

		//update currentPose
		offb.current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
		offb.current_goal.type_mask = POSITION_CONTROL;
		offb.current_goal.position.x = visionPoseTf.getOrigin().x();
		offb.current_goal.position.y = visionPoseTf.getOrigin().y();
		offb.current_goal.position.z = 1.5;
		offb.current_goal.yaw = tf::getYaw(visionPoseTf.getRotation());

		offb.set_goal_vel_zero();

		ROS_INFO("Initial position=(%f,%f,%f) yaw=%f",
				offb.current_goal.position.x,
				offb.current_goal.position.y,
				visionPoseTf.getOrigin().z(),
				offb.current_goal.yaw);
	}
	catch (tf::TransformException & ex){
		ROS_ERROR("%s",ex.what());
		return -1;
	}
	// =============================================================================

	ROS_INFO("about to send setpoints");
	//send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i){
		offb.local_pos_pub.publish(offb.current_goal);
		// local_pos_pub.publish(current_goal);
		ros::spinOnce();
		rate.sleep();
	}

	// geometry_msgs::PoseStamped offboarding.current_pose;
	offb.current_pose.header.frame_id = "map";

	ROS_INFO("entering while");

	while(ros::ok()){

		if(offb.is_a_pressed()) {
			if( offb.set_mode_client.call(offb_set_mode) &&
					offb_set_mode.response.mode_sent &&
					offb.is_request_old() ) {
				ROS_INFO("Offboard enabled");
				ROS_INFO("Vehicle arming... (5 seconds)");
				offb.set_request_time();

				// messing around with retake-off because it doesn't work.
				// try sending points again
				for(int i = 100; ros::ok() && i > 0; --i){
					offb.current_goal.position.z = 1.5;
					offb.local_pos_pub.publish(offb.current_goal);
					// local_pos_pub.publish(current_goal);
					ros::spinOnce();
					rate.sleep();
				}
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
			offb.update_pose(visionPoseTf);
		}
		catch (tf::TransformException & ex){
			ROS_ERROR("%s",ex.what());
		}

		// if(current_state.mode == "AUTO.LAND" && wasFlying == true)
		if(offb.is_autoland() && !offb.is_a_pressed()) {
			tf::StampedTransform visionPoseTf;
			ROS_INFO("Drone is in autolanding mode, skipping");
			offb.update_pose(visionPoseTf);
		}
		else {
			if(offb.is_beat_fresh()) {
				donotprint = false;
				if(!offb.is_offboard() && offb.is_request_old()){
					if( offb.set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
						ROS_INFO("Offboard enabled");
						ROS_INFO("Vehicle arming... (5 seconds)");
					}
					offb.set_request_time();
				} else {
					if(!offb.is_armed() && !offb.is_joystick_down() && offb.is_request_old()){
						if( offb.arming_client.call(arm_cmd) && arm_cmd.response.success){
							ROS_INFO("Vehicle armed");
							ROS_INFO("Take off at 1.5 meter... to position=(%f,%f,%f) yaw=%f",
									offb.current_goal.position.x,
									offb.current_goal.position.y,
									offb.current_goal.position.z,
									offb.current_goal.yaw);
						}
						offb.set_request_time();
						//attempt to set the variable, might not be the right spot
					}
					else if(offb.is_joystick_down() && offb.is_request_old()){
						if( offb.command_client.call(disarm_cmd) && disarm_cmd.response.success){
							ROS_INFO("Vehicle disarmed");
							ros::shutdown();
						} else {
							ROS_INFO("Disarming failed! Still in flight?");
						}
						offb.set_request_time();
					}
				}

				offb.current_goal.header.stamp = ros::Time::now();

				// note about this condition: it could seem strange checking on the header just after the
				// line that sets it but if the joystick is moved the goals are not published hence the check on the timestamp.
				if( offb.is_twist_old() && offb.current_goal.type_mask != POSITION_CONTROL) {
					//switch to position mode with last position if twist is not received for more than 1 sec

					offb.set_pos_goal(offb.current_pose);

					tfScalar yaw, pitch, roll;
					tf::Matrix3x3 mat(tf::Quaternion(offb.current_pose.pose.orientation.x, offb.current_pose.pose.orientation.y, offb.current_pose.pose.orientation.z, offb.current_pose.pose.orientation.w));
					mat.getEulerYPR(yaw, pitch, roll);
					offb.current_goal.yaw = yaw;
					ROS_INFO("Switch to position control (x=%f, y=%f, z=%f, yaw=%f)",
							offb.current_goal.position.x, offb.current_goal.position.y, offb.current_goal.position.z, offb.current_goal.yaw);
				}

				offb.current_pose.header.stamp = offb.current_goal.header.stamp;
				// local_pos_pub.publish(current_goal);
				offb.local_pos_pub.publish(offb.current_goal);

				// Vision pose should be published at a steady
				// frame rate so that EKF from px4 stays stable
				// vision_pos_pub.publish(offboarding.current_pose);
				offb.vision_pos_pub.publish(offb.current_pose);
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


				offb.current_goal.header.stamp = ros::Time::now();

				if( offb.is_twist_old() && offb.current_goal.type_mask != POSITION_CONTROL) {
					//switch to position mode with last position if twist is not received for more than 1 sec

					offb.current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
					offb.current_goal.type_mask = POSITION_CONTROL;
					offb.current_goal.position.x = offb.current_pose.pose.position.x;
					offb.current_goal.position.y = offb.current_pose.pose.position.y;
					offb.current_goal.position.z = 1.5;
					offb.current_goal.yaw = offb.current_pose.pose.orientation.z;

					tfScalar yaw, pitch, roll;
					tf::Matrix3x3 mat(tf::Quaternion(offb.current_pose.pose.orientation.x, offb.current_pose.pose.orientation.y, offb.current_pose.pose.orientation.z, offb.current_pose.pose.orientation.w));
					mat.getEulerYPR(yaw, pitch, roll);
					offb.current_goal.yaw = yaw;
					ROS_INFO("Switch to position control (x=%f, y=%f, z=%f, yaw=%f)",
							offb.current_goal.position.x, offb.current_goal.position.y, offb.current_goal.position.z, offb.current_goal.yaw);
				}

				offb.current_pose.header.stamp = offb.current_goal.header.stamp;
				offb.local_pos_pub.publish(offb.current_goal);

				// Vision pose should be published at a steady
				// frame rate so that EKF from px4 stays stable
				offb.vision_pos_pub.publish(offb.current_pose);
			}
		}

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

