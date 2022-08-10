#include "vision_pose.h"


void VisionPose::state_cb_(const mavros_msgs::State::ConstPtr& msg) {
	current_state_ = *msg;
}

void VisionPose::pose_loop_cb(const ros::TimerEvent &event) {
	try{
		tf::StampedTransform visionPoseTf;
		listener_.lookupTransform("/map", "/base_link", ros::Time(0), visionPoseTf);

		//update currentPose
		current_pose_.header.frame_id = "map";
		current_pose_.header.stamp = ros::Time().now();
		current_pose_.pose.position.x    = visionPoseTf.getOrigin().x();
		current_pose_.pose.position.y    = visionPoseTf.getOrigin().y();
		current_pose_.pose.position.z    = visionPoseTf.getOrigin().z();
		current_pose_.pose.orientation.x = visionPoseTf.getRotation().x();
		current_pose_.pose.orientation.y = visionPoseTf.getRotation().y();
		current_pose_.pose.orientation.z = visionPoseTf.getRotation().z();
		current_pose_.pose.orientation.w = visionPoseTf.getRotation().w();
		ROS_INFO("pose updated");
	}
	catch (tf::TransformException& ex){
		ROS_ERROR("%s",ex.what());
	}
	vision_pos_pub_.publish(current_pose_);
}

VisionPose::VisionPose(ros::NodeHandle nh, ros::NodeHandle nh_private)
	: nh_(nh),
	nh_private_(nh_private),
	rate_{30.0}
	{
	state_sub_ = nh_.subscribe("/mavros/state", 1, &VisionPose::state_cb_, this);
	vision_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1);
	cmdloop_timer_ = nh_.createTimer(ros::Duration(0.10), &VisionPose::pose_loop_cb, this);
	}
