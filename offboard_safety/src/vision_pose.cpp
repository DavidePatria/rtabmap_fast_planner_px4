#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <ros/init.h>
#include <ros/rate.h>
#include <ros/time.h>
#include <tf/transform_listener.h>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "vision_pos_publisher");
	ROS_INFO("vision pose publisher node started");

	ros::NodeHandle nh;
	ros::Publisher vision_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1);
	ros::Subscriber state_sub = nh.subscribe("/mavros/state", 10, state_cb);

	ros::Rate rate(30.0);

	geometry_msgs::PoseStamped current_pose;
	current_pose.header.frame_id = "map";

	tf::TransformListener listener;

	while(ros::ok() && !current_state.connected){
		ROS_INFO_THROTTLE(2, "not connected yet!");
		ros::spinOnce();
		rate.sleep();
	}

	ros::spinOnce();

	// if(!listener.waitForTransform("/map", "/base_link", ros::Time().now(), ros::Duration(5))) {
	// 	ROS_ERROR("Cannot get current position between /map and /base_link");
	// 	return -1;
	// }

	while(ros::ok()){

		try{
			tf::StampedTransform visionPoseTf;
			listener.lookupTransform("/map", "/base_link", ros::Time(0), visionPoseTf);

			//update currentPose
			current_pose.header.stamp = ros::Time().now();
			current_pose.pose.position.x    = visionPoseTf.getOrigin().x();
			current_pose.pose.position.y    = visionPoseTf.getOrigin().y();
			current_pose.pose.position.z    = visionPoseTf.getOrigin().z();
			current_pose.pose.orientation.x = visionPoseTf.getRotation().x();
			current_pose.pose.orientation.y = visionPoseTf.getRotation().y();
			current_pose.pose.orientation.z = visionPoseTf.getRotation().z();
			current_pose.pose.orientation.w = visionPoseTf.getRotation().w();
			ROS_INFO("pose updated");
		}
		catch (tf::TransformException & ex){
			ROS_ERROR("%s",ex.what());
		}
		vision_pos_pub.publish(current_pose);
		ros::spinOnce();
		rate.sleep();
	}
}
