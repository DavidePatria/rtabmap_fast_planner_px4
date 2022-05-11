#include "geometry_msgs/PoseStamped.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/time.h"
#include "tf/transform_listener.h"


int main(int argc, char **argv)
{

	ROS_INFO("about to start, get ready!");
	ros::init(argc, argv, "pos_publisher");

	ros::Publisher vision_pos_pub;
	ros::NodeHandle nh;

	vision_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
		("mavros/vision_pose/pose", 1);

	ros::Rate rate(50.0);

	geometry_msgs::PoseStamped current_pose;
	current_pose.header.frame_id = "map";

	tf::TransformListener listener;

	ros::spinOnce();

	if(!listener.waitForTransform("/map", "/base_link", ros::Time().now(), ros::Duration(5))) {
		ROS_ERROR("Cannot get current position between /map and /base_link");
		return -1;
	}

	while(ros::ok()){

		try{
			tf::StampedTransform visionPoseTf;
			listener.lookupTransform("/map", "/base_link", ros::Time().now(), visionPoseTf);

			//update currentPose
			current_pose.pose.position.x    = visionPoseTf.getOrigin().x();
			current_pose.pose.position.y    = visionPoseTf.getOrigin().y();
			current_pose.pose.position.z    = visionPoseTf.getOrigin().z();
			current_pose.pose.orientation.x = visionPoseTf.getRotation().x();
			current_pose.pose.orientation.y = visionPoseTf.getRotation().y();
			current_pose.pose.orientation.z = visionPoseTf.getRotation().z();
			current_pose.pose.orientation.w = visionPoseTf.getRotation().w();
		}
		catch (tf::TransformException & ex){
			ROS_ERROR("%s",ex.what());
		}
		vision_pos_pub.publish(current_pose);
		ros::spinOnce();
		rate.sleep();
	}
}
