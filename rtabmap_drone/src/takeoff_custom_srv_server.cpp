#include "ros/ros.h"
#include "rtabmap_drone/MakeTakeoff.h"




bool send_takeoff(MakeTakeoff::MakeTakeoff.srv::Request &req, MakeTakeoff::MakeTakeoff::Response &res) {
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "custom_takeoff");
	ros::NodeHandle nh;

	ros::ServiceServer service = nh.advertiseService("custom/make_takeoff", send_takeoff);

	ROS_INFO("make_takeoff server has started");
	ros::spin();

	return 0;
}
