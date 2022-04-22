#include "ros/ros.h"
#include "offboard_safety/MakeTakeoff.h"
#include "ros/time.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"

// should this be ifdefined?
#include "offboarding.h"

// forward declaration instead of include. needs checking
// class OffBoarding;

// pointer to main class so the service can call a method in it
TeichingOfServis::TeichingOfServis(OffBoarding *offboarding):nh_(""), point_(offboarding) {
	service_ = nh_.advertiseService("custom/make_takeoff", &TeichingOfServis::send_takeoff, this);
	ROS_INFO("make_takeoff server has started");
}


bool TeichingOfServis::send_takeoff(offboard_safety::MakeTakeoff::Request &req, offboard_safety::MakeTakeoff::Response &res) {
	point_->set_autoland(req.setTakeoff);
	return true;
}



// int main(int argc, char **argv) {
//
// 	// fix constructor to get the object reference from the main
//
// 	ros::init(argc, argv, "custom_takeoff");
// 	ros::NodeHandle nh;
//
// 	ros::ServiceServer service = nh.advertiseService("custom/make_takeoff", &TeichingOfServis::send_takeoff, &teicoff);
//
// 	ROS_INFO("make_takeoff server has started");
// 	ros::spin();
//
// 	return 0;
// }
