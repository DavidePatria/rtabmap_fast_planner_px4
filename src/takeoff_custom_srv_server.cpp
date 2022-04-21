#include "ros/ros.h"
#include "offboard_safety/MakeTakeoff.h"
#include "ros/time.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"

// should this be ifdefined?
// #include "offboarding.h"

// forward declaration instead of include. needs checking
class OffBoarding;

class TeichingOfServis {
	public:
		TeichingOfServis(OffBoarding *offboardin):nh_("") {
		// pub_land = nh.advertise<std_msgs::Bool>("land_stream", 1);
		// instead of changing the "landing variable" through a subscriber
		// callback it is better to pass the offboarding class to the service 
		// so it can use the method to change the variable that sets the 
		// autolanding procedure
		
		ros::ServiceServer service = nh_.advertiseService("custom/make_takeoff", &TeichingOfServis::send_takeoff, this);

		ROS_INFO("make_takeoff server has started");
		ros::spin();
		}

		
	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_land_;
		ros::Publisher pub_land_;

		bool send_takeoff(offboard_safety::MakeTakeoff::Request &req, offboard_safety::MakeTakeoff::Response &res) {
			
			return true;
		};
};



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
