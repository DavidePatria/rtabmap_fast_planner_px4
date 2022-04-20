#include "ros/ros.h"
#include "offboard_safety/MakeTakeoff.h"
#include "ros/time.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"

// should this be ifdefined?
#include "offboarding.h"

class TeichingOfServis {
	public:
		TeichingOfServis(OffBoarding &offb) {
		// pub_land = nh.advertise<std_msgs::Bool>("land_stream", 1);
		// instead of changing the "landing variable" through a subscriber
		// callback it is better to pass the offboarding class to the service 
		// so it can use the method to change the variable that sets the 
		// autolanding procedure
		
		}

		
		bool send_takeoff(offboard_safety::MakeTakeoff::Request &req, offboard_safety::MakeTakeoff::Response &res) {
			pub_land.publish(&req);
			return true;
		};
		
	private:
		ros::NodeHandle nh;
		ros::Subscriber sub_land;
		ros::Publisher pub_land;
		// el constructor tendrá los subscriptores
};



int main(int argc, char **argv) {

	// fix constructor to get the object reference from the main
	TeichingOfServis teicoff(&offb);

	ros::init(argc, argv, "custom_takeoff");
	ros::NodeHandle nh;

	ros::ServiceServer service = nh.advertiseService("custom/make_takeoff", &TeichingOfServis::send_takeoff, &teicoff);

	ROS_INFO("make_takeoff server has started");
	ros::spin();

	return 0;
}
