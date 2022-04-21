#ifndef TAKEOFF_SRV_H
#define TAKEOFF_SRV_H

#include "ros/ros.h"
#include "offboard_safety/MakeTakeoff.h"
#include "ros/time.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"


class TeichingOfServis {
	public:
		TeichingOfServis();

		
	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_land_;
		ros::Publisher pub_land_;
		bool send_takeoff(offboard_safety::MakeTakeoff::Request &req, offboard_safety::MakeTakeoff::Response &res);
};


#endif
