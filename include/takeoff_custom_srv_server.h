#ifndef TAKEOFF_SRV_H
#define TAKEOFF_SRV_H

#include "ros/ros.h"
#include "offboard_safety/MakeTakeoff.h"
#include "ros/time.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"

class OffBoarding;

class TeichingOfServis {
	public:
		TeichingOfServis(OffBoarding *offboard);
		
	private:
		ros::ServiceServer service_;
		OffBoarding *point_;
		ros::NodeHandle nh_;
		bool send_takeoff(offboard_safety::MakeTakeoff::Request &req, offboard_safety::MakeTakeoff::Response &res);
};


#endif
