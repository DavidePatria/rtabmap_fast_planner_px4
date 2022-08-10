#include "vision_pose.h"

int main(int argc, char** argv) {

	std::string node_name {"vision_pos_publisher"};
	ros::init(argc, argv, node_name);
	ROS_INFO("started %s", node_name.c_str());

	ros::NodeHandle nh("~");
	ros::NodeHandle nh_private("");

	VisionPose vision_pose(nh, nh_private);

	// set log level at compile time. would be better to have it at runtime through config
	if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
		ros::console::notifyLoggerLevelsChanged();
	}

	ros::spin();
	return 0;
}

