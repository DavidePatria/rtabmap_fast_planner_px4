#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <ros/init.h>
#include <ros/rate.h>
#include <ros/time.h>
#include <tf/transform_listener.h>

class VisionPose {

public:
	VisionPose(ros::NodeHandle nh, ros::NodeHandle nh_private);

private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	ros::Rate rate_;
	ros::Timer cmdloop_timer_;

	// only publisher needed here
	ros::Publisher vision_pos_pub_;
	ros::Subscriber state_sub_;

	void state_cb_(const mavros_msgs::State::ConstPtr& state);
	void pose_loop_cb(const ros::TimerEvent& event);

	geometry_msgs::PoseStamped current_pose_;
	mavros_msgs::State current_state_;

	tf::TransformListener listener_;

};
