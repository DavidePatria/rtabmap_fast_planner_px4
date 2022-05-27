#include <ros/ros.h>
#include <ros/subscriber.h>
#include <pcl/point_cloud.h>
// #include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>

// does it really need to be const?
// a check with the method to rotate is necessary
Eigen::Affine3f create_rotation_matrix(double ax, double ay, double az) {
	Eigen::Affine3f rx = Eigen::Affine3f(Eigen::AngleAxisf(ax, Eigen::Vector3f(1, 0, 0)));
	Eigen::Affine3f ry = Eigen::Affine3f(Eigen::AngleAxisf(ay, Eigen::Vector3f(0, 1, 0)));
	Eigen::Affine3f rz = Eigen::Affine3f(Eigen::AngleAxisf(az, Eigen::Vector3f(0, 0, 1)));
	return rz * ry * rx;
}

Eigen::Affine3f rotation = create_rotation_matrix(-1.57,0,-1.57);

void cloud_sub_cb(const sensor_msgs::PointCloud2 &msg) {
	sensor_msgs::PointCloud2 cloud_out;
	// pcl::transformPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, const Eigen::Affine3f &transform)
	pcl::transformPointCloud(msg, cloud_out, &rotation);
	// pcl::transformPointCloud;
}

int main(int argc, char **argv) {
	// pcl::transformPointCloud()
	ros::NodeHandle nh;
	// std::string 
	ros::init(argc, argv, "cloud_rotation");

	ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>
		("camera_cloud", 1, cloud_sub_cb);


	
}
