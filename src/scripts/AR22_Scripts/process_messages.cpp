#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <iostream>

int pointcloud_downsample = 1;
int pointcloud_counter = 0;
int video_downsample = 1;
int video_counter = 0;
ros::Publisher pointcloud_pub;
ros::Publisher video_pub;

bool is_acceptable_point(const pcl::PointXYZ& point)
{
	/*
	Culls points further than 'max_dist' and lower than 'floor_y'
	*/

	bool accept_all_points = false;
	int max_dist = 4;
	float floor_z = -0.38;

	if (accept_all_points) {
	return true;
	}

	// drop point if lies outside max_dist^3 cube
	if ( abs(point.x) > max_dist || abs(point.y) > max_dist || abs(point.z) > max_dist ) {
	return false;
	}

	// drop point if at/below floor level
	if (point.z <= floor_z) {
	return false;
	}

	return true;
}

void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	pointcloud_counter = (pointcloud_counter + 1) % pointcloud_downsample;
	if (pointcloud_counter != 0) {
		return;
	}

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*cloud_msg, cloud);

	pcl::PointCloud<pcl::PointXYZ> new_cloud;
	new_cloud.header = cloud.header;
	new_cloud.points.reserve(cloud.points.size());

	for (const auto& point : cloud) {

		if (is_acceptable_point(point)) {

		pcl::PointXYZ new_point(point.x, point.y, point.z);
		new_cloud.points.push_back(new_point);
		}
	}

	sensor_msgs::PointCloud2 new_cloud_msg;
	pcl::toROSMsg(new_cloud, new_cloud_msg);
	pointcloud_pub.publish(new_cloud_msg);
}


void video_callback(const sensor_msgs::CompressedImageConstPtr& msg)
{
	video_counter = (video_counter + 1) % video_downsample;
	if (video_counter != 0) {
		return;
	}

	video_pub.publish(msg);
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "process_messages");
	ros::NodeHandle nh;

	std::string pointcloud_intopic = "/velodyne_points";
	std::string pointcloud_outtopic = "/velodyne_points/processed";

	std::string video_intopic = "/camera/color/image_raw/compressed";
	std::string video_outtopic = "/camera/color/image_raw/compressed/processed";

	pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_outtopic, 1);
	ros::Subscriber pointcloud_sub = nh.subscribe(pointcloud_intopic, 1, pointcloud_callback);
	std::cout << "Republishing point clouds from \n  " << pointcloud_intopic << "\nto\n  " << pointcloud_outtopic << "\n";

	video_pub = nh.advertise<sensor_msgs::CompressedImage>(video_outtopic, 1);
	ros::Subscriber video_sub = nh.subscribe(video_intopic, 1, video_callback);
	std::cout << "Republishing video from \n  " << video_intopic << "\nto\n  " << video_outtopic << "\n";

	ros::spin();

	return 0;
}