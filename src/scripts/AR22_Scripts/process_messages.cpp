#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <video_logging/FilterSwitch.h>

#include <iostream>
#include <math.h>
#include <queue>

#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

class Filter
{
	public:
		Filter() {}
		
		virtual bool filter() {
			// Returns true if filtered out
			std::cout << "\nCalling base class filter\n";
		}
		
		bool enabled = false;
};


class StutterFilter : public Filter
{
	protected:
		int stutter_size;
		bool block = false;
		int count = 0;
		int limit = 10;
		int rand_max = 30;
		int rand_min = 4;

	public:

		StutterFilter() : Filter() {
			this->stutter_size = 1;
		}

		StutterFilter(int stutter_size) : Filter() {
			this->stutter_size = stutter_size;
		}

		bool filter() override
		{
			count++;
			if (count >= limit) {
				count = 0;
				limit = rand_int(rand_min, rand_max) * stutter_size;
				block = !block;
			}

			return (this->enabled && block);

		}

		int rand_int(int min, int max) {
			return (std::rand() % (max - min + 1)) + min;
		}

};


class BlockFilter : public Filter
{
	public:

		BlockFilter() : Filter() {}

		bool filter() override
		{
			return this->enabled;
		}
};


int pointcloud_downsample = 1; // downsample factor. 1 = no downsampling
int pointcloud_counter = 0;
int video_downsample = 2;
int video_counter = 0;
ros::Publisher pointcloud_pub;
ros::Publisher video_pub;
ros::Publisher tf_pub;

StutterFilter camera_flicker(1);
BlockFilter camera_blocked;
StutterFilter velodyne_flicker(1);
BlockFilter velodyne_blocked;
bool non_existent_object = true;

bool is_acceptable_point(const pcl::PointXYZ& point)
{
	/*
	Culls points further than 'max_dist' and lower than 'floor_y'
	*/

	bool accept_all_points = false;
	int max_dist = 6;
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


tf2_msgs::TFMessage flatten_tf(const tf2_msgs::TFMessageConstPtr& msg) {

	tf2::Quaternion quat(
		msg->transforms[0].transform.rotation.x,
		msg->transforms[0].transform.rotation.y,
		msg->transforms[0].transform.rotation.z,
		msg->transforms[0].transform.rotation.w);

	tf2::Matrix3x3 m(quat);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	tf2::Quaternion quat_converted;
	quat_converted.setRPY(0, 0, yaw);

	std::string child_frame_id = msg->transforms[0].child_frame_id;
	std::string parent_frame_id = msg->transforms[0].header.frame_id;
	// std::string new_child_frame_id = child_frame_id;
	std::string new_child_frame_id = "flat_" + child_frame_id;
	// std::string new_parent_frame_id = parent_frame_id;
	std::string new_parent_frame_id = (parent_frame_id == "map") ? "map" : "flat_" + parent_frame_id;

	// new tf
	geometry_msgs::TransformStamped tf_converted_stamped;
	tf_converted_stamped.header = msg->transforms[0].header;
	tf_converted_stamped.header.frame_id = new_parent_frame_id;
	tf_converted_stamped.child_frame_id = new_child_frame_id;
	tf_converted_stamped.transform.translation = msg->transforms[0].transform.translation;

	// set height to zero
	tf_converted_stamped.transform.translation.z = 0;
	// make x,y rotation flat
	tf_converted_stamped.transform.rotation.x = quat_converted.x();
	tf_converted_stamped.transform.rotation.y = quat_converted.y();
	tf_converted_stamped.transform.rotation.z = quat_converted.z();
	tf_converted_stamped.transform.rotation.w = quat_converted.w();

	tf2_msgs::TFMessage tf_output_msg;
	tf_output_msg.transforms.push_back(tf_converted_stamped);

	return tf_output_msg;

}


void filter_callback(const video_logging::FilterSwitch::ConstPtr& filter_msg) {

	// std::cout << "\nFilter msg received:\n" << filter_msg;

	velodyne_blocked.enabled = filter_msg->velodyne_blocked;
	velodyne_flicker.enabled = filter_msg->velodyne_flicker;
	camera_blocked.enabled = filter_msg->camera_blocked;
	camera_flicker.enabled = filter_msg->camera_flicker;
	// non_existent_object = filter_msg->camera_flicker;
}

pcl::PointXYZ create_fake_point() {
	float centre_x = 0;
	float centre_y = 1;
	float centre_z = 0;
	float numPoints = 100;

	float x = centre_x + ( ( ((float)(std::rand() % 200)) - 100) / 300);
	float y = centre_y + ( ( ((float)(std::rand() % 200)) - 100) / 300);
	float z = centre_z + ( ( ((float)(std::rand() % 200)) - 100) / 300);
	pcl::PointXYZ new_point(x,y,z);
	return new_point;
}


void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	bool filtered = velodyne_blocked.filter() || velodyne_flicker.filter();
	if (filtered) {
		return;
	}

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

	if (non_existent_object) {
		for(int i=0; i<100; i++) {
			new_cloud.points.push_back(create_fake_point());
		}
	}

	sensor_msgs::PointCloud2 new_cloud_msg;
	pcl::toROSMsg(new_cloud, new_cloud_msg);
	pointcloud_pub.publish(new_cloud_msg);
}


void video_callback(const sensor_msgs::CompressedImageConstPtr& video_msg)
{
	video_counter = (video_counter + 1) % video_downsample;
	if (video_counter != 0) {
		return;
	}

	bool filtered = camera_blocked.filter() || camera_flicker.filter();
	if (filtered) {
		return;
	}

	video_pub.publish(video_msg);
}

void tf_callback(const tf2_msgs::TFMessageConstPtr& msg)
{
	if (msg->transforms[0].child_frame_id == "base_link" || msg->transforms[0].child_frame_id == "odom") {
		tf_pub.publish(flatten_tf(msg));
	}
}

void fakeobj_callback(const std_msgs::BoolConstPtr& msg)
{
	non_existent_object = msg->data;
}



int main(int argc, char** argv)
{

	ros::init(argc, argv, "process_messages");
	ros::NodeHandle nh;

	std::string pointcloud_intopic = "/velodyne_points";
	std::string pointcloud_outtopic = "/velodyne_points/processed";

	std::string video_intopic = "/camera/color/image_raw/compressed";
	std::string video_outtopic = "/camera/color/image_raw/compressed/processed";

	std::string tf_intopic = "/tf";
	std::string tf_outtopic = "/tf";

	std::string filter_intopic = "/filters";
	std::string fakeobj_intopic = "/fake_object";

	// Point cloud
	pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_outtopic, 1);
	ros::Subscriber pointcloud_sub = nh.subscribe(pointcloud_intopic, 1, pointcloud_callback);
	std::cout << "Republishing point clouds from \n  " << pointcloud_intopic << "\nto\n  " << pointcloud_outtopic << "\n";

	// Video
	video_pub = nh.advertise<sensor_msgs::CompressedImage>(video_outtopic, 1);
	ros::Subscriber video_sub = nh.subscribe(video_intopic, 1, video_callback);
	std::cout << "Republishing video from \n  " << video_intopic << "\nto\n  " << video_outtopic << "\n";

	// TF
	tf_pub = nh.advertise<tf2_msgs::TFMessage>(tf_outtopic, 1);
	ros::Subscriber tf_sub = nh.subscribe(tf_intopic, 1, tf_callback);
	std::cout << "Republishing tf from \n  " << tf_intopic << "\nto\n  " << tf_outtopic << "\n";

	// Filters
	ros::Subscriber filter_sub = nh.subscribe(filter_intopic, 1, filter_callback);

	// Fake object
	ros::Subscriber fakeobj_sub = nh.subscribe(fakeobj_intopic, 1, fakeobj_callback);

	ros::spin();

	return 0;
}