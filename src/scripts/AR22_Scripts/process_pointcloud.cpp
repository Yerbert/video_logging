#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

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

ros::Publisher pub;

void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
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
    pub.publish(new_cloud_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "process_messages");
    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/processed", 1);
    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, pointcloud_callback);

    ros::spin();

    return 0;
}
