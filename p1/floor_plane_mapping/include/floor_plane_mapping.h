#pragma once
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>

class FloorPlaneMapping {
protected:
    ros::Subscriber scan_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher gridmap_pub_;
    ros::Publisher imagemap_pub_;
    tf::TransformListener listener_;

    ros::NodeHandle nh_;
    std::string base_frame_;
    double max_range_;

    nav_msgs::OccupancyGrid gridmap;
    pcl::PointCloud<pcl::PointXYZ> lastpcl_;

    Eigen::VectorXf find_normal(const vector<pcl::PointXYZ>& points);
    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr msg);

public:
    FloorPlaneMapping();
};

