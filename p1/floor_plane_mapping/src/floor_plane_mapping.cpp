#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <vector>

namespace {
using std::vector;
using Point = pcl::PointXYZ;
using Matrix = Eigen::MatrixXf;
using Vector = Eigen::VectorXf;
}  // namespace

Vector find_normal(const vector<Point>& points) {
    auto n = points.size();

    Matrix A(n, 3);
    Vector b = Vector::Ones(n);

    for (unsigned int i = 0; i < n; ++i) {
        A(i, 0) = points[i].x;
        A(i, 1) = points[i].y;
        A(i, 2) = points[i].z;

        b(i, 0) = 1;
    }
    Vector X = A.colPivHouseholderQr().solve(b);
    return X; // TODO: Check move semantics
}

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

    // visualization_msgs::MarkerArray marker_array_msg;
    nav_msgs::OccupancyGrid gridmap;
    pcl::PointCloud<pcl::PointXYZ> lastpcl_;

protected:  // ROS Callbacks
    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
        // Receive the point cloud and convert it to the right format
        pcl::PointCloud<pcl::PointXYZ> temp;
        pcl::fromROSMsg(*msg, temp);
        // Make sure the point cloud is in the base-frame
        listener_.waitForTransform(base_frame_, msg->header.frame_id,
                                   msg->header.stamp, ros::Duration(1.0));
        pcl_ros::transformPointCloud(base_frame_, msg->header.stamp, temp,
                                     msg->header.frame_id, lastpcl_, listener_);

        unsigned int n = temp.size();
        // TODO: Iterate through points and store them in grid matrix
        for (unsigned int i = 0; i < n; i++) {
            float x = temp[i].x;
            float y = temp[i].y;
            float d = hypot(x, y);
            // In the sensor frame, this point would be inside the camera
            if (d < 1e-2) {
                // Bogus point, ignore
                continue;
            }
            // Measure the point distance in the base frame
            x = lastpcl_[i].x;
            y = lastpcl_[i].y;
            d = hypot(x, y);
            if (d > max_range_) {
                // too far, ignore
                continue;
            }
            // If we reach this stage, we have an acceptable point, so
            // let's store it
        }
        // TODO: For each gridcell, calculate normal and modify occupancy grid
    }

public:
    FloorPlaneMapping() : nh_("~") {
        // TODO START
        // The parameter below described the frame in which the point cloud
        // must be projected to be estimated. You need to understand TF
        // enough to find the correct value to update in the launch file
        nh_.param("base_frame", base_frame_, std::string("/body"));
        // This parameter defines the maximum range at which we want to
        // consider points. Experiment with the value in the launch file to
        // find something relevant.
        nh_.param("max_range", max_range_, 5.0);
        // END OF TODO

        // Make sure TF is ready
        ros::Duration(0.5).sleep();

        // Subscribe to the point cloud and prepare the marker publisher
        scan_sub_ =
            nh_.subscribe("scans", 1, &FloorPlaneMapping::pcl_callback, this);
        gridmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("gridmap", 1);
    }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "floor_plane_mapping");
    FloorPlaneMapping fpm;

    ros::spin();
    return 0;
}
