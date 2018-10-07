#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
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
#include <opencv2/highgui.hpp>
#include <vector>

namespace {
using std::vector;
using Matrix = Eigen::MatrixXf;
using Vector = Eigen::VectorXf;
using cv::Mat_;
using nav_msgs::OccupancyGrid;
using std::abs;
using std::cout;

constexpr auto GRID_IMAGE_DEFAULT = 50;
constexpr auto GRID_MAX = 100;
}  // namespace

Mat_<uint8_t> to_mat(const OccupancyGrid& grid) {
    Mat_<uint8_t> image(grid.info.height, grid.info.width, GRID_IMAGE_DEFAULT);
    for (unsigned int i; i < grid.data.size(); ++i) {
        auto x = i % grid.info.width;
        auto y = i / grid.info.width;
        if (grid.data[i] >= 0) {
            image(y, x) = (GRID_MAX - grid.data[i]) * 255 / 100;
        }
    }
    // cv_bridge::CvImage ros_image(std_msgs::Header(),"mono8",image);
    cv::imshow("image", image);
    return image;
}

class FloorPlaneMapping {
protected:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    ros::Subscriber scan_sub_;

    ros::Publisher grid_pub_;
    image_transport::Publisher gridimage_pub_;

    tf::TransformListener listener_;

    std::string map_frame_;
    std::string base_frame_;
    int width_;
    int height_;
    double resolution_;
    double max_range_;

    nav_msgs::OccupancyGrid grid_;
    pcl::PointCloud<pcl::PointXYZ> lastpcl_;
    pcl::PointCloud<pcl::PointXYZ> pcl_world_;

protected:  // ROS Callbacks
    Vector find_normal(const vector<unsigned int>& points) {
        auto n = points.size();

        Matrix A(n, 3);
        Vector b(n);

        // Linear regression: z = a*x + b*y + c
        for (unsigned int i = 0; i < n; ++i) {
            auto ix = points[i];
            A(i, 0) = pcl_world_[ix].x;
            A(i, 1) = pcl_world_[ix].y;
            A(i, 2) = 1;

            b(i, 0) = pcl_world_[ix].z;
        }
        Vector X = A.colPivHouseholderQr().solve(b);
        return X;  // TODO: Check move semantics
    }

    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
        // Receive the point cloud and convert it to the right format
        pcl::PointCloud<pcl::PointXYZ> temp;
        pcl::fromROSMsg(*msg, temp);
        // Make sure the point cloud is in the base-frame
        listener_.waitForTransform(base_frame_, msg->header.frame_id,
                                   msg->header.stamp, ros::Duration(1.0));
        pcl_ros::transformPointCloud(base_frame_, msg->header.stamp, temp,
                                     msg->header.frame_id, lastpcl_, listener_);

        listener_.waitForTransform(map_frame_, msg->header.frame_id,
                                   msg->header.stamp, ros::Duration(1.0));
        pcl_ros::transformPointCloud(map_frame_, msg->header.stamp, temp,
                                     msg->header.frame_id, pcl_world_, listener_);

        unsigned int n = temp.size();
        // TODO: Iterate through points and store them in grid matrix
        vector<vector<unsigned int>> gridpoints(width_ * height_);
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
            float z = lastpcl_[i].z;
            d = hypot(x, y);
            if (d > max_range_) {
                // too far, ignore
                continue;
            }
            float x0 = grid_.info.origin.position.x;
            float y0 = grid_.info.origin.position.y;
            x = pcl_world_[i].x - x0;
            y = pcl_world_[i].y - y0;
            if (0 <= x && x < width_ * resolution_ && 0 <= y &&
                y < height_ * resolution_) {
                // If we reach this stage, we have an acceptable point, so
                // let's store it
                int x_grid = x / resolution_;
                int y_grid = y / resolution_;
                int ix = y_grid * width_ + x_grid;
                gridpoints[ix].push_back(i);
            }
        }
        // TODO: For each gridcell, calculate normal and modify occupancy grid
        for (int i = 0; i < gridpoints.size(); ++i) {
            auto N = find_normal(gridpoints[i]);
            auto a = N[0];
            auto b = N[1];
            auto error = abs(1 - (1 / (a * a + b * b + 1)));
            grid_.data[i] = (error < 0.5) ? 0 : GRID_MAX;
        }
        // for (unsigned int i = 0; i < grid_.data.size(); ++i) {
        //   cout << static_cast<int>(grid_.data[i]) << ' ';
        //   if (i % grid_.info.width == 0) {
        //     cout << '\n';
        //   }
        // }
        auto image = to_mat(grid_);
        cv_bridge::CvImage ros_image(std_msgs::Header(), "mono8", image);
        gridimage_pub_.publish(ros_image.toImageMsg());
    }

public:
    FloorPlaneMapping() : nh_("~"), it_(nh_) {
        // The parameter below described the frame in which the point cloud
        // must be projected to be estimated. You need to understand TF
        // enough to find the correct value to update in the launch file
        nh_.param("map_frame", map_frame_, std::string("/world"));
        nh_.param("base_frame", base_frame_, std::string("/bubbleRob"));
        // This parameter defines the maximum range at which we want to
        // consider points. Experiment with the value in the launch file to
        // find something relevant.
        nh_.param("max_range", max_range_, 5.0);

        nh_.param("width", width_, 100);
        nh_.param("height", height_, 100);
        nh_.param("resolution", resolution_, 0.5);

        // Make sure TF is ready
        ros::Duration(0.5).sleep();

        // TODO: Change to param
        scan_sub_ = nh_.subscribe("/vrep/depthSensor", 1,
                                  &FloorPlaneMapping::pcl_callback, this);
        grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid", 1);
        gridimage_pub_ = it_.advertise("gridimage", 1);

        grid_.header.frame_id = base_frame_;

        grid_.info.width = width_;
        grid_.info.height = height_;
        grid_.info.resolution = resolution_;

        grid_.info.origin.position.x = -(resolution_ * width_) / 2;
        grid_.info.origin.position.y = -(resolution_ * height_) / 2;
        grid_.info.origin.position.z = 0;
        grid_.info.origin.orientation.w = 1;

        grid_.data.resize(width_ * height_);

        for (auto& it : grid_.data) {
            it = -1;
        }
    }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "floor_plane_mapping");
    FloorPlaneMapping fpm;

    ros::spin();
    return 0;
}
