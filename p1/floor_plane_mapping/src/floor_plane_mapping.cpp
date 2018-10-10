#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <tuple>
#include <vector>

namespace {
using std::vector;
using Matrix = Eigen::MatrixXf;
using Vector = Eigen::VectorXf;
using cv::Mat_;
using nav_msgs::OccupancyGrid;
using std::make_tuple;
using std::tie;
using std::tuple;
using std::cout;

constexpr auto MIN_NUM_POINTS = 2;
constexpr auto ERROR_THRESH = 0.8;
constexpr auto GRID_IMAGE_DEFAULT = 50;
constexpr auto GRID_MAX = 100;
}  // namespace

Mat_<uint8_t> to_mat(const OccupancyGrid& grid) {
    Mat_<uint8_t> image(grid.info.height, grid.info.width, GRID_IMAGE_DEFAULT);
    for (auto i = 0U; i < grid.data.size(); ++i) {
        auto x = i % grid.info.width;
        auto y = grid.info.height - i / grid.info.width - 1;
        if (grid.data[i] >= 0) {
            image(y, x) = (GRID_MAX - grid.data[i]) * 255 / 100;
        }
    }
    return image;  // Check Move semantics
}

// Reference: http://docs.ros.org/kinetic/api/hector_mapping/html/
class LogOddCell {
private:
    double logodds = 0.0;
    double increment_factor = 0.5;
    double thresh = 30.0;

public:
    bool occupied() { return logodds > 0; }
    bool free() { return logodds < 0; }
    double occupied_probability() {
        auto odds = exp(logodds);
        return odds / (1.0 + odds);
    }
    void update_occupied(double weight = 1.0) {
        if (logodds < thresh) {
            logodds += weight * increment_factor;
        }
    }
    void update_empty(double weight = 1.0) {
        if (logodds > -thresh) {
            logodds -= weight * increment_factor;
        }
    }
};

class CountCell {
private:
    double prob = 0.5;
    double update_factor = 0.1;

public:
    bool occupied() { return prob > 0.5; }
    bool free() { return prob < 0.5; }
    double occupied_probability() { return prob; }
    void update_occupied(double weight = 1.0) {
        if (prob < 1.0 - update_factor) {
            prob += weight * update_factor;
        }
    }
    void update_empty(double weight = 1.0) {
        if (prob > update_factor) {
            prob -= weight * update_factor;
        }
    }
};

template <typename CellType>
struct ProbabilisticOccGrid {
    std_msgs::Header header;
    nav_msgs::MapMetaData info;
    vector<CellType> data;
    // Convert to ros::OccupancyGrid data
    OccupancyGrid ros_grid() {
        OccupancyGrid outgrid;
        outgrid.header = header;
        outgrid.info = info;
        outgrid.data.resize(data.size());
        for (auto i = 0U; i < data.size(); ++i) {
            if (data[i].occupied()) {
                outgrid.data[i] = GRID_MAX;
            } else if (data[i].free()) {
                outgrid.data[i] = 0;
            } else {
                outgrid.data[i] = -1;
            }
        }
        return outgrid;
    }
    int index(double x, double y) {
        auto x0 = info.origin.position.x;
        auto y0 = info.origin.position.y;
        auto res = info.resolution;
        unsigned int xg = (x - x0) / res;
        unsigned int yg = (y - y0) / res;
        if (!(0 <= xg && xg < info.width) || !(0 <= yg && yg < info.height)) {
            // Outside grid
            return -1;
        }
        // Store point for update
        return yg * info.width + xg;
    }
    tuple<double, double> point(int index) {
        auto x0 = info.origin.position.x;
        auto y0 = info.origin.position.y;
        auto res = info.resolution;
        // Get (x, y) in grid frame
        auto xg = index % info.width;
        auto yg = index / info.width;
        // Convert to global frame
        double x = x0 + xg * res;
        double y = y0 + yg * res;
        // Return as tuple
        return make_tuple(x, y);
    }
};

class FloorPlaneMapping {
protected:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    ros::Subscriber scan_sub_;

    ros::Publisher grid_pub_;
    image_transport::Publisher gridimage_pub_;

    tf::TransformListener listener_;

    std::string base_frame_;
    double max_range_;

    ProbabilisticOccGrid<LogOddCell> prob_grid_;
    pcl::PointCloud<pcl::PointXYZ> pcl_base_;
    pcl::PointCloud<pcl::PointXYZ> pcl_world_;

protected:  // ROS Callbacks
    Vector find_normal(const vector<unsigned int>& points) {
        auto n = points.size();
        // AX = b
        Matrix A(n, 3);
        Vector b(n);
        // Linear regression: z = a*x + b*y + c
        for (auto i = 0U; i < n; ++i) {
            auto ix = points[i];
            A(i, 0) = pcl_world_[ix].x;
            A(i, 1) = pcl_world_[ix].y;
            A(i, 2) = 1;
            b(i, 0) = pcl_world_[ix].z;
        }
        Vector X = A.colPivHouseholderQr().solve(b);
        return X;  // TODO: Check move semantics
    }

    vector<vector<unsigned int>> filter_points(
        const pcl::PointCloud<pcl::PointXYZ>& pcl) {
        // Grid to store indices of points
        vector<vector<unsigned int>> points(prob_grid_.data.size());
        // Filter
        for (auto i = 0U; i < pcl.size(); ++i) {
            double x = pcl[i].x;
            double y = pcl[i].y;
            auto d = hypot(x, y);
            // In the sensor frame, this point would be inside the camera
            if (d < 1e-2) {
                // Bogus point, ignore
                continue;
            }
            // Measure the point distance in the base frame
            x = pcl_base_[i].x;
            y = pcl_base_[i].y;
            d = hypot(x, y);
            if (d > max_range_) {
                // too far, ignore
                continue;
            }
            auto ix = prob_grid_.index(pcl_world_[i].x, pcl_world_[i].y);
            if (ix == -1) {
                // Outside grid, ignore
                continue;
            }
            points[ix].push_back(i);
        }
        return points;
    }

    void update_grid(const vector<vector<unsigned int>>& points) {
        for (auto i = 0U; i < points.size(); ++i) {
            // Only update if point was in FOV
            if (points[i].size() > MIN_NUM_POINTS) {
                auto N = find_normal(points[i]);
                auto a = N[0];
                auto b = N[1];
                auto error = 1.0 / (a * a + b * b + 1);
                // Weigth probability by distance
                double x, y;
                tie(x, y) = prob_grid_.point(i);
                geometry_msgs::PointStamped pt;
                pt.header.frame_id = prob_grid_.header.frame_id;
                pt.point.x = x;
                pt.point.y = y;
                geometry_msgs::PointStamped pt_base;
                listener_.transformPoint(base_frame_, pt, pt_base);
                double xb = pt_base.point.x;
                double yb = pt_base.point.y;
                auto d = hypot(xb, yb);
                // Linear
                auto weight = (max_range_ - d) / max_range_;
                // Occupied
                if (error < ERROR_THRESH) {
                    prob_grid_.data[i].update_occupied(weight);
                } else {
                    prob_grid_.data[i].update_empty(weight);
                }
            }
        }
        return;
    }

    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
        // Receive the point cloud and convert it to the right format
        pcl::PointCloud<pcl::PointXYZ> temp;
        pcl::fromROSMsg(*msg, temp);
        // Wait for transforms to become available
        listener_.waitForTransform(base_frame_, msg->header.frame_id,
                                   msg->header.stamp, ros::Duration(1.0));
        listener_.waitForTransform(prob_grid_.header.frame_id,
                                   msg->header.frame_id, msg->header.stamp,
                                   ros::Duration(1.0));
        // Transform to new frame
        pcl_ros::transformPointCloud(base_frame_, msg->header.stamp, temp,
                                     msg->header.frame_id, pcl_base_,
                                     listener_);
        pcl_ros::transformPointCloud(
            prob_grid_.header.frame_id, msg->header.stamp, temp,
            msg->header.frame_id, pcl_world_, listener_);
        // Filter points
        auto gridpoints = filter_points(temp);
        // Update grid
        update_grid(gridpoints);
        // Publish grid
        OccupancyGrid grid = prob_grid_.ros_grid();
        grid_pub_.publish(grid);
        // Generate CV::Mat of grid
        auto image = to_mat(grid);
        cv_bridge::CvImage ros_image(std_msgs::Header(), "mono8", image);
        gridimage_pub_.publish(ros_image.toImageMsg());
    }

public:
    FloorPlaneMapping() : nh_("~"), it_(nh_) {
        // The parameter below described the frame in which the point cloud
        // must be projected to be estimated. You need to understand TF
        // enough to find the correct value to update in the launch file
        std::string map_frame;
        nh_.param("map_frame", map_frame, std::string("/world"));
        nh_.param("base_frame", base_frame_, std::string("/bubbleRob"));
        // This parameter defines the maximum range at which we want to
        // consider points. Experiment with the value in the launch file to
        // find something relevant.
        nh_.param("max_range", max_range_, 4.5);

        // Width, Height in cells
        int width;
        nh_.param("width", width, 102);
        int height;
        nh_.param("height", height, 102);
        // Resolution in m/cell
        double resolution;
        nh_.param("resolution", resolution, 0.1);

        // TODO: Change to param
        scan_sub_ = nh_.subscribe("/vrep/depthSensor", 1,
                                  &FloorPlaneMapping::pcl_callback, this);
        grid_pub_ = nh_.advertise<OccupancyGrid>("grid", 1);
        gridimage_pub_ = it_.advertise("gridimage", 1);

        prob_grid_.header.frame_id = map_frame;

        prob_grid_.info.width = width;
        prob_grid_.info.height = height;
        prob_grid_.info.resolution = resolution;

        prob_grid_.info.origin.position.x = -(resolution * width) / 2;
        prob_grid_.info.origin.position.y = -(resolution * height) / 2;
        prob_grid_.info.origin.position.z = 0;
        prob_grid_.info.origin.orientation.w = 1;

        prob_grid_.data.resize(width * height);
    }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "floor_plane_mapping");
    FloorPlaneMapping fpm;

    ros::spin();
    return 0;
}
