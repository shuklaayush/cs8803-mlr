#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

namespace {
using std::abs;
using std::cout;
using std::sqrt;
using std::vector;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

constexpr auto EPSILON = 0.1;
constexpr auto MIN_HEIGHT_THRESH = 0.05;
constexpr auto RADIUS_MAX = 1.0;
constexpr auto VOTE_THRESH = 1000;

double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
}  // namespace

class Circle {
private:
    double x_;
    double y_;
    double r_;

public:
    int vote_ = 0;

public:
    Circle(double x, double y, double r) : x_(x), y_(y), r_(r){};
    Marker to_marker() {
        Marker m;
        m.type = Marker::CYLINDER;
        m.pose.position.x = x_;
        m.pose.position.y = y_;
        m.pose.position.z = 0;

        m.scale.x = 2 * r_;
        m.scale.y = 2 * r_;
        m.scale.z = 2;

        m.color.a = 0.5;
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;

        return m;
    }

    void merge(const Circle &c) {
        if (c.vote_ > vote_) {
            x_ = c.x_;
            y_ = c.y_;
            r_ = c.r_;
        }
    }

    double error(double x, double y) {
        return sqrt(abs((x - x_) * (x - x_) + (y - y_) * (y - y_) - r_ * r_));
    }

    bool contains(const Circle &c) {
        auto d = sqrt((x_ - c.x_) * (x_ - c.x_) + (y_ - c.y_) * (y_ - c.y_));
        return d < r_ + c.r_;
    }
};

// Reference: http://www.ambrsoft.com/trigocalc/circle3d.htm
class CylinderDetectorRansac {
protected:
    ros::Subscriber scan_sub_;
    ros::Publisher marker_pub_;
    tf::TransformListener listener_;

    ros::NodeHandle nh_;
    std::string base_frame_;
    std::string world_frame_;

    double max_range_;
    double tolerance;
    int n_samples;

    pcl::PointCloud<pcl::PointXYZ> pcl_base_;
    pcl::PointCloud<pcl::PointXYZ> pcl_world_;

    vector<Circle> cylinders;

protected:  // ROS Callbacks
    void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> temp;
        pcl::fromROSMsg(*msg, temp);
        // Make sure the point cloud is in the base-frame
        listener_.waitForTransform(base_frame_, msg->header.frame_id,
                                   msg->header.stamp, ros::Duration(1.0));
        pcl_ros::transformPointCloud(base_frame_, msg->header.stamp, temp,
                                     msg->header.frame_id, pcl_base_,
                                     listener_);

        listener_.waitForTransform(world_frame_, msg->header.frame_id,
                                   msg->header.stamp, ros::Duration(1.0));
        pcl_ros::transformPointCloud(world_frame_, msg->header.stamp, temp,
                                     msg->header.frame_id, pcl_world_,
                                     listener_);

        auto n = temp.size();
        vector<unsigned int> pidx;
        // First count the useful points
        for (auto i = 0U; i < n; i++) {
            double x = temp[i].x;
            double y = temp[i].y;
            double d = hypot(x, y);
            if (d < 1e-2) {
                // Bogus point, ignore
                continue;
            }
            x = pcl_base_[i].x;
            y = pcl_base_[i].y;
            double z = pcl_base_[i].z;
            d = hypot(x, y);
            if (d > max_range_ || z < MIN_HEIGHT_THRESH) {
                // too far, ignore
                continue;
            }
            pidx.push_back(i);
        }

        n = pidx.size();
        ROS_INFO("%lu useful points out of %lu", n, temp.size());
        if (n_samples < static_cast<int>(n)) {
            for (auto i = 0; i < n_samples; ++i) {
                // Instead of selecting three random points, shuffle pcl and
                // select first three points. Don't have to worry about
                // picking same point again.
                std::random_shuffle(pidx.begin(), pidx.end());

                double x1 = pcl_world_[pidx[0]].x;
                double y1 = pcl_world_[pidx[0]].y;

                double x2 = pcl_world_[pidx[1]].x;
                double y2 = pcl_world_[pidx[1]].y;

                double x3 = pcl_world_[pidx[2]].x;
                double y3 = pcl_world_[pidx[2]].y;

                if (distance(x1, y1, x2, y2) < EPSILON ||
                    distance(x2, y2, x3, y3) < EPSILON ||
                    distance(x3, y3, x1, y1) < EPSILON) {
                    continue;
                }

                double A = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2;
                double B = (x1 * x1 + y1 * y1) * (y3 - y2) +
                           (x2 * x2 + y2 * y2) * (y1 - y3) +
                           (x3 * x3 + y3 * y3) * (y2 - y1);
                double C = (x1 * x1 + y1 * y1) * (x2 - x3) +
                           (x2 * x2 + y2 * y2) * (x3 - x1) +
                           (x3 * x3 + y3 * y3) * (x1 - x2);
                double D = (x1 * x1 + y1 * y1) * (x3 * y2 - x2 * y3) +
                           (x2 * x2 + y2 * y2) * (x1 * y3 - x3 * y1) +
                           (x3 * x3 + y3 * y3) * (x2 * y1 - x1 * y2);

                double Cx = -B / (2 * A);
                double Cy = -C / (2 * A);
                double r = sqrt((B * B + C * C - 4 * A * D) / (4 * A * A));

                if (r > RADIUS_MAX) {
                    continue;
                }

                auto xmean = (x1 + x2 + x3) / 3.0;
                auto ymean = (y1 + y2 + y3) / 3.0;

                tf::StampedTransform world_to_base;
                listener_.lookupTransform(world_frame_, base_frame_,
                                         ros::Time(0), world_to_base);
                double xb = world_to_base.getOrigin().x();
                double yb = world_to_base.getOrigin().y();
                if (distance(xb, yb, xmean, ymean) >
                    distance(xb, yb, Cx, Cy)) {
                    continue;
                }
                Circle c(Cx, Cy, r);
                // Iterate over all points
                for (auto j = 0U; j < n; ++j) {
                    double x = pcl_world_[pidx[j]].x;
                    double y = pcl_world_[pidx[j]].y;
                    if (c.error(x, y) < tolerance) {
                        ++c.vote_;
                    }
                }
                if (c.vote_ > VOTE_THRESH) {
                    // Check stored markers
                    bool is_better = true;
                    for (auto &it : cylinders) {
                        if (it.contains(c)) {
                            if (c.vote_ > it.vote_) {
                                it.vote_ = 0;
                            } else {
                                is_better = false;
                            }
                        }
                    }
                    cylinders.erase(
                        std::remove_if(
                            cylinders.begin(), cylinders.end(),
                            [](const Circle &c) { return c.vote_ == 0; }),
                        cylinders.end());
                    if (is_better) {
                        cylinders.push_back(c);
                    }
                }
            }
        }
        MarkerArray MA;
        for (auto i = 0U; i < cylinders.size(); ++i) {
            auto m = cylinders[i].to_marker();
            m.id = i;
            m.header.frame_id = world_frame_;
            MA.markers.push_back(m);
        }
        marker_pub_.publish(MA);
    }

public:
    CylinderDetectorRansac() : nh_("~") {
        nh_.param("world_frame", world_frame_, std::string("/world"));
        nh_.param("base_frame", base_frame_, std::string("/bubbleRob"));
        nh_.param("max_range", max_range_, 4.0);
        nh_.param("n_samples", n_samples, 2000);
        nh_.param("tolerance", tolerance, 0.02);

        ROS_INFO("RANSAC: %d iteration with %f tolerance", n_samples,
                 tolerance);

        // Make sure TF is ready
        ros::Duration(0.5).sleep();

        scan_sub_ = nh_.subscribe("/vrep/depthSensor", 1,
                                  &CylinderDetectorRansac::pc_callback, this);
        marker_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>("cylinders", 1);
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "cylinder_detector_ransac");
    CylinderDetectorRansac cd;

    ros::spin();
    return 0;
}
