#include <unistd.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>

class PointcloudNoisify {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher scan_pub_;

        ros::NodeHandle nh_;
        double noise_;


    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            for (unsigned int i=0;i<temp.size();i++) {
                temp[i].x += (-1 + 2* drand48()) * noise_;
                temp[i].y += (-1 + 2* drand48()) * noise_;
                temp[i].z += (-1 + 2* drand48()) * noise_;
            }
            
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(temp,out);
            scan_pub_.publish(out);
            
        }

    public:
        PointcloudNoisify() : nh_("~") {
            nh_.param("noise",noise_,0.1);
            // END OF TODO

            scan_sub_ = nh_.subscribe("input",1,&PointcloudNoisify::pc_callback,this);
            scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output",1);

        }

};

int main(int argc, char * argv[]) 
{
    srand48(time(NULL));
    ros::init(argc,argv,"pointcloud_noisify");
    PointcloudNoisify fp;

    ros::spin();
    return 0;
}


