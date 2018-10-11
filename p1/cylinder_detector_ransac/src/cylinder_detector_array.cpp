#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


class CylinderDetectorArray {
	public: 
		CylinderDetectorArray () ; 
	protected: 
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        ros::NodeHandle nh_; 
		void callback (const visualization_msgs::Marker* MarkerPtr) ; 
	};

CylinderDetectorArray::CylinderDetectorArray () {
	ROS_INFO("Reader ON !") ; 
	ros::Duration(0.5).sleep();
	scan_sub_ = nh_.subscribe("/floor_plane",1,&CylinderDetectorArray::callback,this);
	marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/Marker_Arrays",1);
	};

void CylinderDetectorArray::callback(const visualization_msgs::Marker* MarkerPtr){
	visualization_msgs::MarkerArray Arr ; 
	Arr.markers.push_back (MarkerPtr) ; 
	marker_pub_.publish(Arr); 	
	};
	
int main (int argc , char** argv) 
{
  ros::init(argc,argv,"Cylinder_Detector_Array");
  CylinderDetectorArray Cylinder_Detector_Array;
  ros::spin() ; 
}
