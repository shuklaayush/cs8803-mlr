#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>

class TurtleJoy {
  public:
    TurtleJoy();
  private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh;

    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;

    int l_axis_pos = 5;
    int l_axis_neg = 2;
    int a_axis = 0;

    double l_scale = 1.0;
    double a_scale = 1.0;
};

TurtleJoy::TurtleJoy() {
  ROS_INFO("Starting turtlejoy node with name %s", ros::this_node::getName().c_str());

  vel_pub = nh.advertise<geometry_msgs::Twist>("/vrep/drone/cmd_vel", 1);
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TurtleJoy::joyCallback, this);
}

void TurtleJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  geometry_msgs::Twist twist;
  twist.linear.x = l_scale * (joy->axes[l_axis_neg] - joy->axes[l_axis_pos]) / 2.0;
  // twist.angular.z = a_scale * joy->axes[a_axis];

  ROS_DEBUG("[%s]: vel_x: %f omega_z: %f", ros::this_node::getName().c_str(), twist.linear.x, twist.angular.z);

  vel_pub.publish(twist);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "turtlejoy");
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
  TurtleJoy turtlejoy;

  ros::spin();
}
