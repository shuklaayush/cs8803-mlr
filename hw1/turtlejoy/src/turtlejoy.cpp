#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class TurtleJoy {
    public:
        TurtleJoy();
    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

        ros::NodeHandle nh_;
        
        ros::Publisher vel_pub;
        ros::Subscriber joy_sub;

        int linear_, angular_;
        double l_scale, a_scale;
};

TurtleJoy::TurtleJoy()
    : linear_(1),
      angular_(2) {
    vel_pub = nh_.advertise<geometry_msgs::Twist>("/vrep/twistCommand", 1);
    joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TurtleJoy::joyCallback, this);

    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_linear", l_scale, 0.1);
    nh_.param("scale_angular", a_scale, 0.1);
}

void TurtleJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale * joy->axes[angular_];
    twist.linear.x = l_scale * joy->axes[linear_];
    vel_pub.publish(twist);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlejoy");
    TurtleJoy turtle_joy;
    
    ros::spin();
}
