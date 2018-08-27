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

};

TurtleJoy::TurtleJoy() {
    vel_pub = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TurtleJoy::joyCallback, this);
}

void TurtleJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    geometry_msgs::Twist twist;
    twist.angular.z = 1;
    twist.linear.x = 1;
    vel_pub.publish(twist);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlejoy");
    TurtleJoy turtle_joy;
    
    ros::spin();
}
