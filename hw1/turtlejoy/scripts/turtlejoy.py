#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TurtleJoy:    
    l_axis_pos = 5
    l_axis_neg = 2
    a_axis = 0

    l_scale = 1.0
    a_scale = 1.0

    def joyCallback(self, data):
        twist = Twist()
        twist.linear.x = self.l_scale * (data.axes[self.l_axis_neg] - data.axes[self.l_axis_pos]) / 2.0
        twist.angular.z = self.a_scale * data.axes[self.a_axis] 

        rospy.logdebug("{}: vel_x: {: .2f} omega_z: {: .2f}".format(rospy.get_name(), twist.linear.x, twist.angular.z))
        self.vel_pub.publish(twist)

    def __init__(self):
        rospy.loginfo("Starting turtlejoy node with name {}".format(rospy.get_name()))

        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joyCallback)
        self.vel_pub = rospy.Publisher("/vrep/twistCommand", Twist, queue_size=1)

if __name__ == '__main__':
    rospy.init_node("turtlejoy", log_level=rospy.DEBUG)
    tj = TurtleJoy()
    rospy.spin()
