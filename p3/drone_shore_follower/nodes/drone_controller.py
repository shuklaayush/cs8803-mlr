#!/usr/bin/env python
import rospy
import message_filters

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from math import sqrt


class DroneController:
    def __init__(self):
        self.velocity = rospy.get_param("velocity", 0.4)
        self.vertical_offset = rospy.get_param("vertical_offset", 0.2)

        lateral_sub = message_filters.Subscriber("/drone/lateral/control_effort", Float64)
        vertical_sub = message_filters.Subscriber("/drone/vertical/state", Float64)
        angular_sub = message_filters.Subscriber("/drone/angular/control_effort", Float64)

        ats = message_filters.ApproximateTimeSynchronizer([lateral_sub, vertical_sub, angular_sub], queue_size=1, slop=1.0, allow_headerless=True)
        ats.registerCallback(self.control_cb)

        self.twist_pub = rospy.Publisher("/vrep/drone/cmd_vel", Twist, queue_size=1)

    def control_cb(self, msg1, msg2, msg3):
        twist = Twist()
        twist.linear.y = self.velocity * msg1.data
        twist.linear.x = self.velocity * sqrt(1 - msg1.data*msg1.data) 
        twist.linear.z = self.vertical_offset + msg2.data
        twist.angular.z = self.velocity * msg3.data
        
        self.twist_pub.publish(twist)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("drone_controller")
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
