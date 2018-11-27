#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32, Float64
from math import pi,sqrt


class ArmController:
    def __init__(self):
        self.input_topic = rospy.get_param("input_topic", "/vertical_distance/control_effort")

        rospy.init_node("twist_arm")
        self.sub = rospy.Subscriber(self.input_topic, Float64,
                                    self.input_cb)
        self.twist_pub = rospy.Publisher("/arm_ik/twist", Twist, queue_size=1)
        self.tool_pub = rospy.Publisher('/arm_ik/tool_orientation', Float32, queue_size=1)

    def input_cb(self, msg):
        twist = Twist()
        twist.linear.z = msg.data
        self.tool_pub.publish(Float32(-pi/2))
        self.twist_pub.publish(twist)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = ArmController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
