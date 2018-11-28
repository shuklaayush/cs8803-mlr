#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32, Float64
from math import pi,sqrt


class ArmController:
    def __init__(self):
        self.input_vertical = rospy.get_param("input_vertical", "/arm_distance/vertical/control_effort")
        self.input_lateral = rospy.get_param("input_lateral", "/arm_distance/lateral/control_effort")

        rospy.init_node("twist_arm")
        self.lateral = 0.0
        self.vertical = 0.0
        self.sub = rospy.Subscriber(self.input_vertical, Float64,
                                    self.vertical_cb)
        self.sub = rospy.Subscriber(self.input_lateral, Float64,
                                    self.lateral_cb)
        self.twist_pub = rospy.Publisher("/arm_ik/twist", Twist, queue_size=1)
        self.tool_pub = rospy.Publisher('/arm_ik/tool_orientation', Float32, queue_size=1)

    def vertical_cb(self, msg):
        self.vertical = msg.data

    def lateral_cb(self, msg):
        self.lateral = msg.data

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = self.lateral
            twist.linear.z = self.vertical
            self.tool_pub.publish(Float32(-pi/2))
            self.twist_pub.publish(twist)
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = ArmController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
