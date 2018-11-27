#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class ArmController:
    def __init__(self):
        self.input_topic = rospy.get_param("input_topic", "/vertical_distance/control_effort")

        rospy.init_node("twist_arm")
        self.sub = rospy.Subscriber(self.input_topic, Float64,
                                    self.input_cb)
        self.twist_pub = rospy.Publisher("/arm_ik/twist", Twist, queue_size=1)

    def input_cb(self, msg):
        twist = Twist()
        twist.linear.z = msg.data
        self.twist_pub.publish(twist)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = ArmController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
