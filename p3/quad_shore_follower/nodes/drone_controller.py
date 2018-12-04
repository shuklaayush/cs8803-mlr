#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class DroneController:
    def __init__(self):
        self.velocity = rospy.get_param("velocity", 0.4)
        self.vertical_offset = rospy.get_param("vertical_offset", 0.2)

        self.lateral_sub = rospy.Subscriber("/drone/lateral/control_effort", Float64,
                                    self.lateral_effort_cb)
        self.angular_sub = rospy.Subscriber("/drone/angular/control_effort", Float64,
                                    self.angular_effort_cb)
        self.vertical_sub = rospy.Subscriber("/drone/vertical/state", Float64, self.vertical_cb)

        self.twist_pub = rospy.Publisher("/vrep/drone/cmd_vel", Twist, queue_size=1)
        self.twist = Twist()
        self.twist.linear.x = self.velocity

    def vertical_cb(self, msg):
        self.twist.linear.z = self.vertical_offset + msg.data

    def lateral_effort_cb(self, msg):
        self.twist.linear.y = self.velocity * msg.data

    def angular_effort_cb(self, msg):
        self.twist.angular.z = self.velocity * msg.data

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.twist_pub.publish(self.twist)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("drone_controller")
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
