#!/usr/bin/env python

import roslib; roslib.load_manifest('vrep_vsv_driver')
import rospy
import copy
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist, Vector3, Point, PointStamped
from std_msgs.msg import Float32
from math import pi,sqrt
import tf
import random
from tf.transformations import euler_from_quaternion

from vrep_vsv_driver.arm_ik import *



class VSVTestPointStamped:
    def __init__(self):
        self.publisher = rospy.Publisher("~position_stamped",PointStamped,queue_size=1)
        self.pose_pub = rospy.Publisher('~position_command', Point, queue_size=1)
        self.tool_pub = rospy.Publisher('~tool_command', Float32, queue_size=1)
        self.listener = tf.TransformListener()
        rospy.sleep(1.0) # Necessary to let the TF arrive
        if rospy.is_shutdown():
            return
        for i in range(10):
            self.tool_pub.publish(Float32(-pi/2))
            self.pose_pub.publish(Point(2.0,0.0,-0.5))
            rospy.sleep(0.1) # Necessary to let the TF arrive

    def run(self):
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            ((a,b,c),_) = self.listener.lookupTransform( '/world', '/VSV/Tool',rospy.Time(0))
            P = PointStamped()
            P.header.frame_id = "/world"
            P.header.stamp = rospy.Time.now()
            P.point = Point(a+0.1*(2*random.random()-1),b+0.1*(2*random.random()-1),c+0.1*(2*random.random()-1))
            #print(P.point)
            self.publisher.publish(P)
            rate.sleep()

        
if __name__ == '__main__':
    try:
        rospy.init_node('test_point_stamped')

        test = VSVTestPointStamped()
        test.run()
        
    except rospy.ROSInterruptException:
        pass
