#!/usr/bin/env python
import numpy as np
import rospy

from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from std_msgs.msg import Float64


class VerticalDistanceCalculator:
    def __init__(self):
        self.pcl_topic = rospy.get_param("pcl_topic", "/vrep/depthSensor")
        self.z_threshold = 0.05

        rospy.init_node('vertical_distance')
        self.sub = rospy.Subscriber(self.pcl_topic, PointCloud2,
                                    self.kinect_cb)
        self.distance_pub = rospy.Publisher('~distance', Float64, queue_size=1)

    def kinect_cb(self, pcl_msg):
        points = []
        min_p = float('inf')
        for p in read_points(
                pcl_msg, field_names=('z'), skip_nans=True):
            if p[0] < min_p:
                min_p = p[0]
        self.distance_pub.publish(min_p)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        VDCalc = VerticalDistanceCalculator()
        VDCalc.run()
    except rospy.ROSInterruptException:
        pass
