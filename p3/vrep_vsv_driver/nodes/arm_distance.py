#!/usr/bin/env python
import numpy as np
import rospy
import tf2_ros as tf2

from math import isinf
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from std_msgs.msg import Float64
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class ArmDistanceCalculator:
    def __init__(self):
        self.pcl_topic = rospy.get_param("pcl_topic", "/vrep/depthSensor")
        self.robot_frame = rospy.get_param("robot_frame", "VSV/ground")

        self.epsilon = 0.01
        self.z_threshold = 0.05
        self.tf_buffer = tf2.Buffer()
        self.listener = tf2.TransformListener(self.tf_buffer)

        rospy.init_node('arm_distance')
        self.sub = rospy.Subscriber(self.pcl_topic, PointCloud2,
                                    self.kinect_cb)
        self.vertical_pub = rospy.Publisher('~vertical/distance', Float64, queue_size=1)
        self.lateral_pub = rospy.Publisher('~lateral/distance', Float64, queue_size=1)

    def kinect_cb(self, pcl_msg):
        min_z = float('inf')
        points = []
        for ix, p in enumerate(read_points(
                pcl_msg, field_names=('x', 'y', 'z'), skip_nans=True)):
            if p[2] < min_z:
                min_z = p[2]
            points.append(p if abs(p[1]) < self.epsilon else [np.nan, np.nan, np.nan])

        points_np = np.array(points)
        indices = points_np[:, 0].argsort()
        points_np = points_np[indices]
        indices = indices[:np.where(np.isnan(points_np[:, 0]))[0][0]]

        self.vertical_pub.publish(min_z)

        source_frame = pcl_msg.header.frame_id
        lookup_time = pcl_msg.header.stamp
        try:
            trans = self.tf_buffer.lookup_transform(self.robot_frame,
                                                    source_frame, lookup_time,
                                                    rospy.Duration(1.0))
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            return
        pcl_robot = do_transform_cloud(pcl_msg, trans)
        points_robot = []
        for ix, p in enumerate(read_points(
                pcl_robot, field_names=('y', 'z'), skip_nans=True)):
            points_robot.append(p)

        points_robot_np = np.array(points_robot)
        points_robot_np[indices]
        above_thresh = points_robot_np[:, 1] > self.z_threshold
        if np.any(above_thresh):
            corner_ix = np.argmax(above_thresh)
            lateral_distance = -points_np[corner_ix][0]
        else:
            lateral_distance = 1.0
        self.lateral_pub.publish(lateral_distance)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        ADCalc = ArmDistanceCalculator()
        ADCalc.run()
    except rospy.ROSInterruptException:
        pass
