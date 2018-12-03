#!/usr/bin/env python
import numpy as np
import rospy
import tf2_ros as tf2
import matplotlib.pyplot as plt

from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from std_msgs.msg import Float64
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class LateralDistanceCalculator:
    def __init__(self):
        self.pcl_topic = rospy.get_param("laser_topic", "/vrep/hokuyoSensor")
        self.robot_frame = rospy.get_param("robot_frame", "drone")
        self.world_frame = rospy.get_param("world_frame", "world")

        rospy.init_node('lateral_distance')

        self.z_threshold = 0.05
        self.tf_buffer = tf2.Buffer()
        self.listener = tf2.TransformListener(self.tf_buffer)

        self.sub = rospy.Subscriber(self.pcl_topic, PointCloud2,
                                    self.hokuyo_cb)
        self.distance_pub = rospy.Publisher('~distance', Float64, queue_size=1)

    def plot_distance(self, points):
        points_np = np.array(points)
        plt.ion()
        ax = plt.gca()
        ax.set_xlim([-10.0, 0.0])
        ax.set_ylim([-0.1, 0.5])
        ax.plot(points_np[:, 1], points_np[:, 2])
        plt.pause(0.0001)
        plt.clf()

    def hokuyo_cb(self, pcl_msg):
        source_frame = pcl_msg.header.frame_id
        lookup_time = pcl_msg.header.stamp
        try:
            trans_robot = self.tf_buffer.lookup_transform(self.robot_frame,
                                                    source_frame, lookup_time,
                                                    rospy.Duration(1.0))
            trans_world = self.tf_buffer.lookup_transform(self.world_frame,
                                                    source_frame, lookup_time,
                                                    rospy.Duration(1.0))
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            return
        pcl_robot = do_transform_cloud(pcl_msg, trans_robot)
        pcl_world = do_transform_cloud(pcl_msg, trans_world)
        points = []
        points_robot = read_points(pcl_robot, field_names=('x', 'y'), skip_nans=True)
        points_world = read_points(pcl_world, field_names=('z'), skip_nans=True)
        for p, p_w in zip(points_robot, points_world):
            if p_w[0] > self.z_threshold:
                corner_point = p
                points.append(p)
        self.plot_distance(points)
        try:
            self.distance_pub.publish(-corner_point[1])
        except NameError:
            return

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        LDCalc = LateralDistanceCalculator()
        LDCalc.run()
    except rospy.ROSInterruptException:
        pass
