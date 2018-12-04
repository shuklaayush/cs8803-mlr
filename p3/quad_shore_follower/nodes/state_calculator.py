#!/usr/bin/env python
import numpy as np
import rospy
import tf2_ros as tf2
import message_filters
import matplotlib.pyplot as plt

from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from std_msgs.msg import Float64
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class StateCalculator:
    def __init__(self):
        self.laser1 = rospy.get_param("laser1_topic", "/vrep/drone/hokuyo1")
        self.laser2 = rospy.get_param("laser2_topic", "/vrep/drone/hokuyo2")
        self.robot_frame = rospy.get_param("robot_frame", "drone")
        self.world_frame = rospy.get_param("world_frame", "world")

        self.z_threshold = 0.05
        self.tf_buffer = tf2.Buffer()
        self.listener = tf2.TransformListener(self.tf_buffer)

        laser1_sub = message_filters.Subscriber(self.laser1, PointCloud2)
        laser2_sub = message_filters.Subscriber(self.laser2, PointCloud2)
        ats = message_filters.ApproximateTimeSynchronizer([laser1_sub, laser2_sub], queue_size=1, slop=0.1)
        ats.registerCallback(self.laser_cb)
        self.lateral_pub = rospy.Publisher('drone/lateral/state', Float64, queue_size=1)
        self.angular = rospy.Publisher('drone/angular/state', Float64, queue_size=1)

    def plot(self, points):
        points_np = np.array(points)
        plt.ion()
        ax = plt.gca()
        ax.set_xlim([-1.0, 1.0])
        ax.set_ylim([-0.05, 0.2])
        ax.plot(points_np[:, 0], points_np[:, 1])
        plt.pause(0.0001)
        plt.clf()

    def laser_cb(self, pcl_msg1, pcl_msg2):
        def find_edge(pcl_msg):
            source_frame = pcl_msg.header.frame_id
            lookup_time = pcl_msg.header.stamp
            try:
                trans_robot = self.tf_buffer.lookup_transform(self.robot_frame,
                                                        source_frame, lookup_time,
                                                        rospy.Duration(5.0))
                trans_world = self.tf_buffer.lookup_transform(self.world_frame,
                                                        source_frame, lookup_time,
                                                        rospy.Duration(5.0))
            except (tf2.LookupException, tf2.ExtrapolationException) as ex:
                rospy.logwarn(ex)
                return
            pcl_robot = do_transform_cloud(pcl_msg, trans_robot)
            pcl_world = do_transform_cloud(pcl_msg, trans_world)
            points_robot = read_points(pcl_robot, field_names=('x', 'y'), skip_nans=True)
            points_world = read_points(pcl_world, field_names=('z'), skip_nans=True)
            #  points = []
            for p, p_w in zip(points_robot, points_world):
                if p_w[0] > self.z_threshold:
                    corner_point = p
                #  points.append([p[1], p_w[0]])
            #  self.plot(points)
            return corner_point

        x1, y1 = find_edge(pcl_msg1)
        x2, y2 = find_edge(pcl_msg2)
 
        self.lateral_pub.publish((y1+y2) / 2)
        self.angular.publish((y1-y2) / (x1-x2))

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('state_calculator')
    try:
        state_calc = StateCalculator()
        state_calc.run()
    except rospy.ROSInterruptException:
        pass
