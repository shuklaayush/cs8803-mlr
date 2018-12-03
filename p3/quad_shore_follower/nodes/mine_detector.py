#!/usr/bin/env python
import rospy
import tf2_ros as tf2

from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from math import sqrt

class MineDetector:
    def __init__(self):
        self.sensor_topic = rospy.get_param("input", "/vrep/metalDetector")
        self.world_frame = rospy.get_param("world_frame", "world")
        self.sensor_frame = rospy.get_param("sensor_frame", "VSV/Tool")

        rospy.init_node("mine_detector")

        self.sensor_thresh = 0.01
        self.distance_thresh = 1.0
        self.tf_buffer = tf2.Buffer()
        self.listener = tf2.TransformListener(self.tf_buffer)

        self.sub = rospy.Subscriber(self.sensor_topic, Float32, self.sensor_cb)
        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1)
        self.mines = list()

    def sensor_cb(self, msg):
        marker_array = MarkerArray()
        for ix, m in enumerate(self.mines):
            mark = Marker()
            mark.header.frame_id = self.world_frame
            mark.type = Marker.CYLINDER
            mark.id = ix
            mark.scale.x = 0.2
            mark.scale.y = 0.2
            mark.scale.z = 0.2
            mark.color.r = 1.0
            mark.color.a = 1.0
            
            mark.pose.position.x = m['x']
            mark.pose.position.y = m['y']
            marker_array.markers.append(mark)
        self.marker_pub.publish(marker_array)

        val = msg.data
        if abs(val - 1.0) < self.sensor_thresh:
            try:
                trans = self.tf_buffer.lookup_transform(self.world_frame, self.sensor_frame, rospy.Time())
            except tf2.LookupException as ex:
                rospy.logwarn(ex)
                return
            except tf2.ExtrapolationException as ex:
                rospy.logwarn(ex)
                return

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            is_unique = True
            for m in self.mines:
                if sqrt((x-m['x'])**2 + (y- m['y'])**2) < self.distance_thresh:
                    m['count'] += 1
                    m['x'] = m['x'] + (x - m['x']) / m['count'] 
                    m['y'] = m['y'] + (y - m['y']) / m['count'] 
                    is_unique = False
            if is_unique:
                self.mines.append({'x': x, 'y': y, 'count': 1})

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        md = MineDetector()
        md.run()
    except rospy.ROSInterruptException:
        pass
