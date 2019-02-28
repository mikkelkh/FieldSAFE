#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
  @Author: Mahir Gulzar, Tambet Matiisen
'''

import rospy
import numpy as np
import os
import rospkg
from visualization_msgs.msg import Marker
#from std_msgs.msg import Header
#import tf


class DynamicVisualizer:
    def __init__(self):
        self.dynamic_pub = rospy.Publisher(
            'dynamic_markers', Marker, queue_size=10)

        #self.listener = tf.TransformListener()
        self.package_path = rospkg.RosPack().get_path('ground_truth')

        # load dynamic ground truth
        self.dynamic_markers = np.loadtxt(
            os.path.join(self.package_path,
                         'data/dynamic_ground_truth.txt'),
            delimiter=',', skiprows=1, usecols=(0, 1, 2, 4, 3),
            dtype={'names': ('track_id', 'x', 'y', 'timestamp', 'frame'),
                   'formats': (np.uint8, np.uint32, np.uint32, np.float64, np.uint32)})

        # sort dynamic ground truth by timestamp, inplace
        self.dynamic_markers.sort(order='timestamp')

        # load pixel to utm transform
        utm2pixels = np.loadtxt(os.path.join(
            self.package_path, 'data/utm2PixelsTransformMatrix.csv'), delimiter=',')
        self.pixels2utm = np.linalg.inv(utm2pixels)

    # Run publish loop
    def run(self):
        '''
        # wait until the transforms are available
        self.listener.waitForTransform(
            'utm', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        '''
        for track_id, x, y, timestamp, frame in self.dynamic_markers:

            # skip drone
            if track_id == 4:
                continue

            # convert current position in pixels to utm
            pos = np.array([y, x, 1])
            pos = np.dot(self.pixels2utm, pos)

            # create marker
            marker = Marker()
            marker.header.frame_id = 'utm'
            marker.header.stamp = rospy.Time.from_sec(timestamp)
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.id = track_id
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1.5
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = 63.0
            '''
            # look up transform between base_frame and UTM at the moment of publishing
            try:
                matrix = self.listener.asMatrix('utm', Header(stamp=rospy.Time(0), frame_id='base_link'))
                pos = matrix.dot([0, 0, 1, 1])
                marker.pose.position.z = pos[2]
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as e:
                rospy.logwarn("Error looking up transform: " + str(e))
            '''

            # check if current stamp is ahead of ros current_time
            current_time = rospy.get_time()
            if timestamp > current_time:
                # sleep until it is the right moment to publish
                try:
                    rospy.sleep(timestamp - current_time)
                except rospy.ROSTimeMovedBackwardsException as e:
                    rospy.logwarn("Error during sleep: " + str(e))
                except rospy.ROSInterruptException:
                    # stop silently when ROS is stopped
                    break

            # publish the marker
            self.dynamic_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node('dynamic_visualizer', anonymous=False)
    dynamic_visualizer = DynamicVisualizer()
    dynamic_visualizer.run()
