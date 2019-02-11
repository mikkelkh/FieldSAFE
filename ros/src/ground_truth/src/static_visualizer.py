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


class StaticVisualizer:
    def __init__(self):
        self.static_pub = rospy.Publisher(
            'static_markers', Marker, queue_size=10)

        #self.listener = tf.TransformListener()
        self.package_path = rospkg.RosPack().get_path('ground_truth')

        # load static markers
        self.static_markers = np.loadtxt(
            os.path.join(self.package_path,
                         'data/gps_marker_positions.csv'),
            delimiter=';', skiprows=1, usecols=(0, 3, 4, 5))

    # Run publish loop
    def run(self):
        '''
        # wait until the transforms are available
        self.listener.waitForTransform(
            'utm', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        '''

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            for gcp, utm_x, utm_y, altitude in self.static_markers:
                marker = Marker()
                marker.header.frame_id = 'utm'
                marker.type = marker.CUBE
                marker.action = marker.ADD
                marker.id = gcp
                marker.scale.x = 2
                marker.scale.y = 2
                marker.scale.z = 10
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = utm_x
                marker.pose.position.y = utm_y
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

                self.static_pub.publish(marker)

            try:
                r.sleep()
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("Error during sleep: " + str(e))
            except rospy.ROSInterruptException:
                # stop silently when ROS is stopped
                break


if __name__ == "__main__":
    rospy.init_node('static_visualizer', anonymous=False)
    static_visualizer = StaticVisualizer()
    static_visualizer.run()
