#!/usr/bin/python

# Courtesy: https://gist.githubusercontent.com/rossbar/ebb282c3b73c41c1404123de6cea4771/raw/f34e0a576ddb1a6afdcd4c0d5f44af877c5e6231/yaml_to_camera_info_publisher.py

'''
  @Modifed by: Mahir Gulzar, Tambet Matiisen
'''

# Publishes camera_info from provided yaml file

import rospy
import yaml
import rospkg
from sensor_msgs.msg import CameraInfo
import os


class CameraInfoPublisher:
    def __init__(self):
        package_path = rospkg.RosPack().get_path('camera_info_publisher')
        yaml_fname = rospy.get_param('~yaml_file', 'data/FlirA65.yaml')
        self.yaml_path = os.path.join(package_path, yaml_fname)
        rospy.loginfo(package_path + ':' + self.yaml_path)

        self.publisher = rospy.Publisher(
            'camera_info', CameraInfo, queue_size=10)
        self.rate = rospy.Rate(10)

    def get_camera_info(self, yaml_fname):
        """
        Parse a yaml file containing camera calibration data
        Parameters
        ----------
        yaml_fname : str
            Path to yaml file containing camera calibration data

        Returns
        -------
        camera_info_msg : sensor_msgs.msg.CameraInfo
            A sensor_msgs.msg.CameraInfo message containing the camera calibration
            data
        """
        # Load data from file
        with open(yaml_fname, "r") as file_handle:
            calib_data = yaml.load(file_handle)
        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["intrinsics"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        camera_info_msg.header.frame_id = calib_data["frame_id"]

        return camera_info_msg

    # Run publish loop
    def run(self):
        msg = self.get_camera_info(self.yaml_path)
        while not rospy.is_shutdown():
            msg.header.stamp = rospy.get_rostime()
            self.publisher.publish(msg)
            try:
                self.rate.sleep()
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("Error during sleep: " + str(e))
            except rospy.ROSInterruptException:
                break


if __name__ == "__main__":

    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=False)

    camera_info_publisher = CameraInfoPublisher()
    camera_info_publisher.run()
