#!/usr/bin/env python

import rospy
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

rospy.init_node('mirror_image', anonymous=False)
nodeName = rospy.get_name()

# Name of input topics from launch-file
topic_image_in = rospy.get_param(nodeName + '/topicImageIn', nodeName + '/UnknownInputTopic')
#print("topic_image_in",topic_image_in)

# Name of output topics from launch-file
topic_image_out = rospy.get_param(nodeName + '/topicImageOut', nodeName + '/UnknownOutputTopic')
#print("topic_image_out",topic_image_out)

# Publishers
pub_image = rospy.Publisher(topic_image_out, Image, queue_size=0)

bridge = CvBridge()


def callback_bb(image):
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")

    image_message = bridge.cv2_to_imgmsg(np.fliplr(np.flipud(cv_image)), encoding="passthrough")
    image_message.header.frame_id = 'cam_thermal_frame'
    pub_image.publish(image_message)


# Get subscripers
rospy.Subscriber(topic_image_in, Image, callback_bb, queue_size=None)


# main
def main():
    rospy.spin()


if __name__ == '__main__':
    main()
