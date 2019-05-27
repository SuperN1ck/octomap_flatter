#!/usr/bin/env python
"""
Purpose of the file: subscribe to a topic called /image_raw of type sensor_msgs/Image
Apply filter to the resulting image
"""
from __future__ import print_function
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

class SubThenFilter:
    def __init__(self, sub_topic, pub_topic):
        self.sub = rospy.Subscriber(sub_topic, Image, self.image_callback, queue_size=1)
        self.pub = rospy.Publisher(pub_topic, Image, queue_size=1)
        self.bridge = CvBridge()
        self.median_blur_size = 5
        self.use_median_blur = True

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"passthrough")
        except CvBridgeError as e:
            print(e)

        cv_image = np.nan_to_num(cv_image)
        if self.use_median_blur:
            cv_image = cv2.medianBlur(cv_image, self.median_blur_size)

        try:
            msg = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
            data.data = msg.data
            data.header.frame_id = "camera_depth_optical_frame_estimate"
            self.pub.publish(data)
        except CvBridgeError as e:
            print(e)

if __name__ == "__main__":
    if len(sys.argv) == 5:
        sub_topic = str(sys.argv[1])
        pub_topic = str(sys.argv[2])

        rospy.init_node("filter_depth_server")
        rospy.loginfo("Starting filter_depth_server. Subscribed from {}, Publish to {}".format(sub_topic, pub_topic))
        sf = SubThenFilter(sub_topic, pub_topic)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("shutting down")
    else:
        rospy.loginfo("Need two inputs: pointcloud subscribe topic and publish topic")
        sys.exit(1)

cv2.destroyAllWindows()