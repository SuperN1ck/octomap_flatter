#! /usr/bin/env python

import rospy
from octomap_flatter.msg import BoundingBox

if __name__ == '__main__':
    rospy.init_node("bounding_box_detectors")
    # Just to test that it finds everything in runtime
    test_message = BoundingBox()