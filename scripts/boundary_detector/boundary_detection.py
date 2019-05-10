#! /usr/bin/env python

import rospy
from octomap_flatter.msg import BoundingBox

import rospy

class BoundaryDetection:
    def __init__(self):
        self.current_boundaries = []

    def process(image, camera_info):
        pass