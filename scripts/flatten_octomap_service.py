#!/usr/bin/env python

from ransac_lib import *
from octomap_flatter.srv import *
import rospy

import matplotlib;
import matplotlib.pyplot as plt

import numpy as np
import numpy.ma as ma
import cv2
from cv_bridge import CvBridge, CvBridgeError
import imutils
import scipy
import scipy.ndimage

from sklearn.cluster import AgglomerativeClustering, KMeans, MeanShift, estimate_bandwidth
from sklearn.neighbors import kneighbors_graph

import os 

def plot_plane(a, b, c, d, h, w):
	xx, yy = np.mgrid[:h, :w]
	return xx, yy, (-d - a * xx - b * yy) / c

def augment(points):
	aug_pts = np.ones((len(points), 4))
	aug_pts[:, :3] = points
	return aug_pts

def estimate(points):
	aug_pts = augment(points[:3])
	return np.linalg.svd(aug_pts)[-1][-1, :]

def is_inlier(coeffs, point, threshold):
	return np.abs(coeffs.dot(augment([point]).T)) < threshold

def flatten(img):
    points = []
    h, w = np.shape(img)
    for i in range(h):
        for j in range(w):
            if not img[i,j] == 0:
                points.append([i,j,img[i,j]])

    points = np.asarray(points)
    point_list = list(points)

    # RANSAC
    N = len(point_list)
    max_iterations = 100
    goal_inliers = N * 0.3
    m, ic = run_ransac(point_list, estimate, lambda x, y: is_inlier(x, y, 0.0005), 3, goal_inliers, max_iterations)
    gnd_list, obj_list = get_others(point_list, m, lambda x, y: is_inlier(x, y, 0.03))

    a, b, c, d = m
    xx, yy, zz = plot_plane(a, b, c, d, h, w)
    gnd_height = int(round(np.average(zz)))

    if len(gnd_list):
        gnd = np.asarray(gnd_list)
        for g in gnd:
            img[g[0],g[1]] = gnd_height
    if len(obj_list):
        obj = np.asarray(obj_list)
        obj_2d = obj[:,:2]

        if len(obj_list) > N * 0.1:
            #cluster
            connectivity = kneighbors_graph(obj_2d, n_neighbors=50, include_self=False)			
            estimator = AgglomerativeClustering(linkage='ward', connectivity=connectivity)
            estimator.fit(obj_2d)
            labels = estimator.labels_
            cluster_list = [[] for c in range(estimator.n_clusters)]
            for idx, val in enumerate(obj_list):
                cluster_list[labels[idx]].append(val)

            for idx, cluster in enumerate(cluster_list):
                N = len(cluster)
                cluster = np.asarray(cluster)
                estimator = MeanShift(bin_seeding=True)
                estimator.fit(cluster)
                labels = estimator.labels_
                lab_cnt = {}
                for l in labels:
                    if not l in lab_cnt:
                        lab_cnt[l] = 0
                    lab_cnt[l] += 1
                for l in lab_cnt:
                    if lab_cnt[l] / float(N) > 0.2:
                        new_clus = cluster[labels == l]
                        new_height = int(np.average(new_clus[:,2]))
                        for p in new_clus:
                            img[p[0],p[1]] = new_height
    return img

def call_flatten(req):
    # img = np.array(req.input.data).reshape(req.input.height, req.input.width)
    bridge = CvBridge()
    try:
        img = np.array(bridge.imgmsg_to_cv2(req.input, "mono8"))
    except CvBridgeError as e:
        print(e)

    new_img = flatten(img)
    return OctoImageResponse(bridge.cv2_to_imgmsg(new_img, "mono8"))

rospy.init_node('flatten_octomap_server')
s = rospy.Service('flatten_octomap', OctoImage, call_flatten)
rospy.loginfo("Starting flatten_octomap_server")

import sys
print(sys.executable)
rospy.spin()