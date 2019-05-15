#!/usr/bin/env python

from octomap_flatter.srv import *
import rospy

import numpy as np
import numpy.ma as ma
import cv2
from cv_bridge import CvBridge, CvBridgeError
import imutils
import scipy

# gets the most likely value from its surround pixels
def get_surround(cont, idx):
    if idx in [(0,0),(0,w-1),(h-1,0),(h-1,w-1)]:
        if idx == (0,0):
            neigh = cont[:2,:2].ravel()
        elif idx == (0,w-1):
            neigh = cont[:2,-2:].ravel()
        elif idx == (h-1,0):
            neigh = cont[-2:,:2].ravel()
        else:
            neigh = cont[-2:,-2:].ravel()
    elif idx[0] in [0, h-1]:
        neigh = cont[idx[0],idx[1]-1:idx[1]+2].ravel()
    elif idx[1] in [0, w-1]:
        neigh = cont[idx[0]-1:idx[0]+2,idx[1]].ravel()
    else:
        neigh = cont[idx[0]-1:idx[0]+2, idx[1]-1:idx[1]+2].ravel()
        neigh = np.delete(neigh,[4])
        for x in neigh:
            if np.sum(neigh == x) == 3 and not (x == 0 or x == 255):
                return x
            elif np.sum(neigh == x) > 1 and not (x == 0 or x == 255):
                return x
    neigh = [x for x in neigh if x != 255]
    return max(neigh)

def flatten(img):
    h, w = np.shape(img)
    mx = int(np.max(img))
    fin = img.copy()

    # pad with some value (200) so that boxes that end outside the image are also considered
    img_pad = np.lib.pad(img, 2, 'constant', constant_values=200)

    # get the edges of the image
    horizontal = scipy.ndimage.sobel(img_pad, 0)
    vertical = scipy.ndimage.sobel(img_pad, 1)
    edge_pad = np.hypot(horizontal, vertical)

    # make all edges value to 255. TODO: How will we choose the threshold?
    edge_pad = cv2.threshold(edge_pad, 5, 255, cv2.THRESH_BINARY)[1]

    # get contours from the images made up of edges
    edge_pad = edge_pad.astype(np.uint8)
    contours = cv2.findContours(edge_pad, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    contours = [c for c in contours if len(c) >= 8]
    contours = sorted(contours, key = cv2.contourArea, reverse = True)

    if len(contours) >= 1:
        col = mx
        # first check the biggest polygon (is it a slope?)
        msk = np.zeros(np.shape(img_pad))
        cv2.fillPoly(msk, pts =[c], color=1)
        for c in contours[1:]:
            cv2.fillPoly(msk, pts =[c], color=0)
        gnd = np.multiply(img_pad,msk)
        k, v = np.unique(gnd, return_counts=True)
        pix_dict = dict(zip(k,v))
        for i in (0,200,255):
            if i in pix_dict:
                del pix_dict[i]
        pix_list = list(pix_dict.items())
        rng = int(pix_list[-1][0] - pix_list[0][0])
        rng //= 5
        peak = int(max(pix_dict, key=pix_dict.get))
        peak_sum = 0
        for i in range(peak-rng,peak+rng+1):
            if i in pix_dict:
                peak_sum += pix_dict[i]
        if not float(peak_sum) / sum(pix_dict.values()) > 0.8:
            contours = contours[1:]

        # fill in the polygon of the edges with the same height (label)
        for c in contours:
            col += 1
            cv2.drawContours(edge_pad, [c], -1, col, 1)
            cv2.fillPoly(edge_pad, pts =[c], color=col)
        
        avg = [[0,0] for box in range(col-mx)]
        edge_fill = edge_pad[2:-2,2:-2]
        
        # sum up the real height values for pixels that belong to the same box
        for idx, val in np.ndenumerate(edge_fill):
            if val == 255:
                val = get_surround(edge_fill, idx)
                edge_fill[idx] = val
            if val > mx:
                avg[val-mx-1][0] += img[idx]
                avg[val-mx-1][1] += 1
        
        # calculate average height of each box
        for a in avg:
            if not a[1] == 0:
                a[0] //= a[1]
        
        # fill in the final image with the average height
        for idx, val in np.ndenumerate(edge_fill):
            if val > mx:
                fin[idx] = avg[val-mx-1][0]
        
        # make ground level 0
        # msk = ma.masked_equal(edge_fill, 0).mask
        # gnd = np.multiply(img,msk)
        # avg = np.sum(gnd) / float(np.sum(msk))
        # if avg <= 1:
        #     fin = np.multiply(fin, ~msk)

        # May be needed later for masking / slopes
	    # msk = ma.masked_where(1 <= edge_fill <= mx, edge_fill).mask
        # unique, counts = np.unique(gnd, return_counts=True)
        # dict(zip(unique, counts))

    return fin

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
print("Starting flatten_octomap_server")

rospy.spin()