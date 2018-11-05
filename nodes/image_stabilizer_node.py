#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import roslib
import rospy
import cv2
import std_msgs.msg

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageStabilizerNode(object):

    def __init__(self):
        self.threshold = rospy.get_param('stabilizer_threshold', 90)
        self.mask_scale = rospy.get_param('stabilizer_mask_scale', 0.95)
        self.input_image = rospy.get_param('stabilizer_input_image', '/camera/image_mono')
        self.output_image = rospy.get_param('stabilizer_output_image', '/stabilized_image')
        self.bridge = CvBridge()

        rospy.init_node('image_stabilizer')
        self.image_sub = rospy.Subscriber(self.input_image, Image, self.on_image_callback)
        self.image_pub = rospy.Publisher(self.output_image, Image, queue_size=10)
        
    def on_image_callback(self,data):
        cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding='mono8')

        height, width = cv_image.shape
        image_cvsize = width, height 
        mid_x, mid_y = 0.5*width, 0.5*height

        # Compute cirlce mask
        mask_radius = int(self.mask_scale*height/2.0)
        vals_x = np.arange(0.0,width)
        vals_y = np.arange(0.0,height)
        grid_x, grid_y = np.meshgrid(vals_x, vals_y)
        circ_mask = (grid_x - width/2.0 + 0.5)**2 + (grid_y - height/2.0 + 0.5)**2 < (mask_radius)**2

        # Threshold, find contours and get contour with the maximum area
        rval, threshold_image = cv2.threshold(cv_image, self.threshold, np.iinfo(cv_image.dtype).max, cv2.THRESH_BINARY_INV)
        dummy, contour_list, dummy = cv2.findContours(threshold_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        max_contour, max_area = get_max_area_contour(contour_list)

        # Get moments and then compute centroid and shift matrix for stabilization
        moments = cv2.moments(max_contour)
        centroid_x, centroid_y = get_centroid(moments)
        print(centroid_y, mid_y - centroid_y)
        shift_mat = np.matrix([[1.0, 0.0, (mid_x - centroid_x)], [0.0, 1.0, 1.0*(mid_y - centroid_y)]]) 
        
        # Stabilize image and apply mask
        stabilized_cv_image = cv2.warpAffine(cv_image, shift_mat, image_cvsize)
        #stabilized_cv_image = stabilized_cv_image*circ_mask

        # Publish stabilized image
        stabilized_imgmsg = self.bridge.cv2_to_imgmsg(stabilized_cv_image,"mono8")
        stabilized_imgmsg.header.stamp = data.header.stamp
        self.image_pub.publish(stabilized_imgmsg)

        cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(cv_image_bgr,  [max_contour], 0, (0,0,255), 2)
        cv2.circle(cv_image_bgr, (int(centroid_x), int(centroid_y)), 5, (0,0,255), 2)

        #cv2.imshow('raw_image', cv_image)
        #cv2.imshow('thredhold', threshold_image)
        cv2.imshow('stabilizer_contour_image', cv_image_bgr)
        cv2.imshow('stabilized_image', stabilized_cv_image)
        cv2.waitKey(1)


    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

# Utility functions
# -------------------------------------------------------------------------------------------------

def get_max_area_contour(contour_list):
    """
    Given a list of contours finds the contour with the maximum area and 
    returns 
    """
    contour_areas = np.array([cv2.contourArea(c) for c in contour_list])
    max_area = contour_areas.max()
    max_ind = contour_areas.argmax()
    max_contour = contour_list[max_ind]
    return max_contour, max_area


def get_centroid(moments): 
    """
    Computer centroid given the image/blob moments
    """
    if moments['m00'] > 0:
        centroid_x = moments['m10']/moments['m00']
        centroid_y = moments['m01']/moments['m00']
    else:
        centroid_x = 0.0
        centroid_y = 0.0
    return centroid_x, centroid_y

# -------------------------------------------------------------------------------------------------

if __name__ == '__main__':

    node = ImageStabilizerNode()
    node.run()


