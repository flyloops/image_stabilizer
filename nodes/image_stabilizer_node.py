#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import roslib
import rospy
import cv2
import std_msgs.msg

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from image_stabilizer.srv import ImageStabilizerEnable 
from image_stabilizer.srv import ImageStabilizerEnableResponse

class ImageStabilizerNode(object):

    def __init__(self):
        rospy.init_node('image_stabilizer')
        node_name = rospy.get_name()
        self.threshold = rospy.get_param('{}/threshold'.format(node_name), 90)
        self.input_image = rospy.get_param('{}/input_image'.format(node_name), '/camera/image_mono')
        self.output_image = rospy.get_param('{}/output_image'.format(node_name), '/stabilized_image')
        self.show_contour_image = rospy.get_param('{}/show_contour_image'.format(node_name), True)
        self.show_stabilized_image = rospy.get_param('{}/show_stabilized_image'.format(node_name), True)
        self.enabled = rospy.get_param('{}/enabled'.format(node_name), True)
        rospy.logwarn(self.enabled)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.input_image, Image, self.on_image_callback)
        self.image_pub = rospy.Publisher(self.output_image, Image, queue_size=10)
        self.enable_srv = rospy.Service('image_stabilizer_enable', ImageStabilizerEnable, self.on_enable_srv)

    def on_enable_srv(self,req):
        self.enabled = req.value
        return ImageStabilizerEnableResponse(True)

        
    def on_image_callback(self,orig_imgmsg):
        cv_image = self.bridge.imgmsg_to_cv2(orig_imgmsg,desired_encoding='mono8')

        height, width = cv_image.shape
        image_cvsize = width, height 
        mid_x, mid_y = 0.5*width, 0.5*height

        # Threshold, find contours and get contour with the maximum area
        rval, threshold_image = cv2.threshold(cv_image, self.threshold, np.iinfo(cv_image.dtype).max, cv2.THRESH_BINARY_INV)
        dummy, contour_list, dummy = cv2.findContours(threshold_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        max_contour, max_area = get_max_area_contour(contour_list)

        # Get moments and then compute centroid and shift matrix for stabilization
        moments = cv2.moments(max_contour)
        centroid_x, centroid_y = get_centroid(moments)
        #print(centroid_y, mid_y - centroid_y)
        shift_mat = np.matrix([[1.0, 0.0, (mid_x - centroid_x)], [0.0, 1.0, 1.0*(mid_y - centroid_y)]]) 
        
        # Stabilize image
        stabilized_cv_image = cv2.warpAffine(cv_image, shift_mat, image_cvsize)

        # Publish stabilized image
        stabilized_imgmsg = self.bridge.cv2_to_imgmsg(stabilized_cv_image,"mono8")
        stabilized_imgmsg.header.stamp = orig_imgmsg.header.stamp
        if self.enabled:
            self.image_pub.publish(stabilized_imgmsg)
        else:
            self.image_pub.publish(orig_imgmsg)

        # Plot image w/contours + center of area and stabilized image
        cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(cv_image_bgr,  [max_contour], 0, (0,0,255), 2)
        cv2.circle(cv_image_bgr, (int(centroid_x), int(centroid_y)), 5, (255,0,0), 2)
        if self.show_contour_image:
            cv2.imshow('stabilizer_contour_image', cv_image_bgr)
        if self.show_stabilized_image:
            if self.enabled:
                cv2.imshow('stabilized_image', stabilized_cv_image)
            else:
                cv2.imshow('stabilized_image', cv_image)
        if self.show_stabilized_image or self.show_contour_image:
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


