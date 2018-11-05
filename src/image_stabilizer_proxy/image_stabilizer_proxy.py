#!/usr/bin/env python
from __future__ import print_function
import rospy

from image_stabilizer.srv import ImageStabilizerEnable 


class ImageStabilizerProxyException(Exception):
    pass


class ImageStabilizerProxy(object):

    def __init__(self):
        print('')
        service_name = '/image_stabilizer_enable'
        rospy.wait_for_service(service_name)
        self.enable_proxy = rospy.ServiceProxy(service_name,ImageStabilizerEnable)

    def enable(self,value):
        if not (value == True or value == False):
            raise ImageStabilizerProxyException('value must be True or False')
        rsp = self.enable_proxy(value)
        return rsp.success

