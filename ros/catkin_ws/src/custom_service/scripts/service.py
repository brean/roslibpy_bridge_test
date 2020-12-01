#!/usr/bin/env python
# -*- coding: utf-8 -*-
# custom ros node providing a service

import rospy
from custom_service.srv import CustomServiceResponse, CustomService
from custom_service.msg import CustomMessage
from std_msgs.msg import String


# Hardcoded data
DATA = {
  "a": 12.34,
  "b": 3.1415
}
SERVICE_NAME = '/my_service'


class ServiceNode(object):
    def __init__(self):
        rospy.Service(
            SERVICE_NAME,
            CustomService,
            self.handle_service)

    def handle_service(self, req):
        rospy.loginfo('service received request: %s' % req)
        data = []
        for name, value in DATA.items():
            custom_data = CustomMessage()
            custom_data.name = name
            custom_data.number = value
            data.append(custom_data)
        return CustomServiceResponse(data)
    
    def start(self):
        rospy.spin()
        rospy.loginfo('service exited.')


if __name__ == '__main__':
    rospy.loginfo('service init')
    rospy.init_node('custom_service', anonymous=True)
    service = ServiceNode()
    rospy.loginfo('service started')
    service.start()

