#!/usr/bin/env python
# -*- coding: utf-8 -*-
# custom ros node to subscribe to a service


import rospy
from std_srvs.srv import Empty
from std_msgs.msg import String
from custom_service.srv import CustomServiceResponse, CustomService
from custom_service.msg import CustomMessage

SERVICE_NAME = '/my_service'


class Robot:
    def __init__(self):
        self.data = {}

    def init(self):
        self.subscribe_service()

    def subscribe_service(self):
        rospy.loginfo('wait for service: %s' % SERVICE_NAME)
        # this never finishes!
        rospy.wait_for_service(SERVICE_NAME, 20)
        rospy.loginfo('service ready, creating proxy!')
        service = rospy.ServiceProxy(
            SERVICE_NAME, CustomService)
        ser = service()
        rospy.loginfo('service response: %s' % ser)
        for msg in ser.msgs:
            self.data[msg.name] = msg.number
        rospy.loginfo('received data from service: %s' % self.data)
        
    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node("robot_node", log_level=rospy.INFO, anonymous=True)
    bot = Robot()
    bot.init()
    bot.spin()