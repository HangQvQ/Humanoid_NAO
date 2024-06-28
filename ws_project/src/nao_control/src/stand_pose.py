#!/usr/bin/env python

# This is a service to control NAO to stand up.

import rospy
import time
import motion
import numpy as np
import almath
import sys
import cv2
from naoqi import ALProxy
from geometry_msgs.msg import Twist
from std_srvs.srv import *
motionProxy = 0


def handle_standup(req):
    postureProxy.goToPosture("StandInit",0.5)
    return EmptyResponse()
    
if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    rospy.init_node('standup_server')

    #init service Standup
    rospy.Service('StandUp', Empty, handle_standup)

    rospy.spin()
			
		
