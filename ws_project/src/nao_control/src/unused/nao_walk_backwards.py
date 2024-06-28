#!/usr/bin/env python

# This is a service to control NAO to walk backwards

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


def handle_NaoWalBackwards(req):

    motionProxy.setWalkArmsEnabled(False, False)
    X = -0.5  #backward
    Y = 0.0
    Theta = 0.0
    Frequency =0.0 # low speed
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)

    time.sleep(4.0)

    X = 0.0
    Y = 0.0
    Theta = 0.0
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
   
    return EmptyResponse()
    




if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('walk_backwards_server')

    #init service naowalkbackwards
    rospy.Service('naowalkbackwards', Empty, handle_NaoWalBackwards)

    rospy.spin()
			
		
