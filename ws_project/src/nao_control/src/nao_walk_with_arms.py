#!/usr/bin/env python

# This is a service to control NAO to walk to a specific place wihtout influencing arms

import rospy
import time
import motion
import numpy as np
import almath
import sys
import cv2
from naoqi import ALProxy
from nao_control.srv import NaoWalkWithArms

from geometry_msgs.msg import Twist
from std_srvs.srv import *

# function to enalbe whol body stiffness
def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)


def walk_to(x,y,theta):
    motionProxy.setWalkArmsEnabled(False, False)
    print("[WALK WITH ARM] Robot Move :", x,y,theta)
    motionProxy.post.moveTo(x, y, theta)
    # wait is useful because with post moveTo is not blocking function
    motionProxy.waitUntilMoveIsFinished()


def handle_NaoWalkWithArms(req):
    walk_x = req.x
    walk_y = req.y
    walk_theta = req.theta
    walk_to(walk_x,walk_y,walk_theta)
    return True

    

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('nao_walk_with_arms_server')

    #init service naowalkwitharms
    rospy.Service('naowalkwitharms', NaoWalkWithArms, handle_NaoWalkWithArms)

    rospy.spin()
			
		
