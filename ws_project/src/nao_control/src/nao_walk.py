#!/usr/bin/env python

# This is a service to control NAO to walk to a specific place but walk can control arms

import rospy
import time
import motion
import numpy as np
import almath
import sys
import cv2
from naoqi import ALProxy
from nao_control.srv import NaoWalk
from geometry_msgs.msg import Twist
from std_srvs.srv import *
motionProxy = 0

# function to enalbe whole body stiffness
def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)


def walk_to(x,y,theta):
    motionProxy.setWalkArmsEnabled(True, True)
    print("[WALK] Robot Move :", x,y,theta)
    motionProxy.post.moveTo(x, y, theta)
    # wait is useful because with post moveTo is not blocking function
    motionProxy.waitUntilMoveIsFinished()


def handle_NaoWalk(req):
    walk_dist = req.dist
    walk_theta = req.theta
    print(walk_dist,walk_theta)
    if (walk_dist == 0):
        walk_to(0,0,walk_theta)
    else:
        walk_to(0,0,walk_theta)
        walk_to(walk_dist,0,0)
    return True

    

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('nao_walk_server')

    #init service naowalk
    rospy.Service('naowalk', NaoWalk, handle_NaoWalk)

    rospy.spin()
			
		
