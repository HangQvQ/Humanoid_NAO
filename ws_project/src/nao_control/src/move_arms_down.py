#!/usr/bin/env python

# this is a service to control the NAO arms to hold the bottle down at lower position

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


def handle_MoveArmsDown(req):

    arm_joint= ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw","LHand", 
                "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw","RHand"]

    arm_grasp = [1.3560140132904053, -0.233338737487793, -1.0048117637634277, -1.0737581253051758, -0.257753849029541, 0.8467999696731567, 
                1.3560140132904053, 0.2333404083251953, 1.0019501209259033, 1.055723762512207, 0.25368995666503906, 0.8479999876022339]
    
          
    time.sleep(2.0)
    fractionMaxSpeed  = 0.03
    
    motionProxy.setAngles(arm_joint, arm_grasp, fractionMaxSpeed)
    motionProxy.waitUntilMoveIsFinished()
    time.sleep(2.0)
    return EmptyResponse()
    
    
    
if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_arms_down_server')

    #init service MoveArmsDown
    rospy.Service('MoveArmsDown', Empty, handle_MoveArmsDown)

    rospy.spin()
			
		
