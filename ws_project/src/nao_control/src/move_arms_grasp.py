#!/usr/bin/env python

# this is a service to control the NAO arms to grasp the bottle

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


def handle_MoveArmsGrasp(req):

    arm_joint = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw","LHand", 
                "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw","RHand"]

    angles_grasp =  [0.120949392318725586, -0.15921796417236328, -0.7624399662017822, -0.6687819957733154, -0.8222661018371582, 0.8519999980926514, 
                    0.12093796730041504, 0.159161949157714844, 0.6534421443939209, 0.7731781005859375, 0.9249601364135742, 0.974399983882904]

    time.sleep(2.0)
    fractionMaxSpeed  = 0.03
    motionProxy.setAngles(arm_joint, angles_grasp, fractionMaxSpeed)
    motionProxy.waitUntilMoveIsFinished()
    time.sleep(2.0)
    
    return EmptyResponse()
    




if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_arms_grasp_server')

    #init service MoveArmsGrasp
    rospy.Service('MoveArmsGrasp', Empty, handle_MoveArmsGrasp)

    rospy.spin()
			
		
