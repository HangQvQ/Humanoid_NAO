#!/usr/bin/env python

# This is a service to control the NAO arms to rotate the bottle

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


def handle_MoveArmsRotate(req):
    arm_joint= ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw","LHand", 
                "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw","RHand"]

    rotate_start = [0.1572060489654541, -0.057627964019775391, -0.6366519927978516, -0.6733839511871338, -0.6013698577880859, 0.8515999913215637, 
                    0.15569999599456787, 0.05072190284729004, 0.6549761295318604, 0.7670419216156006, 0.5261199474334717, 0.9747999906539917]

    rotate_Hands =  [0.15573001861572266, 0.13648414611816406, -0.8069260120391846, -0.6488399505615234, -0.7915859222412109, 0.8515999913215637,  
                     0.155121198654174805, 0.19046411514282227, 0.49697399139404297, 0.7256240844726562, 0.894279956817627, 0.974399983882904]

    rotate_RHand =  [ 0.15573001861572266, 0.13648414611816406, -0.8069260120391846, -0.6488399505615234, -0.7915859222412109, 0.8515999913215637, 
                    0.15569999599456787, -0.202190284729004, 0.49697399139404297, 0.7256240844726562, 0.894279956817627, 0.974399983882904]
          
    time.sleep(2.0)
    fractionMaxSpeed  = 0.03
    motionProxy.setAngles(arm_joint, rotate_start, fractionMaxSpeed)
    time.sleep(2.0)

    motionProxy.setAngles(arm_joint, rotate_Hands, fractionMaxSpeed)
    time.sleep(2.0)

    motionProxy.setAngles(arm_joint, rotate_RHand, fractionMaxSpeed)
    time.sleep(2.0)

    motionProxy.setAngles(arm_joint, rotate_start, fractionMaxSpeed)
    time.sleep(2.0)

    


    return EmptyResponse()
    




if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_arms_rotate_server')

    #init service MoveArmsRotate
    rospy.Service('MoveArmsRotate', Empty, handle_MoveArmsRotate)

    rospy.spin()
			
		
