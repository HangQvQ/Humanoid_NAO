#!/usr/bin/env python

# this is a service to control the NAO arms to move to the pre-grasp position

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


def handle_MoveArmsOpen(req):


    arm_joint= ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw","LHand", 
                "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw","RHand"]


    angles_open_1 =  [1.320949392318725586, 1.3000335798263549805, -0.27309393882751465, -0.38345813751220703, -1.3637681007385254, 0.8500000238418579, 
                    1.320093796730041504, -1.3000713186264038086, 0.6534421443939209, 0.7731781005859375, 0.9249601364135742, 0.974399983882904]

    angles_open_2 =  [0.120949392318725586, 1.3000335798263549805, -0.27309393882751465, -0.38345813751220703, -1.3637681007385254, 0.8500000238418579, 
                    0.120093796730041504, -1.3000713186264038086, 0.6534421443939209, 0.7731781005859375, 0.9249601364135742, 0.974399983882904]

    angles_open_3 =  [0.020949392318725586, 0.20335798263549805, -0.7624399662017822, -0.6687819957733154, -0.8222661018371582, 0.8519999980926514, 
                    0.020093796730041504, -0.20713186264038086, 0.6549761295318604, 0.41728997230529785, 1.8238691091537476, 0.972000002861023]




          
    fractionMaxSpeed  = 0.03
    motionProxy.setAngles(arm_joint, angles_open_1, fractionMaxSpeed)
    time.sleep(2.0)

    motionProxy.setAngles(arm_joint, angles_open_2, fractionMaxSpeed)
    time.sleep(2.0)

    motionProxy.setAngles(arm_joint, angles_open_3, fractionMaxSpeed)
    time.sleep(2.0)
   

    return EmptyResponse()
    




if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_arms_open_server')

    #init service MoveArmsOPen
    rospy.Service('MoveArmsOpen', Empty, handle_MoveArmsOpen)

    rospy.spin()
			
		
