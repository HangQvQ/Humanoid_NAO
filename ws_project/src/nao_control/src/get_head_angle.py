#!/usr/bin/env python

# This is a service to subscribe the current head position

import rospy
import time
import motion
import numpy as np
import almath
import sys
import cv2
from naoqi import ALProxy
from nao_control.srv import GetPos
from geometry_msgs.msg import Twist
motionProxy = 0

def handle_GetPos(req):
    name = req.joint_name
    frame = 0 #FRAME_TORSO
    useSensorValues = True
    curPos = motionProxy.getPosition(name,frame,useSensorValues)
    cur_pos = Twist()
    cur_pos.linear.x = curPos[0]
    cur_pos.linear.y = curPos[1]
    cur_pos.linear.z = curPos[2]
    cur_pos.angular.x = curPos[3]
    cur_pos.angular.y = curPos[4]
    cur_pos.angular.z = curPos[5]
    return cur_pos
    

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_joints_server')

    #init service getposition
    rospy.Service('getposition', GetPos, handle_GetPos)

    rospy.spin()
			
		
