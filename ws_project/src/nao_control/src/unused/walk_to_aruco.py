#!/usr/bin/env python
import rospy
import time
import motion
import numpy as np
import almath
import sys
import cv2
from naoqi import ALProxy
from nao_control.srv import ArucoNavigation
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
motionProxy = 0
i = 0
stage = 0
list = []


def handle_tracking(req):

    frame = 0 # FRAME_TORSO
    #frame = 2 # FRAME_ROBOT

    name = 'CameraTop'
    useSensorValues = True
    result = motionProxy.getTransform(name, frame, useSensorValues)
    
    #homogeneous transform relating CameraBottom_optical to CameraBottom frame
    TF_CamOpt2CamBot = np.array([[0,0,1,0],
                                [1,0,0,0],
                                [0,1,0,0],
                                [0,0,0,1]])
    #rotation matrix relating CameraBottom_optical to CameraBottom frame
    # R_CamOpt2CamBot = np.array([[0,0,1],
    #                             [1,0,0],
    #                             [0,1,0]])
    #homo transform relating CameraBottom frame to Torso
    TF_CamBot2Torso = np.array([[result[0],result[1],result[2],result[3]],
                                [result[4],result[5],result[6],result[7]],
                                [result[8],result[9],result[10],result[11]],
                                [result[12],result[13],result[14],result[15]]])

    #print(TF_CamBot2Torso)
    #rotation matrix relating CameraBottom frame to Torso
    # R_CamBot2Torso = np.array([[result[0],result[1],result[2]],
    #                         [result[4],result[5],result[6]],
    #                         [result[8],result[9],result[10]]])

    p_new_CamOpt = np.array([req.new_pos.linear.x, req.new_pos.linear.y,req.new_pos.linear.z,1]).reshape(-1,1)
    # r_new_CamOpt = np.array([req.new_pos.angular.x, req.new_pos.angular.y,req.new_pos.angular.z]).reshape(-1,1)

    # transform marker position into Torso frame
    p_new_Torso = TF_CamBot2Torso.dot(TF_CamOpt2CamBot.dot(p_new_CamOpt))
    # # transform marker orientation into Torso frame
    # r_new_CamOpt_33,_ = cv2.Rodrigues(r_new_CamOpt)
    # r_new_Torso_33 = R_CamBot2Torso.dot(R_CamOpt2CamBot.dot(r_new_CamOpt_33))
    # r_new_Torso,_ = cv2.Rodrigues(r_new_Torso_33)

    # print(p_new_Torso[0][0])
    # print(p_new_Torso[1][0])
    # r = req.new_pos.angular.y
    r =req.new_pos.angular.y
    #print("[theta]: " + str(theta))
    #T = -1.21*r-1.38
    stage = req.stage


    print("[WALK]: " + str(r))
    print(p_new_Torso[0][0])


    if stage == 0:  

        t = -np.arctan2(p_new_Torso[1][0],p_new_Torso[0][0])
        if t > 0.4 or t < -0.4:
            motionProxy.post.moveTo(0, 0, t)
            motionProxy.waitUntilMoveIsFinished()
        stage = 1
        return stage

    elif stage == 1:
        if p_new_Torso[0][0] <= 0.8:
            motionProxy.setWalkArmsEnabled(False, False)
            # motionProxy.post.moveTo(0, 0, -r)
            x = p_new_Torso[0][0]
            list.append(r)
            theta = sum(list)/len(list)

            if theta <= 0:
                y = -x*np.cos(1.57 + theta)
                x = x*np.sin(1.57 + theta)
                motionProxy.post.moveTo(0, 0, -theta)
                motionProxy.waitUntilMoveIsFinished()
                motionProxy.post.moveTo(0, y, 0)
                motionProxy.waitUntilMoveIsFinished()
                stage = 2
            else:
                y = x*np.cos(1.57-theta)-0.1
                x = x*np.sin(1.57-theta)
                theta = theta - 0.3
                motionProxy.post.moveTo(0, 0, -theta)
                motionProxy.waitUntilMoveIsFinished()
                motionProxy.post.moveTo(0, y, 0)
                motionProxy.waitUntilMoveIsFinished()
                stage = 2
            return stage
            
        else:
            list.append(r)
            x = p_new_Torso[0][0]*0.2
            y = -(-p_new_Torso[1][0]*0.2)
            motionProxy.setWalkArmsEnabled(False, False)
            motionProxy.post.moveTo(x, y, 0)
            motionProxy.waitUntilMoveIsFinished()           
            stage = stage
            return stage

    elif stage==2:
        
        list.append(r)
        theta = sum(list)/len(list)
        x = p_new_Torso[0][0]
        x = x*np.sin(1.57+theta)-0.3
        if theta<=0:
            y = -x*np.cos(1.57+theta)+0.1
            motionProxy.setWalkArmsEnabled(False, False)
            motionProxy.post.moveTo(x, y, 0)
            motionProxy.waitUntilMoveIsFinished()
        else:
            y = x*np.cos(1.57-theta)-0.1
            motionProxy.setWalkArmsEnabled(False, False)
            motionProxy.post.moveTo(0, 0, theta)
            motionProxy.waitUntilMoveIsFinished()
            motionProxy.post.moveTo(x, y, 0)
            motionProxy.waitUntilMoveIsFinished()
        stage = 3
        return stage

    elif stage==3:
            x = p_new_Torso[0][0]-0.35
            y = p_new_Torso[1][0]
            motionProxy.setWalkArmsEnabled(False, False)
            motionProxy.post.moveTo(x, y, 0)
            motionProxy.waitUntilMoveIsFinished()
            stage = 4
            return stage   

    print(stage)
    

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    # print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_joints_server')
    rospy.Service('aruco_navigation_service', ArucoNavigation, handle_tracking)

    rospy.spin()