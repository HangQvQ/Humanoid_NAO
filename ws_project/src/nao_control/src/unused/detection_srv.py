#!/usr/bin/env python
import rospy
import time
import numpy as np
import sys
import cv2
import os

from tutorial_4.srv import detection, detectionResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import roslib
from std_msgs.msg import String
from os.path import join

import os
import fnmatch

# folder path
dir_path = r'/home/hrse/HRS_Group_E/ws_project/resources/train_data'
bridge = CvBridge()
#ORB detector and extractor
kaze = cv2.KAZE.create()
#create matcher based flann
matcher = cv2.BFMatcher()
#create BOW trainer
bow_kmeans_trainer = cv2.BOWKMeansTrainer(40)
#initialize bow_extractor
bow_extractor = cv2.BOWImgDescriptorExtractor(kaze, matcher)
svm = cv2.ml.SVM_create()
#DT = cv2.ml.RTrees.create()

dim = (200, 200)

def get_path(i):
    pos_path = "/home/hrse/HRS_Group_E/ws_project/resources/train_data/pos-%d.png" % (i+1)
    neg_path = "/home/hrse/HRS_Group_E/ws_project/resources/train_data/neg-%d.png" % (i+1)
    return pos_path, neg_path

def add_sample(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    r_img = cv2.resize(img, dim)
    keypoints = kaze.detect(r_img, None)
    keypoints, descriptors = kaze.compute(r_img, keypoints)
    if descriptors is not None:
        bow_kmeans_trainer.add(np.float32(descriptors))

def extract_bow_descriptors(img):
    features = kaze.detect(img)
    return bow_extractor.compute(img, features)


def handle_detection(req):
    Img = Image()
    Img.header.seq = req.img.header.seq
    Img.header.stamp = req.img.header.stamp
    Img.header.frame_id = req.img.header.frame_id
    Img.height = req.img.height
    Img.width = req.img.width
    Img.encoding = req.img.encoding
    Img.is_bigendian = req.img.is_bigendian
    Img.step = req.img.step
    Img.data = req.img.data
    try:
        cv_image = bridge.imgmsg_to_cv2(Img, desired_encoding='passthrough')
    except CvBridgeError as e:
        print(e)

    gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    logo_predict = predict(gray_img)

    if(logo_predict[1][0][0] == 1):
        print('detected')
        return detectionResponse(1)
    else:
        print('not_detected') 
        return detectionResponse(0)       

       

def train_model():

    print("start training")
    for i in range(len(fnmatch.filter(os.listdir(dir_path), '*neg*'))): #
        pos_path, neg_path = get_path(i)
        add_sample(pos_path)
        add_sample(neg_path)

    voc = bow_kmeans_trainer.cluster()
    bow_extractor.setVocabulary(voc)
    traindata, trainlabels = [], []

    for i in range(len(fnmatch.filter(os.listdir(dir_path), '*neg*'))): 
        pos_path, neg_path = get_path(i)

        pos_img = cv2.imread(pos_path, cv2.IMREAD_GRAYSCALE)
        r_pos_img = cv2.resize(pos_img, dim)
        pos_descriptors = extract_bow_descriptors(r_pos_img)
        if pos_descriptors is not None:
            traindata.extend(pos_descriptors)
            trainlabels.append(1)

        neg_img = cv2.imread(neg_path, cv2.IMREAD_GRAYSCALE)
        r_neg_img = cv2.resize(neg_img, dim)
        neg_descriptors = extract_bow_descriptors(r_neg_img)       
        if neg_descriptors is not None:
            traindata.extend(neg_descriptors)
            trainlabels.append(-1)

    svm.train(np.array(traindata), cv2.ml.ROW_SAMPLE, np.array(trainlabels))
    print("finished training")


def predict(fn):
    descriptor = extract_bow_descriptors(fn)
    prediction = svm.predict(descriptor)
    return prediction


if __name__ == '__main__':
    rospy.init_node('logo_detection_server')
    train_model()
    rospy.Service('logo_detection', detection, handle_detection)
    rospy.spin()
			
		
