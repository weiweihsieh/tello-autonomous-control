#!/usr/bin/env python2
import rospy
import roslib
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from std_msgs.msg import Empty
from h264_image_transport.msg import H264Packet
from time import sleep

import av
import threading
import traceback
import time

global choose
choose = -2

def secallback(data):
    global choose
    #print(data,data)
    choose = data.data

def callback(data):
    global choose
    bridge = CvBridge()
    try: 
        img = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    #print(img.shape)
    cv2.imshow('Original', img)
    #cv2.imshow('Canny', cv2.Canny(img, 100, 200))
    pp = img.copy()
    if choose == -2:
        pp = cv2.Canny(pp, 100, 200)
    elif choose == 0:
        pp = cv2.cvtColor(pp, cv2.COLOR_BGR2GRAY)    
    elif choose == 1:
        _, pp = cv2.threshold(pp, 127, 255, cv2.THRESH_BINARY)
    

    title = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(title + '_video.avi', fourcc, 20, (960, 720))

    cv2.imshow('choose', pp)
    out.write(pp)    
    cv2.waitKey(1)


"""
def callback(data):
    bridge = CvBridge()
    try: 
        img = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow('Original',img)
    cv2.imshow('Canny', cv2.Canny(img, 100, 200))
    cv2.waitKey(1)
"""
def turtle_sub():
    rospy.init_node('turtlesim_sub', anonymous=True)
    rospy.Subscriber("/tello/image_raw", Image, callback)
    #rospy.Subscriber("/tello/image_raw/h264", H264Packet, callback)
    rospy.Subscriber("/selfDefined", UInt8, secallback)
    # spin() simply keeps python from exiting until this node is stopped

    rospy.spin()


if __name__ == '__main__':
    turtle_sub()
