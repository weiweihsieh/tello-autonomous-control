#!/usr/bin/env python2
import rospy
import roslib
import cv2
import numpy as np
import av
import threading
import traceback
import time

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from std_msgs.msg import Empty
from h264_image_transport.msg import H264Packet
from tello_driver.msg import test

class StandaloneVideoStream(object):
    def __init__(self):
        self.cond = threading.Condition()
        self.queue = []
        self.closed = False

    def read(self, size):
        self.cond.acquire()
        try:
            if len(self.queue) == 0 and not self.closed:
                self.cond.wait(2.0)
            data = bytes()
            while 0 < len(self.queue) and len(data) + len(self.queue[0]) < size:
                data = data + self.queue[0]
                del self.queue[0]
        finally:
            self.cond.release()
        return data

    def seek(self, offset, whence):
        return -1

    def close(self):
        self.cond.acquire()
        self.queue = []
        self.closed = True
        self.cond.notifyAll()
        self.cond.release()

    def add_frame(self, buf):
        self.cond.acquire()
        self.queue.append(buf)
        self.cond.notifyAll()
        self.cond.release()

stream = StandaloneVideoStream()

def callback(msg):
    #rospy.loginfo('frame: %d bytes' % len(msg.data))
    stream.add_frame(msg.data)

def findMask(img):
    lr0 = np.array([0,150,0])
    ur0 = np.array([7,255,255])
    lr1 = np.array([173,150,0])
    ur1 = np.array([180,255,255])
    rm0 = cv2.inRange(img, lr0, ur0)
    rm1 = cv2.inRange(img, lr1, ur1)
    rm = cv2.bitwise_or(rm0, rm1)
    return rm


def main():
    old_center = [0,0]
    fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
    out = cv2.VideoWriter('test_contour.avi', fourcc, 30.0, (1920, 720))
    rospy.init_node('h264_listener')
    rospy.Subscriber("/tello2/image_raw/h264", H264Packet, callback)
    pub = rospy.Publisher('/selfDefined', test, queue_size = 1)
    container = av.open(stream)
    rospy.loginfo('main: opened')
    frame_skip = 300
    for frame in container.decode(video=0):
        if 0 < frame_skip:
            frame_skip -= 1
            continue
        start_time = time.time()
        image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
        blurred_img = cv2.GaussianBlur(image, (13, 13), 0)
        hsv_img = cv2.cvtColor(blurred_img.copy(), cv2.COLOR_BGR2HSV)
        red_mask = findMask(hsv_img)
        (c_c, c_h) = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #return only 2 arguments in v4.2.0
        c_i = red_mask.copy()
        show_image = cv2.cvtColor(c_i, cv2.COLOR_GRAY2BGR)
        
        
        if len(c_c) != 0:
            """
            cv2.drawContours(show_image, c_c, 0, (0, 0, 255), -1)
            out_max_contours = max(c_c, key = cv2.contourArea)
            rect = cv2.minAreaRect(out_max_contours)
            rect_width, rect_height = rect[1]
            ce_x = rect[0][0] + 1/2*rect_width
            ce_y = rect[0][1] + 1/2*rect_height
            #old_center = [int(ce_x), int(ce_y)]

            cv2.circle(show_image, (int(ce_x), int(ce_y)), 5, (0, 0, 255), -1) #draw center
            cv2.circle(show_image, (480, 180), 3, (255, 0, 0), -1) #draw center of UAV

            #add speed
            if old_center[0] == 0 and old_center[1] == 0:
                old_center = [int(ce_x), int(ce_y)]
                pub.publish(test([int(old_center[0]), int(old_center[1]), 1]))
                #print(old_center)      
            else:
                cv2.putText(show_image, str(rect_width*rect_height/(960*720.0)), (10, 40), 5, 2, 255)
                if rect_width*rect_height >= 960*720*0.25: #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    pub.publish(test([int(old_center[0]), int(old_center[1]), -1]))
                    print(">=100")
                    t = rospy.get_time()
                    while rospy.get_time() - t < 1:
                        pass
                    #rospy.signal_shutdown('Quit')
                else:
                    old_center = [int(ce_x), int(ce_y)]
                    pub.publish(test([int(old_center[0]), int(old_center[1]), 1]))
            
            #pub.publish(test([center[0], center[1]]))
        """
        out.write(np.concatenate((blurred_img, show_image), axis=1))
        #cv2.imshow('result', np.concatenate((blurred_img, show_image), axis=1))
        cv2.imshow('result', blurred_img)
        cv2.waitKey(1)
        if frame.time_base < 1.0/60:
            time_base = 1.0/60
        else:
            time_base = 1.0/60
        frame_skip = int((time.time() - start_time)/time_base)        

if __name__ == '__main__':
    try:
        main()
    except BaseException:
        traceback.print_exc()
    finally:
        stream.close()
        cv2.destroyAllWindows()
