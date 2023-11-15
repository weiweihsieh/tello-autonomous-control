#!/usr/bin/env python2

import rospy
from h264_image_transport.msg import H264Packet
from tello_driver.msg import test
import av
import cv2
import numpy as np
import threading
import traceback
import time


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
  #if len(msg.data) > 1000:  
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
    tracker_type = 'TLD' # 'BOOSTING'
    tracker = cv2.TrackerTLD_create() # cv2.TrackerBoosting_create()
    ok,bbox = 1, (-1,-1)
    setROI = False    
    stop = False
    fourcc = cv2.VideoWriter_fourcc('X','V','I','D')
    out = cv2.VideoWriter('test_tracker1.avi', fourcc, 30.0, (1920, 720))

    rospy.init_node('h264_listener')
    rospy.Subscriber("/tello/image_raw/h264", H264Packet, callback)
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
        #hsv_img = cv2.cvtColor(blurred_img.copy(), cv2.COLOR_BGR2HSV)
        if setROI is not True:
          bbox = cv2.selectROI(blurred_img)
          tracker.init(blurred_img, bbox)
          setROI = True
        ok, bbox = tracker.update(blurred_img)

        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        center = (int((p2[0]+p1[0])/2),int((p2[1]+p1[1])/2))

        show_image = blurred_img.copy()
        cv2.rectangle(show_image, p1, p2, (0,0,255), 2, 1)
        cv2.circle(show_image, center,3,(0,0,255),-1)
        print(center)
        cv2.putText(show_image, str(float(bbox[2] * bbox[3]) / float(960*720)), (10,40),5 ,2, 255)
        if float(bbox[2] * bbox[3]) / float(960*720) >= 0.25:
          pub.publish(test([center[0],center[1],-1]))
          stop = True
        else:
          if stop is not True:
            pub.publish(test([center[0],center[1],1]))

        out.write(np.concatenate((blurred_img, show_image), axis=1))
        cv2.imshow('result', np.concatenate((blurred_img, show_image), axis=1))
        cv2.waitKey(1)
        if frame.time_base < 1.0/60:
          time_base = 1.0/60
        else:
          time_base = frame.time_base
        frame_skip = int((time.time() - start_time)/time_base)

if __name__ == '__main__':
    try:
        main()
    except BaseException:
        traceback.print_exc()
    finally:
        stream.close()
        cv2.destroyAllWindows()
