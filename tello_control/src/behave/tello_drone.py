#!/usr/bin/env python
import rospy
import roslib
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8
from tello_driver.msg import TelloStatus, test

class Drone_Suber:
  def __init__(self):
    self.target = (-1,-1,1)
    self.canLand = False
    self.rec_time = 0

class Drone():
    def __init__(self):        
        self.suber = Drone_Suber()
        self._sensor()

    def _sensor(self):
      _self_pub = rospy.Subscriber('/selfDefined', test, self.cback)
      _land_sub = rospy.Subscriber('/tello/status', TelloStatus, self.ts_callback)
    
    def cback(self, data):
      self.suber.rec_time = rospy.get_time()
      self.suber.target = data.l1

    def ts_callback(self, data):
      if data.fly_mode == 12:
        self.suber.canLand = True