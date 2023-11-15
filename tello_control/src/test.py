#!/usr/bin/env python2
import rospy
import roslib
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import UInt8
from time import sleep

global cont
cont = True

def callback(data):
    global cont
    if data:
        #cont = True
        cont = False

def L():
    global cont
    land_sub = rospy.Subscriber('/tello/land', Empty, callback)
    land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
    self_pub = rospy.Publisher('/selfDefined', UInt8, queue_size=1)
    rate = rospy.Rate(10)

    while cont == True:
        if cont == True:
            self_pub.publish(1)
            rate.sleep()
        
        sleep(3)
        msg = Empty()
        rospy.loginfo(msg)
        land_pub.publish(msg)
        rate.sleep()

    self_pub.publish(0)
    rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('test', anonymous=True)
        while True:
            L()
    except rospy.ROSInterruptException:
        pass
    finally:
        sys.exit(0)