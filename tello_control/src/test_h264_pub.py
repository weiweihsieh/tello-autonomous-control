#!/usr/bin/env python

import rospy
import roslib
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8
from tello_driver.msg import TelloStatus, test
from time import sleep

global target
target = (-1, -1, 1)

global center
center = (480, 180)

global check
check = False

global canLand
canLand = False


def callback(data):
    global target
    target = data.l1


def ts_callback(data):
    global canLand
    if data.fly_mode == 12:
        canLand = True


def cmd():
    # The queue_size argument is New in ROS hydro and limits the amount of queued messages if any subscriber is not receiving them fast enough
    #rospy.init_node('h264_pub', anonymous=True)
    self_pub = rospy.Subscriber('/selfDefined', test, callback)
    cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    count = 0
    global target
    global center
    global check

    while target[0] == -1 and target[1] == -1:
        pass

    while not rospy.is_shutdown():
        dx = target[0] - center[0]
        dy = target[1] - center[1]

        print(target)

        if target[2] == -1:
            #stop    
            msg = Twist()
            cmd_pub.publish(msg)
            rate.sleep()

            print('stop')
            sleep(3)
            #if differerce is large
            print(dx, dy)
            if abs(dx) > 30 and abs(dy) > 30:
                print('dxdy')
                msg = Twist()
                #msg.linear.x = 0.2
                msg.linear.y = -dx / abs(dx) * 0.1
                msg.linear.z = -dy / abs(dy) * 0.1
                cmd_pub.publish(msg)
                rate.sleep()
            elif abs(dx) > 30:
                print('dx')
                msg = Twist()
                #msg.linear.x = 0.2
                msg.linear.y = -dx / abs(dx) * 0.1
                #msg.linear.z = -dy / abs(dy) * 0.1
                cmd_pub.publish(msg)
                rate.sleep()
            elif abs(dy) > 30:
                print('dy')
                msg = Twist()
                #msg.linear.x = 0.2
                #msg.linear.y = -dx / abs(dx) * 0.1
                msg.linear.z = -dy / abs(dy) * 0.1
                cmd_pub.publish(msg)
                rate.sleep()

            sleep(1)

            msg = Twist()
            msg.linear.x = 0.4
            cmd_pub.publish(msg)
            rate.sleep()

            sleep(1)

            msg = Twist()
            msg.linear.x = 0.4
            cmd_pub.publish(msg)
            rate.sleep()

            sleep(2)
            
            msg = Twist()
            cmd_pub.publish(msg)
            rate.sleep()            
            break
            
            """
            msg = Twist()
            msg.linear.x = 0.2
            #msg.linear.y = -dx / abs(dx) * 0.1
            #msg.linear.z = -dy / abs(dy) * 0.1
            cmd_pub.publish(msg)
            rate.sleep()
            sleep(1)
            msg = Twist()
            msg.linear.x = 0.2 
            cmd_pub.publish(msg)
            rate.sleep()
            sleep(2)
            msg = Twist()
            cmd_pub.publish(msg)
            rate.sleep()
            break
            """
        else:
            if check == False:
                if abs(dx) < 48 and abs(dy) < 48:
                    check = True
                else:
                    check = False
            else:
                if abs(dx) >= 50 or abs(dy) >= 50:
                    check = False

            if check == True or (dx == 0 or dy == 0):
                msg = Twist()
                msg.linear.x = 0.2
                cmd_pub.publish(msg)
                rate.sleep()
            else:
                msg = Twist()
                msg.linear.y = -dx / abs(dx) * 0.1
                msg.linear.z = -dy / abs(dy) * 0.2

                cmd_pub.publish(msg)
                rate.sleep()
                # sleep(1)

    msg = Twist()
    cmd_pub.publish(msg)
    rate.sleep()
    print("end loop")

def TO():
    takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
    rospy.init_node('h264_pub', anonymous=True)
    rate = rospy.Rate(10)
    msg = Empty()
    rospy.loginfo(msg)
    takeoff_pub.publish(msg)
    rate.sleep()


def L():
    global canLand
    land_sub = rospy.Subscriber('/tello/status', TelloStatus, ts_callback)
    land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
    rate = rospy.Rate(10)

    while canLand is not True:
        msg = Empty()
        land_pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    TO()
    sleep(3)
    #rospy.init_node('h264_pub', anonymous=True)
    cmd()
    sleep(3)
    L()
    sleep(3)
    sys.exit(0)
