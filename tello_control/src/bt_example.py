#!/usr/bin/env python
import rospy
import roslib
import tello_drone
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8
from behave import *
from time import sleep

rospy.init_node('bt_mission', anonymous=True)
takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
class bt_mission:

    # common member
    drone = tello_drone.Drone()
    sleep(2)
    isContinue = True
    center = (480, 180)
    color = "red"
    cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size = 10)
    land_pub = rospy.Publisher('/tello/land', Empty, queue_size = 1)
    change_pub = rospy.Publisher('/selfChanged', Empty, queue_size = 10)
    
    rate = rospy.Rate(10)

    def __init__(self):
        
        self.tree = (
            self.RedNotFinish >> (self.NotReady2Pass | self.PassAndSwitch) >> ( (self.isNotCenter >> self.FixedPose) | (self.isCenter >> self.Forward) ) >> (self.rec_over1 | self.hover)
            |self.BlueNotFinish >> (self.NotReady2Pass | self.PassAndLand) >> ( (self.isNotCenter >> self.FixedPose) | (self.isCenter >> self.Forward) ) >> (self.rec_over1 | self.hover)
        )
    
        #self.tree = (self.TO >> self.PassAndSwitch >> self.PassAndLand)
        return

    @condition
    def BlueNotFinish(self):
        print("condition: BlueNotFinish")
        return bt_mission.color == "blue"

    @condition
    def RedNotFinish(self):
        print("condition: RedNotFinish")
        return bt_mission.color == "red"

    @condition
    def NotReady2Pass(self):
        print("condition: NotReady2Pass")
        return bt_mission.drone.suber.target[2] != -1

    @condition
    def isNotCenter(self):
        print("condition: isNotCenter")
        return abs(bt_mission.drone.suber.target[0] - bt_mission.center[0]) > 60 or abs(bt_mission.drone.suber.target[1] - bt_mission.center[1]) > 30

    @condition
    def isCenter(self):
        print("condition: isCenter")
        return abs(bt_mission.drone.suber.target[0] - bt_mission.center[0]) <= 60 and abs(bt_mission.drone.suber.target[1] - bt_mission.center[1]) <= 30

    @condition
    def rec_over1(self):
        print("condition: rec_over1")
        return rospy.get_time() - bt_mission.drone.suber.rec_time <= 1.0

    @action
    def PassAndSwitch(self):
        print("action: PassAndSwitch")
        msg = Twist()
        msg.linear.x = 0.4
        #msg.linear.x = 0.24
        #msg.linear.y = -0.1
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()
        
        sleep(2)
        
        msg = Twist()
        msg.linear.x = 0.4
        #msg.linear.x = 0.26 
        #msg.linear.z = 0.2
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()

        sleep(2)

        msg = Twist()
        msg.linear.x = 0.4
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()

        sleep(2)

        msg = Twist()
        msg.linear.x = 0.2
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()

        sleep(2)

        #rotate 180
        start = rospy.get_time()
        t = rospy.get_time()

        while t - start < 2:
            msg = Twist()
            msg.angular.z = 1.0
            #rospy.loginfo(msg)
            bt_mission.cmd_pub.publish(msg)
            bt_mission.rate.sleep()
            t = rospy.get_time()
            #print(t-start)

        msg = Twist()
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()

        sleep(2)

        

        #change color
        bt_mission.change_pub.publish(Empty())
        bt_mission.rate.sleep()
        
        msg = Twist()
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()
        bt_mission.color = "blue"

        sleep(5)


    @action
    def PassAndLand(self):
        print("action: PassAndLand")
        msg = Twist()
        msg.linear.x = 0.35
        #msg.linear.z = 0.2
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()

        sleep(2)
        
        msg = Twist()
        msg.linear.x = 0.35
        #msg.linear.z = 0.2
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()
        
        sleep(2)
        
        msg = Twist()
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()

        sleep(2)

        while bt_mission.drone.suber.canLand is not True:
          msg = Empty()
          bt_mission.land_pub.publish(msg)
          bt_mission.rate.sleep()

        #change color
        bt_mission.change_pub.publish(Empty())
        bt_mission.rate.sleep()

    @action
    def FixedPose(self):
        print("action: FixedPose")
        msg = Twist()
        if abs(bt_mission.drone.suber.target[0] - bt_mission.center[0]) >= 60:
            msg.linear.y = -(bt_mission.drone.suber.target[0] - bt_mission.center[0]) / abs((bt_mission.drone.suber.target[0] - bt_mission.center[0])) * 0.1
        if abs(bt_mission.drone.suber.target[1] - bt_mission.center[1]) >= 30:
            msg.linear.z = -(bt_mission.drone.suber.target[1] - bt_mission.center[1]) / abs((bt_mission.drone.suber.target[1] - bt_mission.center[1])) * 0.2
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()

    @action
    def Forward(self):
        print("action: Forward")
        msg = Twist()
        msg.linear.x = 0.2
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()

    @action
    def hover(self):
        print("action: hover")
        msg = Twist()
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()

    """
    @action
    def TO(self):
        msg = Empty()
        bt_mission.takeoff_pub.publish(msg)
        bt_mission.rate.sleep()
        sleep(2)
    """
    
    def run(self):
        while True:
            if bt_mission.isContinue == False:
                break
            bb = self.tree.blackboard(1)
            state = bb.tick()
            print "state = %s\n" % state
            #if bt_mission.drone.isStop == True:
            #  exec("f = open(\"123.txt\",\'rb\')")           
            while state == RUNNING:
                state = bb.tick()
                print "state = %s\n" % state
                #if bt_mission.drone.isStop == True:
                #  exec("f = open(\"123.txt\",\'rb\')")
            assert state == SUCCESS or state == FAILURE


def TO():
    #rospy.init_node('fly', anonymous=True)
    #takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
    rate = rospy.Rate(10)
    msg = Empty()
    takeoff_pub.publish(msg)
    rate.sleep()

def main():
    print("start...") 
    btCm_n = bt_mission()
    sleep(2)

    try:
        btCm_n.run()
        rospy.spin()
    except IOError, KeyboardInterrupt:
        print("Shutting down ROS module")

if __name__ == "__main__":
    
    TO()
    main()
    """
    drone = tello_drone.Drone()

    global cont
    #land_sub = rospy.Subscriber('/tello/land', Empty, callback)
    land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
    rate = rospy.Rate(10)
    
    sleep(2)

    msg = Empty()
    # rospy.loginfo(msg)
    land_pub.publish(msg)
    rate.sleep()
    """
    

