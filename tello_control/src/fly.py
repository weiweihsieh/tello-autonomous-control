#!/usr/bin/env python2
import rospy
import roslib
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import UInt8
from time import sleep
from tello_driver.msg import TelloStatus


global cont
cont = True

def callback(data):
    global cont
    if data:
        cont = False

def TO(namespaces):
    #takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
    
    takeoff_pub = rospy.Publisher('/' + namespaces + '/takeoff', Empty, queue_size=1)
    
    rate = rospy.Rate(10)

    while takeoff_pub.get_num_connections() == 0:
        pass

    msg = Empty()
    rospy.loginfo(msg)
    takeoff_pub.publish(msg)
    rate.sleep()

def L(namespaces):
    global cont
    #land_sub = rospy.Subscriber('/tello/land', Empty, callback)
    #land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
    rospy.init_node('fly', anonymous=True)
    land_sub = rospy.Subscriber('/' + namespaces + '/land', Empty, callback)
    land_pub = rospy.Publisher('/' + namespaces + '/land', Empty, queue_size=1)

    rate = rospy.Rate(10)

    #while land_pub.get_num_connections() == 0:
        #pass

    while True:
        #sleep(3)
        msg = Empty()
        # rospy.loginfo(msg)
        land_pub.publish(msg)
        rate.sleep()
    
    #sleep(1)
    #cont = True

def rotate(z, rtime):
    # The queue_size argument is New in ROS hydro and limits the amount of queued messages if any subscriber is not receiving them fast enough

    cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    start = rospy.get_time()
    t = rospy.get_time()

    while t - start < rtime:
        msg = Twist()
        msg.angular.z = z
        rospy.loginfo(msg)
        cmd_pub.publish(msg)
        rate.sleep()
        t = rospy.get_time()
        print(t-start)

    msg = Twist()
    cmd_pub.publish(msg)
    rate.sleep()
    
    print("end loop")
    sleep(1)


def forward(x, ftime):
    cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    start = rospy.get_time()
    t = rospy.get_time()

    while t - start < ftime:
        msg = Twist()
        msg.linear.x = x
        rospy.loginfo(msg)
        cmd_pub.publish(msg)
        rate.sleep()
        t = rospy.get_time()

    msg = Twist()
    cmd_pub.publish(msg)
    rate.sleep()

    print("end loop")
    sleep(1)

def vertical(z, a, vtime):
    cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    start = rospy.get_time()
    t = rospy.get_time()

    while t - start < vtime:
        msg = Twist()
        msg.linear.z = z
        msg.angular.z = a
        rospy.loginfo(msg)
        cmd_pub.publish(msg)
        rate.sleep()
        t = rospy.get_time()

    msg = Twist()
    cmd_pub.publish(msg)
    rate.sleep()

    print("end loop")
    #sleep(1)

def circle(ctime):
    cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    start = rospy.get_time()
    t = rospy.get_time()

    while t - start < ctime:
        msg = Twist()
        msg.linear.x = 0.3
        msg.angular.z = 0.75
        rospy.loginfo(msg)
        cmd_pub.publish(msg)
        rate.sleep()
        t = rospy.get_time()

    msg = Twist()
    cmd_pub.publish(msg)
    rate.sleep()

    print("end loop")
    sleep(1)

def flip(n):
    flip_pub = rospy.Publisher('/tello/flip', UInt8, queue_size=1)
    
    rate = rospy.Rate(10)
    while flip_pub.get_num_connections() == 0:
        print("wait")
    
    msg = UInt8()
    msg.data = n
    rospy.loginfo(msg)
    flip_pub.publish(msg)
    rate.sleep()

    sleep(1)

def status_callback(data):
    print(data.battery_percentage)

def get_status(namespaces):
    
    #rospy.init_node('status', anonymous=True)
    status_sub = rospy.Subscriber('/' + namespaces + '/status', TelloStatus, status_callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

def video_callback(data):
    stream.add_frame(msg.data)
    

if __name__ == '__main__':
    try:
        rospy.init_node('fly', anonymous=True)

        #get_status('tello2')

        #get_status('tello2')
        TO('tello1')
        TO('tello2')
        sleep(3)
        
        """
        for i in range(4): 
            forward(0.25, 4)
            rotate(0.5, 2.6) #5s=180 +->left
        """
        """
        forward(0.25, 2.5)
        rotate(0.5, 2.6)
        vertical(0.5, 0, 2)
        sleep(1)

        #heart
        circle(3)
        rotate(1.0, 2) #180d
        circle(3)
        rotate(1.0, 0.5)

        forward(0.25, 3.2)
        rotate(1.0, 0.8)
        forward(0.25, 3)
        
        rotate(1.0, 1.6)       
        forward(0.25, 2.6)
        
        vertical(-0.5, -1.0, 1.5)
        vertical(-0.5, 1.0, 1.5)
        vertical(-0.5, -1.0, 1.5)
        vertical(-0.5, 1.0, 1.5)
        
        #flip()
        sleep(1)
        """

        L('tello1')
        L('tello2')
    except rospy.ROSInterruptException:
        pass
    finally:
        sys.exit(0)
