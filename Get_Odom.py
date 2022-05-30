# !/usr/bin/env python

from time import sleep
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

global my_data

def callback1(data):
    my_data = data
    rospy.loginfo("Pose %s", my_data.pose.pose.position)

def callback2(data):
    my_data = data
    rospy.loginfo("\n\nScan %s", my_data.ranges[360])

def listener_1():
    rospy.init_node('listener_new', anonymous=True)
    rospy.Subscriber("pose", Odometry, callback1)
    #rospy.spin()

def listener_2():
    rospy.init_node('listener_new', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback2)
    #rospy.spin()

if __name__ == '__main__':
    listener_1()
    listener_2()
    rospy.spin()
