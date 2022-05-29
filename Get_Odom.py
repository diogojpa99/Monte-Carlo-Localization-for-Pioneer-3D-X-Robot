# !/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

global my_data

def callback(data):
    my_data = data
    rospy.loginfo("I heard %s", my_data.pose.pose.position)
    #rospy.loginfo("I heard %s", my_data.range[360])

def listener_new():
    rospy.init_node('listener_new', anonymous=True)
    rospy.Subscriber("pose", Odometry, callback)
    #rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener_new()