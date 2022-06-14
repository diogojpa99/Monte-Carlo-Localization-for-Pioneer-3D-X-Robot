# !/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

def callback(msg):
    print(msg.pose.pose.position.x)
    print(msg.pose.pose.position.y)
    print(msg.pose.pose.position.z)

rospy.init_node('listener_new1', anonymous=True)
rospy.Subscriber("pose", Odometry, callback)
rospy.spin()