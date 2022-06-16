# !/usr/bin/env python

from numpy import array
import rospy
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

#class Array:
#    def __init__(self, x, y, z, w):
#        self.array=[x, y, z, w]
global vetor        
list=[]

def callback(msg):
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    z=msg.pose.pose.orientation.z
    w=msg.pose.pose.orientation.w
    global vetor
    vetor=[x, y, z, w]
    list.clear()
    list.append(vetor)
    #print(msg.pose.pose.position.x)
    #print(msg.pose.pose.position.y)
    #print(msg.pose.pose.position.z)



if __name__ == '__main__':
    rospy.init_node('listener_new1', anonymous=True)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.Subscriber("pose", Odometry, callback)
        print(list)
        #print(vetor)
        r.sleep()
    