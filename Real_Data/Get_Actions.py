# !/usr/bin/env python

from cmath import pi
from numpy import array
import numpy as np
import rospy
import math
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

#class Array:
#    def __init__(self, x, y, z, w):
#        self.array=[x, y, z, w]
global vetor     
list_prev=[]
list_new=[]
pose=[]
dist=[]
pose_x=0
pose_y=0
delta_theta=0
counter = 0
def quaternion_to_euler(z, w):
    t3=+2.0*(w*z)
    t4=+1.0-2.0*(z**2)
    yaw_rad=math.atan2(t3, t4)  #radians
    yaw_deg=yaw_rad*180/pi

    return yaw_deg

def callback(msg):
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    z=msg.pose.pose.orientation.z
    w=msg.pose.pose.orientation.w
    global vetor
    vetor=[x, y, z, w]
    #list_prev.clear()
    #list_prev.append(list_new)
    #for x in list_new:
    #    list_prev=list_new[x]

    list_new.clear()
    #list_new.append(vetor)
    list_new.append(x)
    list_new.append(y)
    list_new.append(z)
    list_new.append(w)

    #print(msg.pose.pose.orientation.x)
    #print(msg.pose.pose.orientation.y)
    #print(msg.pose.pose.position.z)

def callback2(data):
    
    vetor =[
    (data.ranges[46]),
    (data.ranges[74]),
    (data.ranges[103]),
    (data.ranges[131]),
    (data.ranges[160]),
    (data.ranges[188]),
    (data.ranges[217]),
    (data.ranges[245]),
    (data.ranges[274]),
    (data.ranges[302]),
    (data.ranges[330]),
    (data.ranges[359]),
    (data.ranges[384]),
    (data.ranges[410]),
    (data.ranges[439]),
    (data.ranges[467]),
    (data.ranges[495]),
    (data.ranges[524]),
    (data.ranges[552]),
    (data.ranges[581]),
    (data.ranges[609]),
    (data.ranges[638]),
    (data.ranges[666]),
    (data.ranges[695]),
    (data.ranges[723])]

    dist.clear()
    dist.append(vetor)


if __name__ == '__main__':
    rospy.init_node('listener_new1', anonymous=True)
    r = rospy.Rate(1)   # leituras por segundo
    while not rospy.is_shutdown():
        rospy.Subscriber("pose", Odometry, callback)
        rospy.Subscriber("scan", LaserScan, callback2)
        #pose=np.array(list_new)-np.array(list_prev)
        #print("list prev: ", list_prev)
        if list_new:
            if list_prev:
                delta_theta=quaternion_to_euler(list_new[2],list_new[3])-quaternion_to_euler(list_prev[2],list_prev[3])
                pose_x=list_new[0]-list_prev[0]
                pose_y=list_new[1]-list_prev[1]
                list_prev.clear()
            
            for x in range(0,4):
                list_prev.append(list_new[x])
            
        deltaD=math.sqrt((pose_x**2)+(pose_y**2))
        #print("list new: ", list_new)
        #print(deltaD, delta_theta)
        #print(pose_x)
        #print(dist)
        r.sleep()
    