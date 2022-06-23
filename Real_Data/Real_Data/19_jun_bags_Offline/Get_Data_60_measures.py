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
global vetor     
list_prev=[]
list_new=[]
pose=[]
dist=[]

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

    list_new.clear()
    list_new.append(x)
    list_new.append(y)
    list_new.append(z)
    list_new.append(w)


def callback2(data):
    
    vetor =[
    (data.ranges[46]),
    (data.ranges[57]),
    (data.ranges[69]),
    (data.ranges[80]),
    (data.ranges[92]),
    (data.ranges[103]),
    (data.ranges[114]),
    (data.ranges[126]),
    (data.ranges[137]),
    (data.ranges[148]),
    (data.ranges[160]),
    (data.ranges[171]),
    (data.ranges[183]),
    (data.ranges[194]),
    (data.ranges[205]),
    (data.ranges[217]),
    (data.ranges[228]),
    (data.ranges[239]),
    (data.ranges[251]),
    (data.ranges[262]),
    (data.ranges[274]),
    (data.ranges[285]),
    (data.ranges[296]),
    (data.ranges[308]),
    (data.ranges[319]),
    (data.ranges[330]),
    (data.ranges[342]),
    (data.ranges[353]),
    (data.ranges[365]),
    (data.ranges[376]),
    (data.ranges[387]),
    (data.ranges[399]),
    (data.ranges[410]),
    (data.ranges[421]),
    (data.ranges[433]),
    (data.ranges[444]),
    (data.ranges[456]),
    (data.ranges[467]),
    (data.ranges[478]),
    (data.ranges[490]),
    (data.ranges[501]),
    (data.ranges[512]),
    (data.ranges[524]),
    (data.ranges[535]),
    (data.ranges[547]),
    (data.ranges[558]),
    (data.ranges[569]),
    (data.ranges[581]),
    (data.ranges[592]),
    (data.ranges[604]),
    (data.ranges[615]),
    (data.ranges[626]),
    (data.ranges[638]),
    (data.ranges[649]),
    (data.ranges[660]),
    (data.ranges[672]),
    (data.ranges[683]),
    (data.ranges[695]),
    (data.ranges[706]),
    (data.ranges[717])]
    
    dist.clear()
    dist.append(vetor)


def get_data():

    pose_x= pose_y = delta_theta=0
    x0 = y0 = theta0 = 0


    rospy.init_node('listener_new1', anonymous=True)

    r = rospy.Rate(1)   # leituras por segundo 

    while not rospy.is_shutdown():

        rospy.Subscriber("pose", Odometry, callback)
        rospy.Subscriber("scan",LaserScan, callback2)

        if list_new:
            if list_prev:
                x0 = list_new[0]
                y0 = list_new[1]
                theta0 = quaternion_to_euler(list_new[2],list_new[3])
                delta_theta = quaternion_to_euler(list_new[2],list_new[3]) - quaternion_to_euler(list_prev[2],list_prev[3])
                pose_x = list_new[0]-list_prev[0]
                pose_y = list_new[1]-list_prev[1]
                list_prev.clear()
            
            for x in range(0,4):
                list_prev.append(list_new[x])
            
        deltaD = math.sqrt((pose_x**2)+(pose_y**2))
        r.sleep()

        return x0, y0, theta0, deltaD, delta_theta, dist
    