# !/usr/bin/env python

from cmath import pi
from numpy import array
import numpy as np
import rospy
import math
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


#class Array:
global vetor     
list_prev=[]
list_new=[]
amcl_list_prev=[]
amcl_list_new=[]
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

    #print("callback")
    #print(" ")


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
    (data.ranges[387]),
    (data.ranges[416]),
    (data.ranges[444]),
    (data.ranges[473]),
    (data.ranges[501]),
    (data.ranges[530]),
    (data.ranges[558]),
    (data.ranges[586]),
    (data.ranges[615]),
    (data.ranges[643]),
    (data.ranges[672]),
    (data.ranges[700])]

    dist.clear()
    dist.append(vetor)
    #print("callback2")
    #print(" ")


def callback3(msg):
    
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    z=msg.pose.pose.orientation.z
    w=msg.pose.pose.orientation.w

    global amcl_vetor
    amcl_vetor=[x, y, z, w]

    amcl_list_new.clear()
    amcl_list_new.append(x)
    amcl_list_new.append(y)
    amcl_list_new.append(z)
    amcl_list_new.append(w)
    #print("callback3")
    #print(" ")

def get_data():

    amcl_pose_x = amcl_pose_y = pose_x = pose_y = delta_theta=0
    amcl_x0 = amcl_y0 = x0 = y0 = amcl_theta0 = theta0 = 0

    rospy.init_node('listener_new1', anonymous=True)

    r = rospy.Rate(1)   # leituras por segundo 

    while not rospy.is_shutdown():

        rospy.Subscriber("pose", Odometry, callback)
        rospy.Subscriber("scan", LaserScan, callback2)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback3)

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
        
        if amcl_list_new:
            if amcl_list_prev:
                amcl_x0 = amcl_list_new[0]
                amcl_y0 = amcl_list_new[1]
                amcl_theta0 = quaternion_to_euler(amcl_list_new[2],amcl_list_new[3])
                amcl_delta_theta = quaternion_to_euler(amcl_list_new[2],amcl_list_new[3]) - quaternion_to_euler(amcl_list_prev[2],amcl_list_prev[3])
                amcl_pose_x = amcl_list_new[0]-amcl_list_prev[0]
                amcl_pose_y = amcl_list_new[1]-amcl_list_prev[1]
                amcl_list_prev.clear()
            
            for x in range(0,4):
                amcl_list_prev.append(amcl_list_new[x])
            
        amcl_deltaD = math.sqrt((amcl_pose_x**2)+(amcl_pose_y**2))
        r.sleep()

        #return x0, y0, theta0, deltaD, delta_theta, dist, amcl_x0, amcl_y0, amcl_theta0, amcl_deltaD, amcl_delta_theta
    
if __name__ == '__main__':
    get_data()