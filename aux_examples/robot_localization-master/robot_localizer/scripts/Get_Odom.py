# !/usr/bin/env python

from time import sleep
from math import pi
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from occupancy_field import OccupancyField

import pandas as pd

global my_data

MEASURES = {
    -110,
    -90,
    -60,
    -30,
    0,
    30,
    60,
    90,
    110
}
x_coordinate_col = []
y_coordinate_col = []
z_rotation_col = []
w_rotation_col = []
cb_signal = False

def callback1(data):
    
    x_coordinate_col.append(data.pose.pose.position.x)
    y_coordinate_col.append(data.pose.pose.position.y)
    z_rotation_col.append(data.pose.pose.orientation.z)
    w_rotation_col.append(data.pose.pose.orientation.w)

def callback2(data):
    my_data = data
    inc = (my_data.angle_increment * 180) / pi
    max_ang = (my_data.angle_max * 180) / pi
    min_ang = (my_data.angle_min * 180) / pi
    #print(inc)
    #print(max_ang)
    #print(min_ang)
    for angle in MEASURES:
        pos = ((angle) - (-(max_ang))) / inc
        print(angle)
        print(int(pos)+44)
        print("\n") 
    print(len(my_data.ranges))
    i = 0
    for range in my_data.ranges:
        if i > 599:
            print(i)
            rospy.loginfo("\n\nScan %s", range)
        i+=1
    print("Done")
    
def callback3(data):
    dist = []
    dist.append(data.ranges[72])
    dist.append(data.ranges[128])
    dist.append(data.ranges[214])
    dist.append(data.ranges[299])
    dist.append(data.ranges[384])
    dist.append(data.ranges[470])
    dist.append(data.ranges[555])
    dist.append(data.ranges[640])
    dist.append(data.ranges[697])

    for distance in dist:
        print(distance)
    print("scan done")

def listener_1():
    rospy.init_node('listener_new1', anonymous=True)
    rospy.Subscriber("pose", Odometry, callback1)
    run()
    #rospy.spin()

def listener_2():
    rospy.init_node('listener_new2', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback2)
    #rospy.spin()

def listener_3():
    #rospy.init_node('listener_new3', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback3)
    #rospy.spin()

def some_other_method():
    print("other method")

def run():
    while not rospy.is_shutdown():
        #Do some other work
        if cb_signal == True:
            some_other_method()
        rospy.Rate.sleep(10) #10Hz

if __name__ == '__main__':
    
    listener_1()
    #listener_2()
    #listener_3()
    
    col1 = "pose_X"
    col2 = "pose_Y"
    col3 = "rotation_Z"
    col4 = "rotation_W"
    
    data = pd.DataFrame({col1:x_coordinate_col, 
                         col2:y_coordinate_col,
                         col3:z_rotation_col,
                         col4:w_rotation_col})

    data.to_excel('Pose_info.xlsx', sheet_name="sheet1", index=False)
