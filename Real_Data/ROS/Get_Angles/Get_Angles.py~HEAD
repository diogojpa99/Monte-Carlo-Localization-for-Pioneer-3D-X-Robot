# !/usr/bin/env python

from math import pi
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np


MEASURES = {
    -119,
    -109,
    -99,
    -89,
    -79,
    -69,
    -59,
    -49,
    -39,
    -29,
    -19,
    -9,
    1,
    11,
    21,
    31,
    41,
    51,
    61,
    71,
    81,
    91,
    101,
    111
}

angles_vector = []

def callback1(data):
    data = data
    inc = (data.angle_increment * 180) / pi
    max_ang = (data.angle_max * 180) / pi
    min_ang = (data.angle_min * 180) / pi

    angles_vector.clear()
    for angle in MEASURES:
        pos = ((angle) - (-(max_ang))) / inc
<<<<<<< Updated upstream:Real_Data/ROS/Get_Angles.py
=======
        print(angle)
        print(int(pos)+44)
        print("\n")
        angles_vector.append(int(pos)+44)
    print(angles_vector)
    print("\n")
    angles_vector.sort()
    print(angles_vector)
        
    print("done")
>>>>>>> Stashed changes:Real_Data/Get_Angles.py

    return
    
def listener_1():
    rospy.init_node('listener_new2', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback1)

if __name__ == '__main__':
    listener_1()
    rospy.spin()