# !/usr/bin/env python

from math import pi
import rospy
from sensor_msgs.msg import LaserScan

MEASURES = {
    -119,
    -115,
    -111,
    -107,
    -103,
    -99,
    -95,
    -91,
    -87,
    -83,
    -79,
    -75,
    -71,
    -67, 
    -63,
    -59,
    -55,
    -51,
    -47,
    -43,
    -39,
    -35,
    -31,
    -27,
    -23,
    -19,
    -15,
    -11,
    -7,
    -3,
    1,
    5,
    9,
    13,
    17,
    21,
    25,
    29,
    33,
    37,
    41,
    45,
    49,
    53,
    57,
    61,
    65,
    69,
    73,
    77,
    81,
    85,
    89,
    93,
    97,
    101,
    105,
    109,
    113,
    117}

angles_vector = []

def callback1(data):
    data = data
    inc = (data.angle_increment * 180) / pi
    max_ang = (data.angle_max * 180) / pi
    min_ang = (data.angle_min * 180) / pi

    angles_vector.clear()
    for angle in MEASURES:
        pos = ((angle) - (-(max_ang))) / inc
        angles_vector.append(int(pos)+44)
    angles_vector.sort()
    print(angles_vector)
    print("\n")
    print("\n")
    print("\n")
    print("\n")
    
def listener_1():
    rospy.init_node('listener_new2', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback1)

if __name__ == '__main__':
    listener_1()
    rospy.spin()