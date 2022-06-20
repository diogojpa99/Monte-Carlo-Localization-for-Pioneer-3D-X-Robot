# !/usr/bin/env python

from math import pi
import rospy
from sensor_msgs.msg import LaserScan

MEASURES = {
    -119,
    -104,
    -89,
    -74,
    -59,
    -44,
    -29,
    -14,
    1,
    16,
    31,
    46,
    61,
    76,
    91,
    106
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