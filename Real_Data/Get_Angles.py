# !/usr/bin/env python

from math import pi
import rospy
from sensor_msgs.msg import LaserScan

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

def callback1(data):
    data = data
    inc = (data.angle_increment * 180) / pi
    max_ang = (data.angle_max * 180) / pi
    min_ang = (data.angle_min * 180) / pi

    for angle in MEASURES:
        pos = ((angle) - (-(max_ang))) / inc
        print(angle)
        print(int(pos)+44)
        print("\n")
    print("done")

def listener_1():
    rospy.init_node('listener_new2', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback1)

if __name__ == '__main__':
    listener_1()
    rospy.spin()