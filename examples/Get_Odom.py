# !/usr/bin/env python

from time import sleep
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from occupancy_field import OccupancyField

global my_data

def callback1(data):
    my_data = data
    rospy.loginfo("Pose %s", my_data)

def callback2(data):
    my_data = data
    rospy.loginfo("\n\nScan %s", my_data.ranges[360])

def listener_1():
    rospy.init_node('listener_new', anonymous=True)
    #rospy.Subscriber("pose", Odometry, callback1)
    # request the map from the map server, the map should be of type nav_msgs/OccupancyGrid
    map_server = rospy.ServiceProxy('static_map', GetMap)
    map = map_server().map
    # for now we have commented out the occupancy field initialization until you can successfully fetch the map
    occupancy_field = OccupancyField(map)
    #rospy.spin()

def listener_2():
    rospy.init_node('listener_new', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback2)
    #rospy.spin()

if __name__ == '__main__':
    listener_1()
    listener_2()
    rospy.spin()
