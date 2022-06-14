import rospy
import occupancy_field as occ

from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from nav_msgs.srv import GetMap

from occupancy_field import OccupancyField

import tf

import math
from math import pi, sin, cos
import random
import time

ROBOT_DIAMETER = 0.07
ROBOT_RADIUS = ROBOT_DIAMETER / 2.0
INFRARED_MAX = 0.07
TOF_MAX = 2.00
MIN_READING = 0.0

DISTANCE_SENSOR_TABLE = {
    0.0: ROBOT_RADIUS + 2.00,   # tof
    -30 * pi / 180: ROBOT_RADIUS + 0.070,  # ps0
    -60 * pi / 180: ROBOT_RADIUS + 0.070,  # ps1
    -90 * pi / 180: ROBOT_RADIUS + 0.070,  # ps2
    -110 * pi / 180: ROBOT_RADIUS + 0.070,  # ps3
    110 * pi / 180: ROBOT_RADIUS + 0.070,  # ps4
    90 * pi / 180: ROBOT_RADIUS + 0.070,  # ps5
    60 * pi / 180: ROBOT_RADIUS + 0.070,  # ps6
    30 * pi / 180: ROBOT_RADIUS + 0.070,  # ps7
}

class Particle(object):
    """
    Class to represent and initialize partile with attributes for x y z position, and oreintation
    """
    def __init__(self,x=0.0,y=0.0,theta=0.0,w=1.0):
        # Initialize paticle position, orientation as 0, weight as 1
        self.w = w
        self.x = x
        self.y = y
        self.theta = theta
        self.occ_scan_mapped = []

    def as_pose(self):
        # Returns particle as Pose object
        orientation_tuple = tf.transformations.quaternion_from_euler(0, 0, self.theta) # generates orientation from euler
        return Pose(position=Point(x=self.x, y=self.y, z=0),
                    orientation=Quaternion(x=orientation_tuple[0],
                                           y=orientation_tuple[1],
                                           z=orientation_tuple[2],
                                           w=orientation_tuple[3]))
       

class Particle_Publisher(object):
    def __init__(self):
        
        rospy.init_node('publisher')

        #self.occupancy_field = occ.OccupancyField()
        #self.bounds =  self.occupancy_field.get_obstacle_bounding_box()
        self.map_frame = "map"

        self.num_particles = 1
        self.particle_cloud = []
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        self.get_position()
        rospy.Subscriber("pose", Odometry, self.get_position)
        print("got pose")

    def get_position(self):
        data_odom = None
        while data_odom is None:
            try:
                data_odom = rospy.wait_for_message("/pose", Odometry, timeout=1)
            except:
                rospy.loginfo("Current odom not ready yet, retrying")


    def initialize_particle_cloud(self):
        """
        Creates initial particle cloud based on robot pose estimate position
        """
        self.particle_cloud = []
        #angle_variance = math.pi/10  # POint the points in the general direction of the robot
        #print("theta_cur: ", theta_cur)
        for i in range(self.num_particles):
            
            # Generate values for and add a new particle!!
            valid = 0
            while valid != 1:
                x_rel = random.uniform(self.bounds[0][0], self.bounds[0][1])
                y_rel = random.uniform(self.bounds[1][0], self.bounds[1][1])
                valid = self.map.occupancy_field.is_empty(x_rel, y_rel)
                
            # TODO: Could use a tf transform to add x and y in the robot's coordinate system
            new_theta = (random.uniform(-math.pi, math.pi))
            new_particle = Particle(x_rel, y_rel, new_theta)
            self.particle_cloud.append(new_particle)
        print("Done initializing particles")
        self.normalize_particles()
        print("normalized correctly")
        # publish particles (so things like rviz can see them)
        self.publish_particles()
        print("published")
    
    def normalize_particles(self):
        """
        Normalizes particle weights to total but retains weightage
        """
        total_weights = sum([particle.w for particle in self.particle_cloud])
        # if your weights aren't normalized then normalize them
        if total_weights != 1.0:
            for i in self.particle_cloud:
                i.w = i.w/total_weights

    def publish_particles(self):
            """
            Publish entire particle cloud as pose array for visualization in RVIZ
            Also publish the top / best particle based on its weight
            """
            # Convert the particles from xy_theta to pose!!
            pose_particle_cloud = []
            for p in self.particle_cloud:
                pose_particle_cloud.append(p.as_pose())
            self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(), frame_id=self.map_frame), 
                                                poses=pose_particle_cloud))
    
class LaserModel:
    def __init__(self):
        self.map = map

    def scanner(self):    
        for particle in self.particle_cloud:
            for reading in DISTANCE_SENSOR_TABLE.items():
                range_angle = pose_in_map.yaw + reading[0]
                range_x = pose_in_map.x + reading[1] * cos(range_angle)
                range_y = pose_in_map.y + reading[1] * sin(range_angle)

                if range_x >= self._map.width * self._map.resolution:
                    range_x = (self._map.width - 1) * self._map.resolution
                if range_y >= self._map.height * self._map.resolution:
                    range_y = (self._map.height - 1) * self._map.resolution

                expected = self._bresenham_line(int(pose_in_map.x / self._map.resolution),
                                                int(range_x / self._map.resolution),
                                                int(pose_in_map.y / self._map.resolution),
                                                int(range_y / self._map.resolution))

        return prob

def callback2(data):
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
    print(dist[0])

def callback1(data):
    robot_pos =  data.pose.pose
    rospy.loginfo("Pose %s", robot_pos)

def listener_1():
    rospy.init_node('listener_new1', anonymous=True)
    rospy.Subscriber("pose", Odometry, callback1)

def listener_2():
    #rospy.init_node('listener_new2', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback2)

if __name__ == '__main__':
    print("Starting particle publisher!")
    n = Particle_Publisher()
    rospy.Rate(0.5)
    n.initialize_particle_cloud()
    #listener_1()
    #listener_2()
