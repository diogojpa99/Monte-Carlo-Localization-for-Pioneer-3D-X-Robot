import rospy
import occupancy_field as occ

from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from nav_msgs.srv import GetMap

import tf

import math
import random
import time

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
        # grab the map from the map server

        rospy.init_node('publisher')
        
        self.occupancy_field = occ.OccupancyField()
        self.bounds =  self.occupancy_field.get_obstacle_bounding_box()
        print(self.bounds[0][0])
        self.map_frame = "map"

        self.num_particles = 100
        self.particle_cloud = []
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)

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
                valid = self.occupancy_field.is_empty(x_rel, y_rel)
                
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
    
    
if __name__ == '__main__':
    print("Starting particle publisher!")
    n = Particle_Publisher()
    i=0
    while i < 1000:
        n.initialize_particle_cloud()
        time.sleep(0.5)
        i+=1