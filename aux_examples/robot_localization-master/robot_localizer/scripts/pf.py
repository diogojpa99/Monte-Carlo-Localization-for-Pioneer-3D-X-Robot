#!/usr/bin/env python3
import rospy

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan, PointCloud
from helper_functions import TFHelper
import random
from copy import deepcopy
from operator import attrgetter

from occupancy_field import OccupancyField
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix

import tf
import math
from tf import TransformListener
from tf import TransformBroadcaster

import numpy as np
from numpy.random import random_sample
from numpy import deg2rad

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


class ParticleFilter(object):
    """
    Class to represent Particle Filter ROS Node
    Subscribes to /initialpose for initial pose estimate
    Publishes top particle estimate to /particlebest and all particles in cloud to /particlecloud
    """
    def __init__(self):
        """
        __init__ function to create main attributes, setup threshold values, setup rosnode subs and pubs
        """
        rospy.init_node('pf')
        self.initialized = False
        self.num_particles = 150
        self.d_thresh = 0.2  # the amount of linear movement before performing an update
        self.a_thresh = math.pi / 6  # the amount of angular movement before performing an update
        self.particle_cloud = []
        self.lidar_points = []
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        self.best_particle_pub = rospy.Publisher("particlebest", PoseStamped, queue_size=10)
        self.base_frame = "base_link"  # the frame of the robot base
        self.map_frame = "map"  # the name of the map coordinate frame
        self.odom_frame = "odom"  # the name of the odometry coordinate frame
        self.scan_topic = "scan"  # the topic where we will get laser scans from
        self.best_guess = (None, None) # (index of particle with highest weight, its weight)
        self.particles_to_replace = .075
        self.n_effective = 0 # this is a measure of the particle diversity

        # pose_listener responds to selection of a new approximate robot
        # location (for instance using rviz)
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose)

        # enable listening for and broadcasting coordinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # laser_subscriber listens for data from the lidar
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_received)

        # create instances of two helper objects that are provided to you
        # as part of the project
        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()
        self.initialized = True

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        print("xy_theta", xy_theta)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta) # creates particle cloud at position passed in
        # by message
        print("INITIALIZING POSE")


        # Use the helper functions to fix the transform
    def initialize_particle_cloud(self, timestamp, xy_theta):
        """
        Creates initial particle cloud based on robot pose estimate position
        """
        self.particle_cloud = []
        angle_variance = math.pi/10  # Point the points in the general direction of the robot
        x_cur = xy_theta[0]
        y_cur = xy_theta[1]
        theta_cur = self.transform_helper.angle_normalize(xy_theta[2])
        # print("theta_cur: ", theta_cur)
        for i in range(self.num_particles):
            # Generate values for and add a new particle!!
            x_rel = random.uniform(-.3, .3)
            y_rel = random.uniform(-.3, .3)
            new_theta = (random.uniform(theta_cur-angle_variance, theta_cur+angle_variance))
            # TODO: Could use a tf transform to add x and y in the robot's coordinate system
            new_particle = Particle(x_cur+x_rel, y_cur+y_rel, new_theta)
            self.particle_cloud.append(new_particle)
        print("Done initializing particles")
        self.normalize_particles()
        # publish particles (so things like rviz can see them)
        self.publish_particles()
        print("normalized correctly")
        self.update_robot_pose(timestamp)
        print("updated robot pose")

    def normalize_particles(self):
        """
        Normalizes particle weights to total but retains weightage
        """
        total_weights = sum([particle.w for particle in self.particle_cloud])
        # if your weights aren't normalized then normalize them
        if total_weights != 1.0:
            for i in self.particle_cloud:
                i.w = i.w/total_weights

    def update_robot_pose(self, timestamp):
        """ Update the estimate of the robot's pose in the map frame given the updated particles.
            There are two logical methods for this:
                (1): compute the mean pose based on all the high weight particles
                (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()
        print("Normalized particles in update robot pose")

        # create average pose for robot pose based on entire particle cloud
        average_x = 0
        average_y = 0
        average_theta = 0

        # walk through all particles, calculate weighted average for x, y, z, in particle map.
        for p in self.particle_cloud:
            average_x += p.x * p.w
            average_y += p.y * p.w
            average_theta += p.theta * p.w

        # # create new particle representing weighted average values, pass in Pose to new robot pose
        self.robot_pose = Particle(average_x,average_y, average_theta).as_pose()

        print(timestamp)
        self.transform_helper.fix_map_to_odom_transform(self.robot_pose, timestamp)
        print("Done fixing map to odom")

    def publish_particles(self):
        """
        Publish entire particle cloud as pose array for visualization in RVIZ
        Also publish the top / best particle based on its weight
        """
        # Convert the particles from xy_theta to pose!!
        pose_particle_cloud = []
        for p in self.particle_cloud:
            pose_particle_cloud.append(p.as_pose())
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(), frame_id=self.map_frame), poses=pose_particle_cloud))


        # doing shit based off best pose
        best_pose_quat = max(self.particle_cloud, key=attrgetter('w')).as_pose()
        self.best_particle_pub.publish(header=Header(stamp=rospy.Time.now(), frame_id=self.map_frame), pose=best_pose_quat)

    def update_particles_with_odom(self, msg):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.
            msg: this is not really needed to implement this, but is here just in case.
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        # TODO: FIX noise incorporation into movement.

        min_travel = 0.25
        xy_spread = 0.02 / min_travel  # More variance with driving forward
        theta_spread = .005 / min_travel

        random_vals_x = np.random.normal(0, abs(delta[0]*xy_spread), self.num_particles)
        random_vals_y = np.random.normal(0, abs(delta[1]*xy_spread), self.num_particles)
        random_vals_theta = np.random.normal(0, abs(delta[2] * theta_spread), self.num_particles)

        for p_num, p in enumerate(self.particle_cloud):
            # compute phi, or basically the angle from 0 that the particle
            # needs to be moving - phi equals OG diff angle - robot angle + OG partilce angle
            # ADD THE NOISE!!
            noisy_x = (delta[0] + random_vals_x[p_num])
            noisy_y = (delta[1] + random_vals_y[p_num])

            ang_of_dest = math.atan2(noisy_y, noisy_x)
            # calculate angle needed to turn in angle_to_dest
            ang_to_dest = self.transform_helper.angle_diff(self.current_odom_xy_theta[2], ang_of_dest)
            d = math.sqrt(noisy_x**2 + noisy_y**2)

            phi = p.theta+ang_to_dest
            p.x += math.cos(phi) * d
            p.y += math.sin(phi) * d
            p.theta += self.transform_helper.angle_normalize(delta[2] + random_vals_theta[p_num])

        self.current_odom_xy_theta = new_odom_xy_theta


    

    def update_particles_with_laser(self, msg):
        """
        calculate particle weights based off laser scan data passed into param
        """
        # print("Updating particles with Laser")
        lidar_points = msg.ranges

        for p_deg, p in enumerate(self.particle_cloud):
            # do we need to compute particle pos in diff frame?
            p.occ_scan_mapped = [] # reset list
            for scan_distance in lidar_points:
                # handle edge case
                if scan_distance == 0.0:
                    continue
                # calc a delta theta and use that to overlay scan data onto the particle headings
                pt_rad = deg2rad(p_deg)
                particle_pt_theta = self.transform_helper.angle_normalize(p.theta + pt_rad)
                particle_pt_x = p.x + math.cos(particle_pt_theta)*scan_distance
                particle_pt_y = p.y + math.sin(particle_pt_theta)*scan_distance
                # calculate distance from every single scan point in particle frame
                occ_value = self.occupancy_field.get_closest_obstacle_distance(particle_pt_x, particle_pt_y)
                # Think about cutting off max penalty if occ_value is too big
                p.occ_scan_mapped.append(occ_value)

            # assign weights based off newly assigned occ_scan_mapped
            # apply gaussian e**-d**2 to every weight, then cube to emphasize
            p.occ_scan_mapped = [(math.e/(d)**2) if (d)**2 != 0 else (math.e/(d+.01)**2) for d in p.occ_scan_mapped]
            p.occ_scan_mapped = [d**3 for d in p.occ_scan_mapped]
            p.w = sum(p.occ_scan_mapped)
            #print("Set weight to: ", p.w)
            p.occ_scan_mapped = []
        self.normalize_particles()

    @staticmethod
    def draw_random_sample(choices, probabilities, n):
        """ Return a random sample of n elements from the set choices with the specified probabilities
            choices: the values to sample from represented as a list
            probabilities: the probability of selecting each element in choices represented as a list
            n: the number of samples
        """
        values = np.array(range(len(choices)))
        probs = np.array(probabilities)
        bins = np.add.accumulate(probs)
        inds = values[np.digitize(random_sample(n), bins)]
        samples = []
        for i in inds:
            samples.append(deepcopy(choices[int(i)]))
        return samples


    def resample_particles(self):
        """
        Re initialize particles in self.particle_cloud
        based on preivous weightages.
        """
        weights = [p.w for p in self.particle_cloud]

        # after calculating all particle weights, we want to calc the n_effective
        # self.n_effective = 0
        self.n_effective = 1/ sum([w**2 for w in weights]) # higher is more diversity, so less noise
        print("n_effective: ", self.n_effective)


        temp_particle_cloud = self.draw_random_sample(self.particle_cloud, weights, int((1-self.particles_to_replace)*self.num_particles))
        # temp_particle_cloud = self.draw_random_sample(self.particle_cloud, weights, self.num_particles)

        particle_cloud_to_transform = self.draw_random_sample(self.particle_cloud,weights, self.num_particles - int((1-self.particles_to_replace)*self.num_particles))

        # NOISE POLLUTION - larger noise, smaller # particles
        # normal_std_xy = .25
        normal_std_xy = 10/self.n_effective # feedback loop? 8,3
        normal_std_theta = 3/self.n_effective
        # normal_std_theta = math.pi/21
        random_vals_x = np.random.normal(0, normal_std_xy, len(particle_cloud_to_transform))
        random_vals_y = np.random.normal(0, normal_std_xy, len(particle_cloud_to_transform))
        random_vals_theta = np.random.normal(0, normal_std_theta, len(particle_cloud_to_transform))

        for p_num , p in enumerate(particle_cloud_to_transform): # add in noise in x,y, theta
            p.x += random_vals_x[p_num]
            p.y += random_vals_y[p_num]
            p.theta += random_vals_theta[p_num]

        # reset the partilce cloud based on the newly transformed particles
        self.particle_cloud = temp_particle_cloud + particle_cloud_to_transform


    def scan_received(self, msg):
        """
        Callback function for recieving laser scan - should pass data into global scan object
        """
        """ This is the default logic for what to do when processing scan data.
            Feel free to modify this, however, we hope it will provide a good
            guide.  The input msg is an object of type sensor_msgs/LaserScan """
        if not(self.initialized):
            # wait for initialization to complete
            return

        if not(self.tf_listener.canTransform(self.base_frame, msg.header.frame_id, msg.header.stamp)):
            # need to know how to transform the laser to the base frame
            # this will be given by either Gazebo or neato_node
            return

        if not(self.tf_listener.canTransform(self.base_frame, self.odom_frame, msg.header.stamp)):
            # need to know how to transform between base and odometric frames
            # this will eventually be published by either Gazebo or neato_node
            return

        # calculate pose of laser relative to the robot base
        p = PoseStamped(header=Header(stamp=rospy.Time(0), frame_id=msg.header.frame_id))
        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # find out where the robot thinks it is based on its odometry
        p = PoseStamped(header=Header(stamp=msg.header.stamp, frame_id=self.base_frame), pose=Pose())
        # grab from listener & store the the odometry pose in a more convenient format (x,y,theta)
        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)
        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        # Now we've done all calcs, we exit the scan_recieved() method by either initializing a cloud
        if not(self.particle_cloud):
            # now that we have all of the necessary transforms we can update the particle cloud
            # TODO: Where do we get the xy_theta needed for initialize_particle_cloud?
            self.initialize_particle_cloud(msg.header.stamp, self.current_odom_xy_theta)


        elif (math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh):
            # we have moved far enough to do an update!
            self.update_particles_with_odom(msg)
            self.update_particles_with_laser(msg)  # update based on laser scan
            self.update_robot_pose(msg.header.stamp)  # update robot's pose
            self.resample_particles()  # resample particles to focus on areas of high density
        # # publish particles (so things like rviz can see them)
        self.publish_particles()


    def run(self):
        """
        main run loop for rosnode
        """
        r = rospy.Rate(5)
        print("Nathan and Adi ROS Loop code is starting!!!")
        while not(rospy.is_shutdown()):
            # in the main loop all we do is continuously broadcast the latest
            # map to odom transform
            self.transform_helper.send_last_map_to_odom_transform()
            r.sleep()


if __name__ == '__main__':
    print("starting PF")
    n = ParticleFilter()
    print("Finished starting PF")
    n.run()