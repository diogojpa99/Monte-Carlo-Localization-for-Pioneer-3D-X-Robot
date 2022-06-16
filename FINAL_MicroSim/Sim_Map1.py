""" *********************************** Libraries ************************************************* """

import numpy as np
import matplotlib.pyplot as plt
from math import sqrt, pi, radians, degrees
from numpy.random import uniform

import plots as pl
import Map1 as map
import Paricle_Filter as pf


""" ************************************* Global Variables ****************************************  """

''' Particles '''

# Number of particles
M = 900

# Particles
particles = np.empty([M, 3])

''' Weights'''

# Weights
w = np.empty([M,1])
w.fill(1.)

''' Robot '''

# Robot Localization
robot_loc = np.empty([3,1])

# Odometry uncertainty
# odom_uncertainty[0]: x
# odom_uncertainty[1]: y
# odom_uncertainty[2]: Rotation
odom_uncertainty = (0.5,0.5,0.1)


''' Laser '''

N_measures = 24 # Number of measures of the laser model
measures = np.empty([N_measures,1])
laser_reach = 5.6
laser_radius = -120 #Variação de ângulo do laser
laser_radius_var = 10 # Angle of the laser variation
laser_uncertanty = 0.05

''' Simulation '''

# Actions
# action[0] : Distance traveled
# action[1] : Rotation
actions = np.empty([2,1])
actions = np.array([(1,-90),(1,0),(1,0),(1,0),(1,0),(1,0), (1,0), (1,0),(1,0),(1,0), (1,0),(1,0),(1,0),
                    (1,90),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),
                    (1,180),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0)])

# Last Iteration
last_iteration = actions.shape[0]


''' Optimize the algorithm '''

likelihood_sd = 4
n_eff = M # n_eff 
resampling_flag = 1 # Resampling Flag
likelihood_avg = 1 #Likelihoods Average
simulation = 0

''' Output '''

# Errors
errors = np.empty([last_iteration,3])
errors.fill(1.)




"""  ************************************ Functions  ******************************************** """

# init_robot_pos():
# Initialize the robot position
# loc[0] : x variable
# loc[1] : y variable
# loc[2] : Orientation

def init_robot_pos(loc):

    loc[0] = 0
    loc[1] = 14
    loc[2] = 0

    return loc

# create_particles():
# Creates a set of particles distributed uniformly in the map
# Input:
# M: Number of particles
# Ouput:
# 2D Array tha contains the localization of each particle (and its rotation)
# particles[:,0] : x position
# particles[:,1] : y position
# particles[:,2] : rotation

def create_particles(M, particles):
    
    particles[0:int(M/2), 0] = uniform(0, 3, size = int(M/2))
    particles[0:int(M/2), 1] = uniform(0, 15, size = int(M/2))
    particles[int(M/2):M, 0] = uniform(3, 15, size = int(M/2))
    particles[int(M/2):M, 1] = uniform(0, 4, size = int(M/2))
    particles[:, 2] = uniform(0, 2*pi, size = M)
    
    return particles



""" *********************************** main() *********************************************** """

# loc: Localization of the robot
robot_loc = init_robot_pos(robot_loc)
robot_prev_loc = np.empty([3,1])
robot_prev_loc[0] = 0
robot_prev_loc[1] = 0
robot_prev_loc[2] = 0

# Create particles
particles = create_particles(M, particles)
for i in range (M):
    particles[i] = map.validate_loc(particles[i])

# Plotting
pl.plot_simulation('Particle Filter Simulation',robot_loc, particles, map.n_walls, map.map)

# Start simulation
print('The simulation as started')
k = 0


while(1):

    # *********************** Robot simulation ******************************** #

    print("\tIteration nº:",k+1)

    # Update localization of the robot
    if (actions[k][0] != 0 or actions[k][1] != 0):
        robot_loc = pf.odometry_model(robot_loc, actions[k][0], radians(actions[k][1]), 0, odom_uncertainty)
        print(robot_loc)
        robot_loc = map.validate_loc(robot_loc)

    # Retrieving data from the laser
    robot_measures = pf.laser_model(robot_loc, measures, N_measures, map.n_walls, laser_radius_var, robot_loc, laser_reach, laser_radius, laser_uncertanty)

    # ************************** Algorithm  ********************************** #
    print(robot_prev_loc)
    if (robot_prev_loc[0][0] == robot_loc[0][0] and robot_prev_loc[1][0] == robot_loc[1][0] and robot_prev_loc[2][0] == robot_loc[2][0]):
        print('ROBOT DID NOT MOVE')
    else:

        # PREDICT
        particles = pf.predict(particles, actions[k], M, odom_uncertainty)
        for i in range (M):
            particles[i] = map.validate_loc(particles[i])
        
        # UPDATE
        w, likelihood_avg = pf.update(w, measures, particles, resampling_flag, likelihood_avg, N_measures, M, likelihood_sd,  laser_radius_var, robot_loc, laser_reach, laser_radius, laser_uncertanty)
        
        # n_eff
        n_eff_inverse = 0

        for m in range (M):
            n_eff_inverse = n_eff_inverse + pow(w[m],2)
        
        n_eff = 1/n_eff_inverse

        if ( n_eff < M/2 ):
            resampling_flag = 1
        else:
            resampling_flag = 0
        
        # RESAMPLING
        if (resampling_flag == 1):
            indexes = pf.low_variance_resample(w)
            particles[:] = particles[indexes]
            w[:] = w[indexes]
            w /= np.sum(w)
        else:
            print('NO RESAMPLE')

        
    # ************************** Output ********************************** #
    
    print('\tOutput')
    print('Real Localization:', robot_loc[0][0],robot_loc[1][0],robot_loc[2][0]*(180/pi))

    # Centroid of the cluster of particles
    print("Predicted Localization:", np.average(particles[:,0]), np.average(particles[:,1]), np.average(particles[:,2])*(180/pi))
   
    errors[k][0] = abs(np.average(particles[:,0])-robot_loc[0][0])
    errors[k][1] = abs(np.average(particles[:,1])-robot_loc[1][0])

    if ( robot_loc[2][0] < 0):
        robot_loc[2][0] = robot_loc[2][0] + 2*pi

    for i in range(M):
        if (particles[i][2] < 0):
             particles[i][2] = particles[i][2] + 2*pi

    errors[k][2] = abs(np.average(particles[:,2]) - robot_loc[2][0])
    print("ERROR:",errors[k][0], errors[k][1], degrees(errors[k][2]))

    if ( ((errors[k][0] < 0.005) and (errors[k][1] < 0.005) and (errors[k][2] < 0.005)) or k == last_iteration-1):
        break

    # Plotting
    plt.clf()
    pl.plot_simulation('Particle Filter Simulation',robot_loc, particles, map.n_walls, map.map)

    k +=1
    robot_prev_loc = robot_loc
    print(robot_prev_loc)



