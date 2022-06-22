""" *********************************** Libraries ************************************************* """

import numpy as np
import matplotlib.pyplot as plt
from math import pi, radians, degrees, sin, cos


import plots as pl
import Real_Map as map
import Paricle_Filter as pf
import Get_Data as data


""" ************************************* Global Variables ****************************************  """

''' Particles '''

# Number of particles
original_M = M = 1000

# Flag that defines the number of particles
# resize_flag = 0 : Don't do nothing
# resize_flag = 1 : Increase number of particles
# resize_flag = 2 : Decrease number of particles
resize_flag = 0

# Particles
particles = np.empty([M, 3])

''' Weights '''

w = np.empty([M,1])
w.fill(1.)

''' Robot '''

# Robot Localization
robot_loc = np.empty([3,1])

''' Actions '''

# Actions
# action[0] : Distance traveled
# action[1] : Rotation
actions = np.empty([2,1])
actions[0] = 0
actions[1] = 1

# Last Iteration
last_iteration = actions.shape[0]

''' Optimize the algorithm '''

likelihood_avg = 1 #Likelihoods Average
n_eff = M # n_eff 
resampling_flag = 1 # Resampling Flag

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

    loc[0] = 9.5
    loc[1] = 0.85
    loc[2] = radians(175)

    return loc


""" *********************************** main() *********************************************** """

# Create particles
particles = map.create_particles(M)
for i in range (M):
    particles[i] = map.validate_loc(particles[i])

# Plotting
# Activationg interactive mode
plt.ion()

robot_loc = init_robot_pos(robot_loc)

k = 0



while(1):

    real_points = np.empty([24,2])
    real_points.fill(0.)

    # *********************** Robot simulation ******************************** #

    print("-----------------------------------------------------------------------------------")
    print("\t\t\tIteration nº:",k+1)

    real_x, real_y, real_theta, actions[0], actions[1], measures = data.get_data()
    measures = np.array(measures)
    if len(measures) != 0:
        robot_measures = measures.reshape([measures.shape[1],1])
        robot_measures[np.isnan(robot_measures)] = 0

    # Update localization of the robot
    robot_loc = pf.odometry_model(robot_loc, actions[0], radians(actions[1]), 0)
    robot_loc = map.validate_loc(robot_loc)

    measures = pf.laser_model(robot_loc,map.n_walls,robot_loc,map)


    if (actions[0] == 0 and actions[1] == 0): 
        print('ROBOT DID NOT MOVE')
    
    # ************************** Plots  ********************************** #
    # Plot Map
    for i in range(31):
        plt.plot((map.map[i][0][0],map.map[i][1][0]),(map.map[i][0][1],map.map[i,1,1]), c = 'black')
    
    # Plot robot
    '''radius = -119
    if len(measures) != 0:
        for i in range (robot_measures.shape[0]):
            real_points[i][0] = robot_loc[0] + robot_measures[i]*cos(robot_loc[2] + radians(radius))
            real_points[i][1] = robot_loc[1] + robot_measures[i]*sin(robot_loc[2] + radians(radius))               
            radius += 10'''   
    
    plt.scatter( real_points[:,0], real_points[:,1], s = 8,  c = '#e377c2')
    #plt.scatter(artificial_points[:,0], artificial_points[:,1], s = 8, c = '#2ca02c') 
    plt.scatter(robot_loc[0], robot_loc[1], marker = (6, 0, robot_loc[2]*(180/pi)), c = '#d62728' , s=80, label = "Real position", edgecolors='black')
    plt.plot((robot_loc[0],(1/15)*cos(robot_loc[2])+robot_loc[0]),(robot_loc[1],(1/15)*sin(robot_loc[2])+robot_loc[1]), c = '#17becf')
    plt.pause(0.01)
    plt.show()
    plt.clf()

    print('Real Loc:',"\t", real_x,"\t", real_y,"\t", real_theta)
    print('Robot Loc:',"\t", robot_loc[0][0],"\t", robot_loc[1][0],"\t", robot_loc[2][0]*(180/pi))

    # ************************** Plots  ********************************** #


    # real_points and artifical_points
    # if (real_points[i][0] == robot_loc[0] and real_points[i][1] == robot_loc[1]):
        # or precious robot loc.
        #Invalid point: Podes por exemplo po-los a np.nan

    k +=1

