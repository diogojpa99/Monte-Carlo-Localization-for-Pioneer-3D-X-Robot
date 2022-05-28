""" Libraries """

import numpy as np
import matplotlib.pyplot as plt
from numpy.random import uniform
from math import cos, sin, sqrt




""" Functions """



# create_map():
# Creates an rectangular map using matix points.
# Returns the matrix with the map
def create_map():

    # Our map is a matrix 184*2
    landmarks= np.empty([184,2])
    i = 0

    # We are going to fiil the matrix
    for landmark in landmarks:

        if  i < 46 :

            if i == 0: x = 50 
            landmarks[i][0] = x
            landmarks[i][1] = 50
            x += 10

        elif i < 92 :

            if i == 46: x = 50 
            landmarks[i][0] = x
            landmarks[i][1] = 500
            x += 10

        elif i < 138:

            if i == 92: y = 50 
            landmarks[i][0] = 50
            landmarks[i][1] = y
            y += 10

        elif i < 184:

            if i == 138: y = 50 
            landmarks[i][0] = 500
            landmarks[i][1] = y
            y += 10
        
        i += 1
    
    return landmarks



# init_robot_pos():
# initialize the robot position
# loc[0] : x variable
# loc[1] : y variable
# loc[2] : angle
def init_robot_pos():
    loc = np.empty([3,1])
    loc[0] = 250
    loc[1] = 250
    loc[2] = 0

    return loc



# odometry_model():
# Basically this function describes the movement model of the robot
# The odometry model of the robot is the sum between the target distribution
# And some gaussian noise (See slides)
# Input:
# prev_loc: Localization of the robot at time t
# vel : Velocity read from the the /pose topic
# angle : Angle read from the /twits topic
# In the simulation we obtain vel & angle from the terminal
# Returns: The localization of the robot at time t+1

def odometry_model(prev_loc, vel, angle):
    loc = np.empty([3,1])

    # Target distribution
    # !Atention! Maybe not exactly like this, but it is something like this
    loc[0] = prev_loc[0] + vel*cos(angle)
    loc[1] = prev_loc[1] + vel*sin(angle)
    loc[2] = prev_loc[2] + angle 

    #Adding some noise:


    return loc



# laser_model():
# Basically it will be the distance from the robot's position to the walls
# !Atenção! Temos que incluir aqui o MATCHING
# input:
# loc: Current position of the robot
# landmarks: Position of the landmarks
# Output: Euclidean distance from the robot to each of the partiles 

# TEMOS QUE VER ESTA PARTE MELHOR
def laser_model(robot_loc, landmarks):
    
    
    #Not sure se isto está bem feito, mas a ideia é esta
    #Provavelmente não será necessário calcular a medida de todas.
    measures= np.empty([184,1])
    i = 0
    for measure in measures:
        measures[i] = sqrt( pow(landmarks[i][0]*robot_loc[0], 2) + pow(landmarks[i][1]*robot_loc[1],2) )
        i += 1

    return measures


# create_particles():
# Creates a set of particles distributed uniformly in the map
# Input:
# M: Number of particles
# Ouput:
# 2D Array tha contains the localization of each particle (and its rotation)
# particles[:,0] : x position
# particles[:,0] : y position
# particles[:,0] : rotation
def create_particles(M):
    
    particles = np.empty([M, 3])
    particles[:, 0] = uniform(50, 500, size = M)
    particles[:, 1] = uniform(50, 500, size = M)
    particles[:, 2] = uniform(0, 360, size = M)
    
    return particles


""" Global Variables """

#Number of particles
M = 100


""" Program """

# landmarks: Matrix of points that mark the 'walls' of our map 
landmarks = create_map()

# loc: Localization of the robot
loc = init_robot_pos()

#Activationg interactive mode
plt.ion()

#Create particles
particles = create_particles(M)

while(1):
    
    # Plotting
    plt.scatter(landmarks[:,0],landmarks[:,1], c = 'black')
    plt.scatter(loc[0], loc[1], marker = 'x', c = 'red' )
    plt.scatter(particles[:,0], particles[:,1], marker = '2', c = 'green', s = 10 )
    plt.show()

    #Getting info from the terminal
    print('Please input the robot velocity:')
    vel = float(input())
    print('Please input the robot rotation')
    angle = float(input())

    # Erase previous robot position
    plt.scatter(loc[0], loc[1], c = 'white', s = 60 )
    plt.scatter(particles[:,0], particles[:,1], c = 'white', s = 60 )

    #Update localization of the robot
    loc = odometry_model(loc, vel, angle)

    # Retrieving data from the laser
    measures = laser_model(loc, landmarks)







