""" *********************************** Libraries ************************************************* """

import numpy as np
from math import sqrt
from numpy.random import uniform
from math import pi


""" ************************************* Global Variables ****************************************  """

''' Map '''

# Create Map
map = np.array([[(1,0), (4,0)],
                [(5,0), (13,0)],
                [(13,3), (6,3)],
                [(4,3), (3,3)],
                [(3,3), (3,7)],
                [(3,8), (3,12)],
                [(3,15), (0,15)],
                [(0,15), (0,0)]] ) 

# Number of walls
n_walls = map.shape[0]

# Rectangle
lower = 0
upper = 15


"""  ************************************ Functions  ******************************************** """

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
    particles[int(M/2):M, 0] = uniform(3, 13, size = int(M/2))
    particles[int(M/2):M, 1] = uniform(0, 3, size = int(M/2))
    particles[0:M, 2] = uniform(0, 2*pi, size = M)
    
    return particles

# Sample Particles because kidnapping

def reposition_particle(particle, reposition_flag):

    if reposition_flag % 2 == 0:
        particle[0] = uniform(0, 3, size = 1)
        particle[1] = uniform(0, 15, size = 1)
    else:
        particle[0] = uniform(3, 15, size = 1)
        particle[1] = uniform(0, 4, size = 1)

    particle[2] = uniform(0, 2*pi, size = 1)
    
    return particle


# Validate Position
# Input: 
# loc = Array that represent the location of the particle/robot
# Output: 
# Function returns '0' if position is invalid, returns '1' if it is valid

def validate_pos(loc):

    if loc[0] < lower - 0.1 or loc[0] > 13 + 0.1 or loc[1] < lower - 0.1 or loc[1] > upper + 0.1:
        return 0
    else:
        return 1



# Validates Localization
# Abstract: Keeps the particles and robot inside the Map
# Output: Localization validated

def validate_loc(loc):

    if loc[0] < lower: loc[0] = lower
    elif loc[0] > upper: loc[0] = upper
    if loc[1] < lower: loc[1] = lower
    elif loc[1] > upper: loc[1] = upper

    if loc[0] > 3 and loc[1] > 3 : 
        avg_pnt = ((3+3)/2,(3+15)/2) 
        err1 = sqrt( pow(avg_pnt[0]-loc[0], 2) + pow( avg_pnt[1]-loc[0],2) ) 
        avg_pnt = ((6+15)/2,(3+3)/2) 
        err2 = sqrt( pow(avg_pnt[0]-loc[0], 2) + pow( avg_pnt[1]-loc[0],2) ) 
        if err1 < err2 :
            loc[0] = 3
        else:
            loc[1] = 3
    

    return loc