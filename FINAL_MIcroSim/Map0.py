""" *********************************** Libraries ************************************************* """

import numpy as np
from math import sqrt
from numpy.random import uniform
from math import pi


""" ************************************* Global Variables ****************************************  """

''' Map '''

# Create Map
map = np.array([[(0,0), (0,15)],
                [(0,15), (12,15)],
                [(12,15), (15,13)], 
                [(15,13), (15,0)],
                [(0,0), (11,0)],
                [(13,0), (15,0)],
                [(3,5), (3,6)], 
                [(3,6), (4,6)],
                [(4,6), (4,5)],
                [(3,5), (4,5)]] ) 

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
    
    particles = np.empty([M, 3])
    particles[:, 0] = uniform(lower, upper, size = M)
    particles[:, 1] = uniform(lower, upper, size = M)
    particles[:, 2] = uniform(0, 2*pi, size = M)
    
    return particles

# Sample Particles because kidnapping

def reposition_particle(particle, reposition_flag):

    particle[0] = uniform(lower, upper, size = 1)
    particle[1] = uniform(lower, upper, size = 1)
    particle[2] = uniform(0, 2*pi, size = 1)
    
    return particle


# Validate Position
# Input: 
# loc = Array that represent the location of the particle/robot
# Output: 
# Function returns '0' if position is invalid, returns '1' if it is valid

def validate_pos(loc):

    if loc[0] < lower - 0.1 or loc[0] > upper + 0.1 or loc[1] < lower - 0.1 or loc[1] > upper + 0.1:
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

    return loc