""" *********************************** Libraries ************************************************* """

import numpy as np
from math import sqrt
from numpy.random import uniform
from math import pi

import matplotlib.pyplot as plt


""" ************************************* Global Variables ****************************************  """

''' Map '''

map = np.array([[(0,0), (0,9.64)],
                [(0,10.51), (0,15.89)],
                [(0,15.89), (1.65,15.89)],
                [(1.65,1.65), (1.65,7.77)],
                [(1.65, 8.91), (1.65, 14.24)],
                [(1.65, 8.91), (6.03, 8.91)],
                [(1.65, 7.77), (6.03, 7.77)],
                [(1.65, 1.65), (3.6, 1.65)],
                [(0, 0), (0.47, 0)],
                [(1.345, 0), (3.91, 0)],
                [(5.495, 0), (15.769, 0)],
                [(5.75, 1.65), (14.119, 1.65)],
                [(15.769, 0), (15.769, 1.65)],
                [(5.75, 1.65), (5.75, 2.03)],
                [(5.75, 2.03), (6.03, 2.03)],
                [(6.03, 2.03), (6.03, 2.795)],
                [(6.03, 4.01), (6.03, 4.37)],
                [(6.03, 5.885), (6.03, 7.39)],
                [(6.03, 4.01), (6.39, 4.01)],
                [(6.39, 2.795), (6.39, 4.01)],
                [(6.03, 2.795), (6.39, 2.795)],
                [(6.03, 5.885), (6.39, 5.885)],
                [(6.39, 4.67), (6.39, 5.885)],
                [(5.73, 4.67), (6.39, 4.67)],
                [(5.73, 4.37), (5.73, 4.67)],
                [(5.73, 4.37), (6.03, 4.37)],
                [(3.11, 7.39), (6.03, 7.39)],
                [(3.11, 2.03), (3.11, 7.39)],
                [(3.11, 2.03), (3.6, 2.03)],
                [(3.6, 1.65), (3.6, 2.03)],
                [(3.11, 4.01), (3.56, 4.01)],
                [(3.11, 5), (3.56, 5)],
                [(3.11, 5.99), (3.56, 5.99)],
                [(5.75, 1.65), (14.119, 1.65)],
                [(3.11, 6.39), (3.8, 6.39)],
                [(3.8, 6.39), (3.8, 6.99)],
                [(3.11, 6.99), (3.8, 6.99)],
                ]) 

# Number of walls
n_walls = map.shape[0]

# Rectangle
lower = 0
upper = 16


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

    particles[0:int(M/2), 0] = uniform(0, 1.65, size = int(M/2))
    particles[0:int(M/2), 1] = uniform(0, 16, size = int(M/2))
    particles[int(M/2):M, 0] = uniform(1.65, 16, size = int(M/2))
    particles[int(M/2):M, 1] = uniform(0, 1.65, size = int(M/2))
    particles[0:M, 2] = uniform(0, 2*pi, size = M)
    
    return particles

# Sample Particles because kidnapping

def reposition_particle(particle, reposition_flag):

    if reposition_flag % 2 == 0:
        particle[0] = uniform(0, 1.65, size = 1)
        particle[1] = uniform(0, 16, size = 1)
    else:
        particle[0] = uniform(1.65, 16, size = 1)
        particle[1] = uniform(0, 1.65, size = 1)

    particle[2] = uniform(0, 2*pi, size = 1)
    
    return particle


# Validate Position
# Input: 
# loc = Array that represent the location of the particle/robot
# Output: 
# Function returns '0' if position is invalid, returns '1' if it is valid

def validate_pos(loc):

    if loc[0] < lower - 0.1 or loc[0] > upper + 0.1 or loc[1] < lower - 0.1 or loc[1] > 16 + 0.1:
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

    if loc[0] > 1.65 and loc[1] > 1.65 : 
        avg_pnt = ((1.65+1.65)/2,(1.65+16)/2) 
        err1 = sqrt( pow(avg_pnt[0]-loc[0], 2) + pow( avg_pnt[1]-loc[0],2) ) 
        avg_pnt = ((3.3+16)/2,(1.65+1.65)/2) 
        err2 = sqrt( pow(avg_pnt[0]-loc[0], 2) + pow( avg_pnt[1]-loc[0],2) ) 
        if err1 < err2 :
            loc[0] = 1.65
        else:
            loc[1] = 1.65
    

    return loc

    
for i in range(n_walls):
    plt.plot((map[i][0][0],map[i][1][0]),(map[i][0][1],map[i,1,1]), c = 'black')

plt.show()