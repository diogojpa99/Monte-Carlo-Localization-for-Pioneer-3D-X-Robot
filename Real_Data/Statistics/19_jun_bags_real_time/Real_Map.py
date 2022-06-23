""" *********************************** Libraries ************************************************* """

import numpy as np
from math import sqrt
from numpy.random import uniform
from math import pi

""" ************************************* Global Variables ****************************************  """

''' Map '''

map = np.array([[(0,0), (0,15.89)],
                [(0,15.89), (1.65,15.89)],
                [(0, 0), (0.47, 0)],
                [(1.345, 0), (15.769, 0)],
                [(1.65,1.65), (1.65,7.58)],
                [(1.65, 8.91), (1.65, 14.24)],
                [(1.65, 8.91), (6.03, 8.91)],
                [(1.65, 7.58), (6.03, 7.58)],
                [(1.65, 1.65), (3.6, 1.65)],
                [(5.75, 1.65), (14.119, 1.65)],
                [(15.769, 0), (15.769, 1.65)],
                [(5.75, 1.65), (5.75, 2.03)],
                [(5.75, 2.03), (6.03, 2.03)],
                [(6.03, 2.03), (6.03, 2.795)],
                [(6.03, 5.885), (6.03, 8.91)],
                [(5.73, 4.01), (6.39, 4.01)],
                [(6.39, 2.795), (6.39, 4.01)],
                [(6.03, 2.795), (6.39, 2.795)],
                [(6.03, 5.885), (6.39, 5.885)],
                [(6.39, 4.67), (6.39, 5.885)],
                [(5.73, 4.67), (6.39, 4.67)],
                [(5.73, 4.01), (5.73, 4.67)],
                [(3.11, 2.03), (3.11, 6.39)],
                [(3.11, 2.03), (3.6, 2.03)],
                [(3.6, 1.65), (3.6, 2.03)],
                [(3.11, 4.01), (3.56, 4.01)],
                [(3.11, 5), (3.56, 5)],
                [(3.11, 5.99), (3.56, 5.99)],
                [(5.75, 1.65), (14.119, 1.65)],
                [(3.11, 6.39), (3.8, 6.39)],
                [(3.8, 6.39), (3.8, 7.58)]]) 

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

def create_particles(M):

    particles = np.empty([M,3])
    particles[0:int(0.35*M), 0] = uniform(0, 1.65, size = int(M*0.35))
    particles[0:int(0.35*M), 1] = uniform(0, 16, size = int(0.35*M))
    particles[int(0.35*M):int(0.7*M), 0] = uniform(1.65, 16, size = int(0.35*M))
    particles[int(0.35*M):int(0.7*M), 1] = uniform(0, 1.65, size = int(0.35*M))
    particles[int(0.7*M):int(0.95*M), 0] = uniform(3.11, 6.39, size = int(0.25*M))
    particles[int(0.7*M):int(0.95*M), 1] = uniform(1.65, 7.58, size = int(0.25*M))
    particles[int(0.95*M):M, 0] = uniform(1.65, 6.03, size = int(0.05*M))
    particles[int(0.95*M):M, 1] = uniform(7.58, 8.91, size = int(0.05*M))
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

    if loc[0] < lower - 0.2 or loc[0] > upper  or loc[1] < lower - 0.2 or loc[1] > upper:
        return 0
    else:
        return 1



# Validates Localization
# Abstract: Keeps the particles and robot inside the Map
# Output: Localization validated

def validate_loc(loc):

    err = np.empty([4,1])
    err[:] = 100
    validate_flag = 0

    if loc[0] < lower: loc[0] = lower
    elif loc[0] > upper: loc[0] = upper
    if loc[1] < lower: loc[1] = lower
    elif loc[1] > upper: loc[1] = upper
    
    if (loc[0] > 1.65 and loc[1] > 8.91 and loc[0] < 6.39): 
        
        avg_pnt = ((1.65+1.65)/2,(8.91+14.24)/2) 
        err[0] = sqrt( pow(avg_pnt[0]-loc[0], 2) + pow( avg_pnt[1]-loc[1],2) ) 
        
        avg_pnt = ((1.65+6.39)/2,(8.91+8.91)/2) 
        err[1] = sqrt( pow(avg_pnt[0]-loc[0], 2) + pow( avg_pnt[1]-loc[1],2) ) 

        validate_flag = 1
    
    elif loc[0] > 6.39 and loc[1] > 1.65 and loc[1]< 8.91: 
        
        avg_pnt = (( 6.39+6.39)/2,(8.91+1.65)/2) 
        err[2] = sqrt( pow(avg_pnt[0]-loc[0], 2) + pow( avg_pnt[1]-loc[1],2) ) 
        
        avg_pnt = ((6.39+14.119)/2,(1.65+1.65)/2) 
        err[3] = sqrt( pow(avg_pnt[0]-loc[0], 2) + pow( avg_pnt[1]-loc[1],2) ) 

        validate_flag = 1

    elif (loc[0] > 6.39 and loc[1] > 7.5): 
                
        avg_pnt = ((1.65+6.39)/2,(8.91+8.91)/2) 
        err[1] = sqrt( pow(avg_pnt[0]-loc[0], 2) + pow( avg_pnt[1]-loc[1],2) ) 
        
        avg_pnt = (( 6.39+6.39)/2,(8.91+1.65)/2) 
        err[2] = sqrt( pow(avg_pnt[0]-loc[0], 2) + pow( avg_pnt[1]-loc[1],2) ) 

        validate_flag = 1

    if (validate_flag == 1):
        if  np.argmin(err) == 0:
            loc[0] = 1.65
        elif np.argmin(err) == 1:
            loc[1] = 8.91
            if loc[0] > 6.39: loc[0] = 6.39
        elif np.argmin(err) == 2:
            loc[0] = 6.39
            if loc[1] > 8.91: loc[1] = 8.91
        elif np.argmin(err) == 3:
            loc[1] = 1.65

    return loc