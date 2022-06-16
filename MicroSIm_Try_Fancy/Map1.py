""" *********************************** Libraries ************************************************* """

import numpy as np
from math import sqrt


""" ************************************* Global Variables ****************************************  """

''' Map '''

sim_flag = 1

# Create Map
map = np.array([[(1,0), (15,0)],
                [(15, 0), (15,4)],
                [(15,4), (6,4)],
                [(6,4), (5,3)],
                [(5,3), (4,4)],
                [(4,4), (3, 4)],
                [(3,4), (3,12)],
                [(3,15), (1.5,14)],
                [(1.5,14),(0,15)],
                [(0,15), (0,0)]] ) 

# Number of walls
n_walls = map.shape[0]

# Rectangle
lower = 0
upper = 15


"""  ************************************ Functions  ******************************************** """


# Validate Position
# Input: 
# loc = Array that represent the location of the particle/robot
# Output: 
# Function returns '0' if position is invalid, returns '1' if it is valid

def validate_pos(loc):

    if loc[0] < lower - 0.1 or loc[0] > upper + 0.1 or loc[1] < lower - 0.1 or loc[1] > 15 + 0.1:
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

    if loc[0] > 3 and loc[1] > 4 : 
        avg_pnt = ((3+3)/2,(3+15)/2) 
        err1 = sqrt( pow(avg_pnt[0]-loc[0], 2) + pow( avg_pnt[1]-loc[0],2) ) 
        avg_pnt = ((6+15)/2,(3+3)/2) 
        err2 = sqrt( pow(avg_pnt[0]-loc[0], 2) + pow( avg_pnt[1]-loc[0],2) ) 
        if err1 < err2 :
            loc[0] = 3
        else:
            loc[1] = 4
    

    return loc

    
