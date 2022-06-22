""" *********************************** Libraries ************************************************* """

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, sqrt, exp, pi, radians

import plots as pl
    
""" ************************************* Global Variables ****************************************  """

''' Weights '''

# Our Likelihood is the sum between
# an normal distrbution and an uniform distribution

w2 = 0.4
w1 = 1 - w2

''' Robot '''

# Odometry uncertainty
# odom_uncertainty[0]: x
# odom_uncertainty[1]: y
# odom_uncertainty[2]: Rotation
odom_uncertainty = (0.08,0.08,0.08)

''' Laser '''

N_measures = 24 # Number of measures of the laser model
laser_reach = 5.6
laser_radius_var = 10 # Angle of the laser variation
laser_uncertanty = 0.05

''' Optimize the algorithm '''

likelihood_sd = 1
likelihood_avg_thresh = pow(10,-6)


"""  ************************************ Functions  *********************************************** """

# line_intersection(line1,line2):
# Abstract: Intersection between two lines
# Output: The intersection point between two lines (if it exist)
# If there is no intersection the fuction returns '-1'

def line_intersection(line1, line2):

    m1 = (line1[1][1] - line1[0][1]) / (line1[1][0] - line1[0][0])
    m2 = (line2[1][1] - line2[0][1]) / (line2[1][0] - line2[0][0])
    b1 = line1[0][1] - m1*line1[0][0]
    b2 = line2[0][1] - m2*line2[0][0]


    
    if (m2-m1) == 0: # Lines are parallel
         return -1 

    elif (line1[1][0] - line1[0][0]) == 0: # Line: x = line1[0][0]

        x = line1[0][0]
        y = m2*x + b2

    elif (line2[1][0] - line2[0][0]) == 0:

        x = line2[0][0]
        y = m1*x + b1
    
    elif ( m1 == 0):

        y = b1
        x = (y-b2)/m2
    
    elif ( m2 == 0):

        y = b2
        x = (y-b1)/m1
    
    else:

        #y1 = y2
        x = (b1-b2)/(m2-m1)
        y = m1*x + b1

    # Intersection is in bound    
    if (((x >= max( min(line1[0][0],line1[1][0]), min(line2[0][0],line2[1][0]) )) and
        (x <= min( max(line1[0][0],line1[1][0]), max(line2[0][0],line2[1][0])))) 
        and
        ((y >= max ( min(line1[0][1],line1[1][1]), min(line2[0][1],line2[1][1]))) and
        (y <= min( max(line1[0][1],line1[1][1]), max(line2[0][1],line2[1][1])))) ):

        return (x,y)  

    return -1


# odometry_model():
# Input:
# prev_loc: Localization of the robot at time t
# deltaD : Distance traveled between time t and time t+1
# rotation : Rotation between time t and time t+1
# Output: The localization of the robot at time t+1

def odometry_model(loc, deltaD, rotation, particle_flag):
        
    # Target distribution
    loc[0] = loc[0] + deltaD*cos(loc[2]) 
    loc[1] = loc[1] + deltaD*sin(loc[2]) 
    loc[2] = loc[2] + rotation 
    
    if (particle_flag == 1): #If it is a particle we add uncertanty

        loc[0] += np.random.normal(loc=0.0, scale= odom_uncertainty[0], size=None)
        loc[1] += np.random.normal(loc=0.0, scale= odom_uncertainty[1], size=None)
        loc[2] += np.random.normal(loc=0.0, scale= odom_uncertainty[2], size=None)
    
    return loc 


# PREDICT
def predict(particles, actions, M):

    particles = particles.reshape([M,3])

    for i in range(M):
        particles[i]  = odometry_model(particles[i], actions[0], actions[1]*(pi/180), 1).reshape((3,))     
        
    return particles 


# Laser Model
def laser_model(loc, n_walls, robot_loc, selected_map):

    # The measures are the intersections between the laser rays and the walls
    measures = np.empty([N_measures,1])
    measures.fill(0.)

    x1 = loc[0][0]
    y1 = loc[1][0]
    teta = loc[2][0]

    # Laser radius
    laser_radius = -119 #Variação de ângulo do laser
    
    # An laser ray only can intersect one wall
    right_intersect = np.empty([2,])

    for i in range(N_measures):

        prev_err = 1000
        right_intersect[0] = x1
        right_intersect[1] = y1
        
        #Creating line segment
        ray = np.array([(x1,y1), (x1+laser_reach*cos(teta + radians(laser_radius)), y1+laser_reach*sin(teta + radians(laser_radius))) ]) 
        '''if loc[0] == robot_loc[0] and loc[1] == robot_loc[1] and loc[2] == robot_loc[2]  :
           plt.plot((x1,laser_reach*cos(teta + radians(laser_radius))+x1),(y1,laser_reach*sin(teta + radians(laser_radius))+y1) , c = '#17becf')'''

        #Intersect Between the laser ray an a wall
        for j in range(n_walls):
            intersect = np.array(line_intersection(selected_map.map[j], ray))

            # The right wall is the one that is closest to the point
            if (intersect.shape == (2,) and selected_map.validate_pos(intersect) == 1):
                
                err = sqrt( pow( intersect[0] -x1, 2) + pow( intersect[1]-y1,2) ) 

                if err < prev_err:
                    prev_err = err
                    right_intersect = intersect

        #Adding noise two the measures
        x2 = right_intersect[0] + np.random.normal(loc=0.0, scale= laser_uncertanty, size=None) 
        y2 = right_intersect[1] + np.random.normal(loc=0.0, scale= laser_uncertanty, size=None)

        # The measure is the distance between the position of the robot and the intersection
        # of the ray and the wall
        measures[i] = sqrt( pow(x2-x1, 2) + pow( y2-y1,2) )
        if loc[0] == robot_loc[0] and loc[1] == robot_loc[1] and loc[2] == robot_loc[2]  :
            pl.plot_laser(x2,y2)      
             
        laser_radius += laser_radius_var
        
    return measures

# normal_distribution():
def normal_dist(x , mean , sd):

    prob = exp( - pow( x - mean, 2) / (2*pow(sd,2)) ) / (sd*sqrt(2*pi))  

    return prob

# UPDATE
def update(w, robot_measurments, particles, resampling_flag, likelihood_avg, M, robot_loc, selected_map):

    # Distance from each particle to each landmark
    # We have N_measures measures and M particles
    distances = np.empty([N_measures,M])
    distances.fill(0.)

    resize_flag = 0 # Flag that determines if we are going to change the numbe rof particles

    if resampling_flag == 1:
        w = np.empty([M,1])
        w.fill(1.)
    elif resampling_flag == 0:
        w = w[0:M]
        prev_weights = w

    # Compute the measures for each particle
    for i in range (M):
        distances[:,i] = laser_model(particles[i].reshape((3,1)), selected_map.n_walls, robot_loc, selected_map).reshape((N_measures,))

    #The weights are the product of the likelihoods of the measurments  
    for i in range (M):

        for j in range(N_measures):
            w[i] *= ( w1*normal_dist(robot_measurments[j], distances[j][i], likelihood_sd) + w2*(1/laser_reach))

        w[i] *= pow(10,15) 

        if ( resampling_flag == 0):
            w[i] = w[i] * prev_weights[i]
        
    # Likelihood average for kidnapping     
    prev_likelihood_avg = likelihood_avg
    print("Likelihood_avg: ",likelihood_avg)
    likelihood_avg = np.average(w)
    
    # We assume that there was a kidnapping 
    if(likelihood_avg <  likelihood_avg_thresh  and prev_likelihood_avg < likelihood_avg_thresh ):
        resize_flag = 1 #We increase the number

    #Normalise
    w /= sum(w) 
    
    #If all the weigths are the same do not resample
    if np.all(w == w[0]):
        resampling_flag = 0
        

    return w,likelihood_avg, resize_flag

# RESAMPLING
def low_variance_resample(weights, M, particles):

    positions = (np.arange(M) + np.random.random()) / M
    indexes = np.zeros(M, 'i')
    cumulative_sum = np.cumsum(weights)
    i, j = 0, 0

    while i < M and j<M:
        if positions[i] < cumulative_sum[j]:
            indexes[i] = j
            i += 1
        else:
            j += 1

    particles[:] = particles[indexes]

    return particles

