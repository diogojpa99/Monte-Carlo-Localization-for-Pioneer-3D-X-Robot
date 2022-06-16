""" *********************************** Libraries ************************************************* """

import numpy as np
from math import cos, sin, sqrt, exp, pi, radians

import plots as pl
import Map1 as map1
if ( map1.sim_flag == 1):
    selected_map = map1


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

def odometry_model(loc, deltaD, rotation, particle_flag, uncertainty):
        
    # Target distribution
    loc[0] = loc[0] + deltaD*cos(loc[2]) 
    loc[1] = loc[1] + deltaD*sin(loc[2]) 
    loc[2] = loc[2] + rotation 
    
    if (particle_flag == 1): #If it is a particle we add uncertanty

        loc[0] += np.random.normal(loc=0.0, scale=uncertainty[0], size=None)
        loc[1] += np.random.normal(loc=0.0, scale=uncertainty[1], size=None)
        loc[2] += np.random.normal(loc=0.0, scale=uncertainty[2], size=None)

    if loc[2] >= (2*pi):
        loc[2] = loc[2] - 2*pi
    
    return loc 


# PREDICT
def predict(particles, actions, M, odom_uncertainty):

    for i in range(M):
        particles[i]  = odometry_model(particles[i], actions[0], actions[1]*(pi/180), 1, odom_uncertainty).reshape((3,))     
        
    return particles 


# Laser Model
def laser_model(loc, measures, N_measures, n_walls, radius_var, robot_loc, reach, radius, laser_uncertanty):

    # The measures are the intersections between the laser rays and the walls
    measures.fill(0.)

    x1 = loc[0][0]
    y1 = loc[1][0]
    teta = loc[2][0]
    
    # Variable that determines which wall the laser intersected
    # An laser ray only can intersect one wall
    right_wall = 0

    for i in range(N_measures):

        prev_err = 100000
        
        #Creating line segment
        ray = np.array([(x1,y1), (x1+reach*cos(teta + radians(radius)), y1+reach*sin(teta + radians(radius))) ]) 
        '''
        if loc[0] == robot_loc[0] and loc[1] == robot_loc[1] and loc[2] == robot_loc[2]  :
           plt.plot((x1,reach*cos(teta + radians(radius))+x1),(y1,reach*sin(teta + radians(radius))+y1) , c = '#17becf')
        '''

        #Intersect Between the laser ray an a wall
        for j in range(n_walls):
            intersect = np.array(line_intersection(selected_map.map[j], ray))

            # The right wall is the one that is closest to the point
            if (intersect.shape == (2,) and selected_map.validate_pos(intersect) == 1):

                avg_pnt = ((selected_map.map[j][0][0]+selected_map.map[j][1][0])/2,(selected_map.map[j][0][1]+selected_map.map[j][1][1])/2) 
                err = sqrt( pow(avg_pnt[0]-x1, 2) + pow( avg_pnt[1]-y1,2) ) 

                if err < prev_err:
                    prev_err = err
                    right_wall = j
                    
        intersect = np.array(line_intersection(selected_map.map[right_wall], ray))

        if (intersect.shape == (2,) and selected_map.validate_pos(intersect) == 1):

            #Adding noise two the measures
            x2 = intersect[0] + np.random.normal(loc=0.0, scale= laser_uncertanty, size=None) 
            y2 = intersect[1] + np.random.normal(loc=0.0, scale= laser_uncertanty, size=None)

            # The measure is the distance between the position of the robot and the intersection
            # of the ray and the wall
            measures[i] = sqrt( pow(x2-x1, 2) + pow( y2-y1,2) )
            if loc[0] == robot_loc[0] and loc[1] == robot_loc[1] and loc[2] == robot_loc[2]  :
                pl.plot_laser(x2,y2)      
             

        radius += radius_var
        
    return measures

# normal_distribution():
def normal_dist(x , mean , sd):

    prob = exp( - pow( x - mean, 2) / (2*pow(sd,2)) ) / (sd*sqrt(2*pi))  

    return prob

# UPDATE
def update(w, measurments, particles, resampling_flag, likelihood_avg, N_measures, M, sd, laser_radius_var, robot_loc, laser_reach, laser_radius, laser_uncertanty):

    # Distance from each particle to each landmark
    # We have N_measures measures and M particles
    distances = np.empty([N_measures,M])
    distances.fill(0.)

    if resampling_flag == 1:
        w.fill(1.)
    elif resampling_flag == 0:
        prev_weights = w

    #Standard deviation
    sd = 4

    # Compute the measures for each particle
    for i in range (M):
        distances[:,i] = laser_model(particles[i].reshape((3,1)), measurments, N_measures, selected_map.n_walls, laser_radius_var, robot_loc, laser_reach, laser_radius, laser_uncertanty ).reshape((N_measures,)) 

    #The weights are the product of the likelihoods of the measurments  
    for i in range (M):

        for j in range (N_measures):
                w[i] *= normal_dist(measurments[j], distances[j][i], sd)

        w[i] *= pow(10,15) 

        if ( resampling_flag == 0):
            w[i] = w[i] * prev_weights[i]
        
    # Likelihood average for kidnapping     
    prev_likelihood_avg = likelihood_avg
    likelihood_avg = np.average(w)
    print("Likelihood Avg:", likelihood_avg)
    if(likelihood_avg < pow(10,-18) and prev_likelihood_avg < pow(10,-18)):
        particles[0:int(M*(3/4))-1] = selected_map.create_particles(int(M*(3/4))-1)
        for i in range (M):
            particles[i] = selected_map.validate_loc(particles[i])

    #Normalise
    w = w / w.sum() 
    
    #If all the weigths are the same do not resample
    if np.all(w == w[0]):
        resampling_flag = 0

    return w,likelihood_avg

# RESAMPLING
def low_variance_resample(weights):
    N = len(weights)
    positions = (np.arange(N) + np.random.random()) / N
 
    indexes = np.zeros(N, 'i')
    cumulative_sum = np.cumsum(weights)
    i, j = 0, 0
    while i < N and j<N:
        if positions[i] < cumulative_sum[j]:
            indexes[i] = j
            i += 1
        else:
            j += 1
    return indexes

