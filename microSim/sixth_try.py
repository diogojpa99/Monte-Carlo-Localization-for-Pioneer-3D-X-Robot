""" Libraries """

from turtle import clear, down
import numpy as np
import matplotlib.pyplot as plt
import scipy
from numpy.random import uniform
from numpy.random import normal
from math import cos, sin, sqrt, exp, pi
from scipy import stats
import cv2 
#import shapely as shapely 
#from shapely.geometry import LineString, Point



""" Functions """

# intersection between two lines 
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0: #Ver se são paralelas
       return -1 # !Atenção PODE TER QUE SER ALTERADO NO FUTURO!

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    point = (x,y)

    # intersection is out of bound
    if ( max(line1[0][0],line1[1][0]) < min(line2[0][0],line2[1][0]) or 
        max(line2[0][0],line2[1][0]) < min(line1[0][0],line1[1][0]) or
        max(line1[0][1],line1[1][1]) < min(line2[0][1],line2[1][1]) or 
        max(line2[0][1],line2[1][1]) < min(line1[0][1],line1[1][1])):

        return -1


    return point


# init_robot_pos():
# initialize the robot position
# loc[0] : x variable
# loc[1] : y variable
# loc[2] : angle
def init_robot_pos():
    loc = np.empty([3,1])
    loc[0] = 2
    loc[1] = 2
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
    loc[0] = prev_loc[0] + vel*cos(angle) + np.random.normal(loc=0.0, scale=0.2, size=None)
    loc[1] = prev_loc[1] + vel*sin(angle) + np.random.normal(loc=0.0, scale=0.2, size=None)
    loc[2] = prev_loc[2] + angle + np.random.normal(loc=0.0, scale=0.2, size=None)


    if loc[2] >= (2*pi):
        loc[2] = 0 + 2*pi - loc[2]


    return loc 

# validate_pos():
def validate_pos(loc):

    if loc[0] < 0 or loc[0] > 10 or loc[1] < 0 or loc[1] > 10:
        return 0
    else:
        return 1

# validate_particle():
# If particle leaves the map they are spread
def validate_particle(loc):


    if loc[0] < 0: loc[0] = 0
    elif loc[0] > 10: loc[0] = 10
    if loc[1] < 0: loc[1] = 0
    elif loc[1] > 10: loc[1] = 10


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
def create_particles(M):
    
    particles = np.empty([M, 3])
    particles[:, 0] = uniform(0, 10, size = M)
    particles[:, 1] = uniform(0, 10, size = M)
    particles[:, 2] = uniform(0, 2*pi, size = M)
    
    return particles
    


# predict():
# Does the predict of the Particle Filter
# We need the target distribution and adding it some random noise
# !! ATENÇÃO !! What is the mean and standard deviation to that normal noise ???
# Input:
# particles: Set of particles
# actions: vel and rotation topics
# Output:
# Set of particles in new positions 
def predict(particles, actions):

    for i in range(M):
        particles[i]  = odometry_model(particles[i], actions[0], actions[1]*(pi/180)).reshape((3,))     
        
    return particles 

#Modelo do laser
def laser_model(loc):

    measures = np.empty([N_measures,1])
    measures.fill(0.)
    radius = 0 #Variação de ângulo do laser
    reach = 100
    x = loc[0][0]
    y = loc[1][0]
    teta = loc[2][0]

    for i in range(N_measures):

        ray = np.array([(x,y), (reach*cos(teta + i*(pi/180)), reach*sin(teta + i*(pi/180))) ]) #creating line segment

        #Intersect
        up = np.array(line_intersection(up_wall, ray))
        down =np.array(line_intersection(down_wall, ray))
        left =np.array(line_intersection(left_wall, ray))
        right =np.array(line_intersection(right_wall, ray))
        
        if (line_intersection(up_wall, ray) != -1 and validate_pos(up) == 1):
            measures[i] = sqrt( pow(up[0]-x, 2) + pow(up[1]-y,2) ) + np.random.normal(loc=0.0, scale=0.1, size=None)
            #plt.scatter(up[0], up[1], c = '#d62728' )


        if (line_intersection(down_wall, ray) != -1 and validate_pos(down) == 1 ):
            measures[i] = sqrt( pow(down[0]-x, 2) + pow(down[1]-y,2) ) + np.random.normal(loc=0.0, scale=0.1, size=None)
            #plt.scatter(down[0], down[1], c = '#d62728' )


        if (line_intersection(left_wall, ray) != -1 and validate_pos(left) == 1 ):
            measures[i] = sqrt( pow( left[0]-x , 2) + pow( left[1]-y ,2) ) + np.random.normal(loc=0.0, scale=0.1, size=None)
            #plt.scatter(left[0], left[1], c = '#d62728' )
 
        if ( line_intersection(right_wall, ray) != -1 and validate_pos(right) == 1):
            measures[i] = sqrt( pow(right[0]-x, 2) + pow(right[1]-y,2) ) + np.random.normal(loc=0.0, scale=0.1, size=None)
            #plt.scatter(right[0], right[1], c = '#d62728' )

        radius += 1
        
    #measures = measures[measures != 0]
    #measures = measures.reshape((measures.shape[0],1))

    return measures

# update():
def update(measurments, particles):


    # Distance from each particle to each landmark
    # We have N_measures measures and M particles
    distances = np.empty([N_measures,M])
    distances.fill(0.)
    weights = np.empty([M,1])
    weights.fill(1.)

    for i in range (M):
        distances[:,i] = laser_model(particles[i].reshape((3,1))).reshape((N_measures,)) 

    #The weights are the product of the likelihoods of the measurments  
    for i in range (M):
        for j in range (len(measurments)):
            #print("error:",abs(measurments[j] - distances[j][i]))
            weights[i] += exp(-0.5* pow(1/0.5,2) * pow( measurments[j] - distances[j][i], 2)) 

    weights /= np.sum(weights) #Normalizar
    
    
    #print(weights)
    plt.close()
    plt.plot(weights, c = '#d62728' )
    plt.show()
    plt.close()
    

    return weights

# Resampling:
def systematic_resample(weights):
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

def plot():
    plt.plot(down_wall[0],down_wall[1], c = 'black')
    plt.plot(up_wall[0],up_wall[1], c = 'black')
    plt.plot(left_wall[0],left_wall[1], c = 'black')
    plt.plot(right_wall[0],right_wall[1], c = 'black')
    plt.scatter(robot_loc[0], robot_loc[1], marker = (3, 0, robot_loc[2]*(180/pi)), c = '#d62728' )
    for i in range(M):
        plt.scatter(particles[i,0], particles[i,1], marker = (3, 2, particles[i,2]*(180/pi)), c =  '#17becf', s = 5)
    plt.show()


""" Global Variables """

#Number of particles
M = 300

#Actions
# action[0] : cmd_vel
# action[1] : twist
actions = np.empty([2,1])

# Numver of measures of the laser model
N_measures = 180


""" main() """

# Create rectangle
down_wall = np.array([ (10,0), (0,0)]) # y = 0
up_wall = np.array([(0,10), (10,10)]) # y = 10
left_wall = np.array([(0,0), (0,10)]) # x = 0
right_wall = np.array([(10,10),(10,0)]) # x = 10

# loc: Localization of the robot
robot_loc = init_robot_pos()

#Create particles
particles = create_particles(M)

#Activationg interactive mode
plt.ion()

while(1):

    #plotting
    plot()

    #Getting info from the terminal
    print('Please input the robot velocity:')
    actions[0] = float(input())
    print('Please input the robot rotation')
    actions[1] = float(input())

    # Erase previous robot position
    plt.scatter(robot_loc[0], robot_loc[1], c = 'white', s = 60 )
    plt.scatter(particles[:,0], particles[:,1], c = 'white', s = 60 )

    # Update localization of the robot
    robot_loc = odometry_model(robot_loc, actions[0], actions[1]*(pi/180))
    if validate_pos(robot_loc) == 0:
        exit(0) #simulation terminates

    # PREDICT
    particles = predict(particles, actions)
    for i in range (M):
        particles[i] = validate_particle(particles[i])
    
    plot()
    
    # Retrieving data from the laser
    robot_measures = laser_model(robot_loc)

    # UPDATE
    weights = update(robot_measures,particles)

    
    # RESAMPLING
    indexes = systematic_resample(weights)
    particles[:] = particles[indexes]
    weights[:] = weights[indexes]
    weights /= np.sum(weights)