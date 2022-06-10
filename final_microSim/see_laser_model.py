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
# !Atenção POSSO TER QUE ALTERAR O RETORNO '-1' NO FUTURO
def line_intersection(line1, line2):

    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0: #Ver se são paralelas
       return -1 

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    """
    print('inspect:')
    print(min ( min(line1[0][0],line1[1][0]), min(line2[0][0],line2[1][0]) ))
    print(max( max(line1[0][0],line1[1][0]), max(line2[0][0],line2[1][0])))
    print(x)
    print('end')
    """

    """
    if ((x < max( min(line1[0][0],line1[1][0]), min(line2[0][0],line2[1][0]) )) or 
        (x > min( max(line1[0][0],line1[1][0]), max(line2[0][0],line2[1][0]))) or
        (y < max ( min(line1[0][1],line1[1][1]), min(line2[0][1],line2[1][1]) )) or 
        (y > min( max(line1[0][1],line1[1][1]), max(line2[0][1],line2[1][1])))):
        
        print('ups:',point)
        return -1  # intersection is out of bound
    """

    if ( max(line1[0][0],line1[1][0]) < min(line2[0][0],line2[1][0]) or 
        max(line2[0][0],line2[1][0]) < min(line1[0][0],line1[1][0]) or
        max(line1[0][1],line1[1][1]) < min(line2[0][1],line2[1][1]) or 
        max(line2[0][1],line2[1][1]) < min(line1[0][1],line1[1][1])):

        return -1

    return (x,y)


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
    loc[0] = prev_loc[0] + vel*cos(angle) + np.random.normal(loc=0.0, scale=0.1, size=None)
    loc[1] = prev_loc[1] + vel*sin(angle) + np.random.normal(loc=0.0, scale=0.1, size=None)
    loc[2] = prev_loc[2] + angle + np.random.normal(loc=0.0, scale=0.1, size=None)


    return loc 

# validate_pos():
def validate_pos(loc):

    if loc[0] < 0 or loc[0] > 10.1 or loc[1] < 0 or loc[1] > 10.1:
        return 0
    else:
        return 1

# validate_particle():
# If particle leaves the map they are spread
def validate_particle(loc):

    if loc[0] < -0.25 or loc[0] > 10.25 or loc[1] < -0.25 or loc[1] > 10.25:
        loc[0] = loc[1] = uniform(0, 10, size = 1) 

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
        particles[i]  = odometry_model(particles[i], actions[0], actions[1]).reshape((3,))   
       
        
    return particles 

#Modelo do laser
def laser_model(loc):

    radius = 0 #Variação de ângulo do laser
    reach = 15
    x = loc[0][0]
    y = loc[1][0]
    teta = loc[2][0]

    for i in range(10):

        ray = np.array([(x,y), (reach*cos(teta + radius*(pi/180)), reach*sin(teta + radius*(pi/180))) ]) #creating line segment

        #plotting
        plt.plot((x,reach*cos(teta + radius*(pi/180))),(y,reach*sin(teta + radius*(pi/180))) , c = '#17becf')


        #Intersect
        up = np.array(line_intersection(up_wall, ray))
        down =np.array(line_intersection(down_wall, ray))
        left =np.array(line_intersection(left_wall, ray))
        right =np.array(line_intersection(right_wall, ray))
        
        if (line_intersection(up_wall, ray) != -1 and validate_pos(up) == 1):
            x2 = up[0] + np.random.normal(loc=0.0, scale=0.07, size=None) #Adding noise two the measures
            y2 = up[1] + np.random.normal(loc=0.0, scale=0.07, size=None)
            plt.scatter(x2, y2, c = '#d62728' )


        elif (line_intersection(down_wall, ray) != -1 and validate_pos(down) == 1 ):
            x2 = down[0] + np.random.normal(loc=0.0, scale=0.07, size=None)
            y2 = down[1] + np.random.normal(loc=0.0, scale=0.07, size=None)
            plt.scatter(x2, y2, c = '#d62728' )


        elif (line_intersection(left_wall, ray) != -1 and validate_pos(left) == 1 ):
            x2 = left[0] + np.random.normal(loc=0.0, scale=0.07, size=None)
            y2 = left[1] + np.random.normal(loc=0.0, scale=0.07, size=None)
            plt.scatter(x2, y2, c = '#d62728' )
 
        elif ( line_intersection(right_wall, ray) != -1 and validate_pos(right) == 1):
            x2 = right[0] + np.random.normal(loc=0.0, scale=0.07, size=None)
            y2 = right[1] + np.random.normal(loc=0.0, scale=0.07, size=None)
            plt.scatter(x2, y2, c = '#d62728' )
        
        radius += 2
    
    return

# normal_distribution():
def normal_dist(x , mean , sd):
    prob = (1/(sd*sqrt(2*pi)))*exp(-0.5*((x - mean)/sd)**2) 
    return prob


""" Global Variables """

#Number of particles
M = 100

#Actions
# action[0] : cmd_vel
# action[1] : twist
actions = np.empty([2,1])



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

#Getting info from the terminal
print('Please input the robot velocity:')
actions[0] = float(input())
print('Please input the robot rotation')
actions[1] = float(input())

#Activationg interactive mode
#plt.ion()

"""
int_pt = up_wall.intersection(left_wall)
point_of_intersection = int_pt.x, int_pt.y

print(point_of_intersection)

int_pt = up_wall.intersection(down_wall)
point_of_intersection = int_pt.x, int_pt.y

print(point_of_intersection)
"""

while(1):

    #plotting
    plt.plot(down_wall[0],down_wall[1], c = 'black')
    plt.plot(up_wall[0],up_wall[1], c = 'black')
    plt.plot(left_wall[0],left_wall[1], c = 'black')
    plt.plot(right_wall[0],right_wall[1], c = 'black')
    plt.scatter(robot_loc[0], robot_loc[1], marker = (3, 0, robot_loc[2]*(180/pi)), c = '#d62728' )
    #for i in range(M):
        #plt.scatter(particles[i,0], particles[i,1], marker = (3, 2, particles[i,2]*(180/pi)), c =  '#17becf', s = 5)
    plt.show()

    # Update localization of the robot
    robot_loc = odometry_model(robot_loc, actions[0], actions[1]*(pi/180))
    if validate_pos(robot_loc) == 0:
        exit(0) #simulation terminates
    
    # PREDICT
    particles = predict(particles, actions)
    for i in range (M):
        particles[i] = validate_particle(particles[i])

    laser_model(robot_loc)

   


