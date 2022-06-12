""" Libraries """

from turtle import clear, down
import numpy as np
import matplotlib.pyplot as plt
import scipy
from numpy.random import uniform
from numpy.random import normal
from math import cos, sin, sqrt, exp, pi
from scipy import stats
from scipy.stats import gaussian_kde
import cv2 

""" Global Variables """

#Rectangle:
lower = 0
upper = 10


#Actions
actions = np.array([(1,90),(1,90),(1,45),(1,45),(1,45), (1,45), (1,0),(1,0),(1,285), (1,285), (1,285),
                    (1,235),(1,235), (1,180), (1,180), (1,135), (1,90), (1,90), (1,45)])

last_iteration = actions.shape[0]



""" Functions """

# init_robot_pos():
# initialize the robot position
# loc[0] : x variable
# loc[1] : y variable
# loc[2] : angle
def init_robot_pos():
    loc = np.empty([3,1])
    loc[0] = 2
    loc[1] = 3
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
    loc[0] = prev_loc[0] + vel*cos(prev_loc[2]) 
    loc[1] = prev_loc[1] + vel*sin(prev_loc[2]) 
    loc[2] = prev_loc[2] + angle 


    if loc[2] >= (2*pi):
        loc[2] = 0 + 2*pi - loc[2]


    return loc 

# odometry model with noise
def odometry_model_noise(prev_loc, vel, angle):
    loc = np.empty([3,1])

    # Target distribution
    loc[2] = prev_loc[2] + angle + np.random.normal(loc=0.0, scale=0.01, size=None)
    loc[0] = prev_loc[0] + vel*cos(prev_loc[2]) + np.random.normal(loc=0.0, scale=0.02, size=None)
    loc[1] = prev_loc[1] + vel*sin(prev_loc[2]) + np.random.normal(loc=0.0, scale=0.02, size=None)

    if loc[2] >= (2*pi):
        loc[2] = 0 + 2*pi - loc[2]


    return loc 

# validate_pos():
def validate_pos(loc):

    if loc[0] < lower or loc[0] > upper or loc[1] < lower or loc[1] > upper:
        return 0
    else:
        return 1

# Plot the map
def plot(label):

    plt.title(label)
    plt.plot(down_wall[0],down_wall[1], c = 'black')
    plt.plot(up_wall[0],up_wall[1], c = 'black')
    plt.plot(left_wall[0],left_wall[1], c = 'black')
    plt.plot(right_wall[0],right_wall[1], c = 'black')
    plt.scatter(loc_noise[:,0], loc_noise[:,1], c = '#e377c2' , s=70, label = "Position with noise")
    plt.scatter(robot_loc[0], robot_loc[1], marker = (6, 1, robot_loc[2]*(180/pi)), c = 'green' , s=70, label = "Real position")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend(loc='upper left')
    plt.show()

    return





""" main() """

# Create rectangle
down_wall = np.array([ (upper,lower), (lower,lower)]) # y = 0
up_wall = np.array([(lower,upper), (upper,upper)]) # y = 10
left_wall = np.array([(lower,lower), (lower,upper)]) # x = 0
right_wall = np.array([(upper,upper),(upper,lower)]) # x = 10

# loc: Localization of the robot
robot_loc = init_robot_pos()

# loc: Localization of the robot
loc_noise = np.empty([5,3])


for i in range(5):
    loc_noise[i][0] = 2
    loc_noise[i][1] = 3
    loc_noise[i][2] = 0    



#Start simulation
print("Simulation has started!")
k = 0
while(1):

    #plotting
    plot('Map after Resampling')


    # Update localization of the robot
    robot_loc = odometry_model(robot_loc, actions[k][0], actions[k][1]*(pi/180))
    if validate_pos(robot_loc) == 0:
        break #simulation terminates

    # Update localization of the robot
    for i in range(5):
        loc_noise[i] = odometry_model_noise(loc_noise[i], actions[k][0], actions[k][1]*(pi/180)).reshape((1,3))
        if validate_pos(loc_noise[i]) == 0:
            break #simulation terminates
    
    # Error 

    print("Error:", abs(robot_loc[0][0] - loc_noise[0][0]), abs(robot_loc[1][0] - loc_noise[1][0]), abs(robot_loc[2][0] - loc_noise[2][0])*(180/pi))

    k +=1

    if ( k == last_iteration):
        break