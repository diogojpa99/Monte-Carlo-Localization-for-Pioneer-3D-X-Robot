""" Libraries """

from turtle import clear
import numpy as np
import matplotlib.pyplot as plt
import scipy
from numpy.random import uniform
from numpy.random import normal
from math import cos, sin, sqrt, exp, pi
from scipy import stats
import cv2 as cv2

""" Functions """

# import_image():
def import_map(file_name):

    img = cv2.imread(file_name)
    print('img:',img.shape)

    grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print('grayscale:', grayscale.shape)

    map = np.array(grayscale)
    print('map:',map.shape)

    # 250 -> Outside
    # 254 -> Free space
    # 0 -> Wall
    
    return map

# init_robot_pos():
# initialize the robot position
# loc[0] : x variable
# loc[1] : y variable
# loc[2] : angle
def init_robot_pos():
    loc = np.empty([3,1])
    loc[0] = int(250)
    loc[1] = int(250)
    loc[2] = int(0)

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
    loc[0] = prev_loc[0] + vel*cos(angle) 
    loc[1] = prev_loc[1] + vel*sin(angle)
    loc[2] = prev_loc[2] + angle 


    return loc 



""" Global Variables """

#Number of particles
M = 300

#Actions
# action[0] : cmd_vel
# action[1] : twist
actions = np.empty([2,1])




""" main() """

# import the map
map = import_map('map.pgm')

# loc: Localization of the robot
robot_loc = init_robot_pos()

#Activationg interactive mode
plt.ion()





while(1):

    #plotting
    #plt.scatter(robot_loc[0], robot_loc[1], marker = (3, 0, robot_loc[0]), c = 'red' )
    if map[int(robot_loc[0]),int(robot_loc[1])] == 254:
        map[int(robot_loc[0]),int(robot_loc[1])] = 0
    plt.imshow(map)
    plt.show() 

    #Getting info from the terminal
    print('Please input the robot velocity:')
    actions[0] = float(input())
    print('Please input the robot rotation')
    actions[1] = float(input())


    # Update localization of the robot
    robot_loc = odometry_model(robot_loc, actions[0], actions[1])
    print("Robot_Loc:",robot_loc)

   
