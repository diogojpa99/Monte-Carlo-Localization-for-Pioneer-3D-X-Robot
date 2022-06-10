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

#Number of particles
M = 250

# Number of measures of the laser model
N_measures = 20

# Angle of the laser variation
radius_var = 12

#Actions
# action[0] : cmd_vel1
# action[1] : twist
actions = np.empty([2,1])

#Errors
errors = np.empty([upper + 15,3])




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

    # intersection is out of bound
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
    loc[0] = 4
    loc[1] = 6
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
    loc[2] = prev_loc[2] + angle + np.random.normal(loc=0.0, scale=0.1, size=None)


    if loc[2] >= (2*pi):
        loc[2] = 0 + 2*pi - loc[2]


    return loc 


# validate_pos():
def validate_pos(loc):

    if loc[0] < lower or loc[0] > upper or loc[1] < lower or loc[1] > upper:
        return 0
    else:
        return 1



# validate_particle():
# If particle leaves the map they are spread
def validate_particle(loc):


    if loc[0] < lower: loc[0] = lower
    elif loc[0] > upper: loc[0] = upper
    if loc[1] < lower: loc[1] = lower
    elif loc[1] > upper: loc[1] = upper


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
    particles[:, 0] = uniform(lower, upper, size = M)
    particles[:, 1] = uniform(lower, upper, size = M)
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
    reach = 4
    x1 = loc[0][0]
    y1 = loc[1][0]
    teta = loc[2][0]

    for i in range(N_measures):

        ray = np.array([(x1,y1), (reach*cos(teta + radius*(pi/180)), reach*sin(teta + radius*(pi/180))) ]) #creating line segment

        #Intersect
        up = np.array(line_intersection(up_wall, ray))
        down =np.array(line_intersection(down_wall, ray))
        left =np.array(line_intersection(left_wall, ray))
        right =np.array(line_intersection(right_wall, ray))
        
        if (line_intersection(up_wall, ray) != -1 and validate_pos(up) == 1):
            x2 = up[0] + np.random.normal(loc=0.0, scale=0.07, size=None) #Adding noise two the measures
            y2 = up[1] + np.random.normal(loc=0.0, scale=0.07, size=None)
            measures[i] = sqrt( pow(x2-x1, 2) + pow( y2-y1,2) ) 
            #plt.scatter(up[0], up[1], c = '#d62728' )


        elif (line_intersection(down_wall, ray) != -1 and validate_pos(down) == 1 ):
            x2 = down[0] + np.random.normal(loc=0.0, scale=0.07, size=None)
            y2 = down[1] + np.random.normal(loc=0.0, scale=0.07, size=None)
            measures[i] = sqrt( pow(x2-x1, 2) + pow( y2-y1,2) ) 
            #plt.scatter(down[0], down[1], c = '#d62728' )


        elif (line_intersection(left_wall, ray) != -1 and validate_pos(left) == 1 ):
            x2 = left[0] + np.random.normal(loc=0.0, scale=0.07, size=None)
            y2 = left[1] + np.random.normal(loc=0.0, scale=0.07, size=None)
            measures[i] = sqrt( pow(x2-x1, 2) + pow( y2-y1,2) ) 
            #plt.scatter(left[0], left[1], c = '#d62728' )
 
        elif ( line_intersection(right_wall, ray) != -1 and validate_pos(right) == 1):
            x2 = right[0] + np.random.normal(loc=0.0, scale=0.07, size=None)
            y2 = right[1] + np.random.normal(loc=0.0, scale=0.07, size=None)
            measures[i] = sqrt( pow(x2-x1, 2) + pow( y2-y1,2) ) 
            #plt.scatter(right[0], right[1], c = '#d62728' )

        radius += radius_var
        
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
    weights.fill(1./M)
    sd = 0.5

    for i in range (M):
        distances[:,i] = laser_model(particles[i].reshape((3,1))).reshape((N_measures,)) 

    #The weights are the product of the likelihoods of the measurments  
    for i in range (M):
        for j in range (len(measurments)):
            #print("error:",abs(measurments[j] - distances[j][i]))
            weights[i] += exp(-0.5* pow(1/sd,2) * pow( measurments[j] - distances[j][i], 2)) 

    weights /= np.sum(weights) #Normalizar
    
    
    #print(weights)
    

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


# Plot the map
def plot(label):

    # Calculate the point density of the particles
    x = particles[:,0]
    y = particles[:,1]
    xy = np.vstack([x,y])
    z = gaussian_kde(xy)(xy)

    # Sort the points by density, so that the densest points are plotted last
    idx = z.argsort()
    x, y, z = x[idx], y[idx], z[idx]
    plt.title(label)
    plt.scatter(x, y, c=z, s=5, label = "particles")
    plt.plot(down_wall[0],down_wall[1], c = 'black')
    plt.plot(up_wall[0],up_wall[1], c = 'black')
    plt.plot(left_wall[0],left_wall[1], c = 'black')
    plt.plot(right_wall[0],right_wall[1], c = 'black')
    #for i in range(M):
       #plt.scatter(particles[i,0], particles[i,1], marker = (3, 2, particles[i,2]*(180/pi)), c =  z, s = 5)
    plt.scatter(robot_loc[0], robot_loc[1], marker = (6, 1, robot_loc[2]*(180/pi)), c = '#d62728' , s=70, label = "Real position")
    plt.xlabel("x")
    plt.ylabel("y")
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

#Create particles
particles = create_particles(M)

#Activationg interactive mode
plt.ion()

#Start simulation
print("Simulation has started!")
k = 0
errors.fill(0.)
while(1):

    #plotting
    plot('Map after Resampling')

    #Getting info from the terminal
    print('Please input the robot traveled distance:')
    actions[0] = float(input())
    print('Please input the robot rotation')
    actions[1] = float(input())

    # Erase previous robot position
    plt.clf()


    # Update localization of the robot
    robot_loc = odometry_model(robot_loc, actions[0], actions[1]*(pi/180))
    if validate_pos(robot_loc) == 0:
        break #simulation terminates
    
    # PREDICT
    particles = predict(particles, actions)
    for i in range (M):
        particles[i] = validate_particle(particles[i])
    plot('Map after Predict')
    
    # Retrieving data from the laser
    robot_measures = laser_model(robot_loc)

    # UPDATE
    weights = update(robot_measures,particles)

    
    # RESAMPLING
    indexes = systematic_resample(weights)
    particles[:] = particles[indexes]
    weights[:] = weights[indexes]
    weights /= np.sum(weights)

    #Output
    print("Iteration nº:",k+1)
    print('Real Localization:', robot_loc[0][0],robot_loc[1][0],robot_loc[2][0]*(180/pi))
    # Centroid of the cluster of particles
    print("Predicted Localization:", np.average(particles[:,0]), np.average(particles[:,1]), np.average(particles[:,2])*(180/pi))
    errors[k][0] = abs(np.average(particles[:,0])-robot_loc[0][0])
    errors[k][1] = abs(np.average(particles[:,1])-robot_loc[1][0])
    errors[k][2] = abs(np.average(particles[:,2])-robot_loc[2][0])
    print("Error:",errors[k][0], errors[k][1], errors[k][2]*(180/pi))

    k +=1

#Plot errors
x_error =  errors[:,0]
x_error = x_error[ x_error !=0]
y_error =  errors[:,1]
y_error = y_error[ y_error !=0]
theta_error =  errors[:,2]
theta_error = theta_error[ theta_error !=0]

plt.ioff()
plt.close()
plt.title('errors')
plt.plot(x_error, c = '#bcbd22', label = "x" )
plt.plot(y_error, c = '#9467bd', label = "y" )
plt.plot(theta_error, c = '#e377c2', label = "theta" )
plt.xlabel("Number of iterations")
plt.ylabel("errors")
plt.legend(loc='upper left')
plt.show()
plt.close()