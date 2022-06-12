""" *********************************** Libraries *********************************************** """

from turtle import clear, down
import numpy as np
import matplotlib.pyplot as plt
import scipy
from numpy.random import uniform
from numpy.random import normal
from math import cos, sin, sqrt, exp, pi, radians, degrees
from scipy import stats
from scipy.stats import gaussian_kde
import matplotlib as mpl
import cv2 





""" ************************************* Global Variables ****************************************  """

# Create Map
map = np.array([ [ (0,0), (0,15)],
                [ (0,15), (9,15)],
                [ (9,10), (9,15)],
                [ (9,10), (11,10)], 
                [ (11,10), (11,15)], 
                [ (11,15), (15,15)], 
                [ (15,6), (15,15)], 
                [ (13,6), (15,6)], 
                [ (13,0), (13,6)], 
                [ (6,0), (13,0)], 
                [ (6,0), (6,4)], 
                [ (3,4), (6,4)],
                [ (3,0), (3,4)],
                [ (0,0), (3,0)] ] ) 

#Rectangle:
lower = 0
upper = 15

#Number of particles
M = 500

# Number of measures of the laser model
N_measures = 10

# Angle of the laser variation
radius_var = 24

#Actions
# action[0] : cmd_vel1
# action[1] : twist
actions = np.empty([2,1])

#Errors
errors = np.empty([50,3])




"""  ************************************ Functions  ******************************************** """


# intersection between two lines 
def line_intersection(line1, line2):

    m1 = (line1[1][1] - line1[0][1]) / (line1[1][0] - line1[0][0])
    m2 = (line2[1][1] - line2[0][1]) / (line2[1][0] - line2[0][0])
    b1 = line1[0][1] - m1*line1[0][0]
    b2 = line2[0][1] - m2*line2[0][0]


    #ver se são paralelas
    if (m2-m1) == 0:
         return -1 # !Atenção PODE TER QUE SER ALTERADO NO FUTURO!

    elif (line1[1][0] - line1[0][0]) == 0: # Recta: x = a

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
        if y != (m2*x + b2): print('ERROR:', m2*x + b2)

    # intersection is out of bound
    print("x:", line1[0][0],line1[1][0],line2[0][0],line2[1][0])
    print("y:", line1[0][1],line1[1][1],line2[0][1],line2[1][1])
    print(x,y)
    
    if (((x >= max( min(line1[0][0],line1[1][0]), min(line2[0][0],line2[1][0]) )) and
        (x <= min( max(line1[0][0],line1[1][0]), max(line2[0][0],line2[1][0])))) 
        and
        ((y >= max ( min(line1[0][1],line1[1][1]), min(line2[0][1],line2[1][1]))) and
        (y <= min( max(line1[0][1],line1[1][1]), max(line2[0][1],line2[1][1])))) ):

        print("99: We got a point")

        return (x,y)  # intersection is out of bound


    return -1


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
    loc[2] = prev_loc[2] + angle + np.random.normal(loc=0.0, scale=0.01, size=None)
    loc[0] = prev_loc[0] + vel*cos(prev_loc[2]) + np.random.normal(loc=0.0, scale=0.02, size=None)
    loc[1] = prev_loc[1] + vel*sin(prev_loc[2]) + np.random.normal(loc=0.0, scale=0.02, size=None)

    if loc[2] >= (2*pi):
        loc[2] = 0 + 2*pi - loc[2]


    return loc 

# validate_pos():
def validate_pos(loc):

    if loc[0] < lower - 0.1 or loc[0] > upper +0.1 or loc[1] < lower-0.1 or loc[1] > upper+0.1:
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
    reach = 4
    x1 = loc[0][0]
    y1 = loc[1][0]
    teta = loc[2][0]
    radius = -120 #Variação de ângulo do laser
    right_wall = 0

    for i in range(N_measures):

        prev_err = 100000
        ray = np.array([(x1,y1), (x1+reach*cos(teta + radians(radius)), y1+reach*sin(teta + radians(radius))) ]) #creating line segment
        if loc[0] == robot_loc[0] and loc[1] == robot_loc[1] and loc[2] == robot_loc[2]  :
            plt.plot((x1,reach*cos(teta + radians(radius))+x1),(y1,reach*sin(teta + radians(radius))+y1) , c = '#17becf')
        
        print("RAY",i)
    
        #Intersect
        for j in range(14):
            intersect = np.array(line_intersection(map[j], ray))

            if (intersect.shape == (2,) and validate_pos(intersect) == 1):

                print("possiblep_oint")
                avg_pnt = ((map[j][0][0]+map[j][1][0])/2,(map[j][0][1]+map[j][1][1])/2) 
                err = sqrt( pow(avg_pnt[0]-x1, 2) + pow( avg_pnt[1]-y1,2) ) 
                print("avg_point:",avg_pnt)
                print("err",err)
                print("prev_err", prev_err)
                if err < prev_err:
                    prev_err = err
                    right_wall = j
                    print("new_right_wall",right_wall)

        intersect = np.array(line_intersection(map[right_wall], ray))

        if (intersect.shape == (2,) and validate_pos(intersect) == 1):

            x2 = intersect[0] + np.random.normal(loc=0.0, scale=0.07, size=None) #Adding noise two the measures
            y2 = intersect[1] + np.random.normal(loc=0.0, scale=0.07, size=None)

            measures[i] = sqrt( pow(x2-x1, 2) + pow( y2-y1,2) )
            print('wall:', map[right_wall])
            print('measure =',measures[i])

            if loc[0] == robot_loc[0] and loc[1] == robot_loc[1] and loc[2] == robot_loc[2]  :
                plt.scatter(x2, y2, c = '#d62728' )                    

        

        radius += radius_var
        
    #measures = measures[measures != 0]
    #measures = measures.reshape((measures.shape[0],1))
    print(measures)

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

    for i in range(14):
        plt.plot((map[i][0][0],map[i][1][0]),(map[i][0][1],map[i,1,1]), c = 'black')

    plt.scatter(robot_loc[0], robot_loc[1], marker = (6, 0, robot_loc[2]*(180/pi)), c = '#d62728' , s=80, label = "Real position", edgecolors='black')
    plt.plot((robot_loc[0],(1/10)*cos(robot_loc[2])+robot_loc[0]),(robot_loc[1],(1/10)*sin(robot_loc[2])+robot_loc[1]), c = '#17becf')

    plt.xlabel("x")
    plt.ylabel("y")
    plt.show()

    return






"""  ************************************ main()  ******************************************** """


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
    robot_loc = odometry_model(robot_loc, actions[0], radians(actions[1]))
    if validate_pos(robot_loc) == 0:
        break #simulation terminates

    # Retrieving data from the laser
    robot_measures = laser_model(robot_loc)

    """
    # PREDICT
    particles = predict(particles, actions)
    for i in range (M):
        particles[i] = validate_particle(particles[i])
    plot('Map after Predict')
    
    
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
    """
    

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