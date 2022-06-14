""" Libraries """

import numpy as np
import matplotlib.pyplot as plt
import scipy
from numpy.random import uniform
from numpy.random import normal
from math import cos, sin, sqrt, exp, pi
from scipy import stats


""" Functions """


# plotting():
def plotting(landmarks, robot_loc, particles):
    plt.scatter(landmarks[:,0],landmarks[:,1], c = 'black')
    plt.scatter(robot_loc[0], robot_loc[1], marker = 'x', c = 'red' )
    plt.scatter(particles[:,0], particles[:,1], marker = '2', c = 'green', s = 10 )
    plt.show()

    return


# create_map():
# Creates an rectangular map using matix points.
# Returns the matrix with the map
def create_map():

    # Our map is a matrix 184*2
    landmarks= np.empty([184,2])
    i = 0

    # We are going to fiil the matrix
    for landmark in landmarks:

        if  i < 46 :

            if i == 0: x = 50 
            landmarks[i][0] = x
            landmarks[i][1] = 50
            x += 10

        elif i < 92 :

            if i == 46: x = 50 
            landmarks[i][0] = x
            landmarks[i][1] = 500
            x += 10

        elif i < 138:

            if i == 92: y = 50 
            landmarks[i][0] = 50
            landmarks[i][1] = y
            y += 10

        elif i < 184:

            if i == 138: y = 50 
            landmarks[i][0] = 500
            landmarks[i][1] = y
            y += 10
        
        i += 1
    
    return landmarks


# validate_pos():
def validate_pos(loc):

    if( loc[0] < 50): loc[0]= 50
    if( loc[0] > 500): loc[0] = 500
    if( loc[1] < 50): loc[1]= 50
    if( loc[1] > 500): loc[1] = 500

    return loc

# init_robot_pos():
# initialize the robot position
# loc[0] : x variable
# loc[1] : y variable
# loc[2] : angle
def init_robot_pos():
    loc = np.empty([3,1])
    loc[0] = 250
    loc[1] = 250
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
    # !Atention! Maybe not exactly like this, but it is something like this
    loc[0] = prev_loc[0] + vel*cos(angle)
    loc[1] = prev_loc[1] + vel*sin(angle)
    loc[2] = prev_loc[2] + angle 


    return loc



# laser_model():
# Basically it will be the distance from the robot's position to the walls
# !Atenção! Temos que incluir aqui o MATCHING
# input:
# loc: Current position of the robot
# landmarks: Position of the landmarks
# Output: Euclidean distance from the robot to each of the partiles 

# TEMOS QUE VER ESTA PARTE MELHOR
def laser_model(robot_loc, landmarks):
    
    
    #Not sure se isto está bem feito, mas a ideia é esta
    #Provavelmente não será necessário calcular a medida de todas.
    measures= np.empty([184,1])
    i = 0
    for measure in measures:
        measures[i] = sqrt( pow(landmarks[i][0]-robot_loc[0], 2) + pow(landmarks[i][1]-robot_loc[1],2) )
        i += 1

    return measures


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
    particles[:, 0] = uniform(50, 500, size = M)
    particles[:, 1] = uniform(50, 500, size = M)
    particles[:, 2] = uniform(0, 360, size = M)
    
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
        model  = odometry_model(particles[i], actions[0], actions[1]).reshape((3,))   
        particles[i] = model + normal(loc=0.0, scale=1.0, size=3)
        

    return particles 


# Funções que faltam:
# Particle_filter(): Incluir tudo -> Tem que ser recursiva
# Particle_filter_init(): Iniciar o algoritmo recursivo
# Init_parameters(): Colocar tudo a zero -> Particulas, pesos
# Ruídos -> Tanto no modelo do robot
# Ruído -> Modelo do laser -> Vai definir os pesos
# Resampling
# Depois temos que ajustar: Nº de partículas e o resampling
# Ver consistência do algoritmo, robustez, etc.


# normal_distribution():
def normal_dist(x , mean , sd):
    prob = (1/(sd*sqrt(2*pi)))*exp(-0.5*((x - mean)/sd)**2) 
    return prob


# update():
def update(measurments, particles, landmarks):


    # Distance from each particle to each landmark
    # We have 184 measures and M particles
    distances = np.empty([184,M])
    #noise = np.empty([184,M])
    #probs = np.empty([184,M])
    weights = np.empty([M,1])
    #sd = np.empty([M,1])
    weights.fill(1.)

    for i in range (M):
        distances[:,i] = laser_model(particles[i], landmarks).reshape((184,))
        #noise[:,i] = abs(measurments.reshape((184,)) - distances[:,i])


    """
    for i in range(M):
        for j in range(184):
            sd[i] = sqrt(pow(measurments[i]-distances[j][i],2)/184)
    """

    #The weights are the product of the likelihoods of the measurments  
    for i in range (M):
        for j in range (len(measurments)):
            #probs[j][i] = normal_dist(measurments[j], distances[j][i], sqrt(pow(measurments[i]-distances[j][i],2)))
            #weights[i] *= normal_dist(measurments[j], distances[j][i], sqrt(pow(measurments[i]-distances[j][i],2)))
            var = sqrt(pow(measurments[j]-distances[j][i],2))
            weights[i] += exp(-0.5* pow(1/50,2) * pow( measurments[j] - distances[j][i], 2)) 

    weights /= np.sum(weights) #Normalizar

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


""" Global Variables """

#Number of particles
M = 300

#Actions
# action[0] : cmd_vel
# action[1] : twist
actions = np.empty([2,1])


""" main() """

# landmarks: Matrix of points that mark the 'walls' of our map 
landmarks = create_map()

# loc: Localization of the robot
robot_loc = init_robot_pos()

#Activationg interactive mode
plt.ion()


""" Particle_filter_init(): """

# Particle_filter_init():
# This function initiates the Particle Filter Algorithm
# Input:
# robot_model: Localization of the robot plus some noise
# particles: Set of particles 
# weights: Set of weights
# laser_model: measures of the laser plus some noise
# Ouput:
# Particle_filter()


#Create particles
particles = create_particles(M)

#Create weights
weights = np.empty([M,1])

while(1):    

    # Plotting
    plotting(landmarks, robot_loc, particles)

    #Getting info from the terminal
    print('Please input the robot velocity:')
    actions[0] = float(input())
    print('Please input the robot rotation')
    actions[1] = float(input())

    # Erase previous robot position
    plt.scatter(robot_loc[0], robot_loc[1], c = 'white', s = 60 )
    plt.scatter(particles[:,0], particles[:,1], c = 'white', s = 60 )

    # Update localization of the robot
    robot_loc = odometry_model(robot_loc, actions[0], actions[1])
    robot_loc = validate_pos(robot_loc)
    print(robot_loc)

    # PREDICT
    particles = predict(particles, actions)
    for i in range (M):
        particles[i] = validate_pos(particles[i])


    # Retrieving data from the laser
    measures = laser_model(robot_loc, landmarks)

    # UPDATE
    weights = update(measures,particles,landmarks)
    print(weights)
    
    # UPDATED SET

    # RESAMPLING
    indexes = systematic_resample(weights)
    particles[:] = particles[indexes]
    weights[:] = weights[indexes]
    weights /= np.sum(weights)







