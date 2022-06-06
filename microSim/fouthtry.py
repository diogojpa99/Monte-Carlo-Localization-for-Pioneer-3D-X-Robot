
import numpy as np
import matplotlib.pyplot as plt
import scipy
from numpy.random import uniform
from numpy.random import normal
from math import cos, sin, sqrt, exp, pi
from scipy import stats
import cv2 
import math


""" Functions """


# plotting():
def plotting(landmarks, robot_loc, particles):
    plt.scatter(landmarks[0],landmarks[1], c = 'black')
    plt.scatter(robot_loc[0], robot_loc[1], marker = 'x', c = 'red' )
    plt.scatter(particles[:,0], particles[:,1], marker = '2', c = 'green', s = 10 )
    plt.show()

    return


# create_map():
# Creates an rectangular map using matix points.
# Returns the matrix with the map
def get_landmarks(map):

    # Our map is a matrix 184*2
    landmarks= np.zeros([np.count_nonzero(map == 255),2])
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            if map[i,j] == 255:
                landmarks = (i,j)

    # We are going to fiil the matrix
   
    
    return landmarks


# validate_pos():
def validate_pos(loc):

    if( loc[0] < 50): loc[0]= 50
    if( loc[0] > 500): loc[0] = 500
    if( loc[1] < 50): loc[0]= 50
    if( loc[1] > 500): loc[0] = 500

    return loc

# init_robot_pos():
# initialize the robot position
# loc[0] : x variable
# loc[1] : y variable
# loc[2] : angle
def init_robot_pos():
    loc = np.zeros(3, int)
    loc[0] = int(2)
    loc[1] = int(2)
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
    loc = np.zeros(3)

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
'''def laser_model(robot_loc, landmarks):
    
    
    #Not sure se isto está bem feito, mas a ideia é esta
    #Provavelmente não será necessário calcular a medida de todas.
    measures= np.empty([184,1])
    i = 0
    for measure in measures:
        measures[i] = sqrt( pow(landmarks[i][0]-robot_loc[0], 2) + pow(landmarks[i][1]-robot_loc[1],2) )
        i += 1

    return measures'''

def uncertainty_add(distance, angle, sigma): #create noise

    mean = np.array([distance, angle])
    covariance = np.diag(sigma**2)
    distance, angle = np.random.multivariate_normal(mean,covariance) # create noise with a sigma an a mean given 
    distance = max(distance,0) # no negative numbers
    angle = max(angle,0) #no negative numbers
    #return[distance, angle]
    return distance

class LaserSensor:

    def __init__( self, Range, map, robot_loc, uncertainty):

        self.Range = Range
        self.speed = 4 #rounds per second
        self.sigma = np.array([uncertainty[0], uncertainty[1]]) # sensor measurment noise, noise in the angle and range
        self.position = (robot_loc[0],robot_loc[1]) # position of the robot/laser
        print(map.shape[0])
        self.W = map.shape[1]
        self.H = map.shape[0]
        #self.Map = map
        self.sensedObstacles = []

    def distance(self, obstaclePosition):

        px = (obstaclePosition[0]-self.position[0])**2
        py= (obstaclePosition[1]-self.position[1])**2
        return math.sqrt(px+py)

# A straight line of segment with the lenght of the range and along this line we will take
#a number of samples, if the number is equal to 255 it means a wall as been reached
    def laser_model(self, map):

        data = []
        count=0
        x1, y1 = self.position[0], self.position[1]
        #print(x1, '' , y1)
        for angle in np.linspace(0 ,math.pi , 10, False):
            x2, y2 = (x1 + self.Range * math.cos(angle), y1 - self.Range * math.sin(angle)) # get the final point of the range
            #print(x2, '' , y2)
            for i in range (0, 100): # calculate a point in the line segment until it intercepts something
                u = i/100
                x = int(x2* u + x1 *(1-u))#Get a x  between the laser range and the laser position
                y = int(y2* u + y1 *(1-u))#Get a y  between the laser range and the laser position
                #print(x, '' , y)
                
                #print(i)
                #print(map)
                if (x < 0) or (y < 0):
                    break

                if 0 < x < self.W and 0< y < self.H: # to see if the point is out of the map
                    #print(map[x,y])
                    if map[x,y] == 255: #in 255 is where the walls are
                        distance = self.distance((x,y))
                        output = uncertainty_add(distance, angle, self.sigma)
                        #output.append(self.position)
                        count=count+1
                        data.append(output) #store the measurments
                        break
        print(count)
        #print('POOOOORCA')
        #print(data)
        if len(data) > 0 :
            return data
        else:
            return False



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
def update(measurments, particles, map):


    # Distance from each particle to each landmark
    # We have 184 measures and M particles
    distances = np.zeros([184,M])
    #noise = np.empty([184,M])
    #probs = np.empty([184,M])
    weights = np.zeros([M,1])
    #sd = np.empty([M,1])
    #weights.fill(1.)

    
    distances = laser.laser_model(map)
    #print(distances.s)
    #print(measurments.shape)
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

    weights /= sum(weights) #Normalizar

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
img = cv2.imread('/home/goncalo/Desktop/ist/SAut_project/microSim/image.png', cv2.IMREAD_GRAYSCALE)
print(img)

Map = np.array(img)
#print(np.count_nonzero(Map == 255))
# landmarks: Matrix of points that mark the 'walls' of our map 
landmarks = get_landmarks(Map)

# loc: Localization of the robot
robot_loc = init_robot_pos()
#print(robot_loc)
x = robot_loc[0]
y = robot_loc[1]
if Map[x][y]==0:
    print('SIMMMMM')

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
laser = LaserSensor(200, Map, robot_loc, uncertainty=(0.5, 0.01))
particles = create_particles(M)

#Create weights
weights = np.empty([M,1])

while(1):    

    # Plotting
    plotting(Map, robot_loc, particles)
    
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

    # PREDICT
    particles = predict(particles, actions)
    for i in range (M):
        particles[i] = validate_pos(particles[i])


    # Retrieving data from the laser
    measures = laser.laser_model(Map)

    # UPDATE
    weights = update(measures,particles, Map)
    #print(weights)
    
    # UPDATED SET

    # RESAMPLING
    indexes = systematic_resample(weights)
    particles[:] = particles[indexes]
    weights[:] = weights[indexes]
    weights /= np.sum(weights)
