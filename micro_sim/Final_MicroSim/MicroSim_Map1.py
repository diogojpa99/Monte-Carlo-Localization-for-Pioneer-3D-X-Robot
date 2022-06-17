""" *********************************** Libraries ************************************************* """

import numpy as np
import matplotlib.pyplot as plt
from numpy.random import uniform
from math import cos, sin, sqrt, exp, pi, radians, degrees
from scipy.stats import gaussian_kde
from scipy import integrate

""" ************************************* Global Variables ****************************************  """

# Create Map
map = np.array([[(1,0), (15,0)],
                [(15, 0), (15,4)],
                [(15,4), (6,4)],
                [(6,4), (5,3)],
                [(5,3), (4,4)],
                [(4,4), (3, 4)],
                [(3,4), (3,12)],
                [(3,15), (1.5,14)],
                [(1.5,14),(0,15)],
                [(0,15), (0,0)]] ) 



# Number of walls
n_walls = map.shape[0]

# Rectangle
lower = 0
upper = 15

# Number of particles
M = 100

# Weights
weights = np.empty([M,1])
weights.fill(0.)

# Number of measures of the laser model
N_measures = 30

# Angle of the laser variation
radius_var = 8

# Actions
# action[0] : Distance traveled
# action[1] : Rotation
actions = np.empty([2,1])
actions = np.array([(1,-90),(1,0),(1,0),(1,0),(1,0),(1,0), (1,0), (1,0),(1,0),(1,0), (1,0),(1,0),(1,0),
                    (1,90),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),
                    (1,180),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0)])


# Last Iteration
last_iteration = actions.shape[0]

# n_eff 
n_eff = M

# Resampling Flag
resampling_flag = 1

# Errors
errors = np.empty([last_iteration,3])




"""  ************************************ Functions  ******************************************** """


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


# init_robot_pos():
# Initialize the robot position
# loc[0] : x variable
# loc[1] : y variable
# loc[2] : Orientation

def init_robot_pos():

    loc = np.empty([3,1])
    loc[0] = 0
    loc[1] = 14
    loc[2] = 0

    return loc



# odometry_model():
# Input:
# prev_loc: Localization of the robot at time t
# deltaD : Distance traveled between time t and time t+1
# rotation : Rotation between time t and time t+1
# Output: The localization of the robot at time t+1

def odometry_model(prev_loc, deltaD, rotation, particle_flag):
    loc = np.empty([3,1])

    # Target distribution
    loc[0] = prev_loc[0] + deltaD*cos(prev_loc[2]) 
    loc[1] = prev_loc[1] + deltaD*sin(prev_loc[2]) 
    loc[2] = prev_loc[2] + rotation 
    
    if (particle_flag == 1): #If it is a particle we add uncertanty

        loc[0] += np.random.normal(loc=0.0, scale=0.05, size=None)
        loc[1] += np.random.normal(loc=0.0, scale=0.05, size=None)
        loc[2] += np.random.normal(loc=0.0, scale=0.1, size=None)

    if loc[2] >= (2*pi):
        loc[2] = loc[2] - 2*pi

    return loc 

# Validate Position
# Input: 
# loc = Array that represent the location of the particle/robot
# Output: 
# Function returns '0' if position is invalid, returns '1' if it is valid

def validate_pos(loc):

    if loc[0] < lower - 0.1 or loc[0] > upper + 0.1 or loc[1] < lower - 0.1 or loc[1] > 15 + 0.1:
        return 0
    else:
        return 1



# Validates Localization
# Abstract: Keeps the particles and robot inside the Map
# Output: Localization validated

def validate_loc(loc):

    if loc[0] < lower: loc[0] = lower
    elif loc[0] > upper: loc[0] = upper
    if loc[1] < lower: loc[1] = lower
    elif loc[1] > upper: loc[1] = upper

    if loc[0] > 3 and loc[1] > 4 : 
        avg_pnt = ((3+3)/2,(3+15)/2) 
        err1 = sqrt( pow(avg_pnt[0]-loc[0], 2) + pow( avg_pnt[1]-loc[0],2) ) 
        avg_pnt = ((6+15)/2,(3+3)/2) 
        err2 = sqrt( pow(avg_pnt[0]-loc[0], 2) + pow( avg_pnt[1]-loc[0],2) ) 
        if err1 < err2 :
            loc[0] = 3
        else:
            loc[1] = 4
    

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
    particles[0:int(M/2), 0] = uniform(0, 3, size = int(M/2))
    particles[0:int(M/2), 1] = uniform(0, 15, size = int(M/2))
    particles[int(M/2):M, 0] = uniform(3, 15, size = int(M/2))
    particles[int(M/2):M, 1] = uniform(0, 4, size = int(M/2))
    particles[:, 2] = uniform(0, 2*pi, size = M)
    
    return particles
    


# PREDICT
def predict(particles, actions):

    for i in range(M):
        particles[i]  = odometry_model(particles[i], actions[0], actions[1]*(pi/180),1).reshape((3,))     
        
    return particles 


# Laser Model
def laser_model(loc):

    # The measures are the intersections between the laser rays and the walls
    measures = np.empty([N_measures,1])
    measures.fill(0.)

  
    x1 = loc[0][0]
    y1 = loc[1][0]
    teta = loc[2][0]

    # Laser
    reach = 5.6
    radius = -120 #Variação de ângulo do laser
    
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
            intersect = np.array(line_intersection(map[j], ray))

            # The right wall is the one that is closest to the point
            if (intersect.shape == (2,) and validate_pos(intersect) == 1):

                avg_pnt = ((map[j][0][0]+map[j][1][0])/2,(map[j][0][1]+map[j][1][1])/2) 
                err = sqrt( pow(avg_pnt[0]-x1, 2) + pow( avg_pnt[1]-y1,2) ) 

                if err < prev_err:
                    prev_err = err
                    right_wall = j
                    
        intersect = np.array(line_intersection(map[right_wall], ray))

        if (intersect.shape == (2,) and validate_pos(intersect) == 1):

            #Adding noise two the measures
            x2 = intersect[0] + np.random.normal(loc=0.0, scale=0.05, size=None) 
            y2 = intersect[1] + np.random.normal(loc=0.0, scale=0.05, size=None)

            # The measure is the distance between the position of the robot and the intersection
            # of the ray and the wall
            measures[i] = sqrt( pow(x2-x1, 2) + pow( y2-y1,2) )
            if loc[0] == robot_loc[0] and loc[1] == robot_loc[1] and loc[2] == robot_loc[2]  :
                plt.scatter(x2, y2, c = '#e377c2')         
             

        radius += radius_var
        
    #measures = measures[measures != 0]
    #measures = measures.reshape((measures.shape[0],1))

    return measures

# normal_distribution():
def normal_dist(x , mean , sd):

    prob = exp( - pow( x - mean, 2) / (2*pow(sd,2)) ) / (sd*sqrt(2*pi))  

    return prob

# UPDATE
def update(w, measurments, particles, resampling_flag, likelihood_avg):

    # Distance from each particle to each landmark
    # We have N_measures measures and M particles
    distances = np.empty([N_measures,M])
    distances.fill(0.)

    if resampling_flag == 1:
        w = np.empty([M,1])
        w.fill(1.)
    elif resampling_flag == 0:
        prev_weights = w

    #Standard deviation
    sd = 4
    erros = np.empty([N_measures,1])
    
    # Compute the measures for each particle
    for i in range (M):
        distances[:,i] = laser_model(particles[i].reshape((3,1))).reshape((N_measures,)) 

    #The weights are the product of the likelihoods of the measurments  
    plt.close()
    for i in range (M):

        for j in range (N_measures):
                erros[j] = measurments[j] - distances[j][i]
                w[i] *= normal_dist(measurments[j], distances[j][i], sd)

        w[i] *= pow(10,15) 
        
        if ( resampling_flag == 0):
            w[i] = w[i] * prev_weights[i]
        
        plt.plot(erros)
        plt.show()  
        plt.close()

    # Likelihood average for kidnapping     
    prev_likelihood_avg = likelihood_avg
    likelihood_avg = np.average(w)
    print("Likelihood Avg:", likelihood_avg)
    if(likelihood_avg < pow(10,-18) and prev_likelihood_avg < pow(10,-18)):
        particles[0:int(M*(3/4))-1] = create_particles(int(M*(3/4))-1)
        for i in range (M):
            particles[i] = validate_loc(particles[i])

    #Normalise
    w = w / w.sum() 
    
    #If all the weigths are the same do not resample
    if np.all(w == w[0]):
        resampling_flag = 0

    return w,likelihood_avg

# RESAMPLING
def low_variance_resample(weights):
    N = len(weights)
    print(N)
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


# Plot the simulation
def plot(label):

    # Calculate the point density of the particles
    x = particles[:,0]
    y = particles[:,1]
    xy = np.vstack([x,y])
    z = gaussian_kde(xy)(xy)
    idx = z.argsort() # Sort the points by density, so that the densest points are plotted last
    x, y, z = x[idx], y[idx], z[idx]

    # Plot Map
    for i in range(n_walls):
        plt.plot((map[i][0][0],map[i][1][0]),(map[i][0][1],map[i,1,1]), c = 'black')
    
    # Plot Particles
    plt.scatter(x, y, c = z, s=10, label = "particles")

    # Plot robot
    plt.scatter(robot_loc[0], robot_loc[1], marker = (6, 0, robot_loc[2]*(180/pi)), c = '#d62728' , s=100, label = "Real position", edgecolors='black')
    plt.plot((robot_loc[0],(1/10)*cos(robot_loc[2])+robot_loc[0]),(robot_loc[1],(1/10)*sin(robot_loc[2])+robot_loc[1]), c = '#17becf')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend(loc='upper right')
    plt.title(label)
    plt.pause(0.01)
    plt.show()
    plt.clf()
  

    return


""" *********************************** main() *********************************************** """


# loc: Localization of the robot
robot_loc = init_robot_pos()

# Create particles
particles = create_particles(M)
for i in range (M):
    particles[i] = validate_loc(particles[i])

# Activationg interactive mode
#plt.ion()

# Start simulation
print("Simulation has started!")
k = 0
errors.fill(1.)
resampling_flag = 1
likelihood_avg = 1

while(1):

    # Plotting
    plot('Particle Filter Simulation')
    print("\tIteration nº:",k+1)

    # *********************** Robot simulation ******************************** #

    robot_prev_loc = robot_loc

    # Update localization of the robot
    if (actions[k][0] != 0 or actions[k][1] != 0):
        robot_loc = odometry_model(robot_loc, actions[k][0], radians(actions[k][1]),0)
        robot_loc = validate_loc(robot_loc)

    # Retrieving data from the laser
    robot_measures = laser_model(robot_loc)


    # ************************** Algorithm  ********************************** #

    if (robot_prev_loc[0][0] == robot_loc[0][0] and robot_prev_loc[1][0] == robot_loc[1][0] and robot_prev_loc[2][0] == robot_loc[2][0]):
        print('ROBOT DID NOT MOVE')
    else:

        # PREDICT
        particles = predict(particles, actions[k])
        for i in range (M):
            particles[i] = validate_loc(particles[i])
        
        # UPDATE
        weights, likelihood_avg = update(weights, robot_measures, particles, resampling_flag, likelihood_avg)
        
        # n_eff
        n_eff_inverse = 0

        for m in range (M):
            n_eff_inverse = n_eff_inverse + pow(weights[m],2)
        
        n_eff = 1/n_eff_inverse

        if ( n_eff < M/2 ):
            resampling_flag = 1
        else:
            resampling_flag = 0
        
        # RESAMPLING
        if (resampling_flag == 1):
            indexes = low_variance_resample(weights)
            particles[:] = particles[indexes]
            weights[:] = weights[indexes]
            weights /= np.sum(weights)
        else:
            print('NO RESAMPLE')

        
    # ************************** Output ********************************** #
    
    print('\tOutput')
    print('Real Localization:', robot_loc[0][0],robot_loc[1][0],robot_loc[2][0]*(180/pi))

    # Centroid of the cluster of particles
    print("Predicted Localization:", np.average(particles[:,0]), np.average(particles[:,1]), np.average(particles[:,2])*(180/pi))
   
    errors[k][0] = abs(np.average(particles[:,0])-robot_loc[0][0])
    errors[k][1] = abs(np.average(particles[:,1])-robot_loc[1][0])

    if ( robot_loc[2][0] < 0):
        robot_loc[2][0] = robot_loc[2][0] + 2*pi

    for i in range(M):
        if (particles[i][2] < 0):
             particles[i][2] = particles[i][2] + 2*pi

    errors[k][2] = abs(np.average(particles[:,2]) - robot_loc[2][0])
    print("ERROR:",errors[k][0], errors[k][1], degrees(errors[k][2]))

    if ( ((errors[k][0] < 0.005) and (errors[k][1] < 0.005) and (errors[k][2] < 0.005)) or k == last_iteration-1):
        break

    k +=1



# Plot errors
x_error =  errors[:,0]
x_error = x_error[ x_error !=0]
y_error =  errors[:,1]
y_error = y_error[ y_error !=0]
theta_error =  errors[:,2]
theta_error = theta_error[ theta_error !=0]

plt.ioff()
plt.close('all')
plt.title('Absolute Errors between Real Position and Predict')
plt.plot(x_error, c = '#bcbd22', label = "x" )
plt.plot(y_error, c = '#9467bd', label = "y" )
plt.plot(theta_error, c = '#e377c2', label = "theta" )
plt.xlabel("Number of iterations")
plt.ylabel("errors")
plt.legend(loc='upper right')
plt.show()
plt.close()