""" *********************************** Libraries ************************************************* """

import numpy as np
import matplotlib.pyplot as plt
from numpy.random import uniform
from math import cos, sin, sqrt, exp, pi, radians, degrees
from scipy.stats import gaussian_kde


""" ************************************* Global Variables ****************************************  """
''' Particles '''

# Number of particles
original_M = M = 1500

# Flag that defines the number of particles
# resize_flag = 0 : Don't do nothing
# resize_flag = 1 : Increase number of particles
# resize_flag = 2 : Decrease number of particles
resize_flag = 0

# Particles
particles = np.empty([M, 3])

''' Weights '''

w = np.empty([M,1])
w.fill(1.)

''' Robot '''

# Robot Localization
robot_loc = np.empty([3,1])

''' Simulation '''

# Actions
# action[0] : Distance traveled
# action[1] : Rotation
actions = np.empty([2,1])
actions = np.array([(1,-90),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),
                    (1,90),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),                   
                    (0,0),(1,-90),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),
                    (1,90),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),
                    (1,90),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),])

# Last Iteration
last_iteration = actions.shape[0]

''' Optimize the algorithm '''

likelihood_avg = 1 #Likelihoods Average
n_eff = M # n_eff 
resampling_flag = 1 # Resampling Flag


''' Output '''

# Errors
errors = np.empty([last_iteration,3])
errors.fill(1.)

''' Map '''

# Create Map
map = np.array([[(3,10), (4,10)],
                [(6,10), (9,10)],
                [(10,9), (10,0)],
                [(0,0), (4,0)],
                [(6,0), (10,0)],
                [(4,10), (5,9)],
                [(6,10), (5,9)],
                [(3,4), (3,15)],
                [(3,15), (0,15)], 
                [(0,15), (0,0)]]) 

# Number of walls
n_walls = map.shape[0]

# Rectangle
lower = 0
upper = 10


''' Weights '''

# Our Likelihood is the sum between
# an normal distrbution and an uniform distribution

w2 = 0.4
w1 = 1 - w2

''' Robot '''

# Odometry uncertainty
# odom_uncertainty[0]: x
# odom_uncertainty[1]: y
# odom_uncertainty[2]: Rotation
odom_uncertainty = (0.15,0.15,0.15)

''' Laser '''

N_measures = 30 # Number of measures of the laser model
laser_reach = 5.6
laser_radius_var = 8 # Angle of the laser variation
laser_uncertanty = 0.05

''' Optimize the algorithm '''

likelihood_sd = 1
likelihood_avg_thresh = pow(10,-4)


"""  ************************************ Functions  ******************************************** """

# init_robot_pos():
# Initialize the robot position
# loc[0] : x variable
# loc[1] : y variable
# loc[2] : Orientation

def init_robot_pos(loc):

    loc[0] = 0
    loc[1] = 14
    loc[2] = 0

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
    particles[0:int(M/6), 0] = uniform(0, 3, size = int(M/6))
    particles[0:int(M/6), 1] = uniform(10, 15, size = int(M/6))
    particles[int(M/6):M, 0] = uniform(0, 10, size = int((5*M)/6))
    particles[int(M/6):M, 1] = uniform(0, 10, size = int((5*M)/6))
    particles[:, 2] = uniform(0, 2*pi, size = M)
    
    return particles

# Sample Particles because kidnapping

def reposition_particle(particle, reposition_flag):

    if reposition_flag % 2 == 0:
        particle[0] = uniform(0, 3, size = 1)
        particle[1] = uniform(0, 15, size = 1)
    else:
        particle[0] = uniform(0, 10, size = 1)
        particle[1] = uniform(0, 10, size = 1)

    particle[2] = uniform(0, 2*pi, size = 1)
    
    return particle


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

    if (loc[0] < 3) and (loc[1] > 15): loc[1] = 15
    elif loc[1] > upper and loc[0] > 3: loc[1] = upper
    
    return loc
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

def odometry_model(loc, deltaD, rotation, particle_flag):
        
    # Target distribution
    loc[0] = loc[0] + deltaD*cos(loc[2]) 
    loc[1] = loc[1] + deltaD*sin(loc[2]) 
    loc[2] = loc[2] + rotation 
    
    if (particle_flag == 1): #If it is a particle we add uncertanty

        loc[0] += np.random.normal(loc=0.0, scale= odom_uncertainty[0], size=None)
        loc[1] += np.random.normal(loc=0.0, scale= odom_uncertainty[1], size=None)
        loc[2] += np.random.normal(loc=0.0, scale= odom_uncertainty[2], size=None)
    
    return loc 


# PREDICT
def predict(particles, actions, M):

    particles = particles.reshape([M,3])

    for i in range(M):
        particles[i]  = odometry_model(particles[i], actions[0], actions[1]*(pi/180), 1).reshape((3,))     
        
    return particles 


# Laser Model
def laser_model(loc, n_walls, robot_loc, selected_map):

    # The measures are the intersections between the laser rays and the walls
    measures = np.empty([N_measures,1])
    measures.fill(0.)

    x1 = loc[0][0]
    y1 = loc[1][0]
    teta = loc[2][0]

    # Laser radius
    laser_radius = -120 #Variação de ângulo do laser
    
    # Variable that determines which wall the laser intersected
    # An laser ray only can intersect one wall
    right_wall = 0

    for i in range(N_measures):

        prev_err = 100000
        
        #Creating line segment
        ray = np.array([(x1,y1), (x1+laser_reach*cos(teta + radians(laser_radius)), y1+laser_reach*sin(teta + radians(laser_radius))) ]) 
        '''
        if loc[0] == robot_loc[0] and loc[1] == robot_loc[1] and loc[2] == robot_loc[2]  :
           plt.plot((x1,laser_reach*cos(teta + radians(laser_radius))+x1),(y1,laser_reach*sin(teta + radians(laser_radius))+y1) , c = '#17becf')
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
            x2 = intersect[0] + np.random.normal(loc=0.0, scale= laser_uncertanty, size=None) 
            y2 = intersect[1] + np.random.normal(loc=0.0, scale= laser_uncertanty, size=None)

            # The measure is the distance between the position of the robot and the intersection
            # of the ray and the wall
            measures[i] = sqrt( pow(x2-x1, 2) + pow( y2-y1,2) )
            if loc[0] == robot_loc[0] and loc[1] == robot_loc[1] and loc[2] == robot_loc[2]  :
                axis[0].scatter(x2, y2, c = '#e377c2')        
             

        laser_radius += laser_radius_var
        
    return measures

# normal_distribution():
def normal_dist(x , mean , sd):

    prob = exp( - pow( x - mean, 2) / (2*pow(sd,2)) ) / (sd*sqrt(2*pi))  

    return prob

# UPDATE
def update(w, robot_measurments, particles, resampling_flag, likelihood_avg, M, robot_loc, selected_map):

    # Distance from each particle to each landmark
    # We have N_measures measures and M particles
    distances = np.empty([N_measures,M])
    distances.fill(0.)

    resize_flag = 0 # Flag that determines if we are going to change the numbe rof particles

    if resampling_flag == 1:
        w = np.empty([M,1])
        w.fill(1.)
    elif resampling_flag == 0:
        w = w[0:M]
        prev_weights = w

    # Compute the measures for each particle
    for i in range (M):
        distances[:,i] = laser_model(particles[i].reshape((3,1)), n_walls, robot_loc, selected_map).reshape((N_measures,))

    #The weights are the product of the likelihoods of the measurments  
    for i in range (M):

        for j in range (N_measures):
            w[i] *= ( w1*normal_dist(robot_measurments[j], distances[j][i], likelihood_sd) + w2*(1/laser_reach))

        w[i] *= pow(10,17) 

        if ( resampling_flag == 0):
            w[i] = w[i] * prev_weights[i]
        
    # Likelihood average for kidnapping     
    prev_likelihood_avg = likelihood_avg
    print("Likelihood_avg: ",likelihood_avg)
    likelihood_avg = np.average(w)
    
    # We assume that there was a kidnapping 
    if(likelihood_avg <  likelihood_avg_thresh  and prev_likelihood_avg < likelihood_avg_thresh ):
        resize_flag = 1 #We increase the number


    # Plot Weights
    for i in range(M):
        axis[1].plot( (i,i), (0,w[i]), c = '#d62728' )

    #Normalise
    w /= sum(w) 
    
    #If all the weigths are the same do not resample
    if np.all(w == w[0]):
        resampling_flag = 0

    return w,likelihood_avg, resize_flag

# RESAMPLING
def low_variance_resample(weights, M, particles):

    positions = (np.arange(M) + np.random.random()) / M
    indexes = np.zeros(M, 'i')
    cumulative_sum = np.cumsum(weights)
    i, j = 0, 0

    while i < M and j<M:
        if positions[i] < cumulative_sum[j]:
            indexes[i] = j
            i += 1
        else:
            j += 1

    particles[:] = particles[indexes]

    return particles

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
        axis[0].plot((map[i][0][0],map[i][1][0]),(map[i][0][1],map[i,1,1]), c = 'black')
    
    # Plot Particles
    axis[0].scatter(x, y, c = z, s=5, label = "particles")

    # Plot robot
    axis[0].scatter(robot_loc[0], robot_loc[1], marker = (6, 0, robot_loc[2]*(180/pi)), c = '#d62728' , s=180, label = "Real position", edgecolors='black')
    axis[0].plot((robot_loc[0],(1/5)*cos(robot_loc[2])+robot_loc[0]),(robot_loc[1],(1/5)*sin(robot_loc[2])+robot_loc[1]), c = '#17becf')
    axis[0].set_xlabel('x')
    axis[0].set_ylabel('y')
    axis[0].set_title(label)
    axis[1].set_xlabel('Particles')
    axis[1].set_title('Weigts')
    plt.pause(0.01)
    plt.show()
  

    return



""" *********************************** main() *********************************************** """

# loc: Localization of the robot
robot_loc = init_robot_pos(robot_loc)

# Create particles
particles = create_particles(M)
for i in range (M):
    particles[i] = validate_loc(particles[i])

# Plotting
# Activationg interactive mode
fig, axis = plt.subplots(1,2,figsize=(15,5))
plt.ion()
plot('Particle Filter Simulation')

print('Press "1" if you want to start the simulation:')
map_flag = float(input())
if ( map_flag != 1):
    exit(0)


# Start simulation
print('The simulation as started')
k = 0


while(1):

    axis[0].axes.clear()
    axis[1].axes.clear()

    # *********************** Robot simulation ******************************** #
    
    print("-----------------------------------------------------------------------------------")
    print("\t\t\tIteration nº:",k+1)

    # Update localization of the robot
    robot_loc = odometry_model(robot_loc, actions[k][0], radians(actions[k][1]), 0)
    robot_loc = validate_loc(robot_loc)

    # Kidnapping
    if( k == 20 ):
        robot_loc[0] = 1
        robot_loc[1] = 14
        robot_loc[2] = 0

    # Retrieving data from the laser
    robot_measures = laser_model(robot_loc, n_walls, robot_loc, map)

    # ************************** Algorithm  ********************************** #

    print("Nº of particles:", M)

    if (actions[k][0] == 0 and actions[k][1] == 0):
        print('ROBOT DID NOT MOVE')
    else:

        # PREDICT
        particles = predict(particles, actions[k], M)
        for i in range (M):
            particles[i] = validate_loc(particles[i])
        
        # UPDATE
        w, likelihood_avg, resize_flag = update(w, robot_measures, particles, resampling_flag, likelihood_avg, M, robot_loc, map)
        
        # n_eff
        n_eff_inverse = 0

        for m in range (M):
            n_eff_inverse = n_eff_inverse + pow(w[m],2)
        
        n_eff = 1/n_eff_inverse
        print("[Neff] -> ", n_eff)
        if ( n_eff < M*0.3 ):
            resampling_flag = 1
        else:
            resampling_flag = 0
            resize_flag = 2
        
        # RESAMPLING
        if (resampling_flag == 1):
            particles = low_variance_resample(w, M , particles)
        else:
            print('NO RESAMPLE')
    
    
    # Resizing the number of particles
    if resize_flag == 1:

        # High probability of kidnapping
        more_particles = create_particles( int(M*1.2))
        more_particles[0:M] = particles
        particles = more_particles
        M = int(M*1.2)
        
        for i in range (int(M*0.9)):
            particles[i] = reposition_particle(particles[i], i )
            particles[i] = validate_loc(particles[i])

    
    elif resize_flag == 2:

        if ( M > int(original_M*0.3)):
            M = int(M*0.8)
            particles = particles[0:M]

        
    # ************************** Output ********************************** #

    # Centroid of the cluster of particles
    # Centroid of the cluster of particles
    pred_angle = np.average(particles[:,2])
    robot_angle = robot_loc[2][0]

    if pred_angle > pi:
        pred_angle -= 2*pi
    elif pred_angle < -pi:
        pred_angle += 2*pi

    if robot_angle > pi:
        robot_angle = robot_angle - 2*pi
    elif robot_angle < -pi:
        robot_angle += 2*pi

    errors[k][0] = abs(np.average(particles[:,0])-robot_loc[0][0])
    errors[k][1] = abs(np.average(particles[:,1])-robot_loc[1][0])   
    errors[k][2] = abs(pred_angle - robot_angle) 

    if ( errors[k][2] > (5/3)*pi):
         errors[k][2] = abs(2*pi - errors[k][2])
        
    print('Real Loc:',"\t", robot_loc[0][0],"\t", robot_loc[1][0],"\t", robot_angle*(180/pi))
    print("Pred Loc:", "\t", np.average(particles[:,0]),"\t", np.average(particles[:,1]),"\t", pred_angle*(180/pi))
    print("ERROR:  ","\t",errors[k][0],"\t", errors[k][1],"\t", degrees(errors[k][2]))

    # Plotting
    plot('Particle Filter Simulation')

    if ( ((errors[k][0] < 0.005) and (errors[k][1] < 0.005) and (errors[k][2] < 0.005)) or k == last_iteration-1):
        break

    resize_flag = 0  
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