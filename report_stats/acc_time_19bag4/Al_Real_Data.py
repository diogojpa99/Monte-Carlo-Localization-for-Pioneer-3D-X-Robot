""" *********************************** Libraries ************************************************* """

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from math import pi, radians, degrees, sin, cos
from scipy.stats import gaussian_kde
import time


import plots as pl
import Real_Map as map
import Paricle_Filter as pf
import Get_Data as data

""" ************************************* Global Variables ****************************************  """

''' Particles '''

# Number of particles
original_M = M = 300

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

''' Time '''
time_actual = []
time_diff = []
tTotal = 0

''' Robot '''

# Robot Localization
robot_loc = np.empty([3,1])

''' Actions '''

# Actions
# action[0] : Distance traveled
# action[1] : Rotation
actions = np.empty([2,1])
actions[0] = actions[1] = 0

# Last Iteration
last_iteration = 200

''' Optimize the algorithm '''

likelihood_avg = 1 #Likelihoods Average
n_eff = M # n_eff 
resampling_flag = 1 # Resampling Flag

''' Output '''

# Errors
errors = np.empty([last_iteration,3])
errors.fill(np.nan)

errors_x_vector = []

errors_y_vector = []

errors_theta_vector = []

#Prediction
predict_loc = np.empty([3,1])
non_normalize_w = np.empty([M,1])
non_normalize_w.fill(1.)

""" *********************************** main() *********************************************** """

# Create particles
particles = map.create_particles(M)
for i in range (M):
    particles[i] = map.validate_loc(particles[i])


# Plotting
# Activationg interactive mode
plt.ion()
pl.plot('Algorithm in Offline Mode', map.n_walls, map.map, particles, robot_loc, predict_loc)

k = 0

## time init 

while(1):

    print("-----------------------------------------------------------------------------------")
    t0 = time.perf_counter()
    print("\t\t\tIteration nº:",k+1)

    # *********************** Get Real Data ******************************** #

    amcl_x, amcl_y, amcl_theta, actions[0], actions[1], measures = data.get_data()

    # ************************** Algorithm  ********************************** #
    
    print("Nº of particles:", M)
    if (actions[0] == 0 and actions[1] == 0):
        print('ROBOT DID NOT MOVE')
    else:

        # PREDICT
        particles = pf.predict(particles, actions, M)
        for i in range (M):
            particles[i] = map.validate_loc(particles[i])
        
        # UPDATE
        w, likelihood_avg, resize_flag, non_normalize_w = pf.update(w, robot_measures, particles, resampling_flag, likelihood_avg, M, robot_loc, map)
        
        # n_eff
        n_eff_inverse = 0

        for m in range (M):
            n_eff_inverse = n_eff_inverse + pow(w[m],2)
        
        n_eff = 1/n_eff_inverse
        print("[Neff] -> ", n_eff)
        if ( n_eff < M/2):
            resampling_flag = 1
        else:
            resampling_flag = 0
            resize_flag = 2
    
        # RESAMPLING
        if (resampling_flag == 1):
            indexes = pf.low_variance_resample(w, M)
            particles[:] = particles[indexes]
            w[:] = w[indexes]
        else:
            print('NO RESAMPLE')
        
    
        # Resizing the number of particles
        if resize_flag == 1:

            # High probability of kidnapping
            more_particles = np.empty([int(M*0.2),3])
            
            for i in range(int(M*0.2)):
                more_particles[i] =  map.reposition_particle(more_particles[i], i )
                more_particles[i] =  map.validate_loc(more_particles[i])
                particles = np.append(particles, [more_particles[i]], axis = 0)
            
            M = int(M*1.2)

            # Kidnapping
            for i in range (int(M*0.8)):
                particles[i] = map.reposition_particle(particles[i], i )
                particles[i] = map.validate_loc(particles[i])
        
        elif resize_flag == 2:

            if ( M > int(original_M*0.3) and n_eff > M*0.75):
                M = int(M*0.9)
                particles = particles[0:M]

    
    '''
    # *************************** Prediction *********************************** #
    
    imp_part = np.empty([M,3])
    n = 0
    for m in range(M):
        if non_normalize_w[m] > likelihood_avg:
            imp_part[m][0] = particles[m][0]
            imp_part[m][2] = particles[m][1]
            imp_part[m][1] = particles[m][2]
        n += 1
    
    predic_particles = imp_part[0:n]
    '''

    # *************************** Errors *********************************** #
        
    # Centroid of the cluster of particles
    predict_loc[0] = np.average(particles[:,0])
    predict_loc[1] = np.average(particles[:,1])
    pred_angle = predict_loc[2] = np.average(particles[:,2])
    robot_angle = amcl_theta

    if pred_angle > pi:
        pred_angle -= 2*pi
    elif pred_angle < -pi:
        pred_angle += 2*pi

    if robot_angle > pi:
        robot_angle = robot_angle - 2*pi
    elif robot_angle < -pi:
        robot_angle += 2*pi
    
    errors_x = abs(predict_loc[0]-amcl_x)
    errors_y = abs(predict_loc[1]-amcl_y)   
    errors_theta = abs(pred_angle - robot_angle) 

    if (  errors_theta > (5/3)*pi):
        errors_theta = abs(2*pi - errors_theta)

    print('amcl Loc:',"\t", amcl_x,"\t", amcl_y,"\t", amcl_theta*(180/pi))
    print("Pred Loc:", "\t", predict_loc[0],"\t", predict_loc[1],"\t", predict_loc[2])
    print("ERROR:  ","\t",  errors_x,"\t",   errors_y,"\t", degrees( errors_theta))

    errors_x_vector.append(errors_x)
    errors_y_vector.append(errors_y)
    errors_theta_vector.append(errors_theta)

    # Get Real Measurments from hokuyo
    measures = np.array(measures)
    if len(measures) != 0:
        robot_measures = measures.reshape([measures.shape[1],1])
        robot_measures[np.isnan(robot_measures)] = 0
    
    robot_loc[0][0] = amcl_x
    robot_loc[1][0] = amcl_y
    robot_loc[2][0] = amcl_theta

    # Update localization of the robot
    # robot_loc = map.validate_loc(robot_loc)

    
    # ************************** Ploting  ********************************** #
    
    radius = -119
    if len(measures) != 0:
        for i in range (robot_measures.shape[0]):
            plt.scatter(predict_loc[0] + robot_measures[i]*cos(predict_loc[2] + radians(radius)), robot_measures[i]*sin(predict_loc[2] + radians(radius)) + predict_loc[1], s = 7,  c = 'green')   
            radius += 10
    pl.plot('Algorithm in Offline Mode', map.n_walls, map.map, particles,robot_loc , predict_loc)
 
    k += 1

    t1 = time.perf_counter()
    tdiff = t1 - t0
    print("Algorithm time elapse:", tdiff)
    tTotal += tdiff
    time_actual.append(tTotal)
    time_diff.append(tdiff)

    if k == last_iteration:
        break

# Plotting Statistics
pl.plot_erros(errors)

col0 = "erro_X"
col1 = "erro_Y"
col2 = "erro_W"
col3 = "time"
col4 = "diff"

metrics = pd.DataFrame({col0: errors_x_vector,
                        col1: errors_y_vector,
                        col2: errors_theta_vector,
                        col3: time_actual,
                        col4: time_diff}) 
metrics.to_excel('1.xlsx', sheet_name="sheet1", index=False)