""" *********************************** Libraries ************************************************* """

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
original_M = M = 800

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

''' Actions '''

# Actions
# action[0] : Distance traveled
# action[1] : Rotation
actions = np.empty([2,1])
actions[0] = actions[1] = 0

# Last Iteration
last_iteration = 300

''' Optimize the algorithm '''

likelihood_avg = 1 #Likelihoods Average
n_eff = M # n_eff 
resampling_flag = 1 # Resampling Flag

''' Output '''

# Errors
errors = np.empty([last_iteration,3])
errors.fill(np.nan)

#Prediction
predict_loc = np.empty([3,1])

""" *********************************** Functions ************************************************* """

# Plot the simulation
def plot(label, n_walls, map):

    # Calculate the point density of the particles
    x = particles[:,0]
    y = particles[:,1]
    xy = np.vstack([x,y])
    #z = gaussian_kde(xy)(xy)
    #idx = z.argsort() # Sort the points by density, so that the densest points are plotted last
    #x, y, z = x[idx], y[idx], z[idx]

    # Plot Map
    for i in range(n_walls):
        axis[0].plot((map[i][0][0],map[i][1][0]),(map[i][0][1],map[i,1,1]), c = 'black')
    
    # Plot Particles
    axis[0].scatter(x, y, c = '#e377c2', s=1, label = "Particles")

    # Plot robot
    axis[0].scatter(robot_loc[0], robot_loc[1], marker = (6, 0, robot_loc[2]*(180/pi)), c = '#d62728' , s=60, label = "AMCL Reference", edgecolors='black')
    axis[0].plot((robot_loc[0],(1/5)*cos(robot_loc[2])+robot_loc[0]),(robot_loc[1],(1/5)*sin(robot_loc[2])+robot_loc[1]), c = '#17becf')
    #axis[0].scatter(predict_loc[0], predict_loc[1],c = '#17becf' , s=50, label = "Prediction", edgecolors='black')
    axis[0].plot((predict_loc[0],(1/4)*cos(predict_loc[2])+predict_loc[0]),(predict_loc[1],(1/4)*sin(predict_loc[2])+predict_loc[1]), c = '#d62728')  
    axis[0].set_xlabel('x [m]')
    axis[0].set_ylabel('y [m]')
    axis[0].set_title(label)
    axis[0].legend(loc='upper right')
    plt.pause(0.01)
    plt.show()
  

    return


# Plot Errors
def plot_erros(errors):

    x_error =  errors[:,0]
    x_error = x_error[ x_error !=0]
    y_error =  errors[:,1]
    y_error = y_error[ y_error !=0]
    theta_error =  errors[:,2]
    theta_error = theta_error[ theta_error !=0]


    axis[1].plot(x_error, c = '#bcbd22', label = "x error [m]" )
    axis[1].plot(y_error, c = '#9467bd', label = "y error [m]" )
    axis[1].plot(theta_error, c = '#e377c2', label = "Orientation error [rad]")
    axis[1].set_xlabel('Iterations')
    axis[1].set_ylabel('Error')
    axis[1].legend(loc='upper right')
    axis[1].set_title('Absolute Errors between AMCL Reference and Algortihm Prediction')

    return

# Define the Prediction of Our algorithm
def prediction(w, particles):

    g_par = particles

    for j in range(5):

        most_important_particles = np.argmax(w)
        print('Most Important Particle:',most_important_particles)
        aux_particles = np.empty([M,3])
        n_particles = 0

        for i in range (M):

            if (g_par[i][0] == g_par[most_important_particles][0]) and (g_par[i][1] == g_par[most_important_particles][1]) and (g_par[i][2] == g_par[most_important_particles][2]):
                aux_particles[i][0] =  g_par[i][0]
                aux_particles[i][1] =  g_par[i][1]
                aux_particles[i][2] =  g_par[i][2]
                n_particles += 1

        w = np.delete(w,most_important_particles)
        


    pred_particles = aux_particles[0:n_particles]
    print(pred_particles.shape)
        
    return pred_particles

""" *********************************** main() *********************************************** """

# Create particles
particles = map.create_particles(M)
for i in range (M):
    particles[i] = map.validate_loc(particles[i])

# Plotting
# Activationg interactive mode
fig, axis = plt.subplots(1,2,figsize=(15,5))
plt.ion()
plot('Algorithm On Real Data', map.n_walls, map.map)

k = 0

while(1):

    print("-----------------------------------------------------------------------------------")
    t0= time.perf_counter()
    print("\t\t\tIteration nº:",k+1)

    axis[0].axes.clear()
    axis[1].axes.clear()

    # *********************** Get Real Data ******************************** #

    amcl_x, amcl_y, amcl_theta, actions[0], actions[1], measures = data.get_data()

    # Get Real Measurments from hokuyo
    measures = np.array(measures)
    if len(measures) != 0:
        robot_measures = measures.reshape([measures.shape[1],1])
        robot_measures[np.isnan(robot_measures)] = 0
    
    robot_loc[0][0] = amcl_x
    robot_loc[1][0] = amcl_y
    robot_loc[2][0] = amcl_theta

    # Update localization of the robot
    #robot_loc = map.validate_loc(robot_loc)

    # ************************** Algorithm  ********************************** #

    important_particles = particles
    
    print("Nº of particles:", M)
    if (actions[0] == 0 and actions[1] == 0):
        print('ROBOT DID NOT MOVE')
    else:

        # PREDICT
        particles = pf.predict(particles, actions, M)
        for i in range (M):
            particles[i] = map.validate_loc(particles[i])
        
        # UPDATE
        w, likelihood_avg, resize_flag = pf.update(w, robot_measures, particles, resampling_flag, likelihood_avg, M, robot_loc, map)
        
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
            particles[:] = particles[indexes] #Resample particles by index
        else:
            print('NO RESAMPLE')
        

        # Prediction
        important_particles = prediction(indexes, particles)

        
        # Resizing the number of particles
        if resize_flag == 1:

            # High probability of kidnapping
            more_particles = map.create_particles( int(M*1.2))
            more_particles[0:M] = particles
            particles = more_particles
            M = int(M*1.2)
            
            for i in range (int(M*0.9)):
                particles[i] = map.reposition_particle(particles[i], i )
                particles[i] = map.validate_loc(particles[i])
    
        elif resize_flag == 2:

            if ( M > int(original_M*0.3)):
                M = int(M*0.8)
                particles = particles[0:M]


    # *************************** Errors *********************************** #

    # Centroid of the cluster of particles
    predict_loc[0] = np.average( important_particles[:,0])
    predict_loc[1] = np.average( important_particles[:,1])
    pred_angle = predict_loc[2] = np.average( important_particles[:,2])
    robot_angle = amcl_theta

    if pred_angle > pi:
        pred_angle -= 2*pi
    elif pred_angle < -pi:
        pred_angle += 2*pi

    if robot_angle > pi:
        robot_angle = robot_angle - 2*pi
    elif robot_angle < -pi:
        robot_angle += 2*pi

    errors[k][0] = abs(predict_loc[0]-amcl_x)
    errors[k][1] = abs(predict_loc[1]-amcl_y)   
    errors[k][2] = abs(pred_angle - robot_angle) 

    if (errors[k][2] > (5/3)*pi):
        errors[k][2] = abs(2*pi - errors[k][2])

    print('amcl Loc:',"\t", amcl_x,"\t", amcl_y,"\t", amcl_theta*(180/pi))
    print("Pred Loc:", "\t", predict_loc[0],"\t", predict_loc[1],"\t", predict_loc[2])
    print("ERROR:  ","\t",  errors[k][0],"\t",   errors[k][1],"\t", degrees( errors[k][2]))

    # Time
    t1= time.perf_counter() - t0
    print("Algorithm time elapse:",t1)


    # ************************** Ploting  ********************************** #

    plot_erros(errors)
    radius = -119
    if len(measures) != 0:
        for i in range (robot_measures.shape[0]):
            axis[0].scatter(predict_loc[0] + robot_measures[i]*cos(predict_loc[2] + radians(radius)), robot_measures[i]*sin(predict_loc[2] + radians(radius)) + predict_loc[1], s = 7,  c = 'blue')   
            radius += 10
    plot('Algorithm in Offline Mode', map.n_walls, map.map)

        
    k +=1

    if k == last_iteration:
        break

# Plotting Statistics
#pl.plot_erros(errors)