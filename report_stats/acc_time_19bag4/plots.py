""" *********************************** Libraries ************************************************* """

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi
from scipy.stats import gaussian_kde

""" ************************************* Global Variables ****************************************  """


def plot_laser(x2,y2):

    plt.scatter(x2, y2, s=5, c = '#e377c2')       

    return


# Plot the simulation
def plot(label, n_walls, map, particles, robot_loc, predict_loc):

    # Calculate the point density of the particles
    x = particles[:,0]
    y = particles[:,1]
    #xy = np.vstack([x,y])
    #z = gaussian_kde(xy)(xy)
    #idx = z.argsort() # Sort the points by density, so that the densest points are plotted last
    #x, y, z = x[idx], y[idx], z[idx]

    # Plot Map
    for i in range(n_walls):
        plt.plot((map[i][0][0],map[i][1][0]),(map[i][0][1],map[i,1,1]), c = 'black')
    
    # Plot Particles
    plt.scatter(x, y, c = '#e377c2', s=1, label = "Particles")

    # Plot robot
    plt.scatter(robot_loc[0], robot_loc[1], marker = (6, 0, robot_loc[2]*(180/pi)), c = 'black' , s=40, label = "AMCL Reference", edgecolors='black')
    plt.plot((robot_loc[0],(1/8)*cos(robot_loc[2])+robot_loc[0]),(robot_loc[1],(1/8)*sin(robot_loc[2])+robot_loc[1]), c = '#17becf')
    plt.plot((predict_loc[0],(1/8)*cos(predict_loc[2])+predict_loc[0]),(predict_loc[1],(1/8)*sin(predict_loc[2])+predict_loc[1]), c = 'black')
    plt.scatter(predict_loc[0], predict_loc[1],c = '#17becf' , s=40, label = "Prediction", edgecolors='black')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title(label)
    plt.legend(loc='upper right')
    plt.pause(0.01)
    plt.show()
    plt.clf()
  

    return


# Plot Errors
def plot_erros(errors):

    x_error =  errors[:,0]
    x_error = x_error[ x_error !=0]
    y_error =  errors[:,1]
    y_error = y_error[ y_error !=0]
    theta_error =  errors[:,2]
    theta_error = theta_error[ theta_error !=0]


    plt.plot(x_error, c = '#bcbd22', label = "x error [m]" )
    plt.plot(y_error, c = '#9467bd', label = "y error [m]" )
    plt.plot(theta_error, c = '#e377c2', label = "Orientation error [rad]")
    plt.xlabel('Iterations')
    plt.ylabel('Error')
    plt.legend(loc='upper right')
    plt.title('Absolute Errors between AMCL Reference and Algortihm Prediction')

    return
