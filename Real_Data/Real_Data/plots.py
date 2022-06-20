""" *********************************** Libraries ************************************************* """

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi
from scipy.stats import gaussian_kde

""" ************************************* Global Variables ****************************************  """


def plot_laser(x2,y2):

    plt.scatter(x2, y2, c = '#e377c2')       

    return


# Plot the simulation
def plot_simulation(label, robot_loc, particles, n_walls, map, M):

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
    plt.scatter(x, y, c = z, s=5, label = "particles")
    '''
    for i in range(M):
       plt.scatter(particles[i,0], particles[i,1], marker = (3, 0, particles[i,2]*(180/pi)), c = 'blue' , s = 10)
    '''

    # Plot robot
    plt.scatter(robot_loc[0], robot_loc[1], marker = (6, 0, robot_loc[2]*(180/pi)), c = '#d62728' , s=180, label = "Real position", edgecolors='black')
    plt.plot((robot_loc[0],(1/6)*cos(robot_loc[2])+robot_loc[0]),(robot_loc[1],(1/6)*sin(robot_loc[2])+robot_loc[1]), c = '#17becf')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend(loc='upper right')
    plt.title(label)
    plt.pause(0.01)
    plt.show()
    #plt.clf()
  

    return


# Plot Errors
def plot_erros(errors):

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

    return

