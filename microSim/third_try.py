""" ******************************* Libraries ************************************* """


from turtle import clear
import numpy as np
import matplotlib.pyplot as plt
import scipy
from numpy.random import uniform
from numpy.random import normal
from math import cos, sin, sqrt, exp, pi
from scipy import stats
import cv2 as cv2


""" ******************************* Global Variables********************************** """


# Map:
# For 'map14.pgm'
# 255 -> Free space
# 0 -> Wal
free_space = 255
wall = 0


""" ******************************* Functions() ************************************* """


# import_image():
def import_map(file_name):

    img = cv2.imread(file_name)
    #print('img:',img.shape)


    grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #print('grayscale:', grayscale.shape)
    
    return img, grayscale

# Plot:
def plot():

    plt.imshow(img)
    plt.scatter(robot_loc[0], robot_loc[1], marker = (6, 1, robot_loc[2]*(180/pi)), c = '#d62728' , s=70, label = "Real position")
    plt.show()

    return

# init_robot_pos():
# initialize the robot position
def init_robot_pos():

    loc = np.empty([3,1])
    loc[0] = 120
    loc[1] = 255
    loc[2] = 0

    return loc

# validate_pos():
# x and y must be integers
# If position is ivalid
# Then the position must be equal to the nearest wall
def validate_position(loc, x, y):

    if map[x][y] < 255: #Invalid position

        print(map[x][y])

        if map[x][y] == 0:
            loc[0] = x
            loc[1] = y

        elif map[x][y] > 150: # Mas se não for igual a zero temos que encontrar a parede mais próxima

            # VER MELHOR ESTA PARTE
            return -1
    
    return loc


""" ******************************* main() ************************************* """

# Import Map
img, map = import_map('map1.png')

plt.imshow(map)
plt.show()

src = cv2.imread('map1.png', cv2.IMREAD_GRAYSCALE)
plt.imshow(src)
plt.show()

dst = cv2.Canny(src, 50, 200, None, 3)

# Copy edges to the images that will display the results in BGR
cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
cdstP = np.copy(cdst)

#  Standard Hough Line Transform
lines = cv2.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)

# Draw the lines
if lines is not None:
    for i in range(0, len(lines)):
        rho = lines[i][0][0]
        theta = lines[i][0][1]
        a = cos(theta)
        b = sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)


linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

if linesP is not None:
    for i in range(0, len(linesP)):
        l = linesP[i][0]
        cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

cv2.imshow("Source", src)
cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
cv2.waitKey()


print(cdstP.shape)
print(lines.shape)
print(np.array(linesP).shape)
print(np.array(linesP))

print(linesP[0][0][0])
print(linesP[0][0])

plt.imshow(cdstP)
#plt.scatter(0,0)
plt.show()

"""
# robot_loc: Localization of the robot
robot_loc = init_robot_pos()
print(map[120][250])
print(int(robot_loc[0]), int(robot_loc[1]))
print(robot_loc[0], robot_loc[1])
#robot_loc = validate_position(robot_loc, int(robot_loc[0]), int(robot_loc[1]))
"""
#plot()
