""" Libraries """

from turtle import clear
import numpy as np
import matplotlib.pyplot as plt
import scipy
from numpy.random import uniform
from numpy.random import normal
from math import cos, sin, sqrt, exp, pi
from scipy import stats
import cv2

""" Functions """

# import_image():
def import_image(file_name):

    img = cv2.imread(file_name, cv2.COLOR_BGR2GRAY)
    print(img)
    grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    return img


""" main() """

#im = import_image('map.pgm')
im = np.load('map.pgm')
plt.imshow(im)
plt.show()
print(im)
print(im.shape)
map = np.array(im)
print(map.shape)
print(map)

