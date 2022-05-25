import numpy as np
import matplotlib.pyplot as plt
import scipy
import scipy.stats
from numpy.random import uniform
import cv2
#import pygame 
#import matplotlib.pyplot.scatter



#Pode isto ser um mapa? 
#Laser seria a do robot a cada um destes pontos.
#Ou seja, em vez de termos uma parede temos muitos pólos?
#Porque na verdade o laser retorna um valor: Uma distância
#Só que faz um varrimento muito rápido, ou seja está a fazer
#Medidas a uma velocidade trementa (acho)
x=[5,6,7,8,9,10,5,5,5,5,5,5,6,7,8,9,10,10,10,10,10]
y=[5,5,5,5,5,5,6,7,8,9,10,10,10,10,10,10,6,7,8,9,10]

#plt.scatter(x,y)
#plt.show()




#-------------------------------------------------------------------------------------

def generate_blank_image():

    return np.ones(shape=(512,512,3), dtype=np.int16)

def create_uniform_particles(x_range, y_range, N):
    particles = np.empty((N, 2))
    particles[:, 0] = uniform(x_range[0], x_range[1], size=N)
    particles[:, 1] = uniform(y_range[0], y_range[1], size=N)
    return particles

N=50
x_range=np.array([100,400])
y_range=np.array([100,400])
particles=create_uniform_particles(x_range, y_range, N)
img = generate_blank_image()
img2 = generate_blank_image()

cv2.rectangle(img2, pt1=(400,400), pt2=(100,100), color=(255,255,0), thickness= -1)



#draw_particles:
for particle in particles:
    cv2.circle(img2,tuple((int(particle[0]),int(particle[1]))),1,(0,0,0),-1)

plt.imshow(img)
plt.imshow(img2)
plt.show()
