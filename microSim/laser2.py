import cv2 
import numpy as np
from matplotlib.pyplot import imshow,show
import math


def uncertainty_add(distance, angle, sigma): #create noise

    mean = np.array([distance, angle])
    covariance = np.diag(sigma**2)
    distance, angle = np.random.multivariate_normal(mean,covariance) # create noise with a sigma an a mean given 
    distance = max(distance,0) # no negative numbers
    angle = max(angle,0) #no negative numbers
    return[distance, angle]

class LaserSensor:

    def __init__( self, Range, map, uncertainty):

        self.Range = Range
        self.speed = 4 #rounds per second
        self.sigma = np.array([uncertainty[0], uncertainty[1]]) # sensor measurment noise, noise in the angle and range
        self.position = (0,0) # position of the robot/laser
        self.W = map.shape[1]
        self.H = map.shape[0]
        self.Map = map
        self.sensedObstacles = []

    def distance(self, obstaclePosition):

        px = (obstaclePosition[0]-self.position[0])**2
        py= (obstaclePosition[1]-self.position[1])**2
        return math.sqrt(px+py)

# A straight line of segment with the lenght of the range and along this line we will take
#a number of samples, if the number is equal to 255 it means a wall as been reached
    def sense_obstacles(self):

        data = []
        x1, y1 = self.position[0], self.position[1]
        for angle in np.linspace(0 , math.pi, 10, False):
            x2, y2 = (x1 + self.Range * math.cos(angle), y1 - self.Range * math.sin(angle)) # get the final point of the range
            for i in range (0, 100): # calculate a point in the line segment until it intercepts something
                u = i/100
                x = int(x2* u + x1 *(1-u))#Get a x  between the laser range and the laser position
                y = int(y2* u + y1 *(1-u))#Get a y  between the laser range and the laser position
                if 0 < x < self.W and 0< y < self.H: # to see if the point is out of the map
                    if self.Map[x,y] == 255: #in 255 is where the walls are
                        distance = self.distance((x,y))
                        output = uncertainty_add(distance, angle, self.sigma)
                        output.append(self.position)
                        data.append(output) #store the measurments
                        break
        if len(data) > 0 :
            return data
        else:
            return False

class envio: # showing the data as a part of the map you got build the point cloud map --> represented by a list of 2d cordinates

    def AD2pos(self, distance, angle, robotPosition):
        x = distance * math.cos(angle)+ robotPosition[0]
        y = -distance * math.sin(angle)+ robotPosition[1]
        return (int(x), int(y))

    def dataStorage(self, data):
        self.pointCloud=[]
        i = 0
        print(len(data))
        for element in data:
            point = self.AD2pos(element[0],element[1],element[2])
            if point not in self.pointCloud:
                i = i +1
                self.pointCloud.append(point)
        print(i)
    def show_sensorData(self, map):
        Map_sensor = np.zeros([600,1200])
        #print()
        for points in self.pointCloud:
            print(self.pointCloud)
            #print(points[0])
            Map_sensor[points] = 1
            print(map[points])

'''def plotting(map ):

    plt.scatter(map, c = 'black')
    plt.scatter(laser.position, marker = 'x', c = 'red' )
    plt.scatter(env.pointCloud, marker = '2', c = 'green', s = 10 )
    plt.show()
    return'''

#-------------main---------
img = cv2.imread('map1.png', cv2.IMREAD_GRAYSCALE)
print(img)
imshow(img)
show()

Map = np.array(img) # An array of occupancy grid 255 black (occupied) and 

laser = LaserSensor(200, Map,uncertainty=(0.5, 0.01) )
env = envio()
while(1):
    print('Position x')
    x = int(input())
    print('position y')
    y = int(input())
    laser.position = [x,y]
    sensor_data = laser.sense_obstacles()
    print(sensor_data)
    env.dataStorage(sensor_data)
    env.show_sensorData(Map)
    #plotting(Map)