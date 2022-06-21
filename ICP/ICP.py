from cmath import isnan
import numpy as np
from scipy.linalg import svd
from scipy.linalg import norm

artificial_points = np.array([[1.65, 9.94840093],
                              [1.65, 10.23237646],
                              [1.65, 10.4924345 ],
                              [1.65, 10.74615764],
                              [1.65, 11.0094627 ],
                              [1.65, 11.3008116 ],
                              [1.65, 11.64698793],
                              [1.65, 12.09489332],
                              [1.65, 12.74290417],
                              [1.65, 13.84963052],
                              [1.52090176, 15.89],
                              [0.57075711, 15.89],
                              [0, 12.50058018],
                              [0, 11.29271486],
                              [0, 11.00470251],
                              [0, 10.86964134],
                              [0, 10.78733403],
                              [0, 10.72897839],
                              [0, 10.68302826],
                              [0, 10.64376766],
                              [0, 10.60781374],
                              [0, 10.57273565],
                              [0, 10.53633282],
                              [0, 10.49605283]])

real_points = np.array([[0.9540857,  10.25824194],
                        [0.93880573, 10.40969711],
                        [0.93062759, 10.54273792],
                        [0.9164665,  10.66906016],
                        [0.8954029,  10.79308547],
                        [0.87630381, 10.92345475],
                        [0.85733249, 11.07108127],
                        [0.82768721, 11.2433635 ],
                        [0.79315515, 11.47257944],
                        [0.74259063, 11.81155571],
                        [0.66611035, 12.46161889],
                        [0.56474877, 15.80407675],
                        [0.20041071, 10.59379966],
                        [-1.23538431, 15.60101194],
                        [-1.18528049, 13.43488765],
                        [0.20041071, 10.59379966],
                        [-0.93758487, 11.69274922],
                        [-0.89308986, 11.33137511],
                        [-0.8647833,  11.06805459],
                        [-0.83489486, 10.85193033],
                        [-0.8171046,  10.66495126],
                        [-0.80306183, 10.48833045],
                        [-0.7752699,  10.31402775],
                        [-0.80623863, 10.10282398],])

artificial_points = np.transpose(artificial_points)
real_points = np.transpose(real_points)
scanSize = 24

def do_icp(targetScan, givenScan):
    
    cores =  np.zeros((3, scanSize))
    R = np.eye(2,2)
    T = np.eye(2,1)
    givenScan2 = givenScan
    for iterations in range(100):
        for i in range(scanSize):
            if np.isnan(targetScan[0,i]) == False and np.isnan(targetScan[1,i]) == False:
                
                distMin = np.inf
                idx = 0
                for j in range(scanSize):
                    if np.isnan(givenScan2[0,i]) == False and np.isnan(givenScan2[1,i]) == False:
                        d = np.linalg.norm(targetScan[:,j]-givenScan2[:,i])
                        if (d < distMin):
                            distMin = d
                            idx = j
                cores[0, i] = i
                cores[1, i] = idx
                cores[2, i] = distMin
        
        for i in range(cores.shape[1]):
            for j in range(cores.shape[1]):
                if (i != j) and (cores[0, i] != 0) and (cores[1, i] != 0):
                    if cores[1, i] == cores[1, j]:
                        if cores[2, i]<= cores[2,j]:
                            cores[1, j] = 0
                            cores[2, j] = 0
                        else:
                            cores[1, i] = 0
                            cores[2, i] = 0
        
        cntCores = 0
        curMean = np.zeros((2,1))
        refMean = np.zeros((2,1))

        K = np.zeros((2,2))

        for i in range(cores.shape[1]):
            if (cores[0, i] != 0) and (cores[1, i] != 0):
                
                x = int(cores[1,i])
                y = int(cores[0,i])
                K = K + np.matmul(targetScan[:, int(cores[1,i])], np.transpose(givenScan2[:, int(cores[0,i])]))
                
                curMean = curMean + givenScan2[:, int(cores[0,i])]
                refMean = refMean + targetScan[:, int(cores[1,i])]
                cntCores = cntCores + 1
        
        curMean = curMean/cntCores
        refMean = refMean/cntCores

        U, S, V = np.linalg.svd(K)   
        R_temp = np.matmul(U,V)
        R = np.matmul(R_temp,R)

        T_temp = refMean - np.matmul(R_temp,curMean)
        T = T+T_temp

        givenScan2_temp = np.matmul(R, givenScan)
        givenScan2 = givenScan2_temp+T

if __name__ == '__main__':
    do_icp(artificial_points, real_points)