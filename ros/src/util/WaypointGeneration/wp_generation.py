'''
This script read a path from csv file for a smooth waypoint generation
It requires python3 and pandas
Written by Punnu Phairatt, Parkopedia

'''


import os
#import pandas as pd
import csv
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
from math import *

from copy import deepcopy
import tf


MAX_ACC = 0.01
MAX_SPEED = 2.78
CSV_HEADER =  ['rosbagTimestamp', 'header', 'seq', 'stamp', 'secs', 'nsecs', 'frame_id', 'pose', 'position', 'x', 'y', 'z', 'orientation', 'qx', 'qy', 'qy', 'qw']

def csv_reader(filepath, n_data=-1):
    data = {}
    with open(filepath) as f:
        reader = csv.DictReader(f, delimiter=',')
        headers = reader.fieldnames
        for header in headers:
            data[header] = np.array([])

        for row in reader:
            for header in headers:
                data[header] = np.append(data[header], row[header])
    f.close()
    return data

def euler_from_quaternion(quaternion):
    return tf.transformations.euler_from_quaternion(quaternion)

def sampling(pose_data,distance=1.0):
    X = pose_data['x'].astype(np.float)
    Y = pose_data['y'].astype(np.float)
    QX = pose_data['qx'].astype(np.float)
    QY = pose_data['qy'].astype(np.float)
    QZ = pose_data['qz'].astype(np.float)
    QW = pose_data['qw'].astype(np.float)
    X0 = X[0]
    Y0 = Y[0]
    YAW0 = euler_from_quaternion((QX[0],QY[0],QZ[0],QW[0]))[2]
    new_x = []
    new_y = []
    new_yaw = []
    new_x.append(X0)
    new_y.append(Y0)
    new_yaw.append(YAW0)
    for x,y,qx,qy,qz,qw in zip(X,Y,QX,QY,QZ,QW):
        dx = x-X0
        dy = y-Y0
        step = sqrt((x-X0)**2 + (y-Y0)**2)
        if( step > distance ):
            # point
            new_x.append(x)
            new_y.append(y)
            X0 = x
            Y0 = y
            # orientation
            new_yaw.append(euler_from_quaternion((qx,qy,qz,qw))[2])

    return new_x, new_y, new_yaw

def smooth(path, weight_data = 0.5, weight_smooth = 0.1, tolerance = 0.000001):
    # Make a deep copy of path into newpath
    newpath = deepcopy(path)
    # y is a new smooth path
    # x is an original path
    # yi' is a new position after iteration from yi given by
    # yi' = yi + alpha (xi - yi) + beta (yi + 1 + yi - 1 - 2 * yi)
    # alpha = weight_data
    # beta  = weight_smooth
    # gamma = 1 (learning rate?)
    # ignore the starting and last point
    sum_diff = tolerance
    while(sum_diff >= tolerance):
        sum_diff = 0.
        for i in range(1,len(path)-1):
            for j in range(len(path[0])):
                previous_itr = newpath[i][j]
                newpath[i][j] += weight_data *(path[i][j] - newpath[i][j]) + weight_smooth *(newpath[i-1][j] + newpath[i+1][j] - 2.0 * newpath[i][j])
                current_itr = newpath[i][j]
                # sum diff of each dx, dy of the current (iterate previously) and new path in pairs
                # it is show the convertion when the rate of change is no more or very small
                sum_diff += abs(current_itr - previous_itr)
    return newpath

def deceleration(u, distance, result):
	if u > MAX_SPEED:
		return 0
	else:
		if u <= MAX_SPEED:
			result.append(u)
		u += sqrt(2*MAX_ACC*distance)
		return deceleration(u, distance, result)

if __name__ == "__main__":
    pose_data_file = "./park_mcity_base_pose_ground_truth.csv"
    pose_data = csv_reader(pose_data_file)

    # car x,y pose
    timestamp =  pose_data['secs'].astype(np.float) +  pose_data['nsecs'].astype(np.float)/1e9
    X = pose_data['x'].astype(np.float)
    Y = pose_data['y'].astype(np.float)

    # distance down sampling
    distance = 1.0
    Xs,Ys,YAWs = sampling(pose_data,distance)

    # path smoothing
    weight_data = 0.8    #0.2
    weight_smooth = 0.2  #0.8
    path = list(map(list, zip(Xs, Ys)))
    smooth_path = smooth(path, weight_data, weight_smooth)
    Xs, Ys = zip(*smooth_path)

    # deceleration to stop gently
    Vs = len(Xs) * [MAX_SPEED]
    #smooth_v = []
    #deceleration(0.0, distance, smooth_v)
    #if len(smooth_v) <= len(Xs):           # check bound
    #    smooth_v = smooth_v[::-1]          # reverse list to decreasing order [5.56,...,0.0]
    #    Vs[len(smooth_v):] = smooth_v      # copy a portion of smooth_v to the end

    # write to csv file
    with open('./park_mcity_path_1m.csv', 'w') as f:
        writer = csv.writer(f)
        #writer.writerow(['X','Y','Z','YAW', 'V'])
        for xx,yy,vv,yaw in zip(Xs, Ys, Vs, YAWs):
            row = [xx,yy,0.0,yaw,round(vv,2)]
            writer.writerow(row)
    f.close()

    # subplot throttlel, v, steering, v
    fig1 = plt.figure(1)
    plt.axis('equal')
    plt.plot(X,Y,'.r')
    plt.plot(Xs,Ys,'*b')
    plt.title("Throttle Input %")

    fig2 = plt.figure(2)
    plt.plot(YAWs)

    plt.show()
