#!/usr/bin/python3

# ========================================
# ENPM673 Spring 2021: Perception for Autonomous Robotics
# Finding the curve equation for a ball thrown on a white background by the below methods:
# 1. Least Square Method
# 2. Total Least Square Method
# 3. RANSAC
#
# Author: Siddharth Telang(stelang@umd.edu)
# ========================================
# Run as 'python3 main.py --vid Ball_travel_10fps.mp4'
#        'python3 main.py --vid Ball_travel_2_updated.mp4'

from myUtils import *
from numpy import array
from numpy.linalg import inv
import argparse
import cv2
import math
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--vid', type=str, required=True)
    args = parser.parse_args()
    name = ''
    if args.vid == 'Ball_travel_10fps.mp4':
        print('Case 1 : Without Noise')
        name = 'Without Noise'
    elif args.vid == 'Ball_travel_2_updated.mp4':
        print('Case 2: With Noise')
        name = 'With Noise'
    x_list, y_list, index_list = [], [], []
    height, width = 480, 640
    cap = cv2.VideoCapture(args.vid)
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")
    count = 0
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret:
            res = cv2.resize(frame, (width, height))
            x, y = centroid(res)
            x_list.append(x)
            y_list.append(height - y)
            index_list.append(count)
            count += 1
        else:
            break

    total_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    print('Number of frames = ' +str(cap.get(cv2.CAP_PROP_FRAME_COUNT)))
    cap.release()

    # Calculate by Least Square method
    a, b, c = calculate_least_square(x_list, y_list)
    print('\nLeast Square Coefficients and equation:')
    print('a = ' + str(a) + ' ; b = ' + str(b) + ' ; c = ' + str(c))
    print('y = ' + str(a)+'x^2 + '+ str(b)+ 'x + '+ str(c)+ '\n')
    x = np.linspace(0, width, width)
    y = a*(x**2) + b*x + c

    # Calculate by Total Least Square method
    a1, b1, c1, d1 = calculate_total_least_square(x_list, y_list)
    print('Total Least Square Coefficients and equation:')
    print('a = ' + str(a1) + ' ; b = ' + str(b1) + ' ; c = ' + str(c1))
    x1 = np.linspace(0, width, width)
    # reform equation : d = ax^2 + bx + cy in terms of y
    y1 = (-a1*(x1**2) - b1*x1 + d1) / c1
    print('y = ' + str(-a1/c1)+'x^2 + '+ str(-b1/c1)+ 'x + '+ str(d1)+'\n')

    # Calculate by RANSAC
    a2, b2, c2 = calculate_RANSAC(x_list, y_list, index_list)
    x2 = np.linspace(0, width, width)
    y2 = a2*(x2**2) + b2*x2 + c2
    print('y = ' + str(a2)+'x^2 + '+ str(b2)+ 'x + '+ str(c2)+'\n')

    plt.figure('Scattered Data ' + name)
    plt.ylim(0, height)
    plt.ylabel('Y Axis')
    plt.xlabel('X Axis')
    plt.scatter(x_list, y_list)
    plt.savefig('Scattered Data ' + name + '.jpg')

    plt.figure("Least Square Approximation " + name)
    plt.ylim(0, height)
    plt.ylabel('Y Axis')
    plt.xlabel('X Axis')
    plt.scatter(x_list, y_list)
    plt.plot(y, linestyle='-', color='r')
    plt.savefig('Least Square Approximation ' + name + '.jpg')

    plt.figure("Total Least Square Approximation " + name)
    plt.ylim(0, height)
    plt.ylabel('Y Axis')
    plt.xlabel('X Axis')
    plt.scatter(x_list, y_list)
    plt.plot(y1, linestyle='--', color='r')
    plt.savefig('Total Least Square Approximation ' + name + '.jpg')

    plt.figure('RANSAC ' + name)
    plt.ylim(0,height)
    plt.ylabel('Y Axis')
    plt.xlabel('X Axis')
    plt.plot(y2, linestyle='-', color='r')
    plt.scatter(x_list, y_list)
    plt.savefig('RANSAC ' + name + '.jpg')

    plt.show()
