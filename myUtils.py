#!/usr/bin/python3

# ========================================
# ENPM673 Spring 2021: Perception for Autonomous Robotics
# Utils for Homework-1 - imported by other python files
#
# Functions:
# 1) calculate_svd - calculate SVD
# 2) calculate_least_square - calculate least square coefficients
# 3) calculate_total_least_square - calculate total least square coefficients
# 4) calculate_RANSAC - calculate RANSAC coefficients
# 5) centroid - calculate centroid of an image
#
# Author: Siddharth Telang(stelang@umd.edu)
# ========================================

from numpy import array
from numpy.linalg import linalg
from numpy.linalg import inv
import cv2
import math
import matplotlib.pyplot as plt
import numpy as np
import random

def calculate_svd(S):
    eval_right, evec_right = linalg.eig(np.dot(S.T, S))
    eval_left, evec_left = linalg.eig(np.dot(S, S.T))

    # sort and reverse the indexes
    idx_right = eval_right.argsort()[::-1]
    idx_left = eval_left.argsort()[::-1]
    
    # sort eigen values in descending order
    eval_right = np.sort(eval_right)[::-1]
    
    # calculate Sigma matrix
    sig = np.diag(abs(eval_right)**0.5)

    # form the sorted eigen right and left eigen vectors
    evec_right = evec_right[:, idx_right]
    evec_left = evec_left[:, idx_left]

    return evec_left, sig, evec_right.T

# Function to calculate coefficients for a parabola by "Least Square Method"
def calculate_least_square(x, y):
    x = np.array(x)
    y = np.array(y)
    X = np.stack(((x**2), x, np.ones(len(x))), axis = 1)
    C = np.dot(inv(np.dot(X.T, X)), (np.dot(X.T, y)))
    return C[0], C[1], C[2]

# Function to calculate coefficients for a parabola by "Total Least Square Method"
def calculate_total_least_square(x, y):
    x = np.array(x)
    y = np.array(y)
    # Calculate mean of x
    xbar = np.mean(x)
    # calculate mean of x^2
    x2bar = np.mean(x**2)
    # calculate mean of y
    ybar = np.mean(y)
    # stack the matrix
    Z = np.stack( ( (((x**2) - x2bar).T), ((x - xbar).T), ((y - ybar).T) ), axis =1)
    # form the matrix to be applied for SVD
    S = np.dot(Z.T, Z)
    # Calculate U, E, VT
    U, E, VT = calculate_svd(S)
    # take the last column of V or last row of V.T as the coefficients
    # as they correspond to the minimum value of sigma
    F = VT[2:]
    F = F.ravel()
    a, b, c = F[0], F[1], F[2]
    # calculate distance d with found coefficients
    d = a*x2bar + b*xbar + c*ybar
    return a, b, c, d

def calculate_RANSAC(x_list, y_list, index_list):
    a_best, b_best, c_best = 0.0, 0.0, 0.0
    inliers_best = 0

    for i in range(100):
        inliers_current = 0
        random_index = random.sample(index_list, 3)
        x_temp, y_temp = [], []
        for j in random_index:
            x_temp.append(x_list[j])
            y_temp.append(y_list[j])
        
        a_current, b_current, c_current = calculate_least_square(x_temp,y_temp)
    
        for k in index_list:
            error = abs(a_current*(x_list[k]**2) + b_current*(x_list[k]) + c_current - y_list[k])       
            if error < 22:
              inliers_current += 1
    
        if inliers_current > inliers_best:
            inliers_best = inliers_current
            a_best, b_best, c_best = a_current, b_current, c_current
            count = i+1

    print('RANSAC Best coeff = ' + str(a_best) + ' ; ' + str(b_best) + ' ; ' + str(c_best))
    print('Total inliers  = ' + str(inliers_best))
    print('iteration count at = ' + str(count))
    return a_best, b_best, c_best


# Function to calculate the centroid of an object
def centroid(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, th = cv2.threshold(image,215, 255, cv2.THRESH_BINARY_INV)
    # calculate the Moments
    M = cv2.moments(th)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return cX, cY
