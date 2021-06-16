#!/usr/bin/python3

# ========================================
# ENPM673 Spring 2021: Perception for Autonomous Robotics
# Calculating Homography matrix for given points
#
# Author: Siddharth Telang(stelang@umd.edu)
# ========================================
# Run as 'python3 homography.py

import numpy as np
import math
from myUtils import *
from numpy import array
from numpy.linalg import linalg

# define points
x = [5, 150, 150, 5]
y = [5, 5, 150, 150]
xp = [100, 200, 220, 100]
yp = [100, 80, 80, 200]
A = np.zeros(shape=(8,9))

# make array
# [-5, -5, -1, 0, 0, 0, 500, 500, 100]
# [0, 0, 0, -5, -5, -1, 500, 500, 100]
# [-150, -5, -1, 0, 0, 0, 30000, 1000, 200]
# [0, 0, 0, -150, -5, -1, 12000, 400, 80]
# [-150, -150, -1, 0, 0, 0, 33000, 33000, 220]
# [0, 0, 0, -150, -150, -1, 12000, 12000, 80]
# [-5, -150, -1, 0, 0, 0, 500, 15000, 100]
# [0, 0, 0, -5, -150, -1, 1000, 30000, 200]

print('Matrix -')
arr, arr2, j = [], [], 0
for i in range(4):
    arr = [-x[i], -y[i], -1, 0, 0, 0, x[i]*xp[i], y[i]*xp[i], xp[i]]
    arr2 = [0, 0, 0, -x[i], -y[i], -1, x[i]*yp[i], y[i]*yp[i], yp[i]]
    print(arr)
    print(arr2)
    A[j] = np.array(arr)
    A[j+1] = np.array(arr2)
    j+=2

# calculate SVD
U, E, VT = calculate_svd(A)
E = E[:8]

print('\nMatrix U')
print(U)
print('\nMatrix E')
print(E)
print('\nMatrix VT')
print(VT)

# take the last row of VT (last column of V) which corresponds to sigma = 0 to get value for AX=0
X = VT[8].reshape(3,3)

# Divide complete array by last element
H = X/X[2][2]
print('\n Homography Matrix with designed function = ')
print(H)

# check with inbuilt function
src = np.array([[5,5], [150,5],[150,150], [5,150]])
dst = np.array([[100,100], [200,80], [220,80], [100,200]])
print('\n Homorgaphy Matrix with inbuilt function = ')
print(cv2.findHomography(src, dst))