'''
Created on Dec 19, 2017

@author: Dominik Baumann
MPI-IS, ICS
dbaumann(at)tuebingen.mpg.de
'''

import numpy as np
# import numpy.matlib as matlib
import matplotlib.pyplot as mpl
import control as ctrl
from scipy.linalg import inv
import copy

# Number of vehicles
num_veh = 3
# Discrete time step
Ts = 0.1
# Number of iterations
num_it = 250
# Desired velocity
with open("desiredspeed.txt", "r") as f:
    data = f.readlines()
    a = str(data[12]).strip(' \n')
des_vel = float(a)

# Distance
with open("position0_all.txt", "r") as f:
    data = f.readlines()
    a = str(data[12]).strip(' \n')
position0 = float(a)

with open("position1_all.txt", "r") as f:
    data = f.readlines()
    b = str(data[12]).strip(' \n')
position1 = float(b)

with open("position2_all.txt", "r") as f:
    data = f.readlines()
    c = str(data[12]).strip(' \n')
position2 = float(c)

dist1 = position0 - position1
dist2 = position2 - position1
print(dist1)

# Prediction horizon
predHor = 5
# Maximum number of iterations for self trigger
Mmax = 350
# Communication cost
delta = 0.7
# Decide which trigger to use (1: Predictive, 2: Self Trigger), trigger_comparison always uses both triggers
trigger = 1
# Maximum noise
w_max = 0.1
v_max = 0.1
# Packet drop probability
pdr = 0.1
# Initial values for state and desired state

#  x_init = [0,80,-10,80,-20,80]T
#  x_des_init = [80,10,80,10,80]T
x_init = np.zeros((2 * num_veh, 1))
x_des_init = np.zeros((2 * num_veh - 1, 1))
x_init[0, 0] = 0
x_init[1, 0] = des_vel
x_des_init[0, 0] = des_vel
x_des_init[1, 0] = dist1
for i in range(1, num_veh):
    x_init[2 * i + 1, 0] = des_vel
    x_init[2 * i, 0] = x_init[2 * i - 2, 0] - dist1
    if i == num_veh - 1:
        x_des_init[-1, 0] = des_vel
    else:
        x_des_init[2 * i, 0] = des_vel
        x_des_init[2 * i + 1, 0] = dist1

# LQR matrices
# 系统矩阵A、输入矩阵B、状态权重矩阵Q和输入权重矩阵R
Q = np.eye(2 * (num_veh - 1) + 1)
R = 1000 * np.eye(num_veh)
