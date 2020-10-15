import numpy as np
import math
from math import sin, cos

# Given the angles of the leg joints, and the pose of the robot's body,  we will be able to find the position of end of each leg i.e. (x4 y4) 
# Inputs are the three angles theta1/2/3 of the leg and a variable to identify which leg

# Dimensions of the body
L = 2 
W = 2

# Dimensions of each leg
d1 = 1
d2 = 3
d3 = 5

def computeFK(xyz, rpy, theta, leg):
    if (type(xyz) and type(rpy) and type(theta) is not np.ndarray):
        print("Please enter numpy arrays")
        return

    x = xyz[0]
    y = xyz[1]
    z = xyz[2]

    r = rpy[0]
    p = rpy[1]
    y = rpy[2]

    t1 = theta[0]
    t2 = theta[1]
    t3 = theta[2]

    Rx = np.array([ [1, 0, 0, 0],
                    [1, cos(r), -sin(x), 0],
                    [0, sin(r), -cos(r), 0],
                    [0, 0, 0, 1] ])

    Ry = np.array([ [cos(p), 0, sin(p), 0],
                    [0, 1, 0, 0],
                    [-sin(p), 0, -cos(p), 0],
                    [0, 0, 0, 1] ])
    
    Rz = np.array([ [cos(y), -sin(y), 0, 0],
                    [sin(y), cos(y), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1] ])
    
    R = np.multiply(Rx, Ry, Rz)

    l = np.array([ [1, 0, 0, x],
                   [0, 1, 0, y],
                   [0, 0, 1, z],
                   [0, 0, 0, 1] ])

    Tm = np.multiply(R, l)

    leg = np.array([ [0, 0, 1, L/2],
                   [0, 1, 0, 0],
                   [-1, 0, 0, W/2],
                   [0, 0, 0, 1] ])

    if leg == 'br': 
        lvar1 = -1
        lvar2 = 1

    if leg == 'fr':
        print('hi')
        lvar1 = 1
        lvar2 = 1

    if leg == 'fl':
        lvar1 = 1
        lvar2 = -1

    if leg == 'bl':
        lvar1 = -1
        lvar2 = -1

    leg[0,3] *= lvar1
    leg[2,3] *= lvar2
    Ttoleg = np.multiply(Tm, leg)

    m11 = cos(t2)*cos(t3)*sin(t1) - sin(t1)*sin(t2)*sin(t3)
    m12 = -cos(t2)*sin(t1)*sin(t3) - cos(t3)*sin(t1)*sin(t2)
    m13 = -cos(t1)
    m14 = d2*cos(t2)*sin(t1) - d1*cos(t1) + d2*cos(t2)*cos(t3)*sin(t1) - d3*sin(t1)*sin(t2)*sin(t3)

    m21 = cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)
    m22 = cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2)
    m23 = -sin(t1)
    m24 = d3*cos(t1)*sin(t2)*sin(t3) - d2*cos(t1)*cos(t2) - d3*cos(t1)*cos(t2)*cos(t3) - d1*sin(t1)

    m31 = cos(t2)*sin(t3) + cos(t3)*sin(t2)
    m32 = cos(t2)*cos(t3) - sin(t2)*sin(t3)
    m33 = 0
    m34 = d2*sin(t2) + d3*cos(t2)*sin(t3) + d3*cos(t3)*sin(t2)

    m41 = 0
    m42 = 0
    m43 = 0
    m44 = 0

    Tlegtotip = np.array([ [m11, m12, m13, m14],
                                   [m21, m22, m23, m24],
                                   [m31, m32, m33, m34],
                                   [m41, m42, m43, m44] ])

    return Ttoleg, Tlegtotip, np.multiply(Ttoleg, Tlegtotip)
    
    
    

