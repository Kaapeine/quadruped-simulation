import numpy as np
import math
from math import atan2, sin, cos, sqrt
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D

import IK_solver
import plotter

# Based on the following paper:
#"https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot"

# There is a small error in the calculations, I still haven't figured out where it arises. The plotting actually works now but it will always look "wrong" because of how matlplotlib scales the axes. There is no solution afaik and it's an open issue.  

def computeIK(leg_dimensions, x, y, z, leg):
    # The problem is with the frames. Their frame doesn't match with Matplotlib's
    L1 = leg_dimensions[0]
    L2 = leg_dimensions[1]
    L3 = leg_dimensions[2]
    print(L1, L2, L3)

    D = (y**2 + (-z)**2 - L1**2 + (-x)**2 - L2**2 -L3**2)/(2*L2*L3)
    print("Term that can go zero: ", 1 - D**2)
    D = checkdomain(D)

    theta1 = -atan2(z,y) - atan2(sqrt(y**2 + (-z)**2 - L1**2), -L1)

    if leg == 1 or leg == 3:
        theta3 = atan2(-sqrt(1 - D**2), D)
    elif leg == 2 or leg == 4:
        theta3 = atan2(sqrt(1 - D**2), D)

    theta2 = atan2(-x, sqrt(y**2 + (-z)**2 - L1**2)) - atan2(L3*sin(theta3), L2 + L3*cos(theta3))

    theta = np.array([theta1, theta2, theta3])
    print("Theta: ", np.degrees(theta1), np.degrees(theta2), np.degrees(theta3))
    return theta 

# Not my code
def checkdomain(D):
    print("D: ", D)
    if D > 1 or D < -1:
        print("____OUT OF DOMAIN____")
        if D > 1:
            D = 0.99
            return D
        elif D < -1:
            D = -0.99
            return D
    else:
        return D

if __name__ == "__main__":

    origin = np.array([0, 0, 0])
    leg_dimensions = np.array([0.1, 0.4, 0.4])
    x = 0 
    y = 0 
    z = -0.65
    # x = 0 
    # y = 0 
    # z = -0.65

    # try:
    theta = computeIK(leg_dimensions, x, y, z, 3)
    coord = np.array([0, 0, -0.65])
    # theta = IK_solver.solve_R(coord, leg_dimensions[0], leg_dimensions[1], leg_dimensions[2])
    plotter.plot3d(leg_dimensions, theta, x, y, z, origin)
    # except:
        # print("Given end point is too far away!")


