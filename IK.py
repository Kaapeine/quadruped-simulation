import numpy as np
import math
from math import atan2, sin, cos, sqrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import IK_solver


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

def plot3d(leg_dimensions, theta, x, y, z):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(0,0,0,color="pink", s=25)
    ax.scatter(origin[0], origin[1], origin[2], color="red", s=25) # origin 
    ax.scatter(x, y, z, color="green", s=25) # end point 

    # The xn yn zn here do not match the frames, skipping x2 y2 z2 frame

    theta[0] = theta[0] 
    theta[1] = np.pi + theta[1]
    theta[2] = -theta[1] - theta[2]

    print("Angles: ", np.degrees(theta[0]), np.degrees(theta[1]), np.degrees(theta[2]))

    x1 = origin[0]
    y1 = origin[1] + leg_dimensions[0]*cos(theta[0])
    z1 = origin[2] + leg_dimensions[0]*sin(theta[0])
    ax.plot([origin[0], x1], [origin[1], y1], [origin[2], z1], label='Line 1') 
    print("Distance 1: ", sqrt((x1-origin[0])**2 + (y1-origin[1])**2 + (z1 - origin[2])**2))

    x2 = x1 - leg_dimensions[1]*sin(theta[1]) 
    y2 = y1
    z2 = z1 + leg_dimensions[1]*cos(theta[1])
    ax.plot([x1, x2], [y1, y2], [z1, z2], label='Line 2')
    print("Distance 2: ", sqrt((x2-x1)**2 + (y2-y1)**2 + (z2 - z1)**2))

    x3 = x2 + leg_dimensions[2]*sin(theta[2]) 
    y3 = y2
    z3 = z2 + leg_dimensions[2]*cos(theta[2])
    ax.plot([x2, x3], [y2, y3], [z2, z3], label='Line 3')
    print("Distance 3: ", sqrt((x3-x2)**2 + (y3-y2)**2 + (z3 - z2)**2))

    ax.scatter(x3, y3, z3, color="red", s=25)
    print("Chain ends here: ", x3, y3, z3)
    print("Supposed to end: ", x, y, z)
    error = sqrt((x3-x)**2 + (y3-y)**2 + (z3 - z)**2)
    print("Error: ", error)

    # ax.set_aspect('equal', adjustable='box')
    ax.set_autoscalez_on(False)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, labels)

    plt.show()

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
    plot3d(leg_dimensions, theta, x, y, z)
    # except:
        # print("Given end point is too far away!")


