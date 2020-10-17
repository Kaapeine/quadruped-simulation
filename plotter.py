#!/usr/bin/env python3
from math import atan2, sin, cos, sqrt
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot3d(leg_dimensions, theta, x, y, z, origin):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(0,0,0,color="pink", s=25)
    ax.scatter(origin[0], origin[1], origin[2], color="red", s=25) # origin 
    # ax.scatter(x, y, z, color="green", s=25) # end point 

    # The xn yn zn here do not match the frames used in the paper, skipping x2 y2 z2 frame

    theta[0] = theta[0] 
    theta[1] = np.pi + theta[1]
    theta[2] = -theta[1] - theta[2]

    print("Angles: ", np.degrees(theta[0]), np.degrees(theta[1]), np.degrees(theta[2]))

    x1 = origin[0]
    y1 = origin[1] + leg_dimensions[0]*cos(theta[0])
    z1 = origin[2] + leg_dimensions[0]*sin(theta[0])
    ax.plot([origin[0], x1], [origin[1], y1], [origin[2], z1], label='Coxa') 
    print("Distance 1: ", sqrt((x1-origin[0])**2 + (y1-origin[1])**2 + (z1 - origin[2])**2))

    x2 = x1 - leg_dimensions[1]*sin(theta[1]) 
    y2 = y1
    z2 = z1 + leg_dimensions[1]*cos(theta[1])
    ax.plot([x1, x2], [y1, y2], [z1, z2], label='Femur')
    print("Distance 2: ", sqrt((x2-x1)**2 + (y2-y1)**2 + (z2 - z1)**2))

    x3 = x2 + leg_dimensions[2]*sin(theta[2]) 
    y3 = y2
    z3 = z2 + leg_dimensions[2]*cos(theta[2])
    ax.plot([x2, x3], [y2, y3], [z2, z3], label='Tibia')
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
