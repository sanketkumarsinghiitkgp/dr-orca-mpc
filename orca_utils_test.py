from orca_utils import *
import matplotlib.pyplot as plt
import numpy as np

while(true):
    x_c = input("Circle Center x (default is 5)")
    if x_c == "":
        x_c = 5
    else:
        x_c = float(x_c)
    y_c = input("Circle Center y (default is 5)")
    if y_c == "":
        y_c = 5
    else:
        y_c = float(y_c)
    radius = input("Radius (default is 4)")
    if radius == "":
        radius = 4
    else:
        radius = float(radius)
    
    x_p = input("Point x (default 1)")
    if x_p == "":
        x_p = 1
    else:
        x_p = float(x_p)
    y_p = input("Point y (default 0)")
    if y_p == "":
        y_p = 0
    else:
        y_p = float(y_p)

    center = np.array([[x_c,y_c]]).T
    point = np.array([[x_p,y_p]]).T
    projected_point = projectOnVO(center, radius, point)
    
    theta = np.linspace(0,2*np.pi,100)

    x1 = center[0,0]+radius*np.cos(theta)
    x2 = center[1,0]+radius*np.sin(theta)

    fig, ax = plt.subplots(1)
    ax.plot(x1,x2)
    ax.plot(point[0,0],point[1,0], 'ro', label="point")
    ax.plot([projected_point[0,0], point[0,0]],[projected_point[1,0], point[1,0]])
    ax.plot([projected_point[0,0], 0],[projected_point[1,0], 0])
    
    ax.plot([point[0,0], 0],[point[1,0], 0])
    ax.plot(projected_point[0,0],projected_point[1,0], 'bo', label="projected point")
    ax.plot([0],[0], 'go', label="origin")
    ax.set_aspect(1)
    plt.legend(loc="best")

    plt.savefig("abc.png")