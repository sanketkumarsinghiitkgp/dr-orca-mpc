from system import System
import numpy as np
import matplotlib.pyplot as plt
import os
side_length = 4.2
radius = 0.2
A = np.array([[1, 0, 0, 0],[0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
B = np.array([[1, 0],[0, 1],[1, 0],[0, 1]])
Q = np.diag([1, 1, 0.1, 0.1])
R = np.diag([0.1,0.1])
N = 10
v_max = radius
H = np.array([[0, 0, 1, 0], [0, 0, -1, 0], [0, 0, 0, 1], [0, 0, 0, -1]])
h = v_max*np.array([[1], [1], [1], [1]])
G = np.zeros((2,2))
g = np.zeros((2,1))
dev = 0.1
vert_deviation_list = [5*dev*radius, dev*radius, -1.5*dev*radius, -4*dev*radius]
# x_0_list, x_F_list = System.line_segment_starting_and_end_points(side_length)
x_0_list, x_F_list = System.square_vertex_starting_and_end_points(side_length)
for i in range(len(x_0_list)):
    x_0_list[i]+=np.array([[0],[vert_deviation_list[i]],[0],[0]])
print(x_F_list)
plot_circles_flag = True
system = System(A, B, G, g, H, h, radius, Q, R, x_0_list, x_F_list )
system.simulate_orca_mpc(N = N, plot_circles_flag = plot_circles_flag)
os.remove("system_trajectory.png")
# system.simulate_orca()
system.plot_trajectory(plot_circles_flag)

for i in range(len(x_0_list)):
    plt.scatter([x_F_list[i][0]], [x_F_list[i][1]], label= "Destination for Agent id: "+str(i))
plt.legend()
plt.xlim(-(side_length+1), (side_length+1))
plt.ylim(-(side_length+1),(side_length+1))
plt.savefig("system_trajectory.png")

#was not working, because the collision condition was wrong.
#head on collision is a corner case.
