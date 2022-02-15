from system import System
import numpy as np
import matplotlib.pyplot as plt
side_length = 4
radius = 1
A = np.array([[1, 0, 1, 0],[0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]])
B = np.array([[0, 0],[0, 0],[1, 0],[0, 1]])
Q = np.diag([1, 1, 0.1, 0.1])
R = np.diag([0.1,0.1])
N = 5
H = np.zeros((4,4))
h = np.zeros((4,1))
G = np.zeros((2,2))
g = np.zeros((2,1))
x_0_list, x_F_list = System.square_vertex_starting_and_end_points(side_length)

system = System(A, B, G, g, H, h, radius, Q, R, x_0_list, x_F_list )
system.simulate_orca_mpc()
system.plot_trajectory()
plt.legend()
plt.savefig("system_trajectory.png")