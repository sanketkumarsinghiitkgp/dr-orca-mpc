from system import System
import numpy as np
import matplotlib.pyplot as plt
side_length = 2
radius = 0.2
A = np.array([[1, 0, 0, 0],[0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
B = np.array([[1, 0],[0, 1],[1, 0],[0, 1]])
Q = np.diag([1, 1, 0.1, 0.1])
R = np.diag([0.1,0.1])
N = 1
v_max = 2*radius
H = np.array([[0, 0, 1, 0], [0, 0, -1, 0], [0, 0, 0, 1], [0, 0, 0, -1]])
h = v_max*np.array([[1], [1], [1], [1]])
G = np.zeros((2,2))
g = np.zeros((2,1))
dev = 0.1
vert_deviation_list = [dev*radius, -dev*radius]
x_0_list, x_F_list = System.line_segment_starting_and_end_points(side_length)
# x_0_list, x_F_list = System.square_vertex_starting_and_end_points(side_length)
for i in range(len(x_0_list)):
    x_0_list[i]+=np.array([[0],[vert_deviation_list[i%2]],[0],[0]])
print(x_F_list)
abc = input("abc")
system = System(A, B, G, g, H, h, radius, Q, R, x_0_list, x_F_list )
system.simulate_orca_mpc(N = N)
# system.simulate_orca()
system.plot_trajectory()

for i in range(len(x_0_list)):
    plt.scatter([x_F_list[i][0]], [x_F_list[i][1]], label= "Destination for Agent id: "+str(i))
plt.legend()
plt.xlim(-2, 2)
plt.ylim(-2,2)
plt.savefig("system_trajectory.png")

#was not working, because the collision condition was wrong.
#head on collision is a corner case.
