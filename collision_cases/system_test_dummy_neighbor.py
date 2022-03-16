from system import System
import numpy as np
import matplotlib.pyplot as plt
import os
N_list = [1,3,5,10]
open('costs_log.txt', 'w').close()
file = open("costs_log.txt","w")
lines = []
cost_vals = []
for N in N_list:
    side_length = 2
    radius = 0.01*side_length
    A = np.array([[1, 0, 0, 0],[0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
    B = np.array([[1, 0],[0, 1],[1, 0],[0, 1]])
    Q = np.diag([1, 1, 0.1, 0.1])
    R = np.diag([0.1,0.1])
    v_max = radius
    H = np.array([[0, 0, 1, 0], [0, 0, -1, 0], [0, 0, 0, 1], [0, 0, 0, -1]])
    h = v_max*np.array([[1], [1], [1], [1]])
    G = np.zeros((2,2))
    g = np.zeros((2,1))
    file_name = "system_trajectory_"+str(N)+".png"
    dev = 0.1
    vert_deviation_list = [-dev*radius,dev*radius,0,2*dev*radius]
    # vert_deviation_list = [0,0,0,0]
    x_0_list, x_F_list = System.line_segment_starting_and_end_points(side_length)
    # x_0_list, x_F_list = System.square_vertex_starting_and_end_points(side_length)
    for i in range(len(x_0_list)):
        x_0_list[i]+=np.array([[0],[vert_deviation_list[i]],[0],[0]])

    plot_circles_flag = True
    dummy_vel = 0.5*radius
    x_0_list[1] += np.array([[0],[0],[dummy_vel],[0]])
    # is_agent_dummy_list = [False, True]
    is_agent_dummy_list = [False, False]
    system = System(A, B, G, g, H, h, radius, Q, R, x_0_list, x_F_list, is_agent_dummy_list=is_agent_dummy_list )
    traj_cost = system.simulate_orca_mpc(N = N, plot_circles_flag = plot_circles_flag)
    if os.path.exists("demofile.txt"):
        os.remove("demofile.txt")
    # system.simulate_orca()
    system.plot_trajectory(plot_circles_flag)

    for i in range(len(x_0_list)):
        plt.scatter([x_F_list[i][0]], [x_F_list[i][1]], label= "Destination for Agent id: "+str(i))
    plt.legend()
    plt.xlim(-(side_length*1.1), (side_length*1.1))
    plt.savefig("system_trajectory_"+str(N)+".png")
    print(traj_cost)
    lines.append("Cost for N = "+str(N)+" "+str(traj_cost)+"\n")
    cost_vals.append(traj_cost)
    #was not working, because the collision condition was wrong.
    #head on collision is a corner case.
plt.plot(N_list, cost_vals)
plt.savefig("Cost_vs_N.png")
file.writelines(lines)
file.close()