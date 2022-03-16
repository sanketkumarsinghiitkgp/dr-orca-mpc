from system import System
import numpy as np
import matplotlib.pyplot as plt
import os
import time
N = 3
# N_list = [2]10se()
file = open("costs_log.txt","w")
lines = []
cost_vals = []
time_vals = []
dataset_size_list = [x for x in range(1,200,20)]
for dataset_size in dataset_size_list:
    side_length = 2
    radius = 0.1*side_length
    A = np.array([[1, 0, 0, 0],[0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
    B = np.array([[1, 0],[0, 1],[1, 0],[0, 1]])
    Q = np.diag([1, 1, 0.1, 0.1])
    R = np.diag([0.1,0.1])
    v_max = 10*radius
    H = np.array([[0, 0, 1, 0], [0, 0, -1, 0], [0, 0, 0, 1], [0, 0, 0, -1]])
    h = v_max*np.array([[1], [1], [1], [1]])
    G = np.zeros((2,2))
    g = np.zeros((2,1))
    file_name = "system_trajectory_ds"+str(dataset_size)+".png"
    dev = 0.01
    vert_deviation_list = [dev*radius,-dev*radius,0,2*dev*radius]
    # vert_deviation_list = [0,0,0,0]
    # x_0_list, x_F_list = System.diagonal_line_segment_starting_and_end_points(side_length)
    
    # x_0_list, x_F_list = System.line_segment_starting_and_end_points(side_length)
    x_0_list, x_F_list = System.square_vertex_starting_and_end_points(side_length)
    for i in range(len(x_0_list)):
        x_0_list[i]+=np.array([[0],[vert_deviation_list[i]],[0],[0]])
    num_agents = len(x_0_list)
    vel_dataset = [dataset_size*[np.zeros((2,1))] for i in range(num_agents)]
    plot_circles_flag = False
    dummy_vel = 0.5*radius
    # x_0_list[1] += np.array([[0],[0],[dummy_vel],[0]])
    # is_agent_dummy_list = [False, True]
    # system = System(A, B, G, g, H, h, radius, Q, R, x_0_list, x_F_list, is_agent_dummy_list=is_agent_dummy_list )
    system = System(A, B, G, g, H, h, radius, Q, R, x_0_list, x_F_list, vel_dataset = vel_dataset, method = "scenario")    
    start_time = time.time()
    traj_cost = system.simulate_orca_mpc(N = N, plot_circles_flag = plot_circles_flag, plot_steps=True)
    end_time = time.time()
    time_vals.append((end_time-start_time)/len(system.agent_list[0].u))
    if os.path.exists(file_name):
        os.remove(file_name)
    # system.simulate_orca()
    plt.figure()
    system.plot_trajectory(plot_circles_flag)

    for i in range(len(x_0_list)):
        plt.scatter([x_F_list[i][0]], [x_F_list[i][1]], label= "Destination for Agent id: "+str(i))
    plt.legend()
    plt.xlim(-(side_length*1.1), (side_length*1.1))
    plt.savefig(file_name)
    print(traj_cost)
    lines.append("Cost for ds = "+str(dataset_size)+" "+str(traj_cost)+"\n")
    cost_vals.append(traj_cost[0,0])
    #was not working, because the collision condition was wrong.
    #head on collision is a corner case.
print(cost_vals)
plt.figure()
plt.plot(dataset_size_list, cost_vals)

plt.xlabel("Dataset Size")
plt.ylabel("Trajectory cost")
plt.savefig("Cost_vs_DS.png")
print(time_vals)
plt.figure()

plt.xlabel("Dataset Size")
plt.ylabel("Time taken for computing 1 control input")
plt.plot(dataset_size_list, time_vals)
plt.savefig("Time_vs_DS.png")
file.writelines(lines)
file.close()