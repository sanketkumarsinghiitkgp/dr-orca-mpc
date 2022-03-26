from system import System
import numpy as np
import matplotlib.pyplot as plt
import os
import time
avg_cost_vals = None
avg_time_vals = None
num_ensembles = 100

dataset_size = 1
frac_collide = []
N_list = [1,3,5,10,15]
for N in N_list:
    # N_list = [2]10se()
    lines = []
    cost_vals = []
    time_vals = []
    cnt_collide = 0
    for ensemble_index in range(num_ensembles):
        side_length = 2
        radius = 0.1*side_length
        A = np.array([[1, 0, 0, 0],[0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
        B = np.array([[1, 0],[0, 1],[1, 0],[0, 1]])
        Q = np.diag([1, 1, 0.1, 0.1])
        R = np.diag([0.1,0.1])
        v_max = radius
        H = np.array([[0, 0, 1, 0], [0, 0, -1, 0], [0, 0, 0, 1], [0, 0, 0, -1]])
        h = v_max*np.array([[1], [1], [1], [1]])
        G = np.zeros((2,2))
        g = np.zeros((2,1))
        file_name = "system_trajectory_ds"+str(dataset_size)+"_"+str(ensemble_index)+".png"
        dev = 0.01
        vert_deviation_list = [dev*radius,-dev*radius,0,2*dev*radius]
        # vert_deviation_list = [0,0,0,0]
        x_0_list, x_F_list = System.diagonal_line_segment_starting_and_end_points(side_length)
        
        # x_0_list, x_F_list = System.line_segment_starting_and_end_points(side_length)
        # x_0_list, x_F_list = System.square_vertex_starting_and_end_points(side_length)
        for i in range(len(x_0_list)):
            x_0_list[i]+=np.array([[0],[vert_deviation_list[i]],[0],[0]])
        num_agents = len(x_0_list)
        vel_dataset = [dataset_size*[np.zeros((2,1))] for i in range(num_agents)]
        plot_circles_flag = False
        dummy_vel = 0.5*radius
        # x_0_list[1] += np.array([[0],[0],[dummy_vel],[0]])
        # is_agent_dummy_list = [False, True]
        # system = System(A, B, G, g, H, h, radius, Q, R, x_0_list, x_F_list, is_agent_dummy_list=is_agent_dummy_list )
        system = System(A, B, G, g, H, h, radius, Q, R, x_0_list, x_F_list, vel_dataset = vel_dataset, method = "scenario", sigma=0.001)    
        start_time = time.time()
        traj_cost = system.simulate_orca_mpc(N = N, plot_circles_flag = plot_circles_flag, plot_steps=False)
        if traj_cost == None:
            cnt_collide += 1
            continue
        end_time = time.time()
    frac_collide.append(cnt_collide/num_ensembles)
plt.figure()
plt.plot(N_list, frac_collide)

plt.xlabel("Prediction Horizon")
plt.ylabel("Fraction_collision")
plt.savefig("Frac_collision_vs_N.png")
plt.figure()
