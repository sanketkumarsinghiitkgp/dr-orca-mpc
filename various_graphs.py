from system import System
import numpy as np
import matplotlib.pyplot as plt
import os
import time
import math
# SELECT_LIST = [1,2,3,4,5]
SELECT_LIST = [3]
for SELECT in SELECT_LIST:
    if SELECT == 1:
        avg_cost_vals = None
        avg_time_vals = None
        num_ensembles = 100
        # sigma = 0.01 gives decent results
        dataset_size_list = [10*(x+1) for x in range(6)]
        frac_collide_list = []
        sigma_list = [0,0.01,0.001,0.00001,0.00003,0.00005]
        for sigma in sigma_list:
            frac_collide = []
            for dataset_size in dataset_size_list:
                N = 5
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
                    system = System(A, B, G, g, H, h, radius, Q, R, x_0_list, x_F_list, vel_dataset = vel_dataset, method = "scenario", sigma=sigma)    
                    start_time = time.time()
                    traj_cost = system.simulate_orca_mpc(N = N, plot_circles_flag = plot_circles_flag, plot_steps=False)
                    if traj_cost == None:
                        cnt_collide += 1
                        continue
                    end_time = time.time()
                frac_collide.append(cnt_collide/num_ensembles)
            frac_collide_list.append(frac_collide)
        plt.figure()
        for i in range(len(frac_collide_list)):
            plt.plot(dataset_size_list, frac_collide_list[i], label="sigma = "+str(sigma_list[i]))

        plt.xlabel("Dataset Size")
        plt.ylabel("Fraction_collision")
        plt.legend()
        plt.savefig("Frac_collision_vs_DS_sigma.png")
        plt.figure()
    elif SELECT == 2:
        avg_cost_vals = None
        avg_time_vals = None
        num_ensembles = 100
        # sigma = 0.01 gives decent results
        N_list = [1, 5, 10, 20]
        frac_collide_list = []
        sigma_list = [0,0.01,0.001,0.00001,0.00003,0.00005]
        for sigma in sigma_list:
            frac_collide = []
            for N in N_list:
                
                dataset_size=5
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
                    system = System(A, B, G, g, H, h, radius, Q, R, x_0_list, x_F_list, vel_dataset = vel_dataset, method = "scenario", sigma=sigma)    
                    start_time = time.time()
                    try:
                        traj_cost = system.simulate_orca_mpc(N = N, plot_circles_flag = plot_circles_flag, plot_steps=False)
                    except:
                        cnt_collide += 1
                        continue

                    if traj_cost == None:
                        cnt_collide += 1
                        continue
                    end_time = time.time()
                frac_collide.append(cnt_collide/num_ensembles)
            frac_collide_list.append(frac_collide)
        plt.figure()
        for i in range(len(frac_collide_list)):
            plt.plot(N_list, frac_collide_list[i], label="sigma = "+str(sigma_list[i]))

        plt.xlabel("Prediction Horizon")
        plt.ylabel("Fraction_collision")
        plt.legend()
        plt.savefig("Frac_collision_vs_N_sigma.png")
        plt.figure()


    if SELECT == 3:
        avg_cost_vals = None
        avg_time_vals = None
        num_ensembles = 1
        # sigma = 0.01 gives decent results
        # dataset_size_list = [(x+1) for x in range(1)]
        dataset_size_list = [1]
        frac_collide_list = []
        sigma_list = [0,0.000000025,0.000000035,0.000000045]
        # sigma_list = [0.001,0.003,0.005,0.007,0.009]
        time_vals_list = []
        for sigma in sigma_list:
            frac_collide = []
            time_vals = []
            for dataset_size in dataset_size_list:
                N = 1
                lines = []
                cost_vals = []
                cnt_collide = 0
                time_taken = 0
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
                    file_name = "system_trajectory_ds_dummy"+str(dataset_size)+"_"+str(ensemble_index)+".png"
                    dev = 0.01
                    vert_deviation_list = [dev*radius,-dev*radius,0,2*dev*radius]
                    # vert_deviation_list = [0,0,0,0]
                    x_0_list, x_F_list = System.diagonal_line_segment_starting_and_end_points(side_length)
                    
                    # x_0_list, x_F_list = System.line_segment_starting_and_end_points(side_length)
                    # x_0_list, x_F_list = System.square_vertex_starting_and_end_points(side_length)
                    for i in range(len(x_0_list)):
                        x_0_list[i]+=np.array([[0],[vert_deviation_list[i]],[0],[0]])
                    num_agents = len(x_0_list)
                    plot_circles_flag = True
                    dummy_vel = 0.5*radius/math.sqrt(2)
                    
                    # vel_dataset = [dataset_size*[np.zeros((2,1))] for i in range(num_agents)]
                    vel_dataset_dummy = [dummy_vel*np.ones((2,1))]
                    for iter in range(dataset_size-1):
                        vel_dataset_dummy.append(vel_dataset_dummy[-1]+np.random.randn(2,1)*sigma)

                    vel_dataset = [dataset_size*[dummy_vel*np.ones((2,1))+np.random.randn(2,1)*sigma] for i in range(num_agents)]
                    is_agent_dummy_list = [False, True]
                    x_0_list[1] += np.array([[0],[0],[dummy_vel],[dummy_vel]])
                    system = System(A, B, G, g, H, h, radius, Q, R, x_0_list, x_F_list, vel_dataset = vel_dataset, method = "scenario", sigma=sigma, is_agent_dummy_list=is_agent_dummy_list)    
                    start_time = time.time()
                    try:
                        traj_cost = system.simulate_orca_mpc(N = N, plot_circles_flag = plot_circles_flag, plot_steps=True)
                    except:
                        cnt_collide += 1
                        continue
                    if traj_cost == None:
                        cnt_collide += 1
                        continue
                    end_time = time.time()
                    time_taken +=end_time-start_time
                frac_collide.append(cnt_collide/num_ensembles)
                time_vals.append(time_taken/num_ensembles)
            frac_collide_list.append(frac_collide)
            time_vals_list.append(time_vals)
        plt.figure()
        for i in range(len(frac_collide_list)):
            plt.plot(dataset_size_list, frac_collide_list[i], label="sigma = "+str(sigma_list[i]))

        plt.xlabel("Dataset Size")
        plt.ylabel("Fraction_collision")
        plt.legend()
        plt.savefig("Frac_collision_vs_DS_sigma_dummy.png")
        plt.figure()
        for i in range(len(time_vals_list)):
            plt.plot(dataset_size_list, time_vals_list[i], label="sigma = "+str(sigma_list[i]))

        plt.xlabel("Dataset Size")
        plt.ylabel("Execution time")
        plt.legend()
        plt.savefig("Execution_Time_vs_DS_sigma_dummy.png")
        plt.figure()
    if SELECT == 4:
        avg_cost_vals = None
        avg_time_vals = None
        num_ensembles = 100 #DEBUG
        # sigma = 0.01 gives decent results
        N_list = [1, 5, 10, 20]
        frac_collide_list = []
        sigma_list = [0,0.01,0.001,0.00001,0.00003,0.00005]
        time_vals_list = []
        for sigma in sigma_list:
            frac_collide = []
            time_vals = []
            for N in N_list:
                
                dataset_size=5
                lines = []
                cost_vals = []
                time_taken = 0
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
                    file_name = "system_trajectory_ds_dummy"+str(dataset_size)+"_"+str(ensemble_index)+".png"
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
                    dummy_vel = 0.5*radius/math.sqrt(2)
                    is_agent_dummy_list = [False, True]
                    x_0_list[1] += np.array([[0],[0],[dummy_vel],[dummy_vel]])
                    system = System(A, B, G, g, H, h, radius, Q, R, x_0_list, x_F_list, vel_dataset = vel_dataset, method = "scenario", sigma=sigma, is_agent_dummy_list=is_agent_dummy_list)    
                    start_time = time.time()
                    traj_cost = system.simulate_orca_mpc(N = N, plot_circles_flag = plot_circles_flag, plot_steps=False)
                    if traj_cost == None:
                        cnt_collide += 1
                        continue
                    end_time = time.time()
                    time_taken +=end_time-start_time
                time_taken /= num_ensembles
                time_vals.append(time_taken)
                frac_collide.append(cnt_collide/num_ensembles)
            time_vals_list.append(time_vals)
            frac_collide_list.append(frac_collide)
        plt.figure()
        for i in range(len(frac_collide_list)):
            plt.plot(N_list, frac_collide_list[i], label="sigma = "+str(sigma_list[i]))

        plt.xlabel("Prediction Horizon")
        plt.ylabel("Fraction_collision")
        plt.legend()
        plt.savefig("Frac_collision_vs_N_sigma_dummy.png")
        plt.figure()
        for i in range(len(time_vals_list)):
            plt.plot(N_list, time_vals_list[i], label="sigma = "+str(sigma_list[i]))

        plt.xlabel("Prediction Horizon")
        plt.ylabel("Execution time")
        plt.legend()
        plt.savefig("Execution_Time_vs_N_sigma_dummy.png")
        plt.figure()
    if SELECT == 5:
        #d_ij
        N_list = [1, 5, 10, 20]
        sigma_list = [0,0.01,0.001,0.00001,0.00003,0.00005]
        dataset_size_list = [x+1 for x in range(6)]
        for sigma in sigma_list:
            for N in N_list:
                for dataset_size in dataset_size_list:    
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
                    file_name = "system_trajectory"+"_"+str(sigma)+"_"+str(N)+"_"+str(dataset_size)+".png"
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
                    plot_circles_flag = True
                    dummy_vel = 0.5*radius/math.sqrt(2)
                    is_agent_dummy_list = [False, True]
                    x_0_list[1] += np.array([[0],[0],[dummy_vel],[dummy_vel]])
                    system = System(A, B, G, g, H, h, radius, Q, R, x_0_list, x_F_list, vel_dataset = vel_dataset, method = "scenario", sigma=sigma, is_agent_dummy_list=is_agent_dummy_list)    
                    start_time = time.time()
                    traj_cost = system.simulate_orca_mpc(N = N, plot_circles_flag = plot_circles_flag, plot_steps=False)
                    system.plot_dij("d_ij"+"_"+str(sigma)+"_"+str(N)+"_"+str(dataset_size)+".png",0,1)
                    system.plot_trajectory(plot_circles_flag=plot_circles_flag)
                    for i in range(len(x_0_list)):
                        plt.scatter([x_F_list[i][0]], [x_F_list[i][1]], label= "Destination for Agent id: "+str(i))
                    plt.legend()
                    plt.xlim(-(side_length*1.1), (side_length*1.1))
                    plt.savefig(file_name)
                    