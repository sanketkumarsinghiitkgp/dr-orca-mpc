import numpy as np
import matplotlib.pyplot as plt
from agent import Agent
import copy
from tqdm import tqdm
class System:
    
    def __init__(self, A, B, G, g, H, h, radius, Q, R, x_0_list, x_F_list , is_agent_dummy_list=[], vel_dataset = [], method = "constant_vel", sigma = 0):
        # it only makes sense to have velocity constraints not position in G
        assert(len(x_0_list) == len(x_F_list))
        self.num_agents = len(x_0_list)
        if is_agent_dummy_list == []:
            is_agent_dummy_list = [False]*self.num_agents
        assert(len(is_agent_dummy_list) == self.num_agents)
        self.agent_list = []
        self.is_agent_dummy_list = is_agent_dummy_list
        self.vel_dataset = copy.deepcopy(vel_dataset)
        self.method = method
        self.sigma = sigma
        self.d_ij = []
        for i in range(self.num_agents):
            new_agent = Agent(A, B, G, g, H, h, radius=radius, _id=i, x_0=x_0_list[i], Q=Q, R=R, x_F=x_F_list[i], is_agent_dummy_list = self.is_agent_dummy_list, sigma=sigma)
            self.agent_list.append(new_agent)


    @staticmethod
    def square_vertex_starting_and_end_points(side_length):
        # x_0, x_F lists
        a = side_length/2
        x_0_list = [np.array([[a], [a], [0], [0]]), np.array([[a], [-a], [0], [0]]), np.array([[-a], [-a], [0], [0]]), np.array([[-a], [a], [0], [0]])]
        x_F_list = []
        for i in range(4):
            x_F_list.append(copy.deepcopy(x_0_list[(i+2)%4]))
        return x_0_list,x_F_list


    @staticmethod
    def line_segment_starting_and_end_points(side_length):
        a = side_length/2
        x_0_list = [np.array([[a], [0], [0], [0]]), np.array([[-a], [0], [0], [0]])]
        x_0_list_duplicate = copy.deepcopy(x_0_list)
        x_F_list = []
        for i in range(2):
            x_F_list.append(copy.deepcopy(x_0_list_duplicate[(i+1)%2]))
        return x_0_list, x_F_list

    @staticmethod
    def diagonal_line_segment_starting_and_end_points(side_length):
        a = side_length/2
        x_0_list = [np.array([[a], [a], [0], [0]]), np.array([[-a], [-a], [0], [0]])]
        x_0_list_duplicate = copy.deepcopy(x_0_list)
        x_F_list = []
        for i in range(2):
            x_F_list.append(copy.deepcopy(x_0_list_duplicate[(i+1)%2]))
        return x_0_list, x_F_list


    def norm_sum(self):
        sm = 0
        for agent in self.agent_list:
            if not self.is_agent_dummy_list[agent._id]:
                sm += agent.find_norm()
        return sm

    def collision(self):
        for i in range(len(self.agent_list)):
            for j in range(len(self.agent_list)):
                if i==j:
                    continue
                if np.linalg.norm(self.agent_list[i].x[-1][0:2]-self.agent_list[j].x[-1][0:2])<0.99*(self.agent_list[i].radius+self.agent_list[j].radius):
                    return True
        return False
    def simulate_orca_mpc(self,max_iter = 200, eps=1e-3, N = 1, plot_circles_flag = True, plot_steps = True):
        traj_cost = 0
        for iter_num in tqdm(range(max_iter)):
            # abc = input("abc")
            for agent in self.agent_list:
                agent.orca_mpc_update(N, self.agent_list, vel_dataset = self.vel_dataset, method = self.method)
            if self.method!="constant_vel":
                new_vel_dataset = []
                for agent in self.agent_list:
                    temp = []
                    for i in range(1,len(self.vel_dataset[agent._id])):
                        temp.append(self.vel_dataset[agent._id][i])
                    temp.append(agent.x[-1][2:4])
                    new_vel_dataset.append(copy.deepcopy(temp))
                self.vel_dataset = new_vel_dataset
            if plot_steps:
                plt.figure().clear()
                self.plot_trajectory(plot_circles_flag)
                plt.savefig("system_trajectory.png")
            cur_norm_sum = self.norm_sum()
            traj_cost += cur_norm_sum
            if self.collision():
                return None
            if(cur_norm_sum<eps):
                print(f'Terminated after {iter_num} iterations')
                return traj_cost
        print(f'Terminated after {max_iter} iterations')
        print(traj_cost)
        return traj_cost

    # def simulate_orca(self,max_iter = 5, eps=1e-6):
    #     for iter_num in tqdm(range(max_iter)):
    #         if(self.norm_sum()<eps):
    #             print(f'Terminated after {iter_num} iterations')
    #             return
    #         for agent in self.agent_list:
    #             agent.orca_update(self.agent_list)
            
    def plot_dij(self, file_name, idx1, idx2):
        plt.figure()
        d_ij = [np.linalg.norm(self.agent_list[idx1].x[i][0:2]-self.agent_list[idx2].x[i][0:2]) for i in range(len(self.agent_list[idx1].x))]
        plt.plot([i for i in range(len(d_ij))], d_ij, label="distance between the agents")
        plt.plot([i for i in range(len(d_ij))], [self.agent_list[idx2].radius+self.agent_list[idx1].radius for i in range(len(d_ij)) ], label="sum of radii of the two agents")
        plt.xlabel("t")
        plt.ylabel("distance between agents")
        plt.legend()
        plt.savefig(file_name)
        plt.figure()

    def plot_trajectory(self, plot_circles_flag):
        for agent in self.agent_list:
            agent.plot_trajectory(plot_circles_flag)