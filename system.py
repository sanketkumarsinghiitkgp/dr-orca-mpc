import numpy as np
import matplotlib.pyplot as plt
from agent import Agent
import copy
from tqdm import tqdm
class System:
    
    def __init__(self, A, B, G, g, H, h, radius, Q, R, x_0_list, x_F_list ):
        # it only makes sense to have velocity constraints not position in G
        assert(len(x_0_list) == len(x_F_list))
        self.num_agents = len(x_0_list)
        self.agent_list = []
        for i in range(self.num_agents):
            new_agent = Agent(A, B, G, g, H, h, radius=radius, _id=i, x_0=x_0_list[i], Q=Q, R=R, x_F=x_F_list[i])
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


    def norm_sum(self):
        sm = 0
        for agent in self.agent_list:
            sm += agent.find_norm()
        return sm


    def simulate_orca_mpc(self,max_iter = 200, eps=1e-6, N = 1):
        for iter_num in tqdm(range(max_iter)):
            if(iter_num == 3):
                print("here")
                print("good")
            if(self.norm_sum()<eps):
                print(f'Terminated after {iter_num} iterations')
                return
            for agent in self.agent_list:
                agent.orca_mpc_update(N, self.agent_list)
            plt.figure().clear()
            self.plot_trajectory()
            plt.savefig("system_trajectory.png")
        print(f'Terminated after {max_iter} iterations')
        print(f'Terminated after {max_iter} iterations')
    
    def simulate_orca(self,max_iter = 5, eps=1e-6):
        for iter_num in tqdm(range(max_iter)):
            if(self.norm_sum()<eps):
                print(f'Terminated after {iter_num} iterations')
                return
            for agent in self.agent_list:
                agent.orca_update(self.agent_list)
            

    def plot_trajectory(self):
        for agent in self.agent_list:
            agent.plot_trajectory()