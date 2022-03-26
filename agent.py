import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt
from math import pi, cos, atan2, asin, tan
import casadi
from orca_utils import projectOnVO
from atharva_code import get_u
import copy
from flags import *
eps = 1e-6


class Agent:
    
    def __init__(self, A, B, G, g, H, h, radius, _id, x_0, Q, R, x_F, is_agent_dummy_list, tau = 1, sigma = 0):
        self.A = A
        self.B = B
        self.G = G
        self.g = g
        self.H = H
        self.h = h
        self.radius = radius
        self._id = _id
        self.x = [x_0]
        self.Q = Q
        self.R = R
        self.x_F = x_F
        self.u = []
        self.tau = tau
        self.color_list = ["b", "g", "r", "c", "m", "y", "k", "w"]
        self.is_agent_dummy_list = is_agent_dummy_list
        n_x = self.A.shape[1]
        n_u = self.B.shape[1]
        self.prev_mpc_traj = None
        self.sigma=sigma


    def find_norm(self):
        return (self.x[-1]-self.x_F).T @ self.Q @ (self.x[-1]-self.x_F)+(self.u[-1]).T @ self.R @ self.u[-1]


    def evolve_state(self, u):
        n_x = self.x[-1].shape[0]
        assert (n_x == 4)
        w =  (self.sigma*np.random.randn(n_x,1))if NOISY_ENV else np.zeros((n_x,1))
        self.x.append(self.A @ self.x[-1] + self.B @ u+w)
        self.u.append(u)


    def orca_update(self, agent_list):
        # TODO urgent implement traditional orca
        u = self.find_u_orca(agent_list)
        self.evolve_state(u)


    def orca_mpc_update(self, N, agent_list, vel_dataset=None, method ="constant_vel"):
        vel_dataset = copy.deepcopy(vel_dataset)
        u = self.find_u_orca_mpc(N, agent_list, vel_dataset, method)
        self.evolve_state(u)


    def add_orca_constraints(self, opti, x, idx, p_a, p_b, v_a, v_b, is_neighbor_dummy):
        # projection = projectOnVO((p_b-p_a)/self.tau, 2*self.radius/self.tau, v_a-v_b)
        # region = projection["region"]
        # u  = projection["projected_point"]-(v_a-v_b)
        
        #TODO DEBUG
        u_new, _ , region = get_u(p_a, p_b, v_a, v_b, self.radius, self.radius, self.tau)
        # assert(np.linalg.norm(u-u_new)<1e-3)
        u = u_new
        #END DEBUG
        is_outside = region == 1
        if is_neighbor_dummy:
            if is_outside:
                opti.subject_to((x[[idx],2:4].T-(v_a+u)).T@u<=0)
                pass
            else:
                opti.subject_to((x[[idx],2:4].T-(v_a+u)).T@u>=0)
                pass
        else:    
            if is_outside:
                opti.subject_to((x[[idx],2:4].T-(v_a+u/2)).T@u<=0)
                pass
            else:
                opti.subject_to((x[[idx],2:4].T-(v_a+u/2)).T@u>=0)
                pass


    def find_u_orca_mpc(self, N, agent_list, vel_dataset, method):
        n_x = self.A.shape[1]
        n_u = self.B.shape[1]
        if self.is_agent_dummy_list[self._id]:
            return self.x[-1][2:4]
        opti = casadi.Opti()
        x = opti.variable(N+1,n_x)
        u = opti.variable(N,n_u)
        objective_sum = (x[[0],:].T-self.x_F).T @ self.Q @ (x[[0],:].T-self.x_F)
        opti.subject_to(x[[0],:].T == self.x[-1])
        x_cur_pred = self.x[-1]
        if self.prev_mpc_traj == None:
            self.prev_mpc_traj = [x_cur_pred]
            for i in range(0,N-1):
                self.prev_mpc_traj.append(np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]])@self.prev_mpc_traj[-1])
        else:
            self.prev_mpc_traj[0] = x_cur_pred
        agent_cur_pred_list = None
        if method == "constant_vel":
            agent_cur_pred_list = [agent.x[len(self.x)-1] for agent in agent_list]
        else:
            agent_cur_vel_dataset = vel_dataset
            agent_cur_state_dataset = [len(vel_dataset[agent._id])*[agent.x[len(self.x)-1]] for agent in agent_list] #
        for i in range(0,N):
            objective_sum += (x[[i+1],:].T-self.x_F ).T @ self.Q @ (x[[i+1],:].T-self.x_F) + u[[i],:] @ self.R @ u[[i],:].T
            opti.subject_to(x[[i+1],:].T == self.A @ x[[i],:].T + self.B @ u[[i],:].T)
            if not (self.H @ x[[i+1],:].T <= self.h).is_constant():
                opti.subject_to(self.H @ x[[i+1],:].T <= self.h)
            if not (self.G @ u[[i],:].T <= self.g).is_constant():
                opti.subject_to(self.G @ u[[i],:].T <= self.g)
            for agent in agent_list:
                if (agent._id == self._id):
                    continue
                #assumes agents are ordered by id (0 indexed)
                # EXPERIMENT
                if method == "constant_vel":
                    p_a = self.prev_mpc_traj[i][0:2]
                    p_b = agent_cur_pred_list[agent._id][0:2]
                    v_a = self.prev_mpc_traj[i][2:4]
                    v_b = agent_cur_pred_list[agent._id][2:4]
                    # END EXPERIMENT
                    self.add_orca_constraints(opti, x, i+1, p_a, p_b, v_a, v_b, is_neighbor_dummy = self.is_agent_dummy_list[agent._id]) # TODO add this for all neighbors and in general upgrade this
                    agent_cur_pred_list[agent._id] = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]])@agent_cur_pred_list[agent._id]
                else:
                    p_a = self.prev_mpc_traj[i][0:2]
                    v_a = self.prev_mpc_traj[i][2:4]
                    # abc = input("abc")
                    for ii in range(len(agent_cur_state_dataset[agent._id])):
                        x_b = agent_cur_state_dataset[agent._id][ii]
                        v_b = x_b[2:4]
                        p_b = x_b[0:2]
                        self.add_orca_constraints(opti, x, i+1, p_a, p_b, v_a, v_b, is_neighbor_dummy = self.is_agent_dummy_list[agent._id])
                        agent_cur_state_dataset[agent._id][ii] = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]])@x_b # NOT REALLY???
            # using previous mpc solution.
            # x_cur_pred = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]])@x_cur_pred
        
        opti.minimize(objective_sum)
        
        opti.solver('ipopt')
        sol = opti.solve()
        self.prev_mpc_traj = [sol.value(x)[[i+1],:].T for i in range(0,N)]
        # abc = input("abc")
        if N >1:
            return sol.value(u)[[0],:].T
        else:
            # print(sol.value(u))
            # print(sol.value(objective_sum))
            #DEBUG
            #EXPERIMENT
            return sol.value(u).reshape((n_u,1))

    # def find_u_orca(self, agent_list):
    #     u = self.find_u_orca_mpc(1, agent_list)
    #     return u


    def plot_circles(self, x_list, y_list, radius):
        for i in range(len(x_list)):
            self.plot_circle(x_list[i], y_list[i], radius)
    

    def plot_circle(self, x, y, radius):
        theta = np.linspace(0,2*np.pi,50)
        x1 = x+radius*np.cos(theta)
        x2 = y+radius*np.sin(theta)
        plt.plot(x1,x2, color = self.color_list[self._id], alpha=0.2)


    def plot_trajectory(self, plot_circles_flag=True):
        plt.plot([x_i[0,0] for x_i in self.x], [x_i[1,0] for x_i in self.x], label="Agent id: "+str(self._id), color=self.color_list[self._id])
        if plot_circles_flag:
            self.plot_circles([x_i[0,0] for x_i in self.x], [x_i[1,0] for x_i in self.x], self.radius)

