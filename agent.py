import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt
import casadi
class Agent:
    
    def __init__(self, A, B, G, g, H, h, radius, _id, x_0, Q, R, x_F):
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


    def find_norm(self):
        return np.linalg.norm(self.x[-1]-self.x_F)


    def evolve_state(self, u):
        self.x.append(self.A @ self.x[-1] + self.B @ u)
        self.u.append(u)


    def orca_update(self):
        u = self.find_u_orca()
        self.evolve_state(u)


    def orca_mpc_update(self, N):
        u = self.find_u_orca_mpc(N)
        self.evolve_state(u)


    def add_orca_constraints(self, constraints):
        pass


    def find_u_orca_mpc(self, N):
        n_x = self.A.shape[1]
        n_u = self.B.shape[1]
        opti = casadi.Opti()
        x = opti.variable(N+1,n_x)
        u = opti.variable(N,n_u)
        objective_sum = (x[[0],:].T-self.x_F).T @ self.Q @ (x[[0],:].T-self.x_F)
        opti.subject_to(x[[0],:].T == self.x[-1])
        for i in range(0,N):
            objective_sum += (x[[i+1],:].T-self.x_F).T @ self.Q @ (x[[i+1],:].T-self.x_F) + u[[i],:] @ self.R @ u[[i],:].T
            opti.subject_to(x[[i+1],:].T == self.A @ x[[i],:].T + self.B @ u[[i],:].T)
            if not (self.H @ x[[i+1],:].T <= self.h).is_constant():
                opti.subject_to(self.H @ x[[i+1],:].T <= self.h)
            if not (self.G @ u[[i],:].T <= self.g).is_constant():
                opti.subject_to(self.G @ u[[i],:].T <= self.g)
            # self.add_orca_constraints(constraints)
        
        opti.minimize(objective_sum)
        opti.solver('ipopt')
        sol = opti.solve()

        return sol.value(u)[[0],:].T


    def find_u_orca(self):
        u = self.find_u_orca_mpc(1)
        return u


    def plot_trajectory(self):
        plt.plot([x_i[0] for x_i in self.x], [x_i[1] for x_i in self.x], label="Agent id: "+str(self._id))
