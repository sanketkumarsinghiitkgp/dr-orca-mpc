import cvxpy as cp
import numpy as np


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

    
    def evolve_state(u):
        self.x.append(self.A@self.x[-1]+self.B*u)
        self.u.append(u)


    def orca_update():
        u = self.find_u_orca()
        self.evolve_state(u)


    def orca_mpc_update():
        u = self.find_u_orca_mpc()
        self.evolve_state(u)


    def find_u_orca_mpc(N):
        n_x = self.A.shape[1]
        n_u = self.B.shape[1]
        x = cp.Variable((N+1,n_x))
        u = cp.Variable((N,n_u))
        objective_sum = (x[[0],:].T-x_F).T @ self.Q @ (x[[0],:].T-x_F)
        constraints = []
        for i in range(0,N):
            objective_sum += (x[[i],:].T-x_F).T @ self.Q @ (x[[i],:].T-x_F)
            constraints.append(x[[i+1],:].T == self.A @ x[[i],:].T + self.B @ u[[i],:].T)
            constraints.append(self.H @ x[[i+1],:].T <= self.h)
            constraints.append(self.G @ u[[i],:].T <= self.g)
            # TODO add ORCA constraints
        objective = cp.Minimize(objective_sum)
        prob = cp.Problem(objective, constraints)
        prob.solve()
        return u[[0],:].T

    def find_u_orca():
        u = find_u_orca_mpc(1)
        return u
