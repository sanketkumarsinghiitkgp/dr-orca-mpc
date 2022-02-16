import numpy as np
from agent import Agent
import matplotlib.pyplot as plt

A = np.array([[1, 0, 1, 0],[0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]])
B = np.array([[0, 0],[0, 0],[1, 0],[0, 1]])
Q = np.diag([1, 1, 0.1, 0.1])
R = np.diag([0.1,0.1])
N = 5
H = np.zeros((4,4))
h = np.zeros((4,1))
G = np.zeros((2,2))
g = np.zeros((2,1))
x_F = np.array([[5],[5],[0],[0]])
agent = Agent(A, B, G, g, H, h, radius=1, _id=1, x_0=np.zeros((4,1)), Q=Q, R=R, x_F=x_F)

for i in range(10):
    agent.orca_mpc_update(N, [])

agent.plot_trajectory()
plt.legend()
plt.savefig("agent_trajectory.png")
