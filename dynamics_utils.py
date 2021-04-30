# python libraries
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

class Dynamics:
	def __init__(self):
		self.n_x = 5
		self.n_u = 2
	def car_continuous_dynamics(x, u):
	    # x = [x position, y position, heading, speed, steering angle] 
	    # u = [acceleration, steering velocity]
	    m = sym if x.dtype == object else np # Check type for autodiff
	    heading = x[2]
	    v = x[3]
	    steer = x[4]
	    x_d = np.array([
	        v*m.cos(heading),
	        v*m.sin(heading),
	        v*m.tan(steer),
	        u[0],
	        u[1]        
	    ])
	    return x_d

	def discrete_dynamics(x, u):
	    dt = 0.1
	    x_next = x + dt * self.car_continuous_dynamics(x, u)
	    return x_next

	def rollout(x0, u_trj):
	    x_trj = np.zeros((u_trj.shape[0]+1, x0.shape[0]))
	    x_trj[0] = x0
	    # TODO: Define the rollout here and return the state trajectory x_trj: [N, number of states]
	    for i in range(1, u_trj.shape[0] + 1):
	      x_trj[i] = self.discrete_dynamics(x_trj[i - 1], u_trj[i - 1])
	    return x_trj
