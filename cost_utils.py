import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

class Cost():
	def __init__(self):
		self.r = 2.0
		self.v_target = 2.0
		self.eps = 1e-6

	v_target = 2.0
	eps = 1e-6 # The derivative of sqrt(x) at x=0 is undefined. Avoid by subtle smoothing
	def cost_stage(x, u):
	    m = sym if x.dtype == object else np # Check type for autodiff
	    c_circle = (m.sqrt(x[0]**2 + x[1]**2 + self.eps) - self.r)**2
	    c_speed = (x[3]-self.v_target)**2
	    c_control= (u[0]**2 + u[1]**2)*0.1
	    return c_circle + c_speed + c_control

	def cost_final(x):
	    m = sym if x.dtype == object else np # Check type for autodiff
	    c_circle = (m.sqrt(x[0]**2 + x[1]**2 + self.eps) - self.r)**2
	    c_speed = (x[3]-self.v_target)**2
	    return c_circle + c_speed

	def cost_trj(x_trj, u_trj):
	    total = 0.0
	    total = sum([cost_stage(x_trj[i, :], u_trj[i, :]) for i in range(x_trj.shape[0]-1)])
	    total += cost_final(x_trj[-1, :])
	    return total