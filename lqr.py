
import cubic_spline_planner
import matplotlib.pyplot as plt
import numpy as np
import math
import traj_opt_utils as traj
import plotting_utils as plot

# LQR parameters
Q = np.eye(4)
R = np.eye(1)
show_animation = True

class LQR():
    def __init__(self, cx, cy, cyaw, ck, s, target_speed):
        self.cx = cx
        self.cy = cy 
        self.cyaw = cyaw 
        self.ck = ck 
        self.target_speed = target_speed
        self.L = 0.5
        self.goal = [cx[-1], cy[-1]]
        self.dt = 0.1
    
    def wrapped(self,angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def get_control(self, state, prev_error, prev_theta_error):
        x, y, th, v = state
        ind, error = traj.find_closest_neighbor(state, self.cx, self.cy, self.cyaw)
        k = self.ck[ind]

        theta_error = self.wrapped(th - self.cyaw[ind])

        A = np.array([[1, self.dt, 0, 0], 
        [0, 0, v, 0],
        [0, 0, 1, self.dt],
        [0, 0, 0, 0]])

        B = np.array([[0],
        [0],
        [0],
        [v/self.L]])

        K = traj.lqr(A, B, Q, R)

        x = np.array([[error], 
        [(error-prev_error)/self.dt], 
        [theta_error], 
        [(theta_error - prev_theta_error)/self.dt]])

        delta = math.atan2(self.L*k, 1) + self.wrapped((-K@x)[0, 0])

        return delta, ind, error, theta_error

    def find_path(self):
        T = 500 
        goal_thresh = 0.3 
        state = [0, 0, 0, 0]
        x, y, th, v = state
        t = 0 
        xs = [x]
        ys = [y]
        ths = [th]
        vs = [v]
        ts = [t]
        error = 0.0
        error_theta = 0.0
        errors = [error]
        while T>=t:
            x, y, th, v = state
            delta, ind, error, error_theta = self.get_control(state, error, error_theta)
            a = traj.PID(self.target_speed, v)
            state = traj.update_state(state, a, delta)
            new_x, new_y, new_th, new_v = state
            t += self.dt

            if abs(new_v) <= 0.05:
                ind += 1

            dist_to_goal = traj.find_dist(self.goal, state)
            if dist_to_goal <= goal_thresh:
                print("DONE")
                break

            xs.append(new_x)
            ys.append(new_y)
            ths.append(new_th)
            vs.append(new_v)
            ts.append(t)
            errors.append(error)

            if show_animation:
                plot.show_animation(self.cx, self.cy, xs, ys, new_v, ind)

        return ts, xs, ys, ths, vs, errors
    
    def plot_traj(self):
        plot.plot_traj(self.cx, self.cy)
    
    def show_final(self,ax, ay, x, y, t, error):
        plot.show_final(self.cx, self.cy, ax, ay, x, y, t, error)


