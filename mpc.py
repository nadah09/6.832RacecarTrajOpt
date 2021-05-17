import numpy as np
import math
import cubic_spline_planner
import cvxpy
import traj_opt_utils as traj
import plotting_utils as plot

NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 500  # horizon length

TH = 5
L = 0.5 
TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
DT = 0.1
show_animation = True
dl = 1.0  # course tick


class MPC():
    def __init__(self, cx, cy, cyaw, ck, s, sp):
        self.cx = cx
        self.cy = cy 
        self.cyaw = self.smooth_yaw(cyaw)
        self.ck = ck 
        self.s = s
        self.goal = [cx[-1], cy[-1]]
    
    def smooth_yaw(self, yaw):
        for i in range(len(yaw) - 1):
            dyaw = yaw[i + 1] - yaw[i]

            while dyaw >= math.pi / 2.0:
                yaw[i + 1] -= math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

            while dyaw <= -math.pi / 2.0:
                yaw[i + 1] += math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

        return yaw
    
    def calc_ref_trajectory(self, state, dl, pind):
        xref = np.zeros((NX, T + 1))
        dref = np.zeros((1, T + 1))
        ncourse = len(self.cx)

        ind, _ = traj.find_closest_neighbor(state, self.cx, self.cy, self.cyaw)

        if pind >= ind:
            ind = pind

        xref[0, 0] = self.cx[ind]
        xref[1, 0] = self.cy[ind]
        xref[2, 0] = TARGET_SPEED
        xref[3, 0] = self.cyaw[ind]
        dref[0, 0] = 0.0  # steer operational point should be 0

        travel = 0.0
        x, y, th, v = state

        for i in range(T + 1):
            travel += abs(v) * DT
            dind = int(round(travel / dl))

            if (ind + dind) < ncourse:
                xref[0, i] = self.cx[ind + dind]
                xref[1, i] = self.cy[ind + dind]
                xref[2, i] = TARGET_SPEED
                xref[3, i] = self.cyaw[ind + dind]
                dref[0, i] = 0.0
            else:
                xref[0, i] = self.cx[ncourse - 1]
                xref[1, i] = self.cy[ncourse - 1]
                xref[2, i] = TARGET_SPEED
                xref[3, i] = self.cyaw[ncourse - 1]
                dref[0, i] = 0.0

        return xref, ind, dref
    
    def find_path(self):
        state = [0, 0, 0, 0]
        x, y, th, v = state
        goal_thresh = 0.3

        t = 0 
        xs = [x]
        ys = [y]
        ths = [th]
        vs = [v]
        ts = [0]
        ds = [0]
        acs = [0]
        target_ind, _ = traj.find_closest_neighbor(state, self.cx, self.cy, self.cyaw)
        odelta, oa = None, None 

        while T>=t:
            xref, target_ind, dref = self.calc_ref_trajectory(state, dl, target_ind)
            x0 = [x, y, v, th]
            oa, odelta, ox, oy, oyaw, ov = traj.iterative_linear_mpc_control(
                xref, x0, dref, oa, odelta)

            if odelta is not None:
                di, ai = odelta[0], oa[0]
            

            state = traj.update_state(state, ai, di)
            t = t + DT
            x, y, th, v = state
        
            xs.append(x)
            ys.append(y)
            ths.append(th)
            vs.append(v)
            ts.append(t)
            ds.append(di)
            acs.append(ai)

            dist_to_goal = traj.find_dist(self.goal, state)
            if dist_to_goal <= goal_thresh:
                print("DONE")
                break
            
            if show_animation:
                plot.show_animation(self.cx, self.cy, xs, ys, v, target_ind)
        return ts, xs, ys, ths, vs, ds, acs

    def plot_traj(self):
        plot.plot_traj(self.cx, self.cy)

    def show_final(self,ax, ay, x, y, t, error):
        plot.show_final(self.cx, self.cy, ax, ay, x, y, t, error)

"""
dl = 1.0  # course tick
ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]

cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
    ax, ay, ds=0.1)
mpc = MPC(cx, cy, cyaw, ck, s)
t, x, y, yaw, v, d, a = mpc.find_path()
mpc.show_final(ax, ay, x, y, t, d)    
"""