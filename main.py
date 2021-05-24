import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import lqr as lqr_utils
import mpc as mpc_utils
import cubic_spline_planner
import math 
import plotting_utils as plot

pi = math.pi

TARGET_SPEED = 10/3.6

class Trajectory():
    def __init__(self, ax, ay):
        self.ax = ax 
        self.ay = ay 
    
    def make_cubic_spline(self):
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        self.ax, self.ay, ds=0.1)
        return cx, cy, cyaw, ck, s

if __name__ == "__main__" :
    straightx = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    straighty = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    complexx = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
    complexy = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]

    curvex = [0, 15, 15, 0]
    curvey = [0, 0, 15, 15]

    axs = [straightx, curvex, complexx]
    ays = [straighty, curvey, complexy]

    for i in range(len(axs)):
        ax = axs[i]
        ay = ays[i]
        traj = Trajectory(ax, ay)
        cx, cy, cyaw, ck, s = traj.make_cubic_spline()

        lqr = lqr_utils.LQR(cx, cy, cyaw, ck, s, TARGET_SPEED)
        lqr.plot_traj()
        t, x, y, yaw, v, d, a, errors, th_errors = lqr.find_path()
        #lqr.show_final(ax, ay, x, y, t, errors)
        print("LQR", sum(errors), sum(th_errors))
        lx = x
        ly = y
        lerror = errors
        ltherror = th_errors
        tl = t

        mpc = mpc_utils.MPC(cx, cy, cyaw, ck, s, TARGET_SPEED)
        t, x, y, yaw, v, d, a, errors, th_errors = mpc.find_path()
        #mpc.show_final(ax, ay, x, y, t, errors)
        print("MPC", sum(errors), sum(th_errors))
        mx = x 
        my = y
        merror = errors
        mtherror = th_errors
        tm = t

        maxtl = max(tl)
        maxtm = max(tm)
        tl = [float(i)/maxtl*maxtm for i in tl]

        plot.show_both_traj(cx, cy, ax, ay, mx, my, lx, ly, t)
        plot.show_both_error(tl, tm, lerror, merror)
        plot.show_both_th_error(tl, tm, ltherror, mtherror)




    

