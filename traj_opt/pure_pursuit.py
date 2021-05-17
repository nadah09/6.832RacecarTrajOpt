import numpy as np
import math
import cubic_spline_planner
import matplotlib.pyplot as plt

# Parameters
k = 0.1  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 2.9  # [m] wheel base of vehicle
max_steer = np.deg2rad(45)

class State():
    def __init__(self, x, y, yaw, v):
        self.x = x
        self.y = y
        self.yaw = yaw 
        self.v = v


def update(state, a, delta):
    if delta >= max_steer:
        delta = max_steer
    if delta <= - max_steer:
        delta = - max_steer
    
    x, y, yaw, v = state
    x = x + v*math.cos(yaw)*dt
    y = y + v*math.sin(yaw)*dt
    yaw = yaw + v/Lfc *math.tan(delta)*dt 
    v = v + a*dt

    return [x, y, yaw, v]    



class PurePursuit():
    def __init__(self, cx, cy, target_speed):
        self.cx = cx 
        self.cy = cy
        self.sp = target_speed
    
    def find_next(self, i):
        return self.cx[i+1], self.cy[i+1]
    
    def calc_nearest_index(state):
        x, y, yaw, v = state
        dx = [x - icx for icx in cx]
        dy = [y - icy for icy in cy]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind)

        mind = math.sqrt(mind)

        dxl = self.cx[ind] - x
        dyl = self.cy[ind] - y
    
        Lf = k * self.sp + Lfc  # update look ahead distance

        return ind, Lf
    

    
    
    def pure_pursuit(self):
        a = 0
        goal = [self.cx[-1], self.cy[-1]]
        t = 0
        T = 100
        state = [self.cx[0], self.cy[0], 0, self.sp]
        
        while T >= t:
            delta = 0
            state = update(state, a, delta)
            self.plot_animation(state[0], state[1])
            pass 
    
    def plot_animation(self, x, y):
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(self.cx, self.cy, "-r", label="course")
        plt.plot(x, y, "ob", label="trajectory")
        plt.plot(self.cx[target_ind], self.cy[target_ind], "xg", label="target")
        plt.axis("equal")
        plt.grid(True)
        plt.title("speed[km/h]:" + str(round(self.sp * 3.6, 2))
                    + ",target index:" + str(target_ind))
        plt.pause(0.0001)
    


def main():
    ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
    goal = [ax[-1], ay[-1]]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)
    target_speed = 10.0 / 3.6  # simulation parameter km/h -> m/s
    plt.plot(cx, cy, "-r", label="spline")
    plt.show()

    pp = PurePursuit(cx, cy, target_speed)
    pp.pure_pursuit()


main()