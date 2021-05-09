
import cubic_spline_planner
import matplotlib.pyplot as plt
import numpy as np
import math

# LQR parameter
Q = np.eye(4)
R = np.eye(1)
show_animation = True
max_steer = np.deg2rad(45)

class LQR():
    def __init__(self, cx, cy, cyaw, ck, s, target_speed):
        self.cx = cx
        self.cy = cy 
        self.cyaw = cyaw 
        self.ck = ck 
        self.s = s
        self.target_speed = target_speed
        self.L = 0.5
        self.goal = [cx[-1], cy[-1]]
        self.dt = 0.1
    
    def update_state(self, state, a, delta):
        x, y, th, v = state 
        delta = np.clip(delta, -max_steer, max_steer)

        x = x + v*math.cos(th)*self.dt 
        y = y + v*math.sin(th)*self.dt
        th = th + v/self.L*math.tan(delta)*self.dt 
        v = v + a * self.dt
        return [x, y, th, v]

    def find_dist(self, state):
        x, y, _ , _= state
        dx = x - self.goal[0]
        dy = y-self.goal[1]
        return math.hypot(dx, dy)
    
    def wrapped(self,angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def find_closest_neighbor(self, state):
        x, y = state[0:2]
        dx = [x -icx for icx in self.cx]
        dy = [y-icy for icy in self.cy]
        dists  = [math.sqrt(idx**2 +idy**2) for (idx, idy) in zip(dx, dy)]
        minval = min(dists)
        ind = dists.index(minval)

        diff_x = self.cx[ind] - x 
        diff_y = self.cy[ind] - y

        angle = self.cyaw[ind] - math.atan2(diff_y, diff_x)
        angle = self.wrapped(angle)

        minval *= np.sign(angle)
        return ind, minval
    
    def solve_DARE(self, A, B, Q, R):
        """
        solve a discrete time_Algebraic Riccati equation (DARE)
        """
        X = Q
        maxiter = 150
        eps = 0.01

        for i in range(maxiter):
            Xn = A.T @ X @ A - A.T @ X @ B @ \
                np.linalg.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q
            if (abs(Xn - X)).max() < eps:
                break
            X = Xn

        return Xn
    
    def lqr(self, A, B, Q, R):
        """Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        # ref Bertsekas, p.151
        """

        # first, try to solve the ricatti equation
        X = self.solve_DARE(A, B, Q, R)

        # compute the LQR gain
        K = np.linalg.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

        return K

    def PID(self, target, current):
        return target-current
    
    def get_control(self, state, prev_error, prev_theta_error):
        x, y, th, v = state
        ind, error = self.find_closest_neighbor(state)
        k = self.ck[ind]

        theta_error = self.wrapped(th - self.cyaw[ind])

        A = np.array([[1, self.dt, 0, 0], 
        [0, 0, self.dt, v],
        [0, 0, 1, self.dt],
        [0, 0, 0, 0]])
        A = np.zeros((4, 4))
        A[0, 0] = 1.0
        A[0, 1] = self.dt
        A[1, 2] = v
        A[2, 2] = 1.0
        A[2, 3] = self.dt

        B = np.array([[0],[0],[0],[v/self.L]])
        B = np.zeros((4, 1))
        B[3, 0] = v / self.L

        K = self.lqr(A, B, Q, R)

        x = np.array([[error], [(error-prev_error)/self.dt], [theta_error], [(theta_error - prev_theta_error)/self.dt]])

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
            a = self.PID(self.target_speed, v)
            state = self.update_state(state, a, delta)
            new_x, new_y, new_th, new_v = state
            t += self.dt

            if abs(new_v) <= 0.05:
                ind += 1

            dist_to_goal = self.find_dist(state)
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
                self.show_animation(xs, ys, new_v, ind)

        return ts, xs, ys, ths, vs, errors
    
    def plot_traj(self):
        plt.plot(self.cx, self.cy, "-r", label="spline")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()
        plt.show()
    
    def show_animation(self, x, y, v, target_ind):
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(self.cx, self.cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(self.cx[target_ind], self.cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[km/h]:" + str(round(v * 3.6, 2))
                      + ",target index:" + str(target_ind))
            plt.pause(0.0001)
    
    def show_final(self,ax, ay, x, y, t, error):
        plt.close()
        plt.subplots(1)
        plt.plot(ax, ay, "xb", label="input")
        plt.plot(self.cx, self.cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()
        plt.subplots(1)
        plt.plot(t, error, "-r", label="error")
        plt.grid(True)
        plt.legend()
        plt.xlabel("time[s]")
        plt.ylabel("error [1/m]")

        plt.show()

ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]

cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
    ax, ay, ds=0.1)
target_speed = 10.0 / 3.6  # simulation parameter km/h -> m/s

lqr = LQR(cx, cy, cyaw, ck, s, target_speed)
t, x, y, yaw, v, errors = lqr.find_path()
lqr.show_final(ax, ay, x, y, t, errors)

