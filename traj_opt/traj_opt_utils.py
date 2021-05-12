import numpy as np
import math

DT = 0.1 
LF = 0.5 
MAX_STEER = np.deg2rad(45)

def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def update_state(state, a, delta):
    x, y, th, v = state 
    delta = np.clip(delta, -MAX_STEER, MAX_STEER)

    x = x + v*math.cos(th)*DT
    y = y + v*math.sin(th)*DT
    th = th + v/LF*math.tan(delta)*DT
    v = v + a * DT

    return [x, y, th, v]

def find_dist(goal, state):
    x, y, _ , _= state
    dx = x - goal[0]
    dy = y - goal[1]
    return math.hypot(dx, dy)

def solve_DARE(A, B, Q, R):
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

def lqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = np.linalg.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

    return K

def PID(target, current):
    return target-current

