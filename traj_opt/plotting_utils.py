import matplotlib.pyplot as plt

def plot_traj(cx, cy):
    plt.plot(cx, cy, "-r", label="spline")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()
    plt.show()

def show_animation(cx, cy, x, y, v, target_ind):
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(cx, cy, "-r", label="course")
        plt.plot(x, y, "ob", label="trajectory")
        plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
        plt.axis("equal")
        plt.grid(True)
        plt.title("speed[km/h]:" + str(round(v * 3.6, 2))
                    + ",target index:" + str(target_ind))
        plt.pause(0.0001)
    
def show_final(cx, cy, ax, ay, x, y, t, error):
    plt.close()
    plt.subplots(1)
    plt.plot(ax, ay, "xb", label="input")
    plt.plot(cx, cy, "-r", label="spline")
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