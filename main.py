import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import dynamics_utils as dynamics
from generate_track import OccupancyGrid
import traj

class TrajOpt():
    def __init__(self, size, obs):
        self.size = size
        self.obs = obs
        self.traj = []
        self.occ_grid = OccupancyGrid(self.size, self.obs)
        self.occ_grid.generate_occupancy_grid()
        self.start = self.occ_grid.get_start()
        self.end = self.occ_grid.get_end()
    
    def create_trajectory(self, method):
        if method == "astar":
            self.run_a_star()
        else:
            print("Running A Star")
            self.run_a_star()
    
    def run_a_star(self):
        path = traj.a_star(self.occ_grid, self.start, self.end)
        self.traj = path
    
    def plot_grid(self):
        self.occ_grid.plot_grid()
    
    def plot_traj(self):
        self.occ_grid.plot_trajectory(self.traj)

    




if __name__ == "__main__" :
    size = 50
    obs = 150
    trajOpt = TrajOpt(size, obs)
    trajOpt.plot_grid()
    trajOpt.create_trajectory("astar")
    trajOpt.plot_traj()

