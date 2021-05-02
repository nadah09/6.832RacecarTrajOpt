import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import dynamics_utils as dynamics
from generate_track import OccupancyGrid

class TrajOpt():
    def __init__(self, size, obs):
        self.size = size
        self.obs = obs
        self.traj = []
        self.occ_grid = OccupancyGrid(self.size, self.obs)
        self.occ_grid.generate_occupancy_grid()
    
    def create_trajectory(self, method):
        if method == "astar":
            self.run_a_star()
        elif method == "RRT":
            self.run_RRT()
    
    def run_a_star(self):
        pass
    
    def run_RRT(self):
        pass
    




if __name__ == "__main__" :
    size = 20
    obs = 15
    trajOpt = TrajOpt(size, obs)
    trajOpt.occ_grid.plot_grid()

