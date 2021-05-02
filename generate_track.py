import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors

class OccupancyGrid():
    def __init__(self, size, num_obstacles):
        self.size = size
        self.num_obstacles = num_obstacles
        self.occ_grid = np.zeros((size, size))
        self.obstacles = set()
        self.generate_obstacles()
        self.start = (0, 0)
        self.end = (0, 0)
        self.traj_occ_grid = np.copy(self.occ_grid)
    
    def is_obstacle(self, x, y):
        return self.occ_grid[x, y] == 1
    
    def is_start(self, x, y):
        return self.occ_grid[x, y] == 2
    
    def is_end(self, x, y):
        return self.occ_grid[x, y] == 3
    
    def get_start(self):
        return self.start
    
    def get_end(self):
        return self.end
    
    def generate_obstacles(self):
        while len(self.obstacles) < self.num_obstacles:
            self.obstacles.add((np.random.randint(self.size), np.random.randint(self.size)))

    def generate_occupancy_grid(self):
        for obs in self.obstacles:
            x, y = obs
            self.occ_grid[x, y] = 1
        self.find_start_and_end()
    
    def get_occ_grid(self):
        return self.occ_grid
    
    def find_start_and_end(self):
        for i in range(2):
            x, y = [np.random.randint(self.size), np.random.randint(self.size)]
            while self.occ_grid[x, y] !=0:
                x, y = [np.random.randint(self.size), np.random.randint(self.size)]
            if i == 0:
                self.start = (x, y)
            else:
                self.end = (x, y)
            self.occ_grid[x, y] = 2 + i
    
    def get_neighbors(self, pos):
        x = pos[0]
        y = pos[1]
        neighbors = []
        for i in range(x-1, x+2):
            for j in range(y-1, y+2):
                if i >= 0 and i < self.size and j >= 0 and j < self.size and self.occ_grid[i, j] != 1:
                    neighbors.append((i, j))
        return neighbors

    def plot_grid(self):
        cmap = colors.ListedColormap(['white','black', 'green', 'red'])
        plt.figure(figsize=(10, 10))
        plt.pcolor(self.occ_grid[::-1],cmap=cmap,edgecolors='k', linewidths=0)
        plt.show()
    
    def add_traj(self):
        for node in self.traj:
            if self.occ_grid[node[0], node[1]] == 0:
                self.traj_occ_grid[node[0], node[1]] = 4

    def reset_traj(self):
        self.traj_occ_grid = np.copy(self.occ_grid)
    
    def plot_trajectory(self, traj):
        self.traj_occ_grid = np.copy(self.occ_grid)
        self.traj = traj
        self.add_traj()
        cmap = colors.ListedColormap(['white','black', 'green', 'red', 'blue'])
        plt.figure(figsize=(10, 10))
        plt.pcolor(self.traj_occ_grid[::-1],cmap=cmap,edgecolors='k', linewidths=0)
        plt.show()

    
    def animate_process(self):
        pass


