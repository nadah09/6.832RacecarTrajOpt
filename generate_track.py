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
    
    def is_obstacle(self, x, y):
        return self.occ_grid[x, y] == 1
    
    def is_start(self, x, y):
        return self.occ_grid[x, y] == 2
    
    def is_end(self, x, y):
        return self.occ_grid[x, y] == 3
    
    def generate_obstacles(self):
        while len(self.obstacles) < self.num_obstacles:
            self.obstacles.add((np.random.randint(self.size), np.random.randint(self.size)))
        return

    def generate_occupancy_grid(self):
        for obs in self.obstacles:
            print(obs)
            x, y = obs
            self.occ_grid[x, y] = 1
        self.find_start_and_end()
    
    def get_occ_grid(self):
        return self.occ_grid

    def plot_grid(self):
        cmap = colors.ListedColormap(['white','black', 'green', 'red'])
        plt.figure(figsize=(10, 10))
        plt.pcolor(self.occ_grid[::-1],cmap=cmap,edgecolors='k', linewidths=0)
        plt.show()

    def find_start_and_end(self):
        for i in range(2):
            x, y = [np.random.randint(self.size), np.random.randint(self.size)]
            while self.occ_grid[x, y] !=0:
                x, y = [np.random.randint(self.size), np.random.randint(self.size)]
            self.start = (x, y)
            self.occ_grid[x, y] = 2 + i