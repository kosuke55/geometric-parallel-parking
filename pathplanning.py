import numpy as np
import math
import scipy.interpolate as scipy_interpolate
from utils import angle_of_line


class PathPlanning:
    def __init__(self, obstacles):

        self.margin = 5
        # sacale obstacles from env margin to pathplanning margin
        obstacles = obstacles + np.array([self.margin, self.margin])

        self.obs = np.concatenate([np.array([[0, i] for i in range(100 + self.margin)]),
                                  np.array([[100 + 2 * self.margin, i] for i in range(100 + 2 * self.margin)]),
                                  np.array([[i, 0] for i in range(100 + self.margin)]),
                                  np.array([[i, 100 + 2 * self.margin] for i in range(100 + 2 * self.margin)]),
                                  obstacles])

        self.ox = [int(item) for item in self.obs[:, 0]]
        self.oy = [int(item) for item in self.obs[:, 1]]
        self.grid_size = 1
        self.robot_radius = 4

    def plan_path(self, sx, sy, gx, gy):
        path = np.array([np.linspace(sx, gx, 50), np.linspace(sy, gy, 50)])
        return path.T
