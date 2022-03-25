import numpy as np
import math
import scipy.interpolate as scipy_interpolate
from utils import angle_of_line


class PathPlanning:
    def __init__(self, obstacles, env):
        self.env = env

        self.R_Elmin = self.env.a / np.tan(self.env.steer_max)
        # self.R_Elmin = 10
        print("self.R_Elmin: ", self.R_Elmin)
        self.R_Bl_min = np.linalg.norm([self.R_Elmin + self.env.b + self.env.d_r,
                                        self.env.a + self.env.d_front], ord=2)
        print("self.R_Bl_min: ", self.R_Bl_min)
        # print(np.sqrt((self.R_Elmin + self.env.b + self.env.d_r)**2 + (self.env.a + self.env.d_front)**2))
        self.L_min = self.env.d_rear + np.sqrt(self.R_Bl_min**2 - (self.env.a / np.tan(self.env.steer_max) - self.env.b - self.env.d_l)**2)
        print("L_min: ", self.L_min)

        self.margin = 5
        # sacale obstacles from env margin to pathplanning margin
        # obstacles = obstacles + np.array([self.margin, self.margin])

        # self.obs = np.concatenate([np.array([[0, i] for i in range(100 + self.margin)]),
        #                           np.array([[100 + 2 * self.margin, i] for i in range(100 + 2 * self.margin)]),
        #                           np.array([[i, 0] for i in range(100 + self.margin)]),
        #                           np.array([[i, 100 + 2 * self.margin] for i in range(100 + 2 * self.margin)]),
        #                           obstacles])

        # self.ox = [int(item) for item in self.obs[:, 0]]
        # self.oy = [int(item) for item in self.obs[:, 1]]

    def plan_path(self, sx, sy, sphi, gx, gy):
        # path = np.array([np.linspace(sx, gx, 50), np.linspace(sy, gy, 50)])

        C_l = np.array([gx, gy + self.R_Elmin])
        C_l2E_init = np.linalg.norm([sx, sy] - C_l, ord=2)
        print("C_l2E_init: ", C_l2E_init)
        alpha = np.pi / 2 + sphi + np.arcsin((C_l[1] - sy) / C_l2E_init)
        print("alpha: ", alpha)
        R_E_init_r = (C_l2E_init**2 - self.R_Elmin**2) \
            / (2 * (self.R_Elmin + C_l2E_init * np.cos(alpha)))
        print("R_E_init_r: ", R_E_init_r)
        delta_r = np.arctan(self.env.a / R_E_init_r)
        C_r = np.array([sx + R_E_init_r * np.sin(sphi),
                       sy - R_E_init_r * np.cos(sphi)])
        print("C_r: ", C_r)

        beta = np.arccos(
            (R_E_init_r**2 + (R_E_init_r + self.R_Elmin)**2 - C_l2E_init**2)
            / (2 * R_E_init_r * (R_E_init_r + self.R_Elmin)))

        point_interval = 0.5
        path = [[sx, sy]]
        theta = 0
        while theta < beta:
            t = np.pi / 2 + sphi + theta
            p_current = C_r + R_E_init_r * np.array([np.cos(t), np.sin(t)])
            distance = np.linalg.norm([path[-1] - p_current])
            if distance > point_interval:
                path.append(p_current)
            theta += 0.01
            # path.append(C_r + (R_E_init_r * np.array([np.cos(t), np.sin(t)])) - [0, 1])p

            # print(C_r + R_E_init_r * np.array([np.cos(t), np.sin(t)]))

        # for theta in np.linspace(beta + sphi, 0, 30):
        theta = beta + sphi
        while theta > 0:
            t = -np.pi / 2 + theta
            p_current = C_l + self.R_Elmin * np.array([np.cos(t), np.sin(t)])
            distance = np.linalg.norm([path[-1] - p_current])
            if distance > point_interval:
                path.append(p_current)
            theta -= 0.01


        return np.array(path)
