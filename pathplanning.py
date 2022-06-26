import numpy as np
import math
import scipy.interpolate as scipy_interpolate
from utils import angle_of_line


class PathPlanning:
    def __init__(self, obstacles, car, parking):
        self.car = car
        self.parking = parking
        self.R_Elmin = self.car.a / np.tan(self.car.steer_max)
        self.R_Ermin = self.R_Elmin
        self.R_Bl_min = np.linalg.norm([self.R_Elmin + self.car.b + self.car.d_r,
                                        self.car.a + self.car.d_front], ord=2)

        # equation (2) in the papaer is wrong
        self.R_Ar_min = np.linalg.norm(
            [self.R_Ermin - self.car.b - self.car.d_r, self.car.d_rear], ord=2)
        self.R_Jr_min = np.linalg.norm(
            [self.R_Ermin + self.car.b + self.car.d_l, self.car.d_rear], ord=2)
        self.L_min = self.car.d_rear + np.sqrt(self.R_Bl_min**2 - (
            self.R_Elmin - self.car.b - self.car.d_l)**2)

    def plan_path(self, sx, sy, sphi, gx, gy,
                  last_backward_length
                  ):
        if self.parking.parking_length > self.L_min + \
                last_backward_length + self.parking.parking_margin * 2:
            path, steer = self.plan_path_one_traial(
                sx, sy, sphi, gx, gy, last_backward_length)
        else:
            path, steer = self.plan_path_several_traial(
                sx, sy, sphi, gx, gy, last_backward_length)
        return path, steer

    def plan_path_one_traial(self, sx, sy, sphi, gx, gy,
                             last_backward_length, Cl=None, psi_last=0
                             ):
        if Cl is None:
            Cl = np.array([gx, gy + self.R_Elmin])
        d_Cl_Einit = np.linalg.norm([sx, sy] - Cl, ord=2)
        alpha = np.pi / 2 + sphi + np.arcsin((Cl[1] - sy) / d_Cl_Einit)
        R_Einit_r = (d_Cl_Einit**2 - self.R_Elmin**2) \
            / (2 * (self.R_Elmin + d_Cl_Einit * np.cos(alpha)))
        delta_r = np.arctan(self.car.a / R_Einit_r)
        Cr = np.array([sx + R_Einit_r * np.sin(sphi),
                       sy - R_Einit_r * np.cos(sphi)])

        beta = np.arccos(
            (R_Einit_r**2 + (R_Einit_r + self.R_Elmin)**2 - d_Cl_Einit**2)
            / (2 * R_Einit_r * (R_Einit_r + self.R_Elmin)))

        point_interval = 0.25
        path = [[sx, sy]]
        steer = [0]

        # turn right
        theta = np.pi / 2 + sphi
        while theta < np.pi / 2 + sphi + beta:
            p_current = Cr + R_Einit_r * \
                np.array([np.cos(theta), np.sin(theta)])
            distance = np.linalg.norm([path[-1] - p_current])
            if distance > point_interval:
                path.append(p_current)
                steer.append(-np.arctan(self.car.a / R_Einit_r))
            theta += 0.01

        # turn left
        theta = beta + sphi - np.pi / 2
        theta_last = -np.pi / 2 + psi_last
        while theta > theta_last:
            p_current = Cl + self.R_Elmin * \
                np.array([np.cos(theta), np.sin(theta)])
            distance = np.linalg.norm([path[-1] - p_current])
            if distance > point_interval:
                path.append(p_current)
                steer.append(self.car.steer_max)
            theta -= 0.01

        return np.array(path), steer

    def plan_path_several_traial(self, sx, sy, sphi, gx, gy,
                                 last_backward_length
                                 ):
        Cl = np.array([gx, gy + self.R_Elmin])
        x_F = self.parking.cars[1][0] - \
            self.car.car_length / 2 - self.parking.parking_margin
        d_Cl_F = x_F - Cl[0]

        point_interval = 0.1
        path = [[gx, gy]]
        steer = [0]
        psi = 0  # goal psi is 0
        while d_Cl_F < self.R_Bl_min + last_backward_length:
            B = np.array([gx + self.car.a + self.car.d_front,
                          gy - self.car.b - self.car.d_r])
            self.B = B
            y_F1 = Cl[1] - np.sqrt(self.R_Bl_min**2 - (x_F - Cl[0])**2)
            F1 = np.array([x_F, y_F1])
            self.F1 = F1
            theta_l = self.al_kashi(Cl, B, F1)

            # turn left
            theta = -np.pi / 2 + psi
            while theta < -np.pi / 2 + psi + theta_l:
                p_current = Cl + self.R_Elmin * \
                    np.array([np.cos(theta), np.sin(theta)])
                distance = np.linalg.norm([path[-1] - p_current])
                if distance > point_interval:
                    path.append(p_current)
                    steer.append(self.car.steer_max)
                theta += 0.01
            psi = theta + np.pi / 2

            Cr = np.array([gx, gy - self.R_Ermin])
            Cr = self.transform(Cr, Cl, theta_l)
            self.Cr = Cr
            J = np.array([gx - self.car.d_rear,
                          gy + self.car.b + self.car.d_l])
            J = self.transform(J, Cl, theta_l)
            x_G = self.parking.cars[0][0] + \
                self.car.car_length / 2 + self.parking.parking_margin
            y_G2 = Cr[1] + np.sqrt(self.R_Jr_min**2 - (x_G - Cr[0])**2)

            G2 = np.array([x_G, y_G2])
            self.G2 = G2
            theta_r = self.al_kashi(Cr, J, G2)

            theta = np.pi / 2 + psi
            while theta < np.pi / 2 + psi + theta_r:
                p_current = Cr + self.R_Ermin * \
                    np.array([np.cos(theta), np.sin(theta)])
                distance = np.linalg.norm([path[-1] - p_current])
                if distance > point_interval:
                    path.append(p_current)
                    steer.append(-self.car.steer_max)
                theta += 0.01
            psi = theta - np.pi / 2

            Cl = self.transform(Cl, Cr, theta_r)
            self.Cl = Cl
            d_Cl_F = x_F - Cl[0]
        retrieve_path, retrieve_steer = self.plan_path_one_traial(
            sx, sy, sphi, gx, gy, last_backward_length, Cl, psi)

        path = np.vstack((retrieve_path, path[::-1]))
        steer = np.hstack((retrieve_steer, steer[::-1]))

        return path, steer

    def al_kashi(self, A, B, C):
        "calc alpha (the angle of A)"
        a = np.linalg.norm(B - C, ord=2)
        b = np.linalg.norm(A - C, ord=2)
        c = np.linalg.norm(A - B, ord=2)
        alpha = np.arccos((b**2 + c**2 - a**2) / (2 * b * c))
        return alpha

    def transform(self, p, C, theta):
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        C_new = C[:, None] + R @ (p - C)[:, None]
        return C_new.T[0]

    def get_vehicle_center_path(self, path):
        # transform bese_link points to vehicle_center points
        vehicle_center_path = []
        for i in range(len(path)):
            if i < len(path) - 1:
                v = path[i + 1] - path[i]
                psi = np.arctan2(v[1], v[0])
            a = 4.0  # todo
            x = path[i][0] + a / 2 * np.cos(psi)
            y = path[i][1] + a / 2 * np.sin(psi)
            vehicle_center_path.append([x, y])

        return vehicle_center_path
