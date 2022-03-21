import cv2
import numpy as np
from time import sleep
import argparse

from environment import Environment, Parking1
from control import Car_Dynamics, MPC_Controller, Linear_MPC_Controller

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--x_start', type=int, default=50, help='X of start')
    parser.add_argument('--y_start', type=int, default=50, help='Y of start')
    parser.add_argument('--psi_start', type=int, default=10, help='psi of start')
    parser.add_argument('--x_end', type=int, default=90, help='X of end')
    parser.add_argument('--y_end', type=int, default=80, help='Y of end')

    args = parser.parse_args()

    start = np.array([args.x_start, args.y_start])
    end   = np.array([args.x_end, args.y_end])

    parking1 = Parking1(1)
    end, obs = parking1.generate_obstacles()

    env = Environment(obs)
    my_car = Car_Dynamics(start[0], start[1], 0, np.deg2rad(args.psi_start), length=4, dt=0.2)

    MPC_HORIZON = 5
    controller = MPC_Controller()

    res = env.render(my_car.x, my_car.y, my_car.psi, 0)
    cv2.imshow('environment', res)
    key = cv2.waitKey()
