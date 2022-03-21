import cv2
import numpy as np
from time import sleep
import argparse

from environment import Environment, Parking1
from control import Car_Dynamics, MPC_Controller, Linear_MPC_Controller
from pathplanning import PathPlanning

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--x_start', type=int, default=80, help='X of start')
    parser.add_argument('--y_start', type=int, default=50, help='Y of start')
    parser.add_argument('--psi_start', type=int, default=0, help='psi of start')
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
    key = cv2.waitKey(1)

    path_planner = PathPlanning(obs)
    path = path_planner.plan_path(int(start[0]),int(start[1]),int(end[0]),int(end[1]))
    env.draw_path(path)

    for i,point in enumerate(path):
        acc, delta = controller.optimize(my_car, path[i:i+MPC_HORIZON])
        my_car.update_state(my_car.move(acc,  delta))
        res = env.render(my_car.x, my_car.y, my_car.psi, delta)
        cv2.imshow('environment', res)
        key = cv2.waitKey(1)
        if key == ord('s'):
            cv2.imwrite('res.png', res*255)

    # zeroing car steer
    res = env.render(my_car.x, my_car.y, my_car.psi, 0)
    cv2.imshow('environment', res)
    key = cv2.waitKey()
    #############################################################################################

    cv2.destroyAllWindows()
