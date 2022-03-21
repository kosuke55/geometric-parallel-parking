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

    # in 1000 coordinates
    car_length = 80
    car_width = 40
    wheel_length = 15
    wheel_width = 7
    # [0,0] is car's center
    wheel_positions = np.array([[25, 15], [25, -15], [-25, 15], [-25, -15]])
    d_front = car_length / 2 - wheel_positions[0][0]
    d_rear = car_length / 2 - (-wheel_positions[2][0])
    d_l = car_width / 2 - wheel_positions[0][1]
    d_r = car_width / 2 - (-wheel_positions[1][1])
    psi_max = 40

    parking1 = Parking1(1)
    end, obs = parking1.generate_obstacles()
    env = Environment(
        obs,
        car_length,
        car_width,
        wheel_length,
        wheel_width,
        wheel_positions)
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
