import cv2
import numpy as np
from time import sleep
import argparse

from environment import Environment, Parking1
from control import Car_Dynamics, MPC_Controller, Linear_MPC_Controller
from pathplanning import PathPlanning

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--x_start', type=int, default=75, help='X of start')
    parser.add_argument('--y_start', type=int, default=45, help='Y of start')
    parser.add_argument('--psi_start', type=int, default=-30, help='psi of start')
    parser.add_argument('--x_end', type=int, default=90, help='X of end')
    parser.add_argument('--y_end', type=int, default=80, help='Y of end')
    parser.add_argument('--parking_length', '-p', type=int, default=20, help='parking_length [m]')
    parser.add_argument('--parking_margin', '-m', type=int, default=1, help='parking_margin [m]')
    parser.add_argument('--last_backward_length', '-b', type=int, default=0, help='parking_margin [m]')
    parser.add_argument('--use_control', '-c', action='store_true', help='use control')

    args = parser.parse_args()

    start = np.array([args.x_start, args.y_start])
    end   = np.array([args.x_end, args.y_end])

    # m
    car_length = 8
    car_width = 4
    wheel_length = 1.5
    wheel_width = 0.7
    wheel_positions = np.array(
        [[2.0, 1.0], [2.0, -1.0], [-2.0, 1.0], [-2.0, -1.0]])  # [0,0] is car's center

    # wheel_base = wheel_positions[0][0] - wheel_positions[2][0]
    my_car = Car_Dynamics(
        start[0],
        start[1],
        0,
        np.deg2rad(args.psi_start),
        # length=wheel_base,
        dt=0.2,
        car_length=car_length,
        car_width=car_width,
        wheel_length=wheel_length,
        wheel_width=wheel_width,
        wheel_positions=wheel_positions,
        )

    parking1 = Parking1(
        my_car,
        args.parking_length,
        args.parking_margin,
        args.last_backward_length)
    end, obs = parking1.generate_obstacles()

    env = Environment(
        obs,
        my_car,
        # car_length * 10,
        # car_width * 10,
        # wheel_length * 10,
        # wheel_width * 10,
        # wheel_positions * 10,
        args.parking_margin
    )

    MPC_HORIZON = 5
    controller = MPC_Controller()

    res = env.render(my_car.x, my_car.y, my_car.psi, 0)

    cv2.imshow('environment', res)
    key = cv2.waitKey()

    path_planner = PathPlanning(obs, my_car, parking1)
    print("start end: ", start,end)
    path, stear = path_planner.plan_path(
        start[0] - my_car.a / 2 * np.cos(np.deg2rad(args.psi_start)),
        start[1] - my_car.a / 2 * np.sin(np.deg2rad(args.psi_start)),
        np.deg2rad(args.psi_start),
        end[0], end[1],
        args.last_backward_length)
    print(len(path), len(stear))
    env.draw_footprint(path)
    env.draw_path(path)

    x_F = parking1.cars[1][0] - my_car.car_length / 2
    # env.plot([[int(end[0]), int(end[1])]])
    # env.plot([[int(end[1]), int(end[0])]])
    # env.plot([[int(x_F), int(end[1])]])
    C_l = np.array(end + [0, path_planner.R_Elmin])
    print(C_l)
    # env.plot([C_l])
    # env.plot([path_planner.B])
    # env.plot([path_planner.F1])
    # env.plot([path_planner.Cr])
    # env.plot([path_planner.Cl])
    # env.plot([path_planner.G2])
    # env.plot([[int(end[1]), int(x_F)]])

    res = env.render(my_car.x, my_car.y, my_car.psi, 0)
    cv2.imshow('environment', res)
    key = cv2.waitKey()
    # vehicle_center_path = path_planner.get_vehicle_center_path(path)
    # for i, point in enumerate(vehicle_center_path):
    for i in range(len(path)):
        if args.use_control:
            vehicle_center_path = path_planner.get_vehicle_center_path(path)
            acc, delta = controller.optimize(
                my_car, vehicle_center_path[i:i + MPC_HORIZON])
            my_car.update_state(my_car.move(acc,  delta))
            x, y, psi = my_car.x, my_car.y, my_car.psi
        else:
            delta = stear[i]
            if delta != stear[i - 1] and i > 0:
                cv2.imshow('environment', res)
                key = cv2.waitKey(1)
                sleep(0.5)
                res = env.render(x, y, psi, delta)
                cv2.imshow('environment', res)
                key = cv2.waitKey(1)
                sleep(0.5)
            point = path[i]
            x = point[0]
            y = point[1]
            if i < len(path) - 1:
                v = path[i + 1] - point
                # v = point - path[i + 1]
                if np.any(v==0):
                    continue
                # psi = np.arctan2(v[1], v[0])
                psi = np.arctan(v[1] / v[0])
            x += my_car.a / 2 * np.cos(psi)
            y += my_car.a / 2 * np.sin(psi)
                # sleep(0.3)
        res = env.render(x, y, psi, delta)
        # sleep(0.1)
        is_collision = env.check_collision(x, y, psi)

        # if is_collision:
        #     break
        cv2.imshow('environment', res)
        key = cv2.waitKey(1)
        if key == ord('s'):
            cv2.imwrite('res.png', res*255)

    # zeroing car steer
    res = env.render(x, y, psi, 0)
    cv2.imshow('environment', res)
    key = cv2.waitKey()
    #############################################################################################

    cv2.destroyAllWindows()
