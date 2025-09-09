import pygame
from math import sqrt
from typing import Tuple
from numpy import array

from mpc_robot.robot import Robot
from mpc_robot.map import Map
from mpc_robot.mpc_controller import Controller


def distance(start: Tuple[int, int], end: Tuple[int, int]) -> float:
    x_s, y_s = start
    x_e, y_e = end

    return sqrt((x_e - x_s) ** 2 + (y_e - y_s) ** 2)


class Simulation:
    def __init__(self, robot: Robot, map: Map, controller: Controller, screen) -> None:
        self.robot = robot
        self.map = map
        self.screen = screen
        self.controller = controller

        self.width, self.height = self.screen.get_size()

        self.obstacles = self.map.objects_list
        self.moving_obstacles = self.map.dynamic_obstacles

    def _check_collisions(self) -> bool:
        x_robot, y_robot = self.robot.state_vec[:2]

        for obs in self.obstacles:
            x_obs, y_obs = obs.rect.center

            c_line = obs.rect.clipline(x_robot, y_robot, x_obs, y_obs)

            if c_line:
                pygame.draw.line(
                    self.screen, (0, 0, 255), (x_robot, y_robot), c_line[0]
                )

                if (
                    distance((x_robot, y_robot), c_line[0]) < self.robot.r * 2 + 5
                ):  # The halo radius, not the robot plus an offset
                    obs.set_vel([0, 0])
                    return True
        for obs in self.moving_obstacles:
            ds = obs.rect.width + 10

            dx = obs.dx
            dy = obs.dy

            left_side = obs.rect.left
            top_side = obs.rect.top

            if left_side < 15:
                obs.set_vel([-dx, dy])
            elif left_side > self.width - ds:
                obs.set_vel([-dx, dy])
            elif top_side < 15:
                obs.set_vel([dx, -dy])
            elif top_side > self.height - ds:
                obs.set_vel([dx, -dy])

        return False

    def update(self):
        self.robot.halo_color = (0, 255, 0)
        coll: bool = self._check_collisions()

        if coll:
            self.robot.set_vel([0, 0])
            self.robot.halo_color = (255, 0, 0)
        else:
            xref = array([*self.map.target.center, 0])
            mov_obs = array([*self.map.moving_obs1.rect.center]) + 15

            res = self.controller.optimize(self.robot.state_vec, xref, mov_obs)
            u_opt = res["u"][0] / self.robot.dt

            self.robot.set_vel(u_opt)
