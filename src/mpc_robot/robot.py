import pygame
import numpy as np
import numpy.typing as npt
from typing import List, Tuple


class Robot:
    def __init__(
        self, name: str, x_init: List[float], screen, radius: int = 30, dt: float = 1e-3
    ) -> None:
        self.name = name
        self.state_vec = np.array(x_init)
        self.input_vec = np.array([0.0, 0.0])

        self.r = radius
        self.dt = dt
        self.screen = screen

        self.halo_color: Tuple = (0, 255, 0)

    def dynamics(
        self, x_st: npt.NDArray[np.float64], u: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """A mathematical model representing the differential drive robot's dynamics"""
        x, y, theta = x_st
        v, w = u.copy()

        if np.isclose(w, 0, 1e-4):
            w += 0.0001

        p1 = v / w

        # Calculate new state without modifying input
        new_x = x - p1 * np.sin(theta) + p1 * np.sin(theta + w * self.dt)
        new_y = y + p1 * np.cos(theta) - p1 * np.cos(theta + w * self.dt)
        new_theta = theta + w * self.dt

        return np.array([new_x, new_y, new_theta])

    def set_vel(self, u: List[float]):
        # If I am setting an input, I always need to move
        self.input_vec = np.array(u)
        self.state_vec = self.move()

    def move(self):

        self.state_vec = self.dynamics(self.state_vec, self.input_vec)
        return self.state_vec

    def draw(self):

        x, y, theta = self.state_vec

        # Calculate the robot's heading to a xy point
        r_x = x + np.cos(theta) * self.r
        r_y = y + np.sin(theta) * self.r

        pygame.draw.circle(self.screen, (255, 255, 255), (x, y), self.r)
        pygame.draw.circle(self.screen, self.halo_color, (x, y), self.r * 2, 2)
        pygame.draw.line(self.screen, [0, 0, 0], (x, y), (r_x, r_y))

    def update(self):
        self.draw()
