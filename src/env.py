import pygame
import numpy as np

from typing import List


class Obstacle:

    def __init__(
        self,
        left: int,
        top: int,
        width: int,
        height: int,
        screen,
        dx: int = 0,
        dy: int = 0,
    ) -> None:
        """Rect(left, top, width, height) -> Rect"""
        self.l = left
        self.t = top
        self.w = width
        self.h = height

        self.dx = dx
        self.dy = dy

        self.screen = screen

        self.rect = pygame.Rect([self.l, self.t, self.w, self.h])

    def set_velocity(self, dx: int, dy: int) -> None:
        """
        Setting the velocity of the obstacle by providing the displacement in pixels per frame

        Args:
            dx (float)  : X axis displacement
            dy (int)    : Y axis displacement
        """

        self.dx = dx
        self.dy = dy

    def draw(self):
        pygame.draw.rect(self.screen, (255, 255, 255), self.rect)


class ObstacleFactory:

    def __init__(self, screen) -> None:
        self.screen = screen

    def __call__(
        self, left: int, top: int, width: int, height: int, dx: int, dy: int
    ) -> Obstacle:
        return Obstacle(left, top, width, height, self.screen, dx, dy)


class Environment:
    def __init__(self, screen) -> None:
        self.static_obstacles: List[Obstacle] = []
        self.dynamic_obstacles: List[Obstacle] = []
        self.obs_factory = ObstacleFactory(screen)

        width, height = screen.get_size()
        self.top_border = self.obs_factory(0, 0, width, 10, 0, 0)
        self.bottom_border = self.obs_factory(0, height - 10, width, 10, 0, 0)
        self.right_border = self.obs_factory(width - 10, 0, 10, height, 0, 0)
        self.left_border = self.obs_factory(0, 0, 10, height, 0, 0)

        self.moving_obs1 = self.obs_factory(width // 2, height // 2, 50, 50, 1, 1)

        self.static_obstacles.extend(
            [self.top_border, self.bottom_border, self.right_border, self.left_border]
        )

        self.dynamic_obstacles.extend([self.moving_obs1])

    def update(self):

        for moving_obs in self.dynamic_obstacles:
            moving_obs.rect.x += moving_obs.dx
            moving_obs.rect.y += moving_obs.dy

            moving_obs.draw()

        for obs in self.static_obstacles:
            obs.draw()
