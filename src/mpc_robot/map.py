import pygame
import numpy as np
import numpy.typing as npt

from typing import List, Optional, Dict, Union


class Obstacle:
    def __init__(
        self,
        name: str,
        left: int,
        top: int,
        width: int,
        height: int,
        screen,
        dx: int = 0,
        dy: int = 0,
    ) -> None:
        """Rect(left, top, width, height) -> Rect"""
        self.name = name
        self.l = left
        self.t = top
        self.w = width
        self.h = height

        self.dx = dx
        self.dy = dy

        self.screen = screen

        self.rect = pygame.Rect([self.l, self.t, self.w, self.h])

    def set_vel(self, u: List[int]) -> None:
        assert isinstance(
            u[0], int
        ), f"Velocity is expressed as pixel displacement, only ints are allowed"
        assert isinstance(
            u[1], int
        ), f"Velocity is expressed as pixel displacement, only ints are allowed"
        """
        Setting the velocity of the obstacle by providing the displacement in pixels per frame

        Args:
            dx (int)    : X axis displacement
            dy (int)    : Y axis displacement
        """

        self.dx = u[0]
        self.dy = u[1]

    def draw(self):
        pygame.draw.rect(self.screen, (255, 255, 255), self.rect)


class ObstacleFactory:

    def __init__(self, screen) -> None:
        self.screen = screen
        self.n_obs: int = -1

    def __call__(
        self, left: int, top: int, width: int, height: int, dx: int, dy: int
    ) -> Obstacle:
        self.n_obs += 1
        return Obstacle(
            f"obs_{self.n_obs}", left, top, width, height, self.screen, dx, dy
        )


class Map:
    def __init__(
        self, name: str, screen, target=pygame.Rect(400, 600, 200, 200)
    ) -> None:
        self.name = name
        self.screen = screen
        self.target = target

        self.robot_pose: Optional[npt.NDArray[np.float64]] = None
        self.static_obstacles: List[Obstacle] = []
        self.dynamic_obstacles: List[Obstacle] = []
        self.obs_factory = ObstacleFactory(screen)

        self.objects_list: List[Obstacle] = []
        self.objects: Dict[str, Obstacle] = {}

        self.width, self.height = screen.get_size()
        self.top_border = self.obs_factory(0, 0, self.width, 10, 0, 0)
        self.bottom_border = self.obs_factory(0, self.height - 10, self.width, 10, 0, 0)
        self.right_border = self.obs_factory(self.width - 10, 0, 10, self.height, 0, 0)
        self.left_border = self.obs_factory(0, 0, 10, self.height, 0, 0)

        self.moving_obs1 = self.obs_factory(
            self.width // 2, self.height // 2, 50, 50, 8, 8
        )

        self.static_obstacles.extend(
            [self.top_border, self.bottom_border, self.right_border, self.left_border]
        )

        self.dynamic_obstacles.extend([self.moving_obs1])

        self.objects_list.extend(
            [*self.static_obstacles, *self.dynamic_obstacles, self.moving_obs1]
        )

        self.obstacles = {obj.name: obj for obj in self.objects_list}

    def update(self, pause: bool) -> None:

        for moving_obs in self.dynamic_obstacles:
            if pause:
                moving_x_speed = 0
                moving_y_speed = 0
            else:
                moving_x_speed = moving_obs.dx
                moving_y_speed = moving_obs.dy

            moving_obs.rect.x += moving_x_speed
            moving_obs.rect.y += moving_y_speed

            moving_obs.draw()

        for obs in self.static_obstacles:
            obs.draw()

        r, g, b = np.random.randint(0, 255, (3,))
        # pygame.draw.rect(self.screen, (r, g, b), self.target, 2)
