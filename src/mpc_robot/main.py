import pygame
import numpy as np

from mpc_robot.map import Map
from mpc_robot.robot import Robot
from mpc_robot.mpc_controller import Controller
from mpc_robot.sim import Simulation


pygame.init()

dt = 1e-2

screen = pygame.display.set_mode((1450, 1000))
clock = pygame.time.Clock()

# Create a Robot object
r1 = Robot("robot_1", [200, 200, 0], screen, dt=dt)

# Create a Map object
map = Map("map_1", screen)

# Create a Controller object
controller = Controller(
    name="controller_1",
    constr_method="DMS",
    model=r1.dynamics,
    nx=3,
    nu=2,
    x_guess=0,
    u_guess=0,
    n_pred=3,
    dt=dt,
    Q=np.diag([20, 20, 1]),
    R=np.diag([20, 10]),
    minimize_method="SLSQP",
)

# Create a Simulation object to handle the logic
sim = Simulation(r1, map, controller, screen)

loop: bool = True
while loop:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            loop = False
    screen.fill((30, 30, 30))

    r1.update()
    map.update()
    sim.update()

    pygame.display.flip()

pygame.quit()
