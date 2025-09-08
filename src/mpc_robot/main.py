import pygame

from env import Environment

pygame.init()

screen = pygame.display.set_mode((1450, 1000))

# Create an Environment object
env = Environment(screen)

sim: bool = True
while sim:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sim = False
    screen.fill((30, 30, 30))

    env.update()

    pygame.display.flip()

pygame.quit()
