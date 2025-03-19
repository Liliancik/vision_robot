import pygame
from CRobot import CRobot

robot = CRobot(
    LMPins=(8, 11),  # AIN1, AIN2
    RMPins=(10, 12),  # BIN1, BIN2
    PWMPins=(7, 9)  # PWMA, PWMB
)

pygame.init()
screen = pygame.display.set_mode((600, 400))  # Creates a window for Realtime control
pygame.display.set_caption("Rover Control")

Finished = False
while not Finished:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            Finished = True
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_s:
                robot.forward(0.4)
            elif event.key == pygame.K_w:
                robot.backward(0.4)
            elif event.key == pygame.K_a:
                robot.left(0.8)
            elif event.key == pygame.K_d:
                robot.right(0.8)
            elif event.key == pygame.K_x:
                robot.stop()
            elif event.key == pygame.K_q:
                robot.stop()
                Finished = True

pygame.quit()