import pygame
import time

pygame.init()
pygame.joystick.init()
joy = pygame.joystick.Joystick(0)
joy.init()
print(f"Джойстик: {joy.get_name()} | Осей: {joy.get_numaxes()} | {joy.get_numhats()} ")

try:
    while True:
        pygame.event.pump()
        axes = [f"Axis{i}: {joy.get_axis(i):6.3f}" for i in range(joy.get_numaxes())]
        hats = [f"Hat: {joy.get_hat(0)}"]
        print("\r" + " | ".join(axes) + " | ".join(hats), end="")

        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nВыход")