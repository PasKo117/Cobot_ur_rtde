"""Keyboard speedL experiment, using the same coordinate conversion as app.py."""

import argparse

from robot_control import RobotWorker
from standalone_rtde import RobotSession


def main(argv=None, *, tool_frame=False, label=None):
    parser = argparse.ArgumentParser(description='Проба ручного управления диагностом')
    parser.add_argument('--speed', type=float, default=0.01, help='Линейная скорость, м/с (0..0.05)')
    args = parser.parse_args(argv)
    if not 0 < args.speed <= 0.05:
        parser.error('Ожидается 0 < --speed <= 0.05 м/с')
    import pygame
    pygame.init()
    pygame.display.set_mode((660, 100))
    pygame.display.set_caption(label or 'RTDE keyboard: Esc — выход')
    print('Стрелки: X/Y, PgUp/PgDown: Z, Q/W A/S Z/X: вращение, Esc: выход')
    try:
        with RobotSession('diagnost') as robot:
            clock = pygame.time.Clock()
            running = True
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT or event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        running = False
                if not running:
                    break
                k = pygame.key.get_pressed()
                linear, angular = args.speed, 0.19
                speeds = [linear * (k[pygame.K_RIGHT]-k[pygame.K_LEFT]),
                          linear * (k[pygame.K_UP]-k[pygame.K_DOWN]),
                          linear * (k[pygame.K_PAGEUP]-k[pygame.K_PAGEDOWN]),
                          angular * (k[pygame.K_w]-k[pygame.K_q]),
                          angular * (k[pygame.K_s]-k[pygame.K_a]),
                          angular * (k[pygame.K_x]-k[pygame.K_z])]
                if tool_frame:
                    speeds = RobotWorker._tool_speed_to_base(speeds, robot.pose())
                robot.speed(speeds)
                clock.tick(20)
    finally:
        pygame.quit()


if __name__ == '__main__':
    main()
