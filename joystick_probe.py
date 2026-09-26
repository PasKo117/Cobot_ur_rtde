"""Show real pygame indices while pressing buttons; never connects to a robot."""
import time
import pygame


def main():
    pygame.init()
    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
    for joy in joysticks:
        joy.init()
        print(f'ID {joy.get_id()}: {joy.get_name()}, buttons={joy.get_numbuttons()}, axes={joy.get_numaxes()}')
    previous = {}
    try:
        while True:
            pygame.event.pump()
            for index, joy in enumerate(joysticks):
                for button in range(joy.get_numbuttons()):
                    state = joy.get_button(button)
                    if state and not previous.get((index, button)):
                        print(f'Joystick {index}: button {button}')
                    previous[(index, button)] = state
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()


if __name__ == '__main__':
    main()
