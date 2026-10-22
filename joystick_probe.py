"""Show real pygame indices while pressing buttons; never connects to a robot."""
import json
import time
from pathlib import Path


def main():
    import pygame
    mapping_path = Path(__file__).with_name('joystick_map.json')
    mapping = json.loads(mapping_path.read_text(encoding='utf-8')) if mapping_path.exists() else {}
    names = {0: 'surgeon', 1: 'diagnost'}
    pygame.init()
    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
    for joy in joysticks:
        joy.init()
        print(f'ID {joy.get_id()}: {joy.get_name()}, buttons={joy.get_numbuttons()}, axes={joy.get_numaxes()}')
    print('Нажимайте кнопки по очереди; Ctrl+C для выхода. Этот скрипт не подключается к роботам.')
    previous = {}
    try:
        while True:
            pygame.event.pump()
            for index, joy in enumerate(joysticks):
                for button in range(joy.get_numbuttons()):
                    state = joy.get_button(button)
                    if state and not previous.get((index, button)):
                        actions = [key for key, value in mapping.get(names.get(index), {}).items()
                                   if value == button]
                        print(f'Joystick {index}: pygame index {button}; '
                              f'физическая кнопка {button + 1} (Logitech Extreme 3D Pro); '
                              f'назначение: {", ".join(actions) if actions else "не назначена"}', flush=True)
                    previous[(index, button)] = state
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()


if __name__ == '__main__':
    main()
