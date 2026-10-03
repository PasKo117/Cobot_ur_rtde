"""Keyboard RTDE speed test (base coordinates).

Run: python speedl_keyboard_83.py [--speed 0.01]. Esc or close window stops robot.
"""
from keyboard_rtde import main as run_keyboard


def main(argv=None):
    return run_keyboard(argv, tool_frame=False, label='speedl_keyboard_83.py')


if __name__ == '__main__':
    main()
