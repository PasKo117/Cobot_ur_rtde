"""Keyboard RTDE speed test (tool coordinates).

Run: python keyboard_tool.py [--speed 0.01]. Esc or close window stops robot.
"""
from keyboard_rtde import main as run_keyboard


def main(argv=None):
    return run_keyboard(argv, tool_frame=True, label='keyboard_tool.py')


if __name__ == '__main__':
    main()
