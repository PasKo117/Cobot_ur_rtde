"""Legacy diagnostic joystick entry point; shares the app's two-robot worker."""
from standalone_joystick import main as run_joysticks


def main(argv=None):
    return run_joysticks('diagnost', argv)


if __name__ == '__main__':
    import multiprocessing as mp
    mp.freeze_support()
    raise SystemExit(main())
