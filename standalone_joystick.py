"""Console frontend for the same two-robot worker used by app.py."""

import argparse
from datetime import datetime
import multiprocessing as mp
from pathlib import Path
from queue import Empty

import robot_control
from route_io import write_poses


def main(source='surgeon', argv=None):
    parser = argparse.ArgumentParser(description='Два джойстика, управление как в app.py')
    parser.add_argument('--speed', type=int, default=100, help='0..100%% от скорости приложения')
    args = parser.parse_args(argv)
    robot_control.speed_fraction(args.speed)
    commands, messages, heartbeat = mp.Queue(), mp.Queue(), mp.Queue()
    stop, shutdown, speed = mp.Event(), mp.Event(), mp.Value('i', args.speed)
    worker = mp.Process(target=robot_control.main,
                        args=(commands, messages, stop, shutdown, heartbeat,
                              robot_control.DIAG_IP, robot_control.SURGEON_IP, speed))
    worker.start()
    print(f'{source}: ожидание подключения; Ctrl+C — остановка', flush=True)
    had_error = False
    try:
        started = False
        while worker.is_alive():
            try:
                name, state, *_ = heartbeat.get_nowait()
                if name == 'robot_worker' and state == 'READY' and not started:
                    commands.put(('manual_start', {}))
                    started = True
            except Empty:
                pass
            try:
                kind, data = messages.get(timeout=0.1)
            except Empty:
                continue
            if kind == 'SAVE_REQUEST':
                output = Path('routes') / f"{data['robot']}_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
                output.parent.mkdir(parents=True, exist_ok=True)
                count = write_poses(output, data['poses'])
                print(f'Сохранено {count} точек в {output}; перезапустите управление', flush=True)
            elif kind in ('ERROR', 'INFO'):
                print(f'{kind}: {data["text"]}', flush=True)
                had_error |= kind == 'ERROR'
            elif kind == 'MODE':
                print(f'Режим: {data["mode"]}', flush=True)
            elif kind == 'PATH':
                print(f'{data["robot"]}: точек {len(data["poses"])}', flush=True)
    except KeyboardInterrupt:
        print('Остановка...', flush=True)
    finally:
        stop.set()
        shutdown.set()
        worker.join(timeout=4)
        if worker.is_alive():
            worker.terminate()
            worker.join(timeout=2)
        while True:
            try:
                kind, data = messages.get_nowait()
            except Empty:
                break
            if kind == 'ERROR':
                print('ERROR:', data['text'], flush=True)
                had_error = True
    return 1 if had_error else (worker.exitcode or 0)
