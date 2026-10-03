"""Read the legacy laser controller on a serial port; no robot connection."""
import argparse

REQUEST = bytes.fromhex('02 43 B0 01 03 F2')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM2')
    parser.add_argument('--once', action='store_true')
    args = parser.parse_args(argv)
    import serial
    with serial.Serial(args.port, baudrate=9600, timeout=1) as link:
        try:
            while True:
                link.write(REQUEST)
                response = link.read(6)
                if len(response) != 6:
                    print('Таймаут: ожидалось 6 байтов')
                else:
                    print('Ответ, hex:', response.hex(' '),
                          'сырое значение:', int.from_bytes(response[2:4], 'big', signed=True))
                if args.once:
                    return
        except KeyboardInterrupt:
            return


if __name__ == '__main__':
    main()
