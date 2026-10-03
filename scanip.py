"""Check selected TCP ports of one host. No sockets are opened at import."""
import argparse
import socket

PORTS = [80, 443, 502, 8000, 8080, 9093, 5000, 6000, 1883, 5001]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', default='192.168.8.3')
    args = parser.parse_args(argv)
    for port in PORTS:
        try:
            with socket.create_connection((args.ip, port), timeout=0.5):
                print(f'{args.ip}:{port} открыт')
        except OSError:
            print(f'{args.ip}:{port} закрыт/недоступен')


if __name__ == '__main__':
    main()
