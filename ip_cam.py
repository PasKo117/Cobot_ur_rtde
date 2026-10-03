"""Optional network-camera PTZ probe; requires explicit credentials/command."""
import argparse
import os


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pan', type=int, help='Одно пробное движение PTZ, -100..100')
    parser.add_argument('--ip', default='192.168.8.65')
    args = parser.parse_args(argv)
    if args.pan is None:
        parser.print_help()
        return
    if not -100 <= args.pan <= 100:
        parser.error('--pan должен быть -100..100')
    user, password = os.getenv('COBOT_CAMERA_USER'), os.getenv('COBOT_CAMERA_PASSWORD')
    if not user or not password:
        parser.error('Задайте COBOT_CAMERA_USER и COBOT_CAMERA_PASSWORD в окружении')
    import requests
    from requests.auth import HTTPDigestAuth
    xml = f'<PTZData><pan>{args.pan}</pan><tilt>0</tilt></PTZData>'
    result = requests.put(f'http://{args.ip}/ISAPI/PTZCtrl/channels/1/continuous',
                          auth=HTTPDigestAuth(user, password), data=xml, timeout=5)
    result.raise_for_status()
    print('PTZ:', result.status_code)


if __name__ == '__main__':
    main()
