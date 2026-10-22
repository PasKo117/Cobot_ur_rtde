# Dashboard examples
# CB-series: https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/dashboard-server-port-29999-15690/
# E-series:  https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/dashboard-server-e-series-port-29999-42728/
import socket
import sys


def main():
    HOST = "192.168.8.3"
    PORT = 29999
    s = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5.0)
        s.connect((HOST, PORT))

        cmd = "unlock protective stop\n"
        s.send(cmd.encode())
        response = s.recv(4096).decode().strip()
        print(f"[Диагност] Ответ на сброс E-Stop: {response}")

    except Exception as e:
        print(f"[Диагност] Ошибка при сбросе E-Stop: {e}")
        sys.exit(1)
    finally:
        if s:
            s.close()


if __name__ == "__main__":
    main()