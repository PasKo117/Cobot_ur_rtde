# Dashboard examples
# CB-series: https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/dashboard-server-port-29999-15690/
# E-series:  https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/dashboard-server-e-series-port-29999-42728/
import socket
import time
import multiprocessing as mp


def main(heartbeat):
    HOST = "192.168.8.4"
    PORT = 29999
    s = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5.0)  # Таймаут на случай, если робот не отвечает
        s.connect((HOST, PORT))

        # 1. Включение питания
        cmd = "power on\n"
        s.send(cmd.encode())
        response = s.recv(4096).decode().strip()
        print(f"[Хирург] Ответ на 'power on': {response}")

        time.sleep(5)  # Ожидание инициализации

        # 2. Снятие тормозов
        cmd = "brake release\n"
        s.send(cmd.encode())
        response = s.recv(4096).decode().strip()
        print(f"[Хирург] Ответ на 'brake release': {response}")

    except Exception as e:
        print(f"[Хирург] Ошибка при включении: {e}")
    finally:
        # ГАРАНТИРОВАННОЕ закрытие сокета в любом случае
        if s:
            s.close()

    heartbeat.put((mp.current_process().name, "FINISHED"))


if __name__ == "__main__":
    # Заглушка для тестирования без multiprocessing
    class DummyQueue:
        def put(self, item):
            print(f"Heartbeat: {item}")


    main(DummyQueue())