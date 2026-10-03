import socket
import time
from random import randint
from tkinter import Tk, Label, mainloop
from time import sleep
import struct

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # конфигурация соединения
sock.connect(('192.168.8.149', 9093))

#Было: 192.168.8.3 6000

#start: 192.168.8.149 9093
#ip now: 192.168.8.3 ...
#9093
#502
#6000


def calculate_bcc(command, data1, data2):
    return command ^ data1 ^ data2

# Для определения пакета
stx = 0x02
cmd = 0x43  # 'C' — команда чтения
set_in = 0x57 #Ввод настроек (скоро) ((но перед вводом чтение применить, костыль))
set_out = 0x52 #Чтение настроек (скоро)
etx = 0x03

def build_packet(command: int, data1: int, data2: int) -> bytes:
    bcc = calculate_bcc(command, data1, data2)
    packet = [stx, command, data1, data2, etx, bcc]
    return bytes(packet)

def on_off():
    #Формат: [STX][ACK][MSB][LSB][ETX][BCC]

    bcc = calculate_bcc(cmd, 0xA0, 0x03)
    comand_on = [stx, cmd, 0xA0, 0x03, etx, bcc]

    bcc = calculate_bcc(cmd, 0xA0, 0x02)
    comand_off = [stx, cmd, 0xA0, 0x02, etx, bcc]

    print("001\n")

    for i in range(4):

        print("002\n")

        sock.send(bytes(comand_on))

        print("003\n")

        #response = sock.recv(6)

        #print("Получен ответ: ", response.decode())

        time.sleep(1)

        sock.send(bytes(comand_off))

        #response = sock.recv(6)
        #print("Получен ответ: ", response.decode())

        time.sleep(1)


#on_off()



def build_read_distance_packet():
    stx = 0x02
    cmd = 0x43  # 'C' — команда чтения
    data1 = 0xB0
    data2 = 0x01
    etx = 0x03

    bcc = calculate_bcc(cmd, data1, data2)
    return bytes([stx, cmd, data1, data2, etx, bcc])


def read_distance(ip='192.168.8.3', port=6000, timeout=2):
    """Подключиться и запросить расстояние"""
    try:
        # Создаём сокет
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((ip, port))
        print(f" Подключено к {ip}:{port}")

        # Отправляем пакет
        packet = build_read_distance_packet()
        print(f" Отправка: {packet.hex()}")
        sock.send(packet)

        # Ждём ответ (6 байт)
        response = sock.recv(6)
        print(f" Ответ:    {response.hex()}")

        # Анализируем ответ
        if len(response) == 6 and response[0] == 0x02 and response[4] == 0x03:
            if response[1] == 0x06:  # ACK = успех
                # Считываем 16-битное значение (дополнительный код)
                value = (response[2] << 8) | response[3]
                if value > 32767:
                    value -= 65536

                # Для модели B035: 1 единица = 10 мкм = 0.01 мм
                distance_mm = value * 0.01
                print(f"Растояние: {distance_mm:.2f} мм")
                return distance_mm
            elif response[1] == 0x15:  # NAK = ошибка
                error_code = response[2]
                print(f" Ошибка датчика: код {error_code:02X}")
                return None
        else:
            print("  Некорректный формат ответа")
            return None

    except Exception as e:
        print(f" Ошибка подключения: {e}")
        return None
    finally:
        if 'sock' in locals():
            sock.close()


# Запуск теста
if __name__ == "__main__":
    print("Тест подключения к датчику OD Mini Pro\n")
    result = read_distance(port=6000)  # ← пробуем порт 6000

    if result is None:
        print("\nПопробуем порт 502...")
        time.sleep(1)
        read_distance(port=502)  # ← если 6000 не сработал
