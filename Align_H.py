import time
from ur_rtde import RTDEControlInterface, RTDEReceiveInterface


def main():
    ip = "192.168.8.4"
    print(f"Подключение к роботу хирурга ({ip})...")

    try:
        # Инициализация интерфейсов ur_rtde
        ctrl = RTDEControlInterface(ip)
        recv = RTDEReceiveInterface(ip)

        if not ctrl.isConnected():
            print("Ошибка: не удалось подключиться к роботу.")
            return

        # Получение текущей позы
        pose = recv.getActualTCPPose()
        print(f"Текущая поза: {pose}")

        # Формирование новой позы (поворот вокруг X на 90 градусов = 3.14/2)
        target_pose = [pose[0], pose[1], pose[2], 3.14 / 2, 0, 0]
        print("Выполнение движения...")

        # Движение (в ur_rtde аргументы называются speed и acceleration)
        ctrl.moveL(target_pose, speed=0.2, acceleration=0.2)

        # Ожидание завершения движения
        while ctrl.isProgramRunning():
            time.sleep(0.01)

        print("Выравнивание завершено.")

        # Корректное отключение
        ctrl.disconnect()
        recv.disconnect()

    except Exception as e:
        print(f"Критическая ошибка: {e}")


if __name__ == "__main__":
    main()