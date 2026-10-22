import time
from ur_rtde import RTDEControlInterface, RTDEReceiveInterface


def main():
    ip = "192.168.8.3"
    print(f"Подключение к диагносту ({ip})...")
    try:
        ctrl = RTDEControlInterface(ip)
        recv = RTDEReceiveInterface(ip)

        if not ctrl.isConnected():
            print("Ошибка подключения!")
            return

        pose = recv.getActualTCPPose()
        print(f"Текущая поза: {pose}")

        # Пример выравнивания (подставь свои целевые координаты)
        target_pose = [pose[0], pose[1], pose[2] + 0.05, pose[3], pose[4], pose[5]]
        print("Выполнение движения...")
        ctrl.moveL(target_pose, speed=0.2, acceleration=0.2)

        while ctrl.isProgramRunning():
            time.sleep(0.1)

        print("Выравнивание завершено.")
        ctrl.disconnect()
        recv.disconnect()
    except Exception as e:
        print(f"Ошибка: {e}")


if __name__ == '__main__':
    main()