# Dashboard examples
# CB-series: https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/dashboard-server-port-29999-15690/
# E-series:  https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/dashboard-server-e-series-port-29999-42728/
import socket
import time
import multiprocessing as mp

def main(heartbeat):
    HOST = "192.168.8.4"
    PORT = 29999
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))

    cmd = "power off\n"
    # cmd = "stop\n"
    # cmd = "play\n"
    # cmd = "pause\n"

    s.send(cmd.encode())
    heartbeat.put((mp.current_process().name, "FINISHED"))

