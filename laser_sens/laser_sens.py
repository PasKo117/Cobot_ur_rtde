import socket
import time
from random import randint
from tkinter import Tk, Label, mainloop
from time import sleep
import struct


sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # конфигурация соединения
sock.connect(('192.168.8.149', 9093))
#sock.send('dal'.encode())
fname = ('laser' + str(time.asctime(time.localtime(time.time()))))

def ones_complement_to_signed(value, bit_length = 16):
    if value & (1 << (bit_length-1)):
        value = ~value & ((1<< bit_length)-1)
        value = -(value +1)
    return value


class App(Tk):
    def __init__(self):
        super().__init__()
        self.geometry("800x400+100+80") # создание окна
        self.label = Label(self, font='arial 17')
        self.label.pack()
        self.run()

    def run(self):


            data = sock.recv(16) # получение данных
    # sock.close()
    # print (data)
            ts = data.decode()
            ts = ts.split('end')
            ts2 = ts[1].split('end')
            print(ones_complement_to_signed(int(ts[0])))
            print(ones_complement_to_signed(int(ts2[0])))

            # print(ts[0], '____')
            b = ts[0]
            b1 = ts2[0]


            self.label["text"] = (str('Показания лазерных датчиков расстояния\n\n') + str(' Робот-диагност: ')+str(b)+str('\n')+ str(' Робот-хирург: ') + str(b1))# Вывод текста в окне

           

            #fname = str(1)

            
            self.after(10, self.run)
#while True:



app = App()
app.mainloop()

