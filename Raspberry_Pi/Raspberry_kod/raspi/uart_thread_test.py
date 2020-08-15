#!/usr/bin/python3

import threading
import RPi.GPIO as GPIO
import serial
import time

UART=serial.Serial()
UART.baudrate=115200
UART.port='/dev/serial0'
UART.open()

masa=""

def UART_Thread():
    global masa
    while True:
        if not masa:
            masa=UART.read(5)
            #masa="345"

if __name__ == "__main__":
    th=threading.Thread(target=UART_Thread)
    th.start()    
    while True:
        time.sleep(1)
        print(masa)
        masa=""
