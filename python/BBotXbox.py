from xboxcontroller import XboxController
import math
import threading
import serial
import time
import struct

DEVICE = '/dev/rfcomm1'
BAUD_RATE = 230400
MAX_OUT = 11

gamepad = XboxController()
gamepad.stickThreshold = 0.07
ser = serial.Serial(DEVICE, BAUD_RATE, timeout=5)

timer = 0

while True:
    try:
        #print(ser.readline())
        time.sleep(0.03)
        timer = time.time()*1000
        x, y, rx, ry, a, b, rb = gamepad.read()
        packet = struct.pack('bb', round(-(y * MAX_OUT)), round(rx * MAX_OUT))
        ser.write(packet)
        ser.write(bytes('\n', 'utf-8'))
        print(packet)
    except serial.SerialException:
        ser = serial.Serial(DEVICE, BAUD_RATE, timeout=2)