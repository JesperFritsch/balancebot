from xboxcontroller import XboxController
import math
import threading
import serial
import time
import struct

DEVICE = '/dev/rfcomm1'
BAUD_RATE = 230400
MAX_OUT = 100
gamepad = XboxController()
gamepad.stickThreshold = 0.21
ser = serial.Serial(DEVICE, BAUD_RATE, timeout=5)

timer = 0

def compensateThr(thr, val):
    return round(val * ((abs(val) - thr) / (1 - thr)) * MAX_OUT)


while True:
    try:
        #print(ser.readline())
        time.sleep(0.01)
        timer = time.time()*1000
        x, y, rx, ry, a, b, rb = gamepad.read()
        xOut = compensateThr(gamepad.stickThreshold, rx)
        yOut = compensateThr(gamepad.stickThreshold, -y)
        print(f"X: {xOut}, Y: {yOut}")
        packet = struct.pack('bb', yOut, xOut)
        ser.write(packet)
        ser.write(bytes('\n', 'utf-8'))
    except serial.SerialException:
        time.sleep(1)
        ser = serial.Serial(DEVICE, BAUD_RATE, timeout=2)