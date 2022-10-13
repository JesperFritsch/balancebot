import serial
from evdev import InputDevice, categorize, ecodes
import time
import struct

DEVICE = '/dev/rfcomm1'
BAUD_RATE = 230400
MAX_OUT = 11

dev = InputDevice('/dev/input/event8')
ser = serial.Serial(DEVICE, BAUD_RATE, timeout=2)


timer = 0
timer2 = 0
driveVal = 0
turnVal = 0

while True:
    try:
        event = dev.read_one()
        timestamp = time.time()*1000
        if event:
            if event.type == ecodes.EV_REL:
                if event.code == ecodes.REL_Y:
                    timer = timestamp
                    value = event.value
                    if value > MAX_OUT:
                        value = MAX_OUT
                    elif value < -MAX_OUT:
                        value = -MAX_OUT
                    driveVal = value
                elif event.code == ecodes.REL_X:
                    timer = timestamp
                    value = event.value
                    if value > MAX_OUT:
                        value = MAX_OUT
                    elif value < -MAX_OUT:
                        value = -MAX_OUT
                    turnVal = value
        elif timestamp > timer + 30:
            time.sleep(0.03)
            driveVal = 0
            turnVal = 0
        if timestamp > timer2 + 30:
            timer2 = timestamp
            packet = struct.pack('bb', -driveVal, turnVal)
            ser.write(packet)
            ser.write(bytes('\n', 'utf-8'))
            #print(str((ser.readline())))
            print(f"drive: {driveVal}, turn: {turnVal}")
    except serial.SerialException:
        ser = serial.Serial(DEVICE, BAUD_RATE, timeout=2)

    #print(f"Drive: {driveVal}, Turn: {turnVal}, timer {timer}, tistamp: {timestamp}")
