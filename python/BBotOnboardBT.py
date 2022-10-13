from time import sleep
import asyncio
import time
import pydbus
from gi.repository import GLib
from evdev import InputDevice, categorize, ecodes

DEVICE_ADDR = 'D7:C6:99:D7:B8:CB' #  micro:bit address
DRIVE_CHAR = 'B1C046E6-77C5-46A3-87C0-6EB629DB8247'
TURN_CHAR = '1CBF62C3-8F2F-4630-B0B4-DFA1B03379A9'

# DBus object paths
BLUEZ_SERVICE = 'org.bluez'
ADAPTER_PATH = '/org/bluez/hci0'
device_path = f"{ADAPTER_PATH}/dev_{DEVICE_ADDR.replace(':', '_')}"

# setup dbus
bus = pydbus.SystemBus()
mngr = bus.get(BLUEZ_SERVICE, '/')
adapter = bus.get(BLUEZ_SERVICE, ADAPTER_PATH) 
device = bus.get(BLUEZ_SERVICE, device_path)

device.Connect()

dev = InputDevice('/dev/input/event17')

while not device.ServicesResolved:
    sleep(0.5)

def get_characteristic_path(dev_path, uuid):
    """Look up DBus path for characteristic UUID"""
    mng_objs = mngr.GetManagedObjects()
    for path in mng_objs:
        chr_uuid = mng_objs[path].get('org.bluez.GattCharacteristic1', {}).get('UUID')
        if path.startswith(dev_path) and chr_uuid == uuid.casefold():
           return path

# Characteristic DBus information
drive_path = get_characteristic_path(device._path, DRIVE_CHAR)
drive = bus.get(BLUEZ_SERVICE, drive_path)
turn_path = get_characteristic_path(device._path, TURN_CHAR)
turn = bus.get(BLUEZ_SERVICE, turn_path)
# Read button A without event loop notifications
new_drive = int(200).to_bytes(1, 'little')
new_turn = int(255).to_bytes(1, 'little')
drive.WriteValue(new_drive, {})
turn.WriteValue(new_turn, {})
print(drive.ReadValue({}))
print(turn.ReadValue({}))

mainloop = GLib.MainLoop()
#btn_a.onPropertiesChanged = btn_handler
#btn_a.StartNotify()

timer = 0
driveValue = 20
turnValue = 20

try:
    while True:
        event = dev.read_one()
        if event:
            if event.type == ecodes.EV_REL:
                if event.code == ecodes.REL_Y:
                    value = event.value
                    if value > 20:
                        value = 20
                    elif value < -20:
                        value = -20
                    driveValue = 20 - value
                elif event.code == ecodes.REL_X:
                    value = event.value
                    if value > 20:
                        value = 20
                    elif value < -20:
                        value = -20
                    turnValue = 20 + value
        print('Drive:{}'. format(driveValue))
        new_drive = int(driveValue).to_bytes(1, 'little')
        drive.WriteValue(new_drive, {})
        print('Turn:{}'. format(turnValue))
        new_turn = int(turnValue).to_bytes(1, 'little')
        turn.WriteValue(new_turn, {})


except KeyboardInterrupt:
    device.Disconnect()
finally:
    #btn_a.StopNotify()
    device.Disconnect()