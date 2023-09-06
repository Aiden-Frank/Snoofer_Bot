#!/usr/bin/env python3

from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.port import LegoPort
import time
from smbus import SMBus
bus=SMBus(1)

def set_port(port,mode,device): 
#   Function to initiate sensors; port should be INPUT_N where N is 1 to 4
#   mode is a string describing which driver to use (see https://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors)
#   device is also a string, see site above
    sensor=LegoPort(port)
    sensor.mode=mode
    if device=='lego-ev3-gyro':
        sensor.set_device=device
    time.sleep(0.5)

snoofer_port=set_port('pistorms:BBS1',"i2c-thru", "Custom")
class Snoofer():
    def __init__(self,bus=SMBus(1)):
        self.bus=bus
        self.bus.write_byte_data(0x62, 0x00, 4)

snoofer=Snoofer(bus=bus)