#!/usr/bin/env python3
from ev3dev2.motor import (Motor, OUTPUT_C, OUTPUT_D, MoveDifferential)
from ev3dev2.wheel import EV3EducationSetTire
import time
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.port import LegoPort
import math

def set_port(port,mode,device): 
#   Function to initiate sensors; port should be INPUT_N where N is 1 to 4
#   mode is a string describing which driver to use (see https://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors)
#   device is also a string, see site above
    sensor=LegoPort(port)
    sensor.mode=mode
    if device=='lego-ev3-gyro':
        sensor.set_device=device
    time.sleep(0.5)

def print_data():
    print('Gyro Angle: ',math.radians(gyro.circle_angle()))
    print('Odometry Angle: ',drive.theta)
drive=MoveDifferential(OUTPUT_C, OUTPUT_D, EV3EducationSetTire, 152)
gyroport=set_port(port='pistorms:BAS1', mode='ev3-uart', device='lego-ev3-gyro')
gyro=GyroSensor('pistorms:BAS1')
drive.gyro=gyro
drive.odometry_start(sleep_time=0.04,use_gyro=True)
time.sleep(1)
print_data()
drive.odometry_start(sleep_time=0.04,use_gyro=True)
drive.turn_degrees(20, 1800, block=True)
time.sleep(40)
print()
drive.off()
print_data()
time.sleep(10)
drive.off()