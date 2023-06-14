#!/usr/bin/env python3
#This code is intended for a Lego Mindstorms robot

from ev3dev2.button import Button
from ev3dev2.display import Display
from ev3dev2.display import fonts
from ev3dev2.motor import (Motor, OUTPUT_A, OUTPUT_B, OUTPUT_D,
                            MoveDifferential, SpeedRPM)
from ev3dev2.port import LegoPort
import math
from ev3dev2.sensor.lego import UltrasonicSensor, GyroSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from smbus import SMBus
import time

mode="Pre-Run"

#Classes

class Snoofer(port='pistorms:BBS1', mode='ev3-uart'):
    def __init__(self) -> None:
        set_port(self.port, self.mode, device='lego-ev3-us')
        time.sleep(0.5)
        self.distance_sensor=UltrasonicSensor('pistorms:BBS1')
    def calibrate(self):
        pass
    


class Whisker(multiplexor_port=1,bus=SMBus(1)):
    def __init__(self) -> None:
        port_address=math.pow(2, self.multiplexor_port)
#       Write to multiplexor, which is at 0x70 by default. to tell it 
#       which port set to use at this moment
        self.bus.write_byte_data(0x70, 0, self.port_address)
#       Write to sensor, which is at 0x21 by default, setting mode to read analog voltage
        self.bus.write_i2c_block_data(0x21, 0x42, [1])
        self.calibration_value=self.bus.read_i2c_block_data(0x21, 0x44, 1)
    def read(self):
#       0x44 and 0x45 are the locations of output
        raw_readout=self.bus.read_i2c_block_data(0x21, 0x44, 1)
        if raw_readout<self.calibration_value+10:
            return(0)
        elif raw_readout>=self.calibration_value+10 and raw_readout<=PLACEHOLDER:
            return(1)
        else:
            return(2)
    def recalibrate(self) -> None:
        self.calibration_value=self.bus.read_i2c_block_data(0x21, 0x44, 1)



#Functions

def set_port(port,mode,device): 
#   Function to initiate sensors; port should be INPUT_N where N is 1 to 4
#   mode is a string describing which driver to use (see https://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors)
#   device is also a string, see site above
    sensor=LegoPort(port)
    sensor.mode=mode
    if device=='lego-ev3-gyro':
        sensor.set_device=device
    time.sleep(0.5)


whiskers_port=set_port('pistorms:BBS2',"i2c-thru")
right_whisker=Whisker(multiplexor_port=1)
left_whisker=Whisker(multiplexor_port=2)
myfont=fonts.load('lutBS18')
show=Display()

#Snoofer Calibration

snoofer_calibarated=False
show.text_pixels("Use whiskers to move \n snoofer so orange \n pointer is over \n the orange peg. \n Press middle button \n when done", x=10, y=10, font=fonts.load('lutBS14'))
show.update()
button=Button()
snoofermotor=Motor(OUTPUT_A)
while snoofer_calibarated==False:
    if left_whisker.read>0 and right_whisker.read>0:
        issue_solved=0
        show.text_pixels("Only press one \n whisker at a time! \n \n Press enter to \n try again." , x=10, y=10, font=myfont)
        show.update()
        while issue_solved==0:
            if button.enter:
                issue_solved=1
    time.sleep(0.1)
    show.text_pixels("Use whiskers to move \n snoofer so orange \n pointer is over \n the orange peg. \n Press middle button \n when done", x=10, y=10, font=fonts.load('lutBS14'))
    show.update()
    if button.enter:
        snoofermotor.run_forever(speed_sp=101)
    if button.enter==False:
        snoofermotor.stop()
    if sensor.distance_centimeters<5:
        initiator=1
snoofermotor=Motor(OUTPUT_A)
time.sleep(0.5)