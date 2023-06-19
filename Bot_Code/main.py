#!/usr/bin/env python3
#This code is intended for a Lego Mindstorms robot

from ev3dev2.button import Button
from ev3dev2.display import Display
from ev3dev2.display import fonts
from ev3dev2.motor import (Motor, OUTPUT_A, OUTPUT_C, OUTPUT_D,
                            MoveDifferential, SpeedRPM)
from ev3dev2.port import LegoPort
from ev3dev2.sensor.lego import UltrasonicSensor, GyroSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.wheel import EV3EducationSetTire
import math
from smbus import SMBus
import time

#   Mode Library    The modes that define the bot's behavior
#       Pre-Run: Preparing functions, classes, hardware, and calibrations.
#       Initiation: Preparing variables
#       Wander: Bumbling about looking for stuff

mode="Pre-Run"

#   Classes
#   Enter new classes in alphabetical order

class Snoofer():
    def __init__(self, port='pistorms:BBS1', mode='ev3-uart') -> None:
        self.port=port
        self.mode=mode
        set_port(self.port, self.mode, device='lego-ev3-us')
        time.sleep(0.5)
        self.distance_sensor=UltrasonicSensor('pistorms:BBS1')
    def calibrate(self):
        pass
    


class Whisker():
    def __init__(self, multiplexor_port=1,bus=SMBus(1), name="W") -> None:
        self.name=name
        self.multiplexor_port=multiplexor_port
        self.port_address=int(math.pow(2, self.multiplexor_port))
        self.bus=bus
#       Write to multiplexor, which is at 0x70 by default. to tell it 
#       which port set to use at this moment
        self.bus.write_byte_data(0x70, 0, self.port_address)
#       Write to sensor, which is at 0x21 by default, setting mode to read analog voltage
        self.bus.write_i2c_block_data(0x21, 0x42, [1])
    def selftest(self):
        try:
            self.bus.read_i2c_block_data(0x21, 0x44, 1)[0]
        except:
            show.text_pixels("Whisker selftest error: \n Failed to read value \n Press GO to stop code" , x=10, y=10, font=fonts.load('lutBS14'))
            show.update
            while button.enter==False:
                time.wait(0.1)
                if button.enter==True:
                    exit
            
    def read(self):
#       0x44 and 0x45 are the locations of output
        raw_readout=self.bus.read_i2c_block_data(0x21, 0x44, 1)[0]
        self.bus.write_byte_data(0x70, 0, self.port_address)
        print(self.name)
        print(raw_readout)
        print(self.calibration_value)
        if raw_readout<self.calibration_value+7:
            return(0)
        elif raw_readout>=self.calibration_value+7 and raw_readout<=self.calibration_value+20:
            return(1)
        else:
            return(2)
    def calibrate(self) -> None:
        time.sleep(0.5)
        self.calibration_value=self.bus.read_i2c_block_data(0x21, 0x44, 1)[0]
        time.sleep(0.5)
        print (self.calibration_value)



#   Functions

def set_port(port,mode,device): 
#   Function to initiate sensors; port should be INPUT_N where N is 1 to 4
#   mode is a string describing which driver to use (see https://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors)
#   device is also a string, see site above
    sensor=LegoPort(port)
    sensor.mode=mode
    if device=='lego-ev3-gyro':
        sensor.set_device=device
    time.sleep(0.5)

#   Hardware

button=Button()
bot_placed=False
drive=MoveDifferential(OUTPUT_C, OUTPUT_D, EV3EducationSetTire, 152)
myfont=fonts.load('lutBS18')
gyroport=set_port(port='pistorms:BAS1', mode='ev3-uart', device='lego-ev3-gyro')
gyro=GyroSensor('pistorms:BAS1')
show=Display()
snoofermotor=Motor(OUTPUT_A)
time.sleep(10)
whiskers_port=set_port('pistorms:BBS2',"i2c-thru", "Custom")
left_whisker=Whisker(multiplexor_port=2, name="LW")
left_whisker.selftest()
left_whisker.calibrate()
right_whisker=Whisker(multiplexor_port=1, name="RW")
right_whisker.selftest()
right_whisker.calibrate()
time.sleep(1)

#   Snoofer Calibration

print("------ \n Use whiskers to move snoofer so orange pointer is over the orange peg. Press middle button when done. \n ------")


print ("------ \n Entering Snoofer Calibration \n ------")
while mode=="Pre-Run":
    left_output=left_whisker.read()
    right_output=right_whisker.read()
    if left_output>0 and right_output>0:
        issue_solved=0
        snoofermotor.stop()
        print(" Only press one whisker at a time! Press GO to try again.")
        while issue_solved==0:
            if button.enter:
                issue_solved=1
                time.sleep(0.5)


    if left_output==1:
        snoofermotor.run_forever(speed_sp=72)
        print("L = 1")
    elif left_output==2:
        snoofermotor.run_forever(speed_sp=100)
        print("L = 2")
    elif right_output==1:
        snoofermotor.run_forever(speed_sp=-72)
        print("R = 1")
    elif right_output==2:
        snoofermotor.run_forever(speed_sp=-100)
        print("R = 2")
    else:
        snoofermotor.stop()
    if button.enter:
        mode="Initiation"
        print("Mode set: Initiation")
        drive.odometry_start()
        gyro.calibrate()
    time.sleep(1)


show.text_pixels("Place bot in \n start position \n \n Press button when done", x=10, y=10, font=myfont)
while bot_placed==False:
    if button.enter:
        bot_placed=True
while mode=="Wander":
    drive.on(40,40)