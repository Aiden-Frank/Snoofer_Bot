#!/usr/bin/env python3
from ev3dev2.button import Button
from ev3dev2.motor import (Motor, OUTPUT_A, SpeedRPM)
from ev3dev2.port import LegoPort
import math
from smbus import SMBus
import time

button=Button()
snoofermotor=Motor(OUTPUT_A)
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
            print("Whisker selftest error: \n Failed to read value \n Press GO to stop code")
            while button.enter==False:
                time.wait(0.1)
                if button.enter==True:
                    exit
            
    def read(self):
#       0x44 and 0x45 are the locations of output
        self.bus.write_byte_data(0x70, 0, self.port_address)
        raw_readout=self.bus.read_i2c_block_data(0x21, 0x44, 1)[0]
        if raw_readout<self.calibration_value+7:
            return(0)
        elif raw_readout>=self.calibration_value+7 and raw_readout<=self.calibration_value+20:
            return(1)
        else:
            return(2)
    def calibrate(self) -> None:
        self.bus.write_byte_data(0x70, 0, self.port_address)
        time.sleep(0.5)
        self.calibration_value=self.bus.read_i2c_block_data(0x21, 0x44, 1)[0]
        time.sleep(0.5)
        print (self.calibration_value)

def set_port(port,mode,device): 
#   Function to initiate sensors; port should be INPUT_N where N is 1 to 4
#   mode is a string describing which driver to use (see https://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors)
#   device is also a string, see site above
    sensor=LegoPort(port)
    sensor.mode=mode
    if device=='lego-ev3-gyro':
        sensor.set_device=device
    time.sleep(0.5)


whiskers_port=set_port('pistorms:BBS2',"i2c-thru", "Custom")
left_whisker=Whisker(multiplexor_port=2, name="LW")
left_whisker.selftest()
left_whisker.calibrate()
right_whisker=Whisker(multiplexor_port=1, name="RW")
right_whisker.selftest()
right_whisker.calibrate()
time.sleep(0.5)

stop=False
calibrated=False
readout_printed=False
while stop==False:
    left_output=left_whisker.read()
    right_output=right_whisker.read()
    if left_output>0 and right_output>0:
        snoofermotor.stop()
        print("Only press one whisker at a time!")

    if left_output==1:
        snoofermotor.run_forever(speed_sp=70)
        readout_printed=False
    elif left_output==2:
        snoofermotor.run_forever(speed_sp=200)
        readout_printed=False
    elif right_output==1:
        snoofermotor.run_forever(speed_sp=-70)
        readout_printed=False
    elif right_output==2:
        snoofermotor.run_forever(speed_sp=-200)
        readout_printed=False
    else:
        snoofermotor.stop()
        readout=snoofermotor.position%360
        if readout_printed==False:
            print(readout)
        readout_printed=True
    if button.enter:
        if calibrated==False:
            snoofermotor.position=0
            readout=snoofermotor.position%360
            print(readout)
            calibrated=True
            time.sleep(0.5)
        if calibrated==True:
            stop==True
    time.sleep(0.1)