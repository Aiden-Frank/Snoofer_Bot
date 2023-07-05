#!/usr/bin/env python3
#This code is intended for a Lego Mindstorms robot

from ev3dev2.button import Button
from ev3dev2.motor import (Motor, OUTPUT_A, OUTPUT_C, OUTPUT_D,
                            MoveDifferential, SpeedRPM)
from ev3dev2.port import LegoPort
from ev3dev2.sensor.lego import UltrasonicSensor, GyroSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.wheel import EV3EducationSetTire
import math
import paho.mqtt.client as mqtt
from smbus import SMBus
import time

#   Mode Library    The modes that define the bot's behavior
#       Pre-Run: Preparing functions, classes, hardware, and calibrations.
#       Initiation: Preparing variables
#       Wander: Bumbling about looking for stuff

mode="Pre-Run"
print("Mode set: Pre-Run")
#   Classes
#   Enter new classes in alphabetical order

class Comms():
    def __init__(self) -> None:
        self.client = mqtt.Client()
        broker_address="192.168.7.212"
        self.client.connect(broker_address)
        def on_message(client, userdata, message):
            str(message.payload.decode("utf-8"))
            global recieved
            recieved=int(message.payload.decode("utf-8"))
            print(recieved)
        self.client.on_message=on_message
        self.client.subscribe('Computer_to_bot')
        self.client.loop_start()
    def send_coords(self,string):
        self.client.publish("Bot_to_computer",string)

class Snoofer():
    def __init__(self, port='pistorms:BBS1', mode='ev3-uart') -> None:
        self.port=port
        self.mode=mode
        set_port('pistorms:BBS1','ev3-uart', device='lego-ev3-us')
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

#   Functions

def get_snoofer_angle():
    motor_angle_deg=snoofermotor.position
    motor_angle=math.radians(motor_angle_deg)
    snoofer_angle=-17.866*math.cos(motor_angle+0.0626)-1.3659
    return snoofer_angle

def set_port(port,mode,device): 
#   Function to initiate sensors; port should be INPUT_N where N is 1 to 4
#   mode is a string describing which driver to use (see https://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors)
#   device is also a string, see site above
    sensor=LegoPort(port)
    sensor.mode=mode
    if device=='lego-ev3-gyro':
        sensor.set_device=device
    time.sleep(0.5)

def zero_in():
    drive.on(15,15)
    while snoofer.distance_centimeters>4:
        left_output=left_whisker.read()
        right_output=right_whisker.read()
        if left_output>0:
            drive.off()
            time.sleep(1)
            drive.on_for_distance(24, -50, brake=False, block=True)
            time.sleep(0.2)
#           IMPORTANT: Do not enable block on following turn command, will hold up forever.
            drive.turn_degrees(20, -20, block=False, use_gyro=False)
            time.sleep(0.5)
            print('Im TRYING to drive')
            drive.on(15,15)
        if right_output>0:
            drive.off()
            time.sleep(1)
            drive.on_for_distance(24, -50, brake=False, block=True)
            time.sleep(0.2)
#           IMPORTANT: Do not enable block on following turn command, will hold up forever.
            drive.turn_degrees(20, 20, block=False, use_gyro=False)
            time.sleep(0.5)
            print('Im TRYING to drive')
            drive.on(15,15)

#   Hardware

button=Button()
bot_placed=False
drive=MoveDifferential(OUTPUT_C, OUTPUT_D, EV3EducationSetTire, 152)
gyroport=set_port(port='pistorms:BAS1', mode='ev3-uart', device='lego-ev3-gyro')
gyro=GyroSensor('pistorms:BAS1')
drive.gyro=gyro
sensorport=set_port(port='pistorms:BBS1', mode='ev3-uart', device='lego-ev3-us')
sensor=UltrasonicSensor('pistorms:BBS1')
snoofermotor=Motor(OUTPUT_A)
time.sleep(2)
set_port('pistorms:BBS1','ev3-uart', device='lego-ev3-us')
snoofer=UltrasonicSensor("pistorms:BBS1")
whiskers_port=set_port('pistorms:BBS2',"i2c-thru", "Custom")
left_whisker=Whisker(multiplexor_port=2, name="LW")
left_whisker.selftest()
left_whisker.calibrate()
right_whisker=Whisker(multiplexor_port=1, name="RW")
right_whisker.selftest()
right_whisker.calibrate()
time.sleep(0.5)

#   Snoofer Calibration

print("------\nUse whiskers to move snoofer so black part from motor faces out, parallel to grey peice below it. Press GO when done.\n------")
while mode=="Pre-Run":
    left_output=left_whisker.read()
    right_output=right_whisker.read()
    if left_output>0 and right_output>0:
        snoofermotor.stop()
        print("Only press one whisker at a time!")

    if left_output==1:
        snoofermotor.run_forever(speed_sp=100)
    elif left_output==2:
        snoofermotor.run_forever(speed_sp=200)
    elif right_output==1:
        snoofermotor.run_forever(speed_sp=-100)
    elif right_output==2:
        snoofermotor.run_forever(speed_sp=-200)
    else:
        snoofermotor.stop()
    if button.enter:
        mode="Initiation"
        print("Mode set: Initiation")
        drive.odometry_start()
        gyro.calibrate()
    time.sleep(0.1)


print("Place bot in start position \n Press GO when done")
while bot_placed==False:
    if button.enter:
        bot_placed=True
        mode="Wander"
while mode=="Wander":
    snoofermotor.on(40)
    drive.on(40,40)
    left_output=left_whisker.read()
    right_output=right_whisker.read()
    if snoofer.distance_centimeters<20:
        zero_in()
    if left_output>0:
        zero_in()
    if right_output>0:
        zero_in()