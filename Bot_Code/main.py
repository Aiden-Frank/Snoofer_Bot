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
import numpy as np
import paho.mqtt.client as mqtt
import random
from smbus import SMBus
import time


#   Mode Library    The modes that define the bot's behavior
#       Pre-Run: Preparing functions, classes, hardware, and calibrations.
#       Initiation: Preparing variables
#       Wander: Bumbling about looking for stuff

mode="Pre-Run"
print("Mode set: Pre-Run")
object_found=False
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
    #Note: LiDAR sensor accuracy changes with object angular width
    def __init__(self,bus=SMBus(1)):
        self.bus=bus
        self.raw_values_list=[6.2,11.1,12.3,12.9,13.9,14.7,16.3,17.7,18.6,19.5,20.5,21.5,22.4,23.1,24.2,25.9,26.6,27.5,28.7,29.0,33.6,39.1,43.4,47.1,51.5,54.2,55.3,58.6,61.7,63,64.1,64.9,66.2,67.6,69.4,78.6,88.5,98.8,109,114.2,123.8,137.4,142.4,148.8,159.3,172.6,182.7,191.5,194]
        self.corrected_values_list=[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,25,30,35,40,45,50,51,52,53,54,55,56,57,58,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200]
        self.raw_values=np.asarray(self.raw_values_list)
        self.corrected_values=np.asarray(self.corrected_values_list)
    def read_direct(self):
        #Reads sensor output in cm, which is often off by 9-10 cm
        self.bus.write_byte_data(0x62, 0x00, 0x04)
        while (self.bus.read_byte_data(0x62,0x01)&1)==True:
            time.sleep(0.05)
        readout_low_byte,readout_high_byte=self.bus.read_i2c_block_data(0x62,0x10,2)
        readout=readout_low_byte+(readout_high_byte<<8)
        return int(readout)
    def read_corrected(self):
        #Attempts to correct sensor output, returns cm
        raw_readout=self.read_direct()
        corrected_readout=np.interp([raw_readout], self.raw_values, self.corrected_values, right=raw_readout)
        return int(corrected_readout)
class Whisker():
    def __init__(self, multiplexor_port=1,bus=SMBus(1), name="W") -> None:
        self.name=name
        self.multiplexor_port=multiplexor_port
        self.port_address=int(math.pow(2, self.multiplexor_port))
        self.bus=bus
#       Write to multiplexor, which is at 0x70 by default. to tell it 
#       which port set to use at this moment
        self.bus.write_byte_data(0x70, 0, self.port_address)
#       Write to sensor, which is at 0x21, setting mode to read analog voltage
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

#   Functions

def get_snoofer_angle():
    motor_angle_deg=snoofermotor.position()
    motor_angle=math.radians(motor_angle_deg)
    snoofer_angle=-17.866*math.cos(motor_angle+0.0626)-1.3659
    return snoofer_angle

def random_turn():
    magnitude=10*random.randint(4,18)
    direction=random.randint(1,2)
    if direction==2:
        direction=-1
    drive.turn_degrees(20, direction*magnitude, block=True)
    time.sleep(magnitude/60)
    print ("Turn argument:")
    print (direction*magnitude)
    print("I'm done with the turn")
    return

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
    drive.stop()
    print("Object detected")
    global object_found
    snoofermotor.stop()
    time.sleep(0.5)
    snoofer_stopped=False
    snoofermotor.on(30)
    while snoofer_stopped==False:
        snooferpos=snoofermotor.position%360
        if snooferpos==92:
            snoofermotor.stop()
            snoofer_stopped=True
            print(snooferpos)
    print("The snoofer should be straight")
    drive.on(15,15)
    while snoofer.read_corrected()>6:
        left_output=left_whisker.read()
        right_output=right_whisker.read()
        time.sleep(0.2)
        print(snoofer.read_corrected())
        if left_output>0:
            time.sleep(0.2)
            drive.on_for_distance(24, -50, brake=False, block=True)
            time.sleep(0.2)
#           IMPORTANT: Do not enable block on following turn command, will hold up forever.
            drive.turn_degrees(20, -20, block=False, use_gyro=False)
            time.sleep(0.5)
            drive.on(15,15)
        if right_output>0:
            drive.off()
            time.sleep(0.3)
            drive.on_for_distance(24, -50, brake=False, block=True)
            time.sleep(0.2)
#           IMPORTANT: Do not enable block on following turn command, will hold up forever.
            drive.turn_degrees(20, 20, block=False, use_gyro=False)
            time.sleep(0.2)
            drive.on(15,15)
    object_found=True
    drive.off()
    drive.on_for_distance(36, -360)
    time.sleep(0.5)
    drive.off()
    random_turn()
    drive.off()
    snoofermotor.on(30)
    drive.on(30,30)
    print("Zero_in funstion complete")
    return

#   Hardware


button=Button()
left_motor=Motor(OUTPUT_C)
left_motor.stop_action='brake'
right_motor=Motor(OUTPUT_D)
right_motor.stop_action='brake'
drive=MoveDifferential(OUTPUT_C, OUTPUT_D, EV3EducationSetTire, 152)
gyroport=set_port(port='pistorms:BAS1', mode='ev3-uart', device='lego-ev3-gyro')
gyro=GyroSensor('pistorms:BAS1')
drive.gyro=gyro
sensorport=set_port(port='pistorms:BBS1', mode='ev3-uart', device='lego-ev3-us')
sensor_connected=False
snoofermotor=Motor(OUTPUT_A)
snoofermotor.stop_action='brake'
time.sleep(2)
snoofer_port=set_port('pistorms:BBS1','i2c-thru','Custom')
snoofer=Snoofer()
whiskers_port=set_port('pistorms:BBS2',"i2c-thru", "Custom")
left_whisker=Whisker(multiplexor_port=2, name="LW")
left_whisker.selftest()
left_whisker.calibrate()
right_whisker=Whisker(multiplexor_port=1, name="RW")
right_whisker.selftest()
right_whisker.calibrate()
print(left_motor.state)
time.sleep(0.5)

#print('Entering snoofer testing mode')
# while True:
#     left_output=left_whisker.read()
#     if left_output>0:
#         time.sleep(0.1)
#         print('Sampling\n(This will take 12 seconds)')
#         iterations=24
#         #cycles_complete=0
#         #readout_sum=0
#         #corrected_readout_sum=0
#         #while cycles_complete<24:
#         readout=snoofer.read_direct()
#             #readout_sum+=readout
#         corrected_readout=snoofer.read_corrected()
#             #corrected_readout_sum+=corrected_readout
#             #cycles_complete+=1
#         print(corrected_readout)
#             #time.sleep(0.01)
#         #print(readout_sum/iterations)
#         #print(corrected_readout_sum/iterations)
#     time.sleep(0.3)


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
        snoofermotor.position=0
        print(snoofermotor.position)
    time.sleep(0.1)

mode="Wander"
print('Mode set: Wander')
snoofermotor.on(30)
drive.on(30,30)
while mode=="Wander":
    object_found=False
    left_output=left_whisker.read()
    right_output=right_whisker.read()
    if snoofer.read_corrected()<20 and object_found==False:
        zero_in()
        print("I have escaped the loop!")
    if left_output>0 and object_found==False:
        zero_in()
        print("I have escaped the loop!")
    if right_output>0 and object_found==False:
        zero_in()
        print("I have escaped the loop!")