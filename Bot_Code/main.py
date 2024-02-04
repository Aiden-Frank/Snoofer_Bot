#!/usr/bin/env python3
#This code is intended for a Lego Mindstorms robot

from ev3dev2.button import Button
from ev3dev2.motor import (Motor, OUTPUT_A, OUTPUT_C, OUTPUT_D,
                            MoveDifferential)
from ev3dev2.port import LegoPort
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.wheel import EV3EducationSetTire
import math
from mpu6050 import mpu6050
import numpy as np
import paho.mqtt.client as mqtt
import random
from smbus import SMBus
import _thread
import threading
import time

#Control variables
operation_mode='Wander'
use_snoofermotor_and_whiskers=False


#   Modes:    The modes that define the bot's behavior
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
            global recieved
            recieved=int(message.payload.decode("utf-8"))
            print(recieved)
        self.client.on_message=on_message
        self.client.subscribe('Computer_to_bot')
        self.client.loop_start()
    def send_data(self,data,type):
        coord_list=[]
        #First item is list indicates list type
        if type=='telemetry':
            coord_list.append('0')
        elif type=="raw_array":
            coord_list.append('1')
        else:
            raise ValueError('Unrecognized input for type')

        for k in data:
            coord_list.append(str(int(k)))
        coord_string=','.join(coord_list)
        self.client.publish("Bot_to_computer",coord_string)

class Gyro:
    def __init__(self) -> None:
        gyroport=set_port(port='pistorms:BAS1', mode='i2c-thru', device='Custom')
        gyro=mpu6050(0x68)
        #Setting range to appropriate quantity
        gyro.set_gyro_range==gyro.GYRO_RANGE_250DEG
        self.circle_angle=0
        self.calibrated=False
        def _gyro_monitor():
            previous_angle_prime=0.0
            previous_time=time.time()
            calibration_list=[]
            angle_error=0.0
            print("Calibrating gyro")
            print("Press GO to skip remaining calibration")
            print('Progress:')
            while True:
                all_rotations=gyro.get_gyro_data()
                current_angle_prime=all_rotations['z']
                current_time=time.time()
                self.circle_angle+=(((current_angle_prime+previous_angle_prime)/2)-angle_error)*(current_time-previous_time)
                previous_time=current_time
                previous_angle_prime=current_angle_prime
                if self.circle_angle<0:
                    self.circle_angle+=360
                elif self.circle_angle>360:
                    self.circle_angle-=360
                if self.calibrated==False:
                    calibration_list.append(current_angle_prime)
                    time.sleep(0.01)
                    length=len(calibration_list)
                    if length/5-int(length/5)==0:
                        print(int(length/7.19),'%',end='\r')
                    if len(calibration_list)>719 or button.enter:
                        angle_error=sum(calibration_list)/len(calibration_list)
                        self.calibrated=True
                        print('Gyro calibration complete')
                        print('Samples:',len(calibration_list))
                        print("Correction factor:",angle_error)
        _thread.start_new_thread(_gyro_monitor,())
        while self.calibrated==False:
            pass
        self.circle_angle=90
class Snoofer():
    #Note: LiDAR sensor accuracy changes with object angular width
    def __init__(self,bus=SMBus(1)):
        self.bus=bus
        self.sensor_length_cm=6
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
    def read_corrected(self,add_sensor_length=False):
        #Attempts to correct sensor output, returns cm
        raw_readout=self.read_direct()
        corrected_readout=np.interp([raw_readout], self.raw_values, self.corrected_values, right=raw_readout)
        if add_sensor_length==True:
            corrected_readout+=6
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
stop_drive_thread=False
thread_stop_time=0
def drive_precise(distance=1):
#Function to drive in straight line, even after turning
    def _drive_precise_thread(distance):
        global stop_drive_thread
        stop_drive_thread=False
        print("Driving in a precise manner")
        reference_frame='normal'
        distance_covered=0
        start_pos=[drive.x_pos_mm,drive.y_pos_mm]
        target_angle=gyro.circle_angle
        current_angle=gyro.circle_angle
        turn_force=0
        drive.on(30,30)
        if target_angle<12 or target_angle>348:
            reference_frame='reversed'
        print(target_angle)
        if reference_frame=='normal':
            print("Reference frame- normal")
            while distance_covered<distance and stop_drive_thread==False:
                turn_force=target_angle-current_angle
                drive.on(30-turn_force,30+turn_force)
                distance_covered=math.sqrt(math.pow(start_pos[0]-drive.x_pos_mm,2)+math.pow(start_pos[1]-drive.y_pos_mm,2))
                current_angle=gyro.circle_angle
                if stop_drive_thread==True:
                    drive.off()
                    return
        if reference_frame=='reversed':
            print("Reference frame- reversed")
            if target_angle>180:
                target_angle-=180
            else:
                target_angle+=180
            while distance_covered<distance and stop_drive_thread==False:
                turn_force=target_angle-current_angle
                current_angle=gyro.circle_angle
                if current_angle>180:
                    current_angle-=180
                else:
                    current_angle+=180
                turn_force=target_angle-current_angle
                drive.on(30-turn_force,30+turn_force)
                distance_covered=math.sqrt(math.pow(start_pos[0]-drive.x_pos_mm,2)+math.pow(start_pos[1]-drive.y_pos_mm,2))
        print("Stopping drive thread")
        drive.off()

#Code to run function
    drive_thread=threading.Thread(target=_drive_precise_thread, args=(distance,))
    drive_thread.start()
    print("Starting drive thread")
    while drive_thread.is_alive==False:
        time.sleep(0.05)  

def find_home():
    snoofermotor.off()
    xlist=[]
    xarray=np.asarray(xlist)
    ylist=[]
    yarray=np.asarray(ylist)
    distance_list=[]
    home_found=False
    print(gyro.circle_angle)
    print(drive.x_pos_mm)
    print(drive.y_pos_mm)
    distance_to_home=np.sqrt(drive.x_pos_mm*drive.x_pos_mm+drive.y_pos_mm*drive.y_pos_mm)
    target_angle_deg=180+math.degrees(math.atan2(drive.y_pos_mm,drive.x_pos_mm))
    print("Target Angle:",target_angle_deg)
    print('Gyro Angle:',gyro.circle_angle)
    turn_angle_deg=(target_angle_deg-gyro.circle_angle)*-1
    print('Turn angle:',turn_angle_deg)
    drive.turn_degrees(use_gyro=True,speed=30,degrees=turn_angle_deg,block=False)
    time.sleep(3)
    print("Final angle:",gyro.circle_angle)
    drive_precise(distance=distance_to_home)
    print('Driving')
    snoofermotor.on(30)
    while home_found==False:
        distance=snoofer.read_corrected()
        if distance<20:
            home_found=True
    global stop_drive_thread
    stop_drive_thread=True
    drive.off()
    snoofermotor.off()
    snoofermotor.on_for_degrees(10,360,block=False,brake=False)
    while snoofermotor.is_running==False:
        pass
    while snoofermotor.is_running==True:
        time.sleep(0.06)
        angle_deg=get_snoofer_angle()
        distance=(snoofer.read_corrected())
        if distance<40:
            angle=(math.radians(angle_deg))
            xarray=np.append(xarray, distance*math.cos(angle))
            yarray=np.append(yarray, distance*math.sin(angle))
    point_distance_array=np.asarray([])
    for k in range(xarray.size):
        point_distance_array=np.append(point_distance_array, np.sqrt((xarray[k]**2)+(yarray[k]**2)))
    min_distance_index=np.argmin(point_distance_array)
    closest_point=[xarray[min_distance_index], yarray[min_distance_index]]
    print(closest_point)
    angle_to_turn=np.rad2deg(np.arctan2(closest_point[0], closest_point[1]))-90
    print(angle_to_turn)
    drive.turn_degrees(20,angle_to_turn,block=False)
    time.sleep(1.5)
    snoofermotor.on_for_degrees(10,360,block=False,brake=False)
    while snoofermotor.is_running==False:
        pass
    while snoofermotor.is_running==True:
        time.sleep(0.06)
        angle_deg=get_snoofer_angle()
        distance=(snoofer.read_corrected(add_sensor_length=True))
        if distance<40:
            angle=(math.radians(angle_deg))
            distance_list.append(distance)
            xarray=np.append(xarray, distance*math.cos(angle))
            yarray=np.append(yarray, distance*math.sin(angle))
    distance_array=np.asarray(distance_list)
    home_center=[xarray[np.argmin(distance_array)],yarray[np.argmin(distance_array)]]
    distance_to_home=np.sqrt(home_center[0]**2+home_center[1]**2)+6 #6 is radius of home
    bot_angle_from_home=np.deg2rad(gyro.circle_angle+180)
    print(np.rad2deg(bot_angle_from_home))
    drive.x_pos_mm=distance_to_home*np.cos(bot_angle_from_home)*10
    drive.y_pos_mm=distance_to_home*np.sin(bot_angle_from_home)*10
    print(drive.x_pos_mm)
    print(drive.y_pos_mm)
    print('---------')  #Working, but new position may not be on same axes as old one
    print(np.min(distance_to_home))
    print(drive.x_pos_mm/10)
    print(drive.y_pos_mm/10)
    time.sleep(144)



def get_snoofer_angle():
    #Returns relative snoofer angle in degrees
    motor_angle_deg=float(snoofermotor.position)
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

def telemetry(transmit=True,mode="absolute"):
    snoofer_angle=get_snoofer_angle()
    snoofer_distance=snoofer.read_corrected(add_sensor_length=True)
    if mode=='absolute':
        bot_x=drive.x_pos_mm/10.0
        bot_y=drive.y_pos_mm/10.0
        snoofer_angle+=gyro.circle_angle
    object_rel_x=math.cos(snoofer_angle)*(snoofer_distance)
    object_rel_y=math.sin(snoofer_angle)*(snoofer_distance)
    if mode=="absolute":
        object_x=object_rel_x+bot_x
        object_y=object_rel_y+bot_y
    elif mode=='relative':
        object_x=object_rel_x
        object_y=object_rel_y
    if transmit==True:
        array_to_send=[object_x,object_y,bot_x,bot_y,left_motor.position/360,right_motor.position/360]
        comms.send_data(array_to_send,'telemetry')
        #print('Outgoing array:',array_to_send)
    return object_x, object_y

def zero_in():
    global stop_drive_thread
    stop_drive_thread=True
    drive.stop()
    global object_found
    snoofermotor.stop()
    time.sleep(0.5)
    snoofer_stopped=False
    if mode=="Map":
        while snoofer_stopped==False:
            snooferpos=snoofermotor.position%360
            snoofermotor.on(20)
            if snooferpos<94 and snooferpos>90:
                snoofermotor.stop()
                snoofer_stopped=True
        drive.on(15,15)
        while snoofer.read_corrected()>6:
            left_output=left_whisker.read()
            right_output=right_whisker.read()
            time.sleep(0.2)
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
    #           Do not enable block on following turn command, will hold up forever.
                drive.turn_degrees(20, 20, block=False, use_gyro=False)
                time.sleep(0.2)
                drive.on(15,15)
        telemetry()
        object_found=True
        drive.off()
    drive.on_for_distance(36, -360)
    time.sleep(0.5)
    drive.off()
    random_turn()
    drive.off()
    drive_precise(98989898)
    if mode=='Map' or use_snoofermotor_and_whiskers==True:
        snoofermotor.on(30)
    return

#   Hardware

#Note:  Motors have 'inertia' after turning in one direction, if told to turn in the other, 
# they will take a bit to speed up all the way
# The drive_precise function counters this effect


button=Button()
left_motor=Motor(OUTPUT_C)
left_motor.stop_action='brake'
right_motor=Motor(OUTPUT_D)
right_motor.stop_action='brake'
drive=MoveDifferential(OUTPUT_C, OUTPUT_D, EV3EducationSetTire, 152)
time.sleep(1)
sensorport=set_port(port='pistorms:BBS1', mode='ev3-uart', device='lego-ev3-us')
sensor_connected=False
snoofermotor=Motor(OUTPUT_A)
snoofermotor.stop_action='brake'
time.sleep(1)
gyroport=set_port(port='pistorms:BAS1', mode='i2c-thru', device='Custom')
gyro=Gyro()
comms=Comms()
snoofer_port=set_port('pistorms:BBS1','i2c-thru','Custom')
snoofer=Snoofer()
whiskers_port=set_port('pistorms:BBS2',"i2c-thru", "Custom")
left_whisker=Whisker(multiplexor_port=2, name="LW")
left_whisker.selftest()
left_whisker.calibrate()
right_whisker=Whisker(multiplexor_port=1, name="RW")
right_whisker.selftest()
right_whisker.calibrate()
time.sleep(1)

objects_found=0
#   Snoofer Calibration

print("------\nUse whiskers to move snoofer so black\npart from motor faces out, parallel to\ngrey peice below it. Press GO when done.\n------")
while mode=="Pre-Run":
    left_output=left_whisker.read()
    right_output=right_whisker.read()
    if left_output>0 and right_output>0:
        snoofermotor.stop()
        print("Only press one whisker at a time!")
    if left_output==1:
        snoofermotor.run_forever(speed_sp=72)
    elif left_output==2:
        snoofermotor.run_forever(speed_sp=172)
    elif right_output==1:
        snoofermotor.run_forever(speed_sp=-72)
    elif right_output==2:
        snoofermotor.run_forever(speed_sp=-172)
    else:
        snoofermotor.stop()
    if button.enter:
        mode="Initiation"
        print("Mode set: Initiation")
        drive.odometry_start(sleep_time=0.04,use_gyro=True)
        drive.gyro=gyro
        drive.y_pos_mm=270
        snoofermotor.position=0
    time.sleep(0.1)

mode=operation_mode
print("Mode set: ",operation_mode)
drive_precise(9898989)
left_output=0
right_output=0
if mode=='Map' or use_snoofermotor_and_whiskers==True:
    snoofermotor.on(30)
else:
    snoofermotor.on_to_position(30,92,block=False)
while mode=="Map":
    if objects_found>2:
        stop_drive_thread=True
        time.sleep(1)
        print('Finding home')
        time.sleep(1)
        find_home()
    object_found=False
    left_output=left_whisker.read()
    right_output=right_whisker.read()
    if snoofer.read_corrected()<20 and object_found==False:
        zero_in()
        objects_found+=1
    elif left_output>0 and object_found==False:
        zero_in()
        objects_found+=1
    elif right_output>0 and object_found==False:
        zero_in()
        objects_found+=1
print("VRMSK")
while mode=="Wander":
    if use_snoofermotor_and_whiskers==True:
        left_output=left_whisker.read()
        right_output=right_whisker.read()
    if snoofer.read_corrected()<20:
        zero_in()
    elif left_output>0:
        zero_in()
    elif right_output>0:
        zero_in()