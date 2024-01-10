#!/usr/bin/env python3
from ev3dev2.motor import (Motor, OUTPUT_C, OUTPUT_D, MoveDifferential)
from ev3dev2.wheel import EV3EducationSetTire
import time
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.port import LegoPort
import math
import _thread
from mpu6050 import mpu6050
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
            while True:
                all_rotations=gyro.get_gyro_data()
                current_angle_prime=all_rotations['z']
                current_time=time.time()
                self.circle_angle+=(((current_angle_prime+previous_angle_prime)/2)-angle_error)*(current_time-previous_time)
                previous_time=current_time
                previous_angle_prime=current_angle_prime
                if self.calibrated==False:
                    calibration_list.append(current_angle_prime)
                    time.sleep(0.01)
                    if len(calibration_list)>1000:
                        angle_error=sum(calibration_list)/len(calibration_list)
                        self.calibrated=True
                        print('Gyro calibration complete') #NEVER GETTING HERE
                        print(angle_error)
        self.circle_angle=0
        _thread.start_new_thread(_gyro_monitor,())
        while self.calibrated==False:
            pass


def set_port(port,mode,device): 
#   Function to initiate sensors; port should be INPUT_N where N is 1 to 4
#   mode is a string describing which driver to use (see https://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors)
#   device is also a string, see site above
    sensor=LegoPort(port)
    sensor.mode=mode
    if device=='lego-ev3-gyro':
        sensor.set_device=device
    time.sleep(0.5)

gyroport=set_port(port='pistorms:BAS1', mode='i2c-thru', device='Custom')
gyro=Gyro()

def drive_precise(distance=1):
#Function to drive in straight line, even after turning
    reference_frame='normal'
    distance_covered=0
    start_pos=[drive.x_pos_mm,drive.y_pos_mm]
    target_angle=gyro.circle_angle
    current_angle=gyro.circle_angle
    turn_force=0
    drive.on(30,30)
    if target_angle<12 or target_angle>348:
        reference_frame='reversed'
    if reference_frame=='normal':
        while distance_covered<distance:
            turn_force=target_angle-current_angle
            #print(target_angle)
            #print(current_angle)
            print(turn_force)
            #print("----------")
            time.sleep(0.2)
            if abs(turn_force)>40:
                drive.off()
                time.sleep(20)
            drive.on(30-turn_force,30+turn_force)
            distance_covered=math.sqrt(math.pow(start_pos[0]-drive.x_pos_mm,2)+math.pow(start_pos[1]-drive.y_pos_mm,2))
            current_angle=gyro.circle_angle
            print(current_angle)
            print('------')
    if reference_frame=='reversed':
        if target_angle>180:
            target_angle-=360
        while distance_covered<distance:
            if gyro.circle_angle>180:
                current_angle=gyro.circle_angle-360
            else:
                current_angle=gyro.circle_angle
            turn_force=int((target_angle-current_angle)/2)
            drive.on(30-turn_force,30+turn_force)
            distance_covered=math.sqrt(math.pow(start_pos[0]-drive.x_pos_mm,2)+math.pow(start_pos[1]-drive.y_pos_mm,2))


drive=MoveDifferential(OUTPUT_C, OUTPUT_D, EV3EducationSetTire, 152)
drive.gyro=gyro
time.sleep(2)
drive.turn_degrees(20,360,use_gyro=True,block=False)
time.sleep(5)
drive.off
time.sleep(1)
drive_precise(3000)
#drive.on_for_distance(30,3000)
time.sleep(5)