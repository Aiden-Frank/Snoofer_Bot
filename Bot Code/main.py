#!/usr/bin/env python3
#This code is intended for a Lego Mindstorms robot

#First, only display programs are imported, so that the bot can display on screen
# how much it has imported.
from ev3dev2.display import Display
from ev3dev2.display import fonts
myfont=fonts.load('lutBS18')
show=Display()
#Then everything else is imported, while the screen displays progress
show.text_pixels("Importing \n 1/11", x=20, y=20, font=myfont)
show.update()
from ev3dev2.motor import (Motor, OUTPUT_A, OUTPUT_B, OUTPUT_D,
                            MoveDifferential, SpeedRPM)
show.text_pixels("Importing \n 2/11", x=20, y=20, font=myfont)
show.update()
from ev3dev2.sensor.lego import UltrasonicSensor, GyroSensor, TouchSensor
show.text_pixels("Importing \n 3/11", x=20, y=20, font=myfont)
show.update()
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
show.text_pixels("Importing \n 4/11", x=20, y=20, font=myfont)
show.update()
import random
show.text_pixels("Importing \n 5/11", x=20, y=20, font=myfont)
show.update()
import math
show.text_pixels("Importing \n 6/11", x=20, y=20, font=myfont)
show.update()
import time
show.text_pixels("Importing \n 7/11", x=20, y=20, font=myfont)
show.update()
from ev3dev2.wheel import EV3EducationSetTire
show.text_pixels("Importing \n 8/11", x=20, y=20, font=myfont)
show.update()
from ev3dev2.button import Button
show.text_pixels("Importing \n 9/11", x=20, y=20, font=myfont)
show.update()
from ev3dev2.sound import Sound
show.text_pixels("Importing \n 10/11", x=20, y=20, font=myfont)
show.update()
import paho.mqtt.client as mqtt
show.text_pixels("Importing \n 11/11", x=20, y=20, font=myfont)
show.update()

initiator=0
show.clear()
show.update()
#This code is for transmitting data
client = mqtt.Client()
client.connect("192.168.7.154")
recieved=0
def on_message(client, userdata, message):
    str(message.payload.decode("utf-8"))
    global recieved
    recieved=int(message.payload.decode("utf-8"))
    print(recieved)
#Address of EV3 broker
broker_address="192.168.7.154"
#Create a client that will recieve messages
client=mqtt.Client()
#Set on_message funcion of client to my on_message function
client.on_message=on_message
#Connects to ev3 broker
client.connect(broker_address)
#Subscribe to channel
client.subscribe('snoofer_bot_reception')
#Starts loop to handle callbacks
client.loop_start()
# This is the Publisher
client = mqtt.Client()
client.connect("192.168.7.154")

#Functions

#This function returns the relative angle of the ultrasonic sensor based on
#the position of the motor that turns it.
def get_snoofer_angle(motorpos):
    step_1=motorpos+15
    step_2=math.radians(step_1)
    step_3=math.cos(step_2)*-19
    return step_3

#This function uses the whiskers to align the utrasound sensor roughly 
# perpendicular to the object
def zero_in():
    move.on(15,15)
    while sensor.distance_centimeters>4:
        if left_whisker.is_pressed==True:
            move.off()
            move.on_for_distance(27, -20, brake=True, block=True)
            move.turn_degrees(30, -20)
            move.on(15,15)
        if right_whisker.is_pressed==True:
            move.off()
            move.on_for_distance(27, -20, brake=True, block=True)
            move.turn_degrees(30, 20)
            move.on(15,15)

#This function returns a random value across a certain set, to be the number of
# degrees the bot turns
def turn():
    turn_angle=10*random.randrange(4,19)
    turn_direction_decider=random.randrange(1,3)
    if turn_direction_decider==1:
        turn_direction=-1
    if turn_direction_decider==2:
        turn_direction=1
    final_turn=turn_angle*turn_direction
    return final_turn

#This function uses four data points to locate objects.
def telemetry(robot_x,robot_y,s_ang,s_dist):
    object_x=math.cos(s_ang)*(s_dist)
    object_y=math.sin(s_ang)*(s_dist)
    f.write(str("\n Relative coordinates:"))
    f.write(str(object_x) + '\n')
    f.write(str(object_y) + '\n')
    object_x+=robot_x
    object_y+=robot_y
    return object_x, object_y

#This function is designed to send pairs of numbers.
def send_coords(string,broker_adress):
    client.publish("snoofer_bot_transmission",string)
#Creates a file to record data
f = open('inputfile', 'w')
#Sensor initiation and position calibration
while continue_boot==-1:
        try:
            snoofermotor=Motor(OUTPUT_B)
            continue_boot=0
        except:
            show.text_pixels("\nSnoofer motor\nis not connected\n to port B.\n\n      Fix it.", x=10, y=10, font=myfont)
            show.update()
show.text_pixels("Use side buttons to \n move snoofer so \n orange pointer is \n over orange peg. \n press mid button \n when done", x=10, y=10, font=fonts.load('lutBS14'))
show.update()
button=Button()
continue_boot=-1
while initiator==0:
    time.sleep(0.1)
    if button.enter:
        time.sleep(2)
        initiator=1
    if button.left:
        snoofermotor.run_forever(speed_sp=240)
    if button.right:
        snoofermotor.run_forever(speed_sp=-240)
    if button.left==False and button.right==False:
        snoofermotor.stop()
        
#Preparing variables
if initiator==1:
    show.clear()
    pre_terminate=0
    iterations=24 #This is how many objects the bot will find before it stops.
    random.seed(12)
    show.update()
    object_found=0
    bot_x=0
    bot_y=0
#Testing sensor connections
    while continue_boot==0:
        try:
            sensor=UltrasonicSensor(INPUT_2)
            continue_boot=1
        except:
            show.text_pixels("\nGyroSensor\nis not connected\n to port 2.\n\n      Fix it.", x=10, y=10, font=myfont)
            show.update()
    while continue_boot==1:
        try:
            left_whisker=TouchSensor(INPUT_4)
            continue_boot=2
        except:
            show.text_pixels("\nLeft TouchSensor\nis not connected\n to port 4.\n\n      Fix it.", x=10, y=10, font=myfont)
            show.update()
    while continue_boot==2:
        try:
            right_whisker=TouchSensor(INPUT_3)
            continue_boot=3
        except:
            show.text_pixels("\nRight TouchSensor\nis not connected\n to port 3.\n\n      Fix it.", x=10, y=10, font=myfont)
            show.update()
    snoofermotor.position=0
    while continue_boot==3:
        try:
            gyro=GyroSensor(INPUT_1)
            continue_boot=4
        except:
            show.text_pixels("\nGyroSensor\nis not connected\n to port 1.\n\n      Fix it.", x=10, y=10, font=myfont)
            show.update()
    move=MoveDifferential(OUTPUT_A, OUTPUT_D, EV3EducationSetTire, 152)
    while continue_boot==4:
        try:
            leftmotor=Motor(OUTPUT_A)
            continue_boot=5
        except:
            show.text_pixels("\nLeft drive motor\nis not connected\n to port A.\n\n      Fix it.", x=10, y=10, font=myfont)
            show.update()
    while continue_boot==5:
        try:
            rightmotor=Motor(OUTPUT_D)
            continue_boot=6
        except:
            show.text_pixels("\nRight drive motor\nis not connected\n to port D.\n\n      Fix it.", x=10, y=10, font=myfont)
            show.update()
#More variable setting
    terminate=0
    move.odometry_start()
    move.gyro=gyro
    noise=Sound()
    gyro.calibrate()
    gyro.reset()
    time.sleep(1)
#Main drive code
    while terminate==0:
        move.on(40,40)
        snoofermotor.run_forever(speed_sp=360)
        distance=sensor.distance_centimeters
        #The following if/elif/else activates the zero_in function after a few
        #preperatory steps.
        if left_whisker.is_pressed==True:
            snoofermotor.stop()
            move.off()
            snoofermotor.position=snoofermotor.position%360
            snoofermotor.run_to_abs_pos(position_sp=70,speed_sp=360)
            move.on_for_distance(27, 20, brake=True, block=True)
            zero_in()
            object_found=1
           
        elif right_whisker.is_pressed==True:
            snoofermotor.stop()
            move.off()
            snoofermotor.position=snoofermotor.position%360
            snoofermotor.run_to_abs_pos(position_sp=70,speed_sp=360)
            move.on_for_distance(27, -20, brake=True, block=True)
            zero_in()
            object_found=1
        
        elif sensor.distance_centimeters<36:
            snoofermotor.stop()
            snoofermotor.position=snoofermotor.position%360
            snoofermotor.run_to_abs_pos(position_sp=70,speed_sp=360)
            zero_in()
            object_found=1
        else: 
            pass
        if object_found==1:
            #This code locates the object.
            move.off()
            snoofer_ang=math.radians(get_snoofer_angle(snoofermotor.position))
            gyro_angle=math.radians(-gyro.angle+90)
            snoofer_ang_final=snoofer_ang+gyro_angle
            snooferdistance=(sensor.distance_centimeters*10)+85
            bot_x=move.x_pos_mm
            bot_y=move.y_pos_mm
            object_x,object_y=telemetry(bot_x,bot_y,snoofer_ang_final,snooferdistance)
            noise.beep()
            object_x_str=str(math.trunc(object_x))
            object_y_str=str(math.trunc(object_y))
            #Coordinate transmission
            coord_str=object_x_str + ',  ' + object_y_str
            send_coords(coord_str,broker_address)
            #File writing
            f.write(str('Bot Position:'))
            f.write(str(bot_x) + '\n')
            f.write(str(bot_y) + '\n')
            #f.write(str(temp)+ '\n')
            #f.write(str(leftmotor.count_per_rot))
            # f.write('Relative snoofer angle: '+'\n')
            #f.write(str(snoofer_ang)+'\n')
            f.write('Gyro angle:'+'\n')
            f.write(str(gyro_angle)+'\n')
            # f.write ("Absolute snoofer angle: "+'\n')
            # f.write (str(snoofer_ang_final) + '\n')
            #f.write ('Snoofer distance:'+'\n')
            # f.write (str(snooferdistance) + '\n')
            #f.write ("Object coordinates:"+'\n')
            # f.write (coord_str)
            
            #Preparing for next interation
            random.seed(math.trunc(snooferdistance*move.x_pos_mm+gyro_angle))
            show.update()
            time.sleep(0.5)
            move.on_for_distance(27, -100, brake=True, block=True)
            move.turn_degrees(31,turn())
            object_found=0
            pre_terminate+=1
            if pre_terminate==iterations: 
                terminate=1


#Sends a special set of numbers to shut off computer program
abort_str=str(440440440440) + ',  ' + str(440440440440)
send_coords(abort_str,broker_address)
time.sleep(1)
#Final gyro mearurement
temp=str(gyro.angle)
f.write(temp)
f.close()