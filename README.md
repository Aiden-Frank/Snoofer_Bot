# Snoofer_Bot
This repository contains code for operating a robot I built, called Snoofer Bot.

Code under Bot_Code is intended to be run on the robot.  Main.py is the primary one, the rest are for testing.
Computer_code contains code to be run on a laptop nearby, to recieve and store data from the robot.

A presentation detailing the robot, its code, and its performance is <a href="https://github.com/Aiden-Frank/Snoofer_Bot/blob/main/Presentation.pdf" target="_blank">here.</a>

Image key:

0: Raspberry Pi with PiStorms-v2: The robot’s central computer.

1: Powered drive wheel: The robot uses two of these to move and rotate itself.

2: LiDAR Distance Sensor: This sensor informs the robot of how far away objects are.  It is mounted on a hinge, and so can swivel side to side.  

3: Sensor motor: This motor is connected by a crank to the LiDAR distance sensor (2).  When running, it swivels the sensor back and forth.

4: Gyroscope (mounted inside robot, so not really visible): A gyroscopic sensor that tracks the robot’s rotation.

5: Flexible resistor: A component that changes its electrical resistance when it is bent.  One is mounted on each side of the robot.

6: Voltage sensor: This Grove sensor measures the voltage of across a voltage divider which includes the the flexible resistor (5).  There are two, one for each resistor.

7: Sensor wiring:  This group of components powers the voltage dividers (6), and converts their data from the Grove format into I2C.

<img width="812" height="607" alt="Screen Shot 2024-03-15 at 9 51 44 AM" src="https://github.com/user-attachments/assets/59d51634-4d70-44b7-bbe9-49cb4476992d" />
<img width="649" height="603" alt="Screen Shot 2024-03-15 at 9 50 56 AM" src="https://github.com/user-attachments/assets/f6e0b19f-fcd8-4a8d-afb1-cf1b86cb9aea" />

