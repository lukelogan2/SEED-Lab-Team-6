# SEED-Lab-Final

## Organization of this Repository
 - The code used for the Arduino to control the drive testing is located in the "Arduino" folder
 - The code used for the raspberry pi to control the computer vision is located in the "Raspberry Pi" folder

### Arduino
 - Final_2_Functions_v2.ino is the working program where the robot follows the tape around the course

### Raspberry Pi
 - Final_cv.py is the working python script used to execute computer vision with the pi camera

## Hardware Instructions
Using the rover we designed...

- Connect the Arduino to the Raspberry Pi via serial USB
- Connect the smaller black battery to the Raspberry Pia via USB C
- Connect the larger battery to the motor driver
- Ensure there are no loose wires between the Arduino and the motors
- 
To follow the course...

- Set the robot at the start of the course
- Upload the Final_2_Functions_v2.ino program to the Arduino
- Run final_cv.py on the Raspberry Pi
