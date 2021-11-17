# SEED-Lab-Demo-2

## Organization of this Repository
- The code used for the Arduino to control the drive testing is located in the "Arduino" folder 
- The code used for the raspberry pi to control the computer vision is located in the "Raspberry Pi" folder

## Arduino 
- demo2_working_full is the working program where the robot searches for the tape, drives to it, and follows it to the end
- demo2_working_half is the working program where the robot searches for the tape, drives to it, and stops

## Raspberry Pi
- demo2_cv.py is the working python script used to execute computer vision with the pi camera

# Hardware Instructions
Using the rover we designed...
- Connect the Arduino to the Raspberry Pi via serial USB
- Connect the smaller black battery to the Raspberry Pia via USB C
- Connect the larger battery to the motor driver
- Ensure there are no loose wires between the Arduino and the motors 
- 
For the drive to tape and stop test...
- Upload the demo2_working_half program to the Arduino
- Run demo2_cv.py on the Raspberry Pi

For the drive to tape and follow test...
- Upload the demo2_working_full program to the Arduino
- Run demo2_cv.py on the Raspberry Pi
