# SEED-Lab-Mini-Project

## Organization of this Repository
- The code used for the Arduino to control the motor is located in the "Arduino" folder 
- The code used for the raspberry pi to control the computer vision and serial communication is located in the "Raspberry Pi" folder

## Arduino 
- Motor Control is the main program used to control the motor and wheel position from the Arduino
- Motor Control Old is a past version of a motor controller that adds an integral controller, but doesn't function as well
- Transfer Function Test v1 is used to collect samples of angular velocity vs time to determine the transfer function
- Transfer Function Test v2 does the same thing as v1, but uses a longer interval to calculate the velocity to smooth the curve

# Raspberry Pi
- CV_raspberrypi.py is the python script used to execute computer vision with the pi camera and send the output to the Arduino 

# Hardware Instructions
- Connect the Arduino to the Raspberry Pi via serial USB
- Mount the motor driver PWM shield on the Arduino and connect the shield to the motor
- Connect the battery pack to the motor driver shield
