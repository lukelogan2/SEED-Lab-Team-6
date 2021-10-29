# SEED-Lab-Demo-1

## Organization of this Repository
- The code used for the Arduino to control the drive testing is located in the "Arduino" folder 
- The code used for the raspberry pi to control the computer vision is located in the "Raspberry Pi" folder

## Arduino 
- arduino_pi_controller.ino is the working program used to control the drive testing of the rover to rotate a specified angle and drive straight a specified distance
- Demo_1_Proportional_controller.ino is an earlier version of the drive testing program that only used a proportional controller

## Raspberry Pi
- demo1_CV_version2.py is the working python script used to execute computer vision with the pi camera to determine the angle of blue tape in front of the rover
- demo1_cv.py is an earlier version that isn't as functional
- CV_raspberrypi.py isn't functional
- All the image files were used for testing the output of the camera
- sendParameters.py is the start of a python script to send parameters from the pi to the Arduino

# Hardware Instructions
Using the rover we designed...
- Connect the Arduino to the Raspberry Pi via serial USB
- Connect the smaller black battery to the Raspberry Pia via USB C
- Connect the larger battery to the motor driver
- Ensure there are no loose wires between the Arduino and the motors 
For the drive test...
- Input an angle and distance in the Arduino script "arduino_pi_controller.ino"
- Set the rover on the ground and push the button on the Arduino to start the program
For the computer vision test...
- Run the python script demo1_CV_version2.py on the raspberry pi
- Ensure the LCD is connected to the raspberry pi and displaying angles
- Place a strip of blue tape on the floor in front of the rover
- Make sure there is no other blue in the background
- Observe the angle on the LCD which is positive left of the rover's field of view and negative right of the rover's field of view
