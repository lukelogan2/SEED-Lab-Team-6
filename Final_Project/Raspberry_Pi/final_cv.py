# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from time import sleep
import numpy as np
import time
import cv2
from matplotlib import pyplot as plt
import os
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import serial

# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA

# Initialise the LCD class
lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)        

corners = 0 # Variable to keep track of the number of corners
on_corner = False
# Initialize serial communiction
try:
    ser = serial.Serial('/dev/ttyACM0', baudrate=115200, write_timeout=2)
    time.sleep(3)
except:
    lcd.message="Serial Failed"

########################################################
# Read Serial Input from Arduino
########################################################
def ReadfromArduino():
    while (ser.in_waiting > 0):
        try:
            line = ser.readline().decode('utf-8').rstrip()
            print("serial output : ", line)
        except:
            print("Communication Error")

########################################################
# Write Serial Data to Arduino
########################################################
def sendSerial(message):
    try:
        ser.write(message.encode())
        print("Writing " + message)
    except:
        print("Serial Message Failed to Send")

########################################################
# Camera Calibration
########################################################
def cam_Calibrate(width, height):
    camera = PiCamera(resolution = (width, height), framerate = 30)
    #camera.rotation = 0
    camera.iso = 100
    sleep(2)
    
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    g = camera.awb_gains
    camera.awb_mode = 'fluorescent'
    camera.awb_gains = g
    
    #camera.start_preview(alpha = 100)
    #sleep(3)
    #camera.stop_preview()
    for i in range(1,4):
        rawCapture = PiRGBArray(camera)
        camera.capture(rawCapture, format="bgr")
        #image = rawCapture.array
    
    
    return camera
    

########################################################
# Take Picture Function
########################################################
def take_Pic(camera):
    # Define camera resolution and initialize camera
    rawCapture = PiRGBArray(camera)

    # set iso to the desired value
    #camera.iso = 100
    #camera.resolution = (1920, 1088)
    # wait for the automatic gain control to settle
    #sleep(1)
   
    #camera.start_preview(alpha=200)
    #camera.annotate_background = Color('blue')
    #camera.annotate_text = 'Assignment 2, Part 1'
    #camera.annotate_text_size = 50
    #sleep(3)
    #camera.stop_preview()

    # grab an image from the camera
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    #path = '/home/pi/Desktop'
    #cv2.imwrite(os.path.join(path, 'original.jpg'), image)
    
    #cv2.imshow('photo', image)
    #cv2.waitKey(3000)
    #cv2.destroyAllWindows()
    
    return image

########################################################
# Convert Image to HSV Then Apply Color Mask Function
########################################################
def color_Mask(output):

    #Convert resized image array from BGR to HSV
    hsvImage = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)
    
    #Define range of yellow color in HSV (use this for assignment 2)
    #lower_Yellow = np.array([15,100,100])
    #upper_Yellow = np.array([30,255,255])
    
    #Define range of blue color in HSV (save for later project)
    lower_blue = np.array([100,100,20])
    upper_blue = np.array([135,255,255])
    
    #Define range of orange color in HSV (save for later project)
    #lower_orange = np.array([10,100,20])
    #upper_orange = np.array([25,255,255])
    
    #Threshold the HSV image to get only blue colors (use this for assignment 2)
    #mask = cv2.inRange(hsvImage, lower_Yellow, upper_Yellow)
    
    #Threshold the HSV image to get only blue colors (save for later project)
    mask = cv2.inRange(hsvImage, lower_blue, upper_blue)
    
    #Threshold the HSV image to get only orange colors (save for later project)
    #mask = cv2.inRange(hsvImage, lower_orange, upper_orange)
    
    # Bitwise-AND mask and the resized/original image
    res = cv2.bitwise_and(output, output, mask=mask)
    
    #print(res)
    
    #path = '/home/pi/Desktop'
    #cv2.imwrite(os.path.join(path, 'color mask.jpg'), res)
    
    #cv2.imshow('Object', res)
    #cv2.waitKey(3000)
    #cv2.destroyAllWindows()
    
    return res
    
########################################################
#Morphological Transformations Function 
########################################################
def morph_Pic(res):
    
    kernel = np.ones((5,5), np.uint8)
    closing = cv2.morphologyEx(res, cv2.MORPH_CLOSE, kernel, iterations = 2)
    # use opening since it is the most useful for this project
    opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel, iterations = 2)
    #erode = cv2.erode(res, kernel, iterations = 3)
    #path = '/home/pi/Desktop'
    #cv2.imwrite(os.path.join(path, 'morph.jpg'), opening)
    
    #cv2.imshow('morphed', opening)
    #cv2.waitKey(3000)
    #cv2.destroyAllWindows()
    
    #return opening
    return opening


########################################################
#Determine Object Location and Calculate Object Angle (in x-aixs) Function 
########################################################
def calc_AngleX(res):
    fov = 53.5        #field of view of camera
    fov_adj = 62      #field of view of camera adjusting for 3 inch offset from center of rotation
    a = np.array(res)
    a0 = np.nonzero(a)
    pix_total=np.count_nonzero(a)
    global corners
    #print(np.mean(a0, axis=1))
    if len(a0[0]) and len(a0[1]):
           #aMeanY, aMeanX, aMeanZ = np.nanmean(a0, axis=1, axis=0)
           aMeanY, aMeanX, aMeanZ = np.nanmean(a0, axis=1)
           #grab image width, divide by 2 to find center pixel, solve for degree per pixel
           #take object position (xaxis) and sub center pixel to find offset
           #multiple pixel offset by degrees per pixel to ifnd total degree off center
           if max(a0[0]) > 800 and len(a0[0]) > 4000:
               on_corner = False
               width = res.shape[1]
               length = res.shape[0]
               centerX = width/2
               centerY = length/2
               degPerPixel_X = fov_adj/width
               blueX = 0
               count = 0
               for i in range(0,len(a0[0])):
                   if (a0[0][i] >= 700):
                       count += 1
                       blueX = blueX + a0[1][i]
               blueX = blueX / count
               #print(count)
               if count == 0:
                   message = "0,0.1"
                   sendSerial(message)
               elif count > 130000:
                   return 1
               #pixelDelta = centerX-80 - aMeanX  #Adjust center point to match camera position by subtracting 80 from center
               pixelDelta = centerX-80 - blueX  #Adjust center point to match camera position by subtracting 80 from center
               angleDelta = pixelDelta * degPerPixel_X
               
               # If the angle is greater than 5 degrees, rotate to correct
               if angleDelta and abs(angleDelta) > 15:
                   # Message format = angle,distance
                   message = str(round(angleDelta,2)) + ",0"
                   sendSerial(message)
                   sleep(2)
                   message = "0,0.1"
                   sendSerial(message)
               # If the angle is less than 5 degrees, keep driving straight
               elif angleDelta:
                   #Message format = angle,distance
                   message = "0,0.1"
                   sendSerial(message)
               return 0
           else:
               # No tape seen, move forward and rotate 90 degrees
               #Message format = angle,distance
               message = "-30,0"
               rotating = True
               sendSerial(message)
               corners += 1
               #on_corner = True
               sleep(3)
               return 0
           
    else:
       # No tape seen, move forward and rotate 90 degrees
       #Message format = angle,distance
       message = "-30,0"
       sendSerial(message)
       corners += 1
       on_corner = True
       sleep(3)
       return 1

def get_image():
    src = take_Pic(camera)
    morphed = morph_Pic(src)
    res = color_Mask(morphed)
    return res

########################################################
# STATE MACHINE IMPLEMENTATION
########################################################

def state1():
    #print("state 1")
    # Follow the curve and the 1st corner
    res = get_image()
    newstate = calc_AngleX(res)
    if newstate:
        return state2
    else:
        return state1

def state2():
    print("state 2")
    # Navigate the gap to arrive at the end
    #Message format = angle,distance
    message = "0,0.15"
    sendSerial(message)
    return state_done

def state_done():
    pass

# State Dictionary
state_dict = {
    state1 : "state1",
    state2: "state2",
    state_done: "state_done"
}

# Initialization
state = state1
finished = False

width = 1920
height = 1088

camera = cam_Calibrate(width, height)

# Loop through all characters in string
while not finished:
    # Call state function with pointer
    if state != state_done:
        new_state = state()
        state = new_state
    else:
        finished = True
    
print("Done with state machine")


