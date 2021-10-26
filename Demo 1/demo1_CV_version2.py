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
    camera.awb_mode = 'off'
    camera.awb_gains = g
    
    #camera.start_preview(alpha = 100)
    #sleep(3)
    #camera.stop_preview()
    
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
    lower_blue = np.array([100,50,20])
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
    #closing = cv2.morphologyEx(res, cv2.MORPH_CLOSE, kernel)
    # use opening since it is the most useful for this project
    #opening = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel, iterations = 2)
    erode = cv2.erode(res, kernel, iterations = 3)
    #path = '/home/pi/Desktop'
    #cv2.imwrite(os.path.join(path, 'morph.jpg'), opening)
    
    #cv2.imshow('morphed', opening)
    #cv2.waitKey(3000)
    #cv2.destroyAllWindows()
    
    #return opening
    return erode


########################################################
#Determine Object Location and Calculate Object Angle (in x-aixs) Function 
########################################################
def calc_AngleX(res):
    fov = 53.5        #field of view of camera
    length_image = 1920     #length of image in pixels
    f = 1550        #focal length of camera in pixels
 
    a = np.array(res)
    a0 = np.nonzero(a)
    
    #print(np.mean(a0, axis=1))
    if len(a0[0]) and len(a0[1]):
           #aMeanY, aMeanX, aMeanZ = np.nanmean(a0, axis=1, axis=0)
           aMeanY, aMeanX, aMeanZ = np.nanmean(a0, axis=1)
           #grab image width, divide by 2 to find center pixel, solve for degree per pixel
           #take object position (xaxis) and sub center pixel to find offset
           #multiple pixel offset by degrees per pixel to ifnd total degree off center
           width = res.shape[1]
           length = res.shape[0]
           centerX = width/2
           centerY = length/2
           degPerPixel_X = fov/width
           pixelDelta = centerX - aMeanX 
           angleDelta = pixelDelta * degPerPixel_X
           # if angleDelta >= -7 and angleDelta < -1:
               # angleDelta += 1.3
           # if angleDelta >= -13 and angleDelta < -7:
               # angleDelta -= 4.5
           # if angleDelta >= -23 and angleDelta < -13:
               # angleDelta -= 4.6
           # if angleDelta > 1 and angleDelta <= 7:
               # angleDelta -= 1.1
           # if angleDelta > 7 and angleDelta <= 13:
               # angleDelta += 2.7
           # if angleDelta > 13 and angleDelta <= 23:
               # angleDelta += 4.8
            
           print('Object coordinates are: x ', aMeanX, 'and y ', aMeanY, '.\n')
           #print('Object is ', angleDelta, 'degrees from center.\n')
           
           value = "1"
           message="Angle = {:.2f}".format(angleDelta)
               
           print(message)
           lcd.clear()
           lcd.message = message
           
    else:
           print('No marker found and do nothing.\n')
            

########################################################
# Take Pictures Continously and Determine Object Location from Camera
########################################################
#Initialize camera
#camera = PiCamera()



width = 1920
height = 1080

camera = cam_Calibrate(width, height)

for i in range(0,8):
   image = take_Pic(camera)

#Loop to do continous obect location calculations
while True:
    #i = input("Enter text to quit or press Enter to continue: ")
    #if not i:
        
    #Store gray-scale image array as a variable 
    src = take_Pic(camera)
    
    #Store image array in a new variable
    #src = image
    
    #Call resize image function and store as a new variable
    #output = resize_Pic(src)
    
    #Call Morphological Transform function to enhance picture
    morphed = morph_Pic(src)
    
    #Call show canny edge function
    #show_Edge(output)
    
    #Call color mask function and store as new variale
    res = color_Mask(morphed)
    
    #Call function to determine object location and angle from camera focal point
    calc_AngleX(res)
        
    #else:
        #Disable camera
        #camera.close()
        #break


