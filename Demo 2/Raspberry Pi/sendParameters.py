import serial
import time
#Set address
ser = serial.Serial('/dev/ttyACM0', 115200)
#Wait for connection to complete
time.sleep(3)
#Function to read serial
def ReadfromArduino():
    while (ser.in_waiting > 0):
        try:
            line = ser.readline().decode('utf-8').rstrip()
            print("serial output : ", line)
        except:
            print("Communication Error")
            
#How to send a string
dist = 1
angle = 90
kp_left = 10
kp_right = 10
data = bytearray([dist,angle,kp_left,kp_right])
#Remeber to encode the string to bytes
ser.write(data)
print("Data sent")

# wait for Arduino to set up response
#while True:
#    ReadfromArduino()

print("Done")

