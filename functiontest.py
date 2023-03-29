import serial
import numpy as np
import time

mobileRobotSer = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
lineSer = serial.Serial1('/dev/ttyUSB1', 9600, timeout=1)

speedL=0
speedR=0
oldposition =0 
oldtime =0


#defining value of each sensors
SRF1 = 0
SRF2 = 1
SRF3 = 2
SRF4 = 3
SRF5 = 4
GYRO = 5
EncoderL = 6
EncoderR = 7
Tombol1 = 8
Tombol2 = 9
Tombol3 =10
Tombol4 = 11

#defining PID variables
P=0
D=0
PD=0
lastError=0
Kp=8
Kd=0

def startSerialRead():
    global mobileRobotSer
    x=0
    while x<10:
        mobileRobotSer.readline()
        x=x+1
def setSpeed(speedLeft, speedRight):
    global speedL,speedR
    speedL = speedLeft
    speedR = speedRight
    serialCommand()
    

def serialCommand():
    
    global mobileRobotSer
    data = np.zeros(8, dtype=np.uint8)

    buzz = 0x00

    #hex of 'm' 'r' 'i' 0x6D 0x72 0x69
    data[0] = 0x6D
    data[1] = 0x72
    data[2] = 0x69

    global speedL 
    global speedR 

    data[4] = speedL >> 8
    data[3] = speedL & 0xFF
    data[6] = speedR >> 8
    data[5] = speedR & 0xFF
    data[7] = buzz

    mobileRobotSer.write(data)

def errorMapping(sensorState):
    errorState=0
    if sensorState == 0b10000000:
        errorState = 7
    if sensorState == 0b11000000:
        errorState = 6
    if sensorState == 0b11100000:
        errorState = 6
    if sensorState == 0b01000000:
        errorState = 5
    if sensorState == 0b01100000:
        errorState = 4
    if sensorState == 0b01110000:
        errorState = 4
    if sensorState == 0b00100000:
        errorState = 3
    if sensorState == 0b00110000:
        errorState = 2
    if sensorState == 0b00111000:
        errorState = 2
    if sensorState == 0b00010000:
        errorState = 1
    if sensorState == 0b00011000:
        errorState = 0
    if sensorState == 0b00001000:
        errorState = -1
    if sensorState == 0b00011100:
        errorState = -2
    if sensorState == 0b00001100:
        errorState = -2
    if sensorState == 0b00000100:
        errorState = -3
    if sensorState == 0b00001110:
        errorState = -4
    if sensorState == 0b00000110:
        errorState = -4
    if sensorState == 0b00000010:
        errorState = -5
    if sensorState == 0b00000111:
        errorState = -6
    if sensorState == 0b00000011:
        errorState = -6
    if sensorState == 0b00000001:
        errorState = -7
    return errorState

def getSerialData(certainData):
    global mobileRobotSer
    takenData   = mobileRobotSer.readline().decode("utf-8").rstrip()
    return takenData.split(',')[certainData]

#===============BASED ON TIME TO DETERMINE THE DISTANCE=================
    # newposition = getSerialData(EncoderR)
    # newtime = round(time.time())
    # setSpeed(0,200)
    # #print(newtime-oldtime)
    # if (newtime-oldtime)>0:
    #     vel = (((int(newposition)-int(oldposition))/32766)*0.15) /(int(newtime)-int(oldtime))
    #     # print ("speed = ")
    #     #print (vel)
    #     # print ("\n")
    #     dist = vel * (newtime-startTime)
    #     print(dist)
    # oldposition = newposition
    # oldtime = newtime

#--------------------------NOTES--------------------------
#       one revolution = 32767 ticks
#       So, how to get the distance based on the ticks, where
#       the tick will be 0 again after 65535
#

#====================Main Program=======================
startSerialRead()
startTime = round(time.time())

while True:
    newposition = getSerialData(EncoderR)
    newtime = time.time()*1000
    setSpeed(0,1000)
    #print(newtime-oldtime)
    if round((newtime-oldtime))==70:
        vel = (((int(newposition)-int(oldposition))/32768)*0.471) / 1000
        #vel = (int(newposition)-int(oldposition)) /(int(newtime)-int(oldtime))
        print ("speed = ")
        print (vel)
        print ("\n")
        #dist = vel * (newtime-startTime)
        #print(dist)
    
    oldposition = newposition
    oldtime = newtime 
   

mobileRobotSer.close()
