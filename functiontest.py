import serial
import numpy as np
import time

mobileRobotSer = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
lineSer = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)

speedL=0
speedR=0
buzz=0
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

#robot movement variables
distanceR = float(0)
distanceL = float(0)
#distanceAll = float(0)
lastEncValL=0
lastEncValR=0

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

    #buzz = 0x00
    global buzz
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

def resetEncoder():
    global buzz, lastEncValL, lastEncValR
    lastEncValL=0
    lastEncValR=0
    buzz = 1
    serialCommand()
    time.sleep(0.1)
    buzz = 0
    serialCommand()

def getDistanceMobileRobot():
    global distanceR, distanceL, distanceAll, lastEncValR, lastEncValL
    
    encValR      = int(getSerialDataMobileRobot(EncoderR))
    encValL      = int(getSerialDataMobileRobot(EncoderL))

    encSampR = 0
    encSampL = 0

    if encValR<lastEncValR:
        encSampR     = (encValR+65535)-lastEncValR
    else: 
        encSampR     = encValR-lastEncValR

    if encValL<lastEncValL:
        encSampL     = (encValL+65535)-lastEncValL
    else: 
        encSampL     = encValL-lastEncValL	

    distanceR   = distanceR + encSampR 
    distanceL   = distanceL + encSampL

    lastEncValR = encValR
    lastEncValL = encValL

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

def getSerialDataMobileRobot(certainData):
    global mobileRobotSer
    takenData   = mobileRobotSer.readline().decode("utf-8").rstrip()
    return takenData.split(',')[certainData]

def followLine():
    global lastError
    value=lineSer.readline()
    error=0
    value=value.decode("utf-8").rstrip()
    if value.isnumeric():
        error   =   int(value)
        error   =   errorMapping(error)
        P       =   Kp*error
        D       =   Kd*(error-lastError)
        PD      =   P+D
        #setSpeed(100-PD,100+PD)
        lastError=error

def line_KS():
    error=0
    while error != 0b11111111:
        value=lineSer.readline()
        error=0
        value=value.decode("utf-8").rstrip()
        if value.isnumeric():
            error   =   int(value)
        #getDistanceMobileRobot()
        followLine()
        #distanceAll=(((distanceL/32767)*471)+((distanceR/32767)*471))/2
        print(distanceAll)


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


setSpeed(0,0)
time.sleep(1)
setSpeed(100,-100)
time.sleep(1)
setSpeed(0,0)
time.sleep(1)
setSpeed(-100,100)
time.sleep(2)  
setSpeed(0,0)
time.sleep(1)  
setSpeed(100,-100)
time.sleep(1)
setSpeed(0,0)
time.sleep(18)
resetEncoder()  

line_KS()

setSpeed(0,0)
    
   
