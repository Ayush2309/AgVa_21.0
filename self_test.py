import os
import numpy as np
import time
import RPi.GPIO as GPIO
import Adafruit_ADS1x15
import serial
from math import *
import smbus

adc = Adafruit_ADS1x15.ADS1115()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(18,GPIO.OUT)
motor_1=GPIO.PWM(18,50)
motor_1.start(0)
GPIO.setup(40,GPIO.OUT)
sol = GPIO.PWM(40, 1000)
sol.start(0)
GPIO.setup(22,GPIO.OUT)
ADDRESS_ABP=0x28
ADDRESS_SDP=0x25
area_1 = 0.000950625 #151755924         #in metres square
area_2 = 0.000085785
dt=0.004
rho=1.225
volume_divisor=((1/(area_2**2))-(1/(area_1**2)))
try:
    bus=smbus.SMBus(1) #The default i2c bus
    bus.write_i2c_block_data(ADDRESS_SDP, 0x3F, [0xF9]) #Stop any cont measurement of the sensor
    time.sleep(0.8)
    bus.write_i2c_block_data(ADDRESS_SDP, 0x36, [0X03]) # The command code 0x3603 is split into two arguments, cmd=0x36 and [val]=0x03
    time.sleep(0.5)
except:
    print("NOT HAPPENING")
ser = serial.Serial(
	port='/dev/ttyAMA0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
)
def ABPcmH2O():
    try:
        val=bus.read_i2c_block_data(ADDRESS_ABP,0)
        data=((val[0] & 0x3f) << 8) + val[1]
        press_psi = float(data- 1638 )/14745
        press_cmH2O=press_psi * 70.307
        return press_cmH2O
    except:
	print('unable to compute ABP')
	return 0
def SDP():
    try:
        reading=bus.read_i2c_block_data(ADDRESS_SDP,0,9)
        pressure_value=reading[0]+float(reading[1])/255
        if pressure_value>=0 and pressure_value<128:
            diffirential_pressure=pressure_value*255/60 #scale factor adjustment
        elif pressure_value>128 and pressure_value<=256:
            diffirential_pressure=-(256-pressure_value)*255/60 #scale factor adjustment
        elif pressure_value==128:
            diffirential_pressure=500
	massFlow=1000*(sqrt((abs(diffirential_pressure)*2*rho)/volume_divisor))
	volFlow= massFlow/rho
	volFlow_min=volFlow*60
        return volFlow_min
    except:
	print('unable to compute SDP')
	return 0
os.system("sudo rm -r /home/pi/AgVa_5.0/i2c.txt")
os.system("sudo i2cdetect -y 1 >> /home/pi/AgVa_5.0/i2c.txt")
f = open("/home/pi/AgVa_5.0/i2c.txt","r")
data = f.read()
f.close()
if(data.find("28") == -1 and data.find("25") == -1 and data.find("48") == -1):
    ser.write("I2C LINE ERROR\n#")
print(data.find("28"))
if(data.find("28") == 186):
    ser.write("ABP FOUND\n#")
else:
    ser.write("ABP NOT FOUND#\n")
if(data.find("25") == 177):
    ser.write("SDP FOUND\n#")
else:
    ser.write("SDP NOT FOUND\n#")
if(data.find("48") == 292):
    ser.write("ADC FOUND\n#")
else:
    ser.write("ADC NOT FOUND\n#")
if(data.find("28") == 186):
    sol.ChangeDutyCycle(100)
    ser.write("ABP VALUE AT 0% DUTY CYCLE " + str(round(ABPcmH2O(),2))+"\n#")
    motor_1.ChangeDutyCycle(50)
    time.sleep(3)
    ser.write("ABP VALUE AT 50% DUTY CYCLE " + str(round(ABPcmH2O(),2)) + "\n#")
    motor_1.ChangeDutyCycle(100)
    time.sleep(3)
    ser.write("ABP VALUE AT 100% DUTY CYCLE " + str(round(ABPcmH2O(),2)) + "\n#")
    motor_1.ChangeDutyCycle(0)
    sol.ChangeDutyCycle(0)
    time.sleep(8)
else:
    ser.write("ABP SENSOR ERROR\n#")
if(data.find("25") == 177):
    sol.ChangeDutyCycle(100)
    ser.write("SDP READING AT 0% DUTY CYCLE " + str(round(SDP(),2)) + "\n#")
    motor_1.ChangeDutyCycle(25)
    ser.write("REMOVE THE LUNG IF CONNECTED\n#")
#    time.sleep(5)
    time.sleep(5)
    ser.write("SDP READING AT 25% DUTY CYCLE " + str(round(SDP(),2)) + "\n#")
    motor_1.ChangeDutyCycle(0)
    sol.ChangeDutyCycle(0)
    time.sleep(5)
else:
    ser.write("SDP SENSOR ERROR \n#")
if(data.find("48") == 292):
    battery = adc.read_adc(0, gain = 1)
    if(battery > 7000):
	ser.write("BATTERY IS OK AND CHARGED\n#")
    elif(battery  > 1000):
	ser.write("BATTERY IS OK AND NOT CHARGED\n#")
    else:
	ser.write("BATTERY NOT CONNECTED\n#")
    ser.write("BATTERY " + str(adc.read_adc(0, gain=1)) + "\n#")
    power = adc.read_adc(1, gain = 1)
    if(power > 8000):
	ser.write("POWER CONNECTED \n#")
    else:
	ser.write("POWER NOT CONNECTED \n#")
    ser.write("AC POWER " + str(adc.read_adc(1, gain = 1)) + "\n#")
    oxygen = adc.read_adc(2, gain = 2/3)
    if(oxygen > 5000):
	ser.write("OXYGEN SENSOR NOT CONNECTED \n#")
    elif(oxygen > 3000):
	ser.write("OXYGEN SENSOR OK \n#")
    else:
	ser.write("OXYGEN SENSOR CALLIBRATION REQUIRED \n#")
    ser.write("OXYGEN " + str(adc.read_adc(2, gain = 2/3)) + "\n#")
ser.write("SELF TEST COMPLETED\n#")
time.sleep(1)
os.system("sudo killall python")
