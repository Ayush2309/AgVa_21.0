import smbus
import time
import serial
from math import *
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(18,GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.output(21, GPIO.HIGH)
motor_1=GPIO.PWM(18,50)
motor_1.start(40)
bus=smbus.SMBus(1) #The default i2c bus
address=0x25
bus.write_i2c_block_data(address, 0x3F, [0xF9]) #Stop any cont measurement of the sensor
time.sleep(0.8)
bus.write_i2c_block_data(address, 0x36, [0X03]) # The command code 0x3603 is split into two arguments, cmd=0x36 and [val]=0x03
time.sleep(0.5)
ser = serial.Serial(
	port='/dev/ttyAMA0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=0.05
)
area_1 = 0.000950625 #151755924         #in metres square
area_2 = 0.000085785
dt=0.004
rho=1.225
volume_divisor=((1/(area_2**2))-(1/(area_1**2)))

while True:   
   
    reading=bus.read_i2c_block_data(address,0,9)
    pressure_value=reading[0]+float(reading[1])/255
    if pressure_value>=0 and pressure_value<128:
        diffirential_pressure=pressure_value*255/60 #scale factor adjustment
    elif pressure_value>128 and pressure_value<=256:
        diffirential_pressure=-(256-pressure_value)*255/60 #scale factor adjustment
    elif pressure_value==128:
        diffirential_pressure=500 #Out of range
#    ser.write(str(diffirential_pressure))
    time.sleep(0.05)
#    print(diffirential_pressure)
    massFlow=1000*(sqrt((abs(diffirential_pressure)*2*rho)/volume_divisor))
    volFlow= massFlow/rho
    volFlow_min=volFlow*60
    print(volFlow_min*0.55)
    time.sleep(0.1)
