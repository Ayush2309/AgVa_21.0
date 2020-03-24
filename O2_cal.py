import os
import serial
from math import *
import smbus
import time
import RPi.GPIO as GPIO
import Adafruit_ADS1x15
adc = Adafruit_ADS1x15.ADS1115()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(18,GPIO.OUT)
GPIO.setup(37, GPIO.OUT)
GPIO.setup(40,GPIO.OUT)
sol = GPIO.PWM(40, 1000)
sol.start(100)
#GPIO.setup(22, GPIO.OUT)
#GPIO.output(22, GPIO.LOW)
motor_1=GPIO.PWM(18,50)
GPIO.setup(37, GPIO.OUT)
#ADDRESS_SDP=0x25
ser = serial.Serial(
	port='/dev/ttyAMA0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
)
try:
    bus=smbus.SMBus(1) #The default i2c bus
    address=0x28
   #-------------------------------------
except:
    print('hello')
motor_1.start(30)
cal_reading = 3521.0
array = []
#ADDRESS_SDP=0x25
area_1 = 0.000950625 #151755924         #in metres square
area_2 = 0.000085785
dt=0.004
rho=1.225
volume_divisor=((1/(area_2**2))-(1/(area_1**2)))
def ABPcmH2O():
    val=bus.read_i2c_block_data(address,0)
    data=((val[0] & 0x3f) << 8) + val[1]
    press_psi = float(data- 1638 )/14745
    press_cmH2O=press_psi * 70.307
    return press_cmH2O

while(1):
    global cal_reading
    motor_1.ChangeDutyCycle(80)
    time.sleep(2) 
    try:
	ser.write("ACK52")
    except:
	print("unablee to send")
    now = time.time()
#    flow = rate()
    elapsed = time.time() - now
    while(elapsed < 10):
	flow = ABPcmH2O()
	print(flow)
	reading = adc.read_adc(2, gain = 2/3)
	if(reading > 5000):
	    print(flow)
	    print(reading)
	    print("getingggggggggggggggggggggggggggggggggggggg")
            try:
	        ser.write("ACK54")
    	    except:
		print("unablee to send")
	    terminate_elapsed = time.time()
	    while(reading > 5000):
#		GPIO.output(22, GPIO.HIGH)
		flow = ABPcmH2O()
		GPIO.output(37, GPIO.HIGH)
		time.sleep(0.7)
		GPIO.output(37, GPIO.LOW)
		time.sleep(0.7)
		reading = adc.read_adc(2, gain = 2/3)
		if(time.time() - terminate_elapsed > 10):
		    ser.write("ACK63")
		    time.sleep(1)
		    os.system("sudo shutdown -h now")
		if(reading > 5000):
		    ser.write("ACK56")
		now = time.time()
		print('flow is less than 80 L/min')
		print(flow)
		print(reading)
            try:
	        ser.write("ACK55")
    	    except:
		print("unablee to send")
#	GPIO.output(22, GPIO.LOW)
	elapsed = time.time() - now
	GPIO.output(37, GPIO.HIGH)
	time.sleep(0.7)
	GPIO.output(37, GPIO.LOW)
	time.sleep(0.7)
    for i in range(0,10,1):
	GPIO.output(37, GPIO.LOW)
        reading = adc.read_adc(2, gain= 2/3)
#	time.sleep(0.7)
	GPIO.output(37, GPIO.HIGH)
#	time.sleep(0.7)
        array.append(reading)
	print(reading)
#        FiO2 = np.interp(int(reading),[0,8880],[0,100])
    if(len(array) >= 10):
        inst_reading = sum(array)/len(array)
        print(sum(array)/len(array))
        print(cal_reading)
        print(inst_reading)
        factor = cal_reading/inst_reading
#	try:
        f = open('/home/pi/AgVa_5.0/factor.txt', "w")
        f.write(str(factor))
        f.close()
        try:
	    ser.write("ACK53")
    	except:
	    print("unablee to send")
#	except IOError:
#	    print('unable to write')
        print(factor)
	os.system("sudo shutdown -h now")
    break
