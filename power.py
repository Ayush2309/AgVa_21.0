# Simple demo of reading each analog input from the ADS1x15 and printing it to
# the screen.
# Author: Tony DiCola
# License: Public Domain
from time import time,sleep
import serial
from collections import Counter
import numpy as np
import RPi.GPIO as GPIO
from math import *
# Import the ADS1x15 module.
import Adafruit_ADS1x15
GPIO.setmode(GPIO.BOARD)
GPIO.setup(22, GPIO.OUT)
# Create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()
#sleep(10)
power_set = 0
power_flag = 0
# Or create an ADS1015 ADC (12-bit) instance.
#adc = Adafruit_ADS1x15.ADS1015()
battery_status = 0
battery = 0
ser = serial.Serial(
	port='/dev/ttyAMA0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
)
battery_array = []
# Note you can change the I2C address from its default (0x48), and/or the I2C
# bus by passing in these optional parameters:
#adc = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=1)

# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
GAIN = 1
factor = 1.0
try:
    f = open("/home/pi/AgVa_5.0/factor.txt","r")
    factor = f.readline()
    f.close()
    factor = float(factor)
    print(factor)
    print(type(factor))
except IOError:
    print("unable top read factor file")
percentage = 0
def buzzer(position, bit):
    try:
	f = open('/home/pi/AgVa_5.0/buzzer.txt', 'r')
	data = f.readline()
	f.close()
#	print(data)
#	print(position)
#	print(bit)
	if(len(data) > 10):
	    data = data.split(',')
	    data = map(int,data)
	    data[position] = bit
	    data = map(str,data)
	    data = ','.join(data)
	else:
	    data = '0,0,0,0,0,0,0,0,0,0'
#	print(data)
        try:
            f = open('/home/pi/AgVa_5.0/buzzer.txt','w')
	    f.write(str(data))
            f.close()
        except:
            print('unable to write to file')
    except:
	print('unable to read file')
FiO2_last = time()
FiO2_array = []
print('Reading ADS1x15 values, press Ctrl-C to quit...')
# Print nice channel column headers.
print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*range(4)))
print('-' * 37)
# Main loop.
try:
    last_time = time()
    while True:
        global FiO2_last, FiO2_array
#       sleep(5)
        if(time() - FiO2_last > 0.5):
            try:
                reading = adc.read_adc(2, gain= 2/3)
		print("the reading in here is ")
		print(reading)
# 		if(reading < 2000):
# 		    ser.write("ACK34")
		if(reading > 20000):
		    ser.write("ACK64")
                FiO2 = np.interp(int(reading),[0,16770],[0,100])
		print(FiO2*factor)
		if(reading > 15000):
		    FiO2 = np.interp(int(reading),[0,16770],[0,100])
                if( len(FiO2_array) > 3):
                    FiO2_array.pop(0)
                if(len(FiO2_array) <= 3):
                    FiO2_array.append(FiO2)
                #return 0
                    FiO2_val = (max(FiO2_array, key = FiO2_array.count))
#		    if(FiO2_val <= 15):
#		        ser.write("ACK34")
#		    elif(FiO2_val >= 16):
#			ser.write("ACK44")
		    FiO2_final = FiO2_val * factor
		    if(FiO2_final > 88.0):
			FiO2_final = 100.0
                    try:
                        f = open('/home/pi/AgVa_5.0/FiO2.txt', "w")
                        f.write(str(ceil(FiO2_final)))
                        f.close()
                    except:
                        print('unable to write')
		FiO2_last = time()
            except:
                print('unable to clculate')


	#sleep(5)
        # Read all the ADC channel values in a list.
        values = [0]*2
        for i in range(2):
            # Read the specified ADC channel using the previously set gain value.
	    try:
                values[i] = adc.read_adc(i, gain=GAIN)
	    except:
		try:
		    ser.write('ACK19')
		except:
		    print('unable to send data')
            # Note you can also pass in an optional data_rate parameter that controls
            # the ADC conversion time (in samples/second). Each chip has a different
            # set of allowed data rate values, see datasheet Table 9 config register
            # DR bit values.
            #values[i] = adc.read_adc(i, gain=GAIN, data_rate=128)
            # Each value will be a 12 or 16 bit signed integer value depending on the
            # ADC (ADS1015 = 12-bit, ADS1115 = 16-bit).
        # Print the ADC values.
	start_time = time()
        print('| {0:>6} | {1:>6} |'.format(*values))
        if (int('{0:>6}'.format(*values)) <  7300):
#	    if(power_flag == 1):
#                GPIO.output(22, GPIO.HIGH)
 #               sleep(0.5)
  #              GPIO.output(22, GPIO.LOW)
	    if(power_flag == 0):
	        buzzer(6,1)
	        try:
	            ser.write('ACK31')
	        except:
	            print('not able to send')
        # Pause for half a second.
        if (int('{0:>6}'.format(*values)) <=  7500):
 #           GPIO.output(22, GPIO.HIGH)
#	    power_flag = 0
   #         sleep(0.5)
    #        GPIO.output(22, GPIO.LOW)
	    if(power_flag == 0):
	        buzzer(1,1)
	        try:
	            ser.write('ACK41')
	        except:
	            print('not able to send')
        if(int('{1:>6}'.format(*values)) <  3000):
	    if((time() - last_time) > 0.5):
		battery_status = int(np.interp(int('{0:>6}'.format(*values)), [7300,8500],[0,99]))
		battery_array.append(battery_status)
		battery_counter = Counter(battery_array)
		battery_common = battery_counter.most_common(1)
		battery_send = battery_common[0][0]
		battery_str = str(battery_send)
		if(len(battery_str) < 2):
		    battery_str = '0'+battery_str
	        try:
		    ser.write('BTRY'+str(battery_str))
		except:
		    print('unable to send')
		last_time = time()
		power_flag = 0
	        try:
	            ser.write('ACK00')
	        except:
		    print('not able to send')
		buzzer(0,1)
#                GPIO.output(22, GPIO.HIGH)
 #               sleep(1)
  #              GPIO.output(22, GPIO.LOW)
   #             sleep(0.5)
	if(int('{1:>6}'.format(*values)) > 4000):
	    battery = 100
	    if((time() - last_time) > 0.5):
                try:
		    percentage_str = str(percentage)
		    if(len(percentage_str) < 2):
		        percentage_str = '0'+percentage_str
                    ser.write('BTRY'+(percentage_str))
		    print('BTRY'+str(percentage_str))
                except:
                    print('unable to send')
                sleep(0.7)
		if(percentage == 75):
		    percentage = 99
		else:
                    percentage = percentage + 25
                if(percentage >= 100):
		    percentage = 00
		print('power up')
		power_flag = 1
		last_time = time()
	        try:
	            ser.write('ACK01')
	        except:
		    print('not able to send data')
		buzzer(0,0)
		buzzer(1,0)
		buzzer(6,0)
#	if(int('{1:>6}'.format(*values)) < 8700 and int('{1:>6}'.format(*values)) > 4000):
#	    if((time() - last_time) > 0.5):
#		print('power up')
#		last_time = time()
#	        try:
#	            ser.write('ACK32')
#	        except:
#		    print('not able to send')
	if(int('{1:>6}'.format(*values)) > 9350):
	    battery = 100
	    if((time() - last_time) > 0.5):
		print('power up')
		last_time = time()
	        try:
	            ser.write('ACK33')
	        except:
		    print('not able to send')
	sleep(0.5)
except IOError:
    GPIO.cleanup()
