import numpy as np
import time
import zlib
from datetime import datetime, time as datetime_time, timedelta
import serial
import os
import time
import RPi.GPIO as GPIO
print('i am in here')
setting_flag = 0
find_date = ''
counter_flag = 0
counter = 6
flag = 0
#time.sleep(5)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(37, GPIO.OUT)
ser = serial.Serial(
	port='/dev/ttyAMA0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=0.05
)
def time_diff(start, end):
    if isinstance(start, datetime_time): # convert to datetime
        assert isinstance(end, datetime_time)
        start, end = [datetime.combine(datetime.min, t) for t in [start, end]]
    if start <= end:
        return end - start
    else:
        end += timedelta(1) 
        assert end > start
        return end - start
def split_list(n):
    return [(x+1) for x,y in zip(n, n[1:]) if y-x != 1]

def get_sub_list(my_list):
    my_index = split_list(my_list)
    output = list()
    prev = 0
    for index in my_index:
        new_list = [ x for x in my_list[prev:] if x < index]
        output.append(new_list)
        prev += len(new_list)
    output.append([ x for x in my_list[prev:]])
    return output
while(1):
    try:
       find_date =ser.readline()
       print(find_date)
    except:
	print('returned nothing')
    if(find_date[0:4] == 'CMT+'):				#Working time for that date
        find_date = find_date[4:]
        try:
            f = open('/home/pi/AgVa_5.0/data.csv', "r")
            data = f.readlines()
	    f.close()
            a = np.asarray(data)
            data_index = np.core.defchararray.find(a,find_date)
            date_index = np.where(data_index> -1)
            Break = get_sub_list(date_index[0])
        except:
            print('file not found')
     	    try:
		ser.write('date not found')
	    except:
		print('not able to send  data as well as date not found')
        my_time = timedelta(0,0,0)  
        try:
            for i in range(0,len(Break),1):
                starting_time = a[Break[i][0]]
                ending_time = a[Break[i][len(Break[i])-1]]
                starting_time_slice = starting_time[11:19]
                ending_time_slice = ending_time[11:19]
                input = str(starting_time_slice)+'-'+str(ending_time_slice)
                s, e = [datetime.strptime(t, '%H:%M:%S') for t in input.split('-')]
                my_time = (time_diff(s,e))+my_time
        except:
            print('date not found')
        try:
	    ser.write(str(my_time))
        except:
            print('not able to send data')
    if(find_date[0:4] == 'CMD+'): 				#UnEncrypted data for that date
	try:
            find_date = find_date[4:]
            f = open('/home/pi/AgVa_5.0/data.csv', "r")
            data = f.readlines()
	    f.close()
            a = np.asarray(data)
            data_index = np.core.defchararray.find(a,find_date)
            date_index = np.where(data_index> -1)
            for i in range(0, len(date_index[0]),1):
	        for j in range(0, len(a[i]), 19):
	            try:
	                ser.write(a[i][j:j+19])
	                print(a[i][j:j+19])
	                time.sleep(0.001)
	            except:
	                print('error sending data')
	    ser.write('done')
	except:
	    ser.write('file not found')
    if(find_date[0:5] == 'CMDE+'): 				#Encrypted data for that date
        try:
	    find_date = find_date[5:]
            f = open('/home/pi/AgVa_5.0/data.csv', "r")
            data = f.readlines()
	    f.close()
            a = np.asarray(data)
            data_index = np.core.defchararray.find(a,find_date)
            date_index = np.where(data_index> -1)
            date_index = date_index[0]
	    DATA = a[date_index[:]]
	    data = ','.join(DATA)
	    compressed_data = zlib.compress(data,9) #zlib.Z_BEST_COMPRESSION)
	    compress_ratio = (float(len(data)) - float(len(compressed_data))) / float(len(data))
            print(compress_ratio*100)
	    print(len(data))
	    print(len(compressed_data))
	    try:
		for i in range(0, len(compressed_data), 19):
		    ser.write(compressed_data[i:i+19])
		    time.sleep(0.001)
	    except:
		print('not able to send data')
        except:
            print('file not found')
     	    try:
		ser.write('date not found')
	    except:
		print('not able to send  data as well as date not found')
    find_date = ''
