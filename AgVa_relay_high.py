#!/user/bin/env python
import datetime
import csv
import numpy as np
import Adafruit_ADS1x15
import os
import serial
from time import time,sleep
from smbus import SMBus
from math import *
import RPi.GPIO as GPIO
air=70
a = 12
#-----------------------------------------
ADDRESS_ABP=0x28
ADDRESS_SDP=0x25
# PIP, VTi, PEEP, RR, Insp time, Trif flow, Plat, inhale_time
#-----------------------------------------
#ser = serial.Serial(port='/dev/ttyAMAO',baudrate=9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)
#-----------------------------------------
#sleep(5)
try :
    ser = serial.Serial(
        port='/dev/ttyAMA0',
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)
except:
    print('BT initilaization error')
# Volume Calculation Constants
area_1 = 0.000950625 #151755924         #in metres square
area_2 = 0.000085785
dt=0.004
rho=1.225
volume_divisor=((1/(area_2**2))-(1/(area_1**2)))
#-----------------------------------------
prev_mode = 11
#-----------------------------------------
#Ventillator Constants
ratio =0
diff = 0
trigflow = 3.0          #in litres per minute
start_time =0
P_plat = 18.0
P_plat_value = 18.0
pump_pressure_array = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,99,100]
P_plat_array = [-0.187,-0.1845,-0.197,-0.203,-0.204,-0.2005,-0.194,-0.1995,-0.185,-0.184,-0.186,-0.185,-0.1885,-0.182,-0.174,-0.141,-0.079,0.043,0.2545,0.479,0.85,1.2305,1.6705,2.0005,2.5025,3.004,3.561,4.0785,4.609,5.112,5.6225,6.172,6.4705,7.203,7.6525,8.4705,8.9095,9.2965,9.972,10.264,10.99,11.2595,11.6125,12.2545,12.547,13.085,13.7955,14.2885,13.2245,12.6995,13.6345,15.049,15.1865,16.354,17.428,17.084,18.3295,19.7815,19.6855,20.8435,21.3315,22.4735,23.269,23.5675,25.056,25.305,26.925,28.3915,28.7505,29.536,30.12,32.1175,32.5535,34.3435,34.4245,35.7485,37.5145,38.31,39.299,40.5385,41.8925,42.9655,45.2675,46.0505,47.1285,48.612,49.702,50.663,52.6815,52.2155,52.6115,53.05,52.28,52.636,52.4185,52.481,52.5275,52.6055,52.3755,52.1915]
#P_plat_array = [0.4035,0.4075,0.2085,0.2145,0.213,0.2315,0.2225,0.224,0.221,0.21,0.2075,0.205,0.2075,0.209,0.217,0.2545,0.334,0.4505,0.6845,0.941,1.1755,1.477,1.868,2.3745,2.7425,3.2655,3.709,4.063,4.475,5.0825,5.5235,6.0795,6.426,7.1935,7.693,8.0885,8.838,9.5835,10.091,10.041,10.752,11.366,11.7645,12.533,12.886,13.483,14.1725,14.2555,12.9815,13.075,14.7235,15.0265,15.9435,16.078,17.132,18.14,19.0845,19.406,19.8975,20.914,21.6035,22.662,23.4835,24.5605,25.004,25.3285,26.657,27.3145,28.805,29.971,30.381,32.247,32.4725,33.264,35.2145,36.1555,36.6975,38.079,38.535,39.966,40.505,42.1015,43.302,44.8605,46.6285,48.1345,48.8025,49.9935,50.471,50.836,50.5825,51.134,50.8335,50.4995,51.093,50.9345,50.0425,50.469,50.442,51.0895]
PIP = 20               #in cmh2o
pump_pressure = 0 # variable initialzed values changes as according to P_plat
time_elapsed = 0
inhale_time = 2.0       #in seconds
peep_hole = [0,2,4,8.9,10.3,13,16.2,20,23,24,27]
indiff = 0
threshold = 1
flag=0
peep=0
BPM=15                #breaths per minute
cycle_time=60/BPM
current_time=0
VTi_max=500
ABP_flag = 0
ABP_fail = 0
SDP_fail = 0
#-----------------------------------------
peep_val = 0
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1
prev_FiO2 = 0
TITOT = 0
#-----------------------------------------
# MOTOR
GPIO.setmode(GPIO.BOARD)
GPIO.setup(18,GPIO.OUT)
#GPIO.setup(18,GPIO.OUT)
#GPIO.setup(22,GPIO.OUT)
motor_1=GPIO.PWM(18,50)
GPIO.setup(35,GPIO.OUT)
#motor_2=GPIO.PWM(18,50)
motor_1.start(30)
#motor_2.start(0)
GPIO.setup(37, GPIO.OUT)

#-------------------------------
FiO2_array = []
ABP_set = 0
setting_set = 0
exhale_time_last = 0
TOT_last = 0
mode_set = 0
SDP_set = 0
time_error_set = 0
ratio_set = 0
patient_set = 0
#-------------------------------
time_error = 0
time_error_last = 0
#------------------------------------------
thread_mode_status=False
flow_error_compensation = 0
flow_error_compensation_array = [0.9515,0.9335,0.976,0.9875,0.958,1.03,0.9155,1.048,0.976,1.001,1.03,1.03,1.012,1.8795,1.012,1.819,1.6375,1.1275,1.663,2.1525,2.298,3.2155,3.9665,3.942,4.6765,5.19,5.379,5.9965,6.7935,6.8405,7.766,8.252,8.14,8.8585,9.1455,10.033,10.2025,10.248,10.7025,9.9325,10.78,11.4935,12.2505,11.7475,11.178,13.5785,14.2295,13.196,13.2685,13.6155,13.3595,14.4435,14.446,14.17,16.0745,15.0535,15.3115,16.348,15.6795,17.3875,18.088,18.334,18.5115,18.6905,18.4465,19.9345,20.0825,20.2425,20.3805,21.626,20.8635,21.603,20.884,22.077,21.8985,22.268,23.038,22.453,24.271,24.5925,24.519,25.0515,25.8975,26.573,26.4235,26.588,27.2285,25.0765,27.5635,27.054,27.2485,26.941,28.658,28.8155,26.5745,26.7815,27.5405,26.3745,27.1165,27.5255]
data= ''
#------------------------------------------
flow_plat = 40
loop = 1
#------------------------------------------
# Global Variables
volume=0
first = 0
#------------------------------------------
previous_data = ''
patient_status = 0
pump_pressure_high = 0
P_plat_high = 0
P_plat_low = 0
pump_pressure_low = 0
volume_count = 0
peep_count = 0
pressure_count = 0
#--------------------------------------------
RR=[]
RR_time=0
SDP_flag = 0
#-----------------------------------------
MVi_array=[]
MVi=0
MVe_array=[]
MVe=0
#------------------------------------------
volume_peak_inhale=0
volume_peak_exhale=0
trigger = '0'
Pmean_array=[]
Pmean=0
clock_t2 = 0
def buzzer(position, bit):
    try:
	f = open('/home/pi/AgVa_5.0/buzzer.txt', 'r')
	data = f.readline()
	f.close()
	print(data)
	print(position)
	print(bit)
	if(len(data) > 10):
	    data = data.split(',')
	    data = map(int,data)
	    data[position] = bit
	    data = map(str,data)
	    data = ','.join(data)
	else:
	    data = '0,0,0,0,0,0,0,0,0,0'
	print(data)
        try:
            f = open('/home/pi/AgVa_5.0/buzzer.txt','w')
	    f.write(str(data))
            f.close()
        except:
            print('unable to write to file')
    except:
	print('unable to read file')
def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]
class Ventilator():
#    global clock_t2
    def __init__(self):
        #-------------------------------------
        #BUS READ AND WRITE
	try:
            self.bus = SMBus(1)  # default i2c bus
            self.bus.write_i2c_block_data(ADDRESS_SDP, 0x3F, [0xF9])
            sleep(0.8)
            self.bus.write_i2c_block_data(ADDRESS_SDP, 0x36, [0x03])
            sleep(0.1)
            print("INITAILIZED RUN")
        #-------------------------------------
	except:
	    self.SDP_initialization()
#	    motor_1.ChangeDutyCycle(0)

    def SDP_initialization(self):
        try:
            self.bus.write_i2c_block_data(ADDRESS_SDP, 0x3F, [0xF9])
            sleep(0.8)
            self.bus.write_i2c_block_data(ADDRESS_SDP, 0x36, [0x03])
            sleep(0.1)
        except:
            print('failed----')
            return 0

    #-----------------------------------------
    def FiO2(self):
	global prev_FiO2
        try:
            f = open('/home/pi/AgVa_5.0/FiO2.txt', "r")
            FiO2_val = f.readline()
	    prev_FiO2 = FiO2_val
            return FiO2_val
        except:
            return prev_FiO2
#-------------------------------------------
    #ABP presure sensor i2c read
    def ABP_pressure(self):
	global ABP_set, ABP_flag
        try:
	    val = self.bus.read_i2c_block_data(ADDRESS_ABP, 0)
            raw_value = ((val[0] & 0x3f) << 8)+val[1]
            press_psi = float(raw_value - 1638)/14745
            press_cmH2O = press_psi * 70.307
	    if(press_cmH2O  < 52 and ABP_set ==1):
		ABP_set = 0
		buzzer(8,0)
		try:
		    ser.write('ACK21')
		except:
		    print('not able to send data')
	    if(press_cmH2O >= 53.0 and ABP_set == 0):
		ABP_set = 1
		try:
		    ser.write('ACK11')
		except:
		    print('not able to send data')
		ABP_fail = 0
		return 0
	    ABP_fail = 0 
            return press_cmH2O
        except:
	    print('failed ABP')
	    if(ABP_set == 0):
		buzzer(8,1)
	        ABP_set = 1
		ABP_fail = 1
	        try:
		    ser.write('ACK11')
	        except:
		    print('not able to send data')
	    #GPIO.output(22,1)
	    return 0

    #----------------------------------------
    # setting file read function 
    def read_data(self):
	global setting_set,P_plat_high,P_plat_low,flow_plat,previous_data,pump_pressure_low,pump_pressure_high, mode_set,inhale_time,P_plat_value,P_plat_array,peep_val,prev_mode,pump_pressure_array,BPM,PIP,VTi_max,peep,P_plat,trigflow,cycle_time,data,pump_pressure
	try:
	    f = open('/home/pi/AgVa_5.0/setting.txt',"r")
	    data = f.readline()
	    f.close()
	    if(data != previous_data):
	        data_split= data.split(',')
#	    print(data_split)
#	    print(float(data_split[0]))
	        flow_plat = float(data_split[7])
#	    print('flow plat is')
#	    print(flow_plat)
	        inhale_time = float(data_split[6])
	        BPM = float(data_split[3])
	        cycle_time= (60.0/BPM)
	        PIP = float(data_split[0])
	        VTi_max = float(data_split[1])
	        peep_val = float(data_split[2])
	        peep_nearest = find_nearest(P_plat_array,peep_val)
	        peep_index = P_plat_array.index(peep_nearest)
	        peep = pump_pressure_array[peep_index]
	        P_plat = float(data_split[5])
	        trigflow = float(data_split[4])
	        trigflow = trigflow + 6
	        P_plat_value=find_nearest(P_plat_array,P_plat)
	        index= P_plat_array.index(P_plat_value)
	        pump_pressure = pump_pressure_array[index]
		pump_pressure_low = pump_pressure - (pump_pressure * 0.15)
		pump_pressure_high = pump_pressure + (pump_pressure * 0.15)
		if(prev_mode == 23 or prev_mode == 32):
		    pump_pressure_high_value = find_nearest(P_plat_array, PIP)
		    index = P_plat_array.index(pump_pressure_high_value)
		    pump_pressure_high = pump_pressure_array[index]
	#	    trigflow = trigFLow + 10
		if(prev_mode == 21 or prev_mode == 22):
		    if(VTi_max <= 100):
			P_plat = 5.0
		    elif(VTi_max <=200):
			P_plat = 10.0
		    elif(VTi_max <= 300):
			P_plat = 13.0
		    elif(VTi_max <= 400):
			P_plat = 16
		    else:
			P_plat = 18
		    P_plat_value = find_nearest(P_plat_array, P_plat)
		    index = P_plat_array.index(P_plat_value)
		    pump_pressure = pump_pressure_array[index]
		    pump_pressure_high_value = find_nearest(P_plat_array, PIP)
		    index = P_plat_array.index(pump_pressure_high_value)
		    pump_pressure_high = pump_pressure_array[index]
		    pump_pressure_low = pump_pressure -2
		if(pump_pressure_high >= 100):
		    pump_pressure_high = 100
		if(pump_pressure_low <= 0):
		    pump_pressure_low = 0
		P_plat_high = P_plat + (P_plat * 0.15)
		P_plat_low = P_plat - (P_plat * 0.15)
		if(P_plat_high >= 100):
		    P_plat_high = 100
		if(P_plat_low <= 0):
		    P_plat_low = 0
#	    print('here begins')
		previous_data = data
#	    print(inhale_time)
#	    print(BPM)
	        cycle_time=60.0/BPM
#	    print(PIP)
#	    print(VTi_max)
#	    print('peep in loop is')
#	    print(peep)
#	    print(P_plat)
#	    print(trigflow)
#	    print(cycle_time)
#	    print('reading the data')
#	    print(data)
	    f.close()
	    if(setting_set == 1):
	        setting_set = 0
	        try:
	            ser.write('ACK25')
	        except:
		    print('unable to send data')
#	    return data
	except:
	    print('error reading file')
	    if(setting_set == 0):
		setting_set = 1
	        try:
	            ser.write('ACK15')
	        except:
		    print('unable to send data')
	try:
	    f = open('/home/pi/AgVa_5.0/mode.txt',"r")
	    data = f.readline()
	    if(mode_set == 1):
	        mode_set = 0
	        try:
	            ser.write('ACK25')
	        except:
		    print('unable to send data')
	    if(data == "11" or data == "12" or data == "13" or data == "21" or data == "23" or data == "22" or data == "31" or data == "32"):
	        mode_read = int(data)
	        if(mode_read != prev_mode):
		    return 1
	        else: 
#		    print('i was here')
		    return 0
	    else:
		return 0
	except:
	    if(mode_set == 0):
		mode_set = 1
	        try:
	            ser.write('ACK15')
	        except:
		    print('unable to send data')
	    print('error reading mode file')

    #------------------------------------------
    # -----------------------------------------
    # SDP810_500Pa presure sensor i2c read
    def SDP_pressure(self):
	global SDP_set, SDP_flag 
        try:
	    read = self.bus.read_i2c_block_data(ADDRESS_SDP, 0)
            pressure_value = read[0] + float(read[1]) / 255
            diff_press=0
            if 0 <= pressure_value < 128:
                diff_press = pressure_value *255/60
            elif 128 < pressure_value <= 256:
                diff_press = -(256 - pressure_value) *255/60
            elif pressure_value == 258:
                diff_press = 500
	    if(SDP_set == 1):
		SDP_set = 0
		buzzer(9,0)
	        try:
	            ser.write('ACK20')
	        except:
		    print('unable to send data')
	    SDP_flag = 0
            return round(diff_press,2)
        except IOError:
	    print('failed')
#	    sleep(1)
#	    motor_1.ChangeDutyCycle(40)
	    self.SDP_initialization()
	    SDP_flag = 1
	    if(SDP_set == 0):
		buzzer(9,1)
		SDP_set = 1
	        try:
	            ser.write('ACK10')
	        except:
		    print('unable to send data')
	   # GPIO.output(22,1)
	    return -9999


    #----------------------------------------------
    # Calculate Volume Flow From SDP810_500Pa Sensor
    def Flow(self):

        global volume,clock_t2,P_plat,flow_error_compensation,indiff
        diff=self.SDP_pressure()
	if(diff == -9999):
	    print('we are in  a mess')
	    return -9999
        #print(diff)
        massFlow=1000*(sqrt((abs(diff)*2*rho)/volume_divisor))
        volFlow =( massFlow/rho)*1000
        volFlow_min=(volFlow*60)
        last = time() - first
       # volFlow = volFlow *1000
	clock_t1= time()
	clock = (clock_t1 - clock_t2)
#	print('clock_t1 is')
#	print(clock_t1)
#	print('clock_t2 is')
#	print(clock_t2)
#	print('clock is')
#	print(clock)
#	print('We have got a P_plat of')
#	print(P_plat)
# 	if(P_plat < 10):
# 	    volFlow_min = volFlow_min - 2500
# 	elif(P_plat >= 10 and P_plat <=15):
# 	    volFlow_min = volFlow_min - 3500
# 	elif(P_plat >15 and P_plat <= 20):
# #	    print('git in here')
# 	    volFlow_min = volFlow_min - 9300
# 	elif(P_plat > 20):
# 	   volFlow_min = volFlow_min - 8700
# 	if((volFlow_min/1000) > 5):
#	    print(volFlow_min/1000)
#	    print('came i here')
	try:
	    flow_P_plat_value=find_nearest(P_plat_array,indiff)
	    flow_index= P_plat_array.index(flow_P_plat_value)
	    pump_pressure_index = pump_pressure_array[flow_index]
	    flow_error_compensation = flow_error_compensation_array[pump_pressure_index]
	    volFlow_min = volFlow_min - (flow_error_compensation*1000)
	except:
	    print('compensation error')
	    volFlow_min = volFlow_min - (27*1000)
	if((volFlow_min/1000) > 4):
            volume=((volFlow*(clock*0.75))+volume)
	clock_t2= time()
#	print('volume is')
#	print(volume)
        return volume

    def rate(self):
	global peep_val,peep_hole,P_plat,indiff,loop
	diff=self.SDP_pressure()
	compensation = 0
	if( diff == -9999):
	    print('rate cannot be calculated SDP failed')
	    return -9999
#	print('SDP reading is')
#	print(diff)
	massFlow=1000*(sqrt((abs(diff)*2*rho)/volume_divisor))
	volFlow= massFlow/rho
	volFlow_min=volFlow*60
# 	if(P_plat < 10):
# 	    volFlow_min  = volFlow_min -  2.5
# 	elif(P_plat >= 10 and P_plat <=15):
# 	    volFlow_min = volFlow_min - 3.5
# 	elif(P_plat > 15 and P_plat <=20):
# 	    volFlow_min = volFlow_min - 5.3
# 	elif(P_plat > 20):
# 	    volFlow_min = volFlow_min - 6.3
	try:
	    flow_P_plat_value=find_nearest(P_plat_array,indiff)
	    flow_index= P_plat_array.index(flow_P_plat_value)
	    pump_pressure_index = pump_pressure_array[flow_index]
	    flow_error_compensation = flow_error_compensation_array[pump_pressure_index]
	    if(loop == 1):
	        volFlow_min = volFlow_min - (flow_error_compensation)
	    elif(loop == 0):
		volFlow_min = volFlow_min + (flow_error_compensation)
	except:
	    volFlow_min = volFlow_min - (27)
 	    print('compenation error')
	volFlow_min = abs(volFlow_min)
	return volFlow_min

    def Vol_Flow(self,volume):
        diff=self.SDP_pressure()
        massFlow=1000*((abs(diff)*volume_divisor)**(0.5))
        volFlow = massFlow/rho
        print('volflow')
        print(volFlow)
	clock_t1=time()
	clock= (clock_t1 - clock_t2)*1.1
	print('clock is')
	print(clock,5)
        volume=volFlow*clock+volume
	clock_t2= time()
        volume_ml=volume*1000
        volFlow_min=volFlow*60
        return volume_ml
    #------------------------------------------------
    def measure_temp(self):
        temp = os.popen("vcgencmd measure_temp").readline()
	temp = temp.replace("temp=","")
        return (temp.replace("'C\n",""))
    #-------------------------------------------------
    # Pressure Control Intermitent Mode of Ventilation
    def PC_IMV(self):
        global loop,patient_set,exhale_time_last,TOT_last,ABP_flag,TITOT,pressure_count,peep_count, SDP_flag,flow_plat,indiff,patient_status,pump_pressure,pump_pressure_low,pump_pressure_high,ratio_set,time_error_set,thread_mode_status,time_error, time_error_last,ABP_flag,P_plat_value,prev_mode,data,ratio,RR_time,indiff,peep_val, peep_hole, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	patient_set = 0
	exhale_time_last = time()
	TOT_last = time()
	peep_count = 0
	loop = 1
	pressure_count = 0
	ratio_set = 0
	peep_set = 0
	time_error_set = 0
	GPIO.output(35, GPIO.HIGH)
        start_time=time() - 5000
	time_error_last = time()
	sending_time = 0.2
        try:
            while True:
		prev_mode = 12
		ABP_flag = 0
#		print('we are in PC_IMV mode')
		data= self.read_data()
#		print('mode is')
#		print(data)
		if(data == 1):
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		print(VTi_max)
#		print('peep in main loop is')
#		print(peep)
#		print(peep_val)
#		print(P_plat)
#		print(P_plat_value)
#		print('here the pump pressure is')
#		print(pump_pressure)
#		GPIO.output(22, GPIO.LOW)
		volFlow_rate = self.rate()
		indiff= self.ABP_pressure()
		if(indiff == -8888):
			ABP_flag = 1
			indiff = 0
		packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(0)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.07):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print('pump_pressure is')
#		print(pump_pressure)
                flow=self.rate()
#		print('before flow')
#		print(flow)
	#	flow = flow - peep_hole[int(peep_val)]
#		print('after flow')
#		print(flow)
                current_time=time()
                volume=0
		TOT_last = current_time - exhale_time_last
		peep_val_send = indiff
		SDP_flag = 0
		time_error = time() - time_error_last
		loop = 1
                if(flow>trigflow or (current_time-start_time)>cycle_time):
		    RR_time=current_time-start_time
		    pump_pressure_now = pump_pressure
		    indiff = self.ABP_pressure()
		    flow_flag = 0
#		    if((pump_pressure /2) < peep):
#			motor_1.ChangeDutyCycle(peep + 2)
#			pump_pressure_now = peep +2
#		    else:
#		    print('indiff right now is')
#		    print(indiff)
#		    if(indiff <= P_plat/2):
                    motor_1.ChangeDutyCycle(100)
		    if(P_plat <= 22):
			motor_1.ChangeDutyCycle(80)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
#		        flow_flag = 0
#		    else:
#			motor_1.ChangeDutyCycle(pump_pressure)
#			flow_flag = 1
#			pump_pressure_now = pump_pressure /2
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
#		    if(time_error > 0.7):
#			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
		    if(RR_time > cycle_time):
			trigger='0'
		    else:
			trigger='1'
                 #   indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    volFlow_rate = self.rate()
		    VTi_volume=0
		    pump_flag = 0
		   # flow_flag = 0
		    current_time = time()
		    peak_insp_pressure = 0
		    pump_pressure_now = pump_pressure /2
#		    flow_zero_intialize = time() # clock starts to detect zero flow
#		    flow_zero_flag = 0
#		    flow_zero_elapsed = 0
                    while(indiff <= PIP and time_elapsed_inhale <= inhale_time):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
                        t3 = time()
			GPIO.output(37, GPIO.HIGH)
#			if(volFlow_rate <= 2):
#			    flow_zero_flag = 1
#			    flow_zero_initialize = time()
#		        if(flow_zero_flag == 1):
#			    flow_zero_elapsed = time() - flow_zero_initialize
#			    if(flow_zero_elapsed >= )
#			    flow_zero_elapsed = time() - flow_zero_initialize 
			if(ABP_flag == 0):
                            indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
                   #     print(indiff)
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volFlow_rate=self.rate()
			if(indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.4): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
#			    print('CMHO reacged')
#			    print(indiff)
			if(P_plat <= 17 and indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.2): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			if(flow_flag == 1 and volFlow_rate <= flow_plat):
			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(volume == -9999):
			    SDP_flag = 1;
			if(sending_time > 0.07):
			    packet_inhalation =('A@' + str(int(indiff)) + ','+str(int(volFlow_rate)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
#			    print(packet_inhalation)
			    send_last_time= time()
#			if(volFlow_rate < flow_plat -3 and pump_pressure_now < pump_pressure):
#			    pump_pressure_now = pump_pressure_now + 4
#			    print('in 1st loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
#			elif(volFlow_rate > flow_plat + 3 and pump_pressure_now > pump_pressure):
#			    pump_pressure_now = pump_pressure_now -4
#			    print('in 2nd loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
			if(flow_flag == 2):
			    if(volFlow_rate <= flow_plat -5 and pump_pressure_now <= P_plat):
		#	   # if(pump_flag == 0):
			      #      pump_pressure_now  = pump_pressure_now + 5
			     #   else:
				pump_pressure_now = pump_pressure_now + 1
	#		    print('the pump_pressure now is')
	#		    print(pump_pressure_now)
	#		    print('pump pressure is')
	#		    print(pump_pressure)
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= P_plat/2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			      #  pump_flag = 1
			sending_time=t3 - send_last_time
#			print('flow is')
#			print(volFlow_rate)
#			print(time_elapsed_inhale)
#			print(inhale_time)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(indiff >= (P_plat+3)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
                    flag=1
		    loop = 0
#		    print('volume is')
#		    print(VTi_volume)
		    motor_1.ChangeDutyCycle(0)
	#	    sleep(0.01)
		    GPIO.output(35, GPIO.LOW) # HIGH level triggered relay
		    Pmean= sum(Pmean_array)/len(Pmean_array)
		    print('P_plat is')
		    print(int(peak_insp_pressure))
		    print(P_plat)
                    RR.append(RR_time)
		    peak_insp_pressure = int(peak_insp_pressure)
		    if(peak_insp_pressure > P_plat and patient_status == 1 and ABP_flag == 0 and peak_insp_pressure < PIP and pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure - 1
			print('-1')
		    elif(peak_insp_pressure < P_plat and patient_status == 1 and ABP_flag == 0 and peak_insp_pressure < PIP and  pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure + 1
			print('+1')
#		    print('volume is')
#		    print(volume*0.4)
#		    print('Vpeak Inspiratory')
#		    print(volume_peak_inhale)
		    MVi_array.append(VTi_volume)
#		    print('trigger is')
#		    print(trigger)
#		    print('Pmean is')
#		    print(Pmean)
		    GPIO.output(37, GPIO.LOW)
		if(flag == 1):
		    patient_set = 0
		    if((volume_peak_inhale > 80 or peak_insp_pressure <= P_plat * 0.5)  and patient_set == 0 ):
			patient_set = 1
			buzzer(5,1)
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		    if((volume_peak_inhale <= 80 or peak_insp_pressure> P_plat * 0.5) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = 60/BPM
			if(BPM > 50):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				serw.write('ACK59')
			    except:
				print('unable to send')
 #                      print('BPM is')
  #                     print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(int(peak_insp_pressure)) + "," + str(int(VTi_volume)) + "," + str(int(volume_peak_inhale)) + "," + str(int(Pmean)) + "," + str(int(MVi)/1000) + '#')
#		        print('packet length is')
#		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    buzzer(7,0)
		    q= time()
		    volume = 0
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    while(time_elap < 0.5):
			w=time()
			time_elap=w-q
                        indiff = self.ABP_pressure()
			ratio = peep_val/P_plat
                        if(ratio > 0.8 and time_elap >= 0.005):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.12 and P_plat < 28):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.2 and P_plat >=28):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.001):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.3):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
# 			if(indiff <= (peep + 1)):
# 			    print('turning off the relay')
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 500 or diff == -500 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
#			print(volFlow_rate)
			volume_peak_exhale= max(volume_peak_exhale, volFlow_rate)
#			print('packet_exhal;ation conatins is')
#			print(packet_exhalation)
                    #motor_2.ChangeDutyCycle(0)
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    start_time=current_time
		    sending_time=200
		    send_last_time = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time-0.5) and diff <= 0) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
# 			if(indiff <= (peep + 1)):
# 			    GPIO.output(35, GPIO.LOW)
# 			    print('turning off the relay')
			volume_peak_exhale=max(volume_peak_exhale,volFlow_rate)
			packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
		    exhale_time_last = time()
                    flag=0 
		    loop = 1
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    print(volFlow_rate)
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    print('peak volume flow rate exhale')
#		    print(volume_peak_exhale)
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    FiO2 = self.FiO2()
		    print('FiO2 is')
		    print(FiO2)
		    MVe_array.append(volume)
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(int(peep_val_send)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(int(volume_peak_exhale)) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
#			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
		    try:
		        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   		            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		            now = datetime.datetime.now()
		            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
		    except:
			print('data file open error')
        except KeyboardInterrupt:
            return
    #--------------------------------------------------

    #--------------------------------------------------
    # Volume Control Intermitent Mode of Ventilation
    def VC_IMV(self):
        global loop,P_plat,P_plat_high,exhale_time_last, TOT_last,TITOT,P_plat_low,ABP_flag,pressure_count,peep_count,volume_count, SDP_flag,patient_set,flow_plat,ratio_set,patient_status,pump_pressure,pump_pressure_low, pump_pressure_high,time_error_set,thread_mode_status,time_error, time_error_last,prev_mode,P_plat_value,peep_hole, peep_val,data,ratio,RR_time,indiff, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	volume_count = 0
	exhale_time_last = time()
	TOT_last = time()
	peep_count = 0
	ratio_set = 0
	loop = 1
	pressure_count = 0
	volume_set = 0
	patient_set = 0
	peep_set = 0
	GPIO.output(35, GPIO.HIGH)
	time_error_set = 0
        start_time=time() - 5000
	time_error_last = time()
	sending_time = 0.2
        try:
            while True:
		prev_mode = 22
		print('we are in VC_IMV mode')
		data= self.read_data()
		print(data)
		if(data == 1):
		    break;
		print(inhale_time)
		print(BPM)
		print(PIP)
		print(VTi_max)
		print(peep)
		print(P_plat)
		print(trigflow)
		print(pump_pressure)
	#	GPIO.output(22, GPIO.LOW)
		clock_t2 = time()
		volFlow_rate = self.rate()
		indiff= self.ABP_pressure()
		packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(0)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.07):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print('pump_pressure is')
#		print(pump_pressure)
                flow=self.rate()
#                flow=self.rate()
#		flow = flow - peep_hole[int(peep_val)]
                current_time=time()
                volume=0
		TOT_last = current_time - exhale_time_last
		peep_val_send = indiff
		time_error = time() - time_error_last
		SDP_flag = 0
		loop = 1
                if(flow>trigflow or (current_time-start_time)>cycle_time):
		    RR_time=current_time-start_time
		    pump_pressure_now = pump_pressure
# 		    if((pump_pressure /2) < peep):
# 			motor_1.ChangeDutyCycle(peep + 2)
# 			pump_pressure_now = peep +2
# 		    else:
                    motor_1.ChangeDutyCycle(100)
		    if(P_plat <= 22):
			motor_1.ChangeDutyCycle(80)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
#			pump_pressure_now = pump_pressure /2
 #                   RR_time=current_time-start_time
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(time_error > 0.7):
			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
		    if(RR_time > cycle_time):
			trigger='0'
		    else:
			trigger='1'
                    #motor_2.ChangeDutyCycle(pump_pressure)
                    indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    volume=self.Flow()
		    print('the value of volume is')
		    print(volume)
		    if(volume == -9999):
			SDP_flag = 1
			print('well hrre you gooooooooooooooooooooooooooooooooo')
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    VTi_volume=0
		    flow_flag = 0
		    volFlow_rate = self.rate()
		    pump_flag = 0
		    pump_pressure_now = pump_pressure/2
		    peak_insp_pressure = 0
		    peak_flow = volFlow_rate
                    while((volume <= VTi_max or trigger == '1') and indiff < PIP and time_elapsed_inhale <= inhale_time):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
			GPIO.output(37, GPIO.HIGH)
                        t3 = time()
			print('here volume is')
                        indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
                        print(volume)
			print('tme elapsed inhale is')
			print(time_elapsed_inhale)
			print('VTi is')
			print(VTi_max)
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
			    print('hello its me')
			    volFlow_rate=self.rate()
			if(volFlow_rate > peak_flow):
			    peak_flow = volFlow_rate
			if(volFlow_rate <  peak_flow * 0.25 and time_elapsed_inhale >= 0.5 and trigger == '1'):
			    break 
			if(sending_time > 0.07):
			    packet_inhalation =('A@' + str(int(indiff)) + ','+str(int(volFlow_rate)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
			    print(packet_inhalation)
			    send_last_time= time()
			sending_time=t3 - send_last_time
			print('inhale_time is')
			if(indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.4): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    print('CMHO reacged')
			    print(indiff)
			if(P_plat <= 17 and indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.2): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			if(flow_flag == 1 and volFlow_rate <= flow_plat):
			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(flow_flag == 2):
			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
				pump_pressure_now = pump_pressure_now + 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= 2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
# 			if(pump_pressure_now <= pump_pressure and volFlow_rate <= flow_plat):
# 			    if(pump_flag == 0):
# 			        pump_pressure_now  = pump_pressure_now + 1
# 			    else:
# 				pump_pressure_now = pump_pressure_now + 0.5
# 	#		    print('the pump_pressure now is')
# 	#		    print(pump_pressure_now)
# 	#		    print('pump pressure is')
# 	#		    print(pump_pressure)
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			elif(volFlow_rate > flow_plat and pump_pressure_now >( pump_pressure /2)):
# 			    pump_pressure_now = pump_pressure_now - 0.5
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			    pump_flag = 1
			print(inhale_time)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(indiff >= (PIP+2)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    GPIO.output(37, GPIO.LOW)
                    flag=1
		    loop = 0
		    motor_1.ChangeDutyCycle(0)
		    sleep(0.01)
		    GPIO.output(35, GPIO.LOW) # low level triggered relay
		    if(VTi_volume < (VTi_max * 0.8)):
			volume_count = volume_count + 1
			if(volume_count >= 3):
			    buzzer(7,1)
			    try:
			        ser.write('ACK36')
			    except:
			        print('unable to send')
		    else:
			buzzer(7,0)
			volume_count = 0
			try:
			    ser.write('ACK46')
			except:
			    print('unable to send')
		    Pmean= sum(Pmean_array)/len(Pmean_array)
#		    print('Pmean is')
		    print('Here it comessssssssss')
		    print(int(VTi_volume))
		    print(VTi_max)
		    print(peak_insp_pressure)
		    print(patient_status)
                    RR.append(RR_time)
		    print(pump_pressure_high)
		    peak_insp_pressure = int(peak_insp_pressure)
		    if(VTi_volume > VTi_max and peak_insp_pressure > peep_val+3 and ABP_flag == 0 and SDP_flag == 0 and patient_status == 1 and pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure - 1
			P_plat = peak_insp_pressure
			print('-1')
		    elif(VTi_volume < VTi_max and ABP_flag == 0 and SDP_flag == 0 and peak_insp_pressure <= PIP and patient_status ==1 and pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure + 2
			P_plat = peak_insp_pressure
			print('+1')
#		    print(volume*0.4)
#		    print('Vpeak Inspiratory')
#		    print(volume_peak_inhale)
		    MVi_array.append(VTi_volume)
#		    print('trigger is')
#		    print(trigger)
#		    print('Pmean is')
#		    print(Pmean)
		if(flag == 1):
	   	    patient_set = 0
		    if((volume_peak_inhale > 80 or peak_insp_pressure <= P_plat * 0.5)  and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		       # GPIO.output(22, GPIO.HIGH)
		    if((volume_peak_inhale <= 80 or peak_insp_pressure> P_plat * 0.5) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = 60/BPM
			if(BPM > 50):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				serw.write('ACK59')
			    except:
				print('unable to send')
 #                       print('BPM is')
  #                      print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(int(peak_insp_pressure)) + "," + str(int(VTi_volume)) + "," + str(int(volume_peak_inhale)) + "," + str(int(Pmean)) + "," + str(int(MVi)/1000) + '#')
#		        print('packet length is')
		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    volume = 0
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    while(time_elap < 0.5):
			w=time()
			time_elap=w-q
                        indiff = self.ABP_pressure()
			P_plat = int(P_plat)
			if(P_plat <= 0):
			    P_plat = 1
			ratio = peep_val/P_plat
                        if(ratio > 0.8 and time_elap >= 0.005):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.12 and P_plat < 28):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.2 and P_plat >=28):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.001):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.3):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 500 or diff == -500 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
#			print((packet_exhalation))
			volume_peak_exhale= max(volume_peak_exhale, volFlow_rate)
			print('packet_exhal;ation conatins is')
			print(packet_exhalation)
                    #motor_2.ChangeDutyCycle(0)
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    start_time=current_time
		    sending_time=200
		    send_last_time = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time-0.5) and diff <= 0) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
			indiff = self.ABP_pressure()
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    diff = self.SDP_pressure()
			    volFlow_rate=self.rate()
			volume_peak_exhale=max(volume_peak_exhale,volFlow_rate)
			packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
                    flag=0
		    loop = 1
		    exhale_time_last = time()
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    print('Ti/Tot')
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    print('peak volume flow rate exhale')
#		    print(volume_peak_exhale)
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    MVe_array.append(volume)
		    FiO2 = self.FiO2()
		    print('FiO2 is')
		    print(FiO2)
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(int(peep_val_send)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(int(volume_peak_exhale)) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
		    try:
		        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   		            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		            now = datetime.datetime.now()
		            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
		    except:
			print('data file open error')
        except KeyboardInterrupt:
            return

    def PC_CMV(self):
	global loop,patient_set,exhale_time_last, TOT_last,flow_plat,TITOT,ABP_flag,peep_count,pressure_count, SDP_flag,pump_pressure_low,patient_status,pump_pressure, pump_pressure_high,ratio_set,time_error_set,thread_mode_status,time_error, time_error_last,prev_mode,data,P_plat_value,ratio,RR_time,indiff, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        volume=0
	patient_set = 0
	exhale_time_last = time()
	TOT_last = time()
	peep_count = 0
	loop = 1
	peep_set = 0
	pressure_count = 0
	ratio_set = 0
	time_error_set = 0
	GPIO.output(35, GPIO.HIGH)
	thread_mode_status = True
	start_time = time() - 5000
	time_error_last = time()
	sending_time = 0.2
	flag = 0
        try:
            while True:
		prev_mode = 11
		print('we are in PC_CMV mode')
		data= self.read_data()
		print(data)
		if(data == 1):
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		print(VTi_max)
#		print(peep)
#		print(P_plat)
#		print(trigflow)
#		print(pump_pressure)
		trigger = 0
	#	GPIO.output(22, GPIO.LOW)
		clock_t2 = time()
		volFlow_rate = self.rate()
		indiff= self.ABP_pressure()
		packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(0)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.07):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print('pump_pressure is')
#		print(pump_pressure)
                flow=self.rate()
                current_time=time()
		TOT_last = current_time - exhale_time_last
                volume=0
		peep_val_send = indiff
		SDP_flag = 0
		loop = 1
		time_error = time() - time_error_last
                if((current_time-start_time)>cycle_time):
		    pump_pressure_now = pump_pressure
#		    if((pump_pressure /2) < peep):
# 			motor_1.ChangeDutyCycle(peep + 2)
# 			pump_pressure_now = peep +2
# 		    else:
                    motor_1.ChangeDutyCycle(100)
		    if(P_plat <= 22):
			motor_1.ChangeDutyCycle(80)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
	#		pump_pressure_now = pump_pressure /2
                    RR_time=current_time-start_time
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(time_error > 0.7):
			time_error_set = 0
#			try:
#			    ser.write('ACK24')
##			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
#		    if(RR_time > cycle_time):
#			trigger='0'
#		    elif(flow>trigflow):
#			trigger='1'
                    #motor_2.ChangeDutyCycle(pump_pressure)
                    indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    volume=self.Flow()
		    if(volume == -9999):
			SDP_flag = 1
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    VTi_volume=0  
		    flow_flag = 0
		    volFlow_rate = self.rate()
		    pump_pressure_now = pump_pressure /2
		    peak_insp_pressure = 0
		    pump_flag = 0
                    while(indiff <= PIP and time_elapsed_inhale <= inhale_time):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
			GPIO.output(37, GPIO.HIGH)
                        t3 = time()
                        indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
                   #     print(indiff)
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volFlow_rate=self.rate()
			if(sending_time > 0.07):
			    packet_inhalation =('A@' + str(int(indiff)) + ','+str(int(volFlow_rate)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
			    print(packet_inhalation)
			    send_last_time= time()
			sending_time=t3 - send_last_time
			if(indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.4): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    print('CMHO reacged')
			    print(indiff)
			if(P_plat <= 17 and indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.2): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			if(flow_flag == 1 and volFlow_rate <= flow_plat):
			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(flow_flag == 2):
			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
				pump_pressure_now = pump_pressure_now + 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= 2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
# 			if(pump_pressure_now <= pump_pressure and volFlow_rate <= flow_plat):
# 			    if(pump_flag == 0):
# 			        pump_pressure_now  = pump_pressure_now + 1
# 			    else:
# 				pump_pressure_now = pump_pressure_now + 0.5
# 	#		    print('the pump_pressure now is')
# 	#		    print(pump_pressure_now)
# 	#		    print('pump pressure is')
# 	#		    print(pump_pressure)
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			elif(volFlow_rate > flow_plat and pump_pressure_now >( pump_pressure /2)):
# 			    pump_pressure_now = pump_pressure_now - 0.5
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			    pump_flag = 1
#			print(time_elapsed_inhale)
#			print(inhale_time)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(indiff >= (P_plat+3)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    GPIO.output(37,GPIO.LOW)
                    flag=1
		    loop = 0
		    motor_1.ChangeDutyCycle(0)
		    sleep(0.01)
		    GPIO.output(35, GPIO.LOW) # low level triggered relay
		    Pmean= sum(Pmean_array)/len(Pmean_array)
#		    print('Pmean is')
		    print('P_plat is')
		    print(int(peak_insp_pressure))
		    print(P_plat)
#		    print(Pmean)
                    RR.append(RR_time)
#		    print('volume is')
#		    print(volume*0.4)
		    peak_insp_pressure = int(peak_insp_pressure)
		    if(peak_insp_pressure > P_plat and patient_status == 1 and ABP_flag == 0 and peak_insp_pressure < PIP and  pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure - 1
			print('-1')
		    elif(peak_insp_pressure < P_plat and patient_status == 1 and ABP_flag == 0 and peak_insp_pressure < PIP and  pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure + 1
			print('+1')
#		    print('Vpeak Inspiratory')
#		    print(volume_peak_inhale)
		    MVi_array.append(VTi_volume)
#		    print('trigger is')
#		    print(trigger)
#		    print('Pmean is')
#		    print(Pmean)
		if(flag == 1):
		    patient_set = 0
		    if((volume_peak_inhale > 80 or peak_insp_pressure <= P_plat * 0.5)  and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		      #  GPIO.output(22, GPIO.HIGH)
		    if((volume_peak_inhale <= 80 or peak_insp_pressure> P_plat * 0.5) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = 60/BPM
			if(BPM > 50):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				serw.write('ACK59')
			    except:
				print('unable to send')
 #                       print('BPM is')
  #                      print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(int(peak_insp_pressure)) + "," + str(int(VTi_volume)) + "," + str(int(volume_peak_inhale)) + "," + str(int(Pmean)) + "," + str(int(MVi)/1000) + '#')
#		        print('packet length is')
		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    volume = 0
		    buzzer(7,0)
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    while(time_elap < 0.5):
			w=time()
			time_elap=w-q
                        indiff = self.ABP_pressure()
			ratio = peep_val/P_plat
                        if(ratio > 0.8 and time_elap >= 0.005):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.12 and P_plat < 28):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.2 and P_plat >=28):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.001):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.3):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			packet_exhalation= ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
#			print((packet_exhalation))
			volume_peak_exhale= max(volume_peak_exhale, volFlow_rate)
			print('packet_exhal;ation conatins is')
			print(packet_exhalation)
                    #motor_2.ChangeDutyCycle(0)
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    start_time=current_time
		    sending_time=200
		    send_last_time = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time-0.5) and diff <= 0) or (SDP_flag == 1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
			indiff = self.ABP_pressure()
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
			volume_peak_exhale=max(volume_peak_exhale,volFlow_rate)
			packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
                    flag=0
		    loop = 1
		    exhale_time_last = time()
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    print('Ti/Tot')
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    print('peak volume flow rate exhale')
#		    print(volume_peak_exhale)
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    MVe_array.append(volume)
	   	    FiO2 = self.FiO2()
		    print('FiO2 is')
		    print(FiO2)
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(int(peep_val_send)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(int(volume_peak_exhale)) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
		    try:
		        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   		            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		            now = datetime.datetime.now()
		            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
		    except:
			print('data file open error')
	except KeyboardInterrupt:
	    return

    def VC_CMV(self):
	global loop,P_plat,exhale_time_last, TOT_last,patient_set,TITOT,P_plat_high,pressure_count,peep_count,volume_count, ABP_flag, SDP_flag, P_plat_low,flow_plat,ratio_set,pump_pressure,pump_pressure_high,pump_pressure_low,patient_status,time_error_set,thread_mode_status,time_error, time_error_last,prev_mode,data,P_plat_value,ratio,RR_time,indiff, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        volume=0
	loop = 1
	peep_count = 0
	volume_count = 0
	exhale_time_last = time()
	TOT_last = time()
	pressure_count = 0
	patient_set = 0
	ratio_set = 0
	time_error_set = 0
	peep_set = 0
	volume_set = 0
	GPIO.output(35, GPIO.LOW)
	thread_mode_status = True
	start_time = time() - 5000
	time_error_last = time()
	flag = 0
	sending_time = 0.2
        try:
            while True:
		prev_mode = 21
		print('we are in VC_CMV mode')
		data= self.read_data()
		print(data)
		if(data == 1):
		    break;
		print(inhale_time)
		print(BPM)
		print(PIP)
		print(VTi_max)
		print(peep)
		print(P_plat)
		print(trigflow)
		print(pump_pressure)
		trigger = 0
	#	GPIO.output(22, GPIO.LOW)
		clock_t2 = time()
		volFlow_rate = self.rate()
		indiff= self.ABP_pressure()
		packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(0)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.07):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print('pump_pressure is')
#		print(pump_pressure)
                flow=self.rate()
                current_time=time()
		TOT_last = current_time - exhale_time_last
                volume=0
		peep_val_send = indiff
		SDP_flag = 0
		time_error = time() - time_error_last
		loop = 1
                if((current_time-start_time)>cycle_time):
		    pump_pressure_now = pump_pressure
# 		    if((pump_pressure /2) < peep):
# 			motor_1.ChangeDutyCycle(peep + 2)
# 			pump_pressure_now = peep +2
# 		    else:
                    motor_1.ChangeDutyCycle(100)
		    if(P_plat <= 22):
			motor_1.ChangeDutyCycle(80)
#	#	    if(P_plat <= 17):
	#		motor_1.ChangeDutyCycle(65)
# 			pump_pressure_now = pump_pressure /2
                    RR_time=current_time-start_time
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(time_error > 0.7):
			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1 
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
	#	    if(RR_time > cycle_time):
	#		trigger='0'
	#	    elif(flow>trigflow):
	#		trigger='1'
                    #motor_2.ChangeDutyCycle(pump_pressure)
                    indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    volume=self.Flow()
		    if(volume == -9999):
			SDP_flag = 1
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    VTi_volume=0
		    flow_flag = 0
		    volFlow_rate = self.rate()
		    pump_pressure_now = pump_pressure /2
		    peak_insp_pressure = 0
		    pump_flag = 0
                    while(volume <= VTi_max and time_elapsed_inhale <= inhale_time):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
			GPIO.output(37, GPIO.HIGH)
                        t3 = time()
                        indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
                   #     print(indiff)
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volFlow_rate=self.rate()
			if(sending_time > 0.07):
			    packet_inhalation =('A@' + str(int(indiff)) + ','+str(int(volFlow_rate)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
			    print(packet_inhalation)
			    send_last_time= time()
			sending_time=t3 - send_last_time
			if(indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.4): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    print('CMHO reacged')
			    print(indiff)
			if(P_plat <= 17 and indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.2): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			if(flow_flag == 1 and volFlow_rate <= flow_plat):
			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(flow_flag == 2):
			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
				pump_pressure_now = pump_pressure_now + 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= 2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
# 			if(pump_pressure_now <= pump_pressure and volFlow_rate <= flow_plat):
# 			    if(pump_flag == 0):
# 			        pump_pressure_now  = pump_pressure_now + 1
# 			    else:
# 				pump_pressure_now = pump_pressure_now + 0.5
# 	#		    print('the pump_pressure now is')
# 	#		    print(pump_pressure_now)
# 	#		    print('pump pressure is')
# 	#		    print(pump_pressure)
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			elif(volFlow_rate > flow_plat and pump_pressure_now >( pump_pressure /2)):
# 			    pump_pressure_now = pump_pressure_now - 0.5
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			    pump_flag = 1
#			print(time_elapsed_inhale)
#			print(inhale_time)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(indiff >= (PIP+2)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    GPIO.output(37, GPIO.LOW)
		    loop = 0
                    flag=1
		    motor_1.ChangeDutyCycle(0)
		    sleep(0.01)
		    GPIO.output(35, GPIO.LOW) # low level triggered relay
		    if(VTi_volume < (VTi_max * 0.8)):
			volume_count = volume_count + 1
			if(volume_count >= 3):
			    buzzer(7,1)
			    try:
			        ser.write('ACK36')
			    except:
			        print('unable to send')
		    else:
			buzzer(7,0)
			volume_count = 0
			try:
			    ser.write('ACK46')
			except:
			    print('unable to send')
		    Pmean= sum(Pmean_array)/len(Pmean_array)
#		    print('Pmean is')
#		    print(Pmean)
		    print('P_plat is')
		    print(int(peak_insp_pressure))
		    print(P_plat)
                    RR.append(RR_time)
#		    print('volume is')
		    peak_insp_pressure = int(peak_insp_pressure)
		    if(VTi_volume > VTi_max and peak_insp_pressure > P_plat and ABP_flag == 0 and SDP_flag == 0 and patient_status == 1 and pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure - 1
			P_plat = peak_insp_pressure
		    elif((VTi_volume < VTi_max or peak_insp_pressure < P_plat) and ABP_flag == 0 and SDP_flag == 0 and peak_insp_pressure <= PIP and patient_status == 1 and pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure + 1
			P_plat = peak_insp_pressure
			print('+1')
#		    print(volume*0.4)
#		    print('Vpeak Inspiratory')
#		    print(volume_peak_inhale)
		    MVi_array.append(VTi_volume)
#		    print('trigger is')
#		    print(trigger)
#		    print('Pmean is')
#		    print(Pmean)
		if(flag == 1):
		    patient_set = 0
		    if((volume_peak_inhale > 80 or peak_insp_pressure <= P_plat * 0.5)  and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		       # GPIO.output(22, GPIO.HIGH)
		    if((volume_peak_inhale <= 80 or peak_insp_pressure> P_plat * 0.5) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = 60/BPM
			if(BPM > 50):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				serw.write('ACK59')
			    except:
				print('unable to send')
 #                       print('BPM is')
  #                      print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(int(peak_insp_pressure)) + "," + str(int(VTi_volume)) + "," + str(int(volume_peak_inhale)) + "," + str(int(Pmean)) + "," + str(int(MVi)/1000) + '#')
#		        print('packet length is')
		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    volume = 0
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    while(time_elap < 0.5):
			w=time()
			time_elap=w-q
                        indiff = self.ABP_pressure()
			P_plat = int(P_plat)
			if(P_plat <= 0):
			    P_plat = 1
			ratio = peep_val/P_plat
                        if(ratio > 0.8 and time_elap >= 0.005):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.12 and P_plat < 28):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.2 and P_plat >=28):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.001):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.3):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 500 or diff == -500 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
#			print((packet_exhalation))
			volume_peak_exhale= max(volume_peak_exhale, volFlow_rate)
			print('packet_exhal;ation conatins is')
			print(packet_exhalation)
                    #motor_2.ChangeDutyCycle(0)
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    start_time=current_time
		    sending_time=200
		    send_last_time = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time - 0.5) and diff <= 0) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
			indiff = self.ABP_pressure()
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
			volume_peak_exhale=max(volume_peak_exhale,volFlow_rate)
			packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
                    flag=0
		    exhale_time_last = time()
		    loop = 1
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    print('Ti/Tot')
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    print('peak volume flow rate exhale')
#		    print(volume_peak_exhale)
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    MVe_array.append(volume)
		    FiO2 = self.FiO2()
		    print('FiO2 is')
		    print(FiO2)
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(int(peep_val_send)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(int(volume_peak_exhale)) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
		    try:
		        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   		            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		            now = datetime.datetime.now()
		            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
		    except:
			print('data file open error')
	except KeyboardInterrupt:
	    return


    #------------------------------------------------
    def PSV(self):
        global loop,patient_set,TITOT,exhale_time_last, TOT_last,patient_status,pressure_count,peep_count, ABP_flag, SDP_flag, pump_pressure_high, pump_pressure_low, pump_pressure,flow_plat,ratio_set,time_error_set,thread_mode_status,ABP_flag,time_error, time_error_last,P_plat_value,prev_mode,data,ratio,RR_time,indiff, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	pressure_count = 0
	peep_count = 0
	exhale_time_last = time()
	TOT_last = time()
	ratio_set = 0
	loop = 1
	patient_set = 0
	peep_set = 0
	time_error_set = 0
	flow_plat = 50
	GPIO.output(35, GPIO.HIGH)
        start_time=time() - 5000
	time_error_last = time()
	sending_time = 0.2
        try:
            while True:
		prev_mode = 13
		ABP_flag = 0
		print('we are in PSV mode')
		data= self.read_data()
		print('mode is')
		print(data)
		if(data == 1):
		    break;
		print(inhale_time)
		print(BPM)
		print(PIP)
		print(VTi_max)
		print(peep)
		print(P_plat)
		print(trigflow)
		print(pump_pressure)
	#	GPIO.output(22, GPIO.LOW)
		volFlow_rate = self.rate()
		indiff= self.ABP_pressure()
		if(indiff == -8888):
			ABP_flag = 1
			indiff = 0
		packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(0)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.07):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print('pump_pressure is')
#		print(pump_pressure)
                flow=self.rate()
 #               flow=self.rate()
#		flow = flow - peep_hole[int(peep_val)]
                current_time=time()
                volume=0
		TOT_last = current_time - exhale_time_last
		peep_val_send = indiff
		SDP_flag = 0
		flow_flag_unset = 0
		loop = 1
		time_error = time() - time_error_last
                if(flow>trigflow or (current_time-start_time)>cycle_time):
		    RR_time=current_time-start_time
		    pump_pressure_now = pump_pressure
		    flow_flag = 0
# 		    if((pump_pressure /2) < peep):
# 			motor_1.ChangeDutyCycle(peep + 2)
# 			pump_pressure_now = peep +2
# 		    else:
#                    motor_1.ChangeDutyCycle(pump_pressure)
                    motor_1.ChangeDutyCycle(100)
		    if(P_plat <= 22):
			motor_1.ChangeDutyCycle(80)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
# 			pump_pressure_now = pump_pressure /2
                    #RR_time=current_time-start_time
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(time_error > 0.7):
			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
		    if(RR_time > cycle_time):
			trigger='0'
		    else:
			trigger='1'
                 #   indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
		    now = time()
		    GPIO.output(37, GPIO.HIGH)
		    time_elapsed = 0
		    volume_peak_inhale=0
		    flow_flag = 0
		    volFlow_rate = self.rate()
		    pump_flag = 0
#		    pump_pressure_now = pump_pressure /2
#		    while(time_elapsed < 0.3):
#			time_elapsed = time() - now
#			indiff = self.ABP_pressure()
#			if(indiff >= (P_plat*1.15)):
#			    ser.write('ACK07')
#			if(SDP_flag == 0):
#			    volFlow_rate = self.rate()
#			    volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
#			if(indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.3): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
#			    flow_flag = 1
#			    print('CMHO reacged')
#			    print(indiff)
#			if(flow_flag == 1 and volFlow_rate <= flow_plat):
#			    motor_1.ChangeDutyCycle(pump_pressure)
#			if(volFlow_rate >= flow_plat and flow_flag ==1):
#			    flow_flag = 2
#			if(flow_flag == 2):
#			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
#				pump_pressure_now = pump_pressure_now + 1
#			        motor_1.ChangeDutyCycle(pump_pressure_now)
#			    if(volFlow_rate >= flow_plat and pump_pressure_now >= 2): #and pump_pressure_now >( pump_pressure /2)):
#			        pump_pressure_now = pump_pressure_now - 1
#			        motor_1.ChangeDutyCycle(pump_pressure_now)
#			packet_inhalation =('A@' + str(int(indiff)) + ','+str(int(volFlow_rate)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
#			if(sending_time >0.7):
#			    try:
#			        ser.write(packet_inhalation)
#				sending_time = 0.4
#			    except:
#			        print('BT error send Inhalation')
		    time_elapsed  = 0
		    Pmean = 0
		    del Pmean_array[:]
		    Pmean_array.append(1)
		    volFlow_rate = self.rate()
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    sending_time = 0.2
		    send_last_time=0
		    VTi_volume=0
		    peak_insp_pressure = 0
		    flow_flag_100 = 0
		    pump_pressure_now = pump_pressure /2
		    volFlow_rate_previous = volFlow_rate
		    peak_flow = volFlow_rate
                    while((peak_flow*0.25 <= volFlow_rate   and time_elapsed_inhale <= inhale_time) or time_elapsed_inhale <= 0.5):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
                        t3 = time()
			GPIO.output(37, GPIO.HIGH)
			if(ABP_flag == 0):
                            indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
                   #     print(indiff)
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volFlow_rate=self.rate()
			    if(volFlow_rate >= peak_flow):
				peak_flow = volFlow_rate
			if(volume == -9999):
			    SDP_flag = 1;
			if(sending_time > 0.07):
			    packet_inhalation =('A@' + str(int(indiff)) + ','+str(int(volFlow_rate)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
			    print(packet_inhalation)
			    send_last_time= time()
			sending_time=t3 - send_last_time
 			if(indiff >= (P_plat*0.75) and flow_flag == 0 and time_elapsed_inhale >= 0.4): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    print('CMHO reacged')
			    print(indiff)
			if(P_plat <= 17 and indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.2): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			if(flow_flag == 1 and volFlow_rate <= flow_plat):
			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(flow_flag == 2):
			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
				pump_pressure_now = pump_pressure_now + 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >=  P_plat/2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
# 			if(pump_pressure_now <= pump_pressure and volFlow_rate <= flow_plat):
# 			    if(pump_flag == 0):
# 			        pump_pressure_now  = pump_pressure_now + 1
# 			    else:
# 				pump_pressure_now = pump_pressure_now + 0.5
# 	#		    print('the pump_pressure now is')
# 	#		    print(pump_pressure_now)
# 	#		    print('pump pressure is')
# 	#		    print(pump_pressure)
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			elif(volFlow_rate > flow_plat and pump_pressure_now >( pump_pressure /2)):
# 			    pump_pressure_now = pump_pressure_now - 0.5
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			    pump_flag = 1
#			print(time_elapsed_inhale)
#			print(inhale_time)
                        #    GPIO.output(22, GPIO.LOW)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(indiff >= (P_plat+3)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
                    flag=1
		    loop = 0
		    motor_1.ChangeDutyCycle(0)
		    sleep(0.01)
		    GPIO.output(35, GPIO.LOW) # low level triggered relay
		    Pmean= sum(Pmean_array)/len(Pmean_array)
#		    print('Pmean is')
#		    print(Pmean)
		    print('P_plat is')
		    print(int(peak_insp_pressure))
		    print(P_plat)
                    RR.append(RR_time)
		    peak_insp_pressure = int(peak_insp_pressure)
		    if(peak_insp_pressure > P_plat and patient_status == 1 and ABP_flag == 0 and peak_insp_pressure < PIP and pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure - 1
			print('-1')
		    elif(peak_insp_pressure < P_plat and patient_status == 1 and ABP_flag == 0 and peak_insp_pressure < PIP and  pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure + 1
			print('+1')
#		    print('volume is')
#		    print(volume*0.4)
#		    print('Vpeak Inspiratory')
#		    print(volume_peak_inhale)
		    MVi_array.append(VTi_volume)
#		    print('trigger is')
#		    print(trigger)
#		    print('Pmean is')
#		    print(Pmean)
		    GPIO.output(37, GPIO.LOW)
		if(flag == 1):
		    patient_set = 0
		    if((volume_peak_inhale > 80 or peak_insp_pressure <= P_plat * 0.5)  and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		       # GPIO.output(22, GPIO.HIGH)
		    if((volume_peak_inhale <= 80 or peak_insp_pressure> P_plat * 0.5) and patient_set == 0):
			patient_set = 1
			patient_status = 1
			buzzer(5,0)
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = 60/BPM
			if(BPM > 50):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				serw.write('ACK59')
			    except:
				print('unable to send')
 #                      print('BPM is')
  #                     print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(int(peak_insp_pressure)) + "," + str(int(VTi_volume)) + "," + str(int(volume_peak_inhale)) + "," + str(int(Pmean)) + "," + str(int(MVi)/1000) + '#')
#		        print('packet length is')
		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    volume = 0
		    buzzer(7,0)
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    while(time_elap < 0.5):
			w=time()
			time_elap=w-q
			ratio = peep_val/P_plat
                        indiff = self.ABP_pressure()
                        if(ratio > 0.8 and time_elap >= 0.005):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.12 and P_plat < 28):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.2 and P_plat >=28):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.001):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.3):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
			diff = self.SDP_pressure()
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 500 or diff == -500 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
#			print((packet_exhalation))
			volume_peak_exhale= max(volume_peak_exhale, volFlow_rate)
			print('packet_exhal;ation conatins is')
			print(packet_exhalation)
                    #motor_2.ChangeDutyCycle(0)
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    start_time=current_time
		    sending_time=200
		    send_last_time = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time-0.5) and diff <= 0) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
			volume_peak_exhale=max(volume_peak_exhale,volFlow_rate)
			packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
                    flag=0
		    exhale_time_last = time()
		    loop = 1
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    print('Ti/Tot')
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    print('peak volume flow rate exhale')
#		    print(volume_peak_exhale)
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    MVe_array.append(volume)
		    FiO2 = self.FiO2()
		    print('FiO2 is')
		    print(FiO2)
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(int(peep_val_send)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(int(volume_peak_exhale)) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
		    try:
		        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   		            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		            now = datetime.datetime.now()
		            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
		    except:
			print('data file open error')
        except KeyboardInterrupt:
            return
    def NIV_CPAP(self):
        global loop,patient_set,exhale_time_last, TOT_last,flow_plat,TITOT,pressure_count,pump_pressure_low,ABP_flag, SDP_flag,pump_pressure,pump_pressure_high, patient_status,ratio_set,time_error_set,thread_mode_status,time_error, time_error_last,ABP_flag,P_plat_value,prev_mode,data,ratio,RR_time,indiff,peep_val, peep_hole, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	pressure_count = 0
	patient_set = 0
	exhale_time_last = time()
	TOT_last = time()
	ratio_set = 0
	loop = 1
	peep_set = 0
	inhale_time = 2
	BPM = 10
	GPIO.output(35, GPIO.HIGH)
	VTi_max = 500
	time_error_set = 0
        start_time=time() - 5000
	time_error_last = time()
	sending_time = 0.2
        try:
            while True:
		prev_mode = 31
		ABP_flag = 0
		print('we are in NIV_CPAP mode')
		data= self.read_data()
#		print('mode is')
#		print(data)
		if(data == 1):
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		print(VTi_max)
		inhale_time = 2.0
		BPM = 10
		VTi_max = 500
		PIP = P_plat +5
		cycle_time = BPM/60
		trig_flow = 5
#		print(peep)
		print(peep_val)
		print(P_plat)
		print(P_plat_value)
		print('here the pump pressure is')
		print(pump_pressure)
	#	GPIO.output(22, GPIO.LOW)
		volFlow_rate = self.rate()
		indiff= self.ABP_pressure()
		if(indiff == -8888):
			ABP_flag = 1
			indiff = 0
		packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(0)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.07):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print('pump_pressure is')
#		print(pump_pressure)
                flow=self.rate()
#		print('before flow')
		print(flow)
#		flow = flow - peep_hole[int(peep_val)]
#		print('after flow')
#		print(flow)
                current_time=time()
                volume=0
		TOT_last = current_time - exhale_time_last
		SDP_flag = 0
		peep_val_send = indiff
		loop = 1
		time_error = time() - time_error_last
                if(flow>trigflow or (current_time-start_time)>cycle_time):
		    pump_pressure_now = pump_pressure
		    indiff = self.ABP_pressure()
		    flow_flag = 0
#		    if((pump_pressure /2) < peep):
#			motor_1.ChangeDutyCycle(peep + 2)
#			pump_pressure_now = peep +2
#		    else:
#		    print('indiff right now is')
#		    print(indiff)
#		    if(indiff <= P_plat/2):
                    motor_1.ChangeDutyCycle(pump_pressure)
		    #    flow_flag = 0
#		    else:
#			motor_1.ChangeDutyCycle(pump_pressure)
#			flow_flag = 1
#			pump_pressure_now = pump_pressure /2
                    RR_time=current_time-start_time
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(time_error > 0.7):
			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
#		    if(RR_time > cycle_time):
#			trigger='0'
#		    elif(flow>trigflow):
#			trigger='1'
                 #   indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    volFlow_rate = self.rate()
		    VTi_volume=0
		    pump_flag = 0
		   # flow_flag = 0
		    current_time = time()
		    peak_insp_pressure = 0
#		    flow_zero_intialize = time() # clock starts to detect zero flow
#		    flow_zero_flag = 0
#		    flow_zero_elapsed = 0
                    while(indiff <= PIP and time_elapsed_inhale <= inhale_time):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
                        t3 = time()
			GPIO.output(37, GPIO.HIGH)
#			if(volFlow_rate <= 2):
#			    flow_zero_flag = 1
#			    flow_zero_initialize = time()
#		        if(flow_zero_flag == 1):
#			    flow_zero_elapsed = time() - flow_zero_initialize
#			    if(flow_zero_elapsed >= )
#			    flow_zero_elapsed = time() - flow_zero_initialize 
			if(ABP_flag == 0):
                            indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
                   #     print(indiff)
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volFlow_rate=self.rate()
#			if(indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.3): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
#			    flow_flag = 1
#			    print('CMHO reacged')
#			    print(indiff)
#			if(flow_flag == 1 and volFlow_rate <= flow_plat):
#			    motor_1.ChangeDutyCycle(pump_pressure)
#			if(volFlow_rate >= flow_plat and flow_flag ==1):
#			    flow_flag = 2
			if(volume == -9999):
			    SDP_flag = 1;
			if(sending_time > 0.07):
			    packet_inhalation =('A@' + str(int(indiff)) + ','+str(int(volFlow_rate)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
#			    print(packet_inhalation)
			    send_last_time= time()
#			if(volFlow_rate < flow_plat -3 and pump_pressure_now < pump_pressure):
#			    pump_pressure_now = pump_pressure_now + 4
#			    print('in 1st loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
#			elif(volFlow_rate > flow_plat + 3 and pump_pressure_now > pump_pressure):
#			    pump_pressure_now = pump_pressure_now -4
#			    print('in 2nd loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
#			if(flow_flag == 2):
#			    if(volFlow_rate <= flow_plat -5 and pump_pressure_now <= P_plat):
		#	   # if(pump_flag == 0):
			      #      pump_pressure_now  = pump_pressure_now + 5
			     #   else:
#				pump_pressure_now = pump_pressure_now + 1
	#		    print('the pump_pressure now is')
	#		    print(pump_pressure_now)
	#		    print('pump pressure is')
	#		    print(pump_pressure)
#			        motor_1.ChangeDutyCycle(pump_pressure_now)
#			    if(volFlow_rate >= flow_plat and pump_pressure_now >= P_plat/2): #and pump_pressure_now >( pump_pressure /2)):
#			        pump_pressure_now = pump_pressure_now - 1
#			        motor_1.ChangeDutyCycle(pump_pressure_now)
			      #  pump_flag = 1
			sending_time=t3 - send_last_time
#			print('flow is')
			print(volFlow_rate)
#			print(time_elapsed_inhale)
#			print(inhale_time)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(indiff >= (P_plat+3)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
                    flag=1
		    loop = 0
#		    print('volume is')
#		    print(VTi_volume)
		    motor_1.ChangeDutyCycle(pump_pressure)
	#	    sleep(0.01)
#		    GPIO.output(35, GPIO.HIGH) # low level triggered relay
		    Pmean= sum(Pmean_array)/len(Pmean_array)
#		    print('Pmean is')
		    print('P_plat is')
		    print(int(peak_insp_pressure))
		    print(P_plat)
#		    print(Pmean)
                    RR.append(RR_time)
#		    print('volume is')
		    peak_insp_pressure = int(peak_insp_pressure)
		    if(peak_insp_pressure > P_plat and patient_status == 1 and ABP_flag == 0 and peak_insp_pressure < PIP and  pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure - 1
			print('-1')
		    elif(peak_insp_pressure < P_plat and patient_status == 1 and ABP_flag == 0 and peak_insp_pressure < PIP and  pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure + 1
			print('+1')
#		    print(volume*0.4)
#		    print('Vpeak Inspiratory')
#		    print(volume_peak_inhale)
		    MVi_array.append(VTi_volume)
#		    print('trigger is')
#		    print(trigger)
#		    print('Pmean is')
#		    print(Pmean)
		    GPIO.output(37, GPIO.LOW)
		if(flag == 1):
		    patient_set = 0
		    if((volume_peak_inhale > 80 or peak_insp_pressure <= P_plat * 0.5)  and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		  #      GPIO.output(22, GPIO.HIGH)
		    if((volume_peak_inhale <= 80 or peak_insp_pressure> P_plat * 0.5) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = 60/BPM
			if(BPM > 50):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				serw.write('ACK59')
			    except:
				print('unable to send')
 #                      print('BPM is')
  #                     print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(int(peak_insp_pressure)) + "," + str(int(VTi_volume)) + "," + str(int(volume_peak_inhale)) + "," + str(int(Pmean)) + "," + str(int(MVi)/1000) + '#')
#		        print('packet length is')
		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    volume = 0
		    buzzer(7,0)
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    while(time_elap < 0.5):
			w=time()
			time_elap=w-q
#			if(time_elap > 0.2): # for motor relay
#			    print('turning off the relay')
#		    	    GPIO.output(35, GPIO.LOW)
#			    sleep(0.05)
#			    motor_1.ChangeDutyCycle(peep)
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
# 			if(indiff <= (peep + 1)):
# 			    GPIO.output(35, GPIO.LOW)
# 			    print('turning off the relay')
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 500 or diff == -500 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
			print(volFlow_rate)
			volume_peak_exhale= max(volume_peak_exhale, volFlow_rate)
#			print('packet_exhal;ation conatins is')
#			print(packet_exhalation)
                    #motor_2.ChangeDutyCycle(0)
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    start_time=current_time
		    sending_time=200
		    send_last_time = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time-0.5) and diff <= 0) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
# 			if(indiff <= (peep + 1)):
# 			    GPIO.output(35, GPIO.LOW)
# 			    print('turning off the relay')
			volume_peak_exhale=max(volume_peak_exhale,volFlow_rate)
			packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
                    flag=0
		    loop = 1
		    exhale_time_last = time()
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
		    print(volFlow_rate)
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    print('peak volume flow rate exhale')
#		    print(volume_peak_exhale)
		    if(peep_val_send >= 15):
			try:
			    ser.write('ACK08')
			except:
			    print('unable to send')
		    else:
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    MVe_array.append(volume)
		    FiO2 = self.FiO2()
		    print('FiO2 is')
		    print(FiO2)
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(int(peep_val_send)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(int(volume_peak_exhale)) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
		    try:
		        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   		            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		            now = datetime.datetime.now()
		            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
		    except:
			print('data file open error')
        except KeyboardInterrupt:
            return
#-----------------------------------------------------------
    def NIV_BIPAP(self):
        global loop,patient_set,ABP_flag,exhale_time_last,TOT_last,TITOT,pressure_count,peep_count, SDP_flag,flow_plat,indiff,patient_status,pump_pressure,pump_pressure_low,pump_pressure_high,ratio_set,time_error_set,thread_mode_status,time_error, time_error_last,ABP_flag,P_plat_value,prev_mode,data,ratio,RR_time,indiff,peep_val, peep_hole, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	patient_set = 0
	exhale_time_last = time()
	TOT_last = time()
	peep_count = 0
	loop = 1
	pressure_count = 0
	peep_set = 0
	ratio_set = 0
	time_error_set = 0
	GPIO.output(35, GPIO.HIGH)
        start_time=time() - 5000
	time_error_last = time()
	sending_time = 0.2
        try:
            while True:
		prev_mode = 32
		ABP_flag = 0
#		print('we are in PC_IMV mode')
		data= self.read_data()
#		print('mode is')
#		print(data)
		if(data == 1):
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
		PIP = P_plat +1
		trigflow = 30
#		print(VTi_max)
#		print('peep in main loop is')
#		print(peep)
#		print(peep_val)
#		print(P_plat)
#		print(P_plat_value)
#		print('here the pump pressure is')
#		print(pump_pressure)
	#	GPIO.output(22, GPIO.LOW)
		volFlow_rate = self.rate()
		indiff= self.ABP_pressure()
		if(indiff == -8888):
			ABP_flag = 1
			indiff = 0
		packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(0)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.07):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print('pump_pressure is')
#		print(pump_pressure)
                flow=self.rate()
#		print('before flow')
#		print(flow)
	#	flow = flow - peep_hole[int(peep_val)]
#		print('after flow')
#		print(flow)
                current_time=time()
                volume=0
		TOT_last = current_time - exhale_time_last
		peep_val_send = indiff
		SDP_flag = 0
		time_error = time() - time_error_last
		loop = 1
                if(flow>trigflow or (current_time-start_time)>cycle_time):
		    RR_time=current_time-start_time
		    pump_pressure_now = pump_pressure
		    indiff = self.ABP_pressure()
		    flow_flag = 0
#		    if((pump_pressure /2) < peep):
#			motor_1.ChangeDutyCycle(peep + 2)
#			pump_pressure_now = peep +2
#		    else:
#		    print('indiff right now is')
#		    print(indiff)
#		    if(indiff <= P_plat/2):
                    motor_1.ChangeDutyCycle(pump_pressure)
		    if(P_plat <= 22):
			motor_1.ChangeDutyCycle(pump_pressure)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
#		        flow_flag = 0
#		    else:
#			motor_1.ChangeDutyCycle(pump_pressure)
#			flow_flag = 1
#			pump_pressure_now = pump_pressure /2
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
#		    if(time_error > 0.7):
#			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
		    if(RR_time > cycle_time):
			trigger='0'
		    else:
			trigger='1'
                 #   indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    volFlow_rate = self.rate()
		    VTi_volume=0
		    pump_flag = 0
		   # flow_flag = 0
		    current_time = time()
		    peak_insp_pressure = 0
		    pump_pressure_now = pump_pressure /2
#		    flow_zero_intialize = time() # clock starts to detect zero flow
#		    flow_zero_flag = 0
#		    flow_zero_elapsed = 0
                    while(indiff <= PIP and time_elapsed_inhale <= inhale_time):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
                        t3 = time()
			GPIO.output(37, GPIO.HIGH)
#			if(volFlow_rate <= 2):
#			    flow_zero_flag = 1
#			    flow_zero_initialize = time()
#		        if(flow_zero_flag == 1):
#			    flow_zero_elapsed = time() - flow_zero_initialize
#			    if(flow_zero_elapsed >= )
#			    flow_zero_elapsed = time() - flow_zero_initialize 
			if(ABP_flag == 0):
                            indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
                   #     print(indiff)
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volFlow_rate=self.rate()
			if(indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.4): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
#			    print('CMHO reacged')
#			    print(indiff)
			if(P_plat <= 17 and indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.2): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			if(flow_flag == 1 and volFlow_rate <= flow_plat):
			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==7):
			    flow_flag = 2
			if(volume == -9999):
			    SDP_flag = 1;
			if(sending_time > 0.07):
			    packet_inhalation =('A@' + str(int(indiff)) + ','+str(int(volFlow_rate)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
#			    print(packet_inhalation)
			    send_last_time= time()
#			if(volFlow_rate < flow_plat -3 and pump_pressure_now < pump_pressure):
#			    pump_pressure_now = pump_pressure_now + 4
#			    print('in 1st loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
#			elif(volFlow_rate > flow_plat + 3 and pump_pressure_now > pump_pressure):
#			    pump_pressure_now = pump_pressure_now -4
#			    print('in 2nd loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
			if(flow_flag == 2):
			    if(volFlow_rate <= flow_plat -5 and pump_pressure_now <= P_plat):
		#	   # if(pump_flag == 0):
			      #      pump_pressure_now  = pump_pressure_now + 5
			     #   else:
				pump_pressure_now = pump_pressure_now + 1
	#		    print('the pump_pressure now is')
	#		    print(pump_pressure_now)
	#		    print('pump pressure is')
	#		    print(pump_pressure)
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= P_plat/2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			      #  pump_flag = 1
			sending_time=t3 - send_last_time
#			print('flow is')
#			print(volFlow_rate)
#			print(time_elapsed_inhale)
#			print(inhale_time)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(indiff >= (P_plat+3)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
                    flag=1
		    loop = 0
#		    print('volume is')
#		    print(VTi_volume)
		    motor_1.ChangeDutyCycle(0)
	#	    sleep(0.01)
		    GPIO.output(35, GPIO.LOW) # HIGH level triggered relay
		    Pmean= sum(Pmean_array)/len(Pmean_array)
		    print('P_plat is')
		    print(int(peak_insp_pressure))
		    print(P_plat)
                    RR.append(RR_time)
		    peak_insp_pressure = int(peak_insp_pressure)
		    if(peak_insp_pressure > P_plat and patient_status == 1 and ABP_flag == 0 and peak_insp_pressure < PIP and pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure - 1
			print('-1')
		    elif(peak_insp_pressure < P_plat and patient_status == 1 and ABP_flag == 0 and peak_insp_pressure < PIP and  pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure + 1
			print('+1')
#		    print('volume is')
#		    print(volume*0.4)
#		    print('Vpeak Inspiratory')
#		    print(volume_peak_inhale)
		    MVi_array.append(VTi_volume)
#		    print('trigger is')
#		    print(trigger)
#		    print('Pmean is')
#		    print(Pmean)
		    GPIO.output(37, GPIO.LOW)
		if(flag == 1):
		    patient_set = 0
		    if((volume_peak_inhale > 100 or peak_insp_pressure <= P_plat * 0.5)  and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		#        GPIO.output(22, GPIO.HIGH)
		    if((volume_peak_inhale <= 100 or peak_insp_pressure> P_plat * 0.5) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = 60/BPM
			if(BPM > 50):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				serw.write('ACK59')
			    except:
				print('unable to send')
 #                      print('BPM is')
  #                     print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(int(peak_insp_pressure)) + "," + str(int(VTi_volume)) + "," + str(int(volume_peak_inhale)) + "," + str(int(Pmean)) + "," + str(int(MVi)/1000) + '#')
#		        print('packet length is')
#		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    buzzer(7,0)
		    volume = 0
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    while(time_elap < 0.5):
			w=time()
			time_elap=w-q
                        indiff = self.ABP_pressure()
			ratio = peep_val/P_plat
                        if(ratio > 0.8 and time_elap >= 0.005):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.12 and P_plat < 28):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.2 and P_plat >=28):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.001):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.3):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
# 			if(indiff <= (peep + 1)):
# 			    print('turning off the relay')
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 500 or diff == -500 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
#			print(volFlow_rate)
			volume_peak_exhale= max(volume_peak_exhale, volFlow_rate)
#			print('packet_exhal;ation conatins is')
#			print(packet_exhalation)
                    #motor_2.ChangeDutyCycle(0)
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    start_time=current_time
		    sending_time=200
		    send_last_time = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time-0.5) and diff <= 0) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
# 			if(indiff <= (peep + 1)):
# 			    GPIO.output(35, GPIO.LOW)
# 			    print('turning off the relay')
			volume_peak_exhale=max(volume_peak_exhale,volFlow_rate)
			packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
                    flag=0 
		    loop = 1
		    exhale_time_last = time()
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    print(volFlow_rate)
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    print('peak volume flow rate exhale')
#		    print(volume_peak_exhale)
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    FiO2 = self.FiO2()
		    print('FiO2 is')
		    print(FiO2)
		    MVe_array.append(volume)
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(int(peep_val_send)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(int(volume_peak_exhale)) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
#			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
		    try:
		        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   		            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		            now = datetime.datetime.now()
		            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
		    except:
			print('data file open error')
        except KeyboardInterrupt:
            return

#--------------------------------------------------------------------
    def PRVC(self):
        global loop,patient_set,flow_plat,exhale_time_last, TOT_last,TITOT,indiff,volume_count,peep_count,pressure_count,ABP_flag, SDP_flag,patient_status,pump_pressure,pump_pressure_low,pump_pressure_high,ratio_set,time_error_set,thread_mode_status,time_error, time_error_last,ABP_flag,P_plat_value,prev_mode,data,ratio,RR_time,indiff,peep_val, peep_hole, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	patient_set = 0
	volume_count = 0
	peep_count = 0
	peep_set = 0
	exhale_time_last = time()
	TOT_last = time()
	pressure_count = 0
	loop = 1
	volume_set = 0
	ratio_set = 0
	time_error_set = 0
	GPIO.output(35, GPIO.HIGH)
        start_time=time() - 5000
	time_error_last = time()
	sending_time = 0.2
        try:
            while True:
		prev_mode = 23
		ABP_flag = 0
#		print('we are in PC_IMV mode')
		data= self.read_data()
#		print('mode is')
#		print(data)
		if(data == 1):
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		print(VTi_max)
#		print('peep in main loop is')
#		print(peep)
#		print(peep_val)
#		print(P_plat)
#		print(P_plat_value)
#		print('here the pump pressure is')
#		print(pump_pressure)
	#	GPIO.output(22, GPIO.LOW)
		volFlow_rate = self.rate()
		indiff= self.ABP_pressure()
		if(indiff == -8888):
			ABP_flag = 1
			indiff = 0
		packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(0)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.07):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print('pump_pressure is')
#		print(pump_pressure)
                flow=self.rate()
#		print('before flow')
#		print(flow)
	#	flow = flow - peep_hole[int(peep_val)]
#		print('after flow')
#		print(flow)
                current_time=time()
                volume=0
		TOT_last = current_time - exhale_time_last
		peep_val_send = indiff
		SDP_flag = 0
		loop = 1
		time_error = time() - time_error_last
                if(flow>trigflow or (current_time-start_time)>cycle_time):
		    RR_time=current_time-start_time
		    pump_pressure_now = pump_pressure
		    indiff = self.ABP_pressure()
		    flow_flag = 0
#		    if((pump_pressure /2) < peep):
#			motor_1.ChangeDutyCycle(peep + 2)
#			pump_pressure_now = peep +2
#		    else:
#		    print('indiff right now is')
#		    print(indiff)
#		    if(indiff <= P_plat/2):
                    motor_1.ChangeDutyCycle(100)
		    if(P_plat <= 22):
			motor_1.ChangeDutyCycle(80)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
#		        flow_flag = 0
#		    else:
#			motor_1.ChangeDutyCycle(pump_pressure)
#			flow_flag = 1
#			pump_pressure_now = pump_pressure /2
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
#		    if(time_error > 0.7):
#			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
		    if(RR_time > cycle_time):
			trigger='0'
		    else:
			trigger='1'
                 #   indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    volFlow_rate = self.rate()
		    VTi_volume=0
		    pump_flag = 0
		   # flow_flag = 0
		    current_time = time()
		    peak_insp_pressure = 0
		    pump_pressure_now = pump_pressure /2
#		    flow_zero_intialize = time() # clock starts to detect zero flow
#		    flow_zero_flag = 0
#		    flow_zero_elapsed = 0
                    while(indiff <= PIP and time_elapsed_inhale <= inhale_time and (VTi_volume <= VTi_max or trigger == '1')):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
                        t3 = time()
			GPIO.output(37, GPIO.HIGH)
#			if(volFlow_rate <= 2):
#			    flow_zero_flag = 1
#			    flow_zero_initialize = time()
#		        if(flow_zero_flag == 1):
#			    flow_zero_elapsed = time() - flow_zero_initialize
#			    if(flow_zero_elapsed >= )
#			    flow_zero_elapsed = time() - flow_zero_initialize 
			if(ABP_flag == 0):
                            indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
                   #     print(indiff)
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volFlow_rate=self.rate()
			if(indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.4): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
#			    print('CMHO reacged')
#			    print(indiff)
			if(P_plat <= 17 and indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.2): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			if(flow_flag == 1 and volFlow_rate <= flow_plat):
			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(volume == -9999):
			    SDP_flag = 1;
			if(sending_time > 0.07):
			    packet_inhalation =('A@' + str(int(indiff)) + ','+str(int(volFlow_rate)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
#			    print(packet_inhalation)
			    send_last_time= time()
#			if(volFlow_rate < flow_plat -3 and pump_pressure_now < pump_pressure):
#			    pump_pressure_now = pump_pressure_now + 4
#			    print('in 1st loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
#			elif(volFlow_rate > flow_plat + 3 and pump_pressure_now > pump_pressure):
#			    pump_pressure_now = pump_pressure_now -4
#			    print('in 2nd loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
			if(flow_flag == 2):
			    if(volFlow_rate <= flow_plat -5 and pump_pressure_now <= P_plat):
		#	   # if(pump_flag == 0):
			      #      pump_pressure_now  = pump_pressure_now + 5
			     #   else:
				pump_pressure_now = pump_pressure_now + 1
	#		    print('the pump_pressure now is')
	#		    print(pump_pressure_now)
	#		    print('pump pressure is')
	#		    print(pump_pressure)
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= P_plat/2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			      #  pump_flag = 1
			sending_time=t3 - send_last_time
#			print('flow is')
#			print(volFlow_rate)
#			print(time_elapsed_inhale)
#			print(inhale_time)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(indiff >= (PIP + 1)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
                    flag=1
	 	    loop = 0
#		    print('volume is')
#		    print(VTi_volume)
		    motor_1.ChangeDutyCycle(0)
	#	    sleep(0.01)
		    GPIO.output(35, GPIO.LOW) # HIGH level triggered relay
		    if(VTi_volume < (VTi_max * 0.8)):
			volume_count = volume_count + 1
			if(volume_count >= 3):
			    buzzer(7,1)
			    try:
			        ser.write('ACK36')
			    except:
			        print('unable to send')
		    else:
			buzzer(7,0)
			volume_count =0
			try:
			    ser.write('ACK46')
			except:
			    print('unable to send')
		    Pmean= sum(Pmean_array)/len(Pmean_array)
		    print('P_plat is')
		    print(int(peak_insp_pressure))
		    print(P_plat)
                    RR.append(RR_time)
		    peak_insp_pressure = int(peak_insp_pressure)
		    if(peak_insp_pressure > PIP and patient_status == 1 and ABP_flag == 0 and SDP_flag == 0 and pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure - 1
			print('-1')
		    elif(VTi_volume > VTi_max and peak_insp_pressure > P_plat and ABP_flag == 0 and SDP_flag == 0 and patient_status == 1 and pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure - 1
		    elif((VTi_volume < VTi_max or peak_insp_pressure < P_plat) and ABP_flag == 0 and SDP_flag == 0 and peak_insp_pressure <= PIP and patient_status == 1 and pump_pressure >= pump_pressure_low and pump_pressure <= pump_pressure_high):
			pump_pressure = pump_pressure + 1
			print('+1')
#		    print('volume is')
#		    print(volume*0.4)
#		    print('Vpeak Inspiratory')
#		    print(volume_peak_inhale)
		    MVi_array.append(VTi_volume)
#		    print('trigger is')
#		    print(trigger)
#		    print('Pmean is')
#		    print(Pmean)
		    GPIO.output(37, GPIO.LOW)
		if(flag == 1):
		    patient_set = 0
		    if((volume_peak_inhale > 80 or peak_insp_pressure <= P_plat * 0.5)  and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		  #      GPIO.output(22, GPIO.HIGH)
		    if((volume_peak_inhale <= 80 or peak_insp_pressure> P_plat * 0.5) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = 60/BPM
			if(BPM > 50):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				serw.write('ACK59')
			    except:
				print('unable to send')
 #                      print('BPM is')
  #                     print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(int(peak_insp_pressure)) + "," + str(int(VTi_volume)) + "," + str(int(volume_peak_inhale)) + "," + str(int(Pmean)) + "," + str(int(MVi)/1000) + '#')
#		        print('packet length is')
#		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    volume = 0
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    while(time_elap < 0.5):
			w=time()
			time_elap=w-q
                        indiff = self.ABP_pressure()
			ratio = peep_val/P_plat
                        if(ratio > 0.8 and time_elap >= 0.005):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.12 and P_plat < 28):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.2 and P_plat >=28):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.001):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.3):
                            GPIO.output(35, GPIO.HIGH)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
# 			if(indiff <= (peep + 1)):
# 			    print('turning off the relay')
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 500 or diff == -500 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
#			print(volFlow_rate)
			volume_peak_exhale= max(volume_peak_exhale, volFlow_rate)
#			print('packet_exhal;ation conatins is')
#			print(packet_exhalation)
                    #motor_2.ChangeDutyCycle(0)
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    start_time=current_time
		    sending_time=200
		    send_last_time = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time-0.5) and diff <= 0) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
# 			if(indiff <= (peep + 1)):
# 			    GPIO.output(35, GPIO.LOW)
# 			    print('turning off the relay')
			volume_peak_exhale=max(volume_peak_exhale,volFlow_rate)
			packet_exhalation = ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)*-1) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.07):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
                    flag=0
		    loop = 1
		    exhale_time_last = time()
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    print(volFlow_rate)
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    print('peak volume flow rate exhale')
#		    print(volume_peak_exhale)
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    FiO2 = self.FiO2()
		    print('FiO2 is')
		    print(FiO2)
		    MVe_array.append(volume)
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(int(peep_val_send)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(int(volume_peak_exhale)) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
#			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
		    try:
		        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   		            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		            now = datetime.datetime.now()
		            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
		    except:
			print('data file open error')
        except KeyboardInterrupt:
            return

            return
    # MODE SELECTION FUNCTION
    def mode_selection(self,mode):
        global thread_mode_status,pump_pressure
	P_plat_value=find_nearest(P_plat_array,P_plat)
	print('Plat is')
	print(P_plat_value)
#	print(len(pump_pressure_array))
#	print(len(P_plat_array))
	index = P_plat_array.index(P_plat_value)
	pump_pressure =pump_pressure_array[index]
	print(index)
	print(pump_pressure)
	try:
	    exists=os.path.isfile('/home/pi/AgVa_5.0/data.csv')
	except:
	    print('file read error')
	if(exists):
	    print('file exists')
	    try:
		with open('/home/pi/AgVa_5.0/data.csv',mode='a') as data:
		    data=csv.writer(data,delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		    data.writerow(['Initialization or change in Mode'])
	    except:
		print('file opening error')
	else:
	    try:
		with open('/home/pi/AgVa_5.0/data.csv', mode='w') as data:
	            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
	            data.writerow(['Time','Core Temperature','Pmean','volume','volume_peak_inhale','trigger','BPM','MVi','Ti/Tot','volume_peak_exhale','Mve'])
	    except:
		print('file opening error')
      #  while not thread_mode_status:

        #    thread_mode_status=False
        print("INSIDE MODE")
#	    while True : 
	#	for dc in range(10,101,10):
	#	    motor_1.ChangeDutyCycle(dc)
	#	    sleep(2)
	#	    print(self.rate())
#		motor_1.ChangeDutyCycle(air)
 #     	        print(self.rate())
#		sleep(0.1)
#		print('product')
#		print(self.rate()*clock)
#	    clock_t2 = time()
        if mode==22:
            self.VC_IMV()
        elif mode==12:
            self.PC_IMV()
        elif mode == 21:
            self.VC_CMV()
        elif mode == 11:
            self.PC_CMV()
	elif mode == 13:
	    self.PSV()
	elif mode == 31:
	    self.NIV_CPAP()
	elif mode == 32:
	    self.NIV_BIPAP()
	elif mode == 23:
	    self.PRVC()
    #-------------------------------------------------







#    def VC_CMV(self):

while(True):
    v = Ventilator()
    thread_mode_status = False
    try:
	f = open('/home/pi/AgVa_5.0/mode.txt',"r")
	data = f.readline()
	a = int(data)
    except:
	print('error reading mode file')
    v.mode_selection(a)
sleep(1)
print('end')
motor_1.stop()
#motor_2.stop()
GPIO.cleanup()
