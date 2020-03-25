#!/user/bin/env python
import time
import serial
import subprocess
import os
import time
import RPi.GPIO as GPIO
print('i am in here')
setting_flag = 0
counter_flag = 0
ping_delay = 0
ping_flag = 0
counter = 6
flag = 0
ping_time = 0
ping_delay = 0
#time.sleep(5)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(37, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(18,GPIO.OUT)
motor_1=GPIO.PWM(18,50)
ser = serial.Serial(
	port='/dev/ttyAMA0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=0.05
)
ping_set = 1
f = open('/home/pi/AgVa_5.0/MUTE.txt',"w")
f.write("0")
f.close()
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
while 1:
#	print('hello')
        time_now = time.time()
	try:
	    x=ser.readline()
	except:
	    print('returned nothing')
	if((x[0:3] == "IH" or x[0:3] == "EH"):
	    try:
	        f = open("home/pi/AgVa_5.0/hold.txt","w")
	        f.write(x)
		f.close()
	    except:
	   	print("unable to write to file")
	if(x == "BHOLD1"):
	    try:
	        f = open("home/pi/AgVa_5.0/BHOLD.txt","w")
	        f.write("1")
	        f.close()
	    except:
		print("unable to write")
	if(x == 'CM+RTC'):
     	    os.system('sudo modprobe rtc-ds1307')
	    os.system("sudo echo 'ds1307 0x68' | sudo tee /sys/class/i2c-adapter/i2c-1/new_device")
	    os.system('sudo hwclock -r')
	    os.system('sudo hwclock -w')
	    os.system('sudo hwclock -r')
	if(x == 'CM+KILLALL'):
	    os.system('sudo shutdown -h now')
	if(x == 'CMR+LIFE'):
            GPIO.output(37, GPIO.LOW)
	    subprocess.Popen("sudo python /home/pi/AgVa_5.0/life.py", shell = True)
            os.system('sudo pkill -f buzzer.py')
 #           os.system('sudo pkill -f recover.py')
            os.system('sudo pkill -f AgVa_relay_low.py')
            os.system('sudo pkill -f AgVa_relay.py')
            os.system('sudo pkill -f power.py')
            os.system('sudo pkill -f ABP_1.py')
            os.system('sudo pkill -f SDP.py')
            os.system('sudo pkill -f debug.py')
            motor_1.start(0)
	    GPIO.output(22, GPIO.LOW)
#            subprocess.Popen("sudo python /home/pi/AgVa_5.0/life.py", shell = True)
            while(1):
                time.sleep(0.5)
                GPIO.output(37, GPIO.HIGH)
                time.sleep(0.8)
                GPIO.output(37, GPIO.LOW)
	if(x == 'CM+LCOMP1'):
	    try:
		f = open("/home/pi/AgVa_5.0/flowComp.txt","w")
		f.write("1")
		f.close()
	    except:
		print("unable to write CM+LCOMP1")
	if(x == 'CM+LCOMP0'):
	    try:
		f = open("/home/pi/AgVa_5.0/flowComp.txt","w")
		f.write("0")
		f.close()
	    except:
		print("unable to write CM+LCOMP0")
	if(x == 'CM+CALIBRATE'):
	    GPIO.output(37, GPIO.LOW)
            os.system('sudo pkill -f buzzer.py')
 #           os.system('sudo pkill -f recover.py')
            os.system('sudo pkill -f AgVa_relay_low.py')
            os.system('sudo pkill -f AgVa_relay.py')
            os.system('sudo pkill -f power.py')
            os.system('sudo pkill -f ABP_1.py')
            os.system('sudo pkill -f SDP.py')
            os.system('sudo pkill -f debug.py')
	    subprocess.Popen('sudo python /home/pi/AgVa_5.0/O2_cal.py', shell = True)
	    GPIO.output(22, GPIO.LOW)
            while(1):
                time.sleep(0.5)
                GPIO.output(37, GPIO.HIGH)
                time.sleep(0.8)
                GPIO.output(37, GPIO.LOW)
	if(x == 'CM+STANDBY'):
	    GPIO.output(37, GPIO.LOW)
	    os.system('sudo pkill -f AgVa_relay_low.py')
	    os.system('sudo pkill -f buzzer.py')
	    GPIO.output(22, GPIO.LOW)
	    ser.write("STND01")
	    GPIO.output(18, GPIO.LOW)
	    while(1):
		x=ser.readline()
		if(x == 'CM+WAKEUP'):
		    subprocess.Popen('sudo python /home/pi/AgVa_5.0/AgVa_relay_low.py', shell = True)
		    subprocess.Popen('sudo python /home/pi/AgVa_5.0/buzzer.py', shell = True)
		    ser.write("STND00")
		    break;
                time.sleep(0.5)
                GPIO.output(37, GPIO.HIGH)
                time.sleep(0.8)
                GPIO.output(37, GPIO.LOW)
	if(x == 'CMR+CLEAN'):
	    os.system('sudo pkill -f AgVa_relay_low.py')
	    os.system('sudo pkill -f buzzer.py')
	    GPIO.output(22, GPIO.LOW)
	    GPIO.output(18, GPIO.HIGH)
	    while(1):
                time.sleep(0.5)
                GPIO.output(37, GPIO.HIGH)
                time.sleep(0.8)
                GPIO.output(37, GPIO.LOW)       
	if(x == 'CM+MUTE0'):
	    try:
	        f = open('/home/pi/AgVa_5.0/MUTE.txt','w')
	        f.write('0')
		f.close()
	    except:
		print("not able to open file")
        elif(x == 'CM+MUTE1'):
            try:
                f = open('/home/pi/AgVa_5.0/MUTE.txt','w')
                f.write('1')
                f.close()
            except:
                print("not able to open file")
        elif(x == 'CM+MUTE2'):
            try:
                f = open('/home/pi/AgVa_5.0/MUTE.txt','w')
                f.write('2')
                f.close()
            except:
                print("not able to open file")
	if(x == 'CMR+STP'):
	    GPIO.output(37, GPIO.LOW)
	    os.system('sudo pkill -f AgVa_relay_low.py')
	    os.system('sudo pkill -f buzzer.py')
	    GPIO.output(22, GPIO.LOW)
	    ser.write("STP007")
	    GPIO.output(18, GPIO.LOW)
	    subprocess.Popen('sudo python /home/pi/AgVa_5.0/self_test.py', shell = True)
	    while(1):
                time.sleep(0.5)
                GPIO.output(37, GPIO.HIGH)
                time.sleep(0.8)
                GPIO.output(37, GPIO.LOW)
	if(x == 'CMR+AgVa'):
	    subprocess.Popen("sudo python /home/pi/AgVa_5.0/AgVa_relay.py", shell = True)
	    try:
		ser.write('Running AgVa')
	    except:
		print('Error 101')
	if(x == 'CMR+power'):
	    subprocess.Popen("sudo python /home/pi/AgVa_5.0/power.py", shell = True)
	    try:
		ser.write('Running power')
	    except:
		print('Error 102')
	if(x == 'CM+KILLAgVa'):
	    os.system('sudo pkill -f AgVa.py')
	if(x == 'CM+KILLpower'):
	    os.system('sudo pkill -f power.py')
	if(x == 'CMR+ABP'):
	    subprocess.Popen("sudo python /home/pi/AgVa_5.0/ABP_1.py", shell = True)
	if(x == 'CMR+SDP'):
	    subprocess.Popen("sudo python /home/pi/AgVa_5.0/SDP.py", shell = True)
	if(x == 'CM+KILLABP'):
	    os.system('sudo pkill -f ABP_1.py')
	if(x == 'CM+KILLSDP'):
	    os.system('sudo pkill -f SDP.py')
	if(x == 'CMR+DATA.py'):
	    subprocess.Popen("sudo python /home/pi/AgVa_5.0/debug.py", shell = True)
	if(x == 'CM+SHUTDOWN'):
	    try:
		ser.write('ACK50')
	    except:
		print('unable to send')
	    os.system('sudo shutdown -h now')
	if(x == 'CM+KILLDATA'):
	    os.system('sudo pkill -f debug.py')
	    ser.write('DATA killed')
	if(x == 'CMR+BOOT'):
	    subprocess.Popen("sudo python /home/pi/AgVa_5.0/fastboot.py", shell = True)
	if(x == "HS"):
	    GPIO.output(37, GPIO.HIGH)
	    time.sleep(0.1)
	    counter = 6
	    try:
		ser.write('ACK51')
	    except:
		print('unable to send')
	    ping_time = time.time()
	    counter_flag = 1
	    GPIO.output(37, GPIO.LOW)
	exists= os.path.isfile('/home/pi/AgVa_5.0/setting.txt')
#	print(len(x))
	if(exists and x == "222"):
	    try:
	        ser.write('ACK03')
	    except:
		print('not able to send')
	if(x == 'PING'):
	    ping_delay = time.time() - ping_time
	    ping_time = time.time()
	    ping_flag = 1
	if(ping_flag == 1):
	    if(ping_set == 1):
		buzzer(3,0)
		ping_set = 0
	    ping_delay = time.time() - ping_time
	if(ping_delay > 30):
	    if(ping_set == 0):
	        buzzer(3,1)
		ping_set = 1
	if(len(x) >4):
	    values = x.split(',')
	    print(x)
	    try:
		if(values[0] == 'S1'):
		    try:
			values.pop(0)
			if(len(values) == 4):
		            f = open("/home/pi/AgVa_5.0/setting.txt","w")
			    now_str = (','.join(values))
		            f.write(now_str + ',')
		            f.close()
			    setting_flag = 1
		    except:
			print('unable to open file')
		elif(values[0] == 'S2' and setting_flag == 1):
		    print('1st step')
		    try:
#			print('2nd step')
			values.pop(0)
#			print('3rd step')
#			print('trying to append')
		        f = open("/home/pi/AgVa_5.0/setting.txt","a")
		        f.write(','.join(values))
		        f.close()
			try:
			    ser.write("S@" + ','.join(values) + "#")
			except:
			    print("unable to send setting updated")
			setting_flag = 0
		        try:
			    ser.write('ACK04')
		        except:
			    print('not able to send')
		    except:
			print('unable to open send')
	    except:
	        print('error opening file')
	print(x)
	mode_exists = os.path.isfile('/home/pi/AgVa_5.0/mode.txt')
#	print('counter is')
#	print(counter)
#	print('counter flag is')
#	print(counter_flag)
	if(counter < 1 and counter_flag == 1):
	    print(counter)
	    counter = counter + 1
	    flag = 1
	    try:
		ser.write('ACK02')
	    except:
		print('not able to send')
	if(counter >=5 and flag == 1):
	    flag = 0
	    print('setting flag to zero')
	    counter_flag = 0
	if((len(x) ==3) and (x == "110" or x =="111" or x =="120" or x =="121" or x =="130" or x =="131" or x =="140" or x =="141" or x =="210" or x =="211" or x =="220" or x =="221" or x =="220" or x =="221" or x =="230" or x =="231" or x =="240" or x =="241"or x =="250" or x =="251" or x =="310" or x =="311" or x =="320" or x =="321" or x =="330" or x =="331" or x =="340" or x =="341" or x =="350" or x =="351" or x == "260" or x == "261")):
	    try:
		f = open("/home/pi/AgVa_5.0/mode.txt","w")
		f.write(str(x))
		f.close()
		print('sending the data')
		counter = 0
#		print(counter)
		try:
		    ser.write('ACK02')
		    counter = counter +1
		except:
		    print('unable to update mode')
	    except:
		print('unable to open mode file')
##	ser.flush()
#        print(time.time() - time_now)
