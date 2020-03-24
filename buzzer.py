import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(22, GPIO.OUT)
buzzer = GPIO.PWM(22, 1000)
buzzer.start(0)
counter = 0
trigger = 0
data= ""
MUTE = '0'
def priority(data):
    global MUTE
    try:
	try:
	    f = open('/home/pi/AgVa_5.0/MUTE.txt','r')
	    MUTE = f.readline()
	    f.close()
	except:
	    print("unable to read MUTE file")
	print('!')
        data = data.split(',')
	time.sleep(0.1)
	print('@')
	data = map(int,data)
	time.sleep(0.1)
	print('mute is')
	print(MUTE)
	if(len(data) == 10):
            low = data[0]
            medium = sum(data[1:5])
	    high = sum(data[5:8])
	    if(MUTE == "1" or MUTE == "2"):
		low = 0
		medium = 0
                high = 0
	    if(MUTE == "1"):
		if(data[3] == 1):
 		    medium = 1
		if(data[5] == 1 or data[6] == 1 or data[7] == 1):
 		    high = 1		
            highest = sum(data[8:10])
        print('low')
        print(low)
        print('med')
        print(medium)
        print('high')
        print(high)
        print('highest')
        print(highest)
        if(highest >= 1):
	    return 4
        elif(high >= 1):
	    return 3
        elif(medium >= 1):
	    return 2
        elif(low >= 1):
	    return 1
        else:
	    return 0;
    except:
	print('unable to compute')
	return 0
while(1):
    global buzzer,counter,trigger,data
    try:
#	print('trying to read file')
	try:
            f = open('/home/pi/AgVa_5.0/buzzer.txt', 'r')
            data = f.readline()
            f.close()
	except:
	    print('unable to read file')
	counter = 0
	trigger = priority(data)
        print('file read')
	print(data)
#	print(trigger)
	time.sleep(1)
	if(trigger == 0):
	    buzzer.ChangeDutyCycle(0)
	    while(1):
		try:
		    f = open('/home/pi/AgVa_5.0/buzzer.txt','r')
		    data = f.readline()
		    f.close()
		except:
		    print('unable to read file')
		trigger = priority(data)
		if(trigger != 0):
		    break
		print('In 0 loop')
		time.sleep(0.5)
	if(trigger == 1):							#LOW LEVEL TRIGGER
	    while(1):
		print('in 1 loop')
		buzzer.ChangeDutyCycle(0)
		time.sleep(0.05)
		buzzer.ChangeDutyCycle(0)
		for i in range(0,3,1):
		    try:
		        f = open('/home/pi/AgVa_5.0/buzzer.txt', 'r')
		        data = f.readline()
		        f.close()
			trigger = priority(data)
			print('trigger in 1 loop is')
			print(trigger)
		    except:
			print('unable to read file')
		    time.sleep(2)
		if(trigger != 1):
		    break
        if(trigger == 2):							#MEDIUM LEVEL TRIGGER
            while(1):
		print('in 2 loop')
                buzzer.ChangeDutyCycle(100)
                time.sleep(0.5)
                buzzer.ChangeDutyCycle(0)
		time.sleep(2)
		try:
                    f = open('/home/pi/AgVa_5.0/buzzer.txt', 'r')
                    data = f.readline()
                    f.close()
		    trigger = priority(data)
		except:
		    print('unable to read file')
		if(trigger != 2):
		    break
                time.sleep(1)
        if(trigger == 3):							#HIGH LEVEL TRIGGER
            while(1):
		print('in 3 loop')
                buzzer.ChangeDutyCycle(100)
                time.sleep(0.1)
                buzzer.ChangeDutyCycle(0)
		try:
                    f = open('/home/pi/AgVa_5.0/buzzer.txt', 'r')
                    data = f.readline()
                    f.close()
		    trigger = priority(data)
		except:
		    print('unable to read file')
		if(trigger != 3):
		    break
        if(trigger == 4):                                                   #HIGH LEVEL TRIGGER
            while(1):
		print('in 4 loop')
                buzzer.ChangeDutyCycle(100)
                time.sleep(0.3)
                buzzer.ChangeDutyCycle(0)
		time.sleep(0.3)
                try:
                    f = open('/home/pi/AgVa_5.0/buzzer.txt', 'r')
                    data = f.readline()
                    f.close()
		    print('got in here')
		    trigger = priority(data)
		    print('trigger is')
		    print(trigger)
                except:
                    print('unable to read file')
		if(trigger != 4):
		    break
    except:
	print('file not found')
#END
