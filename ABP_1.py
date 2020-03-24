import smbus
import time
import serial
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(18,GPIO.OUT)			#motor
GPIO.setup(40,GPIO.OUT)			#exhale valve
GPIO.setup(35,GPIO.OUT)			#SSR
GPIO.output(40,GPIO.LOW)
GPIO.output(35,GPIO.LOW)
motor = GPIO.PWM(18, 50)
#GPIO.setup(19, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
#GPIO.output(19, GPIO.LOW)
GPIO.output(21, GPIO.HIGH)
motor.start(30)
bus=smbus.SMBus(1) #The default i2c bus
address=0x28
ser = serial.Serial(
	port='/dev/ttyS0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=0.05
)
val = []
def ABPcmH2O():
    val=bus.read_i2c_block_data(address,0)
    data=((val[0] & 0x3f) << 8) + val[1]
    press_psi = float(data- 1638 )/14745
    press_cmH2O=press_psi * 70.307
    return press_cmH2O

while True:
    for i in range(0,100,1):
#	print("pwm is "+ str(i))
	motor.ChangeDutyCycle(i)
	time.sleep(3)
	for j in range(20):
	    val.append(round(ABPcmH2O(),2))
	    time.sleep(0.01)
#	print(sum(val))
#	print(val)
#	print(len(val))
        print(round((sum(val)/len(val)),2))
	val = [] 
        time.sleep(1)
#	print(round(ABPcmH2O(),2))
