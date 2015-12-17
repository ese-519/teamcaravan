import serial
global ser

import time

ser = serial.Serial ("/dev/ttyAMA0")    #Open named port 
ser.baudrate = 115200                     #Set baud rate to 9600
ser.flushOutput()
ser.flushInput()
# ser.close()
# def makePacket(dir):

class Global:
	def __init__(self):
		self.PACKET_SIZE = 10
		self.PACKET_START = '2'
		self.PACKET_END = '3'
		self.INFO_MSG = '4'
		self.ACK_MSG = '5'
		self.TURN_MSG = '6'
		self.FWD_MSG = '7'
		self.DOOR_MSG = '8'
		self.BRK_MSG = '9'
		
		self.INFO_OSB = '5'
		self.INFO_HLWY = '6'
		self.INFO_HAZ_RNG = '7'
		self.INFO_WALL_RNG = '8'
		
		self.DIR_LEFT = '1'
		self.DIR_RIGHT = '2'

def sendPiSerial(buf):
	global globes
	key = ''.join(x for x in buf)
	ser.write(key)

def send_command_with_data(msg_type, b):
	global globes
	tx_buf = ['0'] * (globes.PACKET_SIZE + 1)
	tx_buf[0] = globes.PACKET_START
	tx_buf[1] = msg_type
	for i in range(2, globes.PACKET_SIZE - 1):
		if (i-2 < len(b)):
			tx_buf[i] = b[i-2]
		else:
			tx_buf[i] = '0'

	tx_buf[globes.PACKET_SIZE - 1] = globes.PACKET_END
	tx_buf[globes.PACKET_SIZE] = '\0'
	sendPiSerial(tx_buf)

def send_info(info, vel):
	global globes
	b = None * 4
	b[0] = info
	b[1] = '0' if vel < 10 else chr(int(vel)/10 + 48)
	b[2] = chr(int(vel)%10 + 48)
	b[3] = chr(vel-int(vel) + 48)

	send_command_with_data(globes.INFO_MSG, b)

def send_command(msg_type, vel=0):
	global globes
	b = ['0'] * 4

	if vel == 0:
		send_command_with_data(msg_type, b)

	off = 1 if msg_type == globes.TURN_MSG else 0
	if (msg_type == globes.TURN_MSG):
		if (vel < 0):
			vel = -1 * vel
			drive_dir = globes.DIR_LEFT
		else:
			drive_dir = globes.DIR_RIGHT
		b[0] = drive_dir
	else:
		b[3] = '0'

	if (vel < 0):
		vel = -1 * vel

	b[0+off] = chr(int(vel)/10 + 48)
	b[1+off] = chr(int(vel)%10 + 48)
	b[2+off] = chr(vel-int(vel) + 48)
	send_command_with_data(msg_type, b)	

def sendDoor(dist):
	global globes
	send_command(globes.DOOR_MSG, dist)
	print "Door: ", dist

def moveForward(dist):
	global globes
	send_command(globes.FWD_MSG, dist)
	print "Move Forward: ", dist

def moveBack(dist):
	global globes
	send_command(globes.BWD_MSG, dist)
	print "Move Back: ", dist

def turnLeft(deg):
	global globes
	
	if (deg < 0):
		deg = (deg + 360)%360
	
	send_command(globes.TURN_MSG, -1 * deg)
	print "Turn Left: ", deg

def turnRight(deg):
	global globes
	if (deg < 0):
		deg = (deg + 360)%360
	
	send_command(globes.TURN_MSG, deg)
	print "Turn Right: ", deg

def brake():
	global globes
	send_command(globes.BRK_MSG)
	print "Brake"

global globes
globes = Global()

# while (True):
# 	if (ser.inWaiting() > 0):
# 		time.sleep(0.1)
# 		data = ser.read(1)
# 		if (data == globes.ACK_MSG):
# 			print "ACK"
# 		else:
# 			print "Not ACK", data