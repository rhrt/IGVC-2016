import time
import roboclaw

def displayspeed():
	enc1 = roboclaw.ReadEncM1(address)
	enc2 = roboclaw.ReadEncM2(address)
	speed1 = roboclaw.ReadSpeedM1(address)
	speed2 = roboclaw.ReadSpeedM2(address)

	print("Encoder1:"),
	if(enc1[0]==1):
		print enc1[1],
		print format(enc1[2],'02x'),
	else:
		print "failed",
	print "Encoder2:",
	if(enc2[0]==1):
		print enc2[1],
		print format(enc2[2],'02x'),
	else:
		print "failed " ,
	print "Speed1:",
	if(speed1[0]):
		print speed1[1],
	else:
		print "failed",
	print("Speed2:"),
	if(speed2[0]):
		print speed2[1]
	else:
		print "failed "

#Windows comport name
#roboclaw.Open("COM10",38400)
#Linux comport name
roboclaw.Open("/dev/ttyACM0",115200)

address = 0x80

version = roboclaw.ReadVersion(address)
if version[0]==False:
	print "GETVERSION Failed"
else:
	print repr(version[1])

#Velocity PID coefficients
Kp = 35.0
Ki = 10.0
Kd = 0
qpps = 44000

#Set PID Coefficients
roboclaw.SetM1VelocityPID(address,Kp,Kd,Ki,qpps);
roboclaw.SetM2VelocityPID(address,Kp,Kd,Ki,qpps);  

while(1):
	roboclaw.SpeedM1(address,12000)
	roboclaw.SpeedM2(address,-12000)
	for i in range(0,200):
		displayspeed()
		time.sleep(0.01)

	roboclaw.SpeedM1(address,-12000)
	roboclaw.SpeedM2(address,12000)
	for i in range(0,200):
		displayspeed()
		time.sleep(0.01)
  
