#!/usr/bin/env python

import rospy
import roboclaw
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String

def callback(joy):
    roboclaw.SpeedM1(address, int(12000*joy.axes[1]))

def joyDrive():
    roboclaw.Open("/dev/ttyACM0", 115200)
    global address
    address = 0x80
    version = roboclaw.ReadVersion(address)
    if version[0] == False:
        print "GETVERSION Failed"
    else:
        print repr(version[1])


    #PID coefficients
    Kp = 35.0
    Ki = 10.0
    Kd = 0
    qpps = 44000

    #Set PID Coefficients
    #roboclaw.SetM1VelocityPID(address,Kp,Kd,Ki,qpps)
    #roboclaw.SetM2VelocityPID(address,Kp,Kd,Ki,qpps) 
    
    global pub
    pub = rospy.Publisher('joyDrive', String, queue_size=10)
    rospy.Subscriber("joy", Joy, callback)
    rospy.init_node('joyDrive', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    joyDrive()
