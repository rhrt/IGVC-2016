#!/usr/bin/env python

import rospy
import roboclaw
from math import *
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def velCallback(cmd_vel):
    #TODO Check which wheel is connected to which port
    #TODO Check the constant to make sure it's reasonable
    CONVERSION = 12000 # Conversion factor from the -1...1 input to the total velocity
    netVel = (cmd_vel.linear.x**2 + cmd_vel.linear.y**2)**0.5
    angle = atan2(cmd_vel.linear.y, cmd_vel.linear.x)

    roboclaw.SpeedM1(FLeftAddress, int(CONVERSION * (netVel * cos(angle + pi/4) - cmd_vel.angular.z)))
    # roboclaw.SpeedM1(FRightAddress, int(CONVERSION * (netVel * sin(angle + pi/4) + cmd_vel.angular.z)))
    # roboclaw.SpeedM1(BLeftAddress, int(CONVERSION * (netVel * sin(angle + pi/4) - cmd_vel.angular.z)))
    # roboclaw.SpeedM1(BRightAddress, int(CONVERSION * (netVel * cos(angle + pi/4) + cmd_vel.angular.z)))

    # Print speeds of motors
    rospy.loginfo("Front right " + str(int(CONVERSION * (netVel * sin(angle + pi/4) + cmd_vel.angular.z))))
    rospy.loginfo("Front left " + str(int(CONVERSION * (netVel * cos(angle + pi/4) - cmd_vel.angular.z))))
    rospy.loginfo("Back right " + str(int(CONVERSION * (netVel * cos(angle + pi/4) + cmd_vel.angular.z))))
    rospy.loginfo("Back left " + str(int(CONVERSION * (netVel * sin(angle + pi/4) - cmd_vel.angular.z))))

def joyCallback(joy):
    #Left in here only for testing
    joyY = joy.axes[1]
    joyX = joy.axes[0]
    roboclawFront.SpeedM1(address, int(12000*joyY))
    roboclawFront.SpeedM2(address, int(12000*joyY))
    roboclawBack.SpeedM1(address, int(12000*joyY))
    roboclawBack.SpeedM2(address, int(12000*joyY))

def joyDrive():
    #TODO Check that this works for running two motor controllers

    roboclaw.Open("/dev/ttyACM0", 115200)
    # roboclawFRight.Open("/dev/ttyACM1", 115200)
    # roboclawBLeft.Open("/dev/ttyACM2", 115200)
    # roboclawBRight.Open("/dev/ttyACM3", 115200)

    global FLeftAddress
    global FRightAddress
    global BLeftAddress
    global BRightAddress

    FLeftAddress = 0x80 # note that this assumes that the frontleft controller is in mode 7,
    FRightAddress = 0x81  # and the the frontright is in mode 8
    BLeftAddress = 0x82
    BRightAddress = 0x83

    version = roboclaw.ReadVersion(FLeftAddress)
    # for x in range(0x0, 0xFF):
    #     print version
    #     if version[0] != False:
    #         print str(hex(x)) + " worked"
    #         print "------------------"
    #         break
    #     else:
    #         print str(hex(x)) + " failed"
    if version[0] == False:
        print "GETVERSION Failed"
    else:
        print repr(version[1])

    print "Nope"


    #PID coefficients
    Kp = 35.0
    Ki = 10.0
    Kd = 0
    qpps = 44000

    #Set PID Coefficients
    roboclaw.SetM1VelocityPID(FLeftAddress,Kp,Kd,Ki,qpps)
    # roboclaw.SetM1VelocityPID(FRightAddress,Kp,Kd,Ki,qpps)
    # roboclaw.SetM1VelocityPID(BLeftAddress,Kp,Kd,Ki,qpps)
    # roboclaw.SetM1VelocityPID(BRightAddress,Kp,Kd,Ki,qpps)

    # global pub
    # pub = rospy.Publisher('joyDrive', String, queue_size=10)
    rospy.Subscriber("cmd_vel", Twist, velCallback)
    rospy.init_node('joyDrive', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    joyDrive()
