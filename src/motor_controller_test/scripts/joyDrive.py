#!/usr/bin/env python

import rospy
import roboclaw
from math import *
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def velCallback(cmd_vel):
    CONVERSION = 60000 # Conversion factor from the -1...1 input to the total velocity
    ANGLE_DIM = 0.3 # Conversion factor to make the twists solwer

    roboclaw.Open("/dev/ttyACM0", 115200)
    roboclaw.SpeedM1(FLeftAddress, int(CONVERSION * (cmd_vel.linear.y + cmd.vel.linear.x) / ((cmd_vel.linear.y**2 + cmd_vel.linear.x**2)**(0.5))))
    roboclaw.Open("/dev/ttyACM1", 115200)
    roboclaw.SpeedM1(FRightAddress, int(CONVERSION * (cmd_vel.linear.y - cmd.vel.linear.x) / ((cmd_vel.linear.y**2 + cmd_vel.linear.x**2)**(0.5))))
    roboclaw.Open("/dev/ttyACM2", 115200)
    roboclaw.SpeedM1(BRightAddress, int(CONVERSION * (cmd_vel.linear.y - cmd.vel.linear.x) / ((cmd_vel.linear.y**2 + cmd_vel.linear.x**2)**(0.5))))
    roboclaw.SpeedM2(BLeftAddress, int(CONVERSION * (cmd_vel.linear.y + cmd.vel.linear.x) / ((cmd_vel.linear.y**2 + cmd_vel.linear.x**2)**(0.5))))

    # Print speeds of motors
    rospy.loginfo("-----------------------------------")
    rospy.loginfo("Front right " + str(int(CONVERSION * (cmd_vel.linear.y - cmd.vel.linear.x) / ((cmd_vel.linear.y**2 + cmd_vel.linear.x**2)**(0.5)))))
    rospy.loginfo("Front left " + str(int(CONVERSION * (cmd_vel.linear.y + cmd.vel.linear.x) / ((cmd_vel.linear.y**2 + cmd_vel.linear.x**2)**(0.5)))))
    rospy.loginfo("Back right " + str(int(CONVERSION * (cmd_vel.linear.y - cmd.vel.linear.x) / ((cmd_vel.linear.y**2 + cmd_vel.linear.x**2)**(0.5)))))
    rospy.loginfo("Back left " + str(int(CONVERSION * (cmd_vel.linear.y + cmd.vel.linear.x) / ((cmd_vel.linear.y**2 + cmd_vel.linear.x**2)**(0.5)))))

def joyCallback(joy):
    #Left in here only for testing
    joyY = joy.axes[1]
    joyX = joy.axes[0]
    roboclawFront.SpeedM1(address, int(12000*joyY))
    roboclawFront.SpeedM2(address, int(12000*joyY))
    roboclawBack.SpeedM1(address, int(12000*joyY))
    roboclawBack.SpeedM2(address, int(12000*joyY))

def joyDrive():
    global FLeftAddress
    global FRightAddress
    global BLeftAddress
    global BRightAddress

    FLeftAddress = 0x80   # note that this assumes that the frontleft controller is in mode 7,
    FRightAddress = 0x81  # and the the frontright is in mode 8
    BLeftAddress = 0x82
    BRightAddress = 0x83

    #PID coefficients
    Kp = 35.0
    Ki = 10.0
    Kd = 0
    qpps = 44000

    #Set PID Coefficients
    roboclaw.Open("/dev/ttyACM0", 115200)
    roboclaw.SetM1VelocityPID(FLeftAddress,Kp,Kd,Ki,qpps)
    roboclaw.Open("/dev/ttyACM1", 115200)
    roboclaw.SetM1VelocityPID(FRightAddress,Kp,Kd,Ki,qpps)
    roboclaw.Open("/dev/ttyACM2", 115200)
    roboclaw.SetM1VelocityPID(BLeftAddress,Kp,Kd,Ki,qpps)
    roboclaw.SetM2VelocityPID(BRightAddress,Kp,Kd,Ki,qpps)

    # global pub
    # pub = rospy.Publisher('joyDrive', String, queue_size=10)
    rospy.Subscriber("cmd_vel", Twist, velCallback)
    rospy.init_node('joyDrive', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    joyDrive()
