#!/usr/bin/env python

import rospy
import serial
from geometry_msgs.msg import Twist

VMAX = 0.22
OMEGAMAX = 2.84



class drive():
    def __init__(self):
        self.vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)
        self.rate = rospy.Rate(100)
        #	self.my_serial = serial.Serial('/dev/ttyACM0',115200,timeout=1) #port number to be checked


    def cmd_cb(self, data):
        self.v = data.linear.x
        self.omega = data.angular.z

        left_wheel = self.v + self.omega
        right_wheel = self.v - self.omega

        if (left_wheel > 0):
            left_direc = 1
            left_speed = self.mymap(left_wheel, 0, VMAX+OMEGAMAX, 0, 63) # we can change to 0 to 63
        else:
            left_direc = 0
            left_speed = self.mymap(-left_wheel, 0, VMAX+OMEGAMAX, 0, 63)
        
        if (right_wheel > 0):
            right_direc = 1
            right_speed = self.mymap(right_wheel, 0, VMAX+OMEGAMAX, 0, 63)
        else:
            right_direc = 0
            right_speed = self.mymap(-right_wheel, 0, VMAX+OMEGAMAX, 0, 63)
        
        if (self.v == 0 and self.omega == 0):
            right_wheel = 0
            left_wheel = 0

        # first send command for left wheels
        #command = '0' + str(left_direc) + str(left_speed)
        #command = int(bin(int(command)).replace("0b",""))

        command = '3' + str(left_direc) + "000000"
        bin_vel = int(bin(int(left_speed)).replace("0b",""))
        command = str(int(command) + bin_vel)
    	command = command.replace('3','0')
        command = bytearray(command)
        print "Left wheels",command
       # self.my_serial.write(command)

        # right wheels
        #command = '1' + str(right_direc) + str(right_speed)
        #command = int(bin(int(command)).replace("0b",""))
        command = '1' + str(right_direc) + "000000"
        bin_vel = int(bin(int(right_speed)).replace("0b",""))
        command = str(int(command) + bin_vel)
        command = bytearray(command)
        print "Right wheels",command
        #self.my_serial.write(command)
        

        self.rate.sleep()

    def mymap(self,c,a,b,d,e):
        return d + (c-a)*(e-d)/(b-a)



if __name__ == "__main__":
    rospy.init_node("motor_controller")

    chal_jaa = drive()

    rospy.spin()