#!/usr/bin/env python3

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

import time
from RpiMotorLib import rpi_dc_lib

class MotorDriver:
    def __init__(self, gear_ratio, wheel_sep, wheel_radius):
        self.test_mode = rospy.get_param("~test_mode",False)
        self._wheel_sep = wheel_sep
        self._wheel_rad = wheel_radius
        self._gear_ratio = gear_ratio
        self._max_rpm = 1800 # 2400 With new battery, 1800 with 50% used battery, 9V
        self._max_pwm = 100

        self._motor_left = rpi_dc_lib.L298NMDc(22, 17, 12 ,50 ,True, "motor_one")
        self._motor_right = rpi_dc_lib.L298NMDc(23, 24, 13 ,50 ,True, "motor_two")
        
        self.last_msg_time = None

        self.motors_on = False
        rospy.on_shutdown(self.turnOffMotors)
        
    # recommended for auto-disabling motors on shutdown!
    def turnOffMotors(self):
        print("Cleaning up before stopping the motor.")
        self._motor_left.cleanup(True)
        self._motor_right.cleanup(True)
        self.motors_on = False
        
    def drive(self,twist):
        x = twist.linear.x
        w = twist.angular.z
        vel_left = (x-w*self._wheel_sep/2)/self._wheel_rad*self._gear_ratio
        vel_right = (x+w*self._wheel_sep/2)/self._wheel_rad*self._gear_ratio
        
        print(vel_left, vel_right)

        pwm_left = vel_left*self._max_pwm/self._max_rpm
        pwm_right = vel_right*self._max_pwm/self._max_rpm

        if (pwm_left < 0):
            pwm_left = -pwm_left
            pwm_left = max(min(pwm_left,100),0)
            self._motor_left.backward(pwm_left)
            
        else:
            pwm_left = max(min(pwm_left,100),0)
            self._motor_left.forward(pwm_left)
        
        #if pwm_left < 45:
        #    pwm_left = 0
        
        if (pwm_right < 0):
            pwm_right = -pwm_right
            pwm_right = max(min(pwm_right,100),0)
            self._motor_right.backward(pwm_right)
            
        else:
            pwm_right = max(min(pwm_right,100),0)
            self._motor_right.forward(pwm_right)

        #if pwm_right < 45:
        #    pwm_right = 0
        
        #print "Final pwm is:"
        #print int(pwm_left), int(pwm_right)
        
        self.motors_on = True
        
        #rospy.sleep(1)
        
        self.last_msg_time = rospy.get_rostime()
        

if __name__ == '__main__':
    node = rospy.init_node('motor_driver')
    
    
    param_path = rospy.get_param("~param_path")
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    gear_ratio = params['gear_ratio']
    wheel_sep = params['wheel_sep']
    wheel_radius = params['wheel_radius']
    
    driver = MotorDriver(gear_ratio, wheel_sep, wheel_radius)
    
    rospy.Subscriber('cmd_vel', Twist, driver.drive)

    rate = rospy.Rate(10)

    if driver.test_mode:
        timeout = 1
        rospy.loginfo("[motor_driver]: Test mode on")
    else:
        timeout = 0.1
    
    while not rospy.is_shutdown():
        if driver.last_msg_time is not None and (((rospy.get_rostime() - driver.last_msg_time).to_sec()) > timeout) and driver.motors_on:
            driver.turnOffMotors()
        rate.sleep()
