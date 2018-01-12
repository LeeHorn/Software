#!/usr/bin/python

# Wrapping the Adafruit API to talk to DC motors with a simpler interface
#
# date:    11/17/2015
#
# authors: Valerio Varricchio <valerio@mit.edu>
#          Luca Carlone <lcarlone@mit.edu>
#          Dmitry Yershov <dmitry.s.yershov@gmail.com>
#          Shih-Yuan Liu <syliu@mit.edu>

from Adafruit_MotorHAT import Adafruit_MotorHAT
from math import fabs, floor

class DaguWheelsDriver:
    LEFT_MOTOR_MIN_PWM = 60        # Minimum speed for left motor  
    LEFT_MOTOR_MAX_PWM = 255       # Maximum speed for left motor  
    RIGHT_MOTOR_MIN_PWM = 60       # Minimum speed for right motor  
    RIGHT_MOTOR_MAX_PWM = 255      # Maximum speed for right motor  
    # AXEL_TO_RADIUS_RATIO = 1.0     # The axel length and turning radius ratio
    SPEED_TOLERANCE = 1.e-2       # speed tolerance level

    def __init__(self, verbose=False, debug=False, left_flip=False, right_flip=False):
        self.motorhat = Adafruit_MotorHAT(addr=0x60)
        self.leftMotor = self.motorhat.getMotor(1)
        self.rightMotor = self.motorhat.getMotor(2)
        self.verbose = verbose or debug
        self.debug = debug
        
        self.left_sgn = 1.0
        if left_flip:
            self.left_sgn = -1.0

        self.right_sgn = 1.0
        if right_flip:
            self.right_sgn = -1.0

        self.leftSpeed = 0.0
        self.rightSpeed = 0.0

        #add motor3,4
        self.leftMotor2 = self.motorhat.getMotor(3)
        self.rightMotor2 = self.motorhat.getMotor(4)    
        self.leftSpeed2 = 0.0
        self.rightSpeed2 = 0.0

        self.updatePWM()

    def PWMvalue(self, v, minPWM, maxPWM):
        pwm = 0
        if fabs(v) > self.SPEED_TOLERANCE:
            pwm = int(floor(fabs(v) * (maxPWM - minPWM) + minPWM))
        return min(pwm, maxPWM)

    def updatePWM(self):
        vl = self.leftSpeed*self.left_sgn
        vr = self.rightSpeed*self.right_sgn

        pwml = self.PWMvalue(vl, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)
        pwmr = self.PWMvalue(vr, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)

        if self.debug:
            print "v = %5.3f, u = %5.3f, vl = %5.3f, vr = %5.3f, pwml = %3d, pwmr = %3d" % (v, u, vl, vr, pwml, pwmr)

        if fabs(vl) < self.SPEED_TOLERANCE:
            leftMotorMode = Adafruit_MotorHAT.RELEASE
        elif vl > 0:
            leftMotorMode = Adafruit_MotorHAT.FORWARD
        elif vl < 0: 
            leftMotorMode = Adafruit_MotorHAT.BACKWARD

        if fabs(vr) < self.SPEED_TOLERANCE:
            rightMotorMode = Adafruit_MotorHAT.RELEASE
            pwmr = 0
        elif vr > 0:
            rightMotorMode = Adafruit_MotorHAT.FORWARD
        elif vr < 0: 
            rightMotorMode = Adafruit_MotorHAT.BACKWARD

        self.leftMotor.setSpeed(pwml)
        self.leftMotor.run(leftMotorMode)
        self.rightMotor.setSpeed(pwmr)
        self.rightMotor.run(rightMotorMode)

        #add motor3,4
        vl2 = self.leftSpeed2*self.left_sgn
        vr2 = self.rightSpeed2*self.right_sgn

        pwml2 = self.PWMvalue(vl2, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)
        pwmr2 = self.PWMvalue(vr2, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)

        if self.debug:
            print "v = %5.3f, u = %5.3f, vl = %5.3f, vr = %5.3f, pwml = %3d, pwmr = %3d" % (v, u, vl2, vr2, pwml2, pwmr2)

        if fabs(vl2) < self.SPEED_TOLERANCE:
            leftMotor2Mode = Adafruit_MotorHAT.RELEASE
        elif vl2 > 0:
            leftMotor2Mode = Adafruit_MotorHAT.FORWARD
        elif vl2 < 0: 
            leftMotor2Mode = Adafruit_MotorHAT.BACKWARD

        if fabs(vr2) < self.SPEED_TOLERANCE:
            rightMotor2Mode = Adafruit_MotorHAT.RELEASE
            pwmr2 = 0
        elif vr2 > 0:
            rightMotor2Mode = Adafruit_MotorHAT.FORWARD
        elif vr2 < 0: 
            rightMotor2Mode = Adafruit_MotorHAT.BACKWARD

        self.leftMotor2.setSpeed(pwml2)
        self.leftMotor2.run(leftMotor2Mode)
        self.rightMotor2.setSpeed(pwmr2)
        self.rightMotor2.run(rightMotor2Mode)
        

    def setWheelsSpeed(self, left, right, sign):
        if sign==1:
            self.leftSpeed = left
            self.rightSpeed = right
        #add
        elif sign ==2:
            self.leftSpeed2 = left
            self.rightSpeed2 = right

        self.updatePWM()

    def __del__(self):
        self.leftMotor.run(Adafruit_MotorHAT.RELEASE)
        self.rightMotor.run(Adafruit_MotorHAT.RELEASE)
        #add
        self.leftMotor2.run(Adafruit_MotorHAT.RELEASE)
        self.rightMotor2.run(Adafruit_MotorHAT.RELEASE)
        del self.motorhat

# Simple example to test motors
if __name__ == '__main__':
    from time import sleep

    N = 10
    delay = 100. / 1000.

    dagu = DAGU_Differential_Drive()

    # turn left
    dagu.setSteerAngle(1.0)
    # accelerate forward
    for i in range(N):
        dagu.setSpeed((1.0 + i) / N)
        sleep(delay)
    # decelerate forward
    for i in range(N):
        dagu.setSpeed((-1.0 - i + N) / N)
        sleep(delay)

    # turn right
    dagu.setSteerAngle(-1.0)
    # accelerate backward
    for i in range(N):
        dagu.setSpeed(-(1.0 + i) / N)
        sleep(delay)
    # decelerate backward
    for i in range(N):
        dagu.setSpeed(-(-1.0 - i + N) / N)
        sleep(delay)

    # turn left
    dagu.setSteerAngle(1.0)
    # accelerate forward
    for i in range(N):
        dagu.setSpeed((1.0 + i) / N)
        sleep(delay)
    # decelerate forward
    for i in range(N):
        dagu.setSpeed((-1.0 - i + N) / N)
        sleep(delay)

    del dagu
