# Servo Driver
__author__ = 'Matthew Ng, Randy Kropp, Yavisht Fitter'
__date__ = '03/19/2018'

import pyb
import time

class ServoDriver:
    '''This class implements a servo object and includes a function to control the duty cycle'''
    def __init__(self, pin, timer, channel, frequency):
        #print('create a servo driver')
        self.pin = pyb.Pin (pin, pyb.Pin.OUT_PP)
        self.tim = pyb.Timer (timer, freq=frequency)
        self.ch = self.tim.channel (channel, pyb.Timer.PWM, pin=self.pin)

    def set_duty_cycle(self, level):
        '''Level = 23 for clockwise 90 degrees. Level = 5 for counterclockwise 90 degrees)'''
        self.ch.pulse_width_percent (level)
