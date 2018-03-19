import pyb
import micropython

__author__ = 'Matthew Ng, Randy Kropp, Yavisht Fitter'
__date__ =  '03/19/18'

class fet:
    '''This class creates a mosfet driver and includes functions to turn mosfet on and off'''
    def __init__(self):
        self.pinC0 = pyb.Pin (pyb.Pin.board.PC0, pyb.Pin.OUT_PP)
        self.turn_off()

    def turn_on(self):
        '''Set Voltage high and turn fet on'''
        self.pinC0.high ()
        #print('ON')

    def turn_off(self):
        '''Set voltage low on gate to turn fet off'''
        self.pinC0.low()
        #print('OFF')

