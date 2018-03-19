##-*- coding: utf-8 -*-
"""
@file mma845x.py
This file contains a MicroPython driver for the MMA8451 and MMA8452
accelerometers. 

@author JR Ridgely
@copyright GPL Version 3.0
"""

import micropython


## The register address of the STATUS register in the BNO055
STATUS_REG = micropython.const (0x39)

## The register address of the OUT_X_MSB register in the BNO055
OUT_X_MSB = micropython.const (0x09)

## The register address of the OUT_X_LSB register in the BNO055
OUT_X_LSB = micropython.const (0x08)

## The register address of the OUT_Y_MSB register in the BNO055
A845x
OUT_Y_MSB = micropython.const (0x0B)

## The register address of the OUT_Y_LSB register in the BNO055
OUT_Y_LSB = micropython.const (0x0A)

## The register address of the OUT_Z_MSB register in the BNO055
45x
OUT_Z_MSB = micropython.const (0x0D)

## The register address of the OUT_Z_LSB register in the BNO0555
OUT_Z_LSB = micropython.const (0x0C)

## The register address of the WHO_AM_I register in the MMA845xBNO055
WHO_AM_I = micropython.const (0x00)


## OPR MODE
OPR_MODE_CFG = 0x3D
OPR_MODE_NDOF = int('00001100',2)
OPR_MODE_IMU = int('00001000',2)

##Accel Range Reg


##Unit select UNIT_SEL xxxxxxx0b
UNIT_SEL_REG = 0x3B

#default unit select.  windows, celsius, degrees, dps, m/s
UNIT_SELECT = 0x80

#Acceleration xxxxx0b
linear = int('00000000',2) # linear m/s 
gravity = int('00000001',2) #Gravity vector mg

##Angle Rate xxxxx0xb
degrees = int('00000000',2) #degrees/s
radians = int('00000010',2) #rad/s

#Euler angles xxxx0xxb
euler_degree = int('00000000',2)
euler_rad = int('00000100',2)

#Temperature 
celsius = ('00000000',2)
fahrenheit = int('00010000',2)

#Fusion data output format 0xxxxxxxb
'''Roll: -90° to +90° (increasing with increasing inclination)
Yaw: 0° to 360° (turning clockwise increases values)'''
windows = int('00000000',2) # Pitch -180° to +180° (turing clock-wise increases values
android = int('10000000',2) # Pitch +180° to -180° (turning clockwise decreases values)

## Eul_Roll MSB 
OUT_EUL_ROLL_MSB = 0x1D

## Eul_Roll LSB
OUT_EUL_ROLL_LSB = 0x1C

## Eul_Pitch MSB
OUT_EUL_PITCH_MSB = 0x1F

## Eul_PITCH LSB
OUT_EUL_PITCH_LSB = 0x1E

## EUL_HEADING MSB
OUT_EUL_HEADING_MSB = 0x1B 

## EUL_HEADING_LSB
OUT_EUL_HEADING_LSB = 0x1A


class BNO055:
    """ This class implements a simple driver for MMA8451 and MMA8452
    accelerometers. These inexpensive phone accelerometers talk to the CPU 
    over I<sup>2</sup>C. Only basic functionality is supported: 
    * The device can be switched from standby mode to active mode and back
    * Readings from all three axes can be taken in A/D bits or in g's
    * The range can be set to +/-2g, +/-4g, or +/-8g

    There are many other functions supported by the accelerometers which could 
    be added by someone with too much time on her or his hands :P 
    
    An example of how to use this driver:
    @code
    mma = mma845x.MMA845x (pyb.I2C (1, pyb.I2C.MASTER, baudrate = 100000), 29)
    mma.active ()
    mma.get_accels ()
    @endcode 
    The example code works for an MMA8452 on a SparkFun<sup>TM</sup> breakout
    board. """

    def __init__ (self, i2c, address, accel_range = 0):
        """ Initialize an MMA845x driver on the given I<sup>2</sup>C bus. The 
        I<sup>2</sup>C bus object must have already been initialized, as we're
        going to use it to get the accelerometer's WHO_AM_I code right away. 
        @param i2c An I<sup>2</sup>C bus already set up in MicroPython
        @param address The address of the accelerometer on the I<sup>2</sup>C
            bus 
        @param accel_range The range of accelerations to measure; it must be
            either @c RANGE_2g, @c RANGE_4g, or @c RANGE_8g (default: 2g)
        """

        ## The I2C driver which was created by the code which called this
        self.i2c = i2c

        ## The I2C address at which the accelerometer is located
        self.addr = address
        self.i2c.mem_write(0x18, 40, 0x3D)
        
        '''
        # Request the WHO_AM_I device ID byte from the accelerometer
        self._dev_id = ord (i2c.mem_read (1, address, WHO_AM_I))

        # The WHO_AM_I codes from MMA8451Q's and MMA8452Q's are recognized
        if self._dev_id == 0xA0 or self._dev_id == 160:
            self._works = True
        else:
            self._works = False
            raise ValueError ('Unknown accelerometer device ID ' 
                + str (self._dev_id) + ' at I2C address ' + address)
        '''
            
        # Ensure the accelerometer is in standby mode so we can configure it
        ##self.standby ()

        # Set the acceleration range to the given one if it's legal
        ##self.set_range (accel_range)


        '''def active (self):
        """ Put the MMA845x into active mode so that it takes data. In active
        mode, the accelerometer's settings can't be messed with. Active mode
        is set by setting the @c ACTIVE bit in register @c CTRL_REG1 to one.
        """

        if self._works:
            reg1 = ord (self.i2c.mem_read (1, self.addr, CTRL_REG1))
            reg1 |= 0x01
            self.i2c.mem_write (chr (reg1), self.addr, CTRL_REG1)
        '''

    def standby (self):
        """ Put the MMA845x into standby mode so its settings can be changed.
        No data will be taken in standby mode, so before measurements are to
        be made, one must call @c active(). """

        print ('MMA845x no can has standby')

    def set_accel(self,select):
        '''Unit select for acceleration'''
        ## @param select, 0 for linear accel m/s, 1 for gravity vector mg
        if select==1:
            unit = linear|UNIT_SELECT
            print("accel has been set to m/s")
        if select==0:
            unit = gravity|UNIT_SELECT
        
            print("accel has been set to mg")

    def get_ax_bits (self):
        """ Get the X acceleration from the accelerometer in A/D bits and 
        return it.
        @return The measured X acceleration in A/D conversion bits """
        a = self.i2c.mem_read(2, self.addr, OUT_X_LSB)
        ax = ((a[1]<<8) + a[0])/16.0
        print ('ax',ax)
        return 0


    def get_ay_bits (self):
        """ Get the Y acceleration from the accelerometer in A/D bits and 
        return it.
        @return The measured Y acceleration in A/D conversion bits """
        y = self.i2c.mem_read(2, self.addr, OUT_Y_LSB)
        ay = ((y[1]<<8) + y[0])/16.0
        print ('ay',ay)
        return 0


    def get_az_bits (self):
        """ Get the Z acceleration from the accelerometer in A/D bits and 
        return it.
        @return The measured Z acceleration in A/D conversion bits """
        z = self.i2c.mem_read(2, self.addr, OUT_Z_LSB)
        az = ((z[1]<<8) + z[0])/100.0
        print ('az',az)
        return 0

    def get_ax (self):
        """ Get the X acceleration from the accelerometer in g's, assuming
        that the accelerometer was correctly calibrated at the factory.
        @return The measured X acceleration in g's """

        print ('MMA845x uncalibrated X')
        return 0


    def get_ay (self):
        """ Get the Y acceleration from the accelerometer in g's, assuming
        that the accelerometer was correctly calibrated at the factory. The
        measurement is adjusted for the range (2g, 4g, or 8g) setting.
        @return The measured Y acceleration in g's """

        print ('MMA845x uncalibrated Y')
        return 0


    def get_az (self):
        """ Get the Z acceleration from the accelerometer in g's, assuming
        that the accelerometer was correctly calibrated at the factory. The
        measurement is adjusted for the range (2g, 4g, or 8g) setting.
        @return The measured Z acceleration in g's """

        print ('MMA845x uncalibrated Z')
        return 0


    def get_accels (self):
        """ Get all three accelerations from the MMA845x accelerometer. The
        measurement is adjusted for the range (2g, 4g, or 8g) setting.
        @return A tuple containing the X, Y, and Z accelerations in g's """

        return (self.get_ax (), self.get_ay (), self.get_az ())

    def get_EUL_Heading(self):
        """Orientation output only available in fusion operation modes.The  fusion  
        algorithm  output  offset  and  tilt  compensated  orientation  data  in 
        Eulerangles format for each DOF Heading/Roll/Pitch, the output data can be 
        read from the  appropriate EUL<dof>_ LSB and EUL_ <dof>_ MSB registers. Refer 
        table below for information regarding the data types and the unit 
        representation for the Euler angle format. Signed 2 Bytes. 1 Degree = 16LSB.
        1 RAD = 900 LSB"""
        eul_raw = self.i2c.mem_read(2, self.addr, OUT_EUL_HEADING_LSB)
        eul_heading = self.sign_val(((eul_raw[1]<<8) + eul_raw[0]))/16.0
        return eul_heading
        #print(eul_heading)
    
    def get_EUL_Roll(self):
        """Returns EUL roll bytes. Signed 2 Bytes"""
        eul_raw = self.i2c.mem_read(2, self.addr, OUT_EUL_ROLL_LSB)
        eul_roll = self.sign_val(((eul_raw[1]<<8) + eul_raw[0]))/16.0
        return eul_roll
        #print(eul_roll)
    
    def get_EUL_Pitch(self):
        """Returns EUL Pitch bytes. Signed 2 Bytes"""
        eul_raw = self.i2c.mem_read(2, self.addr, OUT_EUL_PITCH_LSB)
        eul_pitch = self.sign_val(((eul_raw[1]<<8) + eul_raw[0]))/16.0
        return (eul_pitch)
        #print(eul_pitch)


    def get_eul(self):
        roll = self.get_EUL_Roll()
        heading = self.get_EUL_Heading()
        pitch = self.get_EUL_Pitch()
        print(' roll',roll,'\n','heading',heading,'\n','pitch',pitch,'\n')

    def sign_val(self,value):
        """converts number to signed number"""
        if value >= 0x8000:
            value -= 0x10000
        return value

    '''
    def __repr__ (self):
        """ 'Convert' The MMA845x accelerometer to a string. The string 
        contains information about the configuration and status of the
        accelerometer. 
        @return A string containing diagnostic information """

        if not self._works:
            return ('No working bno055 at I2C address ' + str (self.addr))
        else:
            reg1 = ord (self.i2c.mem_read (1, self.addr, CTRL_REG1))
            diag_str = 'MMA845' + str (self._dev_id >> 4) \
                + ': I2C address ' + hex (self.addr) \
                + ', Range=' + str (1 << (self._range + 1)) + 'g, Mode='
            diag_str += 'active' if reg1 & 0x01 else 'standby'

            return diag_str
    '''


