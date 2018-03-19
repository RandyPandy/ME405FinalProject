# Nerf Bingo Main
__author__ = 'Matthew Ng, Randy Kropp, Yavisht Fitter'
__date__ = '03/15/2018'

import pyb
import micropython
import bno055 as imu
import controller_Ng_Kropp_Fitter as controller
import encoder_Ng_Kropp_Fitter as encode
import motor_Ng_Kropp_Fitter as driver
import utime
import power
import servo
import controller_degrees as ctr_deg


accel_range = int('00000000',2)

location = []

def setup():
    '''This function stores the locations of all 25 cells of the bingo board. Move the laser to the cell desired then press enter. The routine expects cells from top left to bottom right one row at a time. Each cell contains a tuple (horizontal encoder value in ticks, vertical roll in degrees). The function displays all cell values at the end of the routine.'''
    enc_hor.read()  ##Clearing encoders
    enc_hor.zero()
    enc_hor.read()
    enc_hor.zero()
    enc_ver.read()
    enc_ver.zero()
    enc_ver.read()
    enc_ver.zero()

    print('Starting at A1(Top Left) and finishing at E5(Bottom Right)')

    for i in range(0,5):    ## Wait for user input to store horizontal encoder value and vertical roll for the cell
        nextRow = []
        print('Row #{}'.format(i+1))
        for j in range(0,5):
            input('Awaiting Column Input {}: '.format(chr(j+65)))
            enc_hor_val = enc_hor.read()
            bno_roll = bno.get_EUL_Roll()
            print((enc_hor_val,bno_roll))
            nextRow.append([enc_hor_val,bno_roll])
        location.append(nextRow)
    
    [print(x) for x in location]    ## Print stored cell values

def x_rot(location):
    '''This function rotates the chassis by #location ticks on the x axis'''
    ctr_hor.set_setpoint(location)  ## Initialize the horizontal controller to location
    for i in range(0, 100): ## Iterate the control loop 100 times
        error = ctr_hor.err_calc(enc_hor.read())    ## Calculate error between location and current position
        actuation = ctr_hor.do_work()   ## Use error to find new duty cycle
        drv_hor.set_duty_cycle(actuation)   ## Set motor to new duty cycle
        if abs(error) < 50: ## Turn motor off within window
            drv_hor.set_duty_cycle(0)
        utime.sleep_ms(10)

def y_rot(location):
    '''This function rotates/pivots the chassis by #location degrees on the y axis'''
    ctr_ver.set_setpoint(location)  ## initialize the vertical controller to location
    for i in range(0, 100): ## Iterate the control loop 100 times
        error = ctr_ver.err_calc(bno.get_EUL_Roll()) ## Calculate error between location and current position
        actuation = ctr_ver.do_work()   ## Use error to find new duty cycle
        #print('{}_{}_{}'.format(bno.get_EUL_Roll(), error, actuation))  
        drv_ver.set_duty_cycle(actuation)   ## Set motor to new duty cycle
        if abs(error) < 0.3:    ## Turn Motor_Y off within window
            drv_ver.set_duty_cycle(0)
        utime.sleep_ms(10)

def shoot():
    '''This function is shoots the dart. The sequence is power on the nerf motors, move servo to shoot position, move servo back to cocked position, then turn off nerf motors'''
    serv.set_duty_cycle(5)
    pwr.turn_on()   ## Power on nerf motors
    utime.sleep_ms(3500)    ## Wait for motors to reach steady state speed
    serv.set_duty_cycle(23) ## Pull trigger with servo
    utime.sleep_ms(1000)    ## Wait for servo 
    serv.set_duty_cycle(5) ## Reset trigger with servo
    pwr.turn_off()  ## Turn off nerf motors

drv_hor = driver.MotorDriver(pyb.Pin.board.PA10, pyb.Pin.board.PB4, pyb.Pin.board.PB5, 3,1,2)   ## Initialize Motor_X
drv_ver= driver.MotorDriver(pyb.Pin.board.PC1, pyb.Pin.board.PA0, pyb.Pin.board.PA1, 5,1,2) ## Initialize Motor_Y
drv_hor.set_duty_cycle(0)   
drv_ver.set_duty_cycle(0)
enc_hor = encode.MotorEncoder(pyb.Pin.board.PB7, pyb.Pin.board.PB6, 4, 1, 2)    ## Initialize Motor_X Encoder
enc_ver = encode.MotorEncoder(pyb.Pin.board.PC7, pyb.Pin.board.PC6, 8, 1, 2)    ## Initialize Motor_Y Encoder
ctr_hor = controller.MotorController(0.081, 0)  ## Initialize Motor_X Gain
ctr_ver = ctr_deg.MotorController(2, 0) ## Initialize Motor_Y Gain

aye = pyb.I2C (1, pyb.I2C.MASTER)   ## Initialize I2C object
bno = imu.BNO055(aye, 0x28, accel_range) ## Initialize BNO055 IMU
pwr = power.fet()   ## Initialize mosfet
pwr.turn_off()
serv = servo.ServoDriver(pyb.Pin.board.PB3,2,2,150) ##Initialize servo for trigger actuator

setup() ## Start calibration sequence

if __name__ == "__main__":
    '''This function integrates all subsytems. The function waits for user to input desired location, moves Motor_X and Motor_Y to said location, then finally shoots toward the target.'''
    while True:
        var = input("Awaiting Location: ")  ## Location to target
        if len(var) == 2:
            try:
                col = ord(var[0].upper())-65
                row = int(var[1])-1
                nextLoc = location[row][col]
                print('{}'.format(nextLoc))
                x_rot(nextLoc[0])
                utime.sleep_ms(200)
                y_rot(nextLoc[1])
                utime.sleep_ms(200)
                shoot()

            except:
                print("Some Problem happened with the input!")

        elif len(var) == 1:
            if var == 'q':
                break
        else:
            print("Input like 'C3'")

