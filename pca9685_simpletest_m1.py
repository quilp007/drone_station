from board import SCL, SDA
import busio
import time

from adafruit_pca9685 import PCA9685

i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)

# Set the PWM frequency to 60hz.
pca.frequency = 50  # 19.73ms

# Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
# but the PCA9685 will only actually give 12 bits of resolution.
# pca.channels[0].duty_cycle = 0x7fff
#          0x1377
#   0xcfa \  |  / 0x19f4
#          \ | /
# 0x800     \|/       0x2000
#  ---------------------

# define global variable
LOW_LIMIT   = 0x800
HIGH_LIMIT  = 0x2000
MID_POSITION = 0x1377

CURRENT_SERVO_POSITION = [0 for i in range(16)]     # save current servo position

POSITION_MATRIX = [
    [3,     [LOW_LIMIT, HIGH_LIMIT], [0, 0], [0, 0]],     # A position [start, end]
    [2.5,   [HIGH_LIMIT, LOW_LIMIT], [0, 0], [0, 0]],     # B position [start, end]
    [2,     [LOW_LIMIT, HIGH_LIMIT], [0, 0], [0, 0]],     # C position [start, end]
    [0.5,     [HIGH_LIMIT, LOW_LIMIT], [0, 0], [0, 0]]
]

# set default value
servo = [0 for i in range(16)]
servo[0] = pca.channels[0]
servo[1] = pca.channels[15]
servo[2] = pca.channels[2]

def set_one_servo(no_servo, position):
    servo[no_servo].duty_cycle = position
    CURRENT_SERVO_POSITION[no_servo] = position

set_one_servo(0, LOW_LIMIT)
#servo[0].duty_cycle = 0xa00
set_one_servo(1, MID_POSITION)
set_one_servo(2, MID_POSITION)
time.sleep(1) 



def set_servo(no_servo, start_position, end_position, interval):
    if start_position < end_position: 
        HIGH = end_position
        LOW = start_position
    else: 
        HIGH = start_position
        LOW = end_position

    # servo[no_servo].duty_cycle = start_position
    if start_position != CURRENT_SERVO_POSITION[no_servo]: print("position error!!!!!!!!!!!!!!!!")
    set_one_servo(no_servo, start_position)
    step = int(interval*1000/20) 
    one_step_angle = int((end_position - start_position)/step)
    position = start_position
    for i in range(step+5): # +2 fragmentation of one_step_angle from float to int 
        position += one_step_angle
        if position > HIGH: position = HIGH
        elif position < LOW: position = LOW
        # servo[no_servo].duty_cycle = position
        set_one_servo(no_servo, position)
        time.sleep(0.02) 

INTERVAL_IDX = 0
START_IDX = 0
END_IDX = 1
SERVO_0_IDX = 1

def test_func(position):
    interval        = POSITION_MATRIX[position][INTERVAL_IDX]
    servo_0_start   = POSITION_MATRIX[position][SERVO_0_IDX][START_IDX]
    servo_0_end     = POSITION_MATRIX[position][SERVO_0_IDX][END_IDX]

    set_servo(0, servo_0_start, servo_0_end, interval)


def main():
    # servo[0].dutu_cycle = 0x800
    # set_servo(0, 0x980, 0x25ff, 1)

    for i in range(4):
        test_func(i)
        time.sleep(1)


if __name__ == "__main__":
    main()
