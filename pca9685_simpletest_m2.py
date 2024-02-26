from board import SCL, SDA
import busio
import time

from adafruit_pca9685 import PCA9685

i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)

# Set the PWM frequency to 60hz.
pca.frequency = 70      # 19.73ms
ONE_CLOCK_PERIOD = 1000/pca.frequency   # ms (1/pca.frequency)

# Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
# but the PCA9685 will only actually give 12 bits of resolution.
# pca.channels[0].duty_cycle = 0x7fff
#          0x1377
#   0xcfa \  |  / 0x19f4
#          \ | /
# 0x800     \|/       0x2000
#  ---------------------

# define global variable
LOW_LIMIT       = 0x800
LEFT_LIMIT      = 0xcfa
MID_POSITION    = 0x1377
RIGHT_LIMIT     = 0x19f4
HIGH_LIMIT      = 0x2000

NO_OF_SERVO = 3

SAVED_SERVO_POSITION = [0 for i in range(16)]     # save current servo position

INTERVAL_IDX = 0
START_IDX = 0
END_IDX = 1
ONE_STEP_IDX = 0
SERVO_POSITION_IDX = 1

OVER_STEP = 5   # meet target position (for fragmentation float -> int)

SERVO_POSITION_INFO = [0 for i in range(NO_OF_SERVO)]
MOVE_POSITION = [0 for i in range(NO_OF_SERVO)]


POSITION_MATRIX = [
    [2000,   [LEFT_LIMIT, RIGHT_LIMIT], [LEFT_LIMIT, RIGHT_LIMIT], [LEFT_LIMIT, RIGHT_LIMIT]],   # A position [start, end]
    [2000,   [RIGHT_LIMIT, LEFT_LIMIT], [RIGHT_LIMIT, LEFT_LIMIT], [RIGHT_LIMIT, LEFT_LIMIT]],   # B position [start, end]
    [2000,   [LEFT_LIMIT, RIGHT_LIMIT], [LEFT_LIMIT, RIGHT_LIMIT], [LEFT_LIMIT, RIGHT_LIMIT]],   # C position [start, end]
    [2000,   [RIGHT_LIMIT, LEFT_LIMIT], [RIGHT_LIMIT, LEFT_LIMIT], [RIGHT_LIMIT, LEFT_LIMIT]] 
]

# POSITION_MATRIX = [
#     [2000,   [LOW_LIMIT, HIGH_LIMIT], [MID_POSITION, LOW_LIMIT],  [LOW_LIMIT, LEFT_LIMIT]],     # A position [start, end]
#     [2000,   [HIGH_LIMIT, LOW_LIMIT], [LOW_LIMIT, MID_POSITION],  [LEFT_LIMIT, MID_POSITION]],     # B position [start, end]
#     [2000,   [LOW_LIMIT, HIGH_LIMIT], [MID_POSITION, HIGH_LIMIT], [MID_POSITION, RIGHT_LIMIT]],     # C position [start, end]
#     [2000,   [HIGH_LIMIT, LOW_LIMIT], [HIGH_LIMIT, MID_POSITION], [RIGHT_LIMIT, HIGH_LIMIT]]
# ]


# set default value
servo = [0 for i in range(16)]
servo[1] = pca.channels[0]
servo[0] = pca.channels[15]
servo[2] = pca.channels[2]

def set_one_servo(no_servo, position):
    servo[no_servo].duty_cycle = position
    SAVED_SERVO_POSITION[no_servo] = position

set_one_servo(0, LOW_LIMIT)
set_one_servo(1, MID_POSITION)
set_one_servo(2, LOW_LIMIT)
time.sleep(1) 

def test_func(wayPoint):
    interval = POSITION_MATRIX[wayPoint][INTERVAL_IDX]  # interval: xx ms
    step = int(interval/ONE_CLOCK_PERIOD)               # 20: 50Hz -> 20ms

    for i in range(NO_OF_SERVO):
        servo_start   = POSITION_MATRIX[wayPoint][i+1][START_IDX]
        servo_end     = POSITION_MATRIX[wayPoint][i+1][END_IDX]

        one_step_value = int((servo_end - servo_start)/step)
        # ons_step_angle (value) if + -> low to high, if - -> high to low
        SERVO_POSITION_INFO[i] = [one_step_value, [POSITION_MATRIX[wayPoint][i+1][START_IDX], POSITION_MATRIX[wayPoint][i+1][END_IDX]]]

        # check servo position
        if servo_start != SAVED_SERVO_POSITION[i]: print("servo {} position error!!!!!!!!!!!!!!!!".format(i))

        # move start position 
        set_one_servo(i, servo_start)


    for i in range(step + OVER_STEP): # +2 fragmentation of one_step_angle from float to int 
        for j in range(NO_OF_SERVO):
            position = SAVED_SERVO_POSITION[j] + SERVO_POSITION_INFO[j][ONE_STEP_IDX]

            if (position > SERVO_POSITION_INFO[j][SERVO_POSITION_IDX][END_IDX] and SERVO_POSITION_INFO[j][ONE_STEP_IDX] > 0) or \
               (position < SERVO_POSITION_INFO[j][SERVO_POSITION_IDX][END_IDX] and SERVO_POSITION_INFO[j][ONE_STEP_IDX] < 0 ): 
                position = SERVO_POSITION_INFO[j][SERVO_POSITION_IDX][END_IDX]

            set_one_servo(j, position)
            time.sleep(0.02) 


def main():
    for i in range(4):
        test_func(i)
        # time.sleep(1)


if __name__ == "__main__":
    main()
