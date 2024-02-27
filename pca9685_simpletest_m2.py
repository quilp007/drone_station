from board import SCL, SDA
import busio
import time

from adafruit_pca9685 import PCA9685

i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)

# Set the PWM frequency to 60hz. -----------------------------
pca.frequency = 50      # 19.73ms
ONE_CLOCK_PERIOD = 1000/pca.frequency   # ms (1/pca.frequency)
# ------------------------------------------------------------

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

# temp. varialble
SERVO_POSITION_INFO = [0 for i in range(NO_OF_SERVO)]

CLOSE = 0
OPEN  = 1

# A <-> B <-> C <-> D <-> E
SERVO_0_A = LOW_LIMIT
SERVO_0_B = LEFT_LIMIT
SERVO_0_C = MID_POSITION
SERVO_0_D = RIGHT_LIMIT
SERVO_0_E = HIGH_LIMIT

SERVO_1_A = LEFT_LIMIT
SERVO_1_B = RIGHT_LIMIT
SERVO_1_C = LEFT_LIMIT
SERVO_1_D = RIGHT_LIMIT
SERVO_1_E = LEFT_LIMIT

SERVO_2_A = LEFT_LIMIT
SERVO_2_B = RIGHT_LIMIT
SERVO_2_C = LEFT_LIMIT
SERVO_2_D = RIGHT_LIMIT
SERVO_2_E = LEFT_LIMIT


POSITION_MATRIX = [
#interval       # SERVO 0               # SERVO 1               # SERVO 2
    [2000,   [SERVO_0_A, SERVO_0_B], [SERVO_1_A, SERVO_1_B], [SERVO_2_A, SERVO_2_B]],   # A -> B position 
    [2000,   [SERVO_0_B, SERVO_0_C], [SERVO_1_B, SERVO_1_C], [SERVO_2_B, SERVO_2_C]],   # B -> C position
    [2000,   [SERVO_0_C, SERVO_0_D], [SERVO_1_C, SERVO_1_D], [SERVO_2_C, SERVO_2_D]],   # C -> D position
    [2000,   [SERVO_0_D, SERVO_0_E], [SERVO_1_D, SERVO_1_E], [SERVO_2_D, SERVO_2_E]]    # D -> E position
]

servo = [0 for i in range(16)]
# servo NO.            PCA channel  => PLEASE CHECK CONNECTION!!!
servo[0] = pca.channels[0]
servo[1] = pca.channels[15]
servo[2] = pca.channels[2]

def set_one_servo(no_servo, position):
    servo[no_servo].duty_cycle = position
    SAVED_SERVO_POSITION[no_servo] = position

# servo go to init. position
set_one_servo(0, SERVO_0_A)
set_one_servo(1, SERVO_1_A)
set_one_servo(2, SERVO_2_A)
time.sleep(1) 

def arm_ctrl(wayPoint, direction):
    interval = POSITION_MATRIX[wayPoint][INTERVAL_IDX]  # interval: xx ms
    step = int(interval/ONE_CLOCK_PERIOD)               # 20: 50Hz -> 20ms

    for i in range(NO_OF_SERVO):
        SERVO_NO = i + 1
        if direction == OPEN:
            servo_start   = POSITION_MATRIX[wayPoint][SERVO_NO][START_IDX]
            servo_end     = POSITION_MATRIX[wayPoint][SERVO_NO][END_IDX]
        else:
            servo_start   = POSITION_MATRIX[wayPoint][SERVO_NO][END_IDX]
            servo_end     = POSITION_MATRIX[wayPoint][SERVO_NO][START_IDX]

        # ons_step_angle (value) if + -> low to high, if - -> high to low
        one_step_value = int((servo_end - servo_start)/step)

        SERVO_POSITION_INFO[i] = [one_step_value, [servo_start, servo_end]]

        # check servo position
        if servo_start != SAVED_SERVO_POSITION[i]: print("servo {} position error!!!!!!!!!!!!!!!!".format(i))

        # move start position 
        set_one_servo(i, servo_start)

    for i in range(step + OVER_STEP): # +2 fragmentation of one_step_angle from float to int 
        for j in range(NO_OF_SERVO):
            SERVO_NO = j
            position = SAVED_SERVO_POSITION[SERVO_NO] + SERVO_POSITION_INFO[SERVO_NO][ONE_STEP_IDX]

            servo_end = SERVO_POSITION_INFO[SERVO_NO][SERVO_POSITION_IDX][END_IDX]
            one_step_value = SERVO_POSITION_INFO[SERVO_NO][ONE_STEP_IDX]

            if (position > servo_end and one_step_value > 0) or \
               (position < servo_end and one_step_value < 0 ): 
                position = servo_end

            set_one_servo(SERVO_NO, position)
            time.sleep(0.02) 


def door_ctrl(flag):
    if flag == CLOSE:
        print("door CLOSE!!")
    elif flag == OPEN:
        print("door OPEN!!")
    else:
        print("ERROR!!")


def main():
    door_ctrl(OPEN)
    for i in range(4):
        arm_ctrl(i, OPEN)

    door_ctrl(CLOSE)
    for i in range(3, -1, -1):
        arm_ctrl(i, CLOSE)


if __name__ == "__main__":
    main()
