#!/usr/bin/env python3          
                                
import signal                   
import sys
import RPi.GPIO as GPIO

import time
import threading
from threading import Thread
import pca9685_simpletest_m2 as servo

OPEN_BUTTON_GPIO    = 16
CLOSE_BUTTON_GPIO   = 26

MODE_BUTTON_GPIO    = 19    # auto/manual mode

GREEN_LED_GPIO      = 20
RED_LED_GPIO        = 21

LED_OFF = 0
LED_ON  = 1

CLOSE = 0
OPEN  = 1

AUTO_MODE = 0
MANUAL_MODE = 1

# --------------------------------------------------------------
# [THREAD] Button Thread
# --------------------------------------------------------------
class THREAD_BUTTON(Thread):
    #intReady = pyqtSignal()

    def __init__(self, servo_ctrl_obj):
        super(THREAD_BUTTON, self).__init__()
        # Thread.__init__(self)

        self.pause_cond = threading.Condition(threading.Lock())

        self.__suspend = False
        self.__exit = False

        self.servo_ctrl_obj = servo_ctrl_obj

    def run(self):
        while True:
            ### Suspend ###
            with self.pause_cond:
                while self.__suspend:
                    print("thread paused!!")
                    #time.sleep(0.5)
                    self.pause_cond.wait()

            if self.servo_ctrl_obj.mode_state == AUTO_MODE:
                self.servo_ctrl_obj.current_state = not self.servo_ctrl_obj.current_state

            if self.servo_ctrl_obj.current_state == CLOSE:
                self.close_process()
            elif self.servo_ctrl_obj.current_state == OPEN:
                self.open_process()
            else:
                print("command error!!!!")

            ### Exit ###
            if self.__exit:
                break

    def open_process(self):
        print("open starting!!")
        print("red led on for 3s!!")

        self.servo_ctrl_obj.led_on_off(RED_LED_GPIO, LED_ON)
        time.sleep(3)

        self.servo_ctrl_obj.red_led_stop_timer = False
        threading.Timer(0.5, self.servo_ctrl_obj.red_led_blinking).start() # start timer for red led blinking

        print("[[door openning!!]]")
        servo.door_ctrl(OPEN)
        # time.sleep(3)

        print("[[drone out!!]]")
        servo.arm_out()

        self.servo_ctrl_obj.red_led_stop_timer = True   # stop timer for red led blinking
        time.sleep(0.5)

        self.servo_ctrl_obj.led_on_off(RED_LED_GPIO, LED_ON)
        self.servo_ctrl_obj.running_state = False

        if self.servo_ctrl_obj.mode_state == MANUAL_MODE:
            self.mySuspend()

    def close_process(self):
        print("close starting!!")
        print("red led on for 3s!!")

        self.servo_ctrl_obj.led_on_off(RED_LED_GPIO, LED_ON)
        time.sleep(3)

        self.servo_ctrl_obj.red_led_stop_timer = False
        threading.Timer(0.5, self.servo_ctrl_obj.red_led_blinking).start() # start timer for red led blinking

        print("[[drone in!!]]")
        servo.arm_in()

        print("[[door close!!]]")
        servo.door_ctrl(CLOSE)
        # time.sleep(3)

        #self.servo_ctrl_obj.led_on_off(GREEN_LED_GPIO, LED_ON)
        # time.sleep(10)

        self.servo_ctrl_obj.red_led_stop_timer = True
        time.sleep(0.5)

        self.servo_ctrl_obj.led_on_off(GREEN_LED_GPIO, LED_OFF)
        self.servo_ctrl_obj.led_on_off(RED_LED_GPIO, LED_OFF)

        self.servo_ctrl_obj.running_state = False

        if self.servo_ctrl_obj.mode_state == MANUAL_MODE:
            self.mySuspend()


    def mySuspend(self):
        with self.pause_cond:
            self.__suspend = True

    def myResume(self):
        with self.pause_cond:
            self.__suspend = False
            self.pause_cond.notify()

    def myExit(self):
        self.__exit = True

    def close(self):
        self.mySuspend()

# ----------------------
# thread = THREAD_BUTTON(self)
# ----------------------

class SERVO_CTRL:
    green_led_state     = 0
    green_led_stop_timer= 0

    red_led_state       = 0
    red_led_stop_timer  = 0

    open_state = False
    close_state = False

    running_state = False
    mode_state = MANUAL_MODE

    current_state = CLOSE

    def __init__(self):
        super(SERVO_CTRL, self).__init__()
        self.thread = THREAD_BUTTON(self)


    def led_on_off(self, led_num, on_off):
        if led_num == GREEN_LED_GPIO: self.green_led_state = on_off
        elif led_num == RED_LED_GPIO: self.red_led_state = on_off

        GPIO.output(led_num, on_off)


    def red_led_blinking(self):
        self.red_led_state = not self.red_led_state 
        self.led_on_off(RED_LED_GPIO, self.red_led_state)
        if not self.red_led_stop_timer:
            threading.Timer(0.5, self.red_led_blinking).start()


    def green_led_blinking(self):
        self.green_led_state = not self.green_led_state 
        GPIO.output(GREEN_LED_GPIO, self.green_led_state)
        if not self.green_led_stop_timer:
            threading.Timer(0.5, self.green_led_blinking).start()


# Off all leds(green, red), timer stop
    def led_all_off(self):
        self.red_led_stop_timer = True
        self.green_led_stop_timer = True
        time.sleep(0.5)

        self.led_on_off(RED_LED_GPIO, LED_OFF)
        self.led_on_off(GREEN_LED_GPIO, LED_OFF)


    def button_pressed_callback(self, channel):
        if channel == MODE_BUTTON_GPIO and self.mode_state == AUTO_MODE:
            print(">> MANUAL MODE")
            self.mode_state = MANUAL_MODE
        elif channel == MODE_BUTTON_GPIO and self.mode_state == MANUAL_MODE:
            print(">> AUTO MODE")
            self.mode_state = AUTO_MODE
            if not self.running_state:
                self.running_state = True
                self.thread.myResume()
        elif self.running_state:
            if self.current_state == OPEN:
                print(">> OPEN is running!!")
            else:
                print(">> CLOSE is running!!")
        elif (channel == OPEN_BUTTON_GPIO) and (not self.running_state) and (self.current_state != OPEN):
            self.running_state = True
            self.current_state = OPEN
            self.thread.myResume()
        elif (channel == CLOSE_BUTTON_GPIO) and (not self.running_state) and (self.current_state != CLOSE):
            self.running_state = True
            self.current_state = CLOSE
            self.thread.myResume()


def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)


if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(OPEN_BUTTON_GPIO,    GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(CLOSE_BUTTON_GPIO,   GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(MODE_BUTTON_GPIO,    GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(GREEN_LED_GPIO,      GPIO.OUT)   
    GPIO.setup(RED_LED_GPIO,        GPIO.OUT)   

    servo_controller = SERVO_CTRL()

    GPIO.add_event_detect(OPEN_BUTTON_GPIO,  GPIO.FALLING, callback = servo_controller.button_pressed_callback, bouncetime=200)
    GPIO.add_event_detect(CLOSE_BUTTON_GPIO, GPIO.FALLING, callback = servo_controller.button_pressed_callback, bouncetime=200)
    GPIO.add_event_detect(MODE_BUTTON_GPIO,  GPIO.FALLING, callback = servo_controller.button_pressed_callback, bouncetime=200)

    servo_controller.led_all_off()

    servo_controller.thread.mySuspend()
    servo_controller.thread.start()

    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()
