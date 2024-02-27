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

GREEN_LED_GPIO      = 20
RED_LED_GPIO        = 21

GREEN_LED_STATE         = 0
GREEN_LED_STOP_TIMER    = 0

RED_LED_STATE       = 0
RED_LED_STOP_TIMER  = 0

LED_ON  = 1
LED_OFF = 0

CLOSE = False
OPEN  = True

open_state = False
close_state = False

def led_on_off(led_num, on_off):
    global GREEN_LED_STATE
    global RED_LED_STATE
    if led_num == GREEN_LED_GPIO: GREEN_LED_STATE = on_off
    elif led_num == RED_LED_GPIO: RED_LED_STATE = on_off

    GPIO.output(led_num, on_off)

# --------------------------------------------------------------
# [THREAD] Button Thread
# --------------------------------------------------------------
class THREAD_BUTTON(Thread):
    #intReady = pyqtSignal()

    def __init__(self):
        super(THREAD_BUTTON, self).__init__()
        # Thread.__init__(self)

        self.pause_cond = threading.Condition(threading.Lock())

        self.__suspend = False
        self.__exit = False

    def run(self):
        global open_state
        global close_state
        while True:
            ### Suspend ###
            with self.pause_cond:
                while self.__suspend:
                    print("thread paused!!")
                    #time.sleep(0.5)
                    self.pause_cond.wait()

            if close_state:
                self.close_process()
            elif open_state:
                self.open_process()

            ### Exit ###
            if self.__exit:
                break

    def open_process(self):
        global RED_LED_STOP_TIMER
        global RED_LED_STATE
        global open_state

        print("open starting!!")
        print("red led on for 3s!!")

        led_on_off(RED_LED_GPIO, LED_ON)
        time.sleep(3)

        RED_LED_STOP_TIMER = False
        threading.Timer(0.5, red_led_blinking).start() # start timer for red led blinking

        print("[[door openning!!]]")
        servo.door_ctrl(OPEN)
        # time.sleep(3)

        print("[[drone out!!]]")
        for i in range(4):
            servo.arm_ctrl(i, OPEN)
        #time.sleep(3)

        RED_LED_STOP_TIMER = True   # stop timer for red led blinking
        time.sleep(0.5)

        led_on_off(RED_LED_GPIO, LED_ON)
        open_state = False

        self.mySuspend()

    def close_process(self):
        global GREEN_LED_STOP_TIMER
        global RED_LED_STOP_TIMER
        global GREEN_LED_STATE
        global close_state
        print("close starting!!")
        print("red led on for 3s!!")

        led_on_off(RED_LED_GPIO, LED_ON)
        time.sleep(3)

        RED_LED_STOP_TIMER = False
        threading.Timer(0.5, red_led_blinking).start() # start timer for red led blinking

        print("[[drone in!!]]")
        for i in range(3, -1, -1):
            servo.arm_ctrl(i, CLOSE)
        # time.sleep(3)

        print("[[door close!!]]")
        servo.door_ctrl(CLOSE)
        # time.sleep(3)

        led_on_off(GREEN_LED_GPIO, LED_ON)
        time.sleep(10)

        RED_LED_STOP_TIMER = True
        time.sleep(0.5)

        led_on_off(GREEN_LED_GPIO, LED_OFF)
        led_on_off(RED_LED_GPIO, LED_OFF)

        close_state = False

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
thread = THREAD_BUTTON()
# ----------------------

def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)


def red_led_blinking():
    global RED_LED_STATE
    RED_LED_STATE = not RED_LED_STATE 
    # GPIO.output(RED_LED_GPIO, RED_LED_STATE)
    led_on_off(RED_LED_GPIO, RED_LED_STATE)
    if not RED_LED_STOP_TIMER:
        threading.Timer(0.5, red_led_blinking).start()


def green_led_blinking():
    global GREEN_LED_STATE
    GREEN_LED_STATE = not GREEN_LED_STATE 
    GPIO.output(GREEN_LED_GPIO, GREEN_LED_STATE)
    if not GREEN_LED_STOP_TIMER:
        threading.Timer(0.5, green_led_blinking).start()


# Off all leds(green, red), timer stop
def led_all_off():
    global RED_LED_STOP_TIMER
    global GREEN_LED_STOP_TIMER

    RED_LED_STOP_TIMER = True
    GREEN_LED_STOP_TIMER = True
    time.sleep(0.5)

    led_on_off(RED_LED_GPIO, LED_OFF)
    led_on_off(GREEN_LED_GPIO, LED_OFF)


def button_pressed_callback(channel):
    global open_state
    global close_state

    if open_state:
        print("OPEN is running!!")
    elif close_state:
        print("CLOSE is running!!")
    elif (channel == OPEN_BUTTON_GPIO) and (not open_state):
        open_state = True
        thread.myResume()
    elif (channel == CLOSE_BUTTON_GPIO) and (not close_state):
        close_state = True
        thread.myResume()
        # print("CLOSE BUTTON PRESSED!!")


if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(OPEN_BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(CLOSE_BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(GREEN_LED_GPIO, GPIO.OUT)   
    GPIO.setup(RED_LED_GPIO, GPIO.OUT)   

    GPIO.add_event_detect(OPEN_BUTTON_GPIO, GPIO.FALLING, callback=button_pressed_callback, bouncetime=200)
    GPIO.add_event_detect(CLOSE_BUTTON_GPIO, GPIO.FALLING, callback=button_pressed_callback, bouncetime=200)

    led_all_off()

    thread.mySuspend()
    thread.start()

    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()