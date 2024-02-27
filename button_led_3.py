#!/usr/bin/env python3          
                                
import signal                   
import sys
import RPi.GPIO as GPIO

import time
import threading
from threading import Thread
import pca9685_simpletest_m2 as servo

OPEN_BUTTON_GPIO     = 16
CLOSE_BUTTON_GPIO     = 26

GREEN_LED_GPIO  = 20
RED_LED_GPIO    = 21

last_LED_state  = 0

GREEN_LED_STATE         = 0
GREEN_LED_STOP_TIMER    = 0

RED_LED_STATE       = 0
RED_LED_STOP_TIMER  = 0

LED_ON  = 1
LED_OFF = 0

CLOSE = False
OPEN  = True

timer_count = 0
COUNTER = 3     # led blingking time

OPEN_STATUS = False
CLOSE_STATUS = False

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
        global OPEN_STATUS
        global CLOSE_STATUS
        while True:
            ### Suspend ###
            with self.pause_cond:
                while self.__suspend:
                    print("thread paused!!")
                    #time.sleep(0.5)
                    self.pause_cond.wait()

            if CLOSE_STATUS:
                self.close_process()
            elif OPEN_STATUS:
                self.open_process()

            ### Exit ###
            if self.__exit:
                break

    def open_process(self):
        global RED_LED_STOP_TIMER
        global RED_LED_STATE
        global OPEN_STATUS

        print("open starting!!")
        print("red led on for 3s!!")

        led_on_off(RED_LED_GPIO, LED_ON)
        time.sleep(3)

        RED_LED_STOP_TIMER = False
        threading.Timer(0.5, red_led_blinking).start() # start timer for red led blinking

        print("[[door openning!!]]")
        servo.door_ctrl(OPEN)
        time.sleep(3)

        print("[[drone out!!]]")
        for i in range(4):
            servo.arm_ctrl(i, OPEN)
        #time.sleep(3)

        RED_LED_STOP_TIMER = True   # stop timer for red led blinking
        time.sleep(0.5)

        led_on_off(RED_LED_GPIO, LED_ON)
        OPEN_STATUS = False

        self.mySuspend()

    def close_process(self):
        global GREEN_LED_STOP_TIMER
        global RED_LED_STOP_TIMER
        global GREEN_LED_STATE
        global CLOSE_STATUS
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
        time.sleep(3)

        led_on_off(GREEN_LED_GPIO, LED_ON)
        time.sleep(10)

        RED_LED_STOP_TIMER = True
        time.sleep(0.5)

        led_on_off(GREEN_LED_GPIO, LED_OFF)
        led_on_off(RED_LED_GPIO, LED_OFF)

        CLOSE_STATUS = False

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

def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

def button_callback(channel):
    global RED_LED_STATE
    global GREEN_LED_STATE
    if channel == OPEN_BUTTON_GPIO:
        RED_LED_STATE = not RED_LED_STATE 
        led_on_off(RED_LED_GPIO, RED_LED_STATE)
    else:
        GREEN_LED_STATE = not GREEN_LED_STATE 
        GPIO.output(GREEN_LED_GPIO, GREEN_LED_STATE)

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

def led_all_off():
    global RED_LED_STOP_TIMER
    global GREEN_LED_STOP_TIMER

    RED_LED_STOP_TIMER = True
    GREEN_LED_STOP_TIMER = True
    time.sleep(0.5)

    led_on_off(RED_LED_GPIO, LED_OFF)
    led_on_off(GREEN_LED_GPIO, LED_OFF)


def led_blinking():
    global last_LED_state
    global timer_count
    last_LED_state = not last_LED_state
    GPIO.output(GREEN_LED_GPIO, last_LED_state)
    if(timer_count < 5):
        threading.Timer(1, led_blinking).start()

    timer_count += 1

# --------------
thread = THREAD_BUTTON()
# --------------
# --------------

def button_pressed_callback(channel):
    global OPEN_STATUS
    global CLOSE_STATUS

    if (channel == OPEN_BUTTON_GPIO) and (not OPEN_STATUS):
        OPEN_STATUS = True
        thread.myResume()
    elif (channel == CLOSE_BUTTON_GPIO) and (not CLOSE_STATUS):
        CLOSE_STATUS = True
        thread.myResume()
        # print("CLOSE BUTTON PRESSED!!")
    elif not OPEN_STATUS:
        print("OPEN is running!!")
    elif not CLOSE_STATUS:
        print("CLOSE is running!!")


if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(OPEN_BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(CLOSE_BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(GREEN_LED_GPIO, GPIO.OUT)   
    GPIO.setup(RED_LED_GPIO, GPIO.OUT)   

    # GPIO.add_event_detect(OPEN_BUTTON_GPIO, GPIO.FALLING, callback=button_callback, bouncetime=200)
    # GPIO.add_event_detect(CLOSE_BUTTON_GPIO, GPIO.FALLING, callback=button_callback, bouncetime=200)

    GPIO.add_event_detect(OPEN_BUTTON_GPIO, GPIO.FALLING, callback=button_pressed_callback, bouncetime=200)
    GPIO.add_event_detect(CLOSE_BUTTON_GPIO, GPIO.FALLING, callback=button_pressed_callback, bouncetime=200)

    timer_count = 0
    # timer = threading.Timer(1, led_blinking)
    # timer.start()

    led_all_off()

    thread.mySuspend()
    thread.start()
    # thread.myResume() 

    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()