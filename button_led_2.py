#!/usr/bin/env python3          
                                
import signal                   
import sys
import RPi.GPIO as GPIO

import time
import threading
from threading import Thread

BUTTON_GPIO = 16
LED_GPIO = 20
last_LED_state = 0

timer_count = 0
COUNTER = 3     # led blingking time

OPEN_STATUS = False

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
        while True:
            ### Suspend ###
            with self.pause_cond:
                while self.__suspend:
                    print("thread paused!!")
                    #time.sleep(0.5)
                    self.pause_cond.wait()


            print("thread starting!!")
            print("led blinking!!")
            state = 1
            for i in range(6):
                state^=1
                GPIO.output(LED_GPIO, state)
                time.sleep(0.5)

            print("door openning!!")
            print("drone out!!")

            timer.start()

            # time.sleep(5)
            # timer.cancel()

            self.mySuspend()

            ### Exit ###
            if self.__exit:
                break

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
    if GPIO.input(BUTTON_GPIO):
        GPIO.output(LED_GPIO, GPIO.LOW)
    else:
        GPIO.output(LED_GPIO, GPIO.HIGH) 

def led_blinking():
    global last_LED_state
    global timer_count
    print("timer: {}".format(timer_count))
    threading.Timer(1, led_blinking).start()
    last_LED_state = not last_LED_state
    GPIO.output(LED_GPIO, last_LED_state)
    print("last_LED_state: {}".format(last_LED_state))
    if(timer_count > 3):
        timer.cancel()

    timer_count += 1

# --------------
thread = THREAD_BUTTON()
# --------------
# --------------

def button_pressed_callback(channel):
    global OPEN_STATUS
    if not OPEN_STATUS:
        OPEN_STATUS = True
        thread.myResume()
    else: 
        print("Open process is running!!")


if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(LED_GPIO, GPIO.OUT)   

    # GPIO.add_event_detect(BUTTON_GPIO, GPIO.BOTH, callback=button_callback, bouncetime=100)
    GPIO.add_event_detect(BUTTON_GPIO, GPIO.FALLING, callback=button_pressed_callback, bouncetime=200)

    # timer_count = 0
    timer = threading.Timer(1, led_blinking)
    # timer.start()

    thread.mySuspend()
    thread.start()
    # thread.myResume() 

    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()