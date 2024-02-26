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


            print("thread running!!")
            time.sleep(1) # every 5 sec (for test)


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


# --------------
thread = THREAD_BUTTON()
# --------------


def button_pressed_callback(channel):
    global last_LED_state
    last_LED_state = not last_LED_state
    GPIO.output(LED_GPIO, last_LED_state)
    if last_LED_state: 
        thread.myResume()
    else: 
        thread.mySuspend()

if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(LED_GPIO, GPIO.OUT)   

    # GPIO.add_event_detect(BUTTON_GPIO, GPIO.BOTH, callback=button_callback, bouncetime=100)
    GPIO.add_event_detect(BUTTON_GPIO, GPIO.FALLING, callback=button_pressed_callback, bouncetime=200)

    thread.mySuspend()
    thread.start()

    # thread.myResume() 

    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()