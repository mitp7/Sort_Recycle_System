# drv2605.py
#
# This library creates an instance of the Adafruit DRV2605 haptic feedback motor driver, and 
# defines all related methods. You must install Adafruit Blinka and the DRV2605 Python 
# library. Follow the DRV2605 wiring diagram to connect it to your Raspberry Pi 3 B+.
#
# To install Blinka:
# sudo pip3 install adafruit-blinka
#
# To install the DRV2605 library:
# sudo pip3 install adafruit-circuitpython-drv2605


import board
import busio
import adafruit_drv2605
import time

class drv2605:

    # Define class-level variables 
    
    # DRV2605 specific variables
    _drv = None
   
    # Initilize DRV2605
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self._drv = adafruit_drv2605.DRV2605(i2c)
        
        print ("DRV2605 haptic feedback motor driver initialized.")

    ######################### ADS1015 ANALOG IN METHODS ######################### 

    # Play buzz effect
    def play_effect (self):
        # define effect ID from 1 to 123
        effect_id = 17
        self._drv.sequence[0] = adafruit_drv2605.Effect(effect_id)
        self._drv.play()  # play the effect
        #time.sleep(0.5)  # for 0.5 seconds
        #self._drv.stop()  # and then stop (if it's still running)
    
    # Stop buzz effect
    def stop_effect(self):
        self._drv.stop()
        
    # Close motor driver
    def close(self):
        self.__exit__()
    
    # Exit routine
    def __exit__(self):
        print ("Closing DRV2605 ...")
        print ("DRV2605 closed")
