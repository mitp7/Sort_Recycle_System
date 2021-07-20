# ads1015.py
#
# This library creates an instance of the Adafruit ADS1015 4-channel ADC, and defines all related methods.
# You must install Adafruit Blinka and the ADS1x15 Python library. Follow the ADS1015 wiring diagram to connect 
# the ADS1015 to your Raspberry Pi 3 B+.
#
# To install Blinka:
# sudo pip3 install adafruit-blinka
#
# To install the ADS1x15 library:
# sudo pip3 install adafruit-circuitpython-ads1x15


import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

class ads1015:

    # Define class-level variables 

    # ADS1015 specific variables
    _chan0 = None
    _chan1 = None
    _chan2 = None
    _chan3 = None
   
    # Initilize ADS1015
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1015(i2c)
        self._chan0 = AnalogIn(ads, ADS.P0)
        self._chan1 = AnalogIn(ads, ADS.P1)
        self._chan2 = AnalogIn(ads, ADS.P2)
        self._chan3 = AnalogIn(ads, ADS.P3)
        
        print ("ADS1015 DAQ initialized.")

    ######################### ADS1015 ANALOG IN METHODS ######################### 

    # Read single analog input channel
    def read_analog_input (self, channel):
        if channel == 0:
            return self._chan0.voltage            
        elif channel == 1:
            return self._chan1.voltage
        elif channel == 2:
            return self._chan2.voltage            
        elif channel == 3:
            return self._chan3.voltage            
        else:
            return -1
    
    # Close DAQ
    def close(self):
        self.__exit__()
    
    # Exit routine
    def __exit__(self):
        print ("Closing ADS1015 ...")
        print ("ADS1015 closed")
