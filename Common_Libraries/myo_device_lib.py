# myo_device_lib.py
# 
# Use this library to create an instance of the myo device, and defines all related myo device methods. 
# The myo device consists of an instance of a DAQ (Quanser Q2-USB or Adafruit ADS 1015) and up to two MyoWare sensors.
# The sensors do not require any software initialization. Connect the output of the MyoWare sensors to the AI 
# terminals of the chosen DAQ. Power the MyoWave sensors using the Raspberry Pi's onboard 5 V supply.

from Common_Libraries.q2usb_lib import q2usb
from Common_Libraries.ads1015_lib import ads1015
    
class myo_device:

    # Define class-level variables   
    _daq = None
    
    # Initilize myo device, pass DAQ type (q2-usb or ads1015) as argument
    def __init__(self, daqtype):
        
        if daqtype == "q2-usb":       # Initilize Q2-USB DAQ
            self._daq = q2usb()        
            print ("Myo device initialized with Q2-USB")

        elif daqtype == "ads1015":    # Initilize ADS1015 DAQ
            self._daq = ads1015()
            print ("Myo device initialized with ADS1015")
        else:
            print ("Please specify correct DAQ type.")
    
    ######################### MYO DEVICE METHODS ######################### 

    # Read MyoWorks sensor output on specified AI channel
    def read_raw_output (self, channel):
        output_value = self._daq.read_analog_input(channel)
        return output_value

    ################## DEVICE MANAGEMENT ##################

    # Close myo device
    def close (self):
        self._daq.close()
        print ("Myo device closed")