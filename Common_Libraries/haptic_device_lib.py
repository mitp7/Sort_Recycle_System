# haptic_device_lib.py
# 
# Use this library to create an instance of the haptic device, and defines all related haptic device methods. 
# The haptic device consists of an instance of the Adafruit DRV2605 motor driver and a vibrating mini motor disc.
# Follow the Adafruit instructions on how to wire the driver and motor to you Raspberry Pi.

from Common_Libraries.drv2605_lib import drv2605

    
class haptic_device:

    # Define class-level variables   
    _motor_driver = None
    
    # Initilize haptic device
    def __init__(self):
        
        self._motor_driver = drv2605()        
        print ("haptic device initialized")

    ######################### HAPTIC DEVICE METHODS ######################### 

    # Play buzz effect
    def buzz (self):
        self._motor_driver.play_effect()
        
    # Stop buzz effect
    def stop (self):
        self._motor_driver.stop_effect()

    ################## DEVICE MANAGEMENT ##################

    # Close haptic  device
    def close (self):
        self._motor_driver.close()
        print ("Haptic device closed")