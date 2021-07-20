from Common_Libraries.q2usb_lib import q2usb
from array import array
import time
import board
import busio
import adafruit_vl6180x

class rotarytable:

    # Define class-level variables 
    _daq = None
        
    # ToF sensors variabls
    _i2c = None
    _tof_sensor = None

    # Initilize rotary table
    def __init__(self):
        # Define DAQ
        self._daq = q2usb()
        print ("DAQ Initialized")
        
        # Configure ToF sensor
        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._tof_sensor = adafruit_vl6180x.VL6180X(self._i2c)
        print ("ToF sensor Initialized")
    
     ######################### TOF METHODS ######################### 

    # Read ToF sensor range (mm) output
    def read_tof_sensor (self):
        return self._tof_sensor.range
    
    ######################### INDUCTIVE PROXIMITY METHODS ######################### 

    # Read Omron E2B-M18KN16-WP-C1 2M inductive proximity sensor output
    def read_proximity_sensor (self, channel):
        return self._daq.read_analog_input(channel)

    ######################### CONTROL METHODS ######################### 
    
    # Spin table at low speed (0.5 V); 0 CW, 1 CCW, other values stop table; channel AO0
    def rotate_table_low_speed (self, direction):

        if direction == 0: # CW
            self._daq.write_analog_output (0, 0.5)
        elif direction == 1: # CCW
            self._daq.write_analog_output (0, -0.5)
        else: # stop
            self.stop_table()

    # Command table relative postion in degs
    def command_rel_position(self, angle):

        # SRV-02 encoder count to deg conversion
        K_enc = 360/4096
        # Define encoder channel number
        enc_channel = 0
        
        initial_angle = self._daq.read_encoder_count(enc_channel) * K_enc
        current_angle = initial_angle
        
        if angle >= 0:
            # If anglular displacement is positive
            while current_angle - initial_angle <= angle:
                self._daq.write_analog_output (0, -0.5)
                current_angle = self._daq.read_encoder_count(enc_channel) * K_enc
        else:
            # If angular displacement is negative
            while initial_angle - current_angle <= abs(angle):
                self._daq.write_analog_output (0, 0.5)
                current_angle = self._daq.read_encoder_count(enc_channel) * K_enc

        # Stop table by commanding 0.0 V
        self.stop_table()

    # Stop table by wrting 0 V to AO0
    def stop_table (self):
        self._daq.write_analog_output (0, 0.0)

    ################## DEVICE MANAGEMENT ##################

    # Close rotary table
    def close (self):
        self._daq.close()
           
        print ("Rotary table closed")
